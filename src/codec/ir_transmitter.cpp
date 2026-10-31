/**
 * @file ir_transmitter.cpp
 * @brief IR transmitter implementation.
 */

#include "ir_lasertag/codec/ir_transmitter.hpp"

#include <cstring>

#include "ir_lasertag/codec/ir_encoder.hpp"

#include "esp_check.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"

static const char* TAG = "ir_tx";

namespace ir_lasertag {
namespace codec {

IrTransmitter::IrTransmitter() = default;

IrTransmitter::~IrTransmitter() {
  deinit();
}

esp_err_t IrTransmitter::init(const rmt_tx_channel_config_t& config) {
  if (initialized_) {
    ESP_LOGE(TAG, "already initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (config.trans_queue_depth > kMaxQueueDepth) {
    ESP_LOGE(TAG, "trans_queue_depth %u is over %u",
             static_cast<unsigned>(config.trans_queue_depth),
             static_cast<unsigned>(kMaxQueueDepth));
    return ESP_ERR_INVALID_ARG;
  }
  gpio_ = config.gpio_num;
  queue_depth_ = config.trans_queue_depth > 0 ? config.trans_queue_depth : 1;
  pending_ = 0;
  next_slot_ = 0;

  // Create RMT TX channel
  esp_err_t ret = rmt_new_tx_channel(&config, &channel_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to create TX channel: %s", esp_err_to_name(ret));
    return ret;
  }

  // Register TX done callback
  rmt_tx_event_callbacks_t cbs = {
      .on_trans_done = rmt_tx_done_callback,
  };
  ret = rmt_tx_register_event_callbacks(channel_, &cbs, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to register TX callbacks: %s", esp_err_to_name(ret));
    rmt_del_channel(channel_);
    channel_ = nullptr;
    return ret;
  }

  // Create copy encoder for raw mode
  rmt_copy_encoder_config_t copy_config = {};
  ret = rmt_new_copy_encoder(&copy_config, &copy_encoder_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to create copy encoder: %s", esp_err_to_name(ret));
    rmt_del_channel(channel_);
    channel_ = nullptr;
    return ret;
  }

  // Enable channel
  ret = rmt_enable(channel_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to enable channel: %s", esp_err_to_name(ret));
    rmt_del_encoder(copy_encoder_);
    copy_encoder_ = nullptr;
    rmt_del_channel(channel_);
    channel_ = nullptr;
    return ret;
  }

  initialized_ = true;
  ESP_LOGI(TAG, "initialized on GPIO %d", static_cast<int>(gpio_));

  return ESP_OK;
}

void IrTransmitter::deinit() {
  if (!initialized_) {
    return;
  }

  if (channel_) {
    rmt_disable(channel_);
  }

  destroy_encoder();

  if (copy_encoder_) {
    rmt_del_encoder(copy_encoder_);
    copy_encoder_ = nullptr;
  }

  if (channel_) {
    rmt_del_channel(channel_);
    channel_ = nullptr;
  }

  initialized_ = false;
  protocol_set_ = false;
  pending_ = 0;
  ESP_LOGI(TAG, "deinitialized");
}

esp_err_t IrTransmitter::set_protocol(const ProtocolConfig& config) {
  if (!initialized_) {
    ESP_LOGE(TAG, "not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!config.is_valid()) {
    ESP_LOGE(TAG, "invalid protocol config");
    return ESP_ERR_INVALID_ARG;
  }

  protocol_ = config;

  // Recreate encoder with new config
  destroy_encoder();
  esp_err_t ret = create_encoder();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to create encoder");
    return ret;
  }

  // Apply carrier modulation (duty 1.0 = disable carrier entirely)
  esp_err_t carrier_ret;
  if (protocol_.carrier_duty >= 1.0f) {
    carrier_ret = rmt_apply_carrier(channel_, nullptr);
  } else {
    rmt_carrier_config_t carrier_config = {
        .frequency_hz = protocol_.carrier_freq_hz,
        .duty_cycle = protocol_.carrier_duty,
        .flags = {
            .polarity_active_low = 0,
            .always_on = 0,
        },
    };
    carrier_ret = rmt_apply_carrier(channel_, &carrier_config);
  }

  ret = carrier_ret;
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to apply carrier: %s", esp_err_to_name(ret));
    return ret;
  }

  protocol_set_ = true;
  ESP_LOGI(TAG, "protocol set: carrier=%lu Hz, duty=%.2f, bits=%u",
           protocol_.carrier_freq_hz, protocol_.carrier_duty,
           protocol_.bit_count);
  ESP_LOGI(TAG, "  GPIO=%d, channel=%p, encoder=%p",
           static_cast<int>(gpio_), channel_, encoder_);

  return ESP_OK;
}

esp_err_t IrTransmitter::set_carrier_duty(float duty) {
  if (!initialized_) {
    ESP_LOGE(TAG, "not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (duty < 0.0f || duty > 1.0f) {
    ESP_LOGE(TAG, "invalid duty cycle: %.2f", duty);
    return ESP_ERR_INVALID_ARG;
  }

  protocol_.carrier_duty = duty;

  esp_err_t ret;
  if (duty >= 1.0f) {
    ret = rmt_apply_carrier(channel_, nullptr);
  } else {
    rmt_carrier_config_t carrier_config = {
        .frequency_hz = protocol_.carrier_freq_hz,
        .duty_cycle = duty,
        .flags = {
            .polarity_active_low = 0,
            .always_on = 0,
        },
    };
    ret = rmt_apply_carrier(channel_, &carrier_config);
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to apply carrier: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "carrier duty set to %.0f%%", duty * 100.0f);
  return ESP_OK;
}

bool IRAM_ATTR IrTransmitter::rmt_tx_done_callback(
    rmt_channel_handle_t channel,
    const rmt_tx_done_event_data_t* edata,
    void* user_ctx) {
  (void)channel;
  (void)edata;
  auto* tx = static_cast<IrTransmitter*>(user_ctx);
  portENTER_CRITICAL_ISR(&tx->pending_mux_);
  if (tx->pending_ > 0) {
    tx->pending_ = tx->pending_ - 1;
  }
  portEXIT_CRITICAL_ISR(&tx->pending_mux_);
  if (tx->done_callback_ != nullptr) {
    tx->done_callback_(tx->done_ctx_);
  }
  return false;
}

esp_err_t IrTransmitter::send_async(const uint8_t* data, size_t bit_count,
                                    int loop_count) {
  if (!initialized_) {
    ESP_LOGE(TAG, "not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!protocol_set_) {
    ESP_LOGE(TAG, "protocol not set");
    return ESP_ERR_INVALID_STATE;
  }

  if (data == nullptr || bit_count == 0) {
    ESP_LOGE(TAG, "invalid data");
    return ESP_ERR_INVALID_ARG;
  }

  // Calculate bytes needed (round up)
  size_t byte_count = (bit_count + 7) / 8;
  if (byte_count > IrMessage::kMaxDataBytes) {
    ESP_LOGE(TAG, "%zu bits is over the %zu-byte limit", bit_count,
             IrMessage::kMaxDataBytes);
    return ESP_ERR_INVALID_ARG;
  }

  // Claim a slot. Frames complete in order, so with fewer than
  // queue_depth_ pending, the next slot's frame is done.
  portENTER_CRITICAL(&pending_mux_);
  if (pending_ >= queue_depth_) {
    portEXIT_CRITICAL(&pending_mux_);
    ESP_LOGD(TAG, "queue full (%zu)", queue_depth_);
    return ESP_ERR_INVALID_STATE;
  }
  pending_ = pending_ + 1;
  portEXIT_CRITICAL(&pending_mux_);
  uint8_t* slot = slots_[next_slot_];
  next_slot_ = (next_slot_ + 1) % queue_depth_;
  memcpy(slot, data, byte_count);

  ESP_LOGD(TAG, "send_async: %zu bits, %zu bytes, repeat %d, data=[%02X %02X]",
           bit_count, byte_count, loop_count, slot[0],
           byte_count > 1 ? slot[1] : 0);

  // The driver resets the encoder when it starts each transaction, so a
  // queued frame never disturbs one still being encoded.
  rmt_transmit_config_t tx_config = {
      .loop_count = loop_count,
      .flags = {
          .eot_level = 0,
          .queue_nonblocking = 1,
      },
  };

  esp_err_t ret = rmt_transmit(channel_, encoder_, slot, byte_count, &tx_config);
  if (ret != ESP_OK) {
    portENTER_CRITICAL(&pending_mux_);
    pending_ = pending_ - 1;
    portEXIT_CRITICAL(&pending_mux_);
    next_slot_ = (next_slot_ + queue_depth_ - 1) % queue_depth_;
    ESP_LOGE(TAG, "rmt_transmit failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGD(TAG, "rmt_transmit started OK");
  return ESP_OK;
}

esp_err_t IrTransmitter::wait_done(uint32_t timeout_ms) {
  return rmt_tx_wait_all_done(channel_, timeout_ms);
}

esp_err_t IrTransmitter::send(const uint8_t* data, size_t bit_count,
                              uint32_t timeout_ms, int loop_count) {
  esp_err_t ret = send_async(data, bit_count, loop_count);
  if (ret != ESP_OK) {
    return ret;
  }

  return wait_done(timeout_ms);
}

esp_err_t IrTransmitter::send_raw(const RawTiming* timings, size_t count,
                                  uint32_t timeout_ms) {
  if (!initialized_) {
    ESP_LOGE(TAG, "not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (timings == nullptr || count == 0) {
    ESP_LOGE(TAG, "invalid timings");
    return ESP_ERR_INVALID_ARG;
  }

  // Convert RawTiming to RMT symbols
  // Each RawTiming becomes one symbol
  auto* symbols = static_cast<rmt_symbol_word_t*>(
      malloc(count * sizeof(rmt_symbol_word_t)));
  if (symbols == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  for (size_t i = 0; i < count; i++) {
    symbols[i].duration0 = timings[i].duration_us;
    symbols[i].level0 = (timings[i].level == Level::kMark) ? 1 : 0;
    symbols[i].duration1 = 0;
    symbols[i].level1 = 0;
  }

  // Reset copy encoder
  rmt_encoder_reset(copy_encoder_);

  rmt_transmit_config_t tx_config = {
      .loop_count = 0,
      .flags = {
          .eot_level = 0,
          .queue_nonblocking = 0,
      },
  };

  esp_err_t ret = rmt_transmit(channel_, copy_encoder_, symbols,
                               count * sizeof(rmt_symbol_word_t), &tx_config);
  if (ret != ESP_OK) {
    free(symbols);
    ESP_LOGE(TAG, "raw transmit failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // The driver reads the symbols until the transaction is done (refills
  // happen in its ISR), so they must outlive it.
  ret = rmt_tx_wait_all_done(channel_, timeout_ms);
  if (ret != ESP_OK) {
    // Still in use: stop the channel before the buffer goes away.
    ESP_LOGE(TAG, "wait failed: %s", esp_err_to_name(ret));
    rmt_disable(channel_);
    free(symbols);
    rmt_enable(channel_);
    return ret;
  }
  free(symbols);

  return ret;
}

esp_err_t IrTransmitter::create_encoder() {
  return ir_encoder_create(protocol_, &encoder_);
}

void IrTransmitter::destroy_encoder() {
  if (encoder_) {
    rmt_del_encoder(encoder_);
    encoder_ = nullptr;
  }
}

}  // namespace codec
}  // namespace ir_lasertag
