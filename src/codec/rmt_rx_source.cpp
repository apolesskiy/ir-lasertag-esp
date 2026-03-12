/**
 * @file rmt_rx_source.cpp
 * @brief RMT-based IrRxSource implementation.
 */

#include "ir_lasertag/codec/rmt_rx_source.hpp"

#include "esp_check.h"
#include "esp_log.h"
#include "driver/rmt_rx.h"

static const char* TAG = "rmt_rx_src";

namespace ir_lasertag {
namespace codec {

RmtRxSource::RmtRxSource() = default;

RmtRxSource::~RmtRxSource() {
  deinit();
}

esp_err_t RmtRxSource::init(const rmt_rx_channel_config_t& config,
                            size_t symbol_buffer_size) {
  if (initialized_) {
    ESP_LOGE(TAG, "already initialized");
    return ESP_ERR_INVALID_STATE;
  }

  symbol_buffer_size_ = symbol_buffer_size;

  // Allocate symbol buffer
  symbol_buffer_ = static_cast<rmt_symbol_word_t*>(
      malloc(symbol_buffer_size_ * sizeof(rmt_symbol_word_t)));
  if (symbol_buffer_ == nullptr) {
    ESP_LOGE(TAG, "failed to allocate symbol buffer");
    return ESP_ERR_NO_MEM;
  }

  // Create RMT RX channel
  esp_err_t ret = rmt_new_rx_channel(&config, &channel_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to create RX channel: %s", esp_err_to_name(ret));
    free(symbol_buffer_);
    symbol_buffer_ = nullptr;
    return ret;
  }

  // Register ISR callback
  rmt_rx_event_callbacks_t cbs = {
      .on_recv_done = rmt_rx_done_cb,
  };

  ret = rmt_rx_register_event_callbacks(channel_, &cbs, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to register callbacks: %s", esp_err_to_name(ret));
    rmt_del_channel(channel_);
    channel_ = nullptr;
    free(symbol_buffer_);
    symbol_buffer_ = nullptr;
    return ret;
  }

  initialized_ = true;
  ESP_LOGI(TAG, "initialized on GPIO %d",
           static_cast<int>(config.gpio_num));

  return ESP_OK;
}

void RmtRxSource::deinit() {
  if (!initialized_) {
    return;
  }

  if (started_) {
    stop();
  }

  if (channel_) {
    rmt_del_channel(channel_);
    channel_ = nullptr;
  }

  if (symbol_buffer_) {
    free(symbol_buffer_);
    symbol_buffer_ = nullptr;
  }

  frame_callback_ = nullptr;
  callback_ctx_ = nullptr;
  initialized_ = false;
  ESP_LOGI(TAG, "deinitialized");
}

esp_err_t RmtRxSource::set_on_frame_callback(IrRxFrameCallback callback,
                                             void* ctx) {
  frame_callback_ = callback;
  callback_ctx_ = ctx;
  return ESP_OK;
}

esp_err_t RmtRxSource::start() {
  if (!initialized_) {
    ESP_LOGE(TAG, "not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (started_) {
    return ESP_OK;
  }

  esp_err_t ret = rmt_enable(channel_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to enable channel: %s", esp_err_to_name(ret));
    return ret;
  }

  restart_receive();
  started_ = true;
  ESP_LOGI(TAG, "started");

  return ESP_OK;
}

esp_err_t RmtRxSource::stop() {
  if (!initialized_ || !started_) {
    return ESP_OK;
  }

  esp_err_t ret = rmt_disable(channel_);
  started_ = false;
  ESP_LOGI(TAG, "stopped");

  return ret;
}

esp_err_t RmtRxSource::set_max_symbol_duration_us(uint16_t duration_us) {
  max_symbol_duration_ns_ = static_cast<uint32_t>(duration_us) * 1000;
  ESP_LOGD(TAG, "max_symbol_duration set to %u us (%lu ns)",
           (unsigned)duration_us,
           (unsigned long)max_symbol_duration_ns_);
  return ESP_OK;
}

bool IRAM_ATTR RmtRxSource::rmt_rx_done_cb(
    rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t* edata,
    void* user_ctx) {
  auto* self = static_cast<RmtRxSource*>(user_ctx);

  // Deliver symbols to the registered callback, then restart.
  // The callback (IrReceiver::on_source_frame) copies the symbols
  // and queues them for task-context processing, so it's safe to
  // restart immediately here.
  if (self->frame_callback_) {
    self->frame_callback_(edata->received_symbols, edata->num_symbols,
                          self->callback_ctx_);
  }

  // Restart capture for next frame
  self->restart_receive();
  return false;  // No higher-priority task woken by default
}

void RmtRxSource::restart_receive() {
  rmt_receive_config_t rx_config = {
      .signal_range_min_ns = 1000,
      .signal_range_max_ns = max_symbol_duration_ns_,
      .flags = {
          .en_partial_rx = 0,
      },
  };

  rmt_receive(channel_, symbol_buffer_,
              symbol_buffer_size_ * sizeof(rmt_symbol_word_t),
              &rx_config);
}

}  // namespace codec
}  // namespace ir_lasertag
