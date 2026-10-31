/**
 * @file arclite.cpp
 * @brief Arclite Tag Code protocol wrapper implementation.
 */

#include "ir_lasertag/protocols/arclite/arclite.hpp"

#include <iterator>

#include "esp_log.h"

static const char* TAG = "arclite_tagcode";

namespace ir_lasertag {
namespace protocols {

ArcliteTagCode::ArcliteTagCode() = default;

ArcliteTagCode::~ArcliteTagCode() {
  deinit();
}

esp_err_t ArcliteTagCode::init(codec::IrTransmitter* transmitter,
                                codec::IrReceiver* receiver,
                                const codec::ProtocolConfig& config) {
  if (transmitter == nullptr && receiver == nullptr) {
    ESP_LOGE(TAG, "At least one of transmitter or receiver must be provided");
    return ESP_ERR_INVALID_ARG;
  }

  if (initialized_) {
    deinit();
  }

  transmitter_ = transmitter;
  receiver_ = receiver;

  // Apply protocol config to transmitter
  if (transmitter_ != nullptr) {
    esp_err_t err = transmitter_->set_protocol(config);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to set transmitter protocol: %s",
               esp_err_to_name(err));
      return err;
    }
  }

  // Apply protocol config to receiver
  if (receiver_ != nullptr) {
    esp_err_t err = receiver_->set_protocol(config);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to set receiver protocol: %s",
               esp_err_to_name(err));
      return err;
    }
    // The spec's end-of-frame threshold, not the codec's 1.5 x the longest
    // pulse (which would give 1000 µs, at the limit).
    err = receiver_->set_end_of_frame_us(kArcliteRxIdleUs);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to set the end-of-frame threshold: %s",
               esp_err_to_name(err));
      return err;
    }
  }

  initialized_ = true;
  ESP_LOGI(TAG, "Initialized (tx=%s, rx=%s)",
           transmitter_ ? "yes" : "no",
           receiver_ ? "yes" : "no");
  return ESP_OK;
}

void ArcliteTagCode::deinit() {
  if (receiving_) {
    stop_receive();
  }
  transmitter_ = nullptr;
  receiver_ = nullptr;
  initialized_ = false;
}

esp_err_t ArcliteTagCode::send(uint16_t tag_code, uint32_t timeout_ms) {
  if (!initialized_ || transmitter_ == nullptr) {
    ESP_LOGE(TAG, "Transmitter not available");
    return ESP_ERR_INVALID_STATE;
  }
  if (tag_code > kArcliteCodeMax) {
    ESP_LOGE(TAG, "Tag code 0x%04X does not fit in 14 bits", tag_code);
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t data[2];
  arclite_word_bytes(tag_code, data);
  ESP_LOGD(TAG, "Sending tag code: 0x%04X", tag_code);
  return transmitter_->send(data, kArcliteBitCount, timeout_ms);
}

esp_err_t ArcliteTagCode::send_async(uint16_t tag_code) {
  if (!initialized_ || transmitter_ == nullptr) {
    ESP_LOGE(TAG, "Transmitter not available");
    return ESP_ERR_INVALID_STATE;
  }
  if (tag_code > kArcliteCodeMax) {
    ESP_LOGE(TAG, "Tag code 0x%04X does not fit in 14 bits", tag_code);
    return ESP_ERR_INVALID_ARG;
  }

  // The transmitter copies the bytes into its own queue slot.
  uint8_t data[2];
  arclite_word_bytes(tag_code, data);
  ESP_LOGD(TAG, "Sending tag code async: 0x%04X", tag_code);
  return transmitter_->send_async(data, kArcliteBitCount);
}

esp_err_t ArcliteTagCode::start_receive() {
  if (!initialized_ || receiver_ == nullptr) {
    ESP_LOGE(TAG, "Receiver not available");
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = receiver_->start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start receiver: %s", esp_err_to_name(err));
    return err;
  }

  receiving_ = true;
  ESP_LOGI(TAG, "Receive started");
  return ESP_OK;
}

esp_err_t ArcliteTagCode::stop_receive() {
  if (!receiving_) {
    return ESP_ERR_INVALID_STATE;
  }

  if (receiver_ != nullptr) {
    receiver_->stop();
  }

  receiving_ = false;
  ESP_LOGI(TAG, "Receive stopped");
  return ESP_OK;
}

bool ArcliteTagCode::try_receive(uint16_t* tag_code) {
  if (!receiving_ || receiver_ == nullptr || tag_code == nullptr) {
    return false;
  }
  return receive_burst(tag_code, 0) == ESP_OK;
}

esp_err_t ArcliteTagCode::receive(uint16_t* tag_code, uint32_t timeout_ms) {
  if (!receiving_ || receiver_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  if (tag_code == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  return receive_burst(tag_code, timeout_ms);
}

esp_err_t ArcliteTagCode::receive_burst(uint16_t* tag_code,
                                        uint32_t timeout_ms) {
  rmt_symbol_word_t symbols[codec::IrReceiver::kMaxFrameSymbols];
  size_t count = 0;
  esp_err_t err = receiver_->receive_symbols(symbols, std::size(symbols),
                                             &count, nullptr, timeout_ms);
  if (err != ESP_OK) {
    return err;
  }

  // The first valid frame in the burst, if any.
  struct Found {
    bool ok = false;
    uint16_t code = 0;
  } found;
  ArcliteDecoder decoder;
  decoder.decode_symbols(
      symbols, count,
      [](const ArcliteFrame& frame, void* ctx) {
        auto* f = static_cast<Found*>(ctx);
        if (!f->ok && frame.result == ArcliteResult::kOk) {
          f->ok = true;
          f->code = frame.code;
        }
      },
      &found);
  if (!found.ok) {
    return ESP_ERR_INVALID_RESPONSE;
  }
  *tag_code = found.code;
  return ESP_OK;
}

codec::ProtocolConfig make_arclite_config() {
  return kArcliteConfig;
}

}  // namespace protocols
}  // namespace ir_lasertag
