/**
 * @file irt.cpp
 * @brief IRT Gun protocol wrapper implementation.
 */

#include "ir_lasertag/protocols/irt/irt.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "irt_gun";

namespace ir_lasertag {
namespace protocols {

IrtGun::IrtGun() = default;

IrtGun::~IrtGun() {
  deinit();
}

esp_err_t IrtGun::init(codec::IrTransmitter* transmitter,
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
  }

  initialized_ = true;
  ESP_LOGI(TAG, "Initialized (tx=%s, rx=%s)",
           transmitter_ ? "yes" : "no",
           receiver_ ? "yes" : "no");
  return ESP_OK;
}

void IrtGun::deinit() {
  if (receiving_) {
    stop_receive();
  }
  transmitter_ = nullptr;
  receiver_ = nullptr;
  initialized_ = false;
}

esp_err_t IrtGun::send(uint16_t gun_id, uint32_t timeout_ms) {
  if (!initialized_ || transmitter_ == nullptr) {
    ESP_LOGE(TAG, "Transmitter not available");
    return ESP_ERR_INVALID_STATE;
  }

  // Encode gun ID as 2 bytes, MSB first
  send_data_[0] = static_cast<uint8_t>(gun_id >> 8);
  send_data_[1] = static_cast<uint8_t>(gun_id & 0xFF);

  ESP_LOGD(TAG, "Sending gun ID: 0x%04X (CCM: %d)", gun_id, get_ccm_id(gun_id));

  // Transmit twice atomically via RMT loop; the footer space provides
  // the 2ms inter-packet gap.
  return transmitter_->send(send_data_, 16, timeout_ms, 2);
}

esp_err_t IrtGun::start_receive() {
  if (!initialized_ || receiver_ == nullptr) {
    ESP_LOGE(TAG, "Receiver not available");
    return ESP_ERR_INVALID_STATE;
  }

  // Reset paired packet state
  last_gun_id_ = 0;
  last_receive_time_ = 0;

  esp_err_t err = receiver_->start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start receiver: %s", esp_err_to_name(err));
    return err;
  }

  receiving_ = true;
  ESP_LOGI(TAG, "Receive started");
  return ESP_OK;
}

esp_err_t IrtGun::stop_receive() {
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

bool IrtGun::try_receive(uint16_t* gun_id) {
  if (!receiving_ || receiver_ == nullptr || gun_id == nullptr) {
    return false;
  }

  codec::IrMessage message;
  while (receiver_->try_receive(&message) == ESP_OK) {
    auto decoded = decode_message(message);
    if (decoded.has_value()) {
      if (validate_pair(decoded.value())) {
        *gun_id = decoded.value();
        return true;
      }
    }
  }
  return false;
}

esp_err_t IrtGun::receive(uint16_t* gun_id, uint32_t timeout_ms) {
  if (!receiving_ || receiver_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  if (gun_id == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  uint32_t start_time = static_cast<uint32_t>(esp_timer_get_time() / 1000);
  uint32_t remaining_ms = timeout_ms;

  while (remaining_ms > 0) {
    codec::IrMessage message;
    esp_err_t err = receiver_->receive(&message, remaining_ms);
    
    if (err == ESP_ERR_TIMEOUT) {
      return ESP_ERR_TIMEOUT;
    }
    
    if (err != ESP_OK) {
      return err;
    }

    auto decoded = decode_message(message);
    if (decoded.has_value()) {
      if (validate_pair(decoded.value())) {
        *gun_id = decoded.value();
        return ESP_OK;
      }
    }

    // Update remaining timeout
    uint32_t elapsed = static_cast<uint32_t>(esp_timer_get_time() / 1000) - start_time;
    if (elapsed >= timeout_ms) {
      return ESP_ERR_TIMEOUT;
    }
    remaining_ms = timeout_ms - elapsed;
  }

  return ESP_ERR_TIMEOUT;
}

std::optional<uint16_t> IrtGun::decode_message(const codec::IrMessage& message) {
  if (!message.valid || message.bit_count != 16) {
    return std::nullopt;
  }

  // Decode 16-bit gun ID from MSB-first data
  uint16_t gun_id = (static_cast<uint16_t>(message.data[0]) << 8) |
                    static_cast<uint16_t>(message.data[1]);
  return gun_id;
}

bool IrtGun::validate_pair(uint16_t gun_id) {
  // Get current time in milliseconds
  uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

  // Check for paired packet validation
  // IRT guns send the same ID twice with ~2ms gap
  // We validate by checking if we received the same ID within kPairTimeoutMs
  if (last_gun_id_ == gun_id &&
      (now_ms - last_receive_time_) <= kPairTimeoutMs) {
    // Valid paired packet
    ESP_LOGD(TAG, "Valid paired gun ID: 0x%04X", gun_id);

    // Reset state to avoid triple-triggering
    last_gun_id_ = 0;
    last_receive_time_ = 0;
    return true;
  } else {
    // First packet of potential pair - store and wait
    last_gun_id_ = gun_id;
    last_receive_time_ = now_ms;
    return false;
  }
}

codec::ProtocolConfig make_irt_config() {
  return kIrtConfig;
}

}  // namespace protocols
}  // namespace ir_lasertag
