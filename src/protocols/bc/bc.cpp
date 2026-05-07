/**
 * @file bc.cpp
 * @brief BC Gun protocol wrapper implementation.
 */

#include "ir_lasertag/protocols/bc/bc.hpp"

#include "esp_log.h"

static const char* TAG = "bc_gun";

namespace ir_lasertag {
namespace protocols {

BcGun::BcGun() = default;

BcGun::~BcGun() {
  deinit();
}

esp_err_t BcGun::init(codec::IrTransmitter* transmitter,
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

void BcGun::deinit() {
  if (receiving_) {
    stop_receive();
  }
  transmitter_ = nullptr;
  receiver_ = nullptr;
  initialized_ = false;
}

esp_err_t BcGun::send(uint16_t gun_id, uint32_t timeout_ms) {
  if (!initialized_ || transmitter_ == nullptr) {
    ESP_LOGE(TAG, "Transmitter not available");
    return ESP_ERR_INVALID_STATE;
  }

  // Encode gun ID as 2 bytes, MSB first
  uint8_t data[2];
  data[0] = static_cast<uint8_t>(gun_id >> 8);
  data[1] = static_cast<uint8_t>(gun_id & 0xFF);

  ESP_LOGD(TAG, "Sending gun ID: 0x%04X", gun_id);
  return transmitter_->send(data, 16, timeout_ms);
}

esp_err_t BcGun::send_async(uint16_t gun_id) {
  if (!initialized_ || transmitter_ == nullptr) {
    ESP_LOGE(TAG, "Transmitter not available");
    return ESP_ERR_INVALID_STATE;
  }

  // Encode gun ID into persistent buffer (MSB first)
  send_data_[0] = static_cast<uint8_t>(gun_id >> 8);
  send_data_[1] = static_cast<uint8_t>(gun_id & 0xFF);

  ESP_LOGD(TAG, "Sending gun ID async: 0x%04X", gun_id);
  return transmitter_->send_async(send_data_, 16);
}

esp_err_t BcGun::start_receive() {
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

esp_err_t BcGun::stop_receive() {
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

bool BcGun::try_receive(uint16_t* gun_id) {
  if (!receiving_ || receiver_ == nullptr || gun_id == nullptr) {
    return false;
  }

  codec::IrMessage message;
  if (receiver_->try_receive(&message) != ESP_OK) {
    return false;
  }

  auto decoded = decode_message(message);
  if (decoded.has_value()) {
    *gun_id = decoded.value();
    return true;
  }
  return false;
}

esp_err_t BcGun::receive(uint16_t* gun_id, uint32_t timeout_ms) {
  if (!receiving_ || receiver_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  if (gun_id == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  codec::IrMessage message;
  esp_err_t err = receiver_->receive(&message, timeout_ms);
  if (err != ESP_OK) {
    return err;
  }

  auto decoded = decode_message(message);
  if (decoded.has_value()) {
    *gun_id = decoded.value();
    return ESP_OK;
  }

  return ESP_ERR_INVALID_RESPONSE;
}

std::optional<uint16_t> BcGun::decode_message(const codec::IrMessage& message) {
  if (!message.valid || message.bit_count != 16) {
    return std::nullopt;
  }

  // Decode 16-bit gun ID from MSB-first data
  uint16_t gun_id = (static_cast<uint16_t>(message.data[0]) << 8) |
                    static_cast<uint16_t>(message.data[1]);
  return gun_id;
}

codec::ProtocolConfig make_bc_config() {
  return kBcConfig;
}

}  // namespace protocols
}  // namespace ir_lasertag
