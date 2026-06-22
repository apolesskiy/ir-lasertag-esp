/**
 * @file irt.cpp
 * @brief IRT protocol wrapper implementation (gun + spawner).
 */

#include "ir_lasertag/protocols/irt/irt.hpp"

#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "irt";

namespace ir_lasertag {
namespace protocols {

Irt::Irt() = default;

Irt::~Irt() {
  deinit();
}

esp_err_t Irt::init(codec::IrTransmitter* transmitter,
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

  if (transmitter_ != nullptr) {
    esp_err_t err = transmitter_->set_protocol(config);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to set transmitter protocol: %s",
               esp_err_to_name(err));
      return err;
    }
  }

  if (receiver_ != nullptr) {
    esp_err_t err = receiver_->set_protocol(config);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to set receiver protocol: %s",
               esp_err_to_name(err));
      return err;
    }
  }

  initialized_ = true;
  ESP_LOGI(TAG, "Initialized (tx=%s, rx=%s)", transmitter_ ? "yes" : "no",
           receiver_ ? "yes" : "no");
  return ESP_OK;
}

void Irt::deinit() {
  if (receiving_) {
    stop_receive();
  }
  transmitter_ = nullptr;
  receiver_ = nullptr;
  initialized_ = false;
}

esp_err_t Irt::send_gun(uint16_t id, uint32_t timeout_ms) {
  if (!initialized_ || transmitter_ == nullptr) {
    ESP_LOGE(TAG, "Transmitter not available");
    return ESP_ERR_INVALID_STATE;
  }

  // Encode ID as 2 bytes, MSB first.
  send_data_[0] = static_cast<uint8_t>(id >> 8);
  send_data_[1] = static_cast<uint8_t>(id & 0xFF);

  ESP_LOGD(TAG, "Sending gun ID: 0x%04X (CCM: %d)", id, get_ccm_id(id));

  // Transmit twice atomically via RMT loop; the footer space provides the
  // 2ms inter-packet gap.
  return transmitter_->send(send_data_, 16, timeout_ms, 2);
}

esp_err_t Irt::send_spawner(uint16_t id, uint32_t timeout_ms) {
  if (!initialized_ || transmitter_ == nullptr) {
    ESP_LOGE(TAG, "Transmitter not available");
    return ESP_ERR_INVALID_STATE;
  }

  // Encode ID as 2 bytes, MSB first.
  send_data_[0] = static_cast<uint8_t>(id >> 8);
  send_data_[1] = static_cast<uint8_t>(id & 0xFF);

  ESP_LOGD(TAG, "Sending spawner ID: 0x%04X", id);

  // Single shot (loop_count 0) — spawners do not pair.
  return transmitter_->send(send_data_, 16, timeout_ms, 0);
}

esp_err_t Irt::start_receive() {
  if (!initialized_ || receiver_ == nullptr) {
    ESP_LOGE(TAG, "Receiver not available");
    return ESP_ERR_INVALID_STATE;
  }

  // Reset candidate state.
  candidate_id_ = 0;
  candidate_time_us_ = 0;
  has_candidate_ = false;

  esp_err_t err = receiver_->start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start receiver: %s", esp_err_to_name(err));
    return err;
  }

  receiving_ = true;
  ESP_LOGI(TAG, "Receive started");
  return ESP_OK;
}

esp_err_t Irt::stop_receive() {
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

bool Irt::process_packet(uint16_t id, int64_t now_us, IrtMessage* out) {
  int64_t delta_us = has_candidate_ ? (now_us - candidate_time_us_) : 0;
  IrtPairResult result =
      classify(has_candidate_, candidate_id_, id, delta_us, kPairWindowUs);

  switch (result) {
    case IrtPairResult::kHold:
    case IrtPairResult::kReplace:
      // No candidate, or a stale non-matching one within the window: keep the
      // most recent packet as the candidate, sliding past any prior one. This
      // makes X Y Y resolve as Gun(Y) — X is replaced by Y, then the second Y
      // pairs with it.
      candidate_id_ = id;
      candidate_time_us_ = now_us;
      has_candidate_ = true;
      return false;

    case IrtPairResult::kGun:
      ESP_LOGD(TAG, "Gun: 0x%04X (dt=%lld us)", candidate_id_,
               (long long)delta_us);
      out->id = candidate_id_;
      out->role = IrtRole::kGun;
      has_candidate_ = false;
      return true;

    case IrtPairResult::kSpawner:
      ESP_LOGD(TAG, "Spawner: 0x%04X (dt=%lld us)", candidate_id_,
               (long long)delta_us);
      out->id = candidate_id_;
      out->role = IrtRole::kSpawner;
      // The aged candidate was a lone packet; hold the incoming one next.
      candidate_id_ = id;
      candidate_time_us_ = now_us;
      has_candidate_ = true;
      return true;
  }
  return false;
}

bool Irt::try_receive(IrtMessage* msg) {
  if (!receiving_ || receiver_ == nullptr || msg == nullptr) {
    return false;
  }

  codec::IrMessage raw;
  while (receiver_->try_receive(&raw) == ESP_OK) {
    auto decoded = decode_message(raw);
    if (!decoded.has_value()) {
      continue;  // Drop undecodable frames.
    }
    if (process_packet(decoded.value(), esp_timer_get_time(), msg)) {
      return true;
    }
  }

  // No queued packet produced a result. Flush a lone candidate as a spawner
  // once it has aged past the pair window.
  if (has_candidate_ &&
      (esp_timer_get_time() - candidate_time_us_) > kPairWindowUs) {
    ESP_LOGD(TAG, "Spawner (flush): 0x%04X", candidate_id_);
    msg->id = candidate_id_;
    msg->role = IrtRole::kSpawner;
    has_candidate_ = false;
    return true;
  }

  return false;
}

esp_err_t Irt::receive(IrtMessage* msg, uint32_t timeout_ms) {
  if (!receiving_ || receiver_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  if (msg == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  const int64_t start_us = esp_timer_get_time();
  const int64_t timeout_us = static_cast<int64_t>(timeout_ms) * 1000;

  // Event-driven: block on the codec receive queue. Bound the wait so a held
  // candidate can be flushed as a spawner once the pair window elapses, even
  // when no further packet arrives. No busy-poll / vTaskDelay.
  while (true) {
    int64_t now_us = esp_timer_get_time();

    // Flush an aged candidate as a spawner.
    if (has_candidate_ && (now_us - candidate_time_us_) > kPairWindowUs) {
      ESP_LOGD(TAG, "Spawner (flush): 0x%04X", candidate_id_);
      msg->id = candidate_id_;
      msg->role = IrtRole::kSpawner;
      has_candidate_ = false;
      return ESP_OK;
    }

    int64_t elapsed_us = now_us - start_us;
    if (elapsed_us >= timeout_us) {
      return ESP_ERR_TIMEOUT;
    }

    // Bound the queue wait by the remaining timeout and, if a candidate is
    // held, by the time left until it must be flushed.
    int64_t wait_us = timeout_us - elapsed_us;
    if (has_candidate_) {
      int64_t until_flush_us = kPairWindowUs - (now_us - candidate_time_us_);
      if (until_flush_us < wait_us) {
        wait_us = until_flush_us;
      }
    }
    if (wait_us < 0) {
      wait_us = 0;
    }
    // Queue waits are tick-resolution; round up to whole ms, at least 1 so the
    // call blocks (event-driven) rather than spins.
    uint32_t wait_ms = static_cast<uint32_t>((wait_us + 999) / 1000);
    if (wait_ms == 0) {
      wait_ms = 1;
    }

    codec::IrMessage raw;
    esp_err_t err = receiver_->receive(&raw, wait_ms);
    if (err == ESP_OK) {
      auto decoded = decode_message(raw);
      if (decoded.has_value() &&
          process_packet(decoded.value(), esp_timer_get_time(), msg)) {
        return ESP_OK;
      }
    } else if (err != ESP_ERR_TIMEOUT) {
      return err;
    }
    // On timeout, loop to re-check flush / deadline.
  }
}

std::optional<uint16_t> Irt::decode_message(const codec::IrMessage& message) {
  if (!message.valid || message.bit_count != 16) {
    ESP_LOGD(TAG, "decode_message: rejected (valid=%d, bits=%u)", message.valid,
             (unsigned)message.bit_count);
    return std::nullopt;
  }

  uint16_t id = (static_cast<uint16_t>(message.data[0]) << 8) |
                static_cast<uint16_t>(message.data[1]);
  return id;
}

codec::ProtocolConfig make_irt_config() {
  return kIrtConfig;
}

}  // namespace protocols
}  // namespace ir_lasertag
