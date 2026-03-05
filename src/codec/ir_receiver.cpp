/**
 * @file ir_receiver.cpp
 * @brief IR receiver implementation.
 */

#include "ir_lasertag/codec/ir_receiver.hpp"

#include <cstring>

#include "esp_check.h"
#include "esp_log.h"
#include "driver/rmt_rx.h"

static const char* TAG = "ir_rx";

namespace ir_lasertag {
namespace codec {

namespace {

/**
 * @brief Queue item for raw symbols received from ISR.
 */
struct RawQueueItem {
  size_t num_symbols;
  // Symbols are stored in the shared buffer
};

}  // namespace

IrReceiver::IrReceiver() = default;

IrReceiver::~IrReceiver() {
  deinit();
}

esp_err_t IrReceiver::init(const rmt_rx_channel_config_t& config,
                           size_t symbol_buffer_size) {
  if (initialized_) {
    ESP_LOGE(TAG, "already initialized");
    return ESP_ERR_INVALID_STATE;
  }

  gpio_ = config.gpio_num;
  symbol_buffer_size_ = symbol_buffer_size;

  // Allocate symbol buffer
  symbol_buffer_ = static_cast<rmt_symbol_word_t*>(
      malloc(symbol_buffer_size_ * sizeof(rmt_symbol_word_t)));
  if (symbol_buffer_ == nullptr) {
    ESP_LOGE(TAG, "failed to allocate symbol buffer");
    return ESP_ERR_NO_MEM;
  }

  // Create message queue
  message_queue_ = xQueueCreate(4, sizeof(IrMessage));
  if (message_queue_ == nullptr) {
    ESP_LOGE(TAG, "failed to create message queue");
    free(symbol_buffer_);
    symbol_buffer_ = nullptr;
    return ESP_ERR_NO_MEM;
  }

  // Create raw queue
  raw_queue_ = xQueueCreate(2, sizeof(RawQueueItem));
  if (raw_queue_ == nullptr) {
    ESP_LOGE(TAG, "failed to create raw queue");
    vQueueDelete(message_queue_);
    message_queue_ = nullptr;
    free(symbol_buffer_);
    symbol_buffer_ = nullptr;
    return ESP_ERR_NO_MEM;
  }

  // Create RMT RX channel
  esp_err_t ret = rmt_new_rx_channel(&config, &channel_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to create RX channel: %s", esp_err_to_name(ret));
    vQueueDelete(raw_queue_);
    raw_queue_ = nullptr;
    vQueueDelete(message_queue_);
    message_queue_ = nullptr;
    free(symbol_buffer_);
    symbol_buffer_ = nullptr;
    return ret;
  }

  // Register receive callback
  rmt_rx_event_callbacks_t cbs = {
      .on_recv_done = rmt_rx_done_callback,
  };

  ret = rmt_rx_register_event_callbacks(channel_, &cbs, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to register callbacks: %s", esp_err_to_name(ret));
    rmt_del_channel(channel_);
    channel_ = nullptr;
    vQueueDelete(raw_queue_);
    raw_queue_ = nullptr;
    vQueueDelete(message_queue_);
    message_queue_ = nullptr;
    free(symbol_buffer_);
    symbol_buffer_ = nullptr;
    return ret;
  }

  initialized_ = true;
  ESP_LOGI(TAG, "initialized on GPIO %d",
           static_cast<int>(gpio_));

  return ESP_OK;
}

void IrReceiver::deinit() {
  if (!initialized_) {
    return;
  }

  if (receiving_) {
    stop();
  }

  if (channel_) {
    rmt_del_channel(channel_);
    channel_ = nullptr;
  }

  if (raw_queue_) {
    vQueueDelete(raw_queue_);
    raw_queue_ = nullptr;
  }

  if (message_queue_) {
    vQueueDelete(message_queue_);
    message_queue_ = nullptr;
  }

  if (symbol_buffer_) {
    free(symbol_buffer_);
    symbol_buffer_ = nullptr;
  }

  initialized_ = false;
  protocol_set_ = false;
  ESP_LOGI(TAG, "deinitialized");
}

esp_err_t IrReceiver::set_protocol(const ProtocolConfig& config) {
  if (!initialized_) {
    ESP_LOGE(TAG, "not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!config.is_valid()) {
    ESP_LOGE(TAG, "invalid protocol config");
    return ESP_ERR_INVALID_ARG;
  }

  protocol_ = config;
  protocol_set_ = true;

  ESP_LOGI(TAG, "protocol set: tolerance=%d%%", protocol_.tolerance_percent);

  return ESP_OK;
}

esp_err_t IrReceiver::enable_raw_mode() {
  raw_mode_ = true;
  return ESP_OK;
}

esp_err_t IrReceiver::disable_raw_mode() {
  raw_mode_ = false;
  return ESP_OK;
}

esp_err_t IrReceiver::start() {
  if (!initialized_) {
    ESP_LOGE(TAG, "not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (receiving_) {
    return ESP_OK;  // Already receiving
  }

  esp_err_t ret = rmt_enable(channel_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to enable channel: %s", esp_err_to_name(ret));
    return ret;
  }

  // Start receiving
  restart_receive();

  receiving_ = true;
  ESP_LOGI(TAG, "started receiving");

  return ESP_OK;
}

esp_err_t IrReceiver::stop() {
  if (!initialized_ || !receiving_) {
    return ESP_OK;
  }

  esp_err_t ret = rmt_disable(channel_);
  receiving_ = false;

  ESP_LOGI(TAG, "stopped receiving");

  return ret;
}

esp_err_t IrReceiver::receive(IrMessage* message, uint32_t timeout_ms) {
  if (!initialized_) {
    return ESP_ERR_INVALID_STATE;
  }

  if (raw_mode_) {
    ESP_LOGE(TAG, "cannot receive decoded in raw mode");
    return ESP_ERR_INVALID_STATE;
  }

  if (!protocol_set_) {
    ESP_LOGE(TAG, "protocol not set");
    return ESP_ERR_INVALID_STATE;
  }

  if (message == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xQueueReceive(message_queue_, message, pdMS_TO_TICKS(timeout_ms))) {
    return ESP_OK;
  }

  return ESP_ERR_TIMEOUT;
}

esp_err_t IrReceiver::receive_raw(RawTiming* timings, size_t max_count,
                                  size_t* received_count, uint32_t timeout_ms) {
  if (!initialized_) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!raw_mode_) {
    ESP_LOGE(TAG, "raw mode not enabled");
    return ESP_ERR_INVALID_STATE;
  }

  if (timings == nullptr || received_count == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  RawQueueItem item;
  if (xQueueReceive(raw_queue_, &item, pdMS_TO_TICKS(timeout_ms))) {
    size_t copy_count = (item.num_symbols < max_count)
                            ? item.num_symbols
                            : max_count;

    // Convert RMT symbols to RawTiming
    for (size_t i = 0; i < copy_count; i++) {
      // Each RMT symbol has two parts (duration0/level0, duration1/level1)
      // We output them as separate RawTiming entries
      // But for simplicity, treat each symbol as one timing (mark or space)
      const auto& sym = symbol_buffer_[i];
      
      // First part of symbol
      if (i * 2 < max_count) {
        timings[i * 2].duration_us = sym.duration0;
        timings[i * 2].level = sym.level0 ? Level::kMark : Level::kSpace;
      }
      // Second part of symbol
      if (i * 2 + 1 < max_count && sym.duration1 > 0) {
        timings[i * 2 + 1].duration_us = sym.duration1;
        timings[i * 2 + 1].level = sym.level1 ? Level::kMark : Level::kSpace;
      }
    }

    *received_count = copy_count * 2;  // Each symbol has 2 parts
    return ESP_OK;
  }

  return ESP_ERR_TIMEOUT;
}

bool IRAM_ATTR IrReceiver::rmt_rx_done_callback(
    rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t* edata,
    void* user_ctx) {
  auto* receiver = static_cast<IrReceiver*>(user_ctx);
  bool high_prio_woken = receiver->on_receive_done(edata);

  // Re-start receiving (from ISR context)
  rmt_receive_config_t rx_config = {
      .signal_range_min_ns = 1000,      // 1us minimum
      .signal_range_max_ns = 12000000,  // 12ms maximum (long gaps)
      .flags = {
          .en_partial_rx = 0,
      },
  };

  rmt_receive(channel, receiver->symbol_buffer_,
              receiver->symbol_buffer_size_ * sizeof(rmt_symbol_word_t),
              &rx_config);

  return high_prio_woken;
}

bool IRAM_ATTR IrReceiver::on_receive_done(
    const rmt_rx_done_event_data_t* edata) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (raw_mode_) {
    // Queue raw symbols
    RawQueueItem item = {
        .num_symbols = edata->num_symbols,
    };
    xQueueSendFromISR(raw_queue_, &item, &xHigherPriorityTaskWoken);
  } else if (protocol_set_) {
    // Decode message and queue it
    IrMessage message;
    if (decode_symbols(edata->received_symbols, edata->num_symbols, &message)) {
      xQueueSendFromISR(message_queue_, &message, &xHigherPriorityTaskWoken);
    }
  }

  return xHigherPriorityTaskWoken == pdTRUE;
}

void IrReceiver::restart_receive() {
  rmt_receive_config_t rx_config = {
      .signal_range_min_ns = 1000,      // 1us minimum
      .signal_range_max_ns = 12000000,  // 12ms maximum
      .flags = {
          .en_partial_rx = 0,
      },
  };

  rmt_receive(channel_, symbol_buffer_,
              symbol_buffer_size_ * sizeof(rmt_symbol_word_t),
              &rx_config);
}

bool IrReceiver::timing_matches(uint16_t actual, uint16_t expected,
                                 bool is_mark) const {
  if (expected == 0) {
    return actual == 0;
  }

  // Apply mark_bias compensation:
  // - Marks are elongated by receiver, so subtract bias
  // - Spaces are shortened by receiver, so add bias
  int32_t compensated = static_cast<int32_t>(actual);
  if (is_mark) {
    compensated -= protocol_.mark_bias_us;
  } else {
    compensated += protocol_.mark_bias_us;
  }
  // Clamp to valid range
  if (compensated < 0) {
    compensated = 0;
  }

  uint32_t tolerance = (expected * protocol_.tolerance_percent) / 100;
  uint16_t min_val = (expected > tolerance) ? (expected - tolerance) : 0;
  uint16_t max_val = expected + tolerance;

  return compensated >= min_val && compensated <= max_val;
}

bool IrReceiver::decode_symbols(const rmt_symbol_word_t* symbols, size_t count,
                                IrMessage* message) {
  if (count < 2 || symbols == nullptr || message == nullptr) {
    return false;
  }

  memset(message, 0, sizeof(IrMessage));

  // Dispatch to Manchester decoder if protocol uses Manchester encoding
  if (protocol_.is_manchester()) {
    return decode_manchester(symbols, count, message);
  }

  size_t sym_idx = 0;

  // Check for header
  if (protocol_.header_mark_us > 0) {
    const auto& sym = symbols[sym_idx];

    // Header should be mark followed by space
    if (!timing_matches(sym.duration0, protocol_.header_mark_us, true)) {
      return false;  // Header mark doesn't match
    }
    if (!timing_matches(sym.duration1, protocol_.header_space_us, false)) {
      return false;  // Header space doesn't match
    }

    sym_idx++;
  }

  // Decode data bits using half-bit model
  // Each data bit consists of two half-bits
  // We need to match the pattern: half1 + half2

  bool msb_first = (protocol_.bit_order == BitOrder::kMsbFirst);

  while (sym_idx < count && message->bit_count < IrMessage::kMaxDataBytes * 8) {
    // Read two consecutive half-bits
    // RMT symbols come as mark/space pairs, so we need to reconstruct
    // the half-bit sequence

    // Get current symbol
    const auto& sym = symbols[sym_idx];

    // End of message detection
    if (sym.duration0 == 0 && sym.duration1 == 0) {
      break;
    }

    // Footer detection
    if (protocol_.footer_mark_us > 0 &&
        timing_matches(sym.duration0, protocol_.footer_mark_us, true) &&
        sym.duration1 == 0) {
      break;  // Footer found, end of message
    }

    // Try to match "one" bit
    // duration0 is the first half (level0), duration1 is the second half (level1)
    bool one_half1_matches = timing_matches(
        sym.duration0, protocol_.one_half1.duration_us,
        sym.level0 == 1);
    bool one_half2_matches = timing_matches(
        sym.duration1, protocol_.one_half2.duration_us,
        sym.level1 == 1);

    // Try to match "zero" bit
    bool zero_half1_matches = timing_matches(
        sym.duration0, protocol_.zero_half1.duration_us,
        sym.level0 == 1);
    bool zero_half2_matches = timing_matches(
        sym.duration1, protocol_.zero_half2.duration_us,
        sym.level1 == 1);

    // Determine decoded bit value (-1 = not decoded)
    int decoded_bit = -1;

    // Case 1: Both halves match a complete bit
    if (one_half1_matches && one_half2_matches) {
      decoded_bit = 1;
    } else if (zero_half1_matches && zero_half2_matches) {
      decoded_bit = 0;
    } else if (!one_half2_matches && !zero_half2_matches) {
      // Case 2: Second half doesn't match either pattern. This occurs on
      // the last bit when the trailing space extends into the message gap
      // (or duration1 is 0 at end of RMT reception). Fall back to
      // first-half-only matching for pulse-width protocols where mark
      // durations are distinct between zero and one.
      if (one_half1_matches && !zero_half1_matches) {
        decoded_bit = 1;
      } else if (zero_half1_matches && !one_half1_matches) {
        decoded_bit = 0;
      }
    }

    if (decoded_bit < 0) {
      // Cannot decode this symbol - end of message or error
      break;
    }

    // Store bit in data buffer
    size_t byte_idx = message->bit_count / 8;
    size_t bit_idx = message->bit_count % 8;

    if (decoded_bit == 1) {
      if (msb_first) {
        message->data[byte_idx] |= (0x80 >> bit_idx);
      } else {
        message->data[byte_idx] |= (1 << bit_idx);
      }
    }
    // For zero, bit is already 0

    message->bit_count++;
    sym_idx++;

    // If expected bit count is specified and reached, stop decoding
    if (protocol_.bit_count > 0 && message->bit_count >= protocol_.bit_count) {
      break;
    }
  }

  // Validate bit count if expected count is specified
  if (protocol_.bit_count > 0) {
    message->valid = (message->bit_count == protocol_.bit_count);
  } else {
    message->valid = (message->bit_count > 0);
  }
  return message->valid;
}

// ============================================================================
// Manchester Decoder
// ============================================================================

namespace {

/**
 * @brief Half-symbol for Manchester decoding.
 *
 * Represents one half of an RMT symbol (duration0/level0 or
 * duration1/level1). Duration is consumed incrementally as half-bit
 * durations are matched. When duration reaches 0, the next half-symbol
 * is loaded.
 */
struct HalfSymbol {
  int16_t duration;   ///< Remaining duration (consumed incrementally)
  uint16_t level;     ///< Signal level (1 = mark, 0 = space)
};

/**
 * @brief Consume a half-bit duration from a half-symbol.
 *
 * Checks that the half-symbol has the expected level and at least the
 * expected duration (within tolerance). If the half-symbol is longer
 * than expected, only the needed portion is consumed and the remainder
 * stays. If it matches exactly (within tolerance), duration is set to 0
 * to signal that the next half-symbol should be loaded.
 *
 * @param hs Half-symbol to consume from (modified in place).
 * @param want_duration Expected duration in microseconds.
 * @param want_level Expected signal level (1 = mark, 0 = space).
 * @param tolerance Timing tolerance in microseconds.
 * @return true if consumption succeeded.
 */
inline bool consume_half_symbol(HalfSymbol* hs, uint16_t want_duration,
                                uint16_t want_level, uint16_t tolerance) {
  // Level must match
  if (hs->level != want_level) {
    return false;
  }

  // Duration must be at least (want_duration - tolerance)
  int16_t min_duration = static_cast<int16_t>(want_duration) -
                         static_cast<int16_t>(tolerance);
  if (min_duration < 0) {
    min_duration = 0;
  }
  if (hs->duration < min_duration) {
    return false;
  }

  // Check if this is an exact match (fully consumed)
  int16_t max_duration = static_cast<int16_t>(want_duration) +
                         static_cast<int16_t>(tolerance);
  if (hs->duration <= max_duration) {
    // Fully consumed
    hs->duration = 0;
    return true;
  }

  // Partial consumption — duration is longer than expected, subtract
  // only the wanted amount. The remainder will be consumed by the next
  // half-bit (this happens when adjacent same-level half-bits merge).
  hs->duration -= static_cast<int16_t>(want_duration);
  return true;
}

}  // namespace

bool IrReceiver::decode_manchester(const rmt_symbol_word_t* symbols,
                                   size_t count, IrMessage* message) {
  // Manchester encoding produces merged symbols when adjacent half-bits
  // share the same level. For example, bit sequence "10" in
  // zero=MARK→SPACE, one=SPACE→MARK encoding:
  //   ...MARK(half) | MARK(half)... merges into MARK(2*half)
  //
  // The approach: split each RMT symbol into two half-symbols and
  // consume durations in half-bit increments. When a half-symbol has
  // more duration than one half-bit, only the needed portion is consumed
  // and the remainder carries forward to the next half-bit.

  const uint16_t half_bit_us = protocol_.zero_half1.duration_us;
  const uint16_t tolerance = (half_bit_us * protocol_.tolerance_percent) / 100;

  // State for iterating through half-symbols
  int sym_idx = -1;
  HalfSymbol half_buf[2] = {{0, 0}, {0, 0}};
  HalfSymbol* current_half = &half_buf[0];
  bool on_second_half = true;  // Start true so first advance loads symbol 0

  // Lambda to advance to the next half-symbol when current is consumed.
  // Returns false when no more symbols are available.
  auto advance_half = [&]() -> bool {
    if (current_half->duration != 0) {
      return true;  // Current half still has data
    }

    // Flip to next half
    on_second_half = !on_second_half;

    if (!on_second_half) {
      // Moving to first half of next RMT symbol
      sym_idx++;
      if (sym_idx >= static_cast<int>(count)) {
        return false;  // No more symbols
      }

      // Apply mark_bias compensation: marks are elongated by the
      // receiver demodulator, so subtract bias from marks and add
      // to spaces.
      int16_t d0 = static_cast<int16_t>(symbols[sym_idx].duration0);
      int16_t d1 = static_cast<int16_t>(symbols[sym_idx].duration1);
      if (symbols[sym_idx].level0 == 1) {
        d0 -= protocol_.mark_bias_us;
      } else {
        d0 += protocol_.mark_bias_us;
      }
      if (symbols[sym_idx].level1 == 1) {
        d1 -= protocol_.mark_bias_us;
      } else {
        d1 += protocol_.mark_bias_us;
      }
      if (d0 < 0) d0 = 0;
      if (d1 < 0) d1 = 0;

      half_buf[0] = {d0, symbols[sym_idx].level0};
      half_buf[1] = {d1, symbols[sym_idx].level1};
    }

    current_half = &half_buf[on_second_half ? 1 : 0];
    return true;
  };

  // --- Consume header mark ---
  if (protocol_.header_mark_us > 0) {
    if (!advance_half()) {
      return false;
    }
    uint16_t header_tol = (protocol_.header_mark_us *
                           protocol_.tolerance_percent) / 100;
    if (!consume_half_symbol(current_half, protocol_.header_mark_us,
                             1, header_tol) ||
        current_half->duration != 0) {
      return false;  // Header mark must be fully consumed
    }

    // --- Consume header space ---
    // The header space may partially merge with the first data half-bit
    // if the first data bit starts with a space (i.e., a "one" bit in
    // standard Manchester where one = SPACE→MARK). So we allow partial
    // consumption here.
    if (!advance_half()) {
      return false;
    }
    uint16_t header_space_tol = (protocol_.header_space_us *
                                 protocol_.tolerance_percent) / 100;
    if (!consume_half_symbol(current_half, protocol_.header_space_us,
                             0, header_space_tol)) {
      return false;  // Header space doesn't match
    }
  }

  // --- Decode data bits ---
  bool msb_first = (protocol_.bit_order == BitOrder::kMsbFirst);
  uint8_t max_bits = (protocol_.bit_count > 0)
                         ? protocol_.bit_count
                         : (IrMessage::kMaxDataBytes * 8);

  // Determine which level represents "one" first-half.
  // In our convention: zero = MARK→SPACE, one = SPACE→MARK
  // So one_half1 is SPACE (level 0), zero_half1 is MARK (level 1).
  const uint16_t zero_first_level =
      (protocol_.zero_half1.level == Level::kMark) ? 1 : 0;
  const uint16_t one_first_level =
      (protocol_.one_half1.level == Level::kMark) ? 1 : 0;

  for (uint8_t bit_num = 0; bit_num < max_bits; bit_num++) {
    if (!advance_half()) {
      break;  // No more data
    }

    // Determine bit value from the level of the current half-symbol.
    // The first half-bit of a Manchester bit determines its value.
    bool is_one;
    if (current_half->level == one_first_level) {
      is_one = true;
    } else if (current_half->level == zero_first_level) {
      is_one = false;
    } else {
      break;  // Unexpected level
    }

    // Consume first half-bit
    uint16_t expected_first_level = is_one ? one_first_level : zero_first_level;
    if (!consume_half_symbol(current_half, half_bit_us,
                             expected_first_level, tolerance)) {
      break;
    }

    // Consume second half-bit
    if (!advance_half()) {
      // Special case: last bit ending low — RMT may report duration 0
      // for the trailing space since it detects end-of-frame. If we've
      // already identified the bit from its first half, accept it.
      if (bit_num == max_bits - 1) {
        // Store the bit and break
      } else {
        break;
      }
    } else {
      uint16_t expected_second_level = is_one
          ? ((protocol_.one_half2.level == Level::kMark) ? 1 : 0)
          : ((protocol_.zero_half2.level == Level::kMark) ? 1 : 0);

      // On the last bit, if it ends with a space (level 0) and the
      // RMT reports duration 0, accept it — the RMT signals
      // end-of-frame by truncating the final space.
      bool is_last_bit = (protocol_.bit_count > 0) &&
                         (bit_num == max_bits - 1);
      if (is_last_bit && current_half->duration == 0 &&
          current_half->level == 0 && expected_second_level == 0) {
        // Accept — end of frame
      } else {
        if (!consume_half_symbol(current_half, half_bit_us,
                                 expected_second_level, tolerance)) {
          break;
        }
      }
    }

    // Store bit in data buffer
    size_t byte_idx = message->bit_count / 8;
    size_t bit_idx = message->bit_count % 8;
    if (is_one) {
      if (msb_first) {
        message->data[byte_idx] |= (0x80 >> bit_idx);
      } else {
        message->data[byte_idx] |= (1 << bit_idx);
      }
    }
    message->bit_count++;
  }

  // Validate bit count
  if (protocol_.bit_count > 0) {
    message->valid = (message->bit_count == protocol_.bit_count);
  } else {
    message->valid = (message->bit_count > 0);
  }
  return message->valid;
}

}  // namespace codec
}  // namespace ir_lasertag
