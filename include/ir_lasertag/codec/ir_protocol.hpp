/**
 * @file ir_protocol.hpp
 * @brief IR laser tag protocol abstraction over ESP32 RMT.
 *
 * This module provides configurable IR protocol encoding/decoding using
 * a half-bit model that supports Manchester, pulse width, and pulse
 * distance encoding schemes.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "driver/gpio.h"

namespace ir_lasertag {
namespace codec {

// ============================================================================
// Enums
// ============================================================================

/**
 * @brief Signal level for half-bit encoding.
 */
enum class Level : uint8_t {
  kSpace = 0,  ///< Carrier off (low)
  kMark = 1    ///< Carrier on (high)
};

/**
 * @brief Bit ordering in transmitted/received data.
 */
enum class BitOrder : uint8_t {
  kLsbFirst = 0,  ///< Least significant bit transmitted first
  kMsbFirst = 1   ///< Most significant bit transmitted first
};

// ============================================================================
// Half-Bit Timing Structure
// ============================================================================

/**
 * @brief Timing and level for one half of a bit period.
 */
struct HalfBit {
  uint16_t duration_us;  ///< Duration in microseconds
  Level level;           ///< Signal level (mark or space)

  /**
   * @brief Check if this half-bit represents a mark (carrier on).
   */
  bool is_mark() const { return level == Level::kMark; }

  /**
   * @brief Check if this half-bit represents a space (carrier off).
   */
  bool is_space() const { return level == Level::kSpace; }
};

// ============================================================================
// Protocol Configuration
// ============================================================================

/**
 * @brief IR protocol timing and encoding configuration.
 *
 * Uses a half-bit model where each data bit consists of two half-bit
 * periods. This supports Manchester (transition-based), pulse width,
 * and pulse distance encoding schemes.
 */
struct ProtocolConfig {
  // --- Carrier settings ---
  uint32_t carrier_freq_hz = 38000;  ///< Carrier frequency (typically 38kHz)
  float carrier_duty = 0.33f;        ///< Carrier duty cycle (0.0 - 1.0)

  // --- Header timing ---
  uint16_t header_mark_us = 0;   ///< Header mark duration (0 to skip header)
  uint16_t header_space_us = 0;  ///< Header space duration

  // --- Bit encoding (half-bit model) ---
  HalfBit zero_half1 = {600, Level::kMark};   ///< Zero bit, first half
  HalfBit zero_half2 = {600, Level::kSpace};  ///< Zero bit, second half
  HalfBit one_half1 = {600, Level::kMark};    ///< One bit, first half
  HalfBit one_half2 = {1200, Level::kSpace};  ///< One bit, second half

  // --- Footer ---
  uint16_t footer_mark_us = 0;      ///< Trailing mark (0 to skip)
  uint16_t footer_space_us = 0;     ///< Trailing space after footer mark (0 to skip)

  // --- Message structure ---
  BitOrder bit_order = BitOrder::kMsbFirst;  ///< Bit transmission order
  uint8_t bit_count = 0;  ///< Expected message bit count (0 = any valid count)

  // --- Receiver tolerances ---
  uint8_t tolerance_percent = 25;  ///< Timing tolerance for receive (%)

  /**
   * @brief Mark bias compensation for IR receiver demodulation.
   *
   * IR receivers often stretch marks and shorten spaces due to
   * demodulator characteristics. This value (in microseconds) is
   * subtracted from received mark durations and added to space
   * durations during decode to compensate.
   *
   * Example: If protocol specifies 600us mark + 600us space but
   * receiver returns 680us mark + 520us space, set mark_bias_us = 75.
   */
  int16_t mark_bias_us = 0;

  /**
   * @brief Check if configuration values are valid.
   * @return true if configuration is valid.
   */
  bool is_valid() const;

  /**
   * @brief Check if this configuration uses Manchester encoding.
   *
   * Manchester encoding is identified by the first half-bit levels
   * being opposite for zero and one (transition-based encoding).
   */
  bool is_manchester() const {
    return zero_half1.level != one_half1.level;
  }

  /**
   * @brief Get the total duration of a zero bit.
   */
  uint32_t zero_bit_duration_us() const {
    return zero_half1.duration_us + zero_half2.duration_us;
  }

  /**
   * @brief Get the total duration of a one bit.
   */
  uint32_t one_bit_duration_us() const {
    return one_half1.duration_us + one_half2.duration_us;
  }
};

// ============================================================================
// Message Structures
// ============================================================================

/**
 * @brief Received IR message.
 */
struct IrMessage {
  static constexpr size_t kMaxDataBytes = 16;  ///< Max 128 bits

  uint8_t data[kMaxDataBytes] = {};  ///< Received data bytes
  uint8_t bit_count = 0;             ///< Number of valid bits
  bool is_repeat = false;            ///< True if this is a repeat code
  bool valid = false;                ///< True if decoding succeeded
};

/**
 * @brief Raw IR timing for protocol analysis.
 */
struct RawTiming {
  uint16_t duration_us;  ///< Duration in microseconds
  Level level;           ///< Signal level (mark or space)
};

}  // namespace codec
}  // namespace ir_lasertag
