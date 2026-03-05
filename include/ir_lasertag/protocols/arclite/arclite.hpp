/**
 * @file arclite.hpp
 * @brief Arclite Tag Code protocol wrapper.
 *
 * Provides send/receive operations for Arclite 16-bit Tag Codes
 * over Manchester-encoded IR. Tag Codes uniquely identify emitter
 * capabilities within a game session.
 *
 * This wrapper is application-layer code that runs in task context,
 * using the codec layer's queue-based receive API.
 */

#pragma once

#include <cstdint>
#include <optional>

#include "esp_err.h"
#include "ir_lasertag/codec/ir_protocol.hpp"
#include "ir_lasertag/codec/ir_receiver.hpp"
#include "ir_lasertag/codec/ir_transmitter.hpp"

namespace ir_lasertag {
namespace protocols {

// ============================================================================
// Arclite Protocol Constants
// ============================================================================

/// @name Timing Constants
/// @{

/// Carrier frequency in Hz
constexpr uint32_t kArcliteCarrierHz = 38000;

/// Header mark duration in microseconds
constexpr uint16_t kArcliteHeaderMarkUs = 520;

/// Header space duration in microseconds
constexpr uint16_t kArcliteHeaderSpaceUs = 520;

/// Half-bit duration in microseconds (Manchester encoding)
constexpr uint16_t kArcliteHalfBitUs = 300;

/// Minimum gap between messages in microseconds
constexpr uint32_t kArcliteMessageGapUs = 2500;

/// Message bit count
constexpr uint8_t kArcliteBitCount = 16;

/// @}

/**
 * @brief Default Arclite protocol configuration.
 *
 * Manchester encoding:
 * - 520µs header mark, 520µs header space
 * - 0 bit: MARK(300µs) → SPACE(300µs) (high-to-low transition)
 * - 1 bit: SPACE(300µs) → MARK(300µs) (low-to-high transition)
 * - 16-bit messages, MSB first
 * - 38kHz carrier
 * - 2500µs minimum interval between messages
 */
constexpr codec::ProtocolConfig kArcliteConfig = {
    .carrier_freq_hz = kArcliteCarrierHz,
    .carrier_duty = 0.33f,
    .header_mark_us = kArcliteHeaderMarkUs,
    .header_space_us = kArcliteHeaderSpaceUs,
    .zero_half1 = {kArcliteHalfBitUs, codec::Level::kMark},
    .zero_half2 = {kArcliteHalfBitUs, codec::Level::kSpace},
    .one_half1 = {kArcliteHalfBitUs, codec::Level::kSpace},
    .one_half2 = {kArcliteHalfBitUs, codec::Level::kMark},
    .footer_mark_us = 0,
    .bit_order = codec::BitOrder::kMsbFirst,
    .bit_count = kArcliteBitCount,
    .tolerance_percent = 25,
    .mark_bias_us = 0,
};

// ============================================================================
// ArcliteTagCode Class
// ============================================================================

/**
 * @brief Arclite Tag Code protocol wrapper.
 *
 * Wraps the IR codec layer to provide a Tag Code-level API. Each
 * Tag Code is a 16-bit identifier issued to an emitter capability
 * for a game session. The Arclite system uses IR for collision
 * detection: when a sensor receives a Tag Code, the collision event
 * is forwarded to the game engine.
 *
 * This wrapper runs in task context and uses blocking/non-blocking
 * queue-based receive operations from the codec layer.
 *
 * Usage:
 * @code
 *   ArcliteTagCode atc;
 *   atc.init(tx_channel, rx_channel, arclite_config);
 *
 *   // Send a tag code (blocking)
 *   atc.send(0x1234);
 *
 *   // Poll for received tag codes (non-blocking)
 *   uint16_t tag_code;
 *   if (atc.try_receive(&tag_code)) {
 *     ESP_LOGI("rx", "Got tag code: 0x%04X", tag_code);
 *   }
 *
 *   // Or block until one is received
 *   if (atc.receive(&tag_code, 1000) == ESP_OK) {
 *     ESP_LOGI("rx", "Got tag code: 0x%04X", tag_code);
 *   }
 * @endcode
 */
class ArcliteTagCode {
 public:
  ArcliteTagCode();
  ~ArcliteTagCode();

  // Non-copyable
  ArcliteTagCode(const ArcliteTagCode&) = delete;
  ArcliteTagCode& operator=(const ArcliteTagCode&) = delete;

  /**
   * @brief Initialize with pre-configured transmitter and receiver.
   *
   * The caller retains ownership of the transmitter and receiver.
   * Both must remain valid for the lifetime of this object (or until
   * deinit() is called). Either may be nullptr if only send or
   * receive is needed.
   *
   * The protocol configuration is applied to both the transmitter
   * and receiver.
   *
   * @param transmitter Pointer to an initialized IrTransmitter (nullable).
   * @param receiver Pointer to an initialized IrReceiver (nullable).
   * @param config Arclite protocol configuration.
   * @return ESP_OK on success, ESP_ERR_INVALID_ARG if both are null,
   *         or error from set_protocol().
   */
  esp_err_t init(codec::IrTransmitter* transmitter, codec::IrReceiver* receiver,
                 const codec::ProtocolConfig& config);

  /**
   * @brief Release references to transmitter/receiver.
   *
   * Stops any active receive operation. Does not deinitialize the
   * underlying transmitter/receiver — the caller manages their
   * lifetime.
   */
  void deinit();

  /**
   * @brief Check if the protocol wrapper is initialized.
   *
   * @return true if init() has been called successfully.
   */
  bool is_initialized() const { return initialized_; }

  /**
   * @brief Transmit a 16-bit tag code (blocking).
   *
   * Encodes and transmits the tag code as a single Manchester-encoded
   * 16-bit message. Blocks until transmission is complete.
   *
   * All tag code values are valid. By convention, 0x0000 means "unset"
   * and will record a warning in the game engine.
   *
   * @param tag_code The 16-bit tag code to transmit.
   * @param timeout_ms Maximum time to wait for transmission (default 1000ms).
   * @return ESP_OK on success, ESP_ERR_INVALID_STATE if no transmitter,
   *         ESP_ERR_TIMEOUT on timeout, or codec error.
   */
  esp_err_t send(uint16_t tag_code, uint32_t timeout_ms = 1000);

  /**
   * @brief Transmit a 16-bit tag code (non-blocking).
   *
   * Encodes and starts transmitting the tag code. Returns immediately
   * after the RMT hardware begins transmission. The encoded data is
   * stored in an internal buffer that persists until the next send.
   *
   * Use is_busy() on the underlying transmitter to check completion.
   *
   * @param tag_code The 16-bit tag code to transmit.
   * @return ESP_OK on success, ESP_ERR_INVALID_STATE if no transmitter
   *         or transmitter busy.
   */
  esp_err_t send_async(uint16_t tag_code);

  /**
   * @brief Start the receiver for polling tag codes.
   *
   * After calling this, use try_receive() or receive() to poll
   * for received tag codes.
   *
   * @return ESP_OK on success, ESP_ERR_INVALID_STATE if no receiver.
   */
  esp_err_t start_receive();

  /**
   * @brief Stop receiving tag codes.
   *
   * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not receiving.
   */
  esp_err_t stop_receive();

  /**
   * @brief Check if receiving is active.
   *
   * @return true if start_receive() has been called and not yet stopped.
   */
  bool is_receiving() const { return receiving_; }

  /**
   * @brief Try to receive a tag code without blocking.
   *
   * Polls the receive queue. Returns immediately whether or not
   * a tag code is available.
   *
   * @param[out] tag_code Receives the tag code if one is available.
   * @return true if a tag code was received, false if queue is empty.
   */
  bool try_receive(uint16_t* tag_code);

  /**
   * @brief Receive a tag code with timeout.
   *
   * Blocks until a tag code is received or timeout expires.
   *
   * @param[out] tag_code Receives the tag code.
   * @param timeout_ms Maximum time to wait in milliseconds.
   * @return ESP_OK if received, ESP_ERR_TIMEOUT on timeout,
   *         ESP_ERR_INVALID_STATE if not receiving.
   */
  esp_err_t receive(uint16_t* tag_code, uint32_t timeout_ms);

 private:
  /**
   * @brief Decode an IR message to a tag code.
   *
   * @param message The received IR message.
   * @return The tag code if valid, std::nullopt otherwise.
   */
  static std::optional<uint16_t> decode_message(const codec::IrMessage& message);

  codec::IrTransmitter* transmitter_ = nullptr;
  codec::IrReceiver* receiver_ = nullptr;
  uint8_t send_data_[2] = {};  ///< Persistent buffer for async send
  bool initialized_ = false;
  bool receiving_ = false;
};

/**
 * @brief Get default Arclite protocol configuration.
 *
 * Returns a copy of kArcliteConfig. Prefer using the constexpr
 * kArcliteConfig directly when a reference suffices.
 *
 * @return Protocol configuration for Arclite Tag Code.
 * @see kArcliteConfig
 */
codec::ProtocolConfig make_arclite_config();

}  // namespace protocols
}  // namespace ir_lasertag
