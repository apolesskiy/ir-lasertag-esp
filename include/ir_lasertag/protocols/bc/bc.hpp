/**
 * @file bc.hpp
 * @brief BC Gun protocol wrapper.
 *
 * Provides send/receive operations for BC 16-bit gun ID packets.
 * BC taggers broadcast their 16-bit ID only; all game logic is
 * handled internally by the tagger. Each transmission is a single
 * 16-bit packet (no paired-packet repeat).
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
// BC Gun Protocol Constants
// ============================================================================

/// @name Timing Constants
/// @{

/// Carrier frequency in Hz
constexpr uint32_t kBcCarrierHz = 38000;

/// Header mark duration in microseconds
constexpr uint16_t kBcHeaderMarkUs = 1900;

/// Header space duration in microseconds
constexpr uint16_t kBcHeaderSpaceUs = 450;

/// Zero bit mark duration in microseconds
constexpr uint16_t kBcZeroMarkUs = 500;

/// One bit mark duration in microseconds
constexpr uint16_t kBcOneMarkUs = 1000;

/// Bit space duration in microseconds (shared by zero and one bits)
constexpr uint16_t kBcBitSpaceUs = 450;

/// Message bit count
constexpr uint8_t kBcBitCount = 16;

/// @}

/**
 * @brief Default BC Gun protocol configuration.
 *
 * Pulse-width encoding:
 * - 1900µs header mark, 450µs header space
 * - 0 bit: 500µs mark, 450µs space
 * - 1 bit: 1000µs mark, 450µs space
 * - 16-bit messages, MSB first
 * - 38kHz carrier
 * - No repeat
 */
constexpr codec::ProtocolConfig kBcConfig = {
    .carrier_freq_hz = kBcCarrierHz,
    .carrier_duty = 0.33f,
    .header_mark_us = kBcHeaderMarkUs,
    .header_space_us = kBcHeaderSpaceUs,
    .zero_half1 = {kBcZeroMarkUs, codec::Level::kMark},
    .zero_half2 = {kBcBitSpaceUs, codec::Level::kSpace},
    .one_half1 = {kBcOneMarkUs, codec::Level::kMark},
    .one_half2 = {kBcBitSpaceUs, codec::Level::kSpace},
    .footer_mark_us = 0,
    .footer_space_us = 0,
    .bit_order = codec::BitOrder::kMsbFirst,
    .bit_count = kBcBitCount,
    .tolerance_percent = 25,
    .mark_bias_us = 0,
};

// ============================================================================
// BcGun Class
// ============================================================================

/**
 * @brief BC Gun protocol wrapper.
 *
 * Wraps the IR codec layer to provide a BC Gun-level API. BC guns
 * broadcast a 16-bit ID per shot using pulse-width encoding. Unlike
 * IRT, the message is sent once with no pair validation.
 *
 * This wrapper runs in task context and uses blocking/non-blocking
 * queue-based receive operations from the codec layer.
 *
 * Usage:
 * @code
 *   BcGun gun;
 *   gun.init(tx_channel, rx_channel, bc_config);
 *
 *   // Send a gun ID
 *   gun.send(0x1234);
 *
 *   // Poll for received gun IDs (non-blocking)
 *   uint16_t gun_id;
 *   if (gun.try_receive(&gun_id)) {
 *     ESP_LOGI("rx", "Got BC gun ID: 0x%04X", gun_id);
 *   }
 *
 *   // Or block until one is received
 *   if (gun.receive(&gun_id, 1000) == ESP_OK) {
 *     ESP_LOGI("rx", "Got BC gun ID: 0x%04X", gun_id);
 *   }
 * @endcode
 */
class BcGun {
 public:
  BcGun();
  ~BcGun();

  // Non-copyable
  BcGun(const BcGun&) = delete;
  BcGun& operator=(const BcGun&) = delete;

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
   * @param config BC Gun protocol configuration.
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
   * @brief Transmit a 16-bit gun ID (blocking).
   *
   * Sends the gun ID as a single 16-bit packet. Blocks until
   * transmission is complete.
   *
   * @param gun_id The 16-bit gun ID to transmit.
   * @param timeout_ms Maximum time to wait for transmission (default 1000ms).
   * @return ESP_OK on success, ESP_ERR_INVALID_STATE if no transmitter,
   *         ESP_ERR_TIMEOUT on timeout, or codec error.
   */
  esp_err_t send(uint16_t gun_id, uint32_t timeout_ms = 1000);

  /**
   * @brief Transmit a 16-bit gun ID (non-blocking).
   *
   * Encodes and starts transmitting the gun ID. Returns immediately
   * after the RMT hardware begins transmission. The encoded data is
   * stored in an internal buffer that persists until the next send.
   *
   * Use is_busy() on the underlying transmitter to check completion.
   *
   * @param gun_id The 16-bit gun ID to transmit.
   * @return ESP_OK on success, ESP_ERR_INVALID_STATE if no transmitter
   *         or transmitter busy.
   */
  esp_err_t send_async(uint16_t gun_id);

  /**
   * @brief Start the receiver for polling gun IDs.
   *
   * After calling this, use try_receive() or receive() to poll
   * for received gun IDs.
   *
   * @return ESP_OK on success, ESP_ERR_INVALID_STATE if no receiver.
   */
  esp_err_t start_receive();

  /**
   * @brief Stop receiving gun IDs.
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
   * @brief Try to receive a gun ID without blocking.
   *
   * Polls the receive queue. Returns immediately whether or not
   * a gun ID is available.
   *
   * @param[out] gun_id Receives the gun ID if one is available.
   * @return true if a gun ID was received, false if queue is empty.
   */
  bool try_receive(uint16_t* gun_id);

  /**
   * @brief Receive a gun ID with timeout.
   *
   * Blocks until a gun ID is received or timeout expires.
   *
   * @param[out] gun_id Receives the gun ID.
   * @param timeout_ms Maximum time to wait in milliseconds.
   * @return ESP_OK if received, ESP_ERR_TIMEOUT on timeout,
   *         ESP_ERR_INVALID_STATE if not receiving.
   */
  esp_err_t receive(uint16_t* gun_id, uint32_t timeout_ms);

 private:
  /**
   * @brief Decode an IR message to a gun ID.
   *
   * @param message The received IR message.
   * @return The gun ID if valid, std::nullopt otherwise.
   */
  static std::optional<uint16_t> decode_message(const codec::IrMessage& message);

  codec::IrTransmitter* transmitter_ = nullptr;
  codec::IrReceiver* receiver_ = nullptr;
  uint8_t send_data_[2] = {};  ///< Persistent buffer for async send
  bool initialized_ = false;
  bool receiving_ = false;
};

/**
 * @brief Get default BC Gun protocol configuration.
 *
 * Returns a copy of kBcConfig. Prefer using the constexpr
 * kBcConfig directly when a reference suffices.
 *
 * @return Protocol configuration for BC Gun.
 * @see kBcConfig
 */
codec::ProtocolConfig make_bc_config();

}  // namespace protocols
}  // namespace ir_lasertag
