/**
 * @file irt.hpp
 * @brief IRT Gun protocol wrapper.
 *
 * Provides send/receive operations for IRT Gun 16-bit ID packets.
 * IRT guns transmit a 16-bit ID in format 0xXXYY where XX is the
 * high byte (unknown purpose) and YY is the CCM ID. Guns not assigned
 * a CCM ID send 0xF0F0.
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
// IRT Gun Protocol Constants
// ============================================================================

/// @name Timing Constants
/// @{

/// Carrier frequency in Hz
constexpr uint32_t kIrtCarrierHz = 38000;

/// Header mark duration in microseconds
constexpr uint16_t kIrtHeaderMarkUs = 500;

/// Header space duration in microseconds
constexpr uint16_t kIrtHeaderSpaceUs = 500;

/// Zero bit mark duration in microseconds
constexpr uint16_t kIrtZeroMarkUs = 350;

/// Zero bit space duration in microseconds
constexpr uint16_t kIrtZeroSpaceUs = 450;

/// One bit mark duration in microseconds
constexpr uint16_t kIrtOneMarkUs = 700;

/// One bit space duration in microseconds
constexpr uint16_t kIrtOneSpaceUs = 450;

/// Gap between repeated transmissions in microseconds
constexpr uint32_t kIrtMessageGapUs = 2000;

/// Message bit count
constexpr uint8_t kIrtBitCount = 16;

/// @}

/**
 * @brief Default IRT Gun protocol configuration.
 *
 * Pulse-width encoding:
 * - 500µs header mark, 500µs header space
 * - 0 bit: 350µs mark, 450µs space
 * - 1 bit: 700µs mark, 450µs space
 * - 16-bit messages, MSB first
 * - 38kHz carrier
 *
 * Note: IRT guns repeat each message twice with a 2000µs gap.
 * The gap is encoded as footer_space_us; the protocol wrapper sends
 * with loop_count=2 so the RMT peripheral handles both iterations
 * atomically.
 */
constexpr codec::ProtocolConfig kIrtConfig = {
    .carrier_freq_hz = kIrtCarrierHz,
    .carrier_duty = 0.33f,
    .header_mark_us = kIrtHeaderMarkUs,
    .header_space_us = kIrtHeaderSpaceUs,
    .zero_half1 = {kIrtZeroMarkUs, codec::Level::kMark},
    .zero_half2 = {kIrtZeroSpaceUs, codec::Level::kSpace},
    .one_half1 = {kIrtOneMarkUs, codec::Level::kMark},
    .one_half2 = {kIrtOneSpaceUs, codec::Level::kSpace},
    .footer_mark_us = 0,
    .footer_space_us = kIrtMessageGapUs,
    .bit_order = codec::BitOrder::kMsbFirst,
    .bit_count = kIrtBitCount,
    .tolerance_percent = 25,
    .mark_bias_us = 0,
};

// ============================================================================
// IrtGun Class
// ============================================================================

/**
 * @brief IRT Gun protocol wrapper.
 *
 * Wraps the IR codec layer to provide an IRT Gun-level API.
 * IRT guns send a 16-bit ID twice with a 2ms gap between
 * transmissions. The receiver validates paired packets.
 *
 * This wrapper runs in task context and uses blocking/non-blocking
 * queue-based receive operations from the codec layer.
 *
 * Usage:
 * @code
 *   IrtGun gun;
 *   gun.init(tx_channel, rx_channel, irt_config);
 *
 *   // Send a gun ID (transmits twice with 2ms gap)
 *   gun.send(0xC804);
 *
 *   // Poll for received gun IDs (non-blocking)
 *   uint16_t gun_id;
 *   if (gun.try_receive(&gun_id)) {
 *     ESP_LOGI("rx", "Got gun ID: 0x%04X (CCM: %d)",
 *              gun_id, gun_id & 0xFF);
 *   }
 *
 *   // Or block until one is received
 *   if (gun.receive(&gun_id, 1000) == ESP_OK) {
 *     ESP_LOGI("rx", "Got gun ID: 0x%04X", gun_id);
 *   }
 * @endcode
 */
class IrtGun {
 public:
  /// Gap between repeated transmissions in microseconds.
  static constexpr uint32_t kRepeatGapUs = 2000;

  /// Unassigned gun ID (no CCM assignment).
  static constexpr uint16_t kUnassignedId = 0xF0F0;

  /// Maximum time to wait for second packet during receive validation (ms).
  static constexpr uint32_t kPairTimeoutMs = 10;

  IrtGun();
  ~IrtGun();

  // Non-copyable
  IrtGun(const IrtGun&) = delete;
  IrtGun& operator=(const IrtGun&) = delete;

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
   * @param config IRT Gun protocol configuration.
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
   * Sends the gun ID as two 16-bit packets with a 2ms gap between
   * them as per IRT protocol. Blocks until both transmissions
   * are complete.
   *
   * @param gun_id The 16-bit gun ID to transmit (0xXXYY format).
   * @param timeout_ms Maximum time to wait for each transmission (default 1000ms).
   * @return ESP_OK on success, ESP_ERR_INVALID_STATE if no transmitter,
   *         ESP_ERR_TIMEOUT on timeout, or codec error.
   */
  esp_err_t send(uint16_t gun_id, uint32_t timeout_ms = 1000);

  /**
   * @brief Start the receiver for polling gun IDs.
   *
   * After calling this, use try_receive() or receive() to poll
   * for received gun IDs. Paired packet validation is performed
   * automatically.
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
   * Polls the receive queue and performs paired packet validation.
   * Returns immediately whether or not a gun ID is available.
   *
   * @param[out] gun_id Receives the gun ID if a valid pair is available.
   * @return true if a validated gun ID was received, false otherwise.
   */
  bool try_receive(uint16_t* gun_id);

  /**
   * @brief Receive a gun ID with timeout.
   *
   * Blocks until a validated paired gun ID is received or timeout expires.
   *
   * @param[out] gun_id Receives the gun ID.
   * @param timeout_ms Maximum time to wait in milliseconds.
   * @return ESP_OK if received, ESP_ERR_TIMEOUT on timeout,
   *         ESP_ERR_INVALID_STATE if not receiving.
   */
  esp_err_t receive(uint16_t* gun_id, uint32_t timeout_ms);

  /**
   * @brief Extract CCM ID from gun ID.
   *
   * @param gun_id Full 16-bit gun ID.
   * @return The 8-bit CCM ID (low byte).
   */
  static uint8_t get_ccm_id(uint16_t gun_id) {
    return static_cast<uint8_t>(gun_id & 0xFF);
  }

  /**
   * @brief Check if gun ID indicates unassigned CCM.
   *
   * @param gun_id Full 16-bit gun ID.
   * @return true if gun is unassigned (0xF0F0).
   */
  static bool is_unassigned(uint16_t gun_id) {
    return gun_id == kUnassignedId;
  }

 private:
  /**
   * @brief Decode an IR message to a gun ID.
   *
   * @param message The received IR message.
   * @return The gun ID if valid, std::nullopt otherwise.
   */
  static std::optional<uint16_t> decode_message(const codec::IrMessage& message);

  /**
   * @brief Process a raw packet and validate pairing.
   *
   * @param gun_id The decoded gun ID from the packet.
   * @return true if this completes a valid pair, false otherwise.
   */
  bool validate_pair(uint16_t gun_id);

  codec::IrTransmitter* transmitter_ = nullptr;
  codec::IrReceiver* receiver_ = nullptr;
  uint8_t send_data_[2] = {};     ///< Persistent buffer for transmit
  uint16_t last_gun_id_ = 0;      ///< Last received gun ID for pair validation
  uint32_t last_receive_time_ = 0;  ///< Timestamp of last receive (ms)
  bool initialized_ = false;
  bool receiving_ = false;
};

/**
 * @brief Get default IRT Gun protocol configuration.
 *
 * Returns a copy of kIrtConfig. Prefer using the constexpr
 * kIrtConfig directly when a reference suffices.
 *
 * @return Protocol configuration for IRT Gun.
 * @see kIrtConfig
 */
codec::ProtocolConfig make_irt_config();

}  // namespace protocols
}  // namespace ir_lasertag
