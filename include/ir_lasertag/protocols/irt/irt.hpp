/**
 * @file irt.hpp
 * @brief IRT protocol wrapper (gun + spawner).
 *
 * Provides send/receive operations for IRT 16-bit messages. IRT devices
 * come in two roles, distinguished by transmission pattern:
 *  - Guns transmit the 16-bit ID twice with a ~2ms gap (format 0xXXYY,
 *    where YY is the CCM ID; unassigned guns send 0xF0F0).
 *  - Spawners transmit a static 16-bit ID once (no repeat), repeated at
 *    their own cadence (~50ms).
 *
 * The receiver classifies incoming packets into gun / spawner / invalid
 * by arrival pattern and returns a structured IrtMessage{ id, role }.
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
// IRT Protocol Constants
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

/// Gap between a gun's repeated transmissions in microseconds
constexpr uint32_t kIrtMessageGapUs = 2000;

/// Message bit count
constexpr uint8_t kIrtBitCount = 16;

/// @}

/**
 * @brief Default IRT protocol configuration.
 *
 * Pulse-width encoding:
 * - 500µs header mark, 500µs header space
 * - 0 bit: 350µs mark, 450µs space
 * - 1 bit: 700µs mark, 450µs space
 * - 16-bit messages, MSB first
 * - 38kHz carrier
 *
 * The encoding is identical for guns and spawners; only the repeat
 * pattern differs (see Irt::send_gun / Irt::send_spawner). The 2000µs
 * footer space supplies the gun inter-packet gap when sending a pair.
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
// Receive Result Types
// ============================================================================

/**
 * @brief Role of a received IRT message, inferred from arrival pattern.
 */
enum class IrtRole : uint8_t {
  kGun,      ///< Two identical packets within the pair window.
  kSpawner,  ///< A single packet with no matching pair within the window.
};

/**
 * @brief A classified IRT receive result: the 16-bit ID and its role.
 */
struct IrtMessage {
  uint16_t id;    ///< 16-bit message ID.
  IrtRole role;   ///< Detected role (gun or spawner).
};

/**
 * @brief Outcome of classifying an incoming packet against a held candidate.
 */
enum class IrtPairResult : uint8_t {
  kHold,     ///< No candidate held; store the incoming packet as candidate.
  kGun,      ///< Matching pair within window; emit gun, clear candidate.
  kSpawner,  ///< Incoming arrived beyond window; emit candidate as spawner,
             ///< then hold the incoming packet as the new candidate.
  kReplace,  ///< Mismatch within window; discard the old candidate and hold the
             ///< incoming packet as the new candidate (sliding; no output).
};

// ============================================================================
// Irt Class
// ============================================================================

/**
 * @brief IRT protocol wrapper for gun and spawner messages.
 *
 * Wraps the IR codec layer with IRT semantics. Sends gun shots (paired
 * 16-bit packets, 2ms gap) or spawner pulses (single packets), and
 * classifies received packets into gun / spawner / invalid by arrival
 * pattern, returning an IrtMessage{ id, role }.
 *
 * This wrapper runs in task context and uses blocking/non-blocking
 * queue-based receive operations from the codec layer.
 *
 * Usage:
 * @code
 *   Irt irt;
 *   irt.init(tx_channel, rx_channel, irt_config);
 *
 *   irt.send_gun(0xC804);      // gun shot (transmitted twice, 2ms gap)
 *   irt.send_spawner(0x37FB);  // single spawner packet
 *
 *   irt.start_receive();
 *   IrtMessage msg;
 *   if (irt.try_receive(&msg)) {
 *     ESP_LOGI("rx", "id=0x%04X role=%s", msg.id,
 *              msg.role == IrtRole::kGun ? "gun" : "spawner");
 *   }
 * @endcode
 */
class Irt {
 public:
  /// Gap between a gun's repeated transmissions in microseconds.
  static constexpr uint32_t kRepeatGapUs = 2000;

  /// Unassigned gun ID (no CCM assignment).
  static constexpr uint16_t kUnassignedId = 0xF0F0;

  /// Maximum receive-side inter-arrival delta for two packets to count as a
  /// gun pair (µs). A gun's two packets arrive ~18ms apart (packet duration
  /// plus the 2ms gap); spawner repeats arrive ~50ms apart. 30ms sits between
  /// the two so gun pairs are matched while spawner repeats age out as singles.
  static constexpr int64_t kPairWindowUs = 30000;

  Irt();
  ~Irt();

  // Non-copyable
  Irt(const Irt&) = delete;
  Irt& operator=(const Irt&) = delete;

  /**
   * @brief Initialize with pre-configured transmitter and receiver.
   *
   * The caller retains ownership of the transmitter and receiver. Both
   * must remain valid for the lifetime of this object (or until deinit()).
   * Either may be nullptr if only send or receive is needed. The protocol
   * configuration is applied to both.
   *
   * @param transmitter Pointer to an initialized IrTransmitter (nullable).
   * @param receiver Pointer to an initialized IrReceiver (nullable).
   * @param config IRT protocol configuration.
   * @return ESP_OK on success, ESP_ERR_INVALID_ARG if both are null,
   *         or error from set_protocol().
   */
  esp_err_t init(codec::IrTransmitter* transmitter, codec::IrReceiver* receiver,
                 const codec::ProtocolConfig& config);

  /**
   * @brief Release references to transmitter/receiver.
   *
   * Stops any active receive operation. Does not deinitialize the
   * underlying transmitter/receiver — the caller manages their lifetime.
   */
  void deinit();

  /**
   * @brief Check if the protocol wrapper is initialized.
   * @return true if init() has been called successfully.
   */
  bool is_initialized() const { return initialized_; }

  /**
   * @brief Transmit a gun shot (blocking).
   *
   * Sends the 16-bit ID twice with a 2ms gap, as required for IRT gun
   * shots. Blocks until both transmissions complete.
   *
   * @param id The 16-bit gun ID to transmit (0xXXYY format).
   * @param timeout_ms Maximum time to wait for transmission (default 1000ms).
   * @return ESP_OK on success, ESP_ERR_INVALID_STATE if no transmitter,
   *         ESP_ERR_TIMEOUT on timeout, or codec error.
   */
  esp_err_t send_gun(uint16_t id, uint32_t timeout_ms = 1000);

  /**
   * @brief Transmit a single spawner packet (blocking).
   *
   * Sends the 16-bit ID once (no repeat). Spawners emit continuously at
   * ~50ms intervals; the caller controls that cadence.
   *
   * @param id The 16-bit spawner ID to transmit.
   * @param timeout_ms Maximum time to wait for transmission (default 1000ms).
   * @return ESP_OK on success, ESP_ERR_INVALID_STATE if no transmitter,
   *         ESP_ERR_TIMEOUT on timeout, or codec error.
   */
  esp_err_t send_spawner(uint16_t id, uint32_t timeout_ms = 1000);

  /**
   * @brief Start the receiver for polling messages.
   *
   * After calling this, use try_receive() or receive() to poll for
   * classified gun/spawner messages.
   *
   * @return ESP_OK on success, ESP_ERR_INVALID_STATE if no receiver.
   */
  esp_err_t start_receive();

  /**
   * @brief Stop receiving.
   * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not receiving.
   */
  esp_err_t stop_receive();

  /**
   * @brief Check if receiving is active.
   * @return true if start_receive() has been called and not yet stopped.
   */
  bool is_receiving() const { return receiving_; }

  /**
   * @brief Try to receive a classified message without blocking.
   *
   * Drains the receive queue and runs role detection. Returns immediately.
   * A gun is reported once its matching pair is seen; a spawner is reported
   * once the pair window elapses with no matching pair. Invalid pairs are
   * dropped.
   *
   * @param[out] msg Receives the classified message (id + role) if available.
   * @return true if a classified message was produced, false otherwise.
   */
  bool try_receive(IrtMessage* msg);

  /**
   * @brief Receive a classified message with timeout.
   *
   * Polls at a short interval (so a lone spawner can be flushed after the
   * pair window) until a message is classified or the timeout expires.
   *
   * @param[out] msg Receives the classified message.
   * @param timeout_ms Maximum time to wait in milliseconds.
   * @return ESP_OK if received, ESP_ERR_TIMEOUT on timeout,
   *         ESP_ERR_INVALID_STATE if not receiving.
   */
  esp_err_t receive(IrtMessage* msg, uint32_t timeout_ms);

  /**
   * @brief Classify an incoming packet against a held candidate (pure).
   *
   * Encapsulates the gun/spawner/invalid decision so it can be unit-tested
   * without hardware. Caller supplies the candidate state and the measured
   * inter-arrival delta.
   *
   * @param has_candidate Whether a candidate packet is currently held.
   * @param candidate_id The held candidate's ID (ignored if !has_candidate).
   * @param incoming_id The newly decoded packet's ID.
   * @param delta_us Microseconds since the candidate arrived.
   * @param pair_window_us Max delta for two packets to form a gun pair.
   * @return The classification outcome (see IrtPairResult).
   */
  static IrtPairResult classify(bool has_candidate, uint16_t candidate_id,
                                uint16_t incoming_id, int64_t delta_us,
                                int64_t pair_window_us) {
    if (!has_candidate) {
      return IrtPairResult::kHold;
    }
    if (delta_us > pair_window_us) {
      return IrtPairResult::kSpawner;
    }
    return (incoming_id == candidate_id) ? IrtPairResult::kGun
                                         : IrtPairResult::kReplace;
  }

  /**
   * @brief Extract CCM ID from a gun ID.
   * @param gun_id Full 16-bit gun ID.
   * @return The 8-bit CCM ID (low byte).
   */
  static uint8_t get_ccm_id(uint16_t gun_id) {
    return static_cast<uint8_t>(gun_id & 0xFF);
  }

  /**
   * @brief Check if a gun ID indicates unassigned CCM.
   * @param gun_id Full 16-bit gun ID.
   * @return true if gun is unassigned (0xF0F0).
   */
  static bool is_unassigned(uint16_t gun_id) {
    return gun_id == kUnassignedId;
  }

 private:
  /**
   * @brief Decode an IR message to a 16-bit ID.
   * @param message The received IR message.
   * @return The ID if valid, std::nullopt otherwise.
   */
  static std::optional<uint16_t> decode_message(const codec::IrMessage& message);

  /**
   * @brief Feed one decoded packet through the role-detection state machine.
   *
   * Updates the held-candidate state and, when a gun or spawner is resolved,
   * writes it to @p out.
   *
   * @param id Decoded 16-bit packet ID.
   * @param now_us Arrival timestamp in microseconds (esp_timer).
   * @param[out] out Receives the classified message when one is produced.
   * @return true if a classified message was produced, false otherwise.
   */
  bool process_packet(uint16_t id, int64_t now_us, IrtMessage* out);

  codec::IrTransmitter* transmitter_ = nullptr;
  codec::IrReceiver* receiver_ = nullptr;
  uint8_t send_data_[2] = {};         ///< Persistent buffer for transmit
  uint16_t candidate_id_ = 0;         ///< Held candidate ID (pair detection)
  int64_t candidate_time_us_ = 0;     ///< Candidate arrival timestamp (µs)
  bool has_candidate_ = false;        ///< Whether a candidate is held
  bool initialized_ = false;
  bool receiving_ = false;
};

/**
 * @brief Get default IRT protocol configuration.
 *
 * Returns a copy of kIrtConfig. Prefer using the constexpr kIrtConfig
 * directly when a reference suffices.
 *
 * @return Protocol configuration for IRT.
 * @see kIrtConfig
 */
codec::ProtocolConfig make_irt_config();

}  // namespace protocols
}  // namespace ir_lasertag
