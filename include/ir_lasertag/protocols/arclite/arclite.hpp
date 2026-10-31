/**
 * @file arclite.hpp
 * @brief Arclite Tag Code protocol wrapper.
 *
 * Sends and receives Arclite collision codes over Manchester-encoded IR, as
 * spec/features/protocols/arclite.md defines them: a 14-bit code with two
 * interleaved parity bits. The frame codec is in arclite_frame.hpp.
 *
 * This wrapper is application-layer code that runs in task context,
 * using the codec layer's queue-based receive API.
 */

#pragma once

#include <cstdint>

#include "esp_err.h"
#include "ir_lasertag/codec/ir_protocol.hpp"
#include "ir_lasertag/codec/ir_receiver.hpp"
#include "ir_lasertag/codec/ir_transmitter.hpp"
#include "ir_lasertag/protocols/arclite/arclite_frame.hpp"

namespace ir_lasertag {
namespace protocols {

/// Carrier frequency in Hz (38 kHz ±1%).
constexpr uint32_t kArcliteCarrierHz = 38000;

/**
 * @brief Default Arclite protocol configuration (arclite.md, "IR Encoding").
 *
 * - 520 µs header mark, 520 µs header space
 * - 1 bit: MARK(300 µs) → SPACE(300 µs); 0 bit: SPACE(300 µs) → MARK(300 µs)
 * - 16-bit data word, MSB first: the 14-bit code, then p1, p0
 * - 38 kHz carrier, 33% duty
 * - a 2000 µs space after the last bit (the footer), so frames from one
 *   transmitter are always at least 2 ms apart
 *
 * Receive decoding uses ArcliteDecoder (the receive windows), not the codec's
 * tolerance-based decoder, so `tolerance_percent` is unused for Arclite.
 */
constexpr codec::ProtocolConfig kArcliteConfig = {
    .carrier_freq_hz = kArcliteCarrierHz,
    .carrier_duty = 0.33f,
    .header_mark_us = kArcliteHeaderMarkUs,
    .header_space_us = kArcliteHeaderSpaceUs,
    .zero_half1 = {kArcliteHalfBitUs, codec::Level::kSpace},
    .zero_half2 = {kArcliteHalfBitUs, codec::Level::kMark},
    .one_half1 = {kArcliteHalfBitUs, codec::Level::kMark},
    .one_half2 = {kArcliteHalfBitUs, codec::Level::kSpace},
    .footer_mark_us = 0,
    .footer_space_us = static_cast<uint16_t>(kArcliteMessageGapUs),
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
 * Tag Code is a 14-bit identifier issued to an emitter capability
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
   * @brief Transmit a 14-bit tag code (blocking).
   *
   * Sends one frame: the code's data word (code, p1, p0), Manchester
   * encoded. Blocks until the frame and its 2 ms footer are sent.
   *
   * Which codes a game may use is the caller's concern (the Arclite frame
   * reserves 0x0000 and 0x3FFF); the wrapper sends any code that fits.
   *
   * @param tag_code The tag code, at most kArcliteCodeMax (0x3FFF).
   * @param timeout_ms Maximum time to wait for transmission (default 1000ms).
   * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the code does not fit
   *         in 14 bits, ESP_ERR_INVALID_STATE if no transmitter,
   *         ESP_ERR_TIMEOUT on timeout, or codec error.
   */
  esp_err_t send(uint16_t tag_code, uint32_t timeout_ms = 1000);

  /**
   * @brief Transmit a 14-bit tag code (non-blocking).
   *
   * Queues one frame and returns at once. The transmitter copies the data,
   * so nothing needs to outlive the call. It accepts as many frames as its
   * queue depth (IrTransmitter::init); beyond that this call is refused.
   *
   * Use is_busy() on the underlying transmitter to check completion.
   *
   * @param tag_code The tag code, at most kArcliteCodeMax (0x3FFF).
   * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the code does not fit
   *         in 14 bits, ESP_ERR_INVALID_STATE if no transmitter or its
   *         queue is full.
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
   * Polls the receive queue and decodes what is there with ArcliteDecoder.
   * Returns immediately whether or not a valid tag code is available.
   *
   * @param[out] tag_code Receives the tag code if one is available.
   * @return true if a tag code was received, false if queue is empty.
   */
  bool try_receive(uint16_t* tag_code);

  /**
   * @brief Receive a tag code with timeout.
   *
   * Blocks until a burst is received or timeout expires, and decodes it
   * with ArcliteDecoder. A burst with no valid frame (noise, a timing or
   * parity failure) returns ESP_ERR_INVALID_RESPONSE.
   *
   * @param[out] tag_code Receives the tag code.
   * @param timeout_ms Maximum time to wait in milliseconds.
   * @return ESP_OK if received, ESP_ERR_TIMEOUT on timeout,
   *         ESP_ERR_INVALID_RESPONSE if the burst held no valid frame,
   *         ESP_ERR_INVALID_STATE if not receiving.
   */
  esp_err_t receive(uint16_t* tag_code, uint32_t timeout_ms);

 private:
  /**
   * @brief Receives one burst and decodes it.
   *
   * @param[out] tag_code The first valid code in the burst.
   * @param timeout_ms Maximum time to wait in milliseconds.
   * @return ESP_OK, ESP_ERR_TIMEOUT, or ESP_ERR_INVALID_RESPONSE.
   */
  esp_err_t receive_burst(uint16_t* tag_code, uint32_t timeout_ms);

  codec::IrTransmitter* transmitter_ = nullptr;
  codec::IrReceiver* receiver_ = nullptr;
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
