/**
 * @file ir_rx_source.hpp
 * @brief Abstract interface for IR symbol sources.
 *
 * IrRxSource decouples IrReceiver from the hardware backend.
 * Implementations translate their backend's lifecycle and 
 * configuration to this common interface.
 *
 * A reference RMT implementation (RmtRxSource) is provided with
 * this library. Other backends can be implemented externally.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "driver/rmt_rx.h"

namespace ir_lasertag {
namespace codec {

/**
 * @brief Callback invoked when a complete frame of symbols is received.
 *
 * @param symbols Array of RMT symbols composing the frame.
 * @param count Number of symbols in the array.
 * @param ctx Opaque context pointer registered with the source.
 */
using IrRxFrameCallback = void (*)(const rmt_symbol_word_t* symbols,
                                   size_t count, void* ctx);

/**
 * @brief Abstract interface for IR receive symbol sources.
 *
 * Implementations wrap a specific backend (RMT peripheral,
 * software-ir-receiver, etc.) and deliver decoded symbol frames
 * to IrReceiver through a registered callback.
 */
class IrRxSource {
 public:
  virtual ~IrRxSource() = default;

  /**
   * @brief Register the frame-complete callback.
   *
   * Called by IrReceiver during initialization. The implementation
   * must invoke this callback on every complete frame.
   *
   * @param callback Function to call when a frame is received.
   * @param ctx Opaque context forwarded to the callback.
   * @return ESP_OK on success.
   */
  virtual esp_err_t set_on_frame_callback(IrRxFrameCallback callback,
                                          void* ctx) = 0;

  /**
   * @brief Start receiving symbols.
   *
   * After this call, the backend should begin detecting IR edges
   * and delivering frames via the registered callback.
   *
   * @return ESP_OK on success.
   */
  virtual esp_err_t start() = 0;

  /**
   * @brief Stop receiving symbols.
   *
   * @return ESP_OK on success.
   */
  virtual esp_err_t stop() = 0;

  /**
   * @brief Set the maximum symbol duration (end-of-frame threshold).
   *
   * Any single mark or space lasting longer than this value indicates
   * that the frame has ended. This is distinct from signal gap (the
   * minimum time between consecutive transmissions).
   *
   * The value should be set to slightly above the longest expected
   * mark or space in the protocol. For example, if the longest pulse
   * is 1200µs, a value of ~1800µs is appropriate.
   *
   * Implementations translate this to their backend's native units
   * and semantics (e.g., RMT uses nanoseconds per-receive).
   *
   * @param duration_us Maximum symbol duration in microseconds.
   * @return ESP_OK on success.
   */
  virtual esp_err_t set_max_symbol_duration_us(uint16_t duration_us) = 0;
};

}  // namespace codec
}  // namespace ir_lasertag
