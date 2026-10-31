/**
 * @file ir_receiver.hpp
 * @brief IR receiver — decodes symbols from any IrRxSource backend.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "ir_lasertag/codec/ir_protocol.hpp"
#include "ir_lasertag/codec/ir_rx_source.hpp"

namespace ir_lasertag {
namespace codec {

/**
 * @brief IR receiver — decodes symbol frames into IR messages.
 *
 * Receives symbol frames from an IrRxSource backend and decodes
 * them according to the configured protocol. Supports both decoded
 * message mode and raw timing mode for protocol analysis.
 *
 * The receiver does not own the source — the caller creates and
 * initializes the source, then passes it to init().
 */
class IrReceiver {
 public:
  /// Most symbols one frame can hold; receive_symbols() never returns more.
  static constexpr size_t kMaxFrameSymbols = 64;

  IrReceiver();
  ~IrReceiver();

  // Non-copyable
  IrReceiver(const IrReceiver&) = delete;
  IrReceiver& operator=(const IrReceiver&) = delete;

  /**
   * @brief Initialize the receiver with a symbol source.
   *
   * The caller retains ownership of the source. It must remain
   * valid until deinit() is called.
   *
   * @param source Backend that delivers symbol frames.
   * @return ESP_OK on success, error code otherwise.
   */
  esp_err_t init(IrRxSource* source);

  /**
   * @brief Deinitialize and release resources.
   */
  void deinit();

  /**
   * @brief Check if receiver is initialized.
   *
   * @return true if initialized and ready to receive.
   */
  bool is_initialized() const { return initialized_; }

  /**
   * @brief Set the protocol configuration.
   *
   * Computes max_symbol_duration from protocol timings and pushes
   * it to the source backend.
   *
   * @param config Protocol configuration.
   * @return ESP_OK on success, error code otherwise.
   */
  esp_err_t set_protocol(const ProtocolConfig& config);

  /**
   * @brief Get current protocol configuration.
   *
   * @return Current configuration.
   */
  const ProtocolConfig& get_protocol() const { return protocol_; }

  /**
   * @brief Enable raw timing mode.
   *
   * In raw mode, receive_raw() can be used to get raw timings
   * for protocol analysis. Decoded receive() is disabled.
   *
   * @return ESP_OK on success.
   */
  esp_err_t enable_raw_mode();

  /**
   * @brief Disable raw timing mode and return to decoded mode.
   *
   * @return ESP_OK on success.
   */
  esp_err_t disable_raw_mode();

  /**
   * @brief Check if raw mode is enabled.
   *
   * @return true if raw mode is active.
   */
  bool is_raw_mode() const { return raw_mode_; }

  /**
   * @brief Start receiving IR signals.
   *
   * Delegates to the source backend to begin symbol delivery.
   *
   * @return ESP_OK on success.
   */
  esp_err_t start();

  /**
   * @brief Stop receiving IR signals.
   *
   * @return ESP_OK on success.
   */
  esp_err_t stop();

  /**
   * @brief Check if receiver is actively receiving.
   *
   * @return true if receiving is enabled.
   */
  bool is_receiving() const { return receiving_; }

  /**
   * @brief Try to receive a decoded IR message without blocking.
   *
   * Convenience wrapper for receive() with timeout=0.
   *
   * @param message Output structure for received message.
   * @return ESP_OK if message available, ESP_ERR_TIMEOUT if none queued.
   */
  esp_err_t try_receive(IrMessage* message) { return receive(message, 0); }

  /**
   * @brief Receive a decoded IR message.
   *
   * Waits until a message is received or timeout. Only valid when
   * raw mode is disabled and no callback is set.
   *
   * @param message Output structure for received message.
   * @param timeout_ms Maximum time to wait (0 = no wait/poll).
   * @return ESP_OK if message received, ESP_ERR_TIMEOUT on timeout,
   *         ESP_ERR_INVALID_STATE if in raw mode.
   */
  esp_err_t receive(IrMessage* message, uint32_t timeout_ms);

  /**
   * @brief Receive raw IR timings.
   *
   * Blocks until timings are received or timeout. Only valid when
   * raw mode is enabled.
   *
   * @param timings Output buffer for timings.
   * @param max_count Maximum number of timings to receive.
   * @param received_count Output: actual number of timings received.
   * @param timeout_ms Maximum time to wait (0 = no wait).
   * @return ESP_OK if data received, ESP_ERR_TIMEOUT on timeout,
   *         ESP_ERR_INVALID_STATE if not in raw mode.
   */
  esp_err_t receive_raw(RawTiming* timings, size_t max_count,
                        size_t* received_count, uint32_t timeout_ms);

  /**
   * @brief Receive one frame's symbols as the source delivered them.
   *
   * Blocks until a frame arrives or timeout. Works in either mode, for a
   * protocol that decodes symbols itself (for example ArcliteDecoder).
   *
   * @param[out] symbols Output buffer.
   * @param max_count Its size in symbols; a longer frame is truncated.
   * @param[out] count Number of symbols copied.
   * @param[out] last_edge_us `esp_timer` time of the frame's last edge
   *             (nullable).
   * @param timeout_ms Maximum time to wait (0 = no wait).
   * @return ESP_OK, ESP_ERR_TIMEOUT, ESP_ERR_INVALID_ARG, or
   *         ESP_ERR_INVALID_STATE if not initialized.
   */
  /**
   * @brief Override the end-of-frame threshold set_protocol() computed.
   *
   * For a protocol whose rule isn't 1.5 x the longest pulse (Arclite's
   * threshold is fixed by its spec). Call after set_protocol().
   *
   * @param duration_us The threshold, passed to the source's
   *        set_max_symbol_duration_us().
   * @return ESP_OK, ESP_ERR_INVALID_STATE if not initialized, or the
   *         source's error.
   */
  esp_err_t set_end_of_frame_us(uint16_t duration_us);

  esp_err_t receive_symbols(rmt_symbol_word_t* symbols, size_t max_count,
                            size_t* count, int64_t* last_edge_us,
                            uint32_t timeout_ms);

  /**
   * @brief Compute max_symbol_duration_us from protocol timings.
   *
   * Returns 1.5x the longest individual pulse (mark or space) in the
   * protocol definition. Covers header, data half-bits, and footer.
   *
   * @param config Protocol configuration.
   * @return Computed max symbol duration in microseconds.
   */
  static uint16_t compute_max_symbol_duration_us(const ProtocolConfig& config);

 private:
  /**
   * @brief Callback invoked by IrRxSource when a frame arrives.
   *
   * Copies symbols into the internal buffer and queues the count
   * for task-context processing. May be called from ISR context
   * depending on the backend.
   */
  static void on_source_frame(const rmt_symbol_word_t* symbols,
                              size_t count, int64_t last_edge_us, void* ctx);

  /**
   * @brief Decode received symbols into message.
   */
  bool decode_symbols(const rmt_symbol_word_t* symbols, size_t count,
                      IrMessage* message);

  /**
   * @brief Decode Manchester-encoded symbols into message.
   *
   * Uses a half-symbol consumption approach: each RMT symbol is split
   * into two half-symbols (duration0/level0, duration1/level1). Durations
   * are consumed in half-bit-sized chunks. When adjacent same-level
   * half-bits merge (producing double-length durations), only the needed
   * portion is consumed and the remainder carries forward.
   */
  bool decode_manchester(const rmt_symbol_word_t* symbols, size_t count,
                         IrMessage* message);

  /**
   * @brief Check if timing matches expected value within tolerance.
   * @param actual Measured duration in microseconds.
   * @param expected Expected duration in microseconds.
   * @param is_mark True if this is a mark (carrier on), false for space.
   *
   * Applies mark_bias_us compensation before comparing.
   */
  bool timing_matches(uint16_t actual, uint16_t expected, bool is_mark) const;

  IrRxSource* source_ = nullptr;
  ProtocolConfig protocol_;

  /// Source→task queue: carries num_symbols (size_t).
  QueueHandle_t symbol_queue_ = nullptr;

  // Symbol buffer — frames are copied here by on_source_frame()
  static constexpr size_t kSymbolBufferSize = kMaxFrameSymbols;
  rmt_symbol_word_t symbol_buffer_[kSymbolBufferSize] = {};
  int64_t last_edge_us_ = 0;  ///< Last edge of the frame in symbol_buffer_

  bool initialized_ = false;
  bool protocol_set_ = false;
  bool raw_mode_ = false;
  bool receiving_ = false;
};

}  // namespace codec
}  // namespace ir_lasertag
