/**
 * @file ir_receiver.hpp
 * @brief IR receiver using ESP32 RMT peripheral.
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

namespace ir_lasertag {
namespace codec {

/**
 * @brief IR receiver using RMT peripheral.
 *
 * Receives and decodes IR signals according to the configured protocol.
 * Supports both decoded message mode and raw timing mode for protocol
 * analysis.
 */
class IrReceiver {
 public:
  /// Default symbol buffer size
  static constexpr size_t kDefaultSymbolBufferSize = 128;

  IrReceiver();
  ~IrReceiver();

  // Non-copyable
  IrReceiver(const IrReceiver&) = delete;
  IrReceiver& operator=(const IrReceiver&) = delete;

  /**
   * @brief Initialize the receiver.
   *
   * @param config RMT RX channel configuration (esp-idf type).
   * @param symbol_buffer_size Size of RMT symbol buffer (default 128).
   * @return ESP_OK on success, error code otherwise.
   */
  esp_err_t init(const rmt_rx_channel_config_t& config,
                 size_t symbol_buffer_size = kDefaultSymbolBufferSize);

  /**
   * @brief Deinitialize and release RMT resources.
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
   * Enables the RMT receiver. Received messages are queued.
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

 private:
  /**
   * @brief RMT receive callback (static).
   */
  static bool rmt_rx_done_callback(rmt_channel_handle_t channel,
                                   const rmt_rx_done_event_data_t* edata,
                                   void* user_ctx);

  /**
   * @brief Handle received RMT data.
   * @return true if high-priority task was woken.
   */
  bool on_receive_done(const rmt_rx_done_event_data_t* edata);

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

  /**
   * @brief Restart RMT receive after processing data.
   */
  void restart_receive();

  gpio_num_t gpio_ = GPIO_NUM_NC;
  ProtocolConfig protocol_;
  rmt_channel_handle_t channel_ = nullptr;

  /// ISR→task queue: carries num_symbols (size_t). Task reads symbols
  /// from symbol_buffer_ which is stable until restart_receive().
  QueueHandle_t symbol_queue_ = nullptr;

  // Symbol buffer (allocated during init)
  rmt_symbol_word_t* symbol_buffer_ = nullptr;
  size_t symbol_buffer_size_ = 0;

  /// RMT signal_range_max_ns, computed from protocol config.
  /// Gaps longer than this end a frame. Set so that intra-packet
  /// spaces pass through but inter-packet gaps trigger frame-end.
  uint32_t signal_range_max_ns_ = 12000000;  // default 12ms

  bool initialized_ = false;
  bool protocol_set_ = false;
  bool raw_mode_ = false;
  bool receiving_ = false;
};

}  // namespace codec
}  // namespace ir_lasertag
