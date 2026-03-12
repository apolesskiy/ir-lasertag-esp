/**
 * @file rmt_rx_source.hpp
 * @brief RMT-based IrRxSource implementation.
 *
 * Reference implementation of IrRxSource using the ESP-IDF RMT
 * peripheral. This is the default backend for IrReceiver on
 * ESP32 targets.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "driver/rmt_rx.h"

#include "ir_lasertag/codec/ir_rx_source.hpp"

namespace ir_lasertag {
namespace codec {

/**
 * @brief IrRxSource backed by the ESP32 RMT peripheral.
 *
 * Wraps a single RMT RX channel. The RMT operates in
 * request-response mode: after each frame, `rmt_receive()` must
 * be called again (handled internally).
 *
 * Usage:
 * @code
 *   RmtRxSource source;
 *   rmt_rx_channel_config_t cfg = { .gpio_num = GPIO_NUM_5, ... };
 *   source.init(cfg);
 *
 *   IrReceiver rx;
 *   rx.init(&source);
 * @endcode
 */
class RmtRxSource : public IrRxSource {
 public:
  /// Default symbol buffer size (in rmt_symbol_word_t entries)
  static constexpr size_t kDefaultSymbolBufferSize = 128;

  RmtRxSource();
  ~RmtRxSource() override;

  // Non-copyable
  RmtRxSource(const RmtRxSource&) = delete;
  RmtRxSource& operator=(const RmtRxSource&) = delete;

  /**
   * @brief Initialize the RMT RX channel.
   *
   * @param config RMT RX channel configuration (esp-idf type).
   * @param symbol_buffer_size Number of rmt_symbol_word_t entries
   *        in the receive buffer (default 128).
   * @return ESP_OK on success.
   */
  esp_err_t init(const rmt_rx_channel_config_t& config,
                 size_t symbol_buffer_size = kDefaultSymbolBufferSize);

  /**
   * @brief Deinitialize and release RMT resources.
   */
  void deinit();

  /**
   * @brief Check if the RMT channel is initialized.
   */
  bool is_initialized() const { return initialized_; }

  // --- IrRxSource interface ---
  esp_err_t set_on_frame_callback(IrRxFrameCallback callback,
                                  void* ctx) override;
  esp_err_t start() override;
  esp_err_t stop() override;
  esp_err_t set_max_symbol_duration_us(uint16_t duration_us) override;

 private:
  /**
   * @brief RMT receive-done ISR callback (static).
   */
  static bool rmt_rx_done_cb(rmt_channel_handle_t channel,
                             const rmt_rx_done_event_data_t* edata,
                             void* user_ctx);

  /**
   * @brief Restart RMT capture for the next frame.
   */
  void restart_receive();

  rmt_channel_handle_t channel_ = nullptr;
  rmt_symbol_word_t* symbol_buffer_ = nullptr;
  size_t symbol_buffer_size_ = 0;

  IrRxFrameCallback frame_callback_ = nullptr;
  void* callback_ctx_ = nullptr;

  uint32_t max_symbol_duration_ns_ = 12000000;  // default 12ms

  bool initialized_ = false;
  bool started_ = false;
};

}  // namespace codec
}  // namespace ir_lasertag
