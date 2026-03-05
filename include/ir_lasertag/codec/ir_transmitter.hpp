/**
 * @file ir_transmitter.hpp
 * @brief IR transmitter using ESP32 RMT peripheral.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "ir_lasertag/codec/ir_protocol.hpp"

namespace ir_lasertag {
namespace codec {

/**
 * @brief IR transmitter using RMT peripheral.
 *
 * Encodes and transmits IR signals according to the configured protocol.
 * Protocol can be changed at runtime without reinitializing the channel.
 * Supports both blocking and non-blocking transmission modes.
 *
 * For blocking send, use send() which waits for completion.
 * For non-blocking send, use send_async() and either poll is_busy()
 * or call wait_done() when ready to synchronize.
 */
class IrTransmitter {
 public:
  IrTransmitter();
  ~IrTransmitter();

  // Non-copyable
  IrTransmitter(const IrTransmitter&) = delete;
  IrTransmitter& operator=(const IrTransmitter&) = delete;

  /**
   * @brief Initialize the transmitter.
   *
   * @param config RMT TX channel configuration (esp-idf type).
   * @return ESP_OK on success, error code otherwise.
   */
  esp_err_t init(const rmt_tx_channel_config_t& config);

  /**
   * @brief Deinitialize and release RMT resources.
   */
  void deinit();

  /**
   * @brief Check if transmitter is initialized.
   *
   * @return true if initialized and ready to transmit.
   */
  bool is_initialized() const { return initialized_; }

  /**
   * @brief Check if a transmission is currently in progress.
   *
   * @return true if transmitting.
   */
  bool is_busy() const { return busy_; }

  /**
   * @brief Set the protocol configuration.
   *
   * Creates or updates the encoder for the new protocol.
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
   * @brief Set the carrier duty cycle without full protocol re-setup.
   *
   * Updates both the stored protocol config and the RMT carrier.
   *
   * @param duty Duty cycle (0.0 - 1.0).
   * @return ESP_OK on success, error code otherwise.
   */
  esp_err_t set_carrier_duty(float duty);

  /**
   * @brief Transmit IR data asynchronously.
   *
   * Starts transmission and returns immediately. Use is_busy() to
   * poll for completion or wait_done() to block until complete.
   *
   * @param data Data buffer to transmit.
   * @param bit_count Number of bits to transmit.
   * @param loop_count RMT loop count (0 = single shot, N > 0 = N iterations).
   * @return ESP_OK if transmission started, error code otherwise.
   */
  esp_err_t send_async(const uint8_t* data, size_t bit_count,
                       int loop_count = 0);

  /**
   * @brief Transmit IR data synchronously (blocking).
   *
   * Encodes and transmits the specified data bits. Blocks until
   * transmission is complete or timeout.
   *
   * @param data Data buffer to transmit.
   * @param bit_count Number of bits to transmit.
   * @param timeout_ms Maximum time to wait (default 1000ms).
   * @param loop_count RMT loop count (0 = single shot, N > 0 = N iterations).
   * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout.
   */
  esp_err_t send(const uint8_t* data, size_t bit_count,
                 uint32_t timeout_ms = 1000, int loop_count = 0);

  /**
   * @brief Wait for current transmission to complete.
   *
   * @param timeout_ms Maximum time to wait.
   * @return ESP_OK if completed, ESP_ERR_TIMEOUT on timeout.
   */
  esp_err_t wait_done(uint32_t timeout_ms);

  /**
   * @brief Transmit raw IR timings.
   *
   * Sends raw mark/space timings without protocol encoding.
   *
   * @param timings Array of raw timings.
   * @param count Number of timings.
   * @param timeout_ms Maximum time to wait.
   * @return ESP_OK on success.
   */
  esp_err_t send_raw(const RawTiming* timings, size_t count,
                     uint32_t timeout_ms = 1000);

 private:
  /**
   * @brief RMT transmit done callback (static).
   */
  static bool rmt_tx_done_callback(rmt_channel_handle_t channel,
                                   const rmt_tx_done_event_data_t* edata,
                                   void* user_ctx);

  /**
   * @brief Create or recreate the RMT encoder.
   */
  esp_err_t create_encoder();

  /**
   * @brief Destroy the current encoder.
   */
  void destroy_encoder();

  gpio_num_t gpio_ = GPIO_NUM_NC;
  ProtocolConfig protocol_;
  rmt_channel_handle_t channel_ = nullptr;
  rmt_encoder_handle_t encoder_ = nullptr;
  rmt_encoder_handle_t copy_encoder_ = nullptr;  // For raw mode

  bool initialized_ = false;
  bool protocol_set_ = false;
  volatile bool busy_ = false;
};

}  // namespace codec
}  // namespace ir_lasertag
