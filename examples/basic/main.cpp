/**
 * @file main.cpp
 * @brief ir_lasertag example: Arclite Tag Code send/receive.
 *
 * Demonstrates initializing the IR transmitter and receiver, then
 * periodically sending a tag code while polling for received ones.
 */

#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ir_lasertag/codec/ir_transmitter.hpp"
#include "ir_lasertag/codec/ir_receiver.hpp"
#include "ir_lasertag/codec/rmt_rx_source.hpp"
#include "ir_lasertag/protocols/arclite/arclite.hpp"

static const char* TAG = "ir_example";

/// GPIO for IR LED (transmit)
static constexpr gpio_num_t IR_TX_GPIO = GPIO_NUM_4;

/// GPIO for IR receiver module
static constexpr gpio_num_t IR_RX_GPIO = GPIO_NUM_5;

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "ir_lasertag Arclite example");

  // --- Initialize transmitter ---
  ir_lasertag::codec::IrTransmitter tx;
  rmt_tx_channel_config_t tx_config = {
      .gpio_num = IR_TX_GPIO,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 1000000,
      .mem_block_symbols = 64,
      .trans_queue_depth = 4,
  };

  esp_err_t err = tx.init(tx_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "TX init failed: %s", esp_err_to_name(err));
    return;
  }

  // --- Initialize RMT RX source ---
  ir_lasertag::codec::RmtRxSource rx_source;
  rmt_rx_channel_config_t rx_config = {
      .gpio_num = IR_RX_GPIO,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 1000000,
      .mem_block_symbols = 128,
  };

  err = rx_source.init(rx_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RX source init failed: %s", esp_err_to_name(err));
    return;
  }

  // --- Initialize receiver with the RMT source ---
  ir_lasertag::codec::IrReceiver rx;
  err = rx.init(&rx_source);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RX init failed: %s", esp_err_to_name(err));
    return;
  }

  // --- Set up Arclite Tag Code protocol ---
  ir_lasertag::protocols::ArcliteTagCode atc;
  err = atc.init(&tx, &rx, ir_lasertag::protocols::kArcliteConfig);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ArcliteTagCode init failed: %s", esp_err_to_name(err));
    return;
  }

  // Start receiving
  atc.start_receive();

  // --- Main loop ---
  uint16_t tag_code_to_send = 0x1234;

  while (true) {
    // Send a tag code
    ESP_LOGI(TAG, "Sending tag code: 0x%04X", tag_code_to_send);
    err = atc.send(tag_code_to_send);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Send failed: %s", esp_err_to_name(err));
    }

    // Poll for received tag codes
    uint16_t received;
    if (atc.try_receive(&received)) {
      ESP_LOGI(TAG, "Received tag code: 0x%04X", received);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
