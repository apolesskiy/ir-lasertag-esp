// SPDX-License-Identifier: MIT

/**
 * @file main.cpp
 * @brief Profiling / load-test for ir-lasertag-esp RMT TX + RX.
 *
 * Sends real Arclite protocol-encoded messages via IrTransmitter and
 * receives them on up to 4 IrReceiver channels (via RmtRxSource).
 * Measures CPU impact and TX-to-RX latency.
 *
 * Hardware setup: 2 TX GPIOs wired to 4 RX GPIOs (see README.md).
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_clk_tree.h"
#include "ir_lasertag/codec/ir_receiver.hpp"
#include "ir_lasertag/codec/ir_transmitter.hpp"
#include "ir_lasertag/codec/rmt_rx_source.hpp"
#include "ir_lasertag/protocols/arclite/arclite.hpp"

namespace codec = ir_lasertag::codec;
using codec::IrMessage;
using codec::IrReceiver;
using codec::IrTransmitter;
using codec::RmtRxSource;

static const char* TAG = "profiling";

// ---------------------------------------------------------------------------
// GPIO mapping (see README.md for wiring diagram)
// ---------------------------------------------------------------------------

static constexpr gpio_num_t kTxGpio0 = GPIO_NUM_21;  // Bank A
static constexpr gpio_num_t kTxGpio1 = GPIO_NUM_47;  // Bank B

static constexpr gpio_num_t kRxGpios[] = {
    GPIO_NUM_4,   // ch 0, Bank A → TX0
    GPIO_NUM_5,   // ch 1, Bank A → TX0
    GPIO_NUM_38,  // ch 2, Bank B → TX1
    GPIO_NUM_39,  // ch 3, Bank B → TX1
};

static constexpr int kMaxRxChannels =
    sizeof(kRxGpios) / sizeof(kRxGpios[0]);

// Test parameters
static constexpr int kPhaseDurationMs = 5000;
static constexpr int kNumWorkers = 3;
static constexpr int kWorkerStackSize = 2048;

// RMT configuration
static constexpr uint32_t kRmtResolutionHz = 1000000;  // 1 MHz = 1 µs ticks

// Inter-message gap: protocol-specified gap rounded up to next tick.
// Uses vTaskDelay (not esp_rom_delay_us) to yield CPU during the gap;
// busy-waiting would dominate the CPU impact measurement.
static constexpr TickType_t kTxGapTicks =
    pdMS_TO_TICKS(ir_lasertag::protocols::kArcliteMessageGapUs / 1000 + 1);

// Test matrix
static constexpr int kDutyPcts[] = {100, 10};
static constexpr int kNumDuties = sizeof(kDutyPcts) / sizeof(kDutyPcts[0]);
static constexpr int kChannelCounts[] = {1, 2, 4};
static constexpr int kNumChCounts =
    sizeof(kChannelCounts) / sizeof(kChannelCounts[0]);

// 10% duty burst parameters
static constexpr int kBurstOnMs = 100;

// Results storage
struct ScenarioResult {
  double core0_impact;
  double core1_impact;
  uint32_t total_msgs;
  uint32_t total_tx;
  int64_t latency_min_us;
  int64_t latency_max_us;
  int64_t latency_avg_us;
};

static ScenarioResult results[kNumDuties][kNumChCounts];

// ---------------------------------------------------------------------------
// TX task — sends real Arclite protocol messages
// ---------------------------------------------------------------------------
static IrTransmitter transmitters[2];

// TX timestamp ring buffer: each TX cycle stores its timestamp indexed by
// sequence number.  RX decodes the sequence number from the received message
// and looks up the exact send time — no filtering needed.
static constexpr size_t kTxRingSize = 512;
static int64_t tx_timestamps[kTxRingSize];
static volatile uint16_t tx_seq;

struct TxTaskCtx {
  int duty_pct;
  volatile bool run;
  volatile uint32_t send_count;
};

static TxTaskCtx tx_ctx;

// Send one TX cycle: encode seq as message data, transmit, record completion
// timestamp.  Latency = TX completion to RX receive (excludes message
// duration so it measures only idle-threshold + decode + queue overhead).
static void tx_send_cycle(TxTaskCtx* ctx) {
  const int bit_count = ir_lasertag::protocols::kArcliteBitCount;
  uint16_t seq = tx_seq;
  uint8_t data[] = {static_cast<uint8_t>(seq >> 8),
                    static_cast<uint8_t>(seq & 0xFF)};
  transmitters[0].send_async(data, bit_count);
  transmitters[1].send_async(data, bit_count);
  ctx->send_count++;
  transmitters[0].wait_done(100);
  transmitters[1].wait_done(100);
  // Record timestamp after TX completes — latency excludes the message
  // duration itself and measures only the decode/delivery overhead.
  tx_timestamps[seq % kTxRingSize] = esp_timer_get_time();
  tx_seq = seq + 1;
}

static void tx_task(void* arg) {
  auto* ctx = static_cast<TxTaskCtx*>(arg);

  while (ctx->run) {
    if (ctx->duty_pct < 100) {
      // Burst mode: send for kBurstOnMs, then pause
      int64_t burst_start = esp_timer_get_time();
      int64_t burst_end =
          burst_start + static_cast<int64_t>(kBurstOnMs) * 1000;

      while (ctx->run && esp_timer_get_time() < burst_end) {
        tx_send_cycle(ctx);
        vTaskDelay(kTxGapTicks);
      }

      if (!ctx->run) break;

      // Off period
      uint32_t off_ms = static_cast<uint32_t>(
          kBurstOnMs * (100.0 / ctx->duty_pct - 1.0));
      vTaskDelay(pdMS_TO_TICKS(off_ms));
    } else {
      // Continuous: send as fast as protocol allows
      tx_send_cycle(ctx);
      vTaskDelay(kTxGapTicks);
    }
  }

  vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// RX poll task — counts received messages + measures latency
// ---------------------------------------------------------------------------
struct RxPollCtx {
  IrReceiver* receiver;
  volatile bool run;
  volatile uint32_t msg_count;
  volatile uint32_t timeout_count;
  volatile int64_t latency_sum_us;
  volatile int64_t latency_min_us;
  volatile int64_t latency_max_us;
};

static RxPollCtx rx_poll[kMaxRxChannels];
static TaskHandle_t rx_poll_handles[kMaxRxChannels];

static void rx_poll_task(void* arg) {
  auto* ctx = static_cast<RxPollCtx*>(arg);
  IrMessage msg;

  while (true) {
    // Suspend until run_scenario resumes us — zero CPU overhead
    // while inactive (unlike vTaskDelay polling).
    vTaskSuspend(nullptr);

    while (ctx->run) {
      esp_err_t err = ctx->receiver->receive(&msg, 50);
      if (err == ESP_OK) {
        int64_t now = esp_timer_get_time();
        ctx->msg_count++;
        // Decode sequence number and look up exact TX timestamp
        if (msg.bit_count >= 16) {
          uint16_t seq = (msg.data[0] << 8) | msg.data[1];
          int64_t tx_time = tx_timestamps[seq % kTxRingSize];
          int64_t latency = now - tx_time;
          ctx->latency_sum_us += latency;
          if (latency < ctx->latency_min_us) ctx->latency_min_us = latency;
          if (latency > ctx->latency_max_us) ctx->latency_max_us = latency;
        }
      } else {
        ctx->timeout_count++;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Worker tasks — spin a tight compute loop, counting iterations
// ---------------------------------------------------------------------------
struct WorkerCtx {
  const char* name;
  int core;
  volatile bool run;
  volatile uint32_t iterations;
};

static WorkerCtx workers[kNumWorkers];
static TaskHandle_t worker_handles[kNumWorkers];

static void worker_task(void* arg) {
  auto* ctx = static_cast<WorkerCtx*>(arg);

  while (true) {
    while (!ctx->run) {
      vTaskDelay(1);
    }

    volatile uint32_t acc = 0;
    while (ctx->run) {
      for (int i = 0; i < 1000; i++) {
        acc += (i * 17 + 31);
        acc ^= (acc >> 5);
      }
      ctx->iterations++;
    }
    (void)acc;
  }
}

static void start_workers() {
  for (int i = 0; i < kNumWorkers; i++) {
    workers[i].iterations = 0;
    workers[i].run = true;
  }
}

static void stop_workers() {
  for (int i = 0; i < kNumWorkers; i++) {
    workers[i].run = false;
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}

// ---------------------------------------------------------------------------
// Run a single test scenario (baseline + loaded + recovery)
// ---------------------------------------------------------------------------
static ScenarioResult run_scenario(RmtRxSource* sources,
                                   IrReceiver* receivers,
                                   int num_channels, int duty_pct) {
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "--- Scenario: %d channels, %d%% duty ---", num_channels,
           duty_pct);

  // Phase 1: Baseline
  start_workers();
  vTaskDelay(pdMS_TO_TICKS(kPhaseDurationMs));
  stop_workers();

  uint32_t baseline[kNumWorkers];
  for (int i = 0; i < kNumWorkers; i++) {
    baseline[i] = workers[i].iterations;
  }

  // Phase 2: Loaded — IR receivers ON + TX sending protocol messages
  for (int i = 0; i < num_channels; i++) {
    rx_poll[i].msg_count = 0;
    rx_poll[i].timeout_count = 0;
    rx_poll[i].latency_sum_us = 0;
    rx_poll[i].latency_min_us = std::numeric_limits<int64_t>::max();
    rx_poll[i].latency_max_us = 0;
  }

  for (int i = 0; i < num_channels; i++) {
    receivers[i].start();
  }

  // Start TX task
  tx_ctx.duty_pct = duty_pct;
  tx_ctx.send_count = 0;
  tx_ctx.run = true;
  tx_seq = 0;
  TaskHandle_t tx_handle;
  xTaskCreatePinnedToCore(tx_task, "tx_gen", 4096, &tx_ctx, 3, &tx_handle, 0);

  for (int i = 0; i < num_channels; i++) {
    rx_poll[i].run = true;
    vTaskResume(rx_poll_handles[i]);
  }

  start_workers();
  vTaskDelay(pdMS_TO_TICKS(kPhaseDurationMs));
  stop_workers();

  for (int i = 0; i < num_channels; i++) {
    rx_poll[i].run = false;
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  tx_ctx.run = false;
  vTaskDelay(pdMS_TO_TICKS(200));  // Let TX task exit

  for (int i = 0; i < num_channels; i++) {
    receivers[i].stop();
  }

  uint32_t loaded[kNumWorkers];
  for (int i = 0; i < kNumWorkers; i++) {
    loaded[i] = workers[i].iterations;
  }

  // Gather RX stats
  uint32_t total_msgs = 0, total_timeouts = 0;
  int64_t lat_sum = 0;
  int64_t lat_min = std::numeric_limits<int64_t>::max();
  int64_t lat_max = 0;
  for (int i = 0; i < num_channels; i++) {
    total_msgs += rx_poll[i].msg_count;
    total_timeouts += rx_poll[i].timeout_count;
    lat_sum += rx_poll[i].latency_sum_us;
    if (rx_poll[i].latency_min_us < lat_min)
      lat_min = rx_poll[i].latency_min_us;
    if (rx_poll[i].latency_max_us > lat_max)
      lat_max = rx_poll[i].latency_max_us;
  }
  int64_t lat_avg = total_msgs > 0 ? lat_sum / total_msgs : 0;
  if (total_msgs == 0) {
    lat_min = 0;
  }

  ESP_LOGI(TAG, "  TX: %lu msgs sent",
           static_cast<unsigned long>(tx_ctx.send_count));
  ESP_LOGI(TAG, "  RX: %lu msgs, %lu timeouts across %d channels",
           static_cast<unsigned long>(total_msgs),
           static_cast<unsigned long>(total_timeouts), num_channels);
  if (total_msgs > 0) {
    ESP_LOGI(TAG, "  Latency: min=%lld us, avg=%lld us, max=%lld us",
             (long long)lat_min, (long long)lat_avg, (long long)lat_max);
  }

  // Phase 3: Recovery
  start_workers();
  vTaskDelay(pdMS_TO_TICKS(kPhaseDurationMs));
  stop_workers();

  uint32_t recovery[kNumWorkers];
  for (int i = 0; i < kNumWorkers; i++) {
    recovery[i] = workers[i].iterations;
  }

  // Compute CPU impact
  ESP_LOGI(TAG, "  %-8s %12s %12s %12s %8s", "Worker", "Baseline", "Loaded",
           "Recovery", "Impact");

  double core1_base = 0, core1_load = 0;

  for (int i = 0; i < kNumWorkers; i++) {
    double impact = 0;
    if (baseline[i] > 0) {
      impact = (1.0 - static_cast<double>(loaded[i]) / baseline[i]) * 100.0;
    }
    ESP_LOGI(TAG, "  %-8s %12lu %12lu %12lu %+7.2f%%", workers[i].name,
             static_cast<unsigned long>(baseline[i]),
             static_cast<unsigned long>(loaded[i]),
             static_cast<unsigned long>(recovery[i]), impact);

    if (workers[i].core == 1) {
      core1_base += baseline[i];
      core1_load += loaded[i];
    }
  }

  double core1_impact = 0;
  if (core1_base > 0) {
    core1_impact = (1.0 - core1_load / core1_base) * 100.0;
  }

  double core0_impact = 0;
  if (kNumWorkers > 2 && baseline[2] > 0) {
    core0_impact =
        (1.0 - static_cast<double>(loaded[2]) / baseline[2]) * 100.0;
  }

  ESP_LOGI(TAG, "  Core 1 impact: %+.2f%%  |  Core 0 impact: %+.2f%%",
           core1_impact, core0_impact);

  return {
      .core0_impact = core0_impact,
      .core1_impact = core1_impact,
      .total_msgs = total_msgs,
      .total_tx = tx_ctx.send_count,
      .latency_min_us = lat_min,
      .latency_max_us = lat_max,
      .latency_avg_us = lat_avg,
  };
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
extern "C" void app_main(void) {
  vTaskPrioritySet(nullptr, 3);

  // Suppress verbose library logging
  esp_log_level_set("ir_rx", ESP_LOG_WARN);
  esp_log_level_set("ir_tx", ESP_LOG_WARN);
  esp_log_level_set("ir_encoder", ESP_LOG_WARN);
  esp_log_level_set("rmt_rx_src", ESP_LOG_WARN);

  ESP_LOGI(TAG, "=== ir-lasertag-esp Profiling (RMT TX/RX Load Test) ===");

  // CPU frequency
  uint32_t cpu_freq_hz = 0;
  esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_CPU,
                                ESP_CLK_TREE_SRC_FREQ_PRECISION_EXACT,
                                &cpu_freq_hz);
  ESP_LOGI(TAG, "CPU: %lu MHz",
           static_cast<unsigned long>(cpu_freq_hz / 1000000));

  // For direct-wired TX→RX testing, disable carrier modulation.
  // Real IR LEDs need 38kHz carrier, but direct GPIO wiring needs
  // clean baseband edges for the decoder to work.
  // Duty 0.99 at 38kHz → low glitch ~0.26µs, invisible at 1µs RX resolution.
  codec::ProtocolConfig tx_protocol = ir_lasertag::protocols::kArcliteConfig;
  tx_protocol.carrier_duty = 0.99f;

  // ---- Initialize IrTransmitters ----
  rmt_tx_channel_config_t tx_cfg0 = {
      .gpio_num = kTxGpio0,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = kRmtResolutionHz,
      .mem_block_symbols = 48,
      .trans_queue_depth = 4,
  };
  esp_err_t err = transmitters[0].init(tx_cfg0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "TX0 init failed: %s", esp_err_to_name(err));
    return;
  }
  err = transmitters[0].set_protocol(tx_protocol);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "TX0 set_protocol failed: %s", esp_err_to_name(err));
    return;
  }

  rmt_tx_channel_config_t tx_cfg1 = {
      .gpio_num = kTxGpio1,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = kRmtResolutionHz,
      .mem_block_symbols = 48,
      .trans_queue_depth = 4,
  };
  err = transmitters[1].init(tx_cfg1);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "TX1 init failed: %s", esp_err_to_name(err));
    return;
  }
  err = transmitters[1].set_protocol(tx_protocol);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "TX1 set_protocol failed: %s", esp_err_to_name(err));
    return;
  }
  ESP_LOGI(TAG, "Transmitters ready (GPIO %d, %d)",
           static_cast<int>(kTxGpio0), static_cast<int>(kTxGpio1));

  // ---- Create worker tasks ----
  int worker_cores[kNumWorkers] = {1, 1, 0};
  const char* worker_names[kNumWorkers] = {"w0_c1", "w1_c1", "w2_c0"};

  for (int i = 0; i < kNumWorkers; i++) {
    workers[i].name = worker_names[i];
    workers[i].core = worker_cores[i];
    workers[i].run = false;
    workers[i].iterations = 0;

    xTaskCreatePinnedToCore(worker_task, worker_names[i], kWorkerStackSize,
                            &workers[i], 2, &worker_handles[i],
                            worker_cores[i]);
  }

  // ---- Initialize RMT RX sources + IrReceivers ----
  RmtRxSource sources[kMaxRxChannels];
  IrReceiver receivers[kMaxRxChannels];

  for (int i = 0; i < kMaxRxChannels; i++) {
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = kRxGpios[i],
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = kRmtResolutionHz,
        .mem_block_symbols = 48,
    };

    err = sources[i].init(rx_cfg);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "RX source %d init failed: %s", i,
               esp_err_to_name(err));
      return;
    }

    err = receivers[i].init(&sources[i]);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Receiver %d init failed: %s", i,
               esp_err_to_name(err));
      return;
    }

    err = receivers[i].set_protocol(
        ir_lasertag::protocols::kArcliteConfig);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Receiver %d set_protocol failed: %s", i,
               esp_err_to_name(err));
      return;
    }
  }
  ESP_LOGI(TAG, "Initialized %d RMT RX channels", kMaxRxChannels);

  // ---- Create RX poll tasks (one per channel) ----
  for (int i = 0; i < kMaxRxChannels; i++) {
    rx_poll[i].receiver = &receivers[i];
    rx_poll[i].run = false;
    rx_poll[i].msg_count = 0;
    rx_poll[i].timeout_count = 0;

    char name[12];
    snprintf(name, sizeof(name), "rx_poll%d", i);
    // Priority 4: above TX(3) and workers(2) to minimize scheduling
    // delay in latency measurement.  RX tasks block on xQueueReceive
    // between messages so they don't starve other tasks.
    xTaskCreatePinnedToCore(rx_poll_task, name, 4096, &rx_poll[i], 4,
                            &rx_poll_handles[i], 0);
  }

  vTaskDelay(pdMS_TO_TICKS(500));

  // ---- Run all test scenarios ----
  for (int di = 0; di < kNumDuties; di++) {
    for (int ci = 0; ci < kNumChCounts; ci++) {
      results[di][ci] = run_scenario(sources, receivers,
                                     kChannelCounts[ci], kDutyPcts[di]);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }

  // ---- Summary table ----
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "============================================================");
  ESP_LOGI(TAG, "=== RESULTS SUMMARY ===");
  ESP_LOGI(TAG, "============================================================");

  ESP_LOGI(TAG, "  %-10s %6s %6s %8s %7s %8s %8s %8s",
           "Scenario", "CPU0%%", "CPU1%%", "TX", "RX",
           "Lat min", "Lat avg", "Lat max");

  for (int di = 0; di < kNumDuties; di++) {
    for (int ci = 0; ci < kNumChCounts; ci++) {
      auto& r = results[di][ci];
      char label[24];
      snprintf(label, sizeof(label), "%dch %d%%",
               kChannelCounts[ci], kDutyPcts[di]);
      ESP_LOGI(TAG,
               "  %-10s %+5.1f%% %+5.1f%% %8lu %7lu %5lld us %5lld us"
               " %5lld us",
               label, r.core0_impact, r.core1_impact,
               static_cast<unsigned long>(r.total_tx),
               static_cast<unsigned long>(r.total_msgs),
               (long long)r.latency_min_us,
               (long long)r.latency_avg_us,
               (long long)r.latency_max_us);
    }
  }

  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "============================================================");

  // Cleanup
  for (int i = 0; i < kMaxRxChannels; i++) {
    receivers[i].deinit();
    sources[i].deinit();
  }
  transmitters[0].deinit();
  transmitters[1].deinit();
  for (int i = 0; i < kNumWorkers; i++) {
    vTaskDelete(worker_handles[i]);
  }

  ESP_LOGI(TAG, "Done");
}
