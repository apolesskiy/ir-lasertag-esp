/**
 * @file driver/rmt_tx.h
 * @brief Minimal RMT TX stub for host-based testing.
 */

#pragma once

#include <cstdint>
#include "driver/gpio.h"
#include "driver/rmt_rx.h"      // for rmt_channel_handle_t, rmt_symbol_word_t
#include "driver/rmt_encoder.h" // for rmt_encoder_handle_t

typedef struct {
    size_t num_symbols;
} rmt_tx_done_event_data_t;

typedef struct {
    gpio_num_t gpio_num;
    uint32_t resolution_hz;
    size_t mem_block_symbols;
    size_t trans_queue_depth;
    struct {
        uint32_t invert_out : 1;
        uint32_t with_dma : 1;
        uint32_t io_loop_back : 1;
        uint32_t io_od_mode : 1;
    } flags;
    int clk_src;
} rmt_tx_channel_config_t;
