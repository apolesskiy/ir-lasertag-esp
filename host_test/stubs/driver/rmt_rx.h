/**
 * @file driver/rmt_rx.h
 * @brief Minimal RMT RX stub for host-based testing.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "driver/gpio.h"

typedef void* rmt_channel_handle_t;

typedef struct {
    uint16_t duration0 : 15;
    uint16_t level0 : 1;
    uint16_t duration1 : 15;
    uint16_t level1 : 1;
} rmt_symbol_word_t;

typedef struct {
    gpio_num_t gpio_num;
    uint32_t resolution_hz;
    size_t mem_block_symbols;
    struct {
        uint32_t invert_in : 1;
        uint32_t with_dma : 1;
        uint32_t io_loop_back : 1;
    } flags;
    int clk_src;
} rmt_rx_channel_config_t;

typedef struct {
    rmt_symbol_word_t* received_symbols;
    size_t num_symbols;
    uint32_t flags;
} rmt_rx_done_event_data_t;

typedef bool (*rmt_rx_done_callback_t)(rmt_channel_handle_t, const rmt_rx_done_event_data_t*, void*);

typedef struct {
    rmt_rx_done_callback_t on_recv_done;
} rmt_rx_event_callbacks_t;

typedef struct {
    uint32_t signal_range_min_ns;
    uint32_t signal_range_max_ns;
    struct {
        uint32_t en_partial_rx : 1;
    } flags;
} rmt_receive_config_t;
