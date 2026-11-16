/**
 * @file driver/rmt_encoder.h
 * @brief RMT encoder stub for host-based testing.
 *
 * Enough of the ESP-IDF encoder interface to run ir_encoder.cpp on the
 * host. The copy encoder (rmt_stub.cpp) appends each symbol to
 * rmt_stub_captured(), so tests can check what a protocol encodes.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "driver/rmt_rx.h"  // rmt_channel_handle_t, rmt_symbol_word_t
#include "esp_err.h"

typedef enum {
  RMT_ENCODING_RESET = 0,
  RMT_ENCODING_COMPLETE = (1 << 0),
  RMT_ENCODING_MEM_FULL = (1 << 1),
} rmt_encode_state_t;

inline rmt_encode_state_t& operator|=(rmt_encode_state_t& a,
                                      rmt_encode_state_t b) {
  a = static_cast<rmt_encode_state_t>(a | b);
  return a;
}

typedef struct rmt_encoder_t rmt_encoder_t;
struct rmt_encoder_t {
  size_t (*encode)(rmt_encoder_t* encoder, rmt_channel_handle_t channel,
                   const void* primary_data, size_t data_size,
                   rmt_encode_state_t* ret_state);
  esp_err_t (*reset)(rmt_encoder_t* encoder);
  esp_err_t (*del)(rmt_encoder_t* encoder);
};
typedef rmt_encoder_t* rmt_encoder_handle_t;

typedef struct {
} rmt_copy_encoder_config_t;

esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t* config,
                               rmt_encoder_handle_t* ret_encoder);
esp_err_t rmt_del_encoder(rmt_encoder_handle_t encoder);
esp_err_t rmt_encoder_reset(rmt_encoder_handle_t encoder);

/// Symbols the stub copy encoder has emitted since the last clear.
std::vector<rmt_symbol_word_t>& rmt_stub_captured();
