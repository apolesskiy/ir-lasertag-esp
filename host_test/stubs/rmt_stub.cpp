/**
 * @file rmt_stub.cpp
 * @brief Host implementation of the stub RMT copy encoder.
 */

#include <cstdlib>

#include "driver/rmt_encoder.h"

std::vector<rmt_symbol_word_t>& rmt_stub_captured() {
  static std::vector<rmt_symbol_word_t> captured;
  return captured;
}

namespace {

size_t copy_encode(rmt_encoder_t*, rmt_channel_handle_t, const void* data,
                   size_t size, rmt_encode_state_t* state) {
  const auto* symbols = static_cast<const rmt_symbol_word_t*>(data);
  const size_t n = size / sizeof(rmt_symbol_word_t);
  for (size_t i = 0; i < n; i++) {
    rmt_stub_captured().push_back(symbols[i]);
  }
  *state = RMT_ENCODING_COMPLETE;
  return n;
}

esp_err_t copy_reset(rmt_encoder_t*) { return ESP_OK; }

esp_err_t copy_del(rmt_encoder_t* encoder) {
  free(encoder);
  return ESP_OK;
}

}  // namespace

esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t*,
                               rmt_encoder_handle_t* ret_encoder) {
  auto* e = static_cast<rmt_encoder_t*>(calloc(1, sizeof(rmt_encoder_t)));
  e->encode = copy_encode;
  e->reset = copy_reset;
  e->del = copy_del;
  *ret_encoder = e;
  return ESP_OK;
}

esp_err_t rmt_del_encoder(rmt_encoder_handle_t encoder) {
  return encoder->del(encoder);
}

esp_err_t rmt_encoder_reset(rmt_encoder_handle_t encoder) {
  return encoder->reset(encoder);
}
