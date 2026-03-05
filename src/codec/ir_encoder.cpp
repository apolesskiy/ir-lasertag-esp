/**
 * @file ir_encoder.cpp
 * @brief Custom RMT encoder for IR protocols using half-bit model.
 *
 * The encoder processes data bits and outputs RMT symbols according to
 * the half-bit model. Each data bit produces one combined two-phase
 * RMT symbol from its two half-bit definitions.
 */

#include "ir_lasertag/codec/ir_encoder.hpp"

#include <cstdlib>
#include <cstring>

#include "esp_check.h"
#include "esp_log.h"

static const char* TAG = "ir_encoder";

namespace ir_lasertag {
namespace codec {

namespace {

/**
 * @brief Internal encoder state.
 */
struct IrEncoderState {
  rmt_encoder_t base;           ///< Base encoder interface
  rmt_encoder_t* copy_encoder;  ///< Copy encoder for outputting symbols
  ProtocolConfig config;        ///< Protocol configuration

  // Pre-computed two-phase symbols (one symbol per data bit)
  rmt_symbol_word_t zero_symbol;   ///< Combined {half1, half2} for zero bit
  rmt_symbol_word_t one_symbol;    ///< Combined {half1, half2} for one bit

  // Encoding state machine
  int state;
  size_t byte_index;
  size_t bit_index;
  size_t total_bits_encoded;  ///< Total bits encoded so far
};

/**
 * @brief Build a combined two-phase RMT symbol from two half-bits.
 *
 * Each data bit produces exactly one RMT symbol with both phases filled.
 * This matches how the RMT hardware expects symbols (both duration0 and
 * duration1 should be non-zero for data symbols).
 */
inline rmt_symbol_word_t build_bit_symbol(const HalfBit& half1,
                                         const HalfBit& half2) {
  rmt_symbol_word_t sym = {};
  sym.duration0 = half1.duration_us;
  sym.level0 = (half1.level == Level::kMark) ? 1 : 0;
  sym.duration1 = half2.duration_us;
  sym.level1 = (half2.level == Level::kMark) ? 1 : 0;
  return sym;
}

/**
 * @brief Get bit value from data buffer.
 *
 * @param data Data buffer.
 * @param bit_index Index of bit to read.
 * @param msb_first True for MSB-first bit order.
 * @return Bit value (0 or 1).
 */
inline int get_bit(const uint8_t* data, size_t byte_idx, size_t bit_idx,
                   bool msb_first) {
  if (msb_first) {
    return (data[byte_idx] >> (7 - bit_idx)) & 1;
  } else {
    return (data[byte_idx] >> bit_idx) & 1;
  }
}

/**
 * @brief Encoder state machine states.
 */
enum EncoderState {
  kStateHeader = 0,
  kStateData,
  kStateFooter,
  kStateDone
};

/**
 * @brief Encode function for IR protocol.
 *
 * State machine:
 *   0: Encode header
 *   1: Encode data bits (one two-phase symbol per bit)
 *   2: Encode footer
 *   3: Done
 */
static size_t ir_encode(rmt_encoder_t* encoder, rmt_channel_handle_t channel,
                        const void* primary_data, size_t data_size,
                        rmt_encode_state_t* ret_state) {
  auto* ir_enc = reinterpret_cast<IrEncoderState*>(encoder);
  const auto* data = static_cast<const uint8_t*>(primary_data);
  rmt_encode_state_t session_state = RMT_ENCODING_RESET;
  size_t encoded_symbols = 0;

  // State machine
  while (ir_enc->state != kStateDone) {
    switch (ir_enc->state) {
      case kStateHeader: {
        if (ir_enc->config.header_mark_us > 0) {
          // Encode header as a combined mark+space symbol
          rmt_symbol_word_t header = {
              .duration0 = ir_enc->config.header_mark_us,
              .level0 = 1,
              .duration1 = ir_enc->config.header_space_us,
              .level1 = 0,
          };

          size_t n = ir_enc->copy_encoder->encode(
              ir_enc->copy_encoder, channel, &header,
              sizeof(rmt_symbol_word_t), &session_state);
          encoded_symbols += n;

          if (session_state & RMT_ENCODING_MEM_FULL) {
            *ret_state = RMT_ENCODING_MEM_FULL;
            return encoded_symbols;
          }
        }
        ir_enc->state = kStateData;
        ir_enc->byte_index = 0;
        ir_enc->bit_index = 0;
        ir_enc->total_bits_encoded = 0;
      }
        [[fallthrough]];

      case kStateData: {
        bool msb_first = (ir_enc->config.bit_order == BitOrder::kMsbFirst);
        // Use protocol bit_count if set, otherwise encode all data bytes
        size_t max_bits = (ir_enc->config.bit_count > 0)
                              ? ir_enc->config.bit_count
                              : (data_size * 8);

        while (ir_enc->byte_index < data_size &&
               ir_enc->total_bits_encoded < max_bits) {
          int bit = get_bit(data, ir_enc->byte_index, ir_enc->bit_index,
                            msb_first);

          // Select combined two-phase symbol for this bit
          const rmt_symbol_word_t* sym =
              bit ? &ir_enc->one_symbol : &ir_enc->zero_symbol;

          size_t n = ir_enc->copy_encoder->encode(
              ir_enc->copy_encoder, channel, sym,
              sizeof(rmt_symbol_word_t), &session_state);
          encoded_symbols += n;

          if (session_state & RMT_ENCODING_MEM_FULL) {
            *ret_state = RMT_ENCODING_MEM_FULL;
            return encoded_symbols;
          }

          // Advance to next bit
          ir_enc->total_bits_encoded++;
          ir_enc->bit_index++;
          if (ir_enc->bit_index >= 8) {
            ir_enc->bit_index = 0;
            ir_enc->byte_index++;
          }
        }

        ir_enc->state = kStateFooter;
      }
        [[fallthrough]];

      case kStateFooter: {
        bool has_footer = (ir_enc->config.footer_mark_us > 0 ||
                           ir_enc->config.footer_space_us > 0);
        if (has_footer) {
          rmt_symbol_word_t footer = {};
          if (ir_enc->config.footer_mark_us > 0) {
            // Mark + optional space
            footer.duration0 = ir_enc->config.footer_mark_us;
            footer.level0 = 1;
            footer.duration1 = ir_enc->config.footer_space_us;
            footer.level1 = 0;
          } else {
            // Space-only footer (e.g., inter-message gap for looped TX)
            footer.duration0 = ir_enc->config.footer_space_us;
            footer.level0 = 0;
            footer.duration1 = 0;
            footer.level1 = 0;
          }

          size_t n = ir_enc->copy_encoder->encode(
              ir_enc->copy_encoder, channel, &footer,
              sizeof(rmt_symbol_word_t), &session_state);
          encoded_symbols += n;

          if (session_state & RMT_ENCODING_MEM_FULL) {
            *ret_state = RMT_ENCODING_MEM_FULL;
            return encoded_symbols;
          }
        }
        ir_enc->state = kStateDone;
      }
        break;

      default:
        break;
    }
  }

  *ret_state = RMT_ENCODING_COMPLETE;
  return encoded_symbols;
}

/**
 * @brief Reset encoder state.
 */
static esp_err_t ir_reset(rmt_encoder_t* encoder) {
  auto* ir_enc = reinterpret_cast<IrEncoderState*>(encoder);
  rmt_encoder_reset(ir_enc->copy_encoder);
  ir_enc->state = kStateHeader;
  ir_enc->byte_index = 0;
  ir_enc->bit_index = 0;
  ir_enc->total_bits_encoded = 0;
  return ESP_OK;
}

/**
 * @brief Delete encoder and free resources.
 */
static esp_err_t ir_delete(rmt_encoder_t* encoder) {
  auto* ir_enc = reinterpret_cast<IrEncoderState*>(encoder);
  if (ir_enc->copy_encoder) {
    rmt_del_encoder(ir_enc->copy_encoder);
  }
  free(ir_enc);
  return ESP_OK;
}

}  // namespace

esp_err_t ir_encoder_create(const ProtocolConfig& config,
                            rmt_encoder_handle_t* encoder_out) {
  ESP_RETURN_ON_FALSE(encoder_out != nullptr, ESP_ERR_INVALID_ARG, TAG,
                      "encoder_out is null");
  ESP_RETURN_ON_FALSE(config.is_valid(), ESP_ERR_INVALID_ARG, TAG,
                      "invalid config");

  // Allocate encoder state
  auto* ir_enc = static_cast<IrEncoderState*>(calloc(1, sizeof(IrEncoderState)));
  ESP_RETURN_ON_FALSE(ir_enc != nullptr, ESP_ERR_NO_MEM, TAG,
                      "failed to allocate encoder");

  ir_enc->config = config;
  ir_enc->state = kStateHeader;
  ir_enc->byte_index = 0;
  ir_enc->bit_index = 0;
  ir_enc->total_bits_encoded = 0;

  // Set up base encoder interface
  ir_enc->base.encode = ir_encode;
  ir_enc->base.reset = ir_reset;
  ir_enc->base.del = ir_delete;

  // Create copy encoder
  rmt_copy_encoder_config_t copy_config = {};
  esp_err_t ret = rmt_new_copy_encoder(&copy_config, &ir_enc->copy_encoder);
  if (ret != ESP_OK) {
    free(ir_enc);
    return ret;
  }

  // Pre-compute combined two-phase bit symbols
  ir_enc->zero_symbol = build_bit_symbol(config.zero_half1, config.zero_half2);
  ir_enc->one_symbol = build_bit_symbol(config.one_half1, config.one_half2);

  ESP_LOGI(TAG, "encoder created: bit_count=%u, header=%u/%u us",
           config.bit_count, config.header_mark_us, config.header_space_us);
  ESP_LOGI(TAG, "  zero={d0=%u,l0=%u,d1=%u,l1=%u}",
           ir_enc->zero_symbol.duration0, ir_enc->zero_symbol.level0,
           ir_enc->zero_symbol.duration1, ir_enc->zero_symbol.level1);
  ESP_LOGI(TAG, "  one ={d0=%u,l0=%u,d1=%u,l1=%u}",
           ir_enc->one_symbol.duration0, ir_enc->one_symbol.level0,
           ir_enc->one_symbol.duration1, ir_enc->one_symbol.level1);
  ESP_LOGI(TAG, "  footer=%u/%u us (mark/space)",
           config.footer_mark_us, config.footer_space_us);

  *encoder_out = &ir_enc->base;
  return ESP_OK;
}

}  // namespace codec
}  // namespace ir_lasertag
