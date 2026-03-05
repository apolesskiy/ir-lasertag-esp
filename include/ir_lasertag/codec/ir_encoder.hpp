/**
 * @file ir_encoder.hpp
 * @brief Custom RMT encoder for IR protocols using half-bit model.
 */

#pragma once

#include "esp_err.h"
#include "driver/rmt_encoder.h"

#include "ir_lasertag/codec/ir_protocol.hpp"

namespace ir_lasertag {
namespace codec {

/**
 * @brief Create an IR protocol encoder.
 *
 * Creates a custom RMT encoder that encodes data according to the
 * half-bit model specified in the protocol configuration.
 *
 * @param config Protocol configuration.
 * @param encoder_out Output pointer to receive encoder handle.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t ir_encoder_create(const ProtocolConfig& config,
                            rmt_encoder_handle_t* encoder_out);

}  // namespace codec
}  // namespace ir_lasertag
