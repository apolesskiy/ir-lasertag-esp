/**
 * @file milestag.hpp
 * @brief MilesTag protocol constants and configuration.
 *
 * MilesTag is a widely-used laser tag protocol using pulse-distance encoding.
 * This header provides timing constants and default protocol configuration
 * for encoding/decoding MilesTag IR messages.
 */

#pragma once

#include <cstdint>

#include "ir_lasertag/codec/ir_protocol.hpp"

namespace ir_lasertag {
namespace protocols {

// ============================================================================
// MilesTag Protocol Constants
// ============================================================================

/// @name Timing Constants
/// @{

/// Carrier frequency in Hz
constexpr uint32_t kMilestagCarrierHz = 38000;

/// Header mark duration in microseconds
constexpr uint16_t kMilestagHeaderMarkUs = 2400;

/// Header space duration in microseconds
constexpr uint16_t kMilestagHeaderSpaceUs = 600;

/// Bit mark duration in microseconds (same for 0 and 1)
constexpr uint16_t kMilestagBitMarkUs = 600;

/// Zero bit space duration in microseconds
constexpr uint16_t kMilestagZeroSpaceUs = 600;

/// One bit space duration in microseconds
constexpr uint16_t kMilestagOneSpaceUs = 1200;

/// Footer mark duration in microseconds
constexpr uint16_t kMilestagFooterMarkUs = 600;

/// Minimum gap between messages in microseconds
constexpr uint32_t kMilestagMessageGapUs = 10000;

/// Message bit count (standard MilesTag)
constexpr uint8_t kMilestagBitCount = 14;

/// @}

/**
 * @brief Default MilesTag protocol configuration.
 *
 * Pulse-distance encoding:
 * - 2400µs header mark, 600µs header space
 * - 0 bit: 600µs mark, 600µs space (total 1200µs)
 * - 1 bit: 600µs mark, 1200µs space (total 1800µs)
 * - 14-bit messages, MSB first
 * - 38kHz carrier
 *
 * Note: carrier_duty and mark_bias_us use defaults here but may
 * be overridden at runtime by user-configurable device settings.
 */
constexpr codec::ProtocolConfig kMilestagConfig = {
    .carrier_freq_hz = kMilestagCarrierHz,
    .carrier_duty = 0.33f,
    .header_mark_us = kMilestagHeaderMarkUs,
    .header_space_us = kMilestagHeaderSpaceUs,
    .zero_half1 = {kMilestagBitMarkUs, codec::Level::kMark},
    .zero_half2 = {kMilestagZeroSpaceUs, codec::Level::kSpace},
    .one_half1 = {kMilestagBitMarkUs, codec::Level::kMark},
    .one_half2 = {kMilestagOneSpaceUs, codec::Level::kSpace},
    .footer_mark_us = kMilestagFooterMarkUs,
    .bit_order = codec::BitOrder::kMsbFirst,
    .bit_count = kMilestagBitCount,
    .tolerance_percent = 25,
    .mark_bias_us = 0,
};

}  // namespace protocols
}  // namespace ir_lasertag
