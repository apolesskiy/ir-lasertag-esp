/**
 * @file ir_protocol_config.cpp
 * @brief Protocol configuration validation.
 */

#include "ir_lasertag/codec/ir_protocol.hpp"

namespace ir_lasertag {
namespace codec {

bool ProtocolConfig::is_valid() const {
  // Carrier frequency must be reasonable (10kHz - 100kHz typical for IR)
  if (carrier_freq_hz < 10000 || carrier_freq_hz > 100000) {
    return false;
  }

  // Duty cycle must be between 0 and 1
  if (carrier_duty < 0.0f || carrier_duty > 1.0f) {
    return false;
  }

  // At least one half-bit timing must be non-zero for each bit value
  if (zero_half1.duration_us == 0 && zero_half2.duration_us == 0) {
    return false;
  }
  if (one_half1.duration_us == 0 && one_half2.duration_us == 0) {
    return false;
  }

  // Zero and one must be distinguishable
  // Either the durations or levels must differ
  bool durations_same = (zero_half1.duration_us == one_half1.duration_us) &&
                        (zero_half2.duration_us == one_half2.duration_us);
  bool levels_same = (zero_half1.level == one_half1.level) &&
                     (zero_half2.level == one_half2.level);

  if (durations_same && levels_same) {
    return false;  // Can't distinguish zero from one
  }

  // Tolerance must be reasonable
  if (tolerance_percent > 50) {
    return false;
  }

  // Bit count must not exceed maximum supported
  if (bit_count > IrMessage::kMaxDataBytes * 8) {
    return false;
  }

  return true;
}

}  // namespace codec
}  // namespace ir_lasertag
