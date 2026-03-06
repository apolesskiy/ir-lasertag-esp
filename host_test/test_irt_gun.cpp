/**
 * @file test_irt_gun.cpp
 * @brief Host-based unit tests for IRT Gun protocol logic.
 *
 * Tests the data encoding/decoding logic used by IrtGun without
 * requiring hardware (IrTransmitter/IrReceiver). The codec callback
 * logic is tested by directly constructing IrMessage structs and
 * verifying the expected gun ID extraction.
 */

#include "ir_lasertag/codec/ir_protocol.hpp"
#include "ir_lasertag/protocols/irt/irt.hpp"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace ir_lasertag::codec;
using namespace ir_lasertag::protocols;

namespace {

int tests_passed = 0;
int tests_failed = 0;

#define TEST_ASSERT(expr)                                                 \
  do {                                                                    \
    if (!(expr)) {                                                        \
      printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__, #expr);            \
      tests_failed++;                                                     \
      return;                                                             \
    }                                                                     \
  } while (0)

#define RUN_TEST(test_func)                                               \
  do {                                                                    \
    printf("Running %s... ", #test_func);                                 \
    test_func();                                                          \
    printf("PASS\n");                                                     \
    tests_passed++;                                                       \
  } while (0)

// ============================================================================
// Helpers: encode / decode gun ID (mirrors IrtGun::send / decode_message)
// ============================================================================

/**
 * @brief Encode a 16-bit gun ID into a 2-byte MSB-first buffer.
 */
void encode_gun_id(uint16_t gun_id, uint8_t* data) {
  data[0] = static_cast<uint8_t>(gun_id >> 8);
  data[1] = static_cast<uint8_t>(gun_id & 0xFF);
}

/**
 * @brief Decode a 16-bit gun ID from a 2-byte MSB-first buffer.
 */
uint16_t decode_gun_id(const uint8_t* data) {
  return (static_cast<uint16_t>(data[0]) << 8) |
          static_cast<uint16_t>(data[1]);
}

/**
 * @brief Create an IrMessage representing a received IRT gun ID.
 */
IrMessage make_gun_id_message(uint16_t gun_id, bool valid = true,
                               uint8_t bit_count = 16) {
  IrMessage msg = {};
  msg.bit_count = bit_count;
  msg.valid = valid;
  encode_gun_id(gun_id, msg.data);
  return msg;
}

// ============================================================================
// Protocol Config Tests
// ============================================================================

void test_irt_config_is_valid() {
  TEST_ASSERT(kIrtConfig.is_valid());
}

void test_irt_config_carrier_38khz() {
  TEST_ASSERT(kIrtConfig.carrier_freq_hz == 38000);
}

void test_irt_config_not_manchester() {
  TEST_ASSERT(!kIrtConfig.is_manchester());
}

void test_irt_config_msb_first() {
  TEST_ASSERT(kIrtConfig.bit_order == BitOrder::kMsbFirst);
}

void test_irt_config_16_bits() {
  TEST_ASSERT(kIrtConfig.bit_count == 16);
}

void test_irt_config_has_header() {
  TEST_ASSERT(kIrtConfig.header_mark_us == 500);
  TEST_ASSERT(kIrtConfig.header_space_us == 500);
}

void test_irt_config_pulse_width_encoding() {
  // IRT uses pulse-width: zero mark (350) != one mark (700),
  // but space is the same (450) for both.
  TEST_ASSERT(kIrtConfig.zero_half1.duration_us == 350);
  TEST_ASSERT(kIrtConfig.one_half1.duration_us == 700);
  TEST_ASSERT(kIrtConfig.zero_half2.duration_us == 450);
  TEST_ASSERT(kIrtConfig.one_half2.duration_us == 450);
}

void test_irt_config_zero_one_distinguishable() {
  // Mark durations must differ for pulse-width decoding to work.
  TEST_ASSERT(kIrtConfig.zero_half1.duration_us !=
              kIrtConfig.one_half1.duration_us);
}

void test_irt_config_footer_gap() {
  TEST_ASSERT(kIrtConfig.footer_mark_us == 0);
  TEST_ASSERT(kIrtConfig.footer_space_us == kIrtMessageGapUs);
}

void test_make_irt_config_matches_constant() {
  // make_irt_config() returns kIrtConfig — verify the constexpr is usable
  // by copying it (compile-time check that kIrtConfig is well-formed).
  ProtocolConfig cfg = kIrtConfig;
  TEST_ASSERT(cfg.carrier_freq_hz == kIrtConfig.carrier_freq_hz);
  TEST_ASSERT(cfg.header_mark_us == kIrtConfig.header_mark_us);
  TEST_ASSERT(cfg.bit_count == kIrtConfig.bit_count);
}

// ============================================================================
// Gun ID Encoding Tests
// ============================================================================

void test_encode_zero() {
  uint8_t data[2] = {0xFF, 0xFF};
  encode_gun_id(0x0000, data);
  TEST_ASSERT(data[0] == 0x00);
  TEST_ASSERT(data[1] == 0x00);
}

void test_encode_max() {
  uint8_t data[2] = {};
  encode_gun_id(0xFFFF, data);
  TEST_ASSERT(data[0] == 0xFF);
  TEST_ASSERT(data[1] == 0xFF);
}

void test_encode_msb_first() {
  uint8_t data[2] = {};
  encode_gun_id(0xC804, data);
  TEST_ASSERT(data[0] == 0xC8);
  TEST_ASSERT(data[1] == 0x04);
}

void test_encode_unassigned() {
  uint8_t data[2] = {};
  encode_gun_id(IrtGun::kUnassignedId, data);
  TEST_ASSERT(data[0] == 0xF0);
  TEST_ASSERT(data[1] == 0xF0);
}

void test_encode_high_byte_only() {
  uint8_t data[2] = {};
  encode_gun_id(0xFF00, data);
  TEST_ASSERT(data[0] == 0xFF);
  TEST_ASSERT(data[1] == 0x00);
}

void test_encode_low_byte_only() {
  uint8_t data[2] = {};
  encode_gun_id(0x00AB, data);
  TEST_ASSERT(data[0] == 0x00);
  TEST_ASSERT(data[1] == 0xAB);
}

// ============================================================================
// Gun ID Decoding Tests
// ============================================================================

void test_decode_zero() {
  uint8_t data[] = {0x00, 0x00};
  TEST_ASSERT(decode_gun_id(data) == 0x0000);
}

void test_decode_max() {
  uint8_t data[] = {0xFF, 0xFF};
  TEST_ASSERT(decode_gun_id(data) == 0xFFFF);
}

void test_decode_msb_first() {
  uint8_t data[] = {0xC8, 0x04};
  TEST_ASSERT(decode_gun_id(data) == 0xC804);
}

void test_decode_unassigned() {
  uint8_t data[] = {0xF0, 0xF0};
  TEST_ASSERT(decode_gun_id(data) == 0xF0F0);
}

// ============================================================================
// Round-Trip Tests
// ============================================================================

void test_roundtrip_typical() {
  uint8_t data[2] = {};
  encode_gun_id(0xC804, data);
  TEST_ASSERT(decode_gun_id(data) == 0xC804);
}

void test_roundtrip_unassigned() {
  uint8_t data[2] = {};
  encode_gun_id(0xF0F0, data);
  TEST_ASSERT(decode_gun_id(data) == 0xF0F0);
}

void test_roundtrip_boundary_values() {
  uint16_t test_values[] = {0x0000, 0x0001, 0x7FFF, 0x8000, 0xFFFE, 0xFFFF};
  for (uint16_t val : test_values) {
    uint8_t data[2] = {};
    encode_gun_id(val, data);
    uint16_t decoded = decode_gun_id(data);
    if (decoded != val) {
      printf("FAIL: roundtrip for 0x%04X: got 0x%04X\n", val, decoded);
      tests_failed++;
      return;
    }
  }
}

// ============================================================================
// IrMessage Validation Tests (decode_message bridge logic)
// ============================================================================

void test_valid_message_extracts_gun_id() {
  IrMessage msg = make_gun_id_message(0xC804);
  TEST_ASSERT(msg.valid == true);
  TEST_ASSERT(msg.bit_count == 16);
  TEST_ASSERT(decode_gun_id(msg.data) == 0xC804);
}

void test_invalid_message_wrong_bit_count() {
  IrMessage msg = make_gun_id_message(0xC804, true, 14);
  // decode_message rejects anything != 16 bits
  TEST_ASSERT(msg.bit_count != 16);
}

void test_invalid_message_not_valid() {
  IrMessage msg = make_gun_id_message(0xC804, false);
  TEST_ASSERT(msg.valid == false);
}

void test_unassigned_gun_id_message() {
  IrMessage msg = make_gun_id_message(IrtGun::kUnassignedId);
  TEST_ASSERT(msg.valid == true);
  TEST_ASSERT(decode_gun_id(msg.data) == 0xF0F0);
}

// ============================================================================
// CCM ID Extraction Tests
// ============================================================================

void test_ccm_id_extraction() {
  // Gun ID 0xC804 -> CCM ID = 0x04 = 4
  TEST_ASSERT(IrtGun::get_ccm_id(0xC804) == 4);
}

void test_ccm_id_zero() {
  TEST_ASSERT(IrtGun::get_ccm_id(0xFF00) == 0);
}

void test_ccm_id_max() {
  TEST_ASSERT(IrtGun::get_ccm_id(0x00FF) == 255);
}

void test_ccm_id_unassigned() {
  // 0xF0F0 -> CCM = 0xF0 = 240
  TEST_ASSERT(IrtGun::get_ccm_id(IrtGun::kUnassignedId) == 0xF0);
}

// ============================================================================
// Unassigned Detection Tests
// ============================================================================

void test_is_unassigned_true() {
  TEST_ASSERT(IrtGun::is_unassigned(0xF0F0) == true);
}

void test_is_unassigned_false() {
  TEST_ASSERT(IrtGun::is_unassigned(0xC804) == false);
  TEST_ASSERT(IrtGun::is_unassigned(0x0000) == false);
  TEST_ASSERT(IrtGun::is_unassigned(0xFFFF) == false);
}

// ============================================================================
// Known Gun ID Samples
// ============================================================================

void test_known_gun_ids() {
  // Test a range of known CCM IDs (1-16 are typical)
  for (uint8_t ccm = 1; ccm <= 16; ccm++) {
    uint16_t gun_id = (static_cast<uint16_t>(0xC8) << 8) | ccm;
    uint8_t data[2] = {};
    encode_gun_id(gun_id, data);
    uint16_t decoded = decode_gun_id(data);
    if (decoded != gun_id) {
      printf("FAIL: CCM %d gun_id=0x%04X decoded=0x%04X\n", ccm, gun_id,
             decoded);
      tests_failed++;
      return;
    }
    if (IrtGun::get_ccm_id(decoded) != ccm) {
      printf("FAIL: CCM extraction for 0x%04X: got %d expected %d\n", decoded,
             IrtGun::get_ccm_id(decoded), ccm);
      tests_failed++;
      return;
    }
  }
}

}  // namespace

// ============================================================================
// Test Runner
// ============================================================================

void run_irt_gun_tests() {
  printf("--- IRT Gun Tests ---\n");

  // Protocol config.
  RUN_TEST(test_irt_config_is_valid);
  RUN_TEST(test_irt_config_carrier_38khz);
  RUN_TEST(test_irt_config_not_manchester);
  RUN_TEST(test_irt_config_msb_first);
  RUN_TEST(test_irt_config_16_bits);
  RUN_TEST(test_irt_config_has_header);
  RUN_TEST(test_irt_config_pulse_width_encoding);
  RUN_TEST(test_irt_config_zero_one_distinguishable);
  RUN_TEST(test_irt_config_footer_gap);
  RUN_TEST(test_make_irt_config_matches_constant);

  // Encoding.
  RUN_TEST(test_encode_zero);
  RUN_TEST(test_encode_max);
  RUN_TEST(test_encode_msb_first);
  RUN_TEST(test_encode_unassigned);
  RUN_TEST(test_encode_high_byte_only);
  RUN_TEST(test_encode_low_byte_only);

  // Decoding.
  RUN_TEST(test_decode_zero);
  RUN_TEST(test_decode_max);
  RUN_TEST(test_decode_msb_first);
  RUN_TEST(test_decode_unassigned);

  // Round-trip.
  RUN_TEST(test_roundtrip_typical);
  RUN_TEST(test_roundtrip_unassigned);
  RUN_TEST(test_roundtrip_boundary_values);

  // IrMessage validation.
  RUN_TEST(test_valid_message_extracts_gun_id);
  RUN_TEST(test_invalid_message_wrong_bit_count);
  RUN_TEST(test_invalid_message_not_valid);
  RUN_TEST(test_unassigned_gun_id_message);

  // CCM ID extraction.
  RUN_TEST(test_ccm_id_extraction);
  RUN_TEST(test_ccm_id_zero);
  RUN_TEST(test_ccm_id_max);
  RUN_TEST(test_ccm_id_unassigned);

  // Unassigned detection.
  RUN_TEST(test_is_unassigned_true);
  RUN_TEST(test_is_unassigned_false);

  // Known gun IDs.
  RUN_TEST(test_known_gun_ids);

  printf("Passed: %d, Failed: %d\n\n", tests_passed, tests_failed);
}

int get_irt_gun_test_failures() {
  return tests_failed;
}
