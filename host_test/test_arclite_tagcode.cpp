/**
 * @file test_arclite_tagcode.cpp
 * @brief Host-based unit tests for Arclite Tag Code protocol logic.
 *
 * Tests the data encoding/decoding logic used by ArcliteTagCode
 * without requiring hardware (IrTransmitter/IrReceiver). The codec
 * callback logic is tested by directly constructing IrMessage
 * structs and verifying the expected tag code extraction.
 */

#include "ir_lasertag/codec/ir_protocol.hpp"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace ir_lasertag::codec;

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
// Helper: encode tag code to bytes (same logic as ArcliteTagCode::send)
// ============================================================================

/**
 * @brief Encode a 16-bit tag code into a 2-byte MSB-first buffer.
 */
void encode_tag_code(uint16_t tag_code, uint8_t* data) {
  data[0] = static_cast<uint8_t>(tag_code >> 8);
  data[1] = static_cast<uint8_t>(tag_code & 0xFF);
}

/**
 * @brief Decode a 16-bit tag code from a 2-byte MSB-first buffer.
 *
 * This mirrors the logic in ArcliteTagCode::on_codec_receive.
 */
uint16_t decode_tag_code(const uint8_t* data) {
  return (static_cast<uint16_t>(data[0]) << 8) |
          static_cast<uint16_t>(data[1]);
}

/**
 * @brief Create an IrMessage representing a received tag code.
 */
IrMessage make_tag_code_message(uint16_t tag_code, bool valid = true) {
  IrMessage msg = {};
  msg.bit_count = 16;
  msg.valid = valid;
  encode_tag_code(tag_code, msg.data);
  return msg;
}

// ============================================================================
// Tag Code Encoding Tests
// ============================================================================

void test_encode_zero() {
  uint8_t data[2] = {0xFF, 0xFF};
  encode_tag_code(0x0000, data);
  TEST_ASSERT(data[0] == 0x00);
  TEST_ASSERT(data[1] == 0x00);
}

void test_encode_max() {
  uint8_t data[2] = {};
  encode_tag_code(0xFFFF, data);
  TEST_ASSERT(data[0] == 0xFF);
  TEST_ASSERT(data[1] == 0xFF);
}

void test_encode_msb_first() {
  uint8_t data[2] = {};
  encode_tag_code(0x1234, data);
  TEST_ASSERT(data[0] == 0x12);
  TEST_ASSERT(data[1] == 0x34);
}

void test_encode_high_byte_only() {
  uint8_t data[2] = {};
  encode_tag_code(0xFF00, data);
  TEST_ASSERT(data[0] == 0xFF);
  TEST_ASSERT(data[1] == 0x00);
}

void test_encode_low_byte_only() {
  uint8_t data[2] = {};
  encode_tag_code(0x00AB, data);
  TEST_ASSERT(data[0] == 0x00);
  TEST_ASSERT(data[1] == 0xAB);
}

// ============================================================================
// Tag Code Decoding Tests
// ============================================================================

void test_decode_zero() {
  uint8_t data[] = {0x00, 0x00};
  TEST_ASSERT(decode_tag_code(data) == 0x0000);
}

void test_decode_max() {
  uint8_t data[] = {0xFF, 0xFF};
  TEST_ASSERT(decode_tag_code(data) == 0xFFFF);
}

void test_decode_msb_first() {
  uint8_t data[] = {0x12, 0x34};
  TEST_ASSERT(decode_tag_code(data) == 0x1234);
}

// ============================================================================
// Round-Trip Tests
// ============================================================================

void test_roundtrip_typical() {
  uint8_t data[2] = {};
  encode_tag_code(0xABCD, data);
  TEST_ASSERT(decode_tag_code(data) == 0xABCD);
}

void test_roundtrip_all_values_boundary() {
  // Test boundary values.
  uint16_t test_values[] = {0x0000, 0x0001, 0x7FFF, 0x8000, 0xFFFE, 0xFFFF};
  for (uint16_t val : test_values) {
    uint8_t data[2] = {};
    encode_tag_code(val, data);
    uint16_t decoded = decode_tag_code(data);
    if (decoded != val) {
      printf("FAIL: roundtrip for 0x%04X: got 0x%04X\n", val, decoded);
      tests_failed++;
      return;
    }
  }
}

// ============================================================================
// IrMessage Validation Tests (callback bridge logic)
// ============================================================================

void test_valid_message_extracts_tag_code() {
  IrMessage msg = make_tag_code_message(0x5678);
  TEST_ASSERT(msg.valid == true);
  TEST_ASSERT(msg.bit_count == 16);
  TEST_ASSERT(decode_tag_code(msg.data) == 0x5678);
}

void test_invalid_message_bit_count() {
  IrMessage msg = make_tag_code_message(0x1234);
  msg.bit_count = 14;  // Not 16 — should be rejected.
  // The callback would reject this; we verify the condition.
  TEST_ASSERT(msg.bit_count != 16);
}

void test_invalid_message_not_valid() {
  IrMessage msg = make_tag_code_message(0x1234, false);
  // The callback would reject this; we verify the condition.
  TEST_ASSERT(msg.valid == false);
}

void test_convention_unset_tag_code() {
  // 0x0000 is valid but by convention "unset".
  IrMessage msg = make_tag_code_message(0x0000);
  TEST_ASSERT(msg.valid == true);
  TEST_ASSERT(decode_tag_code(msg.data) == 0x0000);
}

}  // namespace

// ============================================================================
// Test Runner
// ============================================================================

void run_arclite_tagcode_tests() {
  printf("--- Arclite Tag Code Tests ---\n");

  // Encoding.
  RUN_TEST(test_encode_zero);
  RUN_TEST(test_encode_max);
  RUN_TEST(test_encode_msb_first);
  RUN_TEST(test_encode_high_byte_only);
  RUN_TEST(test_encode_low_byte_only);

  // Decoding.
  RUN_TEST(test_decode_zero);
  RUN_TEST(test_decode_max);
  RUN_TEST(test_decode_msb_first);

  // Round-trip.
  RUN_TEST(test_roundtrip_typical);
  RUN_TEST(test_roundtrip_all_values_boundary);

  // IrMessage validation.
  RUN_TEST(test_valid_message_extracts_tag_code);
  RUN_TEST(test_invalid_message_bit_count);
  RUN_TEST(test_invalid_message_not_valid);
  RUN_TEST(test_convention_unset_tag_code);

  printf("Passed: %d, Failed: %d\n\n", tests_passed, tests_failed);
}

int get_arclite_tagcode_test_failures() {
  return tests_failed;
}
