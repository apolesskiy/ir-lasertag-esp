/**
 * @file test_protocol_config.cpp
 * @brief Host-based unit tests for IR protocol configuration.
 */

#include "ir_lasertag/codec/ir_protocol.hpp"

#include <cassert>
#include <cstdio>

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

void test_default_config_is_valid() {
  ProtocolConfig config;
  TEST_ASSERT(config.is_valid());
}

void test_invalid_carrier_frequency_too_low() {
  ProtocolConfig config;
  config.carrier_freq_hz = 5000;
  TEST_ASSERT(!config.is_valid());
}

void test_invalid_carrier_frequency_too_high() {
  ProtocolConfig config;
  config.carrier_freq_hz = 200000;
  TEST_ASSERT(!config.is_valid());
}

void test_invalid_duty_cycle_negative() {
  ProtocolConfig config;
  config.carrier_duty = -0.1f;
  TEST_ASSERT(!config.is_valid());
}

void test_invalid_duty_cycle_too_high() {
  ProtocolConfig config;
  config.carrier_duty = 1.5f;
  TEST_ASSERT(!config.is_valid());
}

void test_invalid_zero_bit_all_zero_duration() {
  ProtocolConfig config;
  config.zero_half1.duration_us = 0;
  config.zero_half2.duration_us = 0;
  TEST_ASSERT(!config.is_valid());
}

void test_invalid_one_bit_all_zero_duration() {
  ProtocolConfig config;
  config.one_half1.duration_us = 0;
  config.one_half2.duration_us = 0;
  TEST_ASSERT(!config.is_valid());
}

void test_invalid_zero_and_one_identical() {
  ProtocolConfig config;
  config.zero_half1 = {600, Level::kMark};
  config.zero_half2 = {600, Level::kSpace};
  config.one_half1 = {600, Level::kMark};
  config.one_half2 = {600, Level::kSpace};
  TEST_ASSERT(!config.is_valid());
}

void test_valid_pulse_distance_config() {
  ProtocolConfig config;
  config.zero_half1 = {560, Level::kMark};
  config.zero_half2 = {560, Level::kSpace};
  config.one_half1 = {560, Level::kMark};
  config.one_half2 = {1690, Level::kSpace};
  TEST_ASSERT(config.is_valid());
}

void test_valid_pulse_width_config() {
  ProtocolConfig config;
  config.zero_half1 = {600, Level::kMark};
  config.zero_half2 = {600, Level::kSpace};
  config.one_half1 = {1200, Level::kMark};
  config.one_half2 = {600, Level::kSpace};
  TEST_ASSERT(config.is_valid());
}

void test_valid_manchester_config() {
  ProtocolConfig config;
  config.zero_half1 = {889, Level::kMark};
  config.zero_half2 = {889, Level::kSpace};
  config.one_half1 = {889, Level::kSpace};
  config.one_half2 = {889, Level::kMark};
  TEST_ASSERT(config.is_valid());
}

void test_bit_duration_helpers() {
  ProtocolConfig config;
  config.zero_half1.duration_us = 560;
  config.zero_half2.duration_us = 560;
  config.one_half1.duration_us = 560;
  config.one_half2.duration_us = 1690;

  TEST_ASSERT(config.zero_bit_duration_us() == 1120);
  TEST_ASSERT(config.one_bit_duration_us() == 2250);
}

void test_halfbit_helpers() {
  HalfBit mark = {500, Level::kMark};
  HalfBit space = {500, Level::kSpace};

  TEST_ASSERT(mark.is_mark());
  TEST_ASSERT(!mark.is_space());
  TEST_ASSERT(!space.is_mark());
  TEST_ASSERT(space.is_space());
}

void test_tolerance_limit() {
  ProtocolConfig config;
  config.tolerance_percent = 50;
  TEST_ASSERT(config.is_valid());

  config.tolerance_percent = 51;
  TEST_ASSERT(!config.is_valid());
}

void test_mark_bias_default_and_range() {
  ProtocolConfig config;
  TEST_ASSERT(config.mark_bias_us == 0);

  config.mark_bias_us = 75;
  TEST_ASSERT(config.is_valid());

  config.mark_bias_us = -30;
  TEST_ASSERT(config.is_valid());
}

void test_bit_count_default_and_range() {
  ProtocolConfig config;
  TEST_ASSERT(config.bit_count == 0);
  TEST_ASSERT(config.is_valid());

  config.bit_count = 14;
  TEST_ASSERT(config.is_valid());

  config.bit_count = 128;
  TEST_ASSERT(config.is_valid());

  config.bit_count = 129;
  TEST_ASSERT(!config.is_valid());
}

void test_is_manchester_pulse_distance() {
  ProtocolConfig config;
  config.zero_half1 = {560, Level::kMark};
  config.zero_half2 = {560, Level::kSpace};
  config.one_half1 = {560, Level::kMark};
  config.one_half2 = {1690, Level::kSpace};
  TEST_ASSERT(!config.is_manchester());
}

void test_is_manchester_pulse_width() {
  ProtocolConfig config;
  config.zero_half1 = {350, Level::kMark};
  config.zero_half2 = {450, Level::kSpace};
  config.one_half1 = {700, Level::kMark};
  config.one_half2 = {450, Level::kSpace};
  TEST_ASSERT(!config.is_manchester());
}

void test_is_manchester_true() {
  ProtocolConfig config;
  config.zero_half1 = {300, Level::kMark};
  config.zero_half2 = {300, Level::kSpace};
  config.one_half1 = {300, Level::kSpace};
  config.one_half2 = {300, Level::kMark};
  TEST_ASSERT(config.is_manchester());
}

void test_valid_irt_gun_config() {
  ProtocolConfig config;
  config.carrier_freq_hz = 38000;
  config.carrier_duty = 0.33f;
  config.header_mark_us = 500;
  config.header_space_us = 500;
  config.zero_half1 = {350, Level::kMark};
  config.zero_half2 = {450, Level::kSpace};
  config.one_half1 = {700, Level::kMark};
  config.one_half2 = {450, Level::kSpace};
  config.footer_mark_us = 0;
  config.bit_order = BitOrder::kMsbFirst;
  config.bit_count = 16;
  config.tolerance_percent = 25;
  TEST_ASSERT(config.is_valid());
  TEST_ASSERT(!config.is_manchester());
}

void test_valid_arclite_config() {
  ProtocolConfig config;
  config.carrier_freq_hz = 38000;
  config.carrier_duty = 0.33f;
  config.header_mark_us = 520;
  config.header_space_us = 520;
  config.zero_half1 = {300, Level::kMark};
  config.zero_half2 = {300, Level::kSpace};
  config.one_half1 = {300, Level::kSpace};
  config.one_half2 = {300, Level::kMark};
  config.footer_mark_us = 0;
  config.bit_order = BitOrder::kMsbFirst;
  config.bit_count = 16;
  config.tolerance_percent = 25;
  TEST_ASSERT(config.is_valid());
  TEST_ASSERT(config.is_manchester());
}

}  // namespace

void run_protocol_config_tests() {
  RUN_TEST(test_default_config_is_valid);
  RUN_TEST(test_invalid_carrier_frequency_too_low);
  RUN_TEST(test_invalid_carrier_frequency_too_high);
  RUN_TEST(test_invalid_duty_cycle_negative);
  RUN_TEST(test_invalid_duty_cycle_too_high);
  RUN_TEST(test_invalid_zero_bit_all_zero_duration);
  RUN_TEST(test_invalid_one_bit_all_zero_duration);
  RUN_TEST(test_invalid_zero_and_one_identical);
  RUN_TEST(test_valid_pulse_distance_config);
  RUN_TEST(test_valid_pulse_width_config);
  RUN_TEST(test_valid_manchester_config);
  RUN_TEST(test_bit_duration_helpers);
  RUN_TEST(test_halfbit_helpers);
  RUN_TEST(test_tolerance_limit);
  RUN_TEST(test_mark_bias_default_and_range);
  RUN_TEST(test_bit_count_default_and_range);
  RUN_TEST(test_is_manchester_pulse_distance);
  RUN_TEST(test_is_manchester_pulse_width);
  RUN_TEST(test_is_manchester_true);
  RUN_TEST(test_valid_irt_gun_config);
  RUN_TEST(test_valid_arclite_config);

  printf("\n--- Protocol Config Tests ---\n");
  printf("Passed: %d, Failed: %d\n", tests_passed, tests_failed);
}

int get_protocol_config_test_failures() {
  return tests_failed;
}
