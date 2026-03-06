/**
 * @file test_main.cpp
 * @brief Main entry point for host-based IR protocol tests.
 */

#include <cstdio>

extern void run_protocol_config_tests();
extern int get_protocol_config_test_failures();
extern void run_arclite_tagcode_tests();
extern int get_arclite_tagcode_test_failures();
extern void run_irt_gun_tests();
extern int get_irt_gun_test_failures();

int main() {
  printf("========================================\n");
  printf("IR Protocol Host Tests\n");
  printf("========================================\n\n");

  run_protocol_config_tests();
  run_arclite_tagcode_tests();
  run_irt_gun_tests();

  printf("\n========================================\n");

  int total_failures = get_protocol_config_test_failures() +
                       get_arclite_tagcode_test_failures() +
                       get_irt_gun_test_failures();

  if (total_failures == 0) {
    printf("All tests PASSED!\n");
    return 0;
  } else {
    printf("FAILED: %d test(s) failed\n", total_failures);
    return 1;
  }
}
