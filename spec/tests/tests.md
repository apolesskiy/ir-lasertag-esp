# Tests

**Version**: 1.0
**Purpose**: Inventory of the library's host tests — what each test file covers and how the single test executable is built and run.
**Status**: Built(1.0)

**Uses**:
.speck/rules/feature-system.md
spec/rules/testing.md

## Output
Subspecs:

Artifacts:
host_test/CMakeLists.txt

## Spec

### Host Test Executable

Single native executable `test_ir_protocol` (ctest test name `ir_protocol_tests`), defined in `host_test/CMakeLists.txt`. C++17, hardware-free. Built from:

| Input | Role |
|---|---|
| `host_test/test_protocol_config.cpp` | Test source |
| `host_test/test_arclite_tagcode.cpp` | Test source |
| `host_test/test_irt_gun.cpp` | Test source |
| `host_test/test_main.cpp` | Runner entry point |
| `host_test/stubs/` | ESP-IDF stub headers (`esp_err.h`, `driver/*`, `freertos/*`) |
| `src/codec/ir_protocol_config.cpp` | Only compiled library source (pure config logic, no hardware deps) |

`test_main.cpp` calls each file's `run_*_tests()` and sums per-file failure counts; non-zero failures → exit 1. Tests use in-file `TEST_ASSERT`/`RUN_TEST` macros (no external framework).

### Coverage by File

| File | Covers |
|---|---|
| `test_protocol_config.cpp` | `ProtocolConfig` validation (`is_valid`): carrier freq/duty bounds, zero/one half-bit durations, zero≠one distinctness, tolerance_percent ≤ 50, mark_bias range, bit_count ≤ 128; helpers `is_manchester`, `zero_bit_duration_us`/`one_bit_duration_us`, `HalfBit::is_mark`/`is_space`; representative Manchester / pulse-width / pulse-distance / Arclite / IRT configs |
| `test_arclite_tagcode.cpp` | Arclite TagCode 16-bit MSB-first encode/decode round-trips and `IrMessage` validation (bit_count==16, valid flag) using the same data logic as `ArcliteTagCode`; reconstructed locally since the hardware wrapper is not linked |
| `test_irt_gun.cpp` | `kIrtConfig`/`make_irt_config` properties (38 kHz, MSB-first, 16-bit, header, pulse-width marks, footer gap), ID encode/decode round-trips, `Irt` static helpers `get_ccm_id`/`is_unassigned`/`kUnassignedId`, and the pure `Irt::classify` role-detection logic (gun/spawner/invalid/hold) plus the `IrtMessage` result struct (links `protocols/irt/irt.hpp` header-only — constants, static methods, inline `classify`) |

### Scope Limits

Only `ir_protocol_config.cpp` is compiled into host tests. Encode/decode, transmit, and receive logic (`ir_encoder.cpp`, `ir_transmitter.cpp`, `ir_receiver.cpp`, `rmt_rx_source.cpp`) depend on RMT and are NOT host-tested; the tagcode/gun-ID encode/decode logic is re-implemented inline in the test files and checked there. On-device RMT TX/RX integration is exercised by the `examples/profiling` harness (see `spec/features/examples.md`).

### Run

Run command and the host vs. on-device tiers are defined in `spec/rules/testing.md` (not duplicated here).
