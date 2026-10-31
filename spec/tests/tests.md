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
| `host_test/stubs/` | ESP-IDF stub headers (`esp_err.h`, `esp_check.h`, `esp_log.h`, `esp_attr.h`, `esp_heap_caps.h`, `sdkconfig.h`, `driver/*`, `freertos/*`) and `rmt_stub.cpp`, a copy encoder that records the symbols it is given |
| `src/codec/ir_protocol_config.cpp` | Library source: config logic |
| `src/codec/ir_encoder.cpp` | Library source: the RMT encoder, run on the stub copy encoder |
| `src/protocols/arclite/arclite_frame.cpp` | Library source: the Arclite frame codec (pure logic) |

`test_main.cpp` calls each file's `run_*_tests()` and sums per-file failure counts; non-zero failures → exit 1. Tests use in-file `TEST_ASSERT`/`RUN_TEST` macros (no external framework).

### Coverage by File

| File | Covers |
|---|---|
| `test_protocol_config.cpp` | `ProtocolConfig` validation (`is_valid`): carrier freq/duty bounds, zero/one half-bit durations, zero≠one distinctness, tolerance_percent ≤ 50, mark_bias range, bit_count ≤ 128; helpers `is_manchester`, `zero_bit_duration_us`/`one_bit_duration_us`, `HalfBit::is_mark`/`is_space`; representative Manchester / pulse-width / pulse-distance / Arclite / IRT configs |
| `test_arclite_tagcode.cpp` | The Arclite frame codec against spec/features/protocols/arclite.md: the data word table, parity round trip of every code, parity detection of single, 2- and 3-adjacent flips and its blind spots (1x1, a run of 4), `arclite_word_bytes` byte order; frame shape for every code (10.64 ms, merged 820 µs header space, final mark), a code that doesn't fit (> 0x3FFF); the real encoder's golden symbols for a known word and its agreement with the frame's runs (bit order, 2 ms footer); `kArcliteConfig`; the receive decoder for every code, every data window edge (inclusive), the header found by mark plus space (both sum windows' edges at header marks of 400, 520 and 800 µs, marks outside 400–800, a stretched header mark, data pairs never forming a header), glitch, out-of-window and missing mid-bit runs, header mark and header space windows, lost edge, parity failure, truncation, noise, an abandoning header starting the next frame, receiver stretch, capture runs and independent bursts. Every decode case runs through both symbol layouts (RMT RX and software receiver) |
| `test_irt_gun.cpp` | `kIrtConfig`/`make_irt_config` properties (38 kHz, MSB-first, 16-bit, header, pulse-width marks, footer gap), ID encode/decode round-trips, `Irt` static helpers `get_ccm_id`/`is_unassigned`/`kUnassignedId`, and the pure `Irt::classify` role-detection logic (gun/spawner/invalid/hold) plus the `IrtMessage` result struct (links `protocols/irt/irt.hpp` header-only — constants, static methods, inline `classify`) |

### Scope Limits

The transmitter, receiver and RMT source (`ir_transmitter.cpp`, `ir_receiver.cpp`, `rmt_rx_source.cpp`) depend on the RMT driver and are NOT host-tested. The encoder and the Arclite codec are compiled and tested; the IRT gun-ID logic is re-implemented inline in its test file. On-device RMT TX/RX integration is exercised by the `examples/profiling` harness (see `spec/features/examples.md`).

### Run

Run command and the host vs. on-device tiers are defined in `spec/rules/testing.md` (not duplicated here).
