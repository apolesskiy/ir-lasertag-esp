# Testing
**Version**: 1.0
**Purpose**: Test strategy and run commands for the library — host tests for logic, on-device tests for RMT integration.
**Status**: Built(1.0)

**Uses**:

## Output
Subspecs:

Artifacts:

## Spec

### Tiers

- **Host tests** (`host_test/`) — native CMake/ctest, hardware-independent. Cover protocol config validation and encode/decode logic. Compile only sources with no hardware dependency (e.g. `src/codec/ir_protocol_config.cpp`) plus ESP-IDF stub headers in `host_test/stubs/`. The single executable `test_ir_protocol` is the test runner. Inventory: `spec/tests/tests.md`.
- **On-device tests** — required for RMT TX/RX integration; need IR hardware (transmitter/receiver) and typically a second board. The `examples/profiling` app exercises the RX/decode stack on target for CPU-load measurement.

### Run

Host tests:

```
cd host_test && cmake -B build && cmake --build build && ctest --test-dir build --output-on-failure
```

### Definition of Done

New functionality is covered by tests; all host tests pass before commit (see `spec/rules/code-structure.md`).
