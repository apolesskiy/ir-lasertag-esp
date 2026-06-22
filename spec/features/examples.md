# Examples

**Version**: 1.0
**Purpose**: Runnable ESP-IDF example applications demonstrating the library's transmit, receive, and protocol wrappers on real hardware.
**Status**: Built(1.0)

**Uses**:
.speck/rules/feature-system.md
spec/features/codec/transmit.md
spec/features/codec/receive.md
spec/features/protocols/arclite.md
spec/features/protocols/irt.md

## Output
Subspecs:

Artifacts:
examples/basic/main.cpp
examples/basic/CMakeLists.txt
examples/profiling/README.md
examples/profiling/main/main.cpp
examples/profiling/main/CMakeLists.txt
examples/profiling/CMakeLists.txt

## Spec

### Scope

Two ESP-IDF example apps under `examples/`, building the library as an external component (`EXTRA_COMPONENT_DIRS = ../..`). They demonstrate library usage on target and are not host-tested.

### basic — Arclite TagCode send/receive demo

`examples/basic/main.cpp`. Minimal Arclite TagCode round-trip on one board:

- Construct `IrTransmitter`, init with `rmt_tx_channel_config_t` (TX GPIO, 1 MHz resolution).
- Construct `RmtRxSource`, init with `rmt_rx_channel_config_t` (RX GPIO, 1 MHz resolution); pass to `IrReceiver::init`.
- Construct `ArcliteTagCode`, `init(&tx, &rx, kArcliteConfig)` to wire codec + protocol config (see `spec/features/protocols/arclite.md`).
- `start_receive()`, then loop: `send(tag_code)` and poll `try_receive(&received)` once per second.

Demonstrates the TX/RX RMT setup and the codec + protocol-wrapper init/polling pattern (see `spec/features/codec/transmit.md`, `spec/features/codec/receive.md`).

### profiling — on-device CPU-load harness

`examples/profiling/main/main.cpp` + `examples/profiling/README.md`. Measures CPU impact of the RMT receive/decode stack under controlled, repeatable signal conditions, using the ESP32's own RMT TX channels as signal generators.

- Two RMT TX channels generate Arclite (Manchester) and IRT (pulse-width) signals into up to four RMT RX channels through the `IrRxSource`/`IrReceiver` stack (see `spec/features/protocols/arclite.md`, `spec/features/protocols/irt.md`).
- Compute-worker tasks spin tight loops counting iterations; throughput reduction = CPU impact.
- Each scenario runs three phases — baseline (receivers off), loaded (receivers on, TX generating), recovery (receivers off). `impact% = (1 − loaded/baseline) × 100`.
- Matrix = protocol × duty cycle (100% continuous / 10% burst) × RX channel count (1 / 2 / 4).
- Target = ESP32-S3-WROOM-1 with documented GPIO allocation and TX→RX wiring.
- Build/flash: `idf.py set-target esp32s3 && idf.py build && idf.py -p <PORT> flash monitor`.

Hardware GPIO allocation, wiring diagram, and the full test matrix are documented in the `examples/profiling/README.md` artifact (preserved, user-facing) — referenced here, not duplicated.

### Acceptance

- basic: builds and flashes; logs sent tag codes and reports any received tag code when TX is looped back to RX.
- profiling: builds for esp32s3, flashes, and prints per-scenario baseline/loaded/recovery iteration counts and impact% across the matrix.
