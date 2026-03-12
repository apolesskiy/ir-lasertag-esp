# Profiling Example — RMT TX/RX Load Test

Measures CPU impact of RMT-based IR receive and decode under controlled,
repeatable signal conditions using the ESP32's own RMT TX channels as
signal sources.

## Test Methodology

Two RMT TX channels generate protocol-encoded IR signals (Arclite Manchester
and IRT pulse-width). Up to four RMT RX channels receive and decode them
through the IrRxSource / IrReceiver stack. Worker tasks on each core spin
tight compute loops and count iterations. CPU impact is measured as throughput
reduction when receivers are active vs. baseline.

### Test Phases

For each configuration (protocol × duty cycle × channel count):

1. **Baseline** — Workers run, IR receivers OFF, TX off.
2. **Loaded** — Workers run, IR receivers ON, TX generating signal.
3. **Recovery** — Workers run, IR receivers OFF, TX off.

Impact % = `(1 - loaded_iterations / baseline_iterations) × 100`.

### Test Matrix

| Parameter | Values |
|-----------|--------|
| Protocols | Arclite (Manchester, 16-bit), IRT (pulse-width, 16-bit) |
| RX channels | 1, 2, 4 |
| Duty cycles | 100% (continuous), 10% |

## Hardware Setup

### Target Module

ESP32-S3-WROOM-1 (same hardware as software-ir-receiver-esp profiling example).

### GPIO Allocation

#### RMT TX Outputs (Signal Generators)

| Function | GPIO | Wired To |
|----------|------|----------|
| RMT TX 0 | 21 | Bank A RX inputs |
| RMT TX 1 | 47 | Bank B RX inputs |

#### RMT RX Inputs (4 channels)

| Channel | GPIO | Bank | Wired To |
|---------|------|------|----------|
| 0 | 4 | A | TX0 (GPIO 21) |
| 1 | 5 | A | TX0 (GPIO 21) |
| 2 | 38 | B | TX1 (GPIO 47) |
| 3 | 39 | B | TX1 (GPIO 47) |

### Wiring

```
  GPIO 21 (TX0) ──── GPIO 4  (RX ch 0)
                └─── GPIO 5  (RX ch 1)

  GPIO 47 (TX1) ──── GPIO 38 (RX ch 2)
                └─── GPIO 39 (RX ch 3)
```

Uses a subset of the software-ir-receiver-esp profiling wiring. Both
examples can share the same hardware.

### Build and Flash

```bash
cd examples/profiling
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```
