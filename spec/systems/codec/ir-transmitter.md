# IR Transmitter

**Version**: 1.0
**Purpose**: `IrTransmitter` wraps one RMT TX channel to send single IR packets (encoded or raw), blocking or async.
**Status**: Built(1.0)

**Uses**:
.speck/rules/feature-system.md
spec/systems/codec/protocol-config.md
spec/systems/codec/ir-encoder.md
spec/rules/cpp-style.md

## Output
Subspecs:

Artifacts:
include/ir_lasertag/codec/ir_transmitter.hpp
src/codec/ir_transmitter.cpp

## Spec

### Role

`IrTransmitter` owns one RMT TX channel plus its IR encoder (see spec/systems/codec/ir-encoder.md) and an internal copy encoder used for raw mode. Non-copyable. The destructor calls `deinit()`. Protocol can change at runtime without reinitializing the channel.

### Lifecycle & State

| Method | Effect |
|---|---|
| `init(const rmt_tx_channel_config_t&)` | create RMT TX channel from the caller's config (GPIO + RMT params), register TX-done callback, create copy encoder, enable channel; `ESP_ERR_INVALID_STATE` if already initialized |
| `deinit()` | disable channel, destroy encoders, delete channel, clear state |
| `is_initialized()` | channel ready |
| `is_busy()` | a transmission is in flight (set true on send start, cleared by the TX-done ISR) |

The caller supplies the GPIO via the RMT channel config; the transmitter does not take a separate GPIO argument.

### Protocol

| Method | Effect |
|---|---|
| `set_protocol(const ProtocolConfig&)` | validate, store, recreate the encoder, apply carrier (`carrier_duty >= 1.0` disables the carrier; else apply `carrier_freq_hz` + duty); `ESP_ERR_INVALID_ARG` on invalid config |
| `get_protocol()` | current `ProtocolConfig` |
| `set_carrier_duty(float)` | update stored duty and reapply carrier without full re-setup; duty `>= 1.0` disables carrier; rejects values outside 0.0-1.0 |

A protocol must be set before encoded sends; `send_async` returns `ESP_ERR_INVALID_STATE` if no protocol is set.

### Send

| Method | Behavior |
|---|---|
| `send_async(data, bit_count, loop_count=0)` | reset encoder, start RMT transmit, return immediately; sets busy; `ESP_ERR_INVALID_STATE` if not initialized / no protocol / already busy; `ESP_ERR_INVALID_ARG` on null data or zero bits |
| `wait_done(timeout_ms)` | block on `rmt_tx_wait_all_done`; `ESP_ERR_TIMEOUT` on timeout |
| `send(data, bit_count, timeout_ms=1000, loop_count=0)` | `send_async` then `wait_done`; blocking |
| `send_raw(timings, count, timeout_ms=1000)` | transmit `RawTiming[]` via the copy encoder, one symbol per timing, bypassing protocol encoding; blocking; requires init only (no protocol) |

`bit_count` is bits; the byte count transmitted = `ceil(bit_count / 8)`.

### Loop Count

`loop_count` maps to the RMT `rmt_transmit_config_t.loop_count`: `0` = single shot, `N > 0` = the packet repeats N times atomically in hardware (no per-repeat CPU involvement). Used for protocols that emit a packet multiple times in one burst.

### Concurrency

`is_busy()` is the async completion signal, cleared by the IRAM TX-done callback. `send_raw` waits internally and does not use the busy flag the same way; it always blocks until done. The transmitter is single-channel and single-in-flight: starting a send while busy is rejected.
