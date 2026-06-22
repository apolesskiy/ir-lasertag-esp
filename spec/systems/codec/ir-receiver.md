# IR Receiver

**Version**: 1.0
**Purpose**: `IrReceiver` decodes symbol frames delivered by a pluggable `IrRxSource` into `IrMessage` (decoded mode) or `RawTiming` (raw mode).
**Status**: Built(1.0)

**Uses**:
.speck/rules/feature-system.md
spec/systems/codec/protocol-config.md
spec/systems/codec/rx-source.md
spec/rules/cpp-style.md

## Output
Subspecs:

Artifacts:
include/ir_lasertag/codec/ir_receiver.hpp
src/codec/ir_receiver.cpp

## Spec

### Role

`IrReceiver` consumes symbol frames from an `IrRxSource` backend (see spec/systems/codec/rx-source.md) and decodes them per the configured `ProtocolConfig`. Non-copyable; destructor calls `deinit()`. The receiver does not own the source — the caller creates, initializes, and outlives it.

### Lifecycle & State

| Method | Effect |
|---|---|
| `init(IrRxSource* source)` | store source, create the source→task symbol queue, register the receiver's frame callback with the source; `ESP_ERR_INVALID_ARG` if source null, `ESP_ERR_INVALID_STATE` if already initialized |
| `deinit()` | stop if receiving, delete queue, clear state |
| `is_initialized()` / `is_receiving()` | state queries |
| `start()` / `stop()` | drain stale queue entries then delegate to `source->start()/stop()`; toggles `receiving_` |

### Protocol

`set_protocol(const ProtocolConfig&)` validates and stores the config, then computes `compute_max_symbol_duration_us(config)` and pushes it to the source via `set_max_symbol_duration_us`. `get_protocol()` returns the current config. A protocol must be set before decoded `receive`.

`compute_max_symbol_duration_us(const ProtocolConfig&)` (static) = `1.5 ×` the longest individual pulse across header mark/space, all four data half-bits, and footer mark; clamped to `[1000, UINT16_MAX]` µs. This is the end-of-frame threshold the source uses for frame separation.

### Frame Delivery Path

The source invokes the receiver's frame callback (may run in ISR context). The callback copies up to `kSymbolBufferSize` (64) symbols into an internal buffer and queues the symbol count on a FreeRTOS queue (`xQueueSendFromISR`). Decoding runs later in task context inside `receive`/`receive_raw`, off the ISR.

### Decoded Mode

| Method | Behavior |
|---|---|
| `receive(IrMessage*, timeout_ms)` | block up to timeout for a frame, decode it; loops past invalid/noise frames until a valid decode or timeout; `ESP_ERR_TIMEOUT`, `ESP_ERR_INVALID_STATE` if in raw mode or no protocol, `ESP_ERR_INVALID_ARG` on null |
| `try_receive(IrMessage*)` | `receive` with `timeout_ms = 0` (poll) |

Decoded mode requires a protocol set and raw mode disabled.

### Two Decode Paths

`decode_symbols` rejects frames shorter than 2 symbols, then dispatches by `protocol_.is_manchester()`:

- General path (pulse-width / pulse-distance and any non-Manchester config): optionally matches a header symbol, then matches each RMT symbol's two phases against `{one_half1, one_half2}` then `{zero_half1, zero_half2}`. Detects end-of-message (zero symbol) and footer (footer mark with empty second phase). Includes a last-bit fallback: when a symbol's second half matches neither pattern (trailing space merged into the message gap, or `duration1 == 0` at frame end), the bit is decided from the first half alone where the mark durations distinguish zero from one.
- Manchester-optimized path (`decode_manchester`): splits each RMT symbol into two half-symbols and consumes durations in half-bit-sized increments. Adjacent same-level half-bits merge into double-length durations; only the needed portion is consumed and the remainder carries forward. The first half-bit's level decides the bit value (`one_first_level` vs `zero_first_level`). Handles a truncated trailing space on the last bit (RMT reports duration 0 at end-of-frame).

Both paths apply `mark_bias_us` compensation and `tolerance_percent` when matching timings: bias is subtracted from received marks and added to received spaces, then the value must fall within `±tolerance` of the expected duration (`timing_matches`). Validity = `bit_count == protocol.bit_count` when `bit_count > 0`, else `bit_count > 0`.

### Raw Mode

| Method | Behavior |
|---|---|
| `enable_raw_mode()` / `disable_raw_mode()` | toggle raw mode flag |
| `is_raw_mode()` | query |
| `receive_raw(timings, max_count, received_count*, timeout_ms)` | block up to timeout for a frame, expand each symbol into up to two `RawTiming` entries (phase0, phase1 if non-zero), report `received_count`; `ESP_ERR_INVALID_STATE` if not in raw mode, `ESP_ERR_INVALID_ARG` on null, `ESP_ERR_TIMEOUT` on timeout |

There is no `RawReceiveConfig` struct; raw mode is a flag plus `receive_raw`. Decoded and raw modes are mutually exclusive: `receive` fails in raw mode and `receive_raw` fails outside it.
