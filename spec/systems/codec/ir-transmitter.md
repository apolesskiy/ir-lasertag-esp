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
| `init(const rmt_tx_channel_config_t&)` | create RMT TX channel from the caller's config (GPIO + RMT params), register TX-done callback, create copy encoder, enable channel; the config's `trans_queue_depth` is the send queue's depth (0 counts as 1); `ESP_ERR_INVALID_STATE` if already initialized, `ESP_ERR_INVALID_ARG` if the depth is over `kMaxQueueDepth` (4) |
| `deinit()` | disable channel, destroy encoders, delete channel, clear state |
| `is_initialized()` | channel ready |
| `is_busy()` | at least one frame is queued or in flight (counted up on send, down by the TX-done ISR) |

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
| `send_async(data, bit_count, loop_count=0)` | copy the data into the next queue slot and queue an RMT transmit (`queue_nonblocking`), return immediately, never block; `ESP_ERR_INVALID_STATE` if not initialized / no protocol / the queue is full; `ESP_ERR_INVALID_ARG` on null data, zero bits or more than `IrMessage::kMaxDataBytes` bytes |
| `wait_done(timeout_ms)` | block on `rmt_tx_wait_all_done`; `ESP_ERR_TIMEOUT` on timeout |
| `send(data, bit_count, timeout_ms=1000, loop_count=0)` | `send_async` then `wait_done`; blocking |
| `send_raw(timings, count, timeout_ms=1000)` | transmit `RawTiming[]` via the copy encoder, one symbol per timing, bypassing protocol encoding; blocking; the symbol buffer is freed only after the transaction is done (on a timeout, after the channel is disabled); requires init only (no protocol) |

`bit_count` is bits; the byte count transmitted = `ceil(bit_count / 8)`.

### Loop Count

`loop_count` maps to the RMT `rmt_transmit_config_t.loop_count`: `0` = single shot, `N > 0` = the packet repeats N times atomically in hardware (no per-repeat CPU involvement). Used for protocols that emit a packet multiple times in one burst.

### Concurrency

`set_done_callback(callback, ctx)` registers an optional callback that the TX-done ISR calls after each frame, once the pending count is lowered (for a caller that keeps its own count of frames in flight). It must be ISR-safe, and in IRAM under `CONFIG_IR_LASERTAG_ISR_IRAM_SAFE`.

`is_busy()` is the async completion signal: a pending count raised by `send_async` and lowered by the IRAM TX-done callback, under a spinlock. Frames go out in order, one in flight and up to depth − 1 queued; a send beyond the depth is refused at once. Each pending frame owns a slot of `IrMessage::kMaxDataBytes` bytes: frames complete in order, so with fewer than depth frames pending, the next slot's frame is done and the slot is free. The driver encodes a queued frame in its TX ISR, after `send_async` has returned, which is why the data is copied. The driver resets the encoder at the start of each transaction, so `send_async` does not. `send_async` is called from one task at a time. `send_raw` waits internally and does not use the pending count; it always blocks until done.

### Flash Busy

With `CONFIG_IR_LASERTAG_ISR_IRAM_SAFE` (Kconfig, default off) the encoder's encode and reset functions are placed in IRAM and the option selects ESP-IDF's `RMT_TX_ISR_CACHE_SAFE`, so a queued frame is encoded and sent while flash is being written. Encoder state is allocated in internal RAM.
