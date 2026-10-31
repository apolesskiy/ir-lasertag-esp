# Transmit

**Version**: 1.0
**Purpose**: Foundational user-facing capability to configure any IR protocol and transmit a message (or raw timings) over RMT.
**Status**: Built(1.0)

**Uses**:
.speck/rules/feature-system.md
spec/systems/codec/protocol-config.md
spec/systems/codec/ir-encoder.md
spec/systems/codec/ir-transmitter.md

## Output
Subspecs:

Artifacts:

## Spec

### Scope

Transmit = consumer-facing TX capability of the library. A consumer describes a protocol once via a `ProtocolConfig` (half-bit model — see `spec/systems/codec/protocol-config.md`), then transmits packed bit buffers. The library encodes bits into RMT symbols and emits a modulated IR signal. Mechanism (encoding, RMT driving) lives in the Uses systems; this spec states the capability and contract.

### Capabilities

| Capability | API (`ir_lasertag::codec::IrTransmitter`) | Behavior |
|---|---|---|
| Configure protocol | `set_protocol(ProtocolConfig)` | Adopt/rebuild encoder for protocol; runtime-switchable without re-init of channel |
| Blocking send | `send(data, bit_count, timeout_ms=1000, loop_count=0)` | Encode + transmit; block until done or timeout (`ESP_ERR_TIMEOUT`) |
| Non-blocking send | `send_async(data, bit_count, loop_count=0)` | Start transmission, return immediately |
| Await async | `wait_done(timeout_ms)` | Block until in-flight transmission completes or times out |
| Poll | `is_busy()` | True while any frame is queued or in flight |
| Raw send | `send_raw(timings, count, timeout_ms=1000)` | Emit an arbitrary `RawTiming` sequence verbatim; bypasses bit encoding |
| Hardware repeat | `loop_count` arg on `send`/`send_async` | RMT repeats the packet N times in hardware (0 = single shot) |
| Carrier duty | `set_carrier_duty(duty)` | Adjust carrier duty (0.0–1.0) without full protocol re-setup |

### User-Facing Contract

- Lifecycle: `init(rmt_tx_channel_config_t)` → `set_protocol` → send. `deinit` releases RMT resources. Channel resolution drives µs-granularity timing (consumer sets `resolution_hz = 1000000`).
- Encoded send: given a configured protocol and a packed bit buffer (MSB- or LSB-first per `bit_order`), the emitted IR carries header, the `bit_count` data bits each as two half-bits, and footer, modulated at `carrier_freq_hz`/`carrier_duty`.
- Raw send: emitted mark/space timings equal the supplied `RawTiming` array exactly, in order; no header/footer/encoding applied.
- Repeat: `loop_count = N` (N > 0) emits N back-to-back copies of the packet via RMT hardware loop.
- Blocking vs async: `send` returns only after the signal is fully emitted (or timeout); `send_async` queues the frame (up to the channel's `trans_queue_depth`, refusing beyond it without blocking), returns immediately, and the consumer synchronizes via `is_busy`/`wait_done`.

### Acceptance

- Set a valid config, call `send` with a bit buffer → corresponding IR signal emitted, call returns `ESP_OK`.
- `send_async` then `wait_done` → returns `ESP_OK` after emission; `is_busy` true between start and completion.
- `send_raw` with a known timing array → IR output reproduces those exact timings.
- `loop_count = N` → N packet copies emitted.
- `set_carrier_duty` mid-session → subsequent transmissions use the new duty.

### Mechanism (reference, not duplicated)

- Bit→symbol encoding, header/footer assembly, half-bit model: `spec/systems/codec/ir-encoder.md`.
- Config structure, validation, Manchester/pulse-width/pulse-distance modeling: `spec/systems/codec/protocol-config.md`.
- RMT channel lifecycle, copy-encoder raw path, busy/done signaling, carrier control: `spec/systems/codec/ir-transmitter.md`.
