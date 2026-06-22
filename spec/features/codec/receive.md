# Receive

**Version**: 1.0
**Purpose**: Foundational user-facing capability to receive and decode a configured IR protocol, or capture raw timings of unknown protocols.
**Status**: Built(1.0)

**Uses**:
.speck/rules/feature-system.md
spec/systems/codec/protocol-config.md
spec/systems/codec/ir-receiver.md
spec/systems/codec/rx-source.md

## Output
Subspecs:

Artifacts:

## Spec

### Scope

Receive = consumer-facing RX capability of the library. A consumer configures a protocol (`ProtocolConfig`, half-bit model — see `spec/systems/codec/protocol-config.md`), supplies a symbol source, and reads decoded `IrMessage`s. For unknown/unsupported protocols, the consumer switches to raw mode and captures `RawTiming` arrays. Mechanism (symbol decoding, tolerance/mark-bias logic, RMT backend) lives in the Uses systems.

### Capabilities

| Capability | API (`ir_lasertag::codec::IrReceiver`) | Behavior |
|---|---|---|
| Configure protocol | `set_protocol(ProtocolConfig)` | Adopt protocol; compute + push end-of-frame threshold to source |
| Start/stop | `start()` / `stop()` | Enable/disable symbol delivery from source |
| Blocking decode | `receive(IrMessage*, timeout_ms)` | Wait for a decoded message or timeout (`ESP_ERR_TIMEOUT`) |
| Non-blocking decode | `try_receive(IrMessage*)` | Poll (= `receive` with timeout 0) |
| Raw capture | `receive_raw(timings, max_count, received_count*, timeout_ms)` | Get raw mark/space timings for analysis |
| Raw mode toggle | `enable_raw_mode()` / `disable_raw_mode()` / `is_raw_mode()` | Switch between decoded and raw; mutually exclusive |
| End-of-frame | `set_max_symbol_duration_us(...)` (on source) / `compute_max_symbol_duration_us(config)` (static) | Set frame-end threshold; helper derives 1.5× longest pulse |

### Pluggable RX Source

- Receiver does not own the source. Consumer constructs/initializes an `IrRxSource` and passes it to `init(IrRxSource*)`; source must outlive the receiver until `deinit`.
- Default backend = `RmtRxSource` (RMT peripheral; see `spec/systems/codec/rx-source.md`).
- Custom backends implement the `IrRxSource` interface (`set_on_frame_callback`/`start`/`stop`/`set_max_symbol_duration_us`), e.g. a non-RMT receiver, without changing decode logic.

### User-Facing Contract

- Lifecycle: source `init` → receiver `init(source)` → `set_protocol` → `start` → `receive`/`try_receive`. `stop`/`deinit` release resources.
- Decoded receive returns an `IrMessage` (`data`, `bit_count`, `is_repeat`, `valid`); `valid` true only when a frame decodes to the configured structure.
- Reliability: decoding accepts timings within `tolerance_percent` of expected and applies `mark_bias_us` compensation for IR-demodulator mark stretching/space shrinking (see `spec/systems/codec/protocol-config.md`, `spec/systems/codec/ir-receiver.md`).
- End-of-frame: a single mark or space longer than the max-symbol-duration threshold terminates a frame; consumer sets it directly or via the static helper.
- Decoded and raw modes are mutually exclusive: `receive`/`try_receive` valid only in decoded mode (else `ESP_ERR_INVALID_STATE`); `receive_raw` valid only in raw mode (else `ESP_ERR_INVALID_STATE`).

### Acceptance

- Configure protocol, `start`, then transmit a matching frame → `receive` yields an `IrMessage` with `valid == true` and expected `bit_count`/`data`.
- `try_receive` with no frame queued → `ESP_ERR_TIMEOUT`.
- Enable raw mode, capture an unknown signal → `receive_raw` returns the observed mark/space timing sequence.
- Decoded receive while in raw mode (or raw receive while decoded) → `ESP_ERR_INVALID_STATE`.
- Custom `IrRxSource` passed to `init` → decoding works without code changes to the receiver.

### Mechanism (reference, not duplicated)

- Symbol decode paths (general + Manchester-optimized), tolerance + mark-bias matching, frame queueing: `spec/systems/codec/ir-receiver.md`.
- Source interface, default RMT backend, symbol buffering, signal-range translation: `spec/systems/codec/rx-source.md`.
- Config structure and timing semantics: `spec/systems/codec/protocol-config.md`.
