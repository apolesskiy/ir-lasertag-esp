# Arclite TagCode Protocol

**Version**: 1.0
**Purpose**: Manchester-encoded IR send/receive wrapper for Arclite 16-bit Tag Codes used for collision detection between game props.
**Status**: Built(1.0)

**Uses**:
.speck/rules/feature-system.md
spec/systems/codec/ir-transmitter.md
spec/systems/codec/ir-receiver.md
spec/systems/codec/protocol-config.md
spec/rules/protocol-authoring.md

## Output
Subspecs:

Artifacts:
include/ir_lasertag/protocols/arclite/arclite.hpp
src/protocols/arclite/arclite.cpp
include/ir_lasertag/protocols/arclite/README.md

## Spec

### Overview

Arclite = laser tag platform using IR for collision detection between props. Each emitter capability registered by a prop is issued a unique 16-bit Tag Code for a game session. A sensor receiving a Tag Code → collision event forwarded to the game engine. In networked sessions the server issues Tag Codes; collision validation runs client-side (player kit). Props are not expected to process Tag Codes beyond optional deduplication.

Tag Code space: all 16-bit values valid. By convention `0x0000` = "unset" → game engine records a warning. The wrapper enforces no validation on the value; semantics are a consumer concern.

This feature is a thin protocol wrapper over the codec layer (spec/systems/codec/ir-transmitter.md, spec/systems/codec/ir-receiver.md). It owns no game logic — it encodes/decodes one 16-bit value per packet. Class = `ArcliteTagCode` in `ir_lasertag::protocols`.

### IR Encoding

`kArcliteConfig` is the authoritative `ProtocolConfig` (see spec/systems/codec/protocol-config.md for the half-bit timing model). `make_arclite_config()` returns a copy.

| Parameter | Value |
|---|---|
| Encoding | Manchester |
| Carrier | 38 kHz |
| Carrier duty | 0.33 |
| Header mark | 520 µs |
| Header space | 520 µs |
| Half-bit | 300 µs |
| Zero | MARK(300 µs) → SPACE(300 µs) |
| One | SPACE(300 µs) → MARK(300 µs) |
| Message size | 16 bits |
| Bit order | MSB first |
| Footer | none |
| Min interval | 2500 µs |
| Repeat | none (single send) |
| Tolerance | 25% |

`is_manchester()` = true (zero/one first-half levels differ). The 2500 µs minimum interval is a consumer-side pacing constraint (constant `kArcliteMessageGapUs`); it is not encoded into the packet.

### API

API surface from arclite.hpp = authoritative. `init` applies `config` to both transmitter and receiver via their `set_protocol` (spec/systems/codec/ir-transmitter.md, spec/systems/codec/ir-receiver.md). Caller retains ownership of TX/RX; either may be null when only one direction is needed; both null → `ESP_ERR_INVALID_ARG`.

| Method | Behavior |
|---|---|
| `init(transmitter, receiver, config)` | Bind TX/RX and apply config; returns `ESP_OK`, `ESP_ERR_INVALID_ARG`, or `set_protocol` error |
| `deinit()` | Stop active receive; release references; does not deinit underlying TX/RX |
| `send(tag_code, timeout_ms=1000)` | Blocking; encode 16-bit MSB-first, single packet; `ESP_ERR_INVALID_STATE` if no TX |
| `send_async(tag_code)` | Non-blocking; encodes into persistent internal buffer; returns after RMT starts |
| `start_receive()` | Start receiver; `ESP_ERR_INVALID_STATE` if no RX |
| `stop_receive()` | Stop receiver; `ESP_ERR_INVALID_STATE` if not receiving |
| `is_receiving()` | true between `start_receive` and `stop_receive` |
| `try_receive(uint16_t* tag_code)` | Non-blocking poll; true if a valid tag code dequeued |
| `receive(uint16_t* tag_code, timeout_ms)` | Blocking until tag code or timeout; `ESP_ERR_TIMEOUT`, `ESP_ERR_INVALID_STATE` if not receiving |
| `is_initialized()` | true after successful `init` |
| `kArcliteConfig` | constexpr default `ProtocolConfig` |
| `make_arclite_config()` | returns copy of `kArcliteConfig` |

Receive is poll/queue-based: the consumer calls `try_receive` (non-blocking) or `receive` (blocking with timeout) from task context. There is no callback registration. Decode accepts only messages with `valid==true` and `bit_count==16`, reconstructing the 16-bit value MSB-first from `IrMessage.data`.

### README

The Output Artifact README (include/ir_lasertag/protocols/arclite/README.md) API section is kept in sync with the header, which is the authoritative API source: poll/queue-based `start_receive()` + `try_receive`/`receive` (task context, no callback), plus `send_async`, `is_initialized`, `is_receiving`, and the `kArcliteConfig`/`make_arclite_config()` config exports. (An earlier README revision documented a non-existent `start_receive(callback)`; corrected to match the header.)
