# BC Protocol

**Version**: 1.0
**Purpose**: Pulse-width-encoded IR send/receive wrapper for BC 16-bit tagger IDs broadcast as a single packet with no repeat.
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
include/ir_lasertag/protocols/bc/bc.hpp
src/protocols/bc/bc.cpp
include/ir_lasertag/protocols/bc/README.md

## Spec

### Overview

BC = commercial laser tag system. A tagger broadcasts only its 16-bit ID; all game logic is internal to the tagger. Each shot = a single 16-bit pulse-width packet with no paired-packet repeat. Unlike IRT (spec/features/protocols/irt.md), there is no pair validation.

This feature wraps the codec layer (spec/systems/codec/ir-transmitter.md, spec/systems/codec/ir-receiver.md), encoding/decoding one 16-bit value per packet. Class = `BcGun` in `ir_lasertag::protocols`.

### IR Encoding

`kBcConfig` = authoritative `ProtocolConfig` (half-bit model: spec/systems/codec/protocol-config.md). `make_bc_config()` returns a copy.

| Parameter | Value |
|---|---|
| Encoding | Pulse-width |
| Carrier | 38 kHz |
| Carrier duty | 0.33 |
| Header mark | 1900 µs |
| Header space | 450 µs |
| Zero | MARK(500 µs) → SPACE(450 µs) |
| One | MARK(1000 µs) → SPACE(450 µs) |
| Bit space | 450 µs (shared by 0 and 1) |
| Message size | 16 bits |
| Bit order | MSB first |
| Footer | none |
| Repeat | none (single packet) |
| Tolerance | 25% |

### API

API surface from bc.hpp = authoritative. `init` applies `config` to both TX/RX (spec/systems/codec/ir-transmitter.md, spec/systems/codec/ir-receiver.md). Caller retains TX/RX ownership; either may be null; both null → `ESP_ERR_INVALID_ARG`.

| Member | Behavior |
|---|---|
| `init(transmitter, receiver, config)` | Bind TX/RX and apply config; `ESP_OK`/`ESP_ERR_INVALID_ARG`/`set_protocol` error |
| `deinit()` | Stop active receive; release references; does not deinit underlying TX/RX |
| `send(gun_id, timeout_ms=1000)` | Blocking; single 16-bit MSB-first packet; `ESP_ERR_INVALID_STATE` if no TX |
| `send_async(gun_id)` | Non-blocking; encodes into persistent internal buffer; returns after RMT starts |
| `start_receive()` | Start receiver; `ESP_ERR_INVALID_STATE` if no RX |
| `stop_receive()` | Stop receiver; `ESP_ERR_INVALID_STATE` if not receiving |
| `is_receiving()` | true between start/stop |
| `try_receive(uint16_t* gun_id)` | Non-blocking poll; true if a valid gun ID dequeued |
| `receive(uint16_t* gun_id, timeout_ms)` | Blocking until gun ID or timeout; `ESP_ERR_TIMEOUT`, `ESP_ERR_INVALID_STATE` if not receiving |
| `is_initialized()` | true after successful `init` |
| `kBcConfig` | constexpr default `ProtocolConfig` |
| `make_bc_config()` | returns copy of `kBcConfig` |

Receive is poll/queue-based in task context (`try_receive`/`receive`); no callback registration and no pair validation. Decode accepts only `valid==true`, `bit_count==16`, MSB-first.

### Field-Test Status

BC is fully implemented (wrapper class, config, README) but is not yet validated against real hardware. It is therefore intentionally absent from the top-level `README.md` "Supported Protocols" table, which lists only field-tested protocols (see spec/rules/protocol-authoring.md).

### README Notes

The Output Artifact README (include/ir_lasertag/protocols/bc/README.md) API section matches the header (lists init/deinit/send/send_async/start_receive/stop_receive/try_receive/receive plus kBcConfig and make_bc_config()). No receive-callback drift. README omits `is_initialized` and `is_receiving`, which are present in the header.
