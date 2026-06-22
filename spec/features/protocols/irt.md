# IRT Protocol

**Version**: 1.0
**Purpose**: Pulse-width-encoded IR send/receive wrapper for IRT 16-bit gun IDs, including the required paired-packet transmission and receive-side pair validation.
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
include/ir_lasertag/protocols/irt/irt.hpp
src/protocols/irt/irt.cpp
include/ir_lasertag/protocols/irt/README.md

## Spec

### Overview

IRT = commercial laser tag system. Messages = 16-bit. Two device roles with differing behavior and ID meaning:

- Guns: transmit a 16-bit gun ID. Format `0xXXYY` — `XX` (high byte) = stable unknown value, likely a checksum; `YY` (low byte) = assigned player/CCM ID. Unassigned (off-CCM) gun = `0xF0F0` (`kUnassignedId`).
- Spawners: transmit a static 16-bit ID once every 50 ms (continuous spawn or while button held). Spawner behavior is documented for reference; the wrapper class models the gun role.

This feature wraps the codec layer (spec/systems/codec/ir-transmitter.md, spec/systems/codec/ir-receiver.md) with IRT semantics: paired transmission and receive-side pair validation. Class = `IrtGun` in `ir_lasertag::protocols`.

### IR Encoding

`kIrtConfig` = authoritative `ProtocolConfig` (half-bit model: spec/systems/codec/protocol-config.md). `make_irt_config()` returns a copy.

| Parameter | Value |
|---|---|
| Encoding | Pulse-width |
| Carrier | 38 kHz |
| Carrier duty | 0.33 |
| Header mark | 500 µs |
| Header space | 500 µs |
| Zero | MARK(350 µs) → SPACE(450 µs) |
| One | MARK(700 µs) → SPACE(450 µs) |
| Bit space | 450 µs (shared by 0 and 1) |
| Message size | 16 bits |
| Bit order | MSB first |
| Footer space | 2000 µs (`kRepeatGapUs`, used as inter-packet gap) |
| Tolerance | 25% |

### Packet Repetition

| Role | Repetition |
|---|---|
| Gun | ID transmitted twice with a 2000 µs gap. REQUIRED: a shot is not received if the repeat is absent. |
| Spawner | Static ID transmitted once every 50 ms. |

`send` emits the required pair internally: it issues the packet to the transmitter with loop count 2, so the RMT peripheral repeats it atomically and the config `footer_space_us` (2000 µs) supplies the inter-packet gap. Constants: `kRepeatGapUs = 2000`, `kPairTimeoutUs = 50000` (max measured delta between the two packets during receive validation).

### API

API surface from irt.hpp = authoritative. `init` applies `config` to both TX/RX (spec/systems/codec/ir-transmitter.md, spec/systems/codec/ir-receiver.md). Caller retains TX/RX ownership; either may be null; both null → `ESP_ERR_INVALID_ARG`.

| Member | Behavior |
|---|---|
| `init(transmitter, receiver, config)` | Bind TX/RX and apply config; `ESP_OK`/`ESP_ERR_INVALID_ARG`/`set_protocol` error |
| `deinit()` | Stop active receive; release references; does not deinit underlying TX/RX |
| `send(gun_id, timeout_ms=1000)` | Blocking; transmits the required pair internally (loop count 2); `ESP_ERR_INVALID_STATE` if no TX |
| `start_receive()` | Start receiver; reset pair state; `ESP_ERR_INVALID_STATE` if no RX |
| `stop_receive()` | Stop receiver; `ESP_ERR_INVALID_STATE` if not receiving |
| `is_receiving()` | true between start/stop |
| `try_receive(uint16_t* gun_id)` | Non-blocking; drains queue, runs pair validation; true only on a validated pair |
| `receive(uint16_t* gun_id, timeout_ms)` | Blocking until a validated pair or timeout; `ESP_ERR_TIMEOUT`, `ESP_ERR_INVALID_STATE` if not receiving |
| `is_initialized()` | true after successful `init` |
| `get_ccm_id(uint16_t)` (static) | Low byte = CCM/player ID |
| `is_unassigned(uint16_t)` (static) | true if value == `0xF0F0` |
| `validate_pair(uint16_t)` (private) | Pair-matching state machine; first matching packet stored, second within `kPairTimeoutUs` completes the pair |
| `kIrtConfig` | constexpr default `ProtocolConfig` |
| `make_irt_config()` | returns copy of `kIrtConfig` |

Receive is poll/queue-based and runs in task context: `try_receive` (non-blocking) and `receive` (blocking with timeout). No callback registration. Pair validation: a decoded ID is held as a candidate; the same ID seen again within `kPairTimeoutUs` (50000 µs) completes a valid pair and returns it (state reset to avoid triple-trigger). Decode accepts only `valid==true`, `bit_count==16`, MSB-first.

### README

The Output Artifact README (include/ir_lasertag/protocols/irt/README.md) API section is kept in sync with the header, which is the authoritative API source: poll/queue-based `start_receive()` + `try_receive`/`receive` (task context, no callback), plus `deinit`, `is_initialized`, `is_receiving`, the static `get_ccm_id`/`is_unassigned`, and the `kIrtConfig`/`make_irt_config()` config exports. (An earlier README revision documented a non-existent `start_receive(callback)`; corrected to match the header.)
