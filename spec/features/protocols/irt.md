# IRT Protocol
**Version**: 2.0
**Purpose**: Pulse-width-encoded IR send/receive wrapper for IRT 16-bit messages, supporting both gun (paired) and spawner (single-packet) roles with receive-side role detection.
**Status**: Built(2.0)

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

IRT = commercial laser tag system. Messages = 16-bit. Two device roles, distinguished by transmission pattern (not by the ID value):

- Guns: transmit a 16-bit gun ID twice with a ~2000 µs gap (the repeat is REQUIRED; a shot is not received without it). Format `0xXXYY` — `XX` (high byte) = stable unknown value, likely a checksum; `YY` (low byte) = assigned player/CCM ID. Unassigned (off-CCM) gun = `0xF0F0` (`kUnassignedId`).
- Spawners: transmit a static 16-bit ID once (no repeat) every ~50 ms (continuous spawn or while button held).

The wrapper class is `Irt` (in `ir_lasertag::protocols`; role-neutral — it sends and receives both roles). It wraps the codec layer (spec/systems/codec/ir-transmitter.md, spec/systems/codec/ir-receiver.md). Receive returns a structured result `IrtMessage{ id, role }` (`role` = `IrtRole::kGun` or `IrtRole::kSpawner`).

### IR Encoding

`kIrtConfig` = authoritative `ProtocolConfig` (half-bit model: spec/systems/codec/protocol-config.md). `make_irt_config()` returns a copy. Encoding is identical for both roles; only the repeat pattern differs.

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
| Footer space | 2000 µs (`kIrtMessageGapUs` / `kRepeatGapUs`, the gun inter-packet gap) |
| Tolerance | 25% |

### Transmission

| Method | Behavior |
|---|---|
| `send_gun(id, timeout_ms)` | Gun shot: transmit `id` twice (`IrTransmitter::send` with loop count 2); the config `footer_space_us` (2000 µs) supplies the inter-packet gap. The RMT peripheral repeats the packet atomically. |
| `send_spawner(id, timeout_ms)` | Spawner pulse: transmit `id` once (loop count 0 = single shot). The caller repeats at its own cadence (e.g. ~50 ms) if continuous spawn is desired. |

### Reception & Role Detection

Receive is poll/queue-based in task context. Decoded 16-bit packets are classified by arrival pattern into gun / spawner / invalid:

- Two identical packets within the pair window → **Gun** (`IrtRole::kGun`), returned once.
- A packet with no matching pair within the pair window → **Spawner** (`IrtRole::kSpawner`), returned once the window elapses (or when the next, non-pairing packet arrives).
- A non-matching packet within the pair window → the older candidate is **discarded** and the new packet becomes the candidate (sliding ring-buffer behavior; nothing returned). Example: `X Y Y` → X is replaced by Y, then the second Y pairs → `Gun(Y)`.

State machine: the most recent decoded packet is held as a candidate with its arrival timestamp. On each decoded packet, `classify()` (pure, see API) maps `(has_candidate, candidate_id, incoming_id, delta_us, pair_window_us)` to an `IrtPairResult`: `kHold` / `kReplace` (store the packet as the candidate), `kGun` (emit gun, clear), `kSpawner` (emit the aged candidate, then hold the new packet). A lone candidate is flushed as a spawner when its age exceeds the pair window even if no further packet arrives.

`receive()` is event-driven: it blocks on the codec receive queue with a wait bounded by the candidate's remaining window time, so a lone spawner is flushed without busy-polling. It does NOT use `vTaskDelay` for inter-packet timing — sub-millisecond IR pulse timing is the codec/RMT's responsibility; the wrapper handles only the ~30 ms window (tick resolution is adequate).

#### Pair-Window Timing

The pair window separates a gun's two packets from a spawner's repeats. Values are receive-side inter-arrival deltas (frame-completion to frame-completion), which include packet duration:

- Gun pair inter-arrival ≈ packet duration (~16 ms for 16 bits) + 2000 µs gap ≈ **~18 ms**.
- Spawner repeat interval ≈ **~50 ms**.

`kPairWindowUs` = 30000 µs (30 ms) sits between them: gun pairs (≤ ~18 ms) classify as a pair; spawner repeats (~50 ms apart) each age out as singles. The window is timing- and scheduling-dependent; the `classify()` decision logic is host-tested with synthetic deltas, while end-to-end role detection requires on-device validation (spec/rules/testing.md).

### API

API surface from irt.hpp = authoritative. `init` applies `config` to both TX/RX. Caller retains TX/RX ownership; either may be null; both null → `ESP_ERR_INVALID_ARG`.

Types: `enum class IrtRole { kGun, kSpawner }`; `struct IrtMessage { uint16_t id; IrtRole role; }`; `enum class IrtPairResult { kHold, kGun, kSpawner, kReplace }` (classification outcome).

| Member | Behavior |
|---|---|
| `init(transmitter, receiver, config)` | Bind TX/RX and apply config; `ESP_OK`/`ESP_ERR_INVALID_ARG`/`set_protocol` error |
| `deinit()` | Stop active receive; release references; does not deinit underlying TX/RX |
| `send_gun(id, timeout_ms=1000)` | Blocking; transmit gun shot (pair, loop count 2); `ESP_ERR_INVALID_STATE` if no TX |
| `send_spawner(id, timeout_ms=1000)` | Blocking; transmit one spawner packet (single shot); `ESP_ERR_INVALID_STATE` if no TX |
| `start_receive()` | Start receiver; reset candidate state; `ESP_ERR_INVALID_STATE` if no RX |
| `stop_receive()` | Stop receiver; `ESP_ERR_INVALID_STATE` if not receiving |
| `is_receiving()` | true between start/stop |
| `try_receive(IrtMessage* msg)` | Non-blocking; drains queue, runs role detection + flush; true on a classified gun or spawner |
| `receive(IrtMessage* msg, timeout_ms)` | Blocking (polls at a short interval so spawner flush can fire) until a classified message or timeout; `ESP_ERR_TIMEOUT`, `ESP_ERR_INVALID_STATE` if not receiving |
| `is_initialized()` | true after successful `init` |
| `classify(has_candidate, candidate_id, incoming_id, delta_us, pair_window_us)` (static) | Pure classifier → `IrtPairResult`; no candidate→`kHold`; within window + match→`kGun`; within window + mismatch→`kReplace` (slide); beyond window→`kSpawner` |
| `get_ccm_id(uint16_t)` (static) | Low byte = CCM/player ID (gun semantics) |
| `is_unassigned(uint16_t)` (static) | true if value == `0xF0F0` |
| `kIrtConfig` | constexpr default `ProtocolConfig` |
| `make_irt_config()` | returns copy of `kIrtConfig` |
| `kRepeatGapUs` / `kUnassignedId` / `kPairWindowUs` | constants (2000 µs; 0xF0F0; 30000 µs) |

Decode accepts only `valid==true`, `bit_count==16`, MSB-first. No callback registration. `IrtMessage` (id + role) establishes the structured-receive result pattern for protocol wrappers (vs. a bare ID).

### README

The Output Artifact README (include/ir_lasertag/protocols/irt/README.md) API section is kept in sync with the header (authoritative): the `Irt` class, `send_gun`/`send_spawner`, the `IrtMessage`/`IrtRole` receive result, poll/queue `start_receive()` + `try_receive`/`receive`, the static `get_ccm_id`/`is_unassigned`, and the `kIrtConfig`/`make_irt_config()` exports.
