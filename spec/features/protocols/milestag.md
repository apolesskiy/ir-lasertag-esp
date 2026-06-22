# MilesTag Protocol

**Version**: 1.0
**Purpose**: Config-only (header-only) MilesTag timing constants and default `ProtocolConfig` for the standardized 14-bit DIY laser-tag format; consumers apply it to a codec transmitter/receiver and encode/decode the field layout themselves.
**Status**: Built(1.0)

**Uses**:
.speck/rules/feature-system.md
spec/systems/codec/protocol-config.md
spec/rules/protocol-authoring.md

## Output
Subspecs:

Artifacts:
include/ir_lasertag/protocols/milestag/milestag.hpp
include/ir_lasertag/protocols/milestag/README.md

## Spec

### Overview

MilesTag = one of the most widely-used laser tag protocols, originally for DIY systems and broadly implemented for compatibility. It defines a standardized 14-bit message carrying player ID, team, and damage over IR.

This feature is CONFIG-ONLY: the header provides timing constants plus the constexpr default `kMilestagConfig` (`ProtocolConfig`, see spec/systems/codec/protocol-config.md). There is NO wrapper class, NO `.cpp`, and the unit is NOT in the CMake SRCS (header-only). It contrasts with Arclite/IRT/BC, which provide wrapper classes.

### IR Encoding

`kMilestagConfig` = authoritative `ProtocolConfig` (half-bit/pulse-distance model: spec/systems/codec/protocol-config.md).

| Parameter | Value |
|---|---|
| Encoding | Pulse-distance |
| Carrier | 38 kHz |
| Carrier duty | 0.33 |
| Header mark | 2400 µs |
| Header space | 600 µs |
| Bit mark (0 and 1) | 600 µs |
| Zero | MARK(600 µs) → SPACE(600 µs) |
| One | MARK(600 µs) → SPACE(1200 µs) |
| Footer mark | 600 µs |
| Message size | 14 bits |
| Bit order | MSB first |
| Message gap | ≥ 10000 µs |
| Tolerance | 25% |

### Message Structure

The 14-bit message decomposes as:

| Bits | Width | Field | Range |
|---|---|---|---|
| 13–7 | 7 | Player ID | 0–127 |
| 6–5 | 2 | Team | 0–3 |
| 4–0 | 5 | Damage | 0–31 |

### Usage

The consumer applies `kMilestagConfig` to a codec transmitter and/or receiver (via `set_protocol`) and encodes/decodes the 14-bit field layout itself. The library does not pack or unpack the player/team/damage fields. Exported names: timing constants (`kMilestagHeaderMarkUs`, `kMilestagHeaderSpaceUs`, `kMilestagBitMarkUs`, `kMilestagZeroSpaceUs`, `kMilestagOneSpaceUs`, `kMilestagFooterMarkUs`, `kMilestagMessageGapUs`, `kMilestagBitCount`, `kMilestagCarrierHz`) and `kMilestagConfig`.

### Status & Decisions

1. Structured-field wrapper — PLANNED, not yet implemented. The protocol is currently config-only; consumers pack/unpack the 14-bit player/team/damage fields themselves (the Message Structure table documents the bit layout). A wrapper class providing structured field encode/decode is planned; the top-level README's "structured field access" describes that intended capability.

2. Carrier frequency — MilesTag does not mandate a carrier frequency; both 38 kHz and 56 kHz are used in practice. `kMilestagConfig` currently fixes 38 kHz (matching this library's other protocols and a typical 38 kHz receiver). A user-configurable carrier is planned; until then the 38 kHz value is the only option exposed.
