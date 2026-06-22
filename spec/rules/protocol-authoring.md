# Protocol Authoring
**Version**: 1.0
**Purpose**: Conventions every protocol wrapper follows — directory layout, single top-level header, required README, and codec usage.
**Status**: Built(1.0)

**Uses**:

## Output
Subspecs:

Artifacts:
include/ir_lasertag/protocols/README.md

## Spec

Applied by every protocol feature (Arclite, IRT, BC, MilesTag, and future protocols). The human-facing form of these guidelines is the artifact `include/ir_lasertag/protocols/README.md`.

### Directory Layout

Each protocol lives in its own subdirectory `include/ir_lasertag/protocols/<name>/` with a single top-level header `<name>.hpp` as the consumer include, plus a `README.md`. Implementation sources go in `src/protocols/<name>/`. A protocol may be header-only (config/constants with no wrapper class and no compiled source).

### Required README

Every protocol provides a `README.md` with:

1. **Overview** — one paragraph: what the protocol is, who uses it, its purpose.
2. **IR Encoding** — table of encoding family, carrier, header mark/space, bit timings (mark/space/half-bit as applicable), message size (bit count), bit order, message gap / repeat behavior, and any protocol-specific timing notes (e.g. paired transmission).
3. **API** — one line per public function/class. The header is the authoritative source for parameter docs; the README links to it and does not duplicate parameter detail.

Each protocol README's API section must match the protocol header (the authoritative API). When the header changes, update the README.

### Field-Tested Listing

The top-level `README.md` "Supported Protocols" table lists only protocols validated against real hardware. A protocol may be fully implemented (header, wrapper, config, its own README) yet absent from that table until it is field-tested.

### Codec Usage

Protocol wrappers use the codec layer (`spec/systems/codec/codec.md`) internally and run in task context (not ISR). Multi-packet sequences use blocking `IrTransmitter::send()` with appropriate gaps. Protocol-specific types (message structs, enums, timing constants, the protocol's `ProtocolConfig`) are defined in the protocol header, not in the codec layer. Style: `spec/rules/cpp-style.md`.
