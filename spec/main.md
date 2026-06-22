# ir-lasertag-esp
**Version**: 1.0
**Purpose**: Reusable ESP-IDF C++ library providing a configurable IR codec and protocol-specific laser-tag APIs over the ESP32 RMT peripheral.
**Status**: Planned(1.0)

**Uses**:
.speck/rules/style.md
.speck/rules/feature-system.md
spec/rules/esp-idf-component.md
spec/rules/cpp-style.md
spec/rules/code-structure.md
spec/rules/testing.md
spec/rules/agent-first-development.md
spec/rules/protocol-authoring.md

## Output
Subspecs:
spec/rules/esp-idf-component.md
spec/rules/cpp-style.md
spec/rules/code-structure.md
spec/rules/testing.md
spec/rules/agent-first-development.md
spec/rules/protocol-authoring.md
spec/systems/codec/codec.md
spec/features/codec/transmit.md
spec/features/codec/receive.md
spec/features/protocols/arclite.md
spec/features/protocols/irt.md
spec/features/protocols/bc.md
spec/features/protocols/milestag.md
spec/features/examples.md
spec/tests/tests.md

Artifacts:
README.md

## Spec

### Product

`ir-lasertag-esp` is a standalone, reusable ESP-IDF component (C++17, MIT) for IR laser-tag signalling. It is a general-use library: downstream consumers are not assumed to use Speck. The user-facing API and documentation are the README files (preserved as artifacts) and the public headers under `include/ir_lasertag/`.

Component metadata: `idf_component.yml` (version 0.3.0; 0.3.0 introduced the breaking IRT API — `IrtGun` → `Irt` with gun/spawner roles). Target: any ESP32 variant with an RMT peripheral; ESP-IDF ≥ v5.3. Consumption and install: see `README.md` and `spec/rules/esp-idf-component.md`.

### Architecture

Two layers in one component (see `spec/systems/codec/codec.md` and `spec/rules/protocol-authoring.md`):

- **Codec layer** (`ir_lasertag::codec`) — low-level single-packet IR encode/decode over RMT, unified by a half-bit timing model that covers Manchester, pulse-width, and pulse-distance encodings. The shared foundational featureset.
- **Protocol layer** (`ir_lasertag::protocols`) — protocol-specific message APIs built on the codec, each with its own semantics (paired transmission, field layout) and its own README.

### Decomposition

Per `.speck/rules/feature-system.md`:

- **Systems** — the codec mechanism: protocol-config (half-bit model), encoder, transmitter, receiver, RX source. Under `spec/systems/codec/`.
- **Foundational features** — transmit and receive capabilities exposed to library users (`spec/features/codec/`).
- **Per-protocol features** — each protocol has its own feature spec and README artifact: Arclite, IRT, BC, MilesTag (`spec/features/protocols/`).
- **Rules** — cross-cutting concerns applied downward: ESP-IDF component conventions, C++ style, code structure, testing, agent-first development, protocol authoring.
- **Examples & tests** — usage examples and the host test suite.

### Protocols

| Protocol | Encoding | Bits | Carrier | Repeat | Spec |
|---|---|---|---|---|---|
| Arclite | Manchester | 16 | 38 kHz | none | `spec/features/protocols/arclite.md` |
| IRT | pulse-width | 16 | 38 kHz | gun: ×2, 2 ms gap | `spec/features/protocols/irt.md` |
| BC | pulse-width | 16 | 38 kHz | none | `spec/features/protocols/bc.md` |
| MilesTag | pulse-distance | 14 | 38 kHz | none | `spec/features/protocols/milestag.md` |

MilesTag is currently config-only (timing constants + `kMilestagConfig`); no wrapper class is implemented (a structured-field wrapper is planned). The other three provide wrapper classes (`ArcliteTagCode`, `Irt`, `BcGun`).

The user-facing `README.md` "Supported Protocols" table lists only protocols validated against real hardware; an implemented-but-untested protocol (currently BC) is absent from it until field-tested (see `spec/rules/protocol-authoring.md`).
