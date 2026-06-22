# Codec

**Version**: 1.0
**Purpose**: Low-level single-packet IR encode/decode over the ESP32 RMT peripheral, unified by a half-bit timing model covering Manchester, pulse-width, and pulse-distance encodings.
**Status**: Built(1.0)

**Uses**:
.speck/rules/feature-system.md
spec/rules/esp-idf-component.md
spec/rules/cpp-style.md

## Output
Subspecs:
spec/systems/codec/protocol-config.md
spec/systems/codec/ir-encoder.md
spec/systems/codec/ir-transmitter.md
spec/systems/codec/ir-receiver.md
spec/systems/codec/rx-source.md

Artifacts:

## Spec

### Scope

Codec = layer `ir_lasertag::codec` = shared foundational mechanism for encoding one IR packet to RMT symbols and decoding one symbol frame back to bits. It owns no protocol semantics (player IDs, repeat pairs, field layouts) — those live in `ir_lasertag::protocols` wrappers built on top. The codec handles exactly one packet at a time; multi-packet behavior is a consumer concern.

The layer comprises five units, each detailed in a subspec:

| Unit | Subspec | Role |
|---|---|---|
| Core types | spec/systems/codec/protocol-config.md | `ProtocolConfig`, `HalfBit`, `IrMessage`, `RawTiming`, enums |
| Encoder | spec/systems/codec/ir-encoder.md | bits → RMT symbols (`ir_encoder_create`) |
| Transmitter | spec/systems/codec/ir-transmitter.md | `IrTransmitter` — owns one RMT TX channel |
| RX source | spec/systems/codec/rx-source.md | `IrRxSource` interface + `RmtRxSource` RMT backend (symbols → frames) |
| Receiver | spec/systems/codec/ir-receiver.md | `IrReceiver` — frames → `IrMessage` / `RawTiming` |

### Half-Bit Timing Model

One unified model spans all three encoding families, so one encoder and one decoder serve every protocol. Each data bit = two half-bit periods (`HalfBit` = duration + level). Zero and one bits are each defined by `{half1, half2}`. Family is an emergent property of the timing constants, not a separate code path:

| Family | Distinguishing rule | Example |
|---|---|---|
| Manchester | zero/one first-half levels differ (transition-based) | Arclite, RC5 |
| Pulse width | zero/one differ by mark duration, same space | IRT, Sony |
| Pulse distance | zero/one differ by space duration, same mark | NEC, MilesTag |

`is_manchester()` is derived from whether the zero and one first-half levels differ; no explicit family enum exists. Adding a protocol = defining a `ProtocolConfig` of timing constants, not writing encoder/decoder code. Field semantics and the per-bit encoding are specified in spec/systems/codec/protocol-config.md.

### Mark-Bias Compensation

IR demodulators systematically stretch marks and shorten spaces (typically ~50-100µs) relative to the transmitted signal. `mark_bias_us` corrects this at decode time: it is subtracted from received mark durations and added to received space durations before timing comparison. This avoids protocol-specific decoder logic for a hardware artifact. Applied by both decode paths (see spec/systems/codec/ir-receiver.md).

### Tolerance-Based Decoding

Decoding matches received timings against expected timings within `tolerance_percent` (default 25%), absorbing natural IR timing variation. Bias compensation is applied before the tolerance window is evaluated.

### RMT Channel Abstraction

`IrTransmitter` and `RmtRxSource` wrap ESP-IDF RMT driver details — channel allocation, carrier modulation, DMA, ISR callbacks. The caller supplies the GPIO and a `ProtocolConfig`; the wrapper handles RMT setup. RMT operates at 1 MHz / 1 µs resolution throughout the codec. The receiver is decoupled from the RMT hardware through the `IrRxSource` interface, allowing alternative symbol-frame backends.

### Dependencies

Codec depends only on ESP-IDF (RMT TX/RX drivers, FreeRTOS, esp_log) and the conventions in spec/rules/esp-idf-component.md and spec/rules/cpp-style.md. It depends on no protocol feature; consuming features depend on the codec, never the reverse.
