# Design

## Architecture

`ir_lasertag_esp` implements a two-layer IR protocol stack on top of ESP-IDF's RMT peripheral.

### Layer 1: Codec (`codec/`)
Low-level single-packet encode/decode. Supports three IR encoding families:
- **Manchester** (e.g., RC5, Arclite): zero and one bits differ by level ordering, same duration.
- **Pulse Width** (e.g., IRT, Sony): zero and one bits differ by mark duration, same space.
- **Pulse Distance** (e.g., NEC): zero and one bits differ by space duration, same mark.

All encodings are described by a unified **half-bit timing model**: each bit consists of two half-bits (`HalfBit`), each with a duration and level (mark/space). This allows a single encoder/decoder to handle all three families.

Key classes:
- `ProtocolConfig` — describes timing, carrier, bit count, tolerance, and mark bias for a protocol.
- `IrEncoder` — converts byte buffers to RMT symbols using a `ProtocolConfig`.
- `IrTransmitter` — wraps an RMT TX channel; sends `IrMessage` using `IrEncoder`.
- `IrReceiver` — wraps an RMT RX channel; decodes incoming symbols to `IrMessage`.

### Layer 2: Protocol Wrappers (`protocols/`)
Protocol-specific message APIs built on the codec layer. Each protocol wrapper handles multi-packet behaviors and field semantics:
- **Arclite** (`arclite.hpp`): 16-bit Manchester, 38kHz. `ArcliteTagCode` — send/receive 16-bit tag codes.
- **IRT** (`irt.hpp`): 16-bit Pulse Width, 38kHz. IRT guns send packet pairs; the protocol wrapper handles both paired and single-packet modes.
- **MilesTag** (`milestag.hpp`): 14-bit Pulse Distance, 56kHz. Header-only definition (implementation pending).

### Namespace
All types are in `ir_lasertag::codec::` (codec layer) or `ir_lasertag::protocols::` (protocol wrappers).

## Design Decisions

### Single Component for Both Layers
The codec and protocol layers are in a single component rather than separate ones. The protocol wrappers are thin and tightly coupled to codec types (IrMessage, ProtocolConfig). Splitting would add overhead without meaningful reuse benefit.

### Half-Bit Timing Model
Rather than separate encoder implementations per encoding family, all are unified through the half-bit model. A `ProtocolConfig` with `zero_half1/zero_half2/one_half1/one_half2` describes any encoding. `is_manchester()` is derived from whether half1 levels differ between zero and one. This simplifies the codebase and makes adding new protocols a matter of defining timing constants.

### Mark Bias Compensation
IR receivers introduce systematic timing errors — marks are typically stretched by 50-100µs relative to transmission. The `mark_bias_us` field in `ProtocolConfig` compensates for this at decode time, improving reliability without protocol-specific decoder logic.

### Tolerance-Based Decoding
The decoder accepts a `tolerance_percent` (default 25%) to match received timings against expected values. This handles natural timing variation in IR communication.

### RMT Channel Abstraction
`IrTransmitter` and `IrReceiver` wrap ESP-IDF RMT driver details (channel allocation, carrier config, DMA). The caller provides GPIO pin and protocol config; the wrapper handles RMT setup.
