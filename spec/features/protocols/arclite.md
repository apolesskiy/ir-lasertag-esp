# Arclite TagCode Protocol

**Version**: 2.0
**Purpose**: The Arclite IR frame: a 14-bit Tag Code with interleaved parity, its codec and decoder, and the send/receive wrapper.
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
include/ir_lasertag/protocols/arclite/arclite.hpp
src/protocols/arclite/arclite.cpp
include/ir_lasertag/protocols/arclite/arclite_frame.hpp
src/protocols/arclite/arclite_frame.cpp
include/ir_lasertag/protocols/arclite/README.md

## Spec

### Overview

Arclite = laser tag platform using IR for collision detection between props. Each emitter capability registered by a prop is issued a Tag Code for a game session. A sensor receiving a Tag Code → collision event forwarded to the game engine. Props are not expected to process Tag Codes beyond optional deduplication.

**This feature is the authoritative definition of the Arclite frame**: carrier, frame, data word and receive rules. Code ranges (`0x0000` never emitted, `0x3FFF` for calibration) are game policy and left to the caller; the wrapper sends any code that fits 14 bits.

Two parts: the frame codec (`arclite_frame.hpp`, pure logic, host-tested) and the `ArcliteTagCode` wrapper over the codec layer (spec/systems/codec/ir-transmitter.md, spec/systems/codec/ir-receiver.md).

### IR Encoding

`kArcliteConfig` is the authoritative `ProtocolConfig`; `make_arclite_config()` returns a copy.

| Parameter | Value |
|---|---|
| Encoding | Manchester |
| Carrier | 38 kHz, duty 0.33 |
| Header mark | 520 µs |
| Header space | 520 µs |
| Half-bit | 300 µs |
| One | MARK(300 µs) → SPACE(300 µs) |
| Zero | SPACE(300 µs) → MARK(300 µs) |
| Message | 16-bit data word, MSB first: `(code << 2) \| (p1 << 1) \| p0` |
| Parity | p0 = XOR of code bits under `0x1555`, p1 = under `0x2AAA` |
| Data word examples | code `0x0001` → word `0x0005`; `0x0123` → `0x048C`; `0x2A31` → `0xA8C4`; `0x3FFE` → `0xFFFA` |
| Parity coverage | detects any error flipping an odd number of bits under either mask: every single-bit error and every run of 2 or 3 adjacent flipped bits; not two flips two bits apart, nor a run of 4 |
| Frame length | 10.64 ms (0.52 + 0.52 + 16 × 0.6) |
| Footer | a 2000 µs space (`footer_space_us`): the 2 ms minimum gap is the frame's tail |
| Repeat | none (single send) |

`is_manchester()` = true. A leading 0 bit's space merges with the header space (820 µs); a frame ending in a 1 bit ends with its mark (its space half is the idle line).

### Frame Codec (`arclite_frame.hpp`)

| Item | Behavior |
|---|---|
| `arclite_word_from_code(code)` / `arclite_code_from_word(word)` / `arclite_word_parity_ok(word)` | data word and its parity |
| `arclite_word_bytes(code, bytes)` | the word as 2 bytes, high byte first, for `IrTransmitter` (its encoder walks bytes in memory order; see spec/systems/codec/ir-encoder.md) |
| `arclite_encode(code, runs)` | frame as merged mark/space runs, header mark first, ending with the last mark; 0 if the code is over `kArcliteCodeMax` (0x3FFF) |
| `ArcliteDecoder` | streaming receive decoder: `feed(level, duration)`, `end()`, and `decode_symbols(symbols, count, callback, ctx)` for a whole burst of `rmt_symbol_word_t` (a zero duration ends it) |
| `ArcliteFrame` | `result` (`kOk`, `kParity`, `kTiming`), `word`, `code`, and the runs as received from the header mark on |
| Constants | `kArcliteHeaderMarkUs`, `kArcliteHeaderSpaceUs`, `kArcliteHalfBitUs`, `kArcliteBitCount`, `kArcliteMessageGapUs` (2000), `kArcliteCodeMax`, parity masks, `kArcliteRxIdleUs` (1500), the receive windows (`kArcliteGlitchUs`, `kArcliteHalfMaxUs`, `kArcliteFullMaxUs`, `kArcliteHeaderMarkMinUs`/`MaxUs`, `kArcliteHeaderOneMinUs`/`MaxUs`, `kArcliteHeaderZeroMinUs`/`MaxUs`), `kArcliteFrameMaxRuns` |

Decoder rules:

- Receivers stretch marks and shorten spaces by up to about 5 carrier cycles (130 µs), so a 520 µs header mark can measure as long as a 600 µs data mark. Stretch moves the edge between a mark and the next space but not their sum, so **the header is found by its mark plus space**: a mark of 400–800 µs, and P = that mark + the next space. P in 970–1119 (nominal 1040) is a header whose first bit is a 1. P in 1270–1410 (nominal 1340) is a header whose space merged with a leading 0 bit's space half (520 + 300), which is consumed. Any other P is not a header and is skipped. No pair of data durations sums into either window: data pairs sum to 600, 900 or 1200 µs, and each window's edges sit midway, about 70 µs from each.
- Data durations, inclusive: glitch < 150 µs, half-bit 150–449, full bit 450–749; anything else is invalid.
- Wait for a header; noise that never forms one yields nothing, and a candidate mark whose sum is no header is not reported.
- Bits: a half-bit run keeps phase; a full-bit run must straddle a bit boundary, or the bit has no mid-bit transition and the frame is abandoned.
- Two runs of one level in a row mean a lost edge: abandoned.
- An abandoned frame (one whose header was found) is reported `kTiming`; if the run that abandoned it is a mark of 400–800 µs, it is the next frame's candidate header mark.
- The frame is decided at the mid-bit transition of its last bit (p0): parity good → `kOk`, else `kParity`. A burst that ends mid-frame reports `kTiming`.

Both receive backends (RMT RX via `RmtRxSource`, and software receivers via other `IrRxSource` implementations) deliver `rmt_symbol_word_t` bursts, so both decode through `ArcliteDecoder`. The codec layer's tolerance-based decoder is not used for Arclite.

Receive end-of-frame threshold: no valid run is longer than 1000 µs (marks are at most 800 µs, the longest space is the merged header space, 820 µs nominal), and frames from one emitter are at least 2 ms apart, so a receiver that ends a frame after steady level sets that time above 1000 µs and below 2 ms. `kArcliteRxIdleUs` is 1500 µs; `ArcliteTagCode::init` sets it on the receiver (`IrReceiver::set_end_of_frame_us`), since the codec's own rule (1.5 × the longest pulse, at least 1000 µs) would give 1000 µs. A decoder that can should report at the frame's last edge rather than wait for the timeout; this one is decided at p0's mid-bit transition, so it does whenever it is fed run by run.

### API

API surface from arclite.hpp = authoritative. `init` applies `config` to both transmitter and receiver via their `set_protocol`. Caller retains ownership of TX/RX; either may be null when only one direction is needed; both null → `ESP_ERR_INVALID_ARG`.

| Method | Behavior |
|---|---|
| `init(transmitter, receiver, config)` | Bind TX/RX and apply config; returns `ESP_OK`, `ESP_ERR_INVALID_ARG`, or `set_protocol` error |
| `deinit()` | Stop active receive; release references; does not deinit underlying TX/RX |
| `send(tag_code, timeout_ms=1000)` | Blocking; one frame of the code's data word plus the 2 ms footer; `ESP_ERR_INVALID_ARG` if the code is over 0x3FFF, `ESP_ERR_INVALID_STATE` if no TX |
| `send_async(tag_code)` | Non-blocking; queued in the transmitter, which copies the data (spec/systems/codec/ir-transmitter.md); `ESP_ERR_INVALID_ARG` if over 0x3FFF, `ESP_ERR_INVALID_STATE` if no TX or its queue is full |
| `start_receive()` / `stop_receive()` / `is_receiving()` | Receiver lifecycle |
| `try_receive(uint16_t* tag_code)` | Non-blocking poll; true if a burst held a valid frame |
| `receive(uint16_t* tag_code, timeout_ms)` | Blocking until a burst or timeout; decodes it with `ArcliteDecoder` (via `IrReceiver::receive_symbols`); `ESP_ERR_TIMEOUT`, `ESP_ERR_INVALID_RESPONSE` if the burst held no valid frame, `ESP_ERR_INVALID_STATE` if not receiving |
| `is_initialized()` | true after successful `init` |
| `kArcliteConfig` / `make_arclite_config()` | the config, and a copy |

Receive is poll/queue-based from task context; there is no callback registration. A consumer that needs every frame's result (parity and timing failures included), its runs, or its time uses `IrRxSource` and `ArcliteDecoder` directly.

### README

The Output Artifact README (include/ir_lasertag/protocols/arclite/README.md) API section is kept in sync with the header, which is the authoritative API source.
