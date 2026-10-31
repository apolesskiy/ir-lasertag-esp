# Arclite Tag Code

## Overview

Arclite is a laser tag platform that uses IR for collision detection between
props. Each emitter capability registered by a prop is issued a Tag Code for
the game session. When a sensor receives a Tag Code, the collision event is
forwarded to the game engine for processing. The frame carries a 14-bit code
with two interleaved parity bits; its full definition is
[`spec/features/protocols/arclite.md`](../../../../spec/features/protocols/arclite.md).
Which codes a game may use (`0x0000` is never emitted, `0x3FFF` is for
calibration) is the caller's concern; this library sends any code that fits
14 bits.

## IR Encoding

| Parameter        | Value                                         |
|------------------|-----------------------------------------------|
| Encoding         | Manchester                                    |
| Carrier          | 38 kHz, 33% duty                              |
| Header mark      | 520 µs                                        |
| Header space     | 520 µs                                        |
| Half-bit         | 300 µs                                        |
| One              | MARK(300 µs) → SPACE(300 µs)                 |
| Zero             | SPACE(300 µs) → MARK(300 µs)                 |
| Message size     | 16 bits: 14-bit code, then p1, p0             |
| Parity           | p0 over code bits `0x1555`, p1 over `0x2AAA` (even) |
| Bit order        | MSB first                                     |
| Frame length     | 10.64 ms                                      |
| Min interval     | 2 ms of idle after the frame (sent as a 2000 µs footer space) |
| Repeat           | None (single send)                            |
| Receive          | the header found by its mark plus space; data windows 150–449 / 450–749 µs (`ArcliteDecoder`); end-of-frame idle between 1000 µs and 2 ms (1500 µs) |

## API

- `kArcliteConfig` — Default `ProtocolConfig` for the Arclite protocol.
- `make_arclite_config()` — Returns a copy of `kArcliteConfig`.
- `ArcliteTagCode::init(transmitter, receiver, config)` — Bind codec-layer TX/RX (either may be null) and apply config.
- `ArcliteTagCode::deinit()` — Release TX/RX references and stop receive.
- `ArcliteTagCode::is_initialized()` — Check initialization state.
- `ArcliteTagCode::send(tag_code, timeout_ms = 1000)` — Transmit a 14-bit tag code (blocking).
- `ArcliteTagCode::send_async(tag_code)` — Transmit a 14-bit tag code (non-blocking, queued).
- `ArcliteTagCode::start_receive()` — Start the receiver for polling tag codes.
- `ArcliteTagCode::stop_receive()` — Stop receiving.
- `ArcliteTagCode::is_receiving()` — Check if receive is active.
- `ArcliteTagCode::try_receive(&tag_code)` — Non-blocking poll for a received tag code.
- `ArcliteTagCode::receive(&tag_code, timeout_ms)` — Blocking receive with timeout.
- `arclite_word_from_code()`, `arclite_code_from_word()`, `arclite_word_parity_ok()`, `arclite_word_bytes()` — The data word ([arclite_frame.hpp](arclite_frame.hpp)).
- `arclite_encode(code, runs)` — The frame as mark/space runs.
- `ArcliteDecoder` — The receive decoder: `feed()`, `end()`, `decode_symbols()`; reports `ArcliteFrame` results (ok, parity, timing) with the runs as received.

Receive through `ArcliteTagCode` is poll/queue-based and runs in task context
(`try_receive` / `receive`); there is no callback registration. To see every
frame's result, its runs, or its time, feed an `IrRxSource`'s bursts to
`ArcliteDecoder` directly.

See [arclite.hpp](arclite.hpp) and [arclite_frame.hpp](arclite_frame.hpp) for parameter details.
