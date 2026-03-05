# Arclite Tag Code

## Overview

Arclite is a laser tag platform that uses IR for collision detection between
props. Each emitter capability registered by a prop is issued a unique 16-bit
Tag Code for the game session. When a sensor receives a Tag Code, the collision
event is forwarded to the game engine for processing. Tag Codes are issued by
the server in networked sessions; collision validation is processed on the
client (player kit). Props are not expected to process Tag Codes beyond optional
deduplication. All values are valid, though 0x0000 is by convention "unset" and
will record a warning in the engine.

## IR Encoding

| Parameter        | Value                                  |
|------------------|----------------------------------------|
| Encoding         | Manchester                             |
| Carrier          | 38 kHz                                 |
| Header mark      | 520 µs                                 |
| Header space     | 520 µs                                 |
| Half-bit         | 300 µs                                 |
| Zero             | MARK(300 µs) → SPACE(300 µs)          |
| One              | SPACE(300 µs) → MARK(300 µs)          |
| Message size     | 16 bits                                |
| Bit order        | MSB first                              |
| Min interval     | 2500 µs                                |
| Repeat           | None (single send)                     |

## API

- `ArcliteTagCode::init(transmitter, receiver, config)` — Initialize with codec-layer TX/RX and protocol config.
- `ArcliteTagCode::deinit()` — Release references and stop receive.
- `ArcliteTagCode::send(tag_code)` — Transmit a 16-bit tag code (blocking).
- `ArcliteTagCode::start_receive(callback)` — Start receiving tag codes with an ISR-context callback.
- `ArcliteTagCode::stop_receive()` — Stop receiving tag codes.
- `ArcliteTagCode::is_initialized()` — Check initialization state.
- `ArcliteTagCode::is_receiving()` — Check if receive is active.

See [arclite.hpp](arclite.hpp) for parameter details and usage examples.
