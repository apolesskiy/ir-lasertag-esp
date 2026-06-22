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

- `kArcliteConfig` — Default `ProtocolConfig` for the Arclite protocol.
- `make_arclite_config()` — Returns a copy of `kArcliteConfig`.
- `ArcliteTagCode::init(transmitter, receiver, config)` — Bind codec-layer TX/RX (either may be null) and apply config.
- `ArcliteTagCode::deinit()` — Release TX/RX references and stop receive.
- `ArcliteTagCode::is_initialized()` — Check initialization state.
- `ArcliteTagCode::send(tag_code, timeout_ms = 1000)` — Transmit a 16-bit tag code (blocking).
- `ArcliteTagCode::send_async(tag_code)` — Transmit a 16-bit tag code (non-blocking).
- `ArcliteTagCode::start_receive()` — Start the receiver for polling tag codes.
- `ArcliteTagCode::stop_receive()` — Stop receiving.
- `ArcliteTagCode::is_receiving()` — Check if receive is active.
- `ArcliteTagCode::try_receive(&tag_code)` — Non-blocking poll for a received tag code.
- `ArcliteTagCode::receive(&tag_code, timeout_ms)` — Blocking receive with timeout.

Receive is poll/queue-based and runs in task context (`try_receive` / `receive`); there is no callback registration.

See [arclite.hpp](arclite.hpp) for parameter details and usage examples.
