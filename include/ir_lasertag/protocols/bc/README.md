# BC Protocol

## Overview

BC is a commercial laser tag system. Taggers broadcast their 16-bit ID
only; all game logic is handled internally by the tagger. Each shot is
transmitted as a single 16-bit pulse-width-encoded packet with no
paired-packet repeat.

## IR Encoding

| Parameter      | Value          |
|----------------|----------------|
| Encoding       | Pulse-width    |
| Carrier        | 38 kHz         |
| Header mark    | 1900 µs        |
| Header space   | 450 µs         |
| Zero mark      | 500 µs         |
| One mark       | 1000 µs        |
| Bit space      | 450 µs         |
| Message size   | 16 bits        |
| Bit order      | MSB first      |
| Repeat         | None           |

## API

- `kBcConfig` — Default `ProtocolConfig` for the BC protocol.
- `make_bc_config()` — Returns a copy of `kBcConfig`.
- `BcGun::init(tx, rx, config)` — Bind transmitter/receiver and apply config.
- `BcGun::deinit()` — Release transmitter/receiver references.
- `BcGun::send(gun_id)` — Transmit a 16-bit gun ID (blocking).
- `BcGun::send_async(gun_id)` — Transmit a 16-bit gun ID (non-blocking).
- `BcGun::start_receive()` / `stop_receive()` — Control receive lifecycle.
- `BcGun::try_receive(&gun_id)` — Non-blocking poll for a received gun ID.
- `BcGun::receive(&gun_id, timeout_ms)` — Blocking receive with timeout.
