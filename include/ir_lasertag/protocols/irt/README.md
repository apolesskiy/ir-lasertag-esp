# IRT Gun Protocol

## Overview

IRT is a commercial laser tag system. IRT guns transmit a 16-bit ID in format 0xXXYY where XX is a high byte (purpose unknown, likely a checksum) and YY is the ID of the player to whom the gun is assigned. Guns not assigned to a player (running in autonomous mode) transmit 0xF0F0.

## IR Encoding

| Parameter      | Value          |
|----------------|----------------|
| Encoding       | Pulse-width    |
| Carrier        | 38 kHz         |
| Header mark    | 500 µs         |
| Header space   | 500 µs         |
| Zero mark      | 350 µs         |
| One mark       | 700 µs         |
| Bit space      | 450 µs         |
| Message size   | 16 bits        |
| Bit order      | MSB first      |
| Repeat         | 2x (same message sent twice) |
| Repeat gap     | 2000 µs        |

## Example ID Patterns

Unassigned guns send 0xF0F0.

The high byte is persistent with respect to the low byte in testing.

| High Byte | Player ID (Low Byte) |
|-----------|-------------------|
| 0xC8      | 0x04              |
| 0x50      | 0x05              |
| 0x60      | 0x06              |
| 0xF8      | 0x07              |
| 0x38      | 0x08              |
| 0xA0      | 0x09              |
| 0x90      | 0x0A              |
| 0x08      | 0x0B              |
| 0xF0      | 0x0C              |
| 0x68      | 0x0D              |
| 0x58      | 0x0E              |
| 0xC0      | 0x0F              |
| 0x58      | 0x10              |
| 0xC0      | 0x11              |
| 0xF0      | 0x12              |
| 0x68      | 0x13              |
| 0x90      | 0x14              |
| 0x08      | 0x15              |
| 0x38      | 0x16              |
| 0xA0      | 0x17              |
| 0x60      | 0x18              |

Note mirrored repeats such as 0xA009 -> 0x900A; 0xF00C -> 0xC00F. However, this is not bidirectional, as 0xC011 and 0xF012 also exist.

## API

- `IrtGun::init()` — Initialize with transmitter and/or receiver.
- `IrtGun::send(uint16_t gun_id)` — Transmit gun ID (2× with 2ms gap).
- `IrtGun::start_receive(callback)` — Start receiving gun IDs asynchronously.
- `IrtGun::stop_receive()` — Stop receiving.
- `IrtGun::get_ccm_id(uint16_t)` — Extract CCM ID from gun ID.
- `IrtGun::is_unassigned(uint16_t)` — Check if gun is unassigned (0xF0F0).

See [irt.hpp](irt.hpp) for detailed API documentation.
