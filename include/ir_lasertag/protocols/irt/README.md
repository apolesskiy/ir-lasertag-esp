# IRT Protocol

## Overview

IRT is a commercial laser tag system. IRT messages are 16 bits. Guns and spawners have slightly different behaviors and meanings of the ID.

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
| Repeat         | See below      |
| Repeat gap     | See below      |

### Packet Repetition

IRT guns and spawners behave differently.

* **Guns** transmit the gun's 16-bit ID twice with a repeat gap of 2000µs. This is *required* and the gunshot will not be received if the repeated message is not present.

* **Spawners** transmit a static 16-bit ID *once* every 50ms during continuous spawn mode or while the button is pressed in manual mode.


## Example ID Patterns

### Guns

Unassigned (off-CCM) guns send `0xF0F0`.

Guns on CCM send `0xXXYY`, where `XX` is a stable unknown value (likely a checksum) and `YY` is the ID of the player to whom the gun is assigned.


The high byte is persistent with respect to the low byte in testing.

| High Byte | Player ID (Low Byte) |
|-----------|-------------------|
| `0xC8`      | `0x04`              |
| `0x50`      | `0x05`              |
| `0x60`      | `0x06`              |
| `0xF8`      | `0x07`              |
| `0x38`      | `0x08`              |
| `0xA0`      | `0x09`              |
| `0x90`      | `0x0A`              |
| `0x08`      | `0x0B`              |
| `0xF0`      | `0x0C`              |
| `0x68`      | `0x0D`              |
| `0x58`      | `0x0E`              |
| `0xC0`      | `0x0F`              |
| `0x58`      | `0x10`              |
| `0xC0`      | `0x11`              |
| `0xF0`      | `0x12`              |
| `0x68`      | `0x13`              |
| `0x90`      | `0x14`              |
| `0x08`      | `0x15`              |
| `0x38`      | `0x16`              |
| `0xA0`      | `0x17`              |
| `0x60`      | `0x18`              |

Note mirrored repeats such as `0xA009` -> `0x900A`; `0xF00C` -> `0xC00F`. However, this is not bidirectional, as `0xC011` and `0xF012` also exist.

### Spawners

Spawners send a static ID.

| High Byte | Spawner ID (Low Byte) |
|-----------|-------------------|
| `0x37`      | `0xFB`              |


### Notes
  Byte 3 [12:8] is 0 or 8 for guns, 7 for spawner


## API

- `IrtGun::init()` — Initialize with transmitter and/or receiver.
- `IrtGun::send(uint16_t gun_id)` — Transmit gun ID (2× with 2ms gap).
- `IrtGun::start_receive(callback)` — Start receiving gun IDs asynchronously.
- `IrtGun::stop_receive()` — Stop receiving.
- `IrtGun::get_ccm_id(uint16_t)` — Extract CCM ID from gun ID.
- `IrtGun::is_unassigned(uint16_t)` — Check if gun is unassigned (0xF0F0).

See [irt.hpp](irt.hpp) for detailed API documentation.
