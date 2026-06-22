# MilesTag Protocol

## Conceptual Overview

MilesTag is one of the most widely-used laser tag protocols, originally developed
for DIY laser tag systems. It provides a standardized 14-bit message format for
transmitting player ID, team, and damage information over IR. Many commercial
and hobbyist laser tag systems implement MilesTag compatibility.

## IR Encoding

| Parameter | Value |
|-----------|-------|
| Encoding Type | Pulse-distance |
| Carrier Frequency | 38 kHz |
| Header Mark | 2400 µs |
| Header Space | 600 µs |
| Bit Mark (0 and 1) | 600 µs |
| Zero Space | 600 µs (total bit: 1200 µs) |
| One Space | 1200 µs (total bit: 1800 µs) |
| Footer Mark | 600 µs |
| Message Gap | 10000 µs minimum |
| Bit Count | 14 bits |
| Bit Order | MSB first |

MilesTag does not mandate a carrier frequency; both 38 kHz and 56 kHz are
used in practice. `kMilestagConfig` currently defaults to 38 kHz. A
user-configurable carrier is planned.

## Message Structure

Standard MilesTag messages are 14 bits:
- Bits 13-7 (7 bits): Player ID (0-127)
- Bits 6-5 (2 bits): Team (0-3)
- Bits 4-0 (5 bits): Damage code (0-31)

## Usage

```cpp
#include "ir_lasertag/protocols/milestag/milestag.hpp"

// Access timing constants
uint16_t header_mark = ir_lasertag::protocols::kMilestagHeaderMarkUs;

// Use the default configuration
const auto& config = ir_lasertag::protocols::kMilestagConfig;
transmitter.set_protocol(config);
```
