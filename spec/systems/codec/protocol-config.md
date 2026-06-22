# Protocol Config

**Version**: 1.0
**Purpose**: Core codec data types — the `ProtocolConfig` half-bit timing description and the message/timing value types shared across encode and decode.
**Status**: Built(1.0)

**Uses**:
.speck/rules/feature-system.md
spec/rules/cpp-style.md

## Output
Subspecs:

Artifacts:
include/ir_lasertag/codec/ir_protocol.hpp
src/codec/ir_protocol_config.cpp

## Spec

### Enums

| Type | Values | Meaning |
|---|---|---|
| `Level` (uint8_t) | `kSpace`=0, `kMark`=1 | carrier off / carrier on |
| `BitOrder` (uint8_t) | `kLsbFirst`=0, `kMsbFirst`=1 | bit transmission/decode order |

### HalfBit

`HalfBit` = `{ uint16_t duration_us; Level level; }` = one half of a bit period. Helpers `is_mark()` / `is_space()` test `level`. Half-bit durations are µs (1 MHz RMT resolution).

### Half-Bit Encoding Of One Bit

Each data bit = two `HalfBit` periods: `half1` then `half2`. Zero is `{zero_half1, zero_half2}`; one is `{one_half1, one_half2}`. This single representation expresses each family by choice of constants:

| Family | half1/half2 relationship | Identified by |
|---|---|---|
| Manchester | zero = MARK→SPACE, one = SPACE→MARK (levels invert) | `zero_half1.level != one_half1.level` |
| Pulse width | both = MARK→SPACE; one's mark longer; spaces equal | first-half levels equal |
| Pulse distance | both = MARK→SPACE; one's space longer; marks equal | first-half levels equal |

### ProtocolConfig Fields

| Field | Type | Default | Meaning |
|---|---|---|---|
| `carrier_freq_hz` | uint32_t | 38000 | carrier frequency (Hz) |
| `carrier_duty` | float | 0.33 | carrier duty cycle 0.0-1.0; `>= 1.0` disables carrier (TX side) |
| `header_mark_us` | uint16_t | 0 | leading mark; 0 = no header |
| `header_space_us` | uint16_t | 0 | header space after the header mark |
| `zero_half1` | HalfBit | {600, kMark} | zero bit, first half |
| `zero_half2` | HalfBit | {600, kSpace} | zero bit, second half |
| `one_half1` | HalfBit | {600, kMark} | one bit, first half |
| `one_half2` | HalfBit | {1200, kSpace} | one bit, second half |
| `footer_mark_us` | uint16_t | 0 | trailing mark; 0 = skip |
| `footer_space_us` | uint16_t | 0 | trailing space after footer mark; 0 = skip |
| `bit_order` | BitOrder | kMsbFirst | bit order |
| `bit_count` | uint8_t | 0 | expected message bit count; 0 = any valid count |
| `tolerance_percent` | uint8_t | 25 | receive timing tolerance (%) |
| `mark_bias_us` | int16_t | 0 | demodulator mark-stretch compensation (subtracted from received marks, added to received spaces at decode) |

Default field values describe a pulse-distance-style bit (zero = 600/600, one = 600/1200), not a wired protocol; real protocols override the full struct.

### ProtocolConfig Methods

| Method | Returns |
|---|---|
| `is_valid()` | validity (see rules below) |
| `is_manchester()` | `zero_half1.level != one_half1.level` |
| `zero_bit_duration_us()` | `zero_half1.duration_us + zero_half2.duration_us` |
| `one_bit_duration_us()` | `one_half1.duration_us + one_half2.duration_us` |

`is_valid()` (in src/codec/ir_protocol_config.cpp) rejects a config when any of: `carrier_freq_hz` outside 10000-100000 Hz; `carrier_duty` outside 0.0-1.0; both half-bit durations zero for zero or for one; zero and one indistinguishable (both durations and both levels equal); `tolerance_percent` > 50; `bit_count` > `IrMessage::kMaxDataBytes * 8` (128).

### IrMessage

`IrMessage` = decoded packet. `kMaxDataBytes` = 16 → max 128 bits. Fields: `data[16]` (bytes), `bit_count`, `is_repeat` (true if a repeat code), `valid` (true if decode succeeded). Default-constructed all-zero/false.

### RawTiming

`RawTiming` = `{ uint16_t duration_us; Level level; }` = one raw mark/space pulse, used by raw TX (`send_raw`) and raw RX (`receive_raw`) for protocol analysis, bypassing bit-level encode/decode.
