# IR Encoder

**Version**: 1.0
**Purpose**: Custom RMT encoder that converts a bit/byte buffer into RMT symbols per the half-bit model defined by a `ProtocolConfig`.
**Status**: Built(1.0)

**Uses**:
.speck/rules/feature-system.md
spec/systems/codec/protocol-config.md
spec/rules/cpp-style.md

## Output
Subspecs:

Artifacts:
include/ir_lasertag/codec/ir_encoder.hpp
src/codec/ir_encoder.cpp

## Spec

### Entry Point

`ir_encoder_create(const ProtocolConfig& config, rmt_encoder_handle_t* encoder_out)` → `esp_err_t`. Builds a custom RMT encoder driven entirely by `config` (see spec/systems/codec/protocol-config.md). Returns `ESP_ERR_INVALID_ARG` if `encoder_out` is null or `config.is_valid()` is false, `ESP_ERR_NO_MEM` on allocation failure. The handle plugs into ESP-IDF RMT TX via `rmt_transmit`; the transmitter owns its lifetime (see spec/systems/codec/ir-transmitter.md).

### Responsibilities

The encoder implements the RMT `encode`/`reset`/`del` interface over an internal copy encoder. It converts the caller's data buffer into a stream of RMT symbols according to the half-bit model. It holds a copy of the `ProtocolConfig` and two pre-computed bit symbols built once at creation.

### Symbol Generation From Config

| Stage | Symbol emitted | Driven by |
|---|---|---|
| Header | one symbol `{header_mark_us @ mark, header_space_us @ space}` | emitted only if `header_mark_us > 0` |
| Data | one two-phase symbol per bit | `zero_half1/zero_half2` or `one_half1/one_half2` |
| Footer | one symbol | emitted if `footer_mark_us > 0` or `footer_space_us > 0` |

Each data bit produces exactly one combined two-phase RMT symbol: phase0 = `half1` (duration + level), phase1 = `half2`. Both phases are filled (matching how RMT expects data symbols), so a bit maps to one symbol rather than two. The zero and one symbols are pre-computed from the config at creation.

Data stage: bits are read MSB-first or LSB-first per `bit_order`. The bit count emitted = `config.bit_count` if non-zero, else `data_size * 8`; encoding also stops at the end of the supplied buffer.

Footer stage: if `footer_mark_us > 0`, emits `{footer_mark @ mark, footer_space @ space}`; otherwise if only `footer_space_us > 0`, emits a space-only symbol (e.g. inter-message gap for looped TX).

### Carrier

The encoder produces only the mark/space envelope. Carrier modulation (frequency, duty) is applied separately by the transmitter via `rmt_apply_carrier`; it is not part of the encoder. See spec/systems/codec/ir-transmitter.md.

### State Machine

`encode` advances a header → data → footer → done state machine, honoring RMT `RMT_ENCODING_MEM_FULL` back-pressure (returns partial progress to be resumed). `reset` returns to the header state and resets the copy encoder; `del` frees the copy encoder and the encoder state.
