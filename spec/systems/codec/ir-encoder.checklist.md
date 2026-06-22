# CHECKLIST: IR Encoder

**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/systems/codec/ir-encoder.md

## Scope
[ ] CHK001 Spec covers only encoder creation and symbol generation, not transmitter or carrier ownership.

## Requirements
[ ] CHK002 ir_encoder_create signature and its error returns (INVALID_ARG, NO_MEM) are stated.
[ ] CHK003 Symbol-generation table covers header, data, footer stages with their emit conditions.
[ ] CHK004 Spec states one data bit → one combined two-phase symbol (phase0=half1, phase1=half2).
[ ] CHK005 Bit count = config.bit_count if non-zero else data_size*8, and bit_order MSB/LSB handling are stated.
[ ] CHK006 Footer mark+space vs space-only behavior is described.

## Dependencies
[ ] CHK007 Uses lists feature-system.md, protocol-config.md, cpp-style.md; no upward refs to codec parent.
[ ] CHK008 Artifacts list ir_encoder.hpp and ir_encoder.cpp; type detail is referenced to protocol-config.md, not duplicated.

## Terminology
[ ] CHK009 Half-bit / two-phase symbol terms match protocol-config.md and the codec parent.

## Consistency
[ ] CHK010 Carrier ownership is attributed to the transmitter, consistent with ir-transmitter.md.
[ ] CHK011 MEM_FULL back-pressure and reset-to-header behavior match the described state machine.

## Style
[ ] CHK012 No code blocks; table used for stage enumeration; terse active voice.
