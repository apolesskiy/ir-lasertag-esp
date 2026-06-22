# CHECKLIST: Protocol Config

**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/systems/codec/protocol-config.md

## Scope
[ ] CHK001 Spec covers only core types (enums, HalfBit, ProtocolConfig, IrMessage, RawTiming), not encode/decode behavior.

## Requirements
[ ] CHK002 Full ProtocolConfig field table lists every field with type, default, and meaning.
[ ] CHK003 The four ProtocolConfig methods are listed with their return semantics.
[ ] CHK004 is_valid() rejection rules match the source (carrier range, duty range, zero-duration, indistinguishable, tolerance>50, bit_count>128).
[ ] CHK005 Half-bit encoding of one bit (two HalfBit periods) is explained and mapped to all three families.
[ ] CHK006 IrMessage fields and kMaxDataBytes=16 (128 bits) are stated.

## Dependencies
[ ] CHK007 Uses lists exactly feature-system.md and cpp-style.md; no upward/parent refs.
[ ] CHK008 Artifacts list both ir_protocol.hpp and ir_protocol_config.cpp, repo-root-relative.

## Terminology
[ ] CHK009 Enum value names (kSpace/kMark, kLsbFirst/kMsbFirst) and default tolerance (25) match the header.

## Consistency
[ ] CHK010 mark_bias_us sign convention (subtract from marks, add to spaces) is consistent with the codec parent and receiver specs.

## Style
[ ] CHK011 No code blocks; tables used for field/method enumerations.
[ ] CHK012 Terse present-tense active voice; articles elided.
