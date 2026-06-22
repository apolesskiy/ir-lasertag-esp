# CHECKLIST: Codec

**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/systems/codec/codec.md

## Scope
[ ] CHK001 Spec frames codec as overview only, deferring per-class detail to the five named subspecs.
[ ] CHK002 Spec states the codec handles exactly one packet and owns no protocol semantics.

## Requirements
[ ] CHK003 Half-bit model section enumerates all three encoding families with their distinguishing rule.
[ ] CHK004 Spec states `is_manchester()` is derived from differing zero/one first-half levels, with no family enum.
[ ] CHK005 Mark-bias section states marks are reduced and spaces increased at decode before tolerance comparison.
[ ] CHK006 Tolerance default (25%) is stated and ordering relative to bias compensation is unambiguous.

## Dependencies
[ ] CHK007 Uses lists exactly feature-system.md, esp-idf-component.md, cpp-style.md; no parent/upward refs.
[ ] CHK008 All five subspec paths are listed under Output Subspecs and are repo-root-relative.

## Terminology
[ ] CHK009 Namespace `ir_lasertag::codec` and unit/class names match the subspecs' terminology.

## Consistency
[ ] CHK010 Cross-references to subspecs use paths, not duplicated detail.
[ ] CHK011 1 MHz / 1 µs RMT resolution claim is consistent with the rx-source and transmitter subspecs.

## Style
[ ] CHK012 No code blocks; tables used for enumerations; terse present-tense active voice.
