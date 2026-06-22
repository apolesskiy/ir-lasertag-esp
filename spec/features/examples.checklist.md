# CHECKLIST: Examples
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/features/examples.md

## Scope
[ ] CHK001 Spec covers exactly the two example apps (basic, profiling) present under examples/.
[ ] CHK002 basic is described as an Arclite TagCode send/receive demo with the RMT TX/RX + codec/wrapper init/polling pattern.
[ ] CHK003 profiling is described as an on-device CPU-load harness (phases, impact formula, matrix, target board).

## Requirements
[ ] CHK004 The impact formula and the three phases (baseline/loaded/recovery) match the profiling README.
[ ] CHK005 Build/flash commands for profiling match the README (set-target esp32s3, build, flash monitor).
[ ] CHK006 Hardware/wiring/matrix detail is deferred to the profiling README artifact, not duplicated.

## Dependencies
[ ] CHK007 Uses lists `.speck/rules/feature-system.md` plus transmit, receive, arclite, irt features; no upward/parent refs.
[ ] CHK008 Artifacts list only real files that exist under examples/; no `agent/` or `.github/` paths.

## Terminology
[ ] CHK009 Manchester (Arclite) and pulse-width (IRT) protocol descriptors used consistently with their feature specs.

## Consistency
[ ] CHK010 GPIO/channel counts mentioned (up to four RX, two TX) agree with the README artifact.

## Style
[ ] CHK011 Terse active-voice present tense; example APIs referenced in prose, no full code bodies.
[ ] CHK012 Status Built(1.0); Subspecs blank.
