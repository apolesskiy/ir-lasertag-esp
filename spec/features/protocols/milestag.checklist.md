# CHECKLIST: MilesTag Protocol
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/features/protocols/milestag.md

## Scope
[ ] CHK001 Spec states MilesTag is CONFIG-ONLY: header-only constants + kMilestagConfig, no wrapper class, no .cpp, not in CMake SRCS.
[ ] CHK002 Overview describes MilesTag as a standardized 14-bit DIY laser-tag format carrying player ID, team, and damage.

## Requirements
[ ] CHK003 IR Encoding table lists pulse-distance, 38 kHz, 2400 µs header mark, 600 µs header space, 600 µs bit mark, 600/1200 µs zero/one space, 600 µs footer mark, 14-bit MSB-first, ≥10000 µs message gap.
[ ] CHK004 Message Structure table gives bits 13–7 player (0–127), 6–5 team (0–3), 4–0 damage (0–31).
[ ] CHK005 Usage states the consumer applies kMilestagConfig and encodes/decodes the field layout itself.

## Dependencies
[ ] CHK006 Uses lists exactly feature-system.md, spec/systems/codec/protocol-config.md, and protocol-authoring.md (and NOT the transmitter/receiver subspecs, since header-only) with no upward references.
[ ] CHK007 No reference to agent/ or .github/ paths.

## Terminology
[ ] CHK008 Player ID, team, damage, and pulse-distance are used consistently with the encoding and structure tables.

## Consistency
[ ] CHK009 Discrepancy flagged: no wrapper class exists yet the README implies structured field access for team/player/damage (unimplemented/planned).
[ ] CHK010 Discrepancy flagged: carrier is 38 kHz in code/README whereas the MilesTag de-facto standard (and old design doc) is 56 kHz; code is ground truth.
[ ] CHK011 Timing/constant values match kMilestagConfig in milestag.hpp exactly.

## Style
[ ] CHK012 No code blocks; tables used for timing and structure; Status Built(1.0); header block matches required format.
