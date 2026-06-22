# CHECKLIST: IRT Protocol
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/features/protocols/irt.md

## Scope
[ ] CHK001 Spec scopes IRT as a pulse-width protocol wrapper, distinguishes gun vs spawner roles, and states the wrapper class models the gun role.
[ ] CHK002 ID semantics defined: 0xXXYY (XX stable/checksum, YY player/CCM ID), unassigned gun = 0xF0F0.

## Requirements
[ ] CHK003 IR Encoding table lists pulse-width, 38 kHz, 500 µs header mark/space, zero mark 350 µs, one mark 700 µs, 450 µs bit space, 16-bit MSB-first.
[ ] CHK004 Packet Repetition states guns transmit twice with a 2000 µs gap (REQUIRED — shot lost if repeat absent) and spawners send once every 50 ms.
[ ] CHK005 Spec states send() emits the required pair internally and references kRepeatGapUs=2000 and kPairTimeoutUs=50000.
[ ] CHK006 API table covers all public IrtGun members plus statics get_ccm_id/is_unassigned, kIrtConfig, and make_irt_config().

## Dependencies
[ ] CHK007 Uses lists exactly feature-system.md, the three codec subspecs, and protocol-authoring.md with no upward references; no agent/ or .github/ paths.

## Terminology
[ ] CHK008 Gun ID, CCM ID, player ID, spawner, and pair validation are used consistently with the Overview.

## Consistency
[ ] CHK009 README drift flagged: README documents start_receive(callback) async receive whereas the header is poll/queue-based (authoritative).
[ ] CHK010 Timing and constant values match kIrtConfig, kRepeatGapUs, kUnassignedId, and kPairTimeoutUs in irt.hpp.

## Style
[ ] CHK011 No code blocks; tables used for timing, repetition, and API; articles elided; present tense.
[ ] CHK012 Header block matches required format (Version, Purpose ≤ one sentence, Status Built(1.0), Uses, Output).
