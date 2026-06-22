# CHECKLIST: IRT Protocol
**Version:** 2.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/features/protocols/irt.md

## Scope
[ ] CHK001 Spec scopes IRT as a pulse-width protocol wrapper supporting BOTH gun (paired) and spawner (single) roles via a role-neutral class `Irt`.
[ ] CHK002 ID semantics defined: 0xXXYY (XX stable/checksum, YY player/CCM ID), unassigned gun = 0xF0F0; spawner = static ID.

## Requirements
[ ] CHK003 IR Encoding table lists pulse-width, 38 kHz, 500 µs header mark/space, zero mark 350 µs, one mark 700 µs, 450 µs bit space, 16-bit MSB-first; encoding identical for both roles.
[ ] CHK004 Transmission section distinguishes send_gun (twice, 2000 µs gap, loop count 2) from send_spawner (single, loop count 0).
[ ] CHK005 Reception section defines the rules: matching pair within window → gun; lone packet past window → spawner; non-matching packet within window → slide (old candidate discarded, new replaces; X Y Y → Gun(Y)). receive() is event-driven (blocks on the codec queue, bounded for flush), no vTaskDelay.
[ ] CHK006 Pair-window timing rationale stated (gun pair ~18 ms inter-arrival vs spawner ~50 ms; kPairWindowUs = 30000 µs between them) and notes host-tested classify vs on-device end-to-end.
[ ] CHK007 API table covers all public `Irt` members (init/deinit/is_initialized, send_gun, send_spawner, start_receive/stop_receive/is_receiving, try_receive(IrtMessage*), receive, classify, statics get_ccm_id/is_unassigned), the IrtRole/IrtMessage/IrtPairResult types, kIrtConfig, make_irt_config().

## Dependencies
[ ] CHK008 Uses lists exactly feature-system.md, the three codec subspecs, and protocol-authoring.md with no upward references; no agent/ or .github/ paths.

## Terminology
[ ] CHK009 Gun, spawner, CCM ID, candidate, pair window, and role are used consistently with the Overview.

## Consistency
[ ] CHK010 README API matches the header (Irt, send_gun/send_spawner, IrtMessage{id,role}, poll/queue receive) — no stale callback or single-send API.
[ ] CHK011 Timing and constant values match kIrtConfig, kRepeatGapUs (2000), kUnassignedId (0xF0F0), and kPairWindowUs (30000) in irt.hpp.

## Style
[ ] CHK012 No code blocks; tables used for timing, transmission, and API; articles elided; present tense.
[ ] CHK013 Header block matches required format (Version 2.0, Purpose ≤ one sentence, Status Built(2.0), Uses, Output with the three artifacts).
