# CHECKLIST: Arclite TagCode Protocol
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/features/protocols/arclite.md

## Scope
[ ] CHK001 Spec scopes Arclite as a protocol wrapper over the codec, owning no game logic, and states 16-bit Tag Code = collision-detection identifier.
[ ] CHK002 Spec states `0x0000` = "unset" by convention → engine warning, and that the wrapper enforces no value validation.

## Requirements
[ ] CHK003 IR Encoding table lists Manchester, 38 kHz, 520 µs header mark/space, 300 µs half-bit, zero=MARK→SPACE, one=SPACE→MARK, 16-bit MSB-first, 2500 µs min interval, no repeat.
[ ] CHK004 API table covers every public member of ArcliteTagCode (init/deinit/send/send_async/start_receive/stop_receive/is_receiving/try_receive/receive/is_initialized) plus kArcliteConfig and make_arclite_config().
[ ] CHK005 Spec states receive is poll/queue-based via try_receive/receive in task context with no callback registration.

## Dependencies
[ ] CHK006 Uses lists exactly .speck/rules/feature-system.md, the three codec subspecs, and spec/rules/protocol-authoring.md with no parent/ancestor (upward) references.
[ ] CHK007 No reference to agent/ or .github/ paths anywhere.

## Terminology
[ ] CHK008 Tag Code, prop, emitter capability, and collision event are used consistently with the Overview definitions.

## Consistency
[ ] CHK009 README drift is flagged: README API documents start_receive(callback)/ISR callback whereas the header (authoritative) is poll/queue-based.
[ ] CHK010 Timing values in the spec match kArcliteConfig in arclite.hpp exactly.

## Style
[ ] CHK011 No code blocks; tables used for timing and API enumerations; articles elided; present tense.
[ ] CHK012 Header block matches the required format (Version, Purpose ≤ one sentence, Status Built(1.0), Uses, Output with Subspecs/Artifacts).
