# CHECKLIST: Arclite TagCode Protocol
**Version:** 2.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/features/protocols/arclite.md

## Scope
[ ] CHK001 Spec scopes Arclite as the authoritative definition of the Arclite frame plus a wrapper over the codec, owning no game logic, and states the 14-bit Tag Code = collision-detection identifier.
[ ] CHK002 Spec states code ranges (0x0000 never emitted, 0x3FFF calibration) are the caller's policy, and that the wrapper refuses only codes over 0x3FFF.

## Requirements
[ ] CHK003 IR Encoding table lists Manchester, 38 kHz at 0.33, 1800 µs header mark, 600 µs header space, 300 µs half-bit, one=MARK→SPACE, zero=SPACE→MARK, 16-bit word (code, p1, p0) MSB-first with the parity masks, 12.0 ms frame, 4000 µs footer space, no repeat.
[ ] CHK004 API table covers every public member of ArcliteTagCode (init/deinit/send/send_async/start_receive/stop_receive/is_receiving/try_receive/receive/is_initialized) plus kArcliteConfig and make_arclite_config(); the frame codec table covers every public item of arclite_frame.hpp.
[ ] CHK013 Decoder rules state the receive windows (inclusive), the header-space cases, the straddle rule, lost edges, the abandoning-header restart, and the decision at p0's mid-bit transition.
[ ] CHK005 Spec states receive is poll/queue-based via try_receive/receive in task context with no callback registration.

## Dependencies
[ ] CHK006 Uses lists exactly .speck/rules/feature-system.md, the three codec subspecs, and spec/rules/protocol-authoring.md with no parent/ancestor (upward) references.
[ ] CHK007 No reference to agent/ or .github/ paths anywhere.

## Terminology
[ ] CHK008 Tag Code, prop, emitter capability, and collision event are used consistently with the Overview definitions.

## Consistency
[ ] CHK009 README API matches arclite.hpp and arclite_frame.hpp.
[ ] CHK010 Timing values in the spec match kArcliteConfig in arclite.hpp exactly.

## Style
[ ] CHK011 No code blocks; tables used for timing and API enumerations; articles elided; present tense.
[ ] CHK012 Header block matches the required format (Version, Purpose ≤ one sentence, Status Built(2.0), Uses, Output with Subspecs/Artifacts).
