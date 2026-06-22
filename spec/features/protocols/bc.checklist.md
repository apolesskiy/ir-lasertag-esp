# CHECKLIST: BC Protocol
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/features/protocols/bc.md

## Scope
[ ] CHK001 Spec scopes BC as a pulse-width wrapper broadcasting a 16-bit tagger ID only, with all game logic internal to the tagger.
[ ] CHK002 Spec states a single packet with no paired repeat and no pair validation (contrasted with IRT).

## Requirements
[ ] CHK003 IR Encoding table lists pulse-width, 38 kHz, 1900 µs header mark, 450 µs header space, zero mark 500 µs, one mark 1000 µs, 450 µs bit space, 16-bit MSB-first, no repeat.
[ ] CHK004 API table covers all public BcGun members (init/deinit/send/send_async/start_receive/stop_receive/try_receive/receive/is_initialized/is_receiving) plus kBcConfig and make_bc_config().
[ ] CHK005 Spec states receive is poll/queue-based in task context with no callback.

## Dependencies
[ ] CHK006 Uses lists exactly feature-system.md, the three codec subspecs, and protocol-authoring.md with no upward references; no agent/ or .github/ paths.

## Terminology
[ ] CHK007 Tagger, gun ID, and broadcast are used consistently with the Overview.

## Consistency
[ ] CHK008 Timing values in the spec match kBcConfig in bc.hpp exactly.
[ ] CHK009 Spec notes the README API section matches the header (no callback drift), accurately reflecting bc/README.md.

## Style
[ ] CHK010 No code blocks; tables used for timing and API; articles elided; present tense.
[ ] CHK011 Header block matches required format (Version, Purpose ≤ one sentence, Status Built(1.0), Uses, Output).
[ ] CHK012 Cross-reference to spec/features/protocols/irt.md is a sibling reference, not an upward Uses dependency.
