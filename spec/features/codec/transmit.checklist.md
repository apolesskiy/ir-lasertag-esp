# CHECKLIST: Transmit
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/features/codec/transmit.md

## Scope
[ ] CHK001 Spec frames Transmit as a user-facing capability, not a mechanism, and defers encoding/RMT detail to Uses systems.
[ ] CHK002 Spec covers all TX modes: blocking send, async send + wait_done, raw send, hardware repeat, carrier duty.

## Requirements
[ ] CHK003 Each capability lists its API name and a one-line behavior, matching `IrTransmitter` surface (init/set_protocol/send/send_async/wait_done/is_busy/send_raw/set_carrier_duty).
[ ] CHK004 Acceptance criteria are stated as observable outcomes (IR emitted, ESP_OK, exact raw timings).
[ ] CHK005 Raw mode is described as bypassing encoding and emitting exact supplied timings.

## Dependencies
[ ] CHK006 Uses lists `.speck/rules/feature-system.md` plus protocol-config, ir-encoder, ir-transmitter systems; no upward/parent refs.
[ ] CHK007 No `agent/` or `.github/` paths appear anywhere.

## Terminology
[ ] CHK008 Half-bit model, mark/space, loop_count, carrier duty used consistently with codec system specs.

## Consistency
[ ] CHK009 Default values cited (timeout_ms=1000, loop_count=0) match the transmitter API contract.

## Style
[ ] CHK010 No code blocks; APIs referenced in prose/tables; terse active-voice present tense.
[ ] CHK011 Artifacts section left blank (capability realized by Uses systems).
