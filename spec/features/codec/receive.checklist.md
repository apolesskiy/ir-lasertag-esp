# CHECKLIST: Receive
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/features/codec/receive.md

## Scope
[ ] CHK001 Spec frames Receive as a user-facing capability covering both decoded and raw capture, deferring mechanism to Uses systems.
[ ] CHK002 Pluggable RX source (RmtRxSource default, custom IrRxSource) is described.

## Requirements
[ ] CHK003 Each capability lists its API name matching `IrReceiver`/`IrRxSource` (set_protocol/start/stop/receive/try_receive/receive_raw/enable_raw_mode/set_max_symbol_duration_us).
[ ] CHK004 Decoded and raw modes are stated as mutually exclusive, with INVALID_STATE on misuse.
[ ] CHK005 Reliability (tolerance_percent + mark_bias_us) and end-of-frame threshold are described and attributed to systems.
[ ] CHK006 Acceptance criteria stated as observable outcomes (valid IrMessage, TIMEOUT, raw sequence, INVALID_STATE).

## Dependencies
[ ] CHK007 Uses lists `.speck/rules/feature-system.md` plus protocol-config, ir-receiver, rx-source; no upward/parent refs.
[ ] CHK008 No `agent/` or `.github/` paths appear anywhere.

## Terminology
[ ] CHK009 IrMessage, RawTiming, mark-bias, tolerance, max-symbol-duration used consistently with codec system specs.

## Consistency
[ ] CHK010 try_receive described as receive with timeout 0, matching the API contract.

## Style
[ ] CHK011 No code blocks; APIs in prose/tables; terse active-voice present tense; Artifacts blank.
