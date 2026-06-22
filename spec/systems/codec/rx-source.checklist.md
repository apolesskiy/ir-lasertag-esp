# CHECKLIST: RX Source

**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/systems/codec/rx-source.md

## Scope
[ ] CHK001 Spec covers the IrRxSource interface and the RmtRxSource reference backend only, not decode logic.

## Requirements
[ ] CHK002 IrRxSource member table covers set_on_frame_callback/start/stop/set_max_symbol_duration_us with contracts.
[ ] CHK003 End-of-frame threshold is defined as a single mark/space longer than the value, distinct from inter-transmission gap.
[ ] CHK004 RmtRxSource init signature and kDefaultSymbolBufferSize=128 are stated.
[ ] CHK005 signal_range_min_ns is stated as fixed ~1000 ns (ESP32 requirement); max derived from set_max_symbol_duration_us.
[ ] CHK006 RX-done ISR forwards the frame then re-arms capture; 1 MHz/1 µs resolution stated.

## Dependencies
[ ] CHK007 Uses lists exactly feature-system.md and cpp-style.md; no upward refs (does not list receiver/parent).
[ ] CHK008 Artifacts list ir_rx_source.hpp, rmt_rx_source.hpp, rmt_rx_source.cpp, repo-root-relative.

## Terminology
[ ] CHK009 IrRxFrameCallback signature and class names match the headers.

## Consistency
[ ] CHK010 Reference to the receiver computing/pushing max_symbol_duration uses a path, not duplicated detail.
[ ] CHK011 Immediate-re-arm safety rationale is consistent with the receiver's copy-then-queue behavior.

## Style
[ ] CHK012 No code blocks; tables for member/field enumeration; terse active voice.
