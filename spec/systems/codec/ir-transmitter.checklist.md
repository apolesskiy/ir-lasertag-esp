# CHECKLIST: IR Transmitter

**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/systems/codec/ir-transmitter.md

## Scope
[ ] CHK001 Spec covers single-channel single-packet TX only; multi-packet protocol logic is out of scope.

## Requirements
[ ] CHK002 Lifecycle table covers init/deinit/is_initialized/is_busy with their state semantics.
[ ] CHK003 Protocol table covers set_protocol/get_protocol/set_carrier_duty including the duty>=1.0 carrier-disable rule.
[ ] CHK004 Send table covers send_async/wait_done/send/send_raw with timeouts and error returns.
[ ] CHK005 loop_count semantics (0=single shot, N=N hardware repeats, atomic) are stated.
[ ] CHK006 send_raw is described as bypassing protocol encoding and requiring init only.

## Dependencies
[ ] CHK007 Uses lists feature-system.md, protocol-config.md, ir-encoder.md, cpp-style.md; no upward refs.
[ ] CHK008 Artifacts list ir_transmitter.hpp and ir_transmitter.cpp, repo-root-relative.

## Terminology
[ ] CHK009 Method names and default arguments (timeout_ms=1000, loop_count=0) match the header.

## Consistency
[ ] CHK010 Carrier/encoder ownership is consistent with ir-encoder.md (encoder envelope vs transmitter carrier).
[ ] CHK011 pending-count raise/lower path (send_async vs IRAM TX-done ISR), the queue depth and slot reuse are internally consistent.

## Style
[ ] CHK012 No code blocks; tables for API enumeration; terse active present-tense voice.
