# CHECKLIST: IR Receiver

**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/systems/codec/ir-receiver.md

## Scope
[ ] CHK001 Spec covers decode of frames into IrMessage/RawTiming only; capture is delegated to the rx-source.

## Requirements
[ ] CHK002 Lifecycle table covers init(IrRxSource*)/deinit/start/stop and the queue-based ISR→task path.
[ ] CHK003 compute_max_symbol_duration_us is stated as 1.5x longest pulse, clamped to [1000, UINT16_MAX].
[ ] CHK004 Two decode paths (general + Manchester-optimized) are described distinctly, with dispatch by is_manchester().
[ ] CHK005 Both paths are stated to apply mark_bias_us and tolerance_percent via timing_matches.
[ ] CHK006 Raw mode API (enable/disable/is_raw_mode/receive_raw) and mutual exclusivity with decoded mode are stated.
[ ] CHK007 Spec explicitly states there is NO RawReceiveConfig struct.

## Dependencies
[ ] CHK008 Uses lists feature-system.md, protocol-config.md, rx-source.md, cpp-style.md; no upward refs to codec parent.
[ ] CHK009 Artifacts list ir_receiver.hpp and ir_receiver.cpp, repo-root-relative.

## Terminology
[ ] CHK010 Method names and validity rule (bit_count match vs >0) match the source.

## Consistency
[ ] CHK011 mark_bias sign convention matches protocol-config.md and the codec parent (subtract marks, add spaces).

## Style
[ ] CHK012 No code blocks; tables for API enumeration; terse active present-tense voice.
