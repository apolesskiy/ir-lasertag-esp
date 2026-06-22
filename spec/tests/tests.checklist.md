# CHECKLIST: Tests
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/tests/tests.md

## Scope
[ ] CHK001 Purpose contains the word "tests".
[ ] CHK002 Spec is an inventory of host tests, not a duplication of run commands or tiers (those defer to spec/rules/testing.md).

## Requirements
[ ] CHK003 Names the single executable `test_ir_protocol` and ctest name `ir_protocol_tests`, matching host_test/CMakeLists.txt.
[ ] CHK004 Lists all build inputs: the three test sources, test_main.cpp, stubs/, and src/codec/ir_protocol_config.cpp.
[ ] CHK005 Describes coverage of each of the three test files accurately.
[ ] CHK006 States that only ir_protocol_config.cpp is compiled and that RMT-dependent encode/decode/transmit/receive is not host-tested.
[ ] CHK007 Points on-device integration coverage to examples/profiling via spec/features/examples.md.

## Dependencies
[ ] CHK008 Uses lists exactly `.speck/rules/feature-system.md` and `spec/rules/testing.md`; no upward/parent refs.
[ ] CHK009 Artifacts lists host_test/CMakeLists.txt; no `agent/` or `.github/` paths.

## Terminology
[ ] CHK010 Uses ProtocolConfig/IrMessage/CCM-ID/gun-ID terms consistently with the codec and protocol specs.

## Style
[ ] CHK011 Terse active-voice present tense; tables for inputs/coverage; no full code bodies.
[ ] CHK012 Status Built(1.0); Subspecs blank.
