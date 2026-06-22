# CHECKLIST: Testing
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/rules/testing.md

## Scope
[ ] CHK001 The spec scopes test strategy into clearly distinguished tiers (host tests vs. on-device tests) matching its Purpose line.

## Requirements
[ ] CHK002 The host-test tier states precisely what it covers and what it compiles (hardware-independent sources plus ESP-IDF stub headers in host_test/stubs/).
[ ] CHK003 The on-device tier states its prerequisites unambiguously (IR hardware, typically a second board) and what app exercises it.
[ ] CHK004 The single host-test runner executable is named explicitly (test_ir_protocol) and the test inventory location is referenced (spec/tests/tests.md).

## Reproducibility
[ ] CHK005 The Run section gives a complete, copy-pasteable command sequence sufficient to build and run host tests.

## Dependencies
[ ] CHK006 The Uses list and cross-references contain no upward references; the Definition-of-Done cross-reference to code-structure.md is to a sibling rule.

## Consistency
[ ] CHK007 The Definition of Done ("new functionality covered by tests; all host tests pass before commit") is consistent with code-structure.md and is not restated with differing wording that could conflict.

## Style
[ ] CHK008 The spec is terse and contains no inlined code beyond the necessary run commands.
