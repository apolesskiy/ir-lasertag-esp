# CHECKLIST: Code Structure & Definition of Done
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/rules/code-structure.md

## Scope
[ ] CHK001 The spec scopes itself to change-granularity, documentation, and Definition-of-Done rules, matching its Purpose line.

## Requirements
[ ] CHK002 The change-granularity rule is concrete (function/method length under 50 lines where possible) and qualified so it is checkable.
[ ] CHK003 The commit-message rule unambiguously states the required prefix (`[Agent]`).
[ ] CHK004 Each Definition-of-Done item is an objectively verifiable condition (tests cover new functionality, host tests pass, style conformance, public API documented).

## Dependencies
[ ] CHK005 The Uses list declares testing.md, and every cross-reference (testing.md, cpp-style.md) points to a sibling rule with no upward references.

## Consistency
[ ] CHK006 The "tests cover new functionality / all host tests pass" Definition-of-Done items are consistent with testing.md's Definition of Done (no contradictory criteria).
[ ] CHK007 The documentation requirement here (each public function/method documented) does not conflict with cpp-style.md's documentation rule.

## Style
[ ] CHK008 The spec is terse and contains no code, expressing rules as prose and bulleted criteria.
