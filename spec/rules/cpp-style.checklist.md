# CHECKLIST: C++ Style
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/rules/cpp-style.md

## Scope
[ ] CHK001 The spec scopes itself as the authoritative C++ style guide and states its basis (Google C++ Style Guide with ESP-IDF modifications).

## Requirements
[ ] CHK002 Formatting rules are unambiguous and specify concrete values (2-space indentation, no tabs, file extensions, header guard mechanism).
[ ] CHK003 The Naming table assigns exactly one convention per element type with no overlapping or conflicting categories.
[ ] CHK004 The documentation requirement is testable (every public function/method/class has a doc comment with @brief and parameter/return docs).

## Consistency
[ ] CHK005 The constant-naming guidance (UPPER_SNAKE_CASE for constants) is consistent with the "prefer constexpr over #define" directive.
[ ] CHK006 The README-vs-header rule (READMEs never duplicate parameter docs) is consistent with protocol-authoring.md, which it cross-references.

## Dependencies
[ ] CHK007 The spec contains no upward references and its only cross-reference (protocol-authoring.md) is a sibling rule.

## Style
[ ] CHK008 The spec is terse and free of code bodies, describing conventions rather than inlining implementation.
