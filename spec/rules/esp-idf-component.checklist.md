# CHECKLIST: ESP-IDF Component Conventions
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/rules/esp-idf-component.md

## Scope
[ ] CHK001 The spec scopes itself to component structure, namespaces, file naming, and consumption conventions, matching its Purpose line.

## Requirements
[ ] CHK002 The Component section specifies the registration mechanism and concrete build parameters (SRCS, INCLUDE_DIRS, REQUIRES) unambiguously.
[ ] CHK003 The single-component decision (codec and protocol in one component) is stated with an explicit rationale.
[ ] CHK004 The Namespaces table assigns each named type to exactly one namespace with no overlap between codec and protocols layers.
[ ] CHK005 The File Naming rules give a concrete convention for each artifact kind (sources, headers, docs, directories) with no ambiguity.

## Dependencies
[ ] CHK006 The spec contains no upward references; cross-references (protocol-authoring.md) point to a sibling rule.

## Consistency
[ ] CHK007 The C++17 / ESP-IDF ≥ v5.3 / MIT / RMT-required facts here match those stated in main.md (no version or license drift).
[ ] CHK008 The include-tree layout (include/ir_lasertag/ split into codec/ and protocols/<name>/) is consistent with the per-protocol layout in protocol-authoring.md.

## Style
[ ] CHK009 The spec is terse and contains no code bodies, describing conventions by reference and table.
