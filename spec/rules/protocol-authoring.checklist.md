# CHECKLIST: Protocol Authoring
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/rules/protocol-authoring.md

## Scope
[ ] CHK001 The spec scopes itself to conventions every protocol wrapper follows (layout, single header, required README, codec usage), matching its Purpose line.
[ ] CHK002 The spec states its applicability explicitly (every protocol feature, current and future) and names the human-facing artifact form.

## Requirements
[ ] CHK003 The Directory Layout rule unambiguously specifies header location, the single top-level consumer header, README, and implementation source location.
[ ] CHK004 The header-only allowance (config/constants with no wrapper class and no compiled source) is stated clearly and does not contradict the single-header requirement.
[ ] CHK005 The Required README section enumerates a complete, checkable set of mandatory sections (Overview, IR Encoding, API) with their required contents.
[ ] CHK006 The Codec Usage rules are concrete and testable (task context not ISR, blocking send() with gaps for multi-packet, protocol-specific types in the protocol header).

## Artifacts
[ ] CHK007 The README artifact (include/ir_lasertag/protocols/README.md) is listed in the Output Artifacts section.

## Dependencies
[ ] CHK008 Cross-references (codec.md, cpp-style.md) point to sibling specs/rules with no upward references.

## Consistency
[ ] CHK009 The "header is authoritative, README does not duplicate parameter detail" rule is consistent with cpp-style.md and agent-first-development.md.

## Style
[ ] CHK010 The spec is terse and contains no code bodies, describing conventions by path reference.
