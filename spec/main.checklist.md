# CHECKLIST: ir-lasertag-esp
**Version:** 1.0
**Spec**: /win/e/projects/arclite_hw/new-arclite-firmware/ir-lasertag-esp/spec/main.md

## Scope
[ ] CHK001 The Purpose line states a single, clear product scope (configurable IR codec plus protocol-specific laser-tag APIs over ESP32 RMT) without ambiguity.
[ ] CHK002 The spec scopes the product as a standalone, reusable library whose consumers are not assumed to use Speck, and this boundary is stated explicitly.

## Decomposition / Subspecs coverage
[ ] CHK003 Every layer named in Architecture (codec, protocol) maps to a subspec or system listed in the Output section.
[ ] CHK004 Every protocol named in the Protocols table (Arclite, IRT, BC, MilesTag) has a corresponding subspec path listed in Output.
[ ] CHK005 The Decomposition section's categories (Systems, Foundational features, Per-protocol features, Rules, Examples & tests) are each backed by entries in the Output Subspecs list.
[ ] CHK006 Every rule referenced in Decomposition is present in the Uses list and the Output Subspecs list consistently.

## Artifacts
[ ] CHK007 READMEs are listed as artifacts in the Output section (README.md appears under Artifacts).

## Dependencies
[ ] CHK008 The Uses list references only sibling or downward specs/rules (no upward references to a parent spec).

## Consistency
[ ] CHK009 The MilesTag "config-only, no wrapper class" statement is consistent with the Protocols table (which lists wrapper classes only for Arclite, IRT, BC).
[ ] CHK010 Version, Status, and ESP-IDF version constraint (≥ v5.3) stated here do not contradict the values in the referenced rule specs.
