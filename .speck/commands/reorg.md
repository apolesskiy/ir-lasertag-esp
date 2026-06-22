---
description: "Command: reorganize specs and artifacts, keeping the graph consistent."
---

# speck reorg
<!-- Built from spec/features/reorg/reorg.md Spec(1.1.1) -->

**Uses**:
.speck/systems/traversal.md
.speck/systems/spec-model.md
.speck/rules/feature-system.md
.speck/rules/style.md
.speck/rules/inbox.md
.speck/rules/git-checkpoints.md

Proactive, reliable counterpart to "manually move files + reconcile". Restructures spec graph, its artifacts in one operation, repairing all links.

## Input

`speck reorg {prompt}`

- `prompt` — desired reorganization: e.g. split spec, merge specs, move subtree, re-partition along features × systems.

MUST address user prompt before proceeding.

## Process

Git checkpoint before operation (skip if working tree clean — see `.speck/rules/git-checkpoints.md`).

1. **Map**: build current graph for affected region. Discover specs, `Uses` edges, `Output > Subspecs`, ancestors, matrix dependents by search (see `.speck/systems/traversal.md`).
2. **Plan moves**: from prompt, determine target structure (new paths, splits, merges, axis re-partition per `.speck/rules/feature-system.md`). Ambiguous → inbox (see `.speck/rules/inbox.md`); stop affected region; continue others.
3. **Apply**: relocate/split/merge spec files, their owned artifacts together (each artifact owned by exactly one spec — see `.speck/systems/spec-model.md`). Moved spec's owned artifact files move to track spec's new location; update spec's `Output > Artifacts` paths to match. Create parent dirs; remove emptied ones.
4. **Repair links**: update every reference to a moved path — `Uses` entries, `Output > Subspecs`, `Output > Artifacts`, ancestors' subspec lists (found by search). Introduce no upward `Uses` (see `.speck/systems/traversal.md`).
5. **Restatus**: spec whose own content or structure changed → version bump (see `.speck/rules/style.md`), Status `Draft` (rebuild via `build` to regenerate artifacts). Pure relocation with consistent links → Status unchanged. Dependent touched only by a `Uses` link repair (no content/structure change) → Status unchanged; not a structural change.

Git checkpoint after operation.

## Rules

- Move spec, its owned artifacts together; never orphan an artifact.
- All references to moved paths repaired in same operation; graph never left dangling.
- No upward `Uses`; preserve features × systems edge direction (see `.speck/rules/feature-system.md`).
- Ambiguous reorganization → inbox; do not guess destructive moves.
- Structural change → `Draft` + rebuild; pure relocation → status unchanged.
