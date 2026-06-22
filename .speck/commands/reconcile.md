---
description: "Propagate manual changes to affected specs."
---

# speck reconcile
<!-- Built from spec/features/reconcile/reconcile.md Spec(2.0.1) -->

**Uses**:
.speck/systems/traversal.md
.speck/systems/spec-model.md
.speck/rules/feature-system.md
.speck/rules/inbox.md
.speck/rules/git-checkpoints.md

## Input

`speck reconcile [files...] {prompt}`

* `files...` — non-exhaustive list of manually changed specs or artifacts. Artifact given → derive owning spec by search (see `.speck/systems/traversal.md`).
* `prompt` — description of changes to reconcile (spec edits, file relocations, artifact/implementation changes).

## Process

Git checkpoint before operation (skip if working tree clean — see `.speck/rules/git-checkpoints.md`).

1. Read each named spec + its links for context (`Uses` closure, `Output > Subspecs`, discovered neighbors per `.speck/systems/traversal.md`).
2. Determine affected scope breadth-first (see `.speck/systems/traversal.md`) from named specs, in all relevant directions:
   * **Subspecs** — from `Output > Subspecs`.
   * **Ancestors** — specs listing a named spec in their `Output > Subspecs` (found by search).
   * **Matrix dependents** — for a changed system, features whose `Uses` lists it (found by search; see `.speck/rules/feature-system.md`).
   * Each candidate: does change logically affect it? Expand until no new affected specs. Circular `Output` ancestry → error; stop.
3. No affected specs → report success; stop. No files modified.
4. Each affected spec:
   a. Analyze conflict between change, spec content.
   b. Resolvable → apply fix (incl. updating `Uses` edges, relocating paths).
   c. Not resolvable → inbox (see `.speck/rules/inbox.md`); continue others.
5. Set Status of each modified spec (and named spec unless already `Draft`) to `Draft` — changes require rebuild via `build` (see `.speck/commands/build.md`).

Git checkpoint after operation.

## Rules

- Scope = affected specs in all directions: subspecs, ancestors, matrix dependents.
- `Uses` targets not modified unless themselves a named/affected spec.
- Conflict resolution: resolve if possible; else inbox.
- Circular `Output` ancestry → error; stop.
- No affected specs → report success; modify nothing.
