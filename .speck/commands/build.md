---
description: "Refine specs and build artifacts toward a goal, optionally iterating with tests."
---

# speck build
<!-- Built from spec/features/build/build.md Spec(2.1) -->

**Uses**:
.speck/systems/spec-model.md
.speck/systems/traversal.md
.speck/rules/feature-system.md
.speck/rules/style.md
.speck/rules/inbox.md
.speck/rules/git-checkpoints.md

Primary command: how to build with speck. Absorbs former `plan` (spec refinement), `iterate` (test loop).

## Input

`speck build [files...] [iterations] {prompt}`

* `files...` — optional, non-exhaustive list of affected specs or artifacts. Artifact given → derive owning spec by search (see `.speck/systems/traversal.md`). None given → derive scope from prompt.
* `iterations` — optional integer ≥ 1. Present → run build→test→refine cycles. Absent → single refine+build pass, no test loop.
* `prompt` — primary goal: what to build or achieve, ideally with acceptance criteria.

Distill prompt into testable, actionable requirements. Resolve ambiguity aggressively via inbox (see `.speck/rules/inbox.md`) before proceeding on a blocked spec.

## Process

Git checkpoint before operation (skip if working tree clean — see `.speck/rules/git-checkpoints.md`).

1. **Scope**: from `files...` (and derived owning specs) + prompt, determine the set of affected specs. Read each + its `Uses` closure per strict reading (see `.speck/systems/traversal.md`).
2. **Refine** the affected spec subgraph breadth-first (see `.speck/systems/traversal.md`): run **Refine Cycle** on all specs at a level before descending to their subspecs.
3. **Build** the affected spec subgraph breadth-first: run **Build Cycle** on all specs at a level before descending.
4. `iterations` absent → done (skip to checkpoint). Present → run **Iterate Loop** up to `iterations` times.

Git checkpoint after operation.

Spec blocked (inbox pending) → report partial completion listing blocked specs.

**Refine Cycle** (one spec, no artifacts written):
1. Read spec + `Uses` closure.
2. Refine toward goal. Scan: scope, requirements (actionable; no vague adjectives), dependencies (`Uses` matches actual; no upward refs), edge cases, terminology, logical consistency, architecture (clear ownership).
   * Extract DRY: shared content across specs → factor into a rule; apply downward (see `.speck/rules/feature-system.md`, `.speck/rules/style.md`).
   * Update `Uses` edges: feature touches a system → `Uses` it (see `.speck/rules/feature-system.md`).
   * Resolvable from content → fix inline. Needs user input → inbox (see `.speck/rules/inbox.md`); mark spec blocked; skip its subspecs.
3. Validate checklist (see `.speck/systems/spec-model.md`): absent → create from `.speck/templates/template-checklist.md` covering scan categories; exists → update. Evaluate each item; fix failures by updating spec.
4. Subspecs from `Output > Subspecs`: absent → create stub from `.speck/templates/template-spec.md`. Queue subspecs for the next breadth-first level.
5. Content changed (steps 2–3) → increment version (see `.speck/rules/style.md`); set Status `Planned({version})`. No change + Draft → `Planned({version})`. No change + Planned/Built → unchanged.

**Build Cycle** (one spec, artifacts written):
1. Read spec + `Uses` closure.
2. Staleness: Status `Built({v})` and current version = `{v}` → not stale → report; skip artifact build; queue subspecs. Else → proceed.
3. Build all `Output > Artifacts` from the spec's `## Spec`. Artifact = standalone, agent-readable; self-contained per its `Uses` (no `spec/` dependency at runtime).
4. Set Status `Built({version})`.
5. Queue subspecs from `Output > Subspecs` for the next breadth-first level.

**Iterate Loop** (`iterations` = max):
Initialize `iteration = 0`.
1. `iteration` ≥ max → report final status; stop.
2. `iteration += 1`.
3. Run Build Cycle over the affected subgraph (breadth-first).
4. Locate + run tests:
   a. Find root project spec: search ancestors (see `.speck/systems/traversal.md`) until a spec with no ancestor.
   b. Identify the test subspec in root's `Output > Subspecs`: path under `tests/` or Purpose containing `test`/`validate`/`verify`. Exactly one expected.
   c. Read its `Output > Artifacts`; first artifact = test runner. Execute per its content; collect output.
   d. None found or ambiguous → warn; treat as pass; stop.
5. All tests pass → checkpoint; report success (iterations used); stop.
6. Classify each failure:
   * **Architectural** — design issue across artifacts → Refine Cycle on affected spec(s); then Build Cycle.
   * **Implementation** — artifact diverges from spec → edit artifact directly (no spec change, no version bump).
   * Ambiguous → treat as implementation; escalate next iteration.
7. No change produced this iteration → fixpoint; report; stop.
8. Checkpoint (end of iteration). Return to step 1.

## Rules

- Prompt = goal; structure the work to achieve it. No "address prompt then proceed" separation.
- Refine before build: specs consistent before artifacts generated.
- `Uses` targets read for context, never built; only `Output > Subspecs` traversed.
- Artifacts exist → overwrite (idempotent). Not stale → skip build, still traverse subspecs.
- Iterations enforced strictly; never exceeded. Checkpoint after each completed iteration, not per sub-step.
- Implementation fixes edit artifacts directly; do not re-run Build Cycle for them (avoids stale-skip on unchanged version).
- No upward `Uses`; extract shared concerns into rules (see `.speck/systems/traversal.md`).
