# Traversal
<!-- Built from spec/systems/traversal/traversal.md Spec(1.1) -->

**Uses**:
.speck/systems/spec-model.md

Commands traverse spec graph. Traversal rules keep context minimal, correct.

## Strict File Reading

`Uses` = context required to understand spec. Reading any spec → MUST read every file in its `Uses` section, recursively.

- Visited set: read each file at most once per operation; skip already-read targets. Terminates on cycles.
- Read only `Uses` targets — not `Output > Subspecs` (decomposition, traversed separately by command logic, not pulled as context).
- Read no irrelevant files at spec ingestion: do not read ancestors, siblings, unrelated specs unless reached through `Uses`. Read other files after full spec ingested, if ambiguity remains.

Goal: granular, automatic context management. Set of files read to understand spec X = X plus its transitive `Uses` closure.

## No Upward References

Specs MUST NOT list parents, ancestors in `Uses`. Upward references pull entire spec tree on every read, eliminate partitioning.

Shared concern between parent, child → parent defines rule, applies to child (child `Uses` rule). Cross-cutting context flows down via rules, never up via `Uses`. See `.speck/rules/feature-system.md`.

## Recursion Order

- **Breadth-first** (build, reconcile): evaluate all direct neighbors before any of their neighbors. Granular context at each level; siblings available; eases subagent instantiation. Used for impact propagation in both directions.

## Dependency Discovery

Spec graph navigated by search, not upward links:

- **Ancestors of X** = specs listing X in their `Output > Subspecs`. Found by searching spec files for X's path.
- **Dependents of system S** = specs listing S in their `Uses`. Found by searching spec files for S's path. The feature↔system matrix column (see `.speck/rules/feature-system.md`).
- **Spec for artifact A** = spec whose `Output > Artifacts` lists A. Found by searching spec files for A's path.

Use built-in file search/grep for discovery; no separate index maintained.

## Cycle Handling

- Reading: visited set prevents infinite loops; not an error.
- `Output > Subspecs` ancestry: true cycle (A subspec of B, B subspec of A) is an error; report, stop.
