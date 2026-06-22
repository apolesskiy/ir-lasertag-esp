# Git Checkpoint Rules
<!-- Built from spec/rules/git-checkpoints/git-checkpoints.md Spec(1.2.1) -->

Atomic, auditable commits before + after each speck operation. Before-commit = user revert point; after-commit = completed state capture.

## When to Checkpoint

Create before + after checkpoint for each operation that writes files:

- `speck init` — new spec file written.
- `speck build` — specs refined/created; checklists written; artifacts built; Status updated; across iterations when iterating.
- `speck reconcile` — spec files updated.
- `speck reorg` — specs, artifacts relocated/updated.

Do not checkpoint when only reading files or when no files changed.

## Before Operation

1. Check for uncommitted changes in working tree.
2. Changes present → commit all: `speck checkpoint: before {command} {spec-name}`.
3. Clean working tree → skip.

## After Operation

1. Files changed → commit all using Commit Message Format below.
2. No files changed → skip.

Inbox questions written → commit immediately after writing, separate from other changes:
`speck inbox: {spec-name}`

## Commit Message Format

After-operation commit:
```
[Speck] {Command}: {prompt-summary}: {changes-made}
```

Before-operation commit:
```
speck checkpoint: before {command} {spec-name}
```

- `{Command}` — capitalized: `Init`, `Build`, `Reconcile`, `Reorg`.
- `{prompt-summary}` — 2–5 word digest of user prompt or spec context (what was targeted).
- `{changes-made}` — 2–5 word description of what actually changed.

Examples:
```
[Speck] Build: robustness: 19 prompt gaps fixed
[Speck] Build: project subspecs: features and systems added
[Speck] Reorg: split commands: features/systems tree created
speck checkpoint: before build project
```

## Rules

- Before-commit covers all pre-existing uncommitted changes in working tree, not only operation-related files.
