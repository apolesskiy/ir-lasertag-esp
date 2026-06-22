---
description: "Command: create one top-level spec from template and user prompt."
---

# speck init
<!-- Built from spec/features/init/init.md Spec(2.0.1) -->

**Uses**:
.speck/systems/spec-model.md
.speck/rules/style.md
.speck/rules/git-checkpoints.md

## Input

Required: spec path.
Optional: user prompt (shapes spec content).

Prompt = what to build. Project-initialization command.

MUST address user prompt before proceeding.

## Process

Git checkpoint before operation (skip if working tree clean — see `.speck/rules/git-checkpoints.md`).

1. Spec file at given path already exists → report error to user; stop. No git checkpoint after (no files changed).
2. Read `.speck/templates/template-spec.md`.
3. Fill from spec path, user prompt (see `.speck/systems/spec-model.md` format):
   - Name — from path stem.
   - Purpose — ≤ one sentence from user prompt.
   - Initial Spec content — top-level requirements from user prompt.
   - `Uses` — leave blank (populated when building).
4. Set Status `Draft`. Version `1.0`.
5. Create parent directories if absent.
6. Write spec file.

Git checkpoint after operation.

## Rules

- Non-recursive: creates one spec. No subspecs.
- Do not prefill Output section — outputs defined later by `build`.
- Spec already exists at target path → report error; do not overwrite.
