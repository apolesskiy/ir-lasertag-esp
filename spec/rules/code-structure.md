# Code Structure & Definition of Done
**Version**: 1.0
**Purpose**: Change-granularity, documentation, and Definition-of-Done rules for library changes.
**Status**: Built(1.0)

**Uses**:
spec/rules/testing.md

## Output
Subspecs:

Artifacts:

## Spec

### Change Granularity

Prefer small, iterative changes. Keep function/method length under 50 lines where possible. Each public function/method has a documentation comment (see `spec/rules/cpp-style.md`).

### Commits

Commit messages start with `[Agent]` and describe the change.

### Definition of Done

- New functionality is covered by tests (see `spec/rules/testing.md`).
- All host tests pass.
- Code conforms to the style guide (`spec/rules/cpp-style.md`).
- All public API is documented.
