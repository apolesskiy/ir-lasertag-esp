# Spec Model
<!-- Built from spec/systems/spec-model/spec-model.md Spec(1.0.1) -->

**Uses**:
.speck/rules/style.md

Spec model is speck's core data structure: graph of spec files linked by `Uses`, decomposed by `Output > Subspecs`.

## File Format

Every spec/rule follows one template (`.speck/templates/template-spec.md`):

```
# {Name}
**Version**: {semver}
**Purpose**: {≤ one sentence}
**Status**: {Draft | Planned(semver) | Built(semver)}

**Uses**:
{path — one per line, no bullets/backticks; blank if none}

## Output
Subspecs:
{path — one per line; blank if none}

Artifacts:
{path — one per line; blank if none}

## Spec
{spec content}
```

## Spec Kinds

- **Feature** — user-facing behavior (PRD). `Uses` systems it touches, applicable rules.
- **System** — architecture/mechanism (design doc). `Uses` subsystems, applicable rules. Never lists consuming features in `Uses` (cycles); dependents found by search.
- **Rule** — cross-cutting concern reused via `Uses`. Outputs optional. Applied downward only.

All three share format. Header fields identical. Kind conveyed by location, role, not a header field.

## Checklist

Unit test for spec (not its artifacts). File: `{spec-dir}/{spec-name}.checklist.md`, created from `.speck/templates/template-checklist.md`. Tests clarity, content, completeness, logical consistency, style conformance. Header fields: Name, Version, Spec. Updated whenever its spec changes.

## Link Semantics

- `Uses` = context required to understand this spec. Read recursively (see `.speck/systems/traversal.md`). Downward/lateral only — never parent/ancestor.
- `Output > Subspecs` = specs this spec decomposes into. NOT context dependencies — not auto-read when reading this spec; traversed by recursive commands.
- `Output > Artifacts` = files built from this spec. Each artifact owned by one spec; no overlap with subspec artifacts.

## Lifecycle

Status tracks spec's outputs:

- `Draft` — no outputs created.
- `Planned(v)` — subspecs defined; no artifacts. Set when specs refined, artifacts not built.
- `Built(v)` — artifacts built at version `v`.
- Spec content edited → version incremented (see `.speck/rules/style.md` version policy).
- Stale = current version > `Built` semver → artifacts out of date.

## Decomposition

Minimum spec = one implementation task. Larger or more abstract → split into subspecs listed in `Output > Subspecs`. Parent owns its scope; children own theirs. Analogy: CMakeLists per directory.
