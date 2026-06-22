# Feature-System Rules
<!-- Built from spec/rules/feature-system/feature-system.md Spec(1.0.1) -->

Organize specs as a features × systems matrix, navigate the relationship. Applied by every feature, system spec.

## Axes

- **Feature** — user-facing behavior; "described by a PRD". What user invokes or observes.
- **System** — underlying architecture; "described by a design doc". How a behavior is realized.

## The Matrix

Features × systems form a matrix. Edges declared one direction only, to keep reading acyclic (see `.speck/systems/traversal.md`):

- Feature `Uses` every system it touches. (Feature's row.)
- System does NOT list features in `Uses`. (Column derived, not stored.)

Navigation (dependency tools):

- **Systems a feature touches** = feature's `Uses` entries pointing at system specs. Read directly.
- **Features touching a system** = specs whose `Uses` lists that system. Found by searching `Uses` links (see `.speck/systems/traversal.md`). System's impact set.

Implications:

- Change a feature → may add/affect one or more systems (edit feature's `Uses`).
- Change a system → affects every feature in its impact set → reconcile those features.

## Recursion

Features × systems is recursive, like spec graph. System large enough to have its own consumers exposes its own **features** (its API surface) backed by **subsystems**. At that scope same rule applies: sub-feature `Uses` its subsystems; subsystem's dependents found by search. Recursion stops when a system is one design (no internal API).

## Application

- Rule applied downward: parent spec owning this rule states "subspecs apply `.speck/rules/feature-system.md`", subspec `Uses` it. Never referenced upward.
- Feature, system specs both `Uses` this rule.
- Keep edges current: feature starts/stops touching a system → update its `Uses`.
