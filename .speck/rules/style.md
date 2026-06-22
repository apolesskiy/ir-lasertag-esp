# Style Rules
<!-- Built from spec/rules/style/style.md Spec(1.3) -->

Spec language = max information, min tokens. Machine-readable. Preserves all technical clarity.

## Writing

- Terse — elide articles (a, an, the) by default; keep only when removal → ambiguity. Omit filler.
- Series — "X and Y", "X or Y" → "X, Y"; keep "and"/"or" only when conjunction carries logic.
- Precise — no vague adjectives ("robust", "flexible", "clean") without measurable criteria.
- Active voice, present tense — "Agent reads file", not "File is read"; drop modals ("would", "will") when stating facts.
- Drop implied subject — "MUST read every `Uses` target", not "the agent MUST read".
- Fragments OK if meaning clear.
- Symbols — `→` causality/flow/trigger→action; `=` equivalence/definition; `≥`, `≤` comparisons. "When X, Y" → "X → Y".
- Abbreviations — common technical abbrevs freely: `semver`, `spec`, `cmd`, `dir`.
- Consistent — one term per concept; no synonyms within a spec.

## Structure

- Purpose: ≤ one sentence. Active, specific.
- `##` top-level sections; `###` sub-sections. No deeper nesting unless necessary.
- Bullets for enumerations; numbered lists for ordered steps only.
- No "Note:", "Important:", "Please" — content only.
- No duplication of content from linked specs — reference instead.

## Omit

- Motivation/rationale unless needed to resolve requirements ambiguity.
- Implementation details unless *how* is the requirement.
- Boilerplate: no disclaimers, preambles, summaries.
- Emphatic tails: "exactly", "simply", "just", "nothing more", "at all".

## Version Policy

- Increment version on content change: patch for wording-only; minor for requirements/structure; major for redesign.

## Rule Application

- Rules apply downward only. Parent spec owning a rule states "subspecs apply {rule-path}"; subspec lists rule in `Uses`. Never reference parent/ancestor in `Uses`.
- Extract DRY: shared content across specs → factor into rule; reference, never repeat.
