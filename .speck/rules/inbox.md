# Inbox Rules
<!-- Built from spec/rules/inbox/inbox.md Spec(1.0.1) -->

Async, auditable user interaction. Use when user input needed during spec operations.

## File Format

Location: `inbox/{spec-name}-questions.md`

Content:
```
# Questions: {Spec Name}
**Spec**: {path to spec}

- [ ] Q1: {question text}
- [ ] Q2: {question text}
```

## Process

1. Write questions as unchecked items (`- [ ]`). Commit immediately: `speck inbox: {spec-name}`.
2. Stop work on blocked spec; continue unblocked specs.
3. On resume: process only checked (`- [x]`) items. Leave unchecked for next cycle.
4. All items processed → delete inbox file.

## Rules

- One file per spec; stem = spec filename stem.
- Do not proceed on blocked spec until inbox processed.
- Questions specific, actionable.
- Do not accumulate unanswered questions across cycles.
