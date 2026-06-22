# C++ Style
**Version**: 1.0
**Purpose**: C++ coding style for the library — Google C++ Style Guide with ESP-IDF modifications.
**Status**: Built(1.0)

**Uses**:

## Output
Subspecs:

Artifacts:

## Spec

This spec is the authoritative C++ style guide for the library. 2-space indentation, no tabs. `.cpp` for C++ sources, `.hpp` for C++ headers. `#pragma once` header guards. Prefer `constexpr` over `#define` for constants.

### Naming

| Element | Convention |
|---|---|
| Classes / structs | `PascalCase` |
| Functions / methods | `snake_case` |
| Variables | `snake_case` |
| Constants / macros | `UPPER_SNAKE_CASE` |
| Enum values | `kPrefixValue` |
| Private members | trailing underscore (`name_`) |

### Documentation

Each public function/method/class has a doc comment (`@brief` and parameter/return docs). The header is the authoritative source of API detail; READMEs summarize and link to headers, never duplicate parameter docs (see `spec/rules/protocol-authoring.md`).
