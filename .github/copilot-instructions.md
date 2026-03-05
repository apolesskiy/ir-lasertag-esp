## Technology Stack
* Refer to `agent/technology-stack.md` for technology stack details.

## Agent-First Development
* Documentation should be readable by humans, but optimized for consumption by AI agents.
* Avoid including code in design documents.

## Style
* Use spaces for indentation. Indentation width is 2 spaces.
* Follow Google C++ Style Guide with ESP-IDF modifications.
* Use `.cpp` extension for C++ source files, `.hpp` for C++ headers.
* Naming conventions:
  - Classes/Structs: `PascalCase`
  - Functions/Methods: `snake_case`
  - Variables: `snake_case`
  - Constants/Macros: `UPPER_SNAKE_CASE`
  - Private members: `snake_case_` (trailing underscore)
  - Enum values: `kPrefixValue`
* Use `#pragma once` for header guards.
* Prefer `constexpr` over `#define` for constants.

## Code Structure
* Prefer small, iterative code changes.
* Keep function length under 50 lines where possible.
* Each public function/method must have a documentation comment.

## Testing
* Host-based tests in `host_test/` — test protocol config and encode/decode logic.
* Run: `cd host_test && cmake -B build && cmake --build build && ctest --test-dir build --output-on-failure`
* On-device tests require IR hardware (transmitter/receiver) and a second board.

## Definition of Done
* New functionality is covered by tests.
* All host tests pass.
* Code conforms to style guide.
* All public API documented.

## Progress Checkpoints
* Each commit description should start with "[Agent]".
