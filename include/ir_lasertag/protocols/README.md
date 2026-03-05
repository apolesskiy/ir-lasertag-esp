# Protocol Implementation Guidelines

## Directory Structure

Each protocol lives in its own subdirectory under `protocols/`:

```
protocols/
├── README.md              # This file
├── irt/
│   ├── irt.hpp        # Top-level include
│   ├── README.md          # Protocol documentation
│   └── (additional headers if needed)
├── milestag/
│   ├── milestag.hpp
│   ├── README.md
│   └── ...
└── arclite/
    ├── arclite.hpp
    ├── README.md
    └── ...
```

Corresponding source files go in `src/protocols/<protocol_name>/`.

## Requirements

### Top-Level Header

Every protocol **must** provide a top-level header named `<protocol_name>.hpp`
at the root of its subdirectory. This is the single include for consumers:

```cpp
#include "ir_lasertag/protocols/irt/irt.hpp"
```

### README

Every protocol **must** include a `README.md` with:

1. **Conceptual Overview** — A single paragraph describing what the protocol is,
   who uses it, and its general purpose. Keep it brief; readers are expected to
   do their own research for in-depth understanding.

2. **IR Encoding** — A table or section covering:
   - Encoding type (pulse-width, pulse-distance, Manchester, etc.)
   - Header mark/space timings
   - Bit timings (mark, space, half-bit durations as applicable)
   - Message size (bit count)
   - Message gap / repeat interval
   - Any protocol-specific timing notes (e.g., paired transmission)

3. **API Summary** — A list of the public functions/classes exposed by the
   protocol header. One line per entry with a brief description. Do **not**
   duplicate detailed parameter documentation from the header — the header
   is the authoritative source for API details.

### Example README Structure

```markdown
# <Protocol Name>

## Overview

<One paragraph describing the protocol.>

## IR Encoding

| Parameter      | Value         |
|----------------|---------------|
| Encoding       | Pulse-width   |
| Header mark    | 500 µs        |
| Header space   | 500 µs        |
| Zero mark      | 350 µs        |
| One mark       | 700 µs        |
| Bit space      | 450 µs        |
| Message size   | 16 bits       |
| Message gap    | 2000 µs       |

## API

- `IrtGun::send(uint16_t id)` — Transmit a gun packet.
- `IrtGun::receive()` — Receive and validate a paired gun packet.
```

### Header Documentation

Individual API documentation (parameter descriptions, return values, error
conditions, usage examples) belongs in the header file as doc-comments.
The README links readers to the header for details; it does not repeat them.

## Implementation Notes

- Protocol wrappers use the codec layer (`ir_lasertag/codec/`) internally.
- Wrappers run in task context (not ISR). Multi-packet sequences use
  blocking `IrTransmitter::send()` with appropriate delays.
- Protocol-specific types (message structs, enums) are defined in the
  protocol header, not in the codec layer.
- The `protocol_registry` in `main/` holds `ProtocolConfig` entries for
  codec-level configuration. Protocol wrappers reference these configs
  but do not own them.
