# ESP-IDF Component Conventions
**Version**: 1.0
**Purpose**: Component structure, namespaces, file naming, and consumption conventions for this ESP-IDF library.
**Status**: Built(1.0)

**Uses**:

## Output
Subspecs:

Artifacts:

## Spec

### Component

Single ESP-IDF component registered via `idf_component_register` in the root `CMakeLists.txt` (SRCS = codec sources + compiled protocol sources; `INCLUDE_DIRS = include`; `REQUIRES = log driver freertos esp_timer`). Public headers under `include/ir_lasertag/`, split into `codec/` and `protocols/<name>/`. Implementation under `src/` mirroring the include tree. C++17, ESP-IDF ≥ v5.3, any ESP32 variant with an RMT peripheral. License MIT.

Codec and protocol layers live in one component (not two): protocol wrappers are thin and tightly coupled to codec types (`IrMessage`, `ProtocolConfig`); splitting adds overhead without reuse benefit.

### Namespaces

| Namespace | Scope |
|---|---|
| `ir_lasertag::codec` | codec layer: `ProtocolConfig`, `HalfBit`, `IrMessage`, `RawTiming`, `IrEncoder`/`ir_encoder_create`, `IrTransmitter`, `IrReceiver`, `IrRxSource`, `RmtRxSource` |
| `ir_lasertag::protocols` | protocol wrappers: `ArcliteTagCode`, `Irt`, `BcGun`, MilesTag constants |

### File Naming

`snake_case.cpp` / `snake_case.hpp` for C++ sources/headers; `kebab-case.md` or `README.md` for docs; `snake_case` directories. Each protocol directory provides a top-level `<protocol>.hpp` as the single consumer include (see `spec/rules/protocol-authoring.md`).

### Consumption

Library is consumed via the ESP component manager (`idf_component.yml` git dependency) or by cloning into a consuming project's `components/`. `idf_component.yml` carries name, version, description, and the ESP-IDF version constraint. Downstream consumers are not assumed to use Speck; the public headers and READMEs are the consumer-facing contract.
