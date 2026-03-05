# ir_lasertag

IR laser tag library for ESP32. Provides both low-level IR codec and
protocol-specific APIs for laser tag systems.

## Features

- **Half-bit model codec**: Configurable IR encoding/decoding supporting
  Manchester, pulse-width, and pulse-distance encoding schemes.
- **ESP32 RMT peripheral**: Hardware-accelerated IR transmit and receive
  via the RMT driver.
- **Protocol wrappers**: High-level protocol APIs with proper message
  semantics (paired transmission, field access, etc.).

## Supported Protocols

| Protocol | Encoding | Bits | Description |
|----------|----------|------|-------------|
| Arclite | Manchester | 16 | Tag Code collision detection |
| IRT Gun | Pulse-width | 16 | Gun ID with paired transmission |
| MilesTag | Pulse-distance | 14 | Player/team/damage standard |

## Installation

### ESP Component Registry

```yaml
# idf_component.yml
dependencies:
  apolesskiy/ir-lasertag:
    git: https://github.com/apolesskiy/ir-lasertag-esp.git
```

### Manual

Clone into your project's `components/` directory:

```bash
cd components
git clone https://github.com/apolesskiy/ir-lasertag-esp.git ir-lasertag
```

## Quick Start

```cpp
#include "ir_lasertag/codec/ir_transmitter.hpp"
#include "ir_lasertag/codec/ir_receiver.hpp"
#include "ir_lasertag/protocols/arclite/arclite.hpp"

using namespace ir_lasertag;

// Initialize transmitter
codec::IrTransmitter tx;
rmt_tx_channel_config_t tx_config = {
    .gpio_num = GPIO_NUM_4,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000,  // 1 MHz
    .mem_block_symbols = 64,
    .trans_queue_depth = 4,
};
tx.init(tx_config);

// Initialize receiver
codec::IrReceiver rx;
rmt_rx_channel_config_t rx_config = {
    .gpio_num = GPIO_NUM_5,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000,
    .mem_block_symbols = 128,
};
rx.init(rx_config);

// Use Arclite Tag Code protocol
protocols::ArcliteTagCode atc;
atc.init(&tx, &rx, protocols::kArcliteConfig);

// Send a tag code
atc.send(0x1234);

// Receive tag codes
atc.start_receive();
uint16_t tag_code;
if (atc.try_receive(&tag_code)) {
  ESP_LOGI("app", "Received: 0x%04X", tag_code);
}
```

## Architecture

```
ir_lasertag/
├── include/ir_lasertag/
│   ├── codec/          # Low-level IR encode/decode (half-bit model)
│   │   ├── ir_protocol.hpp    # Core types: ProtocolConfig, IrMessage, etc.
│   │   ├── ir_encoder.hpp     # RMT encoder factory
│   │   ├── ir_transmitter.hpp # IR transmitter (async/sync send)
│   │   └── ir_receiver.hpp    # IR receiver (decoded/raw modes)
│   └── protocols/      # Protocol-specific message APIs
│       ├── arclite/arclite.hpp  # Arclite TagCode wrapper
│       ├── irt/irt.hpp          # IRT Gun wrapper
│       └── milestag/milestag.hpp # MilesTag constants
└── src/
    ├── codec/          # Codec implementation
    └── protocols/      # Protocol wrapper implementation
```

### Codec Layer

The codec layer provides configurable IR encoding/decoding using a
half-bit model. Each data bit consists of two half-bit periods with
independently specified durations and levels. This supports:

- **Manchester**: Transition-based (zero and one have opposite first-half levels)
- **Pulse-width**: Mark duration varies (e.g., IRT: 350µs vs 700µs mark)
- **Pulse-distance**: Space duration varies (e.g., MilesTag: 600µs vs 1200µs space)

### Protocol Layer

Protocol wrappers provide type-safe, protocol-correct APIs:

- **IRT Gun**: Sends 16-bit packets in identical pairs with 2ms gap.
- **MilesTag**: Structured field access for team/player/damage data.
- **Arclite**: TagCode send/receive with Manchester encoding.

## API Reference

### Codec

See header files for full parameter documentation:

- [`ir_protocol.hpp`](include/ir_lasertag/codec/ir_protocol.hpp) — `ProtocolConfig`, `IrMessage`, `RawTiming`, `HalfBit`
- [`ir_encoder.hpp`](include/ir_lasertag/codec/ir_encoder.hpp) — `ir_encoder_create()`
- [`ir_transmitter.hpp`](include/ir_lasertag/codec/ir_transmitter.hpp) — `IrTransmitter` class
- [`ir_receiver.hpp`](include/ir_lasertag/codec/ir_receiver.hpp) — `IrReceiver` class

### Protocols

- [`arclite.hpp`](include/ir_lasertag/protocols/arclite/arclite.hpp) — `ArcliteTagCode` class
- [`irt.hpp`](include/ir_lasertag/protocols/irt/irt.hpp) — `IrtGun` class
- [`milestag.hpp`](include/ir_lasertag/protocols/milestag/milestag.hpp) — `kMilestagConfig` constants

## Requirements

- ESP-IDF v5.3.0 or later
- ESP32, ESP32-S2, ESP32-S3, or ESP32-C3 (any target with RMT peripheral)
