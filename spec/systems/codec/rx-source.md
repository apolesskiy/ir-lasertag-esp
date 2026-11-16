# RX Source

**Version**: 1.0
**Purpose**: Pluggable symbol-frame source interface (`IrRxSource`) decoupling the receiver from hardware, plus its reference RMT backend (`RmtRxSource`).
**Status**: Built(1.0)

**Uses**:
.speck/rules/feature-system.md
spec/rules/cpp-style.md

## Output
Subspecs:

Artifacts:
include/ir_lasertag/codec/ir_rx_source.hpp
include/ir_lasertag/codec/rmt_rx_source.hpp
src/codec/rmt_rx_source.cpp

## Spec

### IrRxSource Interface

`IrRxSource` = abstract pluggable source of complete IR symbol frames. It decouples `IrReceiver` (decode) from the capture backend, so an RMT peripheral, a software receiver, or a test fixture can feed the same decoder. A frame = an array of `rmt_symbol_word_t` delivered through a registered callback.

| Member | Contract |
|---|---|
| `set_on_frame_callback(IrRxFrameCallback cb, void* ctx)` | register the frame-complete callback; the backend must invoke `cb(symbols, count, last_edge_us, ctx)` on every complete frame |
| `start()` | begin detecting IR edges and delivering frames |
| `stop()` | stop delivering frames |
| `set_max_symbol_duration_us(uint16_t)` | set the end-of-frame threshold (below) |

`IrRxFrameCallback` = `void (*)(const rmt_symbol_word_t* symbols, size_t count, int64_t last_edge_us, void* ctx)`. `last_edge_us` is the `esp_timer` time of the frame's last edge, so a consumer can time the frame from its own symbols: its first edge is `last_edge_us` minus the sum of the symbols' durations. A backend that knows the edge's time passes it; one that only knows when it detected the end of the frame subtracts its end-of-frame threshold. Virtual destructor; implementations own backend resources.

### End-Of-Frame Threshold

`set_max_symbol_duration_us(duration_us)` defines the frame boundary: a single mark or space longer than this duration ends the current frame. This is distinct from the inter-transmission gap. Set it slightly above the longest expected pulse in the protocol (e.g. ~1800µs for a 1200µs longest pulse). The receiver computes and pushes this value from the protocol timings (see spec/systems/codec/ir-receiver.md). Implementations translate it to native backend units.

### RmtRxSource Reference Backend

`RmtRxSource : IrRxSource` = reference backend over one ESP32 RMT RX channel. Non-copyable; destructor calls `deinit()`. RMT runs in request-response mode: each frame requires re-arming `rmt_receive`, handled internally.

| Method | Effect |
|---|---|
| `init(const rmt_rx_channel_config_t&, size_t symbol_buffer_size = kDefaultSymbolBufferSize)` | allocate the symbol buffer, create the RMT RX channel, register the RX-done ISR; `ESP_ERR_INVALID_STATE` if already initialized, `ESP_ERR_NO_MEM` on buffer alloc failure |
| `deinit()` | stop if running, delete channel, free buffer, clear callback |
| `is_initialized()` | channel ready |
| `set_on_frame_callback` / `start` / `stop` / `set_max_symbol_duration_us` | `IrRxSource` overrides |

`kDefaultSymbolBufferSize` = 128 (`rmt_symbol_word_t` entries). RMT resolution is 1 MHz / 1 µs.

### RMT Receive Window & ISR

Each `rmt_receive` is armed with a receive window:

| RMT field | Value | Rationale |
|---|---|---|
| `signal_range_min_ns` | fixed ~1000 ns | ESP32 RMT requirement; pulses shorter than this are treated as noise/glitch |
| `signal_range_max_ns` | derived from `set_max_symbol_duration_us` (µs × 1000) | frame/packet separation = end-of-frame threshold |

`start()` enables the channel then arms the first receive. The RX-done ISR (`IRAM`) forwards the captured frame to the registered callback, with `last_edge_us` = `esp_timer_get_time()` minus the end-of-frame threshold (the channel reports a frame once the line has been idle that long; the error is the ISR latency), then immediately re-arms capture for the next frame (`restart_receive`, in IRAM). The symbol buffer is allocated in internal RAM, since the RX ISR copies into it. With `CONFIG_IR_LASERTAG_ISR_IRAM_SAFE` the option selects `RMT_RX_ISR_CACHE_SAFE` and `RMT_RECV_FUNC_IN_IRAM`; the registered callback then runs with the cache off too and must be in IRAM. The callback is expected to copy/queue the symbols quickly (the receiver does so for task-context decode — see spec/systems/codec/ir-receiver.md), making immediate re-arm safe.
