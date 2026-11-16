# Changelog

## 0.4.0

### Breaking (0.x)

- **The Arclite frame format changed** to the one now defined in
  `spec/features/protocols/arclite.md`. It doesn't interoperate with 0.3.0
  Arclite. Nothing used the old format outside this library, so the version
  stays 0.x.
  - The header is still 520/520 µs, but it is now found by its mark plus
    space, which receiver stretch doesn't move.
  - A 1 bit is now mark then space, and a 0 bit space then mark (both were
    the other way round).
  - The 16-bit message is now a 14-bit code followed by two interleaved
    parity bits, p1 and p0.
  - The minimum gap is 2 ms (was 2.5 ms), sent as a 2000 µs footer space
    that is part of the frame. A frame is 10.64 ms.
  - `ArcliteTagCode::send` and `send_async` refuse a code over `0x3FFF` with
    `ESP_ERR_INVALID_ARG`.
  - Receive decodes with the spec's fixed duration windows, not the codec's
    ±25% tolerance, and checks both parity bits. The receiver's end-of-frame
    threshold is 1500 µs.
- **`IrRxSource`: `IrRxFrameCallback` gains `int64_t last_edge_us`,** the
  `esp_timer` time of the frame's last edge. Every `IrRxSource`
  implementation and every registered callback must change.
  - The frame's first edge is that time minus the sum of the symbols'
    durations.
  - `RmtRxSource` passes the time of its RX-done ISR minus the end-of-frame
    threshold. Other sources must pass the time of the frame's last edge.

### Changed

- **`IrTransmitter` queues frames.**
  - `send_async` accepts up to `trans_queue_depth` frames (one in flight,
    the rest queued, at most `kMaxQueueDepth` = 4) and refuses the next one
    at once with `ESP_ERR_INVALID_STATE`.
  - It copies the data into its own slot, so the caller's buffer need not
    outlive the call.
  - Previously one frame was in flight and every other send was refused.
  - `is_busy()` is true while any frame is queued or in flight.
- The encoder's state and `RmtRxSource`'s symbol buffer are allocated in
  internal RAM.

### Added

- `arclite_frame.hpp`: the Arclite data word (`arclite_word_from_code`,
  `arclite_code_from_word`, `arclite_word_parity_ok`, `arclite_word_bytes`),
  the frame encoder (`arclite_encode`) and `ArcliteDecoder`.
  `ArcliteDecoder` is the receive decoder. It works on a single run or on a whole
  burst of RMT symbols, and reports ok, parity or timing results with the
  runs as received. Both RMT RX and software receive paths feed it.
- `IrTransmitter::set_done_callback` registers a callback that the TX-done
  ISR calls after each frame.
- `IrReceiver::set_end_of_frame_us` overrides the end-of-frame threshold
  `set_protocol` computed; `ArcliteTagCode::init` sets the spec's 1500 µs.
- `IrReceiver::receive_symbols` returns a frame's symbols and last-edge time
  as the source delivered them, for protocols that decode symbols themselves.
- Kconfig `IR_LASERTAG_ISR_IRAM_SAFE` places the encoder and the RX restart
  path in IRAM. It also selects ESP-IDF's `RMT_TX_ISR_CACHE_SAFE`,
  `RMT_RX_ISR_CACHE_SAFE` and `RMT_RECV_FUNC_IN_IRAM`, so IR keeps running
  while flash is written.
- Host tests cover the Arclite codec against every case in its spec,
  with each decode case run through both receive paths' symbol layouts.
  They also check the real encoder's symbols for known words (bit order).

### Fixed

- **Builds with ESP-IDF 6.x.** The component now `REQUIRES esp_driver_rmt
  esp_driver_gpio` instead of the legacy `driver` component, which no longer
  provides the RMT and GPIO headers. The `examples/profiling` app builds
  under IDF 6.1's C++ warnings.
- `IrTransmitter::send_raw` freed its symbol buffer while the driver could
  still be reading it. It now frees the buffer only once the transaction is
  done; on a timeout it disables the channel first.
- `IrTransmitter::send_async` no longer resets the encoder itself. The
  driver resets it at the start of each transaction, and resetting it while
  a frame was queued could corrupt that frame.
