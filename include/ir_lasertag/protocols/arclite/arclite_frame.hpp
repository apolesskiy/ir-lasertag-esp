/**
 * @file arclite_frame.hpp
 * @brief Arclite frame codec: the data word, the frame's runs, and the
 *        receive decoder (spec/features/protocols/arclite.md).
 *
 * Pure logic with no driver calls, so it runs on the host. Both receive
 * paths (RMT RX and software) deliver `rmt_symbol_word_t` bursts, and both
 * decode through ArcliteDecoder.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/rmt_types.h"
#include "ir_lasertag/codec/ir_protocol.hpp"

namespace ir_lasertag {
namespace protocols {

/// @name Frame constants (arclite.md: "IR Encoding", "Decoder rules")
/// @{

/// Header mark in microseconds.
constexpr uint16_t kArcliteHeaderMarkUs = 520;

/// Header space in microseconds.
constexpr uint16_t kArcliteHeaderSpaceUs = 520;

/// Half of a Manchester bit in microseconds (T/2).
constexpr uint16_t kArcliteHalfBitUs = 300;

/// Data bits per frame (14-bit code, then p1, p0).
constexpr uint8_t kArcliteBitCount = 16;

/// Idle after a frame's last mark before the next frame (at least 2 ms).
constexpr uint32_t kArcliteMessageGapUs = 2000;

/// Largest code that fits the 14-bit code field.
constexpr uint16_t kArcliteCodeMax = 0x3FFF;

/// Parity masks over the code bits: p0 covers even positions, p1 odd.
constexpr uint16_t kArcliteParityEvenMask = 0x1555;
constexpr uint16_t kArcliteParityOddMask = 0x2AAA;

/// Receive end-of-burst idle threshold (above 1000 µs, below 2 ms; no
/// valid run is longer than 1000 µs).
constexpr uint16_t kArcliteRxIdleUs = 1500;

/// @name Receive windows (inclusive)
/// @{
constexpr uint16_t kArcliteGlitchUs = 150;        ///< shorter: a glitch
constexpr uint16_t kArcliteHalfMaxUs = 449;       ///< half-bit: 150–449
constexpr uint16_t kArcliteFullMaxUs = 749;       ///< full bit: 450–749
constexpr uint16_t kArcliteHeaderMarkMinUs = 400; ///< a header mark: 400–800
constexpr uint16_t kArcliteHeaderMarkMaxUs = 800;
/// Header mark plus header space, first bit a 1: 970–1119 (nominal 1040).
constexpr uint16_t kArcliteHeaderOneMinUs = 970;
constexpr uint16_t kArcliteHeaderOneMaxUs = 1119;
/// Header mark plus the header space merged with a leading 0 bit's space:
/// 1270–1410 (nominal 1340).
constexpr uint16_t kArcliteHeaderZeroMinUs = 1270;
constexpr uint16_t kArcliteHeaderZeroMaxUs = 1410;
/// @}

/// Upper bound on a frame's runs: header mark, header space, and 32
/// half-bits (an encoded frame has at most 33, since halves merge).
constexpr size_t kArcliteFrameMaxRuns = 34;

/// @}

/// @name Data word
/// @{

/**
 * @brief Builds the 16-bit data word for a code.
 * @param code The code; only its low 14 bits are used.
 * @return `(code << 2) | (p1 << 1) | p0`.
 */
uint16_t arclite_word_from_code(uint16_t code);

/**
 * @brief Extracts the code from a data word.
 * @param word The data word.
 * @return `word >> 2`.
 */
constexpr uint16_t arclite_code_from_word(uint16_t word) {
  return static_cast<uint16_t>(word >> 2);
}

/**
 * @brief Checks both parity bits of a data word.
 * @param word The data word.
 * @return true if p0 and p1 match the code bits.
 */
bool arclite_word_parity_ok(uint16_t word);

/**
 * @brief The data word as the 2 bytes IrTransmitter sends, MSB first.
 *
 * The codec's encoder walks bytes in memory order, so the word's high byte
 * goes first; a plain little-endian `uint16_t` would send the low byte
 * first.
 *
 * @param code The code; only its low 14 bits are used.
 * @param[out] bytes 2 bytes: the word's high byte, then its low byte.
 */
void arclite_word_bytes(uint16_t code, uint8_t bytes[2]);

/// @}

static_assert(kArcliteRxIdleUs > 1000 && kArcliteRxIdleUs < kArcliteMessageGapUs,
              "arclite.md: the end-of-frame idle is above 1000 µs and "
              "below 2 ms");

/**
 * @brief Encodes a frame as alternating mark/space runs.
 *
 * Header mark first. Adjacent half-bits of one level are merged (the header
 * space plus a leading 0 bit's space is one 820 µs space). Ends with the
 * last mark: a trailing space is the idle line. The inter-frame gap is not
 * included.
 *
 * @param code The code, at most kArcliteCodeMax.
 * @param[out] runs At least kArcliteFrameMaxRuns entries.
 * @return The number of runs, or 0 if the code does not fit in 14 bits.
 */
size_t arclite_encode(uint16_t code, codec::RawTiming* runs);

/**
 * @brief Decode outcome.
 */
enum class ArcliteResult : uint8_t {
  kOk = 0,      ///< Valid frame; `code` is set
  kParity = 1,  ///< 16 bits decoded, parity failed; `word` is set
  kTiming = 2,  ///< A run outside the receive windows, or a lost edge, after a header
};

/**
 * @brief One frame that got past a valid header mark.
 */
struct ArcliteFrame {
  ArcliteResult result = ArcliteResult::kTiming;
  uint16_t word = 0;  ///< The data word (kOk and kParity)
  uint16_t code = 0;  ///< The code (kOk only)
  /// The runs as received, from the header mark on (for raw capture).
  codec::RawTiming runs[kArcliteFrameMaxRuns + 1] = {};
  size_t run_count = 0;
};

/**
 * @brief Streaming receive decoder for one receiver channel.
 *
 * Fed run by run, or with a whole burst of RMT symbols. The windows are
 * inclusive.
 *
 * - **The header is found by its mark plus space.** A mark of 400–800 µs is
 *   a candidate header mark. With the space after it, a sum of 970–1119 µs
 *   is a header whose first bit is a 1; 1270–1410 µs is a header whose
 *   space merged with a leading 0 bit's space half. Any other sum is not a
 *   header, and nothing is reported.
 * - **Data runs:** glitch < 150, half-bit 150–449, full bit 450–749;
 *   anything else abandons the frame.
 * - Two runs of one level in a row mean a lost edge: the frame is kTiming.
 * - A frame is reported (kTiming) only once its header is found. A mark that
 *   abandons a frame and could be a header mark (400–800 µs) is the next
 *   frame's candidate header mark.
 * - A frame is decided at the transition in the middle of its last bit. A
 *   frame ending in a 1 bit ends with a space, which is the idle line.
 */
class ArcliteDecoder {
 public:
  ArcliteDecoder() { reset(); }

  /// @brief Drops any frame in progress.
  void reset();

  /**
   * @brief Feeds one run.
   * @param level Mark or space.
   * @param duration_us Its duration.
   * @param[out] out Set when this run completes or abandons a frame.
   * @return true if `out` was set.
   */
  bool feed(codec::Level level, uint16_t duration_us, ArcliteFrame* out);

  /**
   * @brief Ends the burst: the line has been idle past the threshold.
   * @param[out] out Set if a frame was in progress (it is kTiming).
   * @return true if `out` was set.
   */
  bool end(ArcliteFrame* out);

  /// @brief Callback for decode_symbols(): one frame, in order.
  using FrameCallback = void (*)(const ArcliteFrame& frame, void* ctx);

  /**
   * @brief Decodes one burst of RMT symbols, then ends it.
   *
   * Each symbol is two runs (`level0`/`duration0`, `level1`/`duration1`). A
   * zero duration ends the burst, as RMT RX and software receivers both
   * mark it.
   *
   * @param symbols The burst.
   * @param count Number of symbols.
   * @param callback Called for each frame, in order.
   * @param ctx Passed to callback.
   * @return Number of frames reported.
   */
  size_t decode_symbols(const rmt_symbol_word_t* symbols, size_t count,
                        FrameCallback callback, void* ctx);

 private:
  void start_frame(uint16_t duration_us);
  void report(ArcliteResult result, ArcliteFrame* out);
  void finish(ArcliteFrame* out);
  void fill_slots(codec::Level level, unsigned n);
  void abandon(codec::Level level, uint16_t duration_us, ArcliteFrame* out);

  uint8_t state_ = 0;
  uint16_t header_mark_us_ = 0;
  codec::Level last_level_ = codec::Level::kSpace;
  uint8_t slot_ = 0;
  uint16_t bits_ = 0;
  codec::RawTiming runs_[kArcliteFrameMaxRuns + 1] = {};
  size_t run_count_ = 0;
};

}  // namespace protocols
}  // namespace ir_lasertag
