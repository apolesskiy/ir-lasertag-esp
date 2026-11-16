/**
 * @file arclite_frame.cpp
 * @brief Arclite frame codec (spec/features/protocols/arclite.md).
 */

#include "ir_lasertag/protocols/arclite/arclite_frame.hpp"

#include <cstring>

namespace ir_lasertag {
namespace protocols {

using codec::Level;
using codec::RawTiming;

namespace {

// --- Data word ----------------------------------------------------------

unsigned parity16(uint16_t v) {
  v ^= static_cast<uint16_t>(v >> 8);
  v ^= static_cast<uint16_t>(v >> 4);
  v ^= static_cast<uint16_t>(v >> 2);
  v ^= static_cast<uint16_t>(v >> 1);
  return v & 1u;
}

// --- Encoding -----------------------------------------------------------

// Appends a run, merging it into the previous run of the same level.
void append_run(RawTiming* runs, size_t* count, Level level,
                uint16_t duration_us) {
  if (*count > 0 && runs[*count - 1].level == level) {
    runs[*count - 1].duration_us =
        static_cast<uint16_t>(runs[*count - 1].duration_us + duration_us);
    return;
  }
  runs[*count] = {duration_us, level};
  (*count)++;
}

// --- Decoding -----------------------------------------------------------

// Duration classes of the data windows.
enum DurationClass : uint8_t {
  kDurGlitch,   // < 150 µs
  kDurHalf,     // 150–449 µs
  kDurFull,     // 450–749 µs
  kDurInvalid,  // anything else
};

DurationClass classify(uint16_t d) {
  if (d < kArcliteGlitchUs) {
    return kDurGlitch;
  }
  if (d <= kArcliteHalfMaxUs) {
    return kDurHalf;
  }
  if (d <= kArcliteFullMaxUs) {
    return kDurFull;
  }
  return kDurInvalid;
}

bool is_header_mark(Level level, uint16_t d) {
  return level == Level::kMark && d >= kArcliteHeaderMarkMinUs &&
         d <= kArcliteHeaderMarkMaxUs;
}

enum DecoderState : uint8_t {
  kWaitHeader,   // wait for a candidate header mark
  kHeaderSpace,  // a candidate mark: its sum with the next space decides
  kBits,         // the 16 bits
};

// The data word's 32 half-bit slots. The frame is decided at the middle of
// the last bit: slot 31.
constexpr uint8_t kLastSlot = 31;

}  // namespace

uint16_t arclite_word_from_code(uint16_t code) {
  code &= kArcliteCodeMax;
  const unsigned p0 = parity16(code & kArcliteParityEvenMask);
  const unsigned p1 = parity16(code & kArcliteParityOddMask);
  return static_cast<uint16_t>((code << 2) | (p1 << 1) | p0);
}

bool arclite_word_parity_ok(uint16_t word) {
  return arclite_word_from_code(arclite_code_from_word(word)) == word;
}

void arclite_word_bytes(uint16_t code, uint8_t bytes[2]) {
  const uint16_t word = arclite_word_from_code(code);
  bytes[0] = static_cast<uint8_t>(word >> 8);
  bytes[1] = static_cast<uint8_t>(word & 0xFF);
}

size_t arclite_encode(uint16_t code, RawTiming* runs) {
  if (code > kArcliteCodeMax || runs == nullptr) {
    return 0;
  }
  const uint16_t word = arclite_word_from_code(code);
  size_t count = 0;
  append_run(runs, &count, Level::kMark, kArcliteHeaderMarkUs);
  append_run(runs, &count, Level::kSpace, kArcliteHeaderSpaceUs);
  for (int bit = kArcliteBitCount - 1; bit >= 0; bit--) {
    // Manchester: 1 = mark then space, 0 = space then mark.
    const bool one = (word >> bit) & 1u;
    append_run(runs, &count, one ? Level::kMark : Level::kSpace,
               kArcliteHalfBitUs);
    append_run(runs, &count, one ? Level::kSpace : Level::kMark,
               kArcliteHalfBitUs);
  }
  // A frame ending in a 1 bit ends with a space, which is the idle line.
  if (runs[count - 1].level == Level::kSpace) {
    count--;
  }
  return count;
}

void ArcliteDecoder::reset() {
  state_ = kWaitHeader;
  last_level_ = Level::kSpace;
  slot_ = 0;
  bits_ = 0;
  run_count_ = 0;
}

void ArcliteDecoder::start_frame(uint16_t duration_us) {
  state_ = kHeaderSpace;
  header_mark_us_ = duration_us;
  last_level_ = Level::kMark;
  slot_ = 0;
  bits_ = 0;
  runs_[0] = {duration_us, Level::kMark};
  run_count_ = 1;
}

void ArcliteDecoder::report(ArcliteResult result, ArcliteFrame* out) {
  out->result = result;
  out->word = result == ArcliteResult::kTiming ? 0 : bits_;
  out->code = result == ArcliteResult::kOk ? arclite_code_from_word(bits_) : 0;
  memcpy(out->runs, runs_, run_count_ * sizeof runs_[0]);
  out->run_count = run_count_;
  state_ = kWaitHeader;
}

// The parity check, once all 16 bits are in. Code ranges are the caller's.
void ArcliteDecoder::finish(ArcliteFrame* out) {
  report(arclite_word_parity_ok(bits_) ? ArcliteResult::kOk
                                       : ArcliteResult::kParity,
         out);
}

// Fills slots [slot_, slot_ + n) with level; even slots are first halves,
// which carry the bit value.
void ArcliteDecoder::fill_slots(Level level, unsigned n) {
  for (unsigned i = 0; i < n; i++, slot_++) {
    if ((slot_ & 1u) == 0) {
      bits_ = static_cast<uint16_t>((bits_ << 1) |
                                    (level == Level::kMark ? 1u : 0u));
    }
  }
}

// Abandons the frame in progress. The run that abandoned it may be the next
// frame's header mark.
void ArcliteDecoder::abandon(Level level, uint16_t duration_us,
                             ArcliteFrame* out) {
  report(ArcliteResult::kTiming, out);
  if (is_header_mark(level, duration_us)) {
    start_frame(duration_us);
  }
}

bool ArcliteDecoder::feed(Level level, uint16_t duration_us,
                          ArcliteFrame* out) {
  if (state_ == kWaitHeader) {
    if (is_header_mark(level, duration_us)) {
      start_frame(duration_us);
    }
    return false;
  }
  if (state_ == kHeaderSpace) {
    // No frame yet: a pair that is no header is skipped, not reported.
    if (level == Level::kMark) {
      state_ = kWaitHeader;  // a lost edge: two marks in a row
      return feed(level, duration_us, out);
    }
    const uint32_t sum = static_cast<uint32_t>(header_mark_us_) + duration_us;
    if (sum >= kArcliteHeaderOneMinUs && sum <= kArcliteHeaderOneMaxUs) {
      // The first bit starts with a mark: a 1.
      runs_[run_count_++] = {duration_us, level};
      last_level_ = level;
      state_ = kBits;
    } else if (sum >= kArcliteHeaderZeroMinUs &&
               sum <= kArcliteHeaderZeroMaxUs) {
      // The first bit is a 0, and its space half is consumed.
      runs_[run_count_++] = {duration_us, level};
      last_level_ = level;
      state_ = kBits;
      fill_slots(Level::kSpace, 1);
    } else {
      state_ = kWaitHeader;
    }
    return false;
  }
  // kBits. A frame holds at most kArcliteFrameMaxRuns runs before it is
  // decided, so the run that ends it always fits.
  runs_[run_count_++] = {duration_us, level};
  if (level == last_level_) {
    // An edge was lost: a glitch filtered out upstream.
    abandon(level, duration_us, out);
    return true;
  }
  last_level_ = level;
  // A half-bit run keeps the phase; a full-bit run must straddle a bit
  // boundary, or the bit it covers has no mid-bit transition.
  const DurationClass c = classify(duration_us);
  unsigned n;
  if (c == kDurHalf) {
    n = 1;
  } else if (c == kDurFull && (slot_ & 1u) == 1) {
    n = 2;
  } else {
    abandon(level, duration_us, out);
    return true;
  }
  fill_slots(level, n);
  if (slot_ >= kLastSlot) {
    finish(out);
    return true;
  }
  return false;
}

bool ArcliteDecoder::end(ArcliteFrame* out) {
  if (state_ != kBits) {
    state_ = kWaitHeader;  // a candidate header mark alone is no frame
    return false;
  }
  report(ArcliteResult::kTiming, out);
  return true;
}

size_t ArcliteDecoder::decode_symbols(const rmt_symbol_word_t* symbols,
                                      size_t count, FrameCallback callback,
                                      void* ctx) {
  ArcliteFrame frame;
  size_t frames = 0;
  for (size_t i = 0; i < count; i++) {
    const uint16_t d[2] = {static_cast<uint16_t>(symbols[i].duration0),
                           static_cast<uint16_t>(symbols[i].duration1)};
    const uint8_t l[2] = {static_cast<uint8_t>(symbols[i].level0),
                          static_cast<uint8_t>(symbols[i].level1)};
    for (int h = 0; h < 2; h++) {
      if (d[h] == 0) {
        i = count;  // the end-of-burst marker
        break;
      }
      if (feed(l[h] ? Level::kMark : Level::kSpace, d[h], &frame)) {
        callback(frame, ctx);
        frames++;
      }
    }
  }
  if (end(&frame)) {
    callback(frame, ctx);
    frames++;
  }
  return frames;
}

}  // namespace protocols
}  // namespace ir_lasertag
