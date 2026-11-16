/**
 * @file test_arclite_tagcode.cpp
 * @brief Host-based unit tests for the Arclite frame codec.
 *
 * Covers every case the Arclite frame spec (spec/features/protocols/arclite.md)
 * defines: the data word and its parity, the frame's runs, the encoder's RMT
 * symbols (bit order), and the receive decoder's windows and rules. Every
 * decode case
 * runs through both receive paths' symbol layouts.
 */

#include <cstdint>
#include <cstdio>
#include <vector>

#include "driver/rmt_encoder.h"
#include "ir_lasertag/codec/ir_encoder.hpp"
#include "ir_lasertag/protocols/arclite/arclite.hpp"
#include "ir_lasertag/protocols/arclite/arclite_frame.hpp"

using namespace ir_lasertag::codec;
using namespace ir_lasertag::protocols;

namespace {

int tests_passed = 0;
int tests_failed = 0;

#define TEST_ASSERT(expr)                                                 \
  do {                                                                    \
    if (!(expr)) {                                                        \
      printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__, #expr);            \
      tests_failed++;                                                     \
      return;                                                             \
    }                                                                     \
  } while (0)

#define RUN_TEST(test_func)                                               \
  do {                                                                    \
    const int before = tests_failed;                                      \
    printf("Running %s... ", #test_func);                                 \
    test_func();                                                          \
    if (tests_failed == before) {                                         \
      printf("PASS\n");                                                   \
      tests_passed++;                                                     \
    }                                                                     \
  } while (0)

using Runs = std::vector<RawTiming>;

Runs encode(uint16_t code) {
  RawTiming runs[kArcliteFrameMaxRuns];
  const size_t n = arclite_encode(code, runs);
  return Runs(runs, runs + n);
}

// The two receive paths' symbol layouts. RMT RX packs runs two per symbol
// and marks the end with a zero duration in the half after the last run.
// A software receiver does the same, except that a burst ending on a
// full symbol has no marker: it closes the pending half with duration1 = 0
// only when one is pending.
enum class Path { kRmt, kSoftware };

std::vector<rmt_symbol_word_t> to_symbols(const Runs& runs, Path path) {
  std::vector<rmt_symbol_word_t> out;
  for (size_t i = 0; i < runs.size(); i += 2) {
    rmt_symbol_word_t s = {};
    s.duration0 = runs[i].duration_us;
    s.level0 = runs[i].level == Level::kMark;
    if (i + 1 < runs.size()) {
      s.duration1 = runs[i + 1].duration_us;
      s.level1 = runs[i + 1].level == Level::kMark;
    }
    out.push_back(s);
  }
  if (path == Path::kRmt && runs.size() % 2 == 0) {
    out.push_back(rmt_symbol_word_t{});  // the end marker
  }
  return out;
}

std::vector<ArcliteFrame> decode(const Runs& runs, Path path) {
  std::vector<ArcliteFrame> frames;
  const auto symbols = to_symbols(runs, path);
  ArcliteDecoder decoder;
  decoder.decode_symbols(
      symbols.data(), symbols.size(),
      [](const ArcliteFrame& f, void* ctx) {
        static_cast<std::vector<ArcliteFrame>*>(ctx)->push_back(f);
      },
      &frames);
  return frames;
}

// Runs a decode case through both paths; both must agree.
std::vector<ArcliteFrame> decode_both(const Runs& runs) {
  auto rmt = decode(runs, Path::kRmt);
  auto sw = decode(runs, Path::kSoftware);
  if (rmt.size() != sw.size()) {
    return {};  // tests expecting frames fail on the empty result
  }
  for (size_t i = 0; i < rmt.size(); i++) {
    if (rmt[i].result != sw[i].result || rmt[i].word != sw[i].word ||
        rmt[i].code != sw[i].code || rmt[i].run_count != sw[i].run_count) {
      return {};
    }
  }
  return rmt;
}

bool single_ok(const Runs& runs, uint16_t code) {
  const auto f = decode_both(runs);
  return f.size() == 1 && f[0].result == ArcliteResult::kOk &&
         f[0].code == code;
}

bool single_timing(const Runs& runs) {
  const auto f = decode_both(runs);
  return f.size() == 1 && f[0].result == ArcliteResult::kTiming;
}

// --- Data word --------------------------------------------------------------

void test_word_golden_table() {
  // The data word examples (arclite.md, "IR Encoding").
  TEST_ASSERT(arclite_word_from_code(0x0001) == 0x0005);
  TEST_ASSERT(arclite_word_from_code(0x0123) == 0x048C);
  TEST_ASSERT(arclite_word_from_code(0x2A31) == 0xA8C4);
  TEST_ASSERT(arclite_word_from_code(0x3FFE) == 0xFFFA);
}

void test_word_round_trip_all_codes() {
  for (uint32_t code = 0; code <= kArcliteCodeMax; code++) {
    const uint16_t w = arclite_word_from_code(static_cast<uint16_t>(code));
    TEST_ASSERT(arclite_code_from_word(w) == code);
    TEST_ASSERT(arclite_word_parity_ok(w));
  }
}

void test_word_bytes_msb_first() {
  // The encoder walks bytes in memory order: the high byte goes first.
  uint8_t b[2];
  arclite_word_bytes(0x2A31, b);
  TEST_ASSERT(b[0] == 0xA8 && b[1] == 0xC4);
}

void test_parity_detects_single_and_adjacent_flips() {
  for (uint32_t code = 0; code <= kArcliteCodeMax; code += 7) {
    const uint16_t w = arclite_word_from_code(static_cast<uint16_t>(code));
    for (int i = 0; i < 16; i++) {
      TEST_ASSERT(!arclite_word_parity_ok(w ^ (1u << i)));
      if (i < 15) {
        TEST_ASSERT(!arclite_word_parity_ok(w ^ (3u << i)));
      }
      if (i < 14) {
        TEST_ASSERT(!arclite_word_parity_ok(w ^ (7u << i)));
      }
    }
  }
}

void test_parity_blind_spots() {
  // Two flips two positions apart (1x1) go undetected, as does a run
  // of four.
  const uint16_t w = arclite_word_from_code(0x0123);
  TEST_ASSERT(arclite_word_parity_ok(w ^ (5u << 4)));
  TEST_ASSERT(arclite_word_parity_ok(w ^ (0xFu << 4)));
}

// --- Frame ------------------------------------------------------------------

void test_encode_code_that_does_not_fit() {
  RawTiming runs[kArcliteFrameMaxRuns];
  TEST_ASSERT(arclite_encode(0x4000, runs) == 0);
  TEST_ASSERT(arclite_encode(0xFFFF, runs) == 0);
  TEST_ASSERT(arclite_encode(kArcliteCodeMax, runs) > 0);
}

void test_encode_frame_shape() {
  for (uint32_t code = 0; code <= kArcliteCodeMax; code++) {
    const Runs r = encode(static_cast<uint16_t>(code));
    TEST_ASSERT(!r.empty() && r.size() <= 33);
    TEST_ASSERT(r[0].level == Level::kMark && r[0].duration_us == 520);
    TEST_ASSERT(r.back().level == Level::kMark);
    uint32_t total = 0;
    for (size_t i = 0; i < r.size(); i++) {
      total += r[i].duration_us;
      if (i > 0) {
        TEST_ASSERT(r[i].level != r[i - 1].level);
      }
    }
    // 10.64 ms, less the idle 300 µs space of a final 1 bit.
    const bool last_one = arclite_word_from_code(code) & 1u;
    TEST_ASSERT(total + (last_one ? 300u : 0u) == 10640);
    // A leading 0 bit merges with the header space.
    const bool first_one = (arclite_word_from_code(code) >> 15) & 1u;
    TEST_ASSERT(r[1].duration_us == (first_one ? 520 : 820));
  }
}

// --- RMT encoder (bit order) ---------------------------------------------------

std::vector<rmt_symbol_word_t> encoder_symbols(uint16_t code) {
  rmt_encoder_handle_t enc = nullptr;
  if (ir_encoder_create(kArcliteConfig, &enc) != ESP_OK) {
    return {};
  }
  rmt_stub_captured().clear();
  uint8_t bytes[2];
  arclite_word_bytes(code, bytes);
  rmt_encode_state_t state = RMT_ENCODING_RESET;
  enc->encode(enc, nullptr, bytes, sizeof bytes, &state);
  auto out = rmt_stub_captured();
  rmt_del_encoder(enc);
  return state == RMT_ENCODING_COMPLETE ? out : std::vector<rmt_symbol_word_t>{};
}

void test_encoder_golden_symbols() {
  // 0x2A31 -> word 0xA8C4 = 1010 1000 1100 0100, MSB first.
  const auto s = encoder_symbols(0x2A31);
  TEST_ASSERT(s.size() == 18);
  TEST_ASSERT(s[0].level0 == 1 && s[0].duration0 == 520);
  TEST_ASSERT(s[0].level1 == 0 && s[0].duration1 == 520);
  const uint16_t word = 0xA8C4;
  for (int bit = 0; bit < 16; bit++) {
    const bool one = (word >> (15 - bit)) & 1u;
    const auto& b = s[1 + bit];
    TEST_ASSERT(b.duration0 == 300 && b.duration1 == 300);
    TEST_ASSERT(b.level0 == (one ? 1u : 0u) && b.level1 == (one ? 0u : 1u));
  }
  // The 2 ms gap is the frame's tail.
  TEST_ASSERT(s[17].level0 == 0 && s[17].duration0 == 2000);
  TEST_ASSERT(s[17].duration1 == 0);
}

void test_encoder_matches_frame_runs() {
  // The encoder's symbols, merged, are the frame's runs (plus the gap).
  const uint16_t codes[] = {0x0001, 0x0123, 0x2A31, 0x3FFE, 0x0000, 0x3FFF};
  for (uint16_t code : codes) {
    const auto s = encoder_symbols(code);
    Runs merged;
    for (const auto& sym : s) {
      const uint16_t d[2] = {static_cast<uint16_t>(sym.duration0),
                             static_cast<uint16_t>(sym.duration1)};
      const uint8_t l[2] = {static_cast<uint8_t>(sym.level0),
                            static_cast<uint8_t>(sym.level1)};
      for (int h = 0; h < 2; h++) {
        if (d[h] == 0) {
          continue;
        }
        const Level lv = l[h] ? Level::kMark : Level::kSpace;
        if (!merged.empty() && merged.back().level == lv) {
          merged.back().duration_us += d[h];
        } else {
          merged.push_back({d[h], lv});
        }
      }
    }
    TEST_ASSERT(merged.back().level == Level::kSpace);
    merged.pop_back();  // the gap (and a final 1 bit's idle half)
    const Runs r = encode(code);
    TEST_ASSERT(merged.size() == r.size());
    for (size_t i = 0; i < r.size(); i++) {
      TEST_ASSERT(merged[i].level == r[i].level);
      TEST_ASSERT(merged[i].duration_us == r[i].duration_us);
    }
  }
}

void test_config_matches_frame() {
  TEST_ASSERT(kArcliteConfig.is_valid());
  TEST_ASSERT(kArcliteConfig.carrier_freq_hz == 38000);
  TEST_ASSERT(kArcliteConfig.header_mark_us == 520);
  TEST_ASSERT(kArcliteConfig.header_space_us == 520);
  TEST_ASSERT(kArcliteConfig.one_half1.level == Level::kMark);
  TEST_ASSERT(kArcliteConfig.zero_half1.level == Level::kSpace);
  TEST_ASSERT(kArcliteConfig.footer_space_us == 2000);
  TEST_ASSERT(kArcliteConfig.bit_count == 16);
  TEST_ASSERT(kArcliteConfig.bit_order == BitOrder::kMsbFirst);
}

// --- Receive decoder ----------------------------------------------------------------

void test_decode_round_trip_all_codes() {
  for (uint32_t code = 0; code <= kArcliteCodeMax; code++) {
    TEST_ASSERT(single_ok(encode(static_cast<uint16_t>(code)),
                          static_cast<uint16_t>(code)));
  }
}

// Every data run moved to one edge of its window: half-bits (300) by
// half_delta, full bits (600) by full_delta. The header's runs (r[0] and
// r[1]) are left alone.
Runs distort(Runs r, int half_delta, int full_delta) {
  for (size_t i = 2; i < r.size(); i++) {
    const int d = r[i].duration_us;
    if (d == 300) r[i].duration_us = static_cast<uint16_t>(d + half_delta);
    if (d == 600) r[i].duration_us = static_cast<uint16_t>(d + full_delta);
  }
  return r;
}

void test_decode_data_window_edges_inclusive() {
  const uint16_t codes[] = {0x0001, 0x0123, 0x2A31, 0x3FFE};
  for (uint16_t c : codes) {
    const Runs r = encode(c);
    TEST_ASSERT(single_ok(distort(r, -150, -150), c));  // 150 and 450
    TEST_ASSERT(single_ok(distort(r, +149, +149), c));  // 449 and 749
  }
}

// A frame whose header is mark m, then a space making the pair sum p.
Runs with_header(uint16_t code, uint16_t m, uint16_t p) {
  Runs r = encode(code);
  r[0].duration_us = m;
  r[1].duration_us = static_cast<uint16_t>(p - m);
  return r;
}

bool no_frame(const Runs& r) {
  return decode(r, Path::kRmt).empty() && decode(r, Path::kSoftware).empty();
}

void test_decode_header_sum_window_edges() {
  // 0x2A31's word starts with a 1: header pair 520 + 520 = 1040, window
  // 970-1119. 0x0001's starts with a 0: 520 + 820 = 1340, window 1270-1410.
  TEST_ASSERT(encode(0x2A31)[1].duration_us == 520);
  TEST_ASSERT(encode(0x0001)[1].duration_us == 820);
  for (uint16_t m : {400, 520, 800}) {  // the header mark window's edges too
    TEST_ASSERT(single_ok(with_header(0x2A31, m, 970), 0x2A31));
    TEST_ASSERT(single_ok(with_header(0x2A31, m, 1119), 0x2A31));
    TEST_ASSERT(no_frame(with_header(0x2A31, m, 969)));
    TEST_ASSERT(no_frame(with_header(0x2A31, m, 1120)));
    TEST_ASSERT(single_ok(with_header(0x0001, m, 1270), 0x0001));
    TEST_ASSERT(single_ok(with_header(0x0001, m, 1410), 0x0001));
    TEST_ASSERT(no_frame(with_header(0x0001, m, 1269)));
    TEST_ASSERT(no_frame(with_header(0x0001, m, 1411)));
  }
}

void test_decode_header_mark_window() {
  // A mark outside 400-800 is no header, whatever its sum.
  TEST_ASSERT(no_frame(with_header(0x2A31, 399, 1040)));
  TEST_ASSERT(no_frame(with_header(0x2A31, 801, 1040)));
  TEST_ASSERT(no_frame(with_header(0x0001, 399, 1340)));
  TEST_ASSERT(no_frame(with_header(0x0001, 801, 1340)));
}

void test_decode_stretched_header_mark() {
  // A receiver stretches the header mark by up to ~130 µs and shortens the
  // space as much: the mark then measures like a 600 µs data mark, but the
  // pair still sums to the nominal 1040 or 1340.
  TEST_ASSERT(single_ok(with_header(0x2A31, 650, 1040), 0x2A31));
  TEST_ASSERT(single_ok(with_header(0x0001, 650, 1340), 0x0001));
  // ... and shortened, the other way.
  TEST_ASSERT(single_ok(with_header(0x2A31, 400, 1040), 0x2A31));
  TEST_ASSERT(single_ok(with_header(0x0001, 400, 1340), 0x0001));
}

void test_decode_data_pairs_are_never_a_header() {
  // Data pairs sum to 600, 900 or 1200 µs; none is in a header window, and
  // the windows' edges are ~70 µs from each. A data stream with no header
  // (every code's frame, header removed) yields nothing.
  for (uint32_t code = 1; code <= kArcliteCodeMax; code += 97) {
    Runs r = encode(static_cast<uint16_t>(code));
    r.erase(r.begin(), r.begin() + 2);
    if (r.front().level == Level::kSpace) {
      r.erase(r.begin());
    }
    TEST_ASSERT(no_frame(r));
  }
}

void test_decode_glitch_abandons() {
  Runs r = encode(0x0123);
  r[5].duration_us = 149;
  TEST_ASSERT(single_timing(r));
}

void test_decode_half_and_full_out_of_window() {
  // A data run of 1050 µs (no class) abandons.
  Runs r = encode(0x0123);
  r[6].duration_us = 1050;
  TEST_ASSERT(single_timing(r));
  // A data mark over 749 µs is invalid. It may also be a header mark (it is
  // in 400-800), so the decoder may go on to find a header in the rest; the
  // cut frame is still kTiming first, and nothing after it decodes.
  r = encode(0x0123);
  for (auto& x : r) {
    if (x.level == Level::kMark && x.duration_us == 600) {
      x.duration_us = 800;
      break;
    }
  }
  const auto f = decode_both(r);
  TEST_ASSERT(!f.empty() && f[0].result == ArcliteResult::kTiming);
  for (const auto& x : f) {
    TEST_ASSERT(x.result != ArcliteResult::kOk);
  }
}

void test_decode_missing_mid_bit_transition() {
  // A full-bit run must straddle a bit boundary. 0x3FFE (word 0xFFFA)
  // starts 1 1 ...: after the 520 µs header space the first data run is a
  // 300 µs mark. A 600 µs mark there would cover the whole first bit with
  // no mid-bit transition.
  Runs r = encode(0x3FFE);
  TEST_ASSERT(r[2].level == Level::kMark && r[2].duration_us == 300);
  r[2].duration_us = 600;
  TEST_ASSERT(single_timing(r));
}

void test_decode_lost_edge() {
  Runs r = encode(0x0123);
  r[4].level = r[3].level;  // two runs of one level in a row
  const auto f = decode_both(r);
  TEST_ASSERT(f.size() == 1 && f[0].result == ArcliteResult::kTiming);
}

void test_decode_parity_failure() {
  // Flip one data bit's halves: still valid Manchester, parity fails.
  const uint16_t code = 0x2A31;
  const uint16_t bad_word = arclite_word_from_code(code) ^ (1u << 8);
  // Build runs for the bad word by encoding a code whose word differs
  // only there: encode() can't, so build halves directly.
  Runs r = {{520, Level::kMark}};
  auto add = [&r](Level l, uint16_t d) {
    if (r.back().level == l) {
      r.back().duration_us += d;
    } else {
      r.push_back({d, l});
    }
  };
  add(Level::kSpace, 520);
  for (int bit = 15; bit >= 0; bit--) {
    const bool one = (bad_word >> bit) & 1u;
    add(one ? Level::kMark : Level::kSpace, 300);
    add(one ? Level::kSpace : Level::kMark, 300);
  }
  if (r.back().level == Level::kSpace) {
    r.pop_back();
  }
  const auto f = decode_both(r);
  TEST_ASSERT(f.size() == 1 && f[0].result == ArcliteResult::kParity);
  TEST_ASSERT(f[0].word == bad_word && f[0].code == 0);
}

void test_decode_truncated_frame() {
  Runs r = encode(0x0123);
  r.resize(r.size() - 3);
  TEST_ASSERT(single_timing(r));
}

void test_decode_final_mark_and_idle_space() {
  // A frame ending in 1 ends with its mark; ending in 0 likewise (the 0's
  // mark half). Both decode with nothing after them.
  TEST_ASSERT(arclite_word_from_code(0x0001) & 1u);         // ends in 1
  TEST_ASSERT(!(arclite_word_from_code(0x0002) & 1u));      // ends in 0
  TEST_ASSERT(single_ok(encode(0x0001), 0x0001));
  TEST_ASSERT(single_ok(encode(0x0002), 0x0002));
}

void test_decode_noise_without_header() {
  const Runs noise = {{300, Level::kMark}, {300, Level::kSpace},
                      {600, Level::kMark}, {100, Level::kSpace},
                      {900, Level::kMark}};
  TEST_ASSERT(decode(noise, Path::kRmt).empty());
  TEST_ASSERT(decode(noise, Path::kSoftware).empty());
}

void test_decode_abandoning_header_starts_next_frame() {
  // A frame cut short, then another frame in the same burst: the cut frame
  // is kTiming and the next one still decodes. A data mark can't be told from a header mark alone, so the cut is found
  // where the stream stops being valid data: here, a 1200 µs space.
  Runs r = encode(0x0123);
  r.resize(8);
  if (r.back().level == Level::kSpace) {
    r.pop_back();
  }
  r.push_back({1200, Level::kSpace});
  const Runs next = encode(0x2A31);
  r.insert(r.end(), next.begin(), next.end());
  const auto f = decode_both(r);
  TEST_ASSERT(f.size() == 2);
  TEST_ASSERT(f[0].result == ArcliteResult::kTiming);
  TEST_ASSERT(f[1].result == ArcliteResult::kOk && f[1].code == 0x2A31);
}

void test_decode_receiver_stretch() {
  // TSOP-style receivers stretch marks and shorten spaces by up to about
  // 5 carrier cycles: +130 / -130 µs still decodes.
  for (uint16_t c : {0x0001, 0x0123, 0x2A31, 0x3FFE}) {
    Runs r = encode(c);
    for (auto& x : r) {
      x.duration_us = static_cast<uint16_t>(
          x.level == Level::kMark ? x.duration_us + 130 : x.duration_us - 130);
    }
    TEST_ASSERT(single_ok(r, c));
  }
}

void test_decode_reports_runs_for_capture() {
  // The frame is decided at p0's mid-bit transition, so it holds every run
  // up to that one: all of them when p0 is 1 (its mark ends the frame), all
  // but the closing mark when p0 is 0.
  for (uint16_t c : {0x0001, 0x0123}) {
    const Runs r = encode(c);
    const bool p0 = arclite_word_from_code(c) & 1u;
    const auto f = decode_both(r);
    TEST_ASSERT(f.size() == 1 && f[0].run_count == r.size() - (p0 ? 0 : 1));
    for (size_t i = 0; i < f[0].run_count; i++) {
      TEST_ASSERT(f[0].runs[i].duration_us == r[i].duration_us);
      TEST_ASSERT(f[0].runs[i].level == r[i].level);
    }
  }
}

void test_decode_two_bursts_independent() {
  // A decoder is reset by end(): a partial frame doesn't leak into the
  // next burst.
  ArcliteDecoder decoder;
  ArcliteFrame f;
  Runs part = encode(0x0123);
  part.resize(6);
  for (const auto& x : part) {
    TEST_ASSERT(!decoder.feed(x.level, x.duration_us, &f));
  }
  TEST_ASSERT(decoder.end(&f) && f.result == ArcliteResult::kTiming);
  int ok = 0;
  for (const auto& x : encode(0x2A31)) {
    if (decoder.feed(x.level, x.duration_us, &f) &&
        f.result == ArcliteResult::kOk && f.code == 0x2A31) {
      ok++;
    }
  }
  TEST_ASSERT(ok == 1);
}

}  // namespace

void run_arclite_tagcode_tests() {
  printf("\n--- Arclite frame codec ---\n");
  RUN_TEST(test_word_golden_table);
  RUN_TEST(test_word_round_trip_all_codes);
  RUN_TEST(test_word_bytes_msb_first);
  RUN_TEST(test_parity_detects_single_and_adjacent_flips);
  RUN_TEST(test_parity_blind_spots);
  RUN_TEST(test_encode_code_that_does_not_fit);
  RUN_TEST(test_encode_frame_shape);
  RUN_TEST(test_encoder_golden_symbols);
  RUN_TEST(test_encoder_matches_frame_runs);
  RUN_TEST(test_config_matches_frame);
  RUN_TEST(test_decode_round_trip_all_codes);
  RUN_TEST(test_decode_data_window_edges_inclusive);
  RUN_TEST(test_decode_header_sum_window_edges);
  RUN_TEST(test_decode_stretched_header_mark);
  RUN_TEST(test_decode_data_pairs_are_never_a_header);
  RUN_TEST(test_decode_glitch_abandons);
  RUN_TEST(test_decode_half_and_full_out_of_window);
  RUN_TEST(test_decode_header_mark_window);
  RUN_TEST(test_decode_missing_mid_bit_transition);
  RUN_TEST(test_decode_lost_edge);
  RUN_TEST(test_decode_parity_failure);
  RUN_TEST(test_decode_truncated_frame);
  RUN_TEST(test_decode_final_mark_and_idle_space);
  RUN_TEST(test_decode_noise_without_header);
  RUN_TEST(test_decode_abandoning_header_starts_next_frame);
  RUN_TEST(test_decode_receiver_stretch);
  RUN_TEST(test_decode_reports_runs_for_capture);
  RUN_TEST(test_decode_two_bursts_independent);
  printf("Arclite: %d passed, %d failed\n", tests_passed, tests_failed);
}

int get_arclite_tagcode_test_failures() {
  return tests_failed;
}
