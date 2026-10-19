#include <cstdint>

#include "crank_synth.hpp"
#include "tiny_test.hpp"

using crank::CrankSynth;
using crank::micros_to_crank_ticks;

namespace {
constexpr int64_t kSec = 1'000'000;
constexpr int64_t kT0 = 5 * kSec;  // arbitrary non-zero start (time since boot)

// Drives the synth at 1 Hz from `from` (exclusive) to `to` (inclusive) seconds
// relative to kT0, at a constant cadence.
void run(CrankSynth& s, int from_s, int to_s, double rpm) {
    for (int t = from_s + 1; t <= to_s; ++t) s.update(kT0 + t * kSec, rpm);
}
}  // namespace

TEST("micros_to_crank_ticks: 1 s = 1024 ticks, wraps at 64 s") {
    CHECK_EQ(micros_to_crank_ticks(0), 0);
    CHECK_EQ(micros_to_crank_ticks(kSec), 1024);
    CHECK_EQ(micros_to_crank_ticks(750'000), 768);
    CHECK_EQ(micros_to_crank_ticks(64 * kSec), 0);
    CHECK_EQ(micros_to_crank_ticks(65 * kSec), 1024);
}

TEST("first update only sets the reference") {
    CrankSynth s;
    s.update(kT0, 90.0);
    CHECK_EQ(s.output().cumulative_revs, 0);
    CHECK_EQ(s.output().last_event_time, 0);
}

TEST("constant 80 rpm for 60 s: ~80 revs, events every 0.75 s") {
    CrankSynth s;
    s.update(kT0, 80.0);
    // After 1 s: 1.333 revs, rev #1 at 0.75 s.
    s.update(kT0 + kSec, 80.0);
    CHECK_EQ(s.output().cumulative_revs, 1);
    CHECK_EQ(s.output().last_event_time, micros_to_crank_ticks(kT0 + 750'000));
    // After 3 s: 4 revs, rev #4 at exactly 3.0 s.
    run(s, 1, 3, 80.0);
    CHECK_EQ(s.output().cumulative_revs, 4);
    CHECK_NEAR(s.output().last_event_time, micros_to_crank_ticks(kT0 + 3 * kSec), 1);

    run(s, 3, 60, 80.0);
    CHECK_NEAR(s.revs_accum(), 80.0, 1e-9);
    const auto out = s.output();
    CHECK(out.cumulative_revs == 79 || out.cumulative_revs == 80);  // float rounding at exact boundary
    // Event time = time of the last whole rev = revs * 0.75 s.
    const int64_t expected_us = kT0 + static_cast<int64_t>(out.cumulative_revs) * 750'000;
    CHECK_NEAR(out.last_event_time, micros_to_crank_ticks(expected_us), 1);
}

TEST("receiver-side cadence from consecutive notifications is 80 rpm") {
    CrankSynth s;
    s.update(kT0, 80.0);
    run(s, 0, 10, 80.0);
    const auto a = s.output();
    run(s, 10, 13, 80.0);  // 3 s = exactly 4 revs
    const auto b = s.output();
    const uint16_t drevs = static_cast<uint16_t>(b.cumulative_revs - a.cumulative_revs);
    const uint16_t dt = static_cast<uint16_t>(b.last_event_time - a.last_event_time);
    CHECK(dt > 0);
    CHECK_NEAR(drevs * 60.0 * 1024.0 / dt, 80.0, 0.5);
}

TEST("sub-1-rev intervals: 30 rpm increments every other second") {
    CrankSynth s;
    s.update(kT0, 30.0);
    s.update(kT0 + 1 * kSec, 30.0);
    CHECK_EQ(s.output().cumulative_revs, 0);
    CHECK_EQ(s.output().last_event_time, 0);
    s.update(kT0 + 2 * kSec, 30.0);
    CHECK_EQ(s.output().cumulative_revs, 1);
    CHECK_NEAR(s.output().last_event_time, micros_to_crank_ticks(kT0 + 2 * kSec), 1);
    s.update(kT0 + 3 * kSec, 30.0);
    CHECK_EQ(s.output().cumulative_revs, 1);  // unchanged...
    CHECK_NEAR(s.output().last_event_time, micros_to_crank_ticks(kT0 + 2 * kSec), 1);  // ...and so is the time
}

TEST("cadence step change 60 -> 120 rpm") {
    CrankSynth s;
    s.update(kT0, 60.0);
    run(s, 0, 10, 60.0);
    CHECK_EQ(s.output().cumulative_revs, 10);
    CHECK_NEAR(s.output().last_event_time, micros_to_crank_ticks(kT0 + 10 * kSec), 1);
    // 120 rpm over a 1 s interval: 2 revs, at +0.5 s and +1.0 s.
    s.update(kT0 + 11 * kSec, 120.0);
    CHECK_EQ(s.output().cumulative_revs, 12);
    CHECK_NEAR(s.output().last_event_time, micros_to_crank_ticks(kT0 + 11 * kSec), 1);
    run(s, 11, 15, 120.0);
    CHECK_EQ(s.output().cumulative_revs, 20);
}

TEST("non-integer crossing is interpolated inside the interval") {
    CrankSynth s;
    s.update(kT0, 90.0);  // 1.5 rev/s
    s.update(kT0 + kSec, 90.0);
    // revs 0 -> 1.5; rev #1 at 1/1.5 = 0.6667 s
    CHECK_EQ(s.output().cumulative_revs, 1);
    CHECK_NEAR(s.output().last_event_time, micros_to_crank_ticks(kT0 + 666'667), 1);
    s.update(kT0 + 2 * kSec, 90.0);
    // revs 1.5 -> 3.0; rev #3 at exactly 2.0 s
    CHECK_EQ(s.output().cumulative_revs, 3);
    CHECK_NEAR(s.output().last_event_time, micros_to_crank_ticks(kT0 + 2 * kSec), 1);
}

TEST("stop/start: counters freeze, no integration across the gap") {
    CrankSynth s;
    s.update(kT0, 60.0);
    run(s, 0, 5, 60.0);
    const auto stopped = s.output();
    CHECK_EQ(stopped.cumulative_revs, 5);
    run(s, 5, 15, 0.0);  // stopped for 10 s
    CHECK_EQ(s.output().cumulative_revs, stopped.cumulative_revs);
    CHECK_EQ(s.output().last_event_time, stopped.last_event_time);
    run(s, 15, 20, 4.9);  // below threshold still counts as stopped
    CHECK_EQ(s.output().cumulative_revs, stopped.cumulative_revs);
    // Resume: first interval is (20 s, 21 s], not (5 s, 21 s].
    s.update(kT0 + 21 * kSec, 60.0);
    CHECK_EQ(s.output().cumulative_revs, 6);
    CHECK_NEAR(s.output().last_event_time, micros_to_crank_ticks(kT0 + 21 * kSec), 1);
}

TEST("NaN cadence is treated as stopped; time going backwards is ignored") {
    CrankSynth s;
    s.update(kT0, 60.0);
    s.update(kT0 + kSec, 60.0);
    s.update(kT0 + 2 * kSec, std::nan(""));
    CHECK_EQ(s.output().cumulative_revs, 1);
    s.update(kT0 + kSec, 60.0);  // backwards: ignored
    s.update(kT0 + 2 * kSec, 60.0);  // same as last reference: ignored
    CHECK_EQ(s.output().cumulative_revs, 1);
    s.update(kT0 + 3 * kSec, 60.0);
    CHECK_EQ(s.output().cumulative_revs, 2);
}

TEST("event time wraps every 64 s") {
    CrankSynth s;
    s.update(0, 60.0);
    for (int t = 1; t <= 70; ++t) s.update(t * kSec, 60.0);
    // rev #70 at 70 s: 70*1024 = 71680 -> 71680 - 65536 = 6144
    CHECK_EQ(s.output().last_event_time, 6144);
    CHECK_EQ(s.output().cumulative_revs, 70);
}

TEST("cumulative revs wrap at 65536 and deltas stay correct") {
    CrankSynth s;
    s.update(0, 120.0);  // 2 rev/s
    for (int t = 1; t <= 32768; ++t) s.update(t * static_cast<int64_t>(kSec), 120.0);
    CHECK_NEAR(s.revs_accum(), 65536.0, 1e-6);
    const auto before = s.output();
    CHECK(before.cumulative_revs == 0 || before.cumulative_revs == 65535);
    for (int t = 32769; t <= 32778; ++t) s.update(t * static_cast<int64_t>(kSec), 120.0);
    const auto after = s.output();
    CHECK_EQ(static_cast<uint16_t>(after.cumulative_revs - before.cumulative_revs), 20);
    CHECK_EQ(static_cast<uint16_t>(after.last_event_time - before.last_event_time), 10 * 1024);
}
