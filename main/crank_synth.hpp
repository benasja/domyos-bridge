// PURE module: turns a stream of instantaneous-cadence samples into the
// (cumulative crank revolutions, last crank event time) pair that the Cycling
// Power Measurement characteristic carries. No ESP-IDF dependencies; the
// caller supplies timestamps in microseconds (esp_timer_get_time() on target).
//
// Receivers compute cadence as Δrevs / Δevent_time between two notifications,
// so the event time must be the moment the latest whole revolution completed,
// not the moment we happened to send. We integrate cadence over time and
// linearly interpolate the instant the integral crossed its last integer.
#pragma once

#include <cstdint>

namespace crank {

// Below this cadence the crank is treated as stopped (no revolutions counted,
// event time frozen). Avoids a slowly creeping counter from sensor noise.
inline constexpr double kStoppedCadenceRpm = 5.0;

inline constexpr int64_t kMicrosPerSecond = 1'000'000;
inline constexpr double kSecondsPerMinute = 60.0;

// Cycling Power Measurement "Last Crank Event Time" unit: 1/1024 s
// (GATT Specification Supplement, Cycling Power Measurement characteristic).
inline constexpr int64_t kCrankTimeTicksPerSecond = 1024;

// Converts an absolute microsecond timestamp to the uint16 1/1024 s timebase.
// Wraps every 64 s by design; receivers handle the rollover.
constexpr uint16_t micros_to_crank_ticks(int64_t t_us) {
    return static_cast<uint16_t>(static_cast<uint64_t>(t_us) *
                                 static_cast<uint64_t>(kCrankTimeTicksPerSecond) /
                                 static_cast<uint64_t>(kMicrosPerSecond));
}

class CrankSynth {
public:
    struct Output {
        uint16_t cumulative_revs;      // wraps at 65536
        uint16_t last_event_time;      // 1/1024 s, wraps every 64 s
    };

    // Advances the integration to now_us, assuming cadence_rpm held over the
    // interval since the previous call. The first call only sets the time
    // reference. A cadence below kStoppedCadenceRpm (or NaN) counts nothing
    // and freezes the event time, but still moves the time reference so a
    // later restart does not integrate across the stopped gap. Calls with a
    // timestamp that does not move forward are ignored.
    void update(int64_t now_us, double cadence_rpm);

    Output output() const;

    // Exact accumulated revolutions (unwrapped). Mostly for tests/logging.
    double revs_accum() const { return revs_accum_; }

private:
    double revs_accum_ = 0.0;
    int64_t last_update_us_ = 0;
    bool have_reference_ = false;
    uint16_t last_event_time_ = 0;
};

}  // namespace crank
