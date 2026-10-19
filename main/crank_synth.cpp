#include "crank_synth.hpp"

#include <cmath>

namespace crank {

void CrankSynth::update(int64_t now_us, double cadence_rpm) {
    if (!have_reference_) {
        have_reference_ = true;
        last_update_us_ = now_us;
        return;
    }
    const int64_t dt_us = now_us - last_update_us_;
    if (dt_us <= 0) return;

    const int64_t interval_start_us = last_update_us_;
    last_update_us_ = now_us;

    // Negated comparison so NaN is also treated as stopped.
    if (!(cadence_rpm >= kStoppedCadenceRpm)) return;

    const double dt_s = static_cast<double>(dt_us) / kMicrosPerSecond;
    const double increment = (cadence_rpm / kSecondsPerMinute) * dt_s;
    const double before = revs_accum_;
    const double after = before + increment;
    const double last_whole = std::floor(after);

    if (last_whole > std::floor(before)) {
        // Fraction of the interval at which the integral reached last_whole.
        // In (0, 1] because before < last_whole <= after.
        const double fraction = (last_whole - before) / increment;
        const int64_t crossing_us =
            interval_start_us + static_cast<int64_t>(std::llround(fraction * dt_us));
        last_event_time_ = micros_to_crank_ticks(crossing_us);
    }
    revs_accum_ = after;
}

CrankSynth::Output CrankSynth::output() const {
    // Truncate to whole revs, then let the uint16 wrap naturally.
    const auto whole = static_cast<uint64_t>(revs_accum_);
    return Output{static_cast<uint16_t>(whole), last_event_time_};
}

}  // namespace crank
