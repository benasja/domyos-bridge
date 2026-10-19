#include "power_filter.hpp"

#include <algorithm>

namespace power_filter {

void SpikeFilter::reset() {
    history_len_ = 0;
    history_next_ = 0;
    accepted_w_ = 0;
    holding_ = false;
    hold_since_us_ = 0;
    hold_peak_w_ = 0;
}

bool SpikeFilter::is_jump(int16_t raw_w, int16_t reference_w) const {
    const int32_t rise = int32_t{raw_w} - int32_t{reference_w};
    const int32_t relative = std::max<int32_t>(reference_w, 0) * cfg_.jump_percent / 100;
    return rise > std::max<int32_t>(cfg_.min_jump_w, relative);
}

int16_t SpikeFilter::reference(int64_t now_us) const {
    int16_t ref = accepted_w_;
    for (size_t i = 0; i < history_len_; ++i) {
        if (now_us - history_[i].t_us <= cfg_.confirm_us) ref = std::max(ref, history_[i].w);
    }
    return ref;
}

void SpikeFilter::remember(int64_t now_us, int16_t published_w) {
    history_[history_next_] = {now_us, published_w};
    history_next_ = (history_next_ + 1) % kHistory;
    history_len_ = std::min(history_len_ + 1, kHistory);
}

Result SpikeFilter::update(int64_t now_us, int16_t raw_w) {
    if (!cfg_.enabled) return {raw_w, Event::None, 0, 0};

    Result r{raw_w, Event::None, 0, 0};
    // The first value after a reset has nothing to be compared with.
    if (history_len_ == 0 || !is_jump(raw_w, reference(now_us))) {
        if (holding_) {
            r.event = Event::SpikeRejected;
            r.spike_peak_w = hold_peak_w_;
            r.held_w = accepted_w_;
            holding_ = false;
        }
        accepted_w_ = raw_w;
    } else if (!holding_) {
        holding_ = true;
        hold_since_us_ = now_us;
        hold_peak_w_ = raw_w;
        r = {accepted_w_, Event::SpikeStarted, raw_w, accepted_w_};
    } else {
        hold_peak_w_ = std::max(hold_peak_w_, raw_w);
        if (now_us - hold_since_us_ >= cfg_.confirm_us) {
            r = {raw_w, Event::SpikeConfirmed, hold_peak_w_, accepted_w_};
            holding_ = false;
            accepted_w_ = raw_w;
        } else {
            r = {accepted_w_, Event::None, hold_peak_w_, accepted_w_};
        }
    }
    remember(now_us, r.power_w);
    return r;
}

}  // namespace power_filter
