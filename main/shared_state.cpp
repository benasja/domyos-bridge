#include "shared_state.hpp"

double publishable_cadence_rpm(const BikeSnapshot& s, int64_t now_us, int64_t max_age_us) {
    if (!s.cadence_rpm || !s.fresh(s.cadence_updated_us, now_us, max_age_us)) return 0.0;
    return *s.cadence_rpm;
}

int16_t publishable_power_w(const BikeSnapshot& s, int64_t now_us, int64_t max_age_us,
                            double stopped_rpm) {
    if (!s.power_w || !s.fresh(s.power_updated_us, now_us, max_age_us)) return 0;
    const bool cadence_fresh = s.cadence_rpm && s.fresh(s.cadence_updated_us, now_us, max_age_us);
    if (cadence_fresh && *s.cadence_rpm < stopped_rpm) return 0;
    return *s.power_w;
}

void SharedState::configure_power_filter(const power_filter::Config& cfg) {
    std::lock_guard<std::mutex> lock(mutex_);
    power_filter_.configure(cfg);
}

void SharedState::set_link(BikeLink link) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.link = link;
}

power_filter::Result SharedState::apply(const ftms::IndoorBikeData& data, int64_t now_us) {
    std::lock_guard<std::mutex> lock(mutex_);
    power_filter::Result filtered{0, power_filter::Event::None, 0, 0};
    if (data.instantaneous_cadence_rpm) {
        state_.cadence_rpm = data.instantaneous_cadence_rpm;
        state_.cadence_updated_us = now_us;
    }
    if (data.instantaneous_power_w) {
        filtered = power_filter_.update(now_us, *data.instantaneous_power_w);
        state_.power_w = filtered.power_w;
        state_.power_raw_w = data.instantaneous_power_w;
        state_.power_held = power_filter_.holding();
        state_.power_updated_us = now_us;
    }
    if (data.instantaneous_speed_kmh) {
        state_.speed_kmh = data.instantaneous_speed_kmh;
        state_.speed_updated_us = now_us;
    }
    ++state_.packets;
    return filtered;
}

void SharedState::clear_values() {
    std::lock_guard<std::mutex> lock(mutex_);
    const BikeLink link = state_.link;
    const uint32_t packets = state_.packets;
    state_ = BikeSnapshot{};
    state_.link = link;
    state_.packets = packets;
    power_filter_.reset();
}

BikeSnapshot SharedState::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

SharedState& shared_state() {
    static SharedState instance;
    return instance;
}
