// Latest bike data, shared between the NimBLE host task (writer) and the 1 Hz
// publish task (reader). Uses std::mutex (backed by a FreeRTOS mutex on
// ESP-IDF) for a short, bounded critical section: a struct copy. No IDF
// headers here, so it also builds on the host.
#pragma once

#include <cstdint>
#include <mutex>
#include <optional>

#include "ftms_parser.hpp"
#include "power_filter.hpp"

// Bike link as seen by the FTMS client (or the fake data source).
enum class BikeLink : uint8_t { Scanning, Connecting, Connected };

struct BikeSnapshot {
    BikeLink link = BikeLink::Scanning;

    // Each value carries the time (µs) it was last received, because a bike
    // may split its fields across several notifications.
    std::optional<float> cadence_rpm;
    int64_t cadence_updated_us = 0;
    std::optional<int16_t> power_w;  // after the spike filter
    int64_t power_updated_us = 0;
    std::optional<int16_t> power_raw_w;  // as the bike sent it
    bool power_held = false;             // power_w is holding back a suspected spike
    std::optional<float> speed_kmh;
    int64_t speed_updated_us = 0;

    // Count of Indoor Bike Data packets applied since boot (diagnostics).
    uint32_t packets = 0;

    // True when the bike is connected and the value is younger than max_age.
    bool fresh(int64_t updated_us, int64_t now_us, int64_t max_age_us) const {
        return link == BikeLink::Connected && updated_us != 0 &&
               now_us - updated_us <= max_age_us;
    }
};

// Cadence to integrate into crank revolutions: the bike's value while fresh,
// otherwise 0 (which freezes the crank counters).
double publishable_cadence_rpm(const BikeSnapshot& s, int64_t now_us, int64_t max_age_us);

// Power to publish over CPS: the bike's value while fresh, otherwise 0. Also 0
// while the bike's own fresh cadence is below stopped_rpm: the Domyos reports
// power one sample late, so the first packet after stopping still carries the
// last pedalling power. A bike that never reports cadence is not gated.
int16_t publishable_power_w(const BikeSnapshot& s, int64_t now_us, int64_t max_age_us,
                            double stopped_rpm);

class SharedState {
public:
    // Call before the first apply(). Default: filter disabled.
    void configure_power_filter(const power_filter::Config& cfg);
    void set_link(BikeLink link);
    // Merges only the fields present in `data`; absent fields keep their
    // previous value and timestamp. Power goes through the spike filter; the
    // result tells the caller what the filter did (event None without power).
    power_filter::Result apply(const ftms::IndoorBikeData& data, int64_t now_us);
    // Drops every value and the filter history (used when the bike disconnects).
    void clear_values();
    BikeSnapshot snapshot() const;

private:
    mutable std::mutex mutex_;
    BikeSnapshot state_;
    power_filter::SpikeFilter power_filter_;
};

// The one instance used by the firmware.
SharedState& shared_state();
