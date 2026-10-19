// PURE module: holds back single-update power spikes from the bike so they
// never reach the Watch, where one bogus second inflates peak power and the
// FTP estimate. No ESP-IDF dependencies; timestamps are microseconds.
//
// Observed on the Domyos (resistance 3): the bike updates its values about
// once a second and repeats each update in ~4 notifications (~4 Hz). Now and
// then one update carries an absurd speed/cadence/power triple, e.g.
// 135 -> 456 -> 139 W (cadence 94 -> 163 -> 90) or 88 -> 331 -> 19 W, and the
// next update is back to normal. Such a spike lasts ~1 s (one update), while
// real riding changes power by at most ~60 W from one second to the next.
//
// Rule: a rise of more than max(min_jump_w, jump_percent % of the reference)
// is a suspected spike. While suspected, the last accepted value is published.
// If the rise is still there confirm_us after it started, it is real and is
// accepted; if it goes away first, it was a spike and is dropped. Drops in
// power always pass immediately. The reference is the highest value published
// during the last confirm_us, so climbing back after a short low glitch is not
// mistaken for a jump.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace power_filter {

struct Config {
    bool enabled = false;
    int min_jump_w = 60;
    int jump_percent = 50;
    // Must exceed one bike update (~1 s, up to ~1.25 s with 4 Hz packets).
    int64_t confirm_us = 2'000'000;
};

enum class Event : uint8_t {
    None,
    SpikeStarted,    // a suspicious rise began; publishing the previous value
    SpikeRejected,   // the rise went away before confirm_us: it was dropped
    SpikeConfirmed,  // the rise lasted confirm_us: accepted as real
};

struct Result {
    int16_t power_w;        // value to publish
    Event event;
    int16_t spike_peak_w;   // highest raw value of the suspected rise (if any)
    int16_t held_w;         // value published while it was suspected
};

class SpikeFilter {
public:
    explicit SpikeFilter(const Config& cfg = {}) : cfg_(cfg) {}

    void configure(const Config& cfg) {
        cfg_ = cfg;
        reset();
    }
    // Forget all history (bike disconnected). The next value is accepted as is.
    void reset();

    Result update(int64_t now_us, int16_t raw_w);
    bool holding() const { return holding_; }
    const Config& config() const { return cfg_; }

private:
    bool is_jump(int16_t raw_w, int16_t reference_w) const;
    int16_t reference(int64_t now_us) const;
    void remember(int64_t now_us, int16_t published_w);

    // 4 Hz packets x the longest confirm window allowed in Kconfig (5 s).
    static constexpr size_t kHistory = 24;
    struct Sample {
        int64_t t_us;
        int16_t w;
    };

    Config cfg_;
    std::array<Sample, kHistory> history_{};
    size_t history_len_ = 0;
    size_t history_next_ = 0;
    int16_t accepted_w_ = 0;
    bool holding_ = false;
    int64_t hold_since_us_ = 0;
    int16_t hold_peak_w_ = 0;
};

}  // namespace power_filter
