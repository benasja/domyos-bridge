// Tests for the power spike filter, replaying the Domyos packet pattern: one
// value per second, repeated in 4 notifications 250 ms apart.
#include <vector>

#include "power_filter.hpp"
#include "tiny_test.hpp"

using power_filter::Config;
using power_filter::Event;
using power_filter::SpikeFilter;

namespace {

constexpr int64_t kPacketUs = 250'000;
constexpr int kPacketsPerUpdate = 4;

Config enabled_config() {
    Config c;
    c.enabled = true;
    return c;
}

struct Replay {
    std::vector<int16_t> published;  // one per packet
    int rejected = 0;
    int confirmed = 0;
    int16_t max_published = 0;
};

// Feeds one bike update per second, each repeated in `packets_per_update`
// notifications, as the Domyos does.
Replay replay(SpikeFilter& f, const std::vector<int16_t>& per_second,
              int packets_per_update = kPacketsPerUpdate) {
    Replay r;
    int64_t t = 1'000'000;
    for (int16_t w : per_second) {
        for (int i = 0; i < packets_per_update; ++i, t += kPacketUs) {
            const auto out = f.update(t, w);
            r.published.push_back(out.power_w);
            if (out.power_w > r.max_published) r.max_published = out.power_w;
            if (out.event == Event::SpikeRejected) ++r.rejected;
            if (out.event == Event::SpikeConfirmed) ++r.confirmed;
        }
    }
    return r;
}

// Value published during the given second (its last packet).
int16_t at_second(const Replay& r, size_t second) {
    return r.published[(second + 1) * kPacketsPerUpdate - 1];
}

}  // namespace

TEST("spike filter: disabled passes everything through") {
    SpikeFilter f;  // default config: disabled
    const auto r = replay(f, {135, 456, 139});
    CHECK_EQ(r.max_published, 456);
    CHECK_EQ(r.rejected, 0);
}

TEST("spike filter: drops the 456 W one-update spike from the ride log") {
    SpikeFilter f(enabled_config());
    const auto r = replay(f, {132, 135, 135, 456, 139, 135, 129});
    CHECK_EQ(r.max_published, 139);
    CHECK_EQ(r.rejected, 1);
    CHECK_EQ(r.confirmed, 0);
    CHECK_EQ(at_second(r, 3), 135);  // the spike second publishes the previous value
    CHECK_EQ(at_second(r, 4), 139);
}

TEST("spike filter: drops the 331 W spike of a high/low glitch pair") {
    SpikeFilter f(enabled_config());
    const auto r = replay(f, {109, 112, 88, 331, 19, 107, 112});
    CHECK_EQ(r.max_published, 112);
    CHECK_EQ(r.rejected, 1);
    // The low half of the pair is a drop, so it passes; the climb back to
    // 107 is measured against the recent 88-112 W, so it is not held.
    CHECK_EQ(at_second(r, 4), 19);
    CHECK_EQ(at_second(r, 5), 107);
}

TEST("spike filter: drops a spike right before stopping; the stop passes at once") {
    SpikeFilter f(enabled_config());
    const auto r = replay(f, {99, 101, 65, 211, 0, 0});
    CHECK_EQ(r.max_published, 101);
    CHECK_EQ(at_second(r, 4), 0);
}

TEST("spike filter: a spike stretched to 5 packets (1.25 s) is still dropped") {
    SpikeFilter f(enabled_config());
    const auto r = replay(f, {130, 130, 456, 130, 130}, 5);
    CHECK_EQ(r.max_published, 130);
    CHECK_EQ(r.rejected, 1);
}

TEST("spike filter: a real sprint is accepted after the confirm time") {
    SpikeFilter f(enabled_config());
    const auto r = replay(f, {120, 120, 120, 350, 360, 370, 365, 360});
    CHECK_EQ(r.confirmed, 1);
    CHECK_EQ(r.rejected, 0);
    CHECK_EQ(at_second(r, 3), 120);  // held
    CHECK_EQ(at_second(r, 4), 120);  // held (1.75 s < 2 s)
    CHECK_EQ(at_second(r, 5), 370);  // accepted
    CHECK_EQ(r.max_published, 370);
}

TEST("spike filter: steady ramps and the ride's real changes are not delayed") {
    SpikeFilter f(enabled_config());
    // +50 W/s ramp, then the largest real rises seen in the ride log
    // (59 -> 115, 71 -> 114 -> 151), then a sharp drop.
    const std::vector<int16_t> in = {100, 150, 200, 250, 300, 59, 115, 71, 114, 151, 40};
    const auto r = replay(f, in);
    for (size_t s = 0; s < in.size(); ++s) CHECK_EQ(at_second(r, s), in[s]);
    CHECK_EQ(r.rejected + r.confirmed, 0);
}

TEST("spike filter: starting from rest is held for the confirm time, then follows") {
    SpikeFilter f(enabled_config());
    const auto r = replay(f, {0, 0, 121, 125, 130, 128});
    CHECK_EQ(at_second(r, 2), 0);
    CHECK_EQ(at_second(r, 4), 130);
    CHECK_EQ(r.confirmed, 1);
}

TEST("spike filter: after a reset the first value is accepted as is") {
    SpikeFilter f(enabled_config());
    replay(f, {0, 0, 0});
    f.reset();
    // Reconnecting to the bike mid-ride: no 0 W hold.
    const auto out = f.update(10'000'000, 250);
    CHECK_EQ(out.power_w, 250);
    CHECK(out.event == Event::None);
    // ...but the next jump is measured against it.
    CHECK_EQ(f.update(10'250'000, 600).power_w, 250);
}
