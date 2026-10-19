// Tests for the small IDF-free pieces around the pure modules: the CPS payload
// encoder and the shared-state merge semantics.
#include "cps_server.hpp"
#include "shared_state.hpp"
#include "tiny_test.hpp"

TEST("CPS measurement encoding: flags 0x0020, LE fields") {
    uint8_t out[cps::kMeasurementSize];
    cps::encode_measurement({100, 0x1234, 0xABCD}, out);
    const uint8_t expected[] = {0x20, 0x00, 0x64, 0x00, 0x34, 0x12, 0xCD, 0xAB};
    for (size_t i = 0; i < sizeof(expected); ++i) CHECK_EQ(out[i], expected[i]);

    cps::encode_measurement({-1, 0, 0}, out);  // sint16 power
    CHECK_EQ(out[2], 0xFF);
    CHECK_EQ(out[3], 0xFF);
}

TEST("shared state merges only present fields") {
    SharedState st;
    st.set_link(BikeLink::Connected);

    ftms::IndoorBikeData a;
    a.instantaneous_cadence_rpm = 80.0f;
    st.apply(a, 1'000'000);

    ftms::IndoorBikeData b;  // separate notification carrying only power
    b.instantaneous_power_w = 150;
    st.apply(b, 1'500'000);

    const auto s = st.snapshot();
    CHECK_NEAR(*s.cadence_rpm, 80.0, 1e-6);
    CHECK_EQ(s.cadence_updated_us, 1'000'000);
    CHECK_EQ(*s.power_w, 150);
    CHECK_EQ(s.power_updated_us, 1'500'000);
    CHECK_EQ(s.packets, 2u);
}

TEST("freshness: stale after timeout, never fresh when disconnected") {
    SharedState st;
    st.set_link(BikeLink::Connected);
    ftms::IndoorBikeData a;
    a.instantaneous_power_w = 200;
    st.apply(a, 10'000'000);
    auto s = st.snapshot();
    CHECK(s.fresh(s.power_updated_us, 12'999'999, 3'000'000));
    CHECK(!s.fresh(s.power_updated_us, 13'000'001, 3'000'000));
    CHECK(!s.fresh(s.cadence_updated_us, 10'000'000, 3'000'000));  // never received

    st.set_link(BikeLink::Scanning);
    s = st.snapshot();
    CHECK(!s.fresh(s.power_updated_us, 10'000'001, 3'000'000));

    st.clear_values();
    s = st.snapshot();
    CHECK(!s.power_w.has_value());
    CHECK_EQ(s.packets, 1u);
}

namespace {
constexpr int64_t kMaxAge = 3'000'000;
constexpr double kStopped = 5.0;

BikeSnapshot connected_snapshot(std::optional<float> cadence, std::optional<int16_t> power,
                                int64_t at_us) {
    BikeSnapshot s;
    s.link = BikeLink::Connected;
    s.cadence_rpm = cadence;
    s.cadence_updated_us = cadence ? at_us : 0;
    s.power_w = power;
    s.power_updated_us = power ? at_us : 0;
    return s;
}
}  // namespace

TEST("publishable power: bike value while pedalling") {
    const auto s = connected_snapshot(85.0f, 200, 1'000'000);
    CHECK_EQ(publishable_power_w(s, 1'500'000, kMaxAge, kStopped), 200);
    CHECK_NEAR(publishable_cadence_rpm(s, 1'500'000, kMaxAge), 85.0, 1e-6);
}

TEST("publishable power: 0 W once the bike's cadence drops below the stop threshold") {
    // Domyos: the first packet after stopping has cadence 0 but last power.
    CHECK_EQ(publishable_power_w(connected_snapshot(0.0f, 230, 1'000'000), 1'000'000, kMaxAge,
                                 kStopped), 0);
    CHECK_EQ(publishable_power_w(connected_snapshot(4.5f, 230, 1'000'000), 1'000'000, kMaxAge,
                                 kStopped), 0);
    CHECK_EQ(publishable_power_w(connected_snapshot(5.0f, 230, 1'000'000), 1'000'000, kMaxAge,
                                 kStopped), 230);
}

TEST("publishable power: stale or disconnected gives 0 W and 0 rpm") {
    auto s = connected_snapshot(85.0f, 200, 1'000'000);
    CHECK_EQ(publishable_power_w(s, 4'000'001, kMaxAge, kStopped), 0);
    CHECK_NEAR(publishable_cadence_rpm(s, 4'000'001, kMaxAge), 0.0, 1e-9);
    s.link = BikeLink::Scanning;
    CHECK_EQ(publishable_power_w(s, 1'000'000, kMaxAge, kStopped), 0);
    CHECK_NEAR(publishable_cadence_rpm(s, 1'000'000, kMaxAge), 0.0, 1e-9);
}

TEST("publishable power: not gated when the bike sends no cadence") {
    const auto s = connected_snapshot(std::nullopt, 150, 1'000'000);
    CHECK_EQ(publishable_power_w(s, 1'000'000, kMaxAge, kStopped), 150);
    CHECK_NEAR(publishable_cadence_rpm(s, 1'000'000, kMaxAge), 0.0, 1e-9);
}
