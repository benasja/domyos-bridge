#include <cstdint>
#include <vector>

#include "ftms_parser.hpp"
#include "tiny_test.hpp"

using namespace ftms;

namespace {

ParseResult parse(const std::vector<uint8_t>& b) { return parse_indoor_bike_data(b.data(), b.size()); }

}  // namespace

TEST("flags=0x0000: only instantaneous speed (inverted bit 0)") {
    // 0x03E8 = 1000 -> 10.00 km/h
    const auto r = parse({0x00, 0x00, 0xE8, 0x03});
    CHECK(r.status == ParseStatus::Ok);
    CHECK_EQ(r.bytes_consumed, 4u);
    CHECK(r.data.instantaneous_speed_kmh.has_value());
    CHECK_NEAR(*r.data.instantaneous_speed_kmh, 10.0, 1e-4);
    CHECK(!r.data.instantaneous_cadence_rpm);
    CHECK(!r.data.instantaneous_power_w);
}

TEST("flags=0x0001: More Data set -> no speed, empty payload") {
    const auto r = parse({0x01, 0x00});
    CHECK(r.status == ParseStatus::Ok);
    CHECK_EQ(r.bytes_consumed, 2u);
    CHECK(!r.data.instantaneous_speed_kmh);
}

TEST("speed + cadence + power (typical bike packet)") {
    // flags = bit2 | bit6 = 0x0044, bit0 clear -> speed present
    // speed 2550 -> 25.50 km/h, cadence 170 -> 85.0 rpm, power 0x00C8 = 200 W
    const auto r = parse({0x44, 0x00, 0xF6, 0x09, 0xAA, 0x00, 0xC8, 0x00});
    CHECK(r.status == ParseStatus::Ok);
    CHECK_NEAR(*r.data.instantaneous_speed_kmh, 25.5, 1e-4);
    CHECK_NEAR(*r.data.instantaneous_cadence_rpm, 85.0, 1e-6);
    CHECK_EQ(*r.data.instantaneous_power_w, 200);
}

TEST("cadence only, More Data set (bike splits fields across packets)") {
    // flags = bit0 | bit2 = 0x0005, cadence 161 -> 80.5 rpm (odd raw -> .5)
    const auto r = parse({0x05, 0x00, 0xA1, 0x00});
    CHECK(r.status == ParseStatus::Ok);
    CHECK(!r.data.instantaneous_speed_kmh);
    CHECK_NEAR(*r.data.instantaneous_cadence_rpm, 80.5, 1e-6);
}

TEST("all fields present, uint24 distance, negative power") {
    std::vector<uint8_t> b = {
        0xFE, 0x1F,        // flags: bits 1..12 set, bit 0 clear
        0x10, 0x27,        // inst speed 10000 -> 100.00 km/h
        0x88, 0x13,        // avg speed 5000 -> 50.00 km/h
        0xA0, 0x00,        // inst cadence 160 -> 80 rpm
        0x96, 0x00,        // avg cadence 150 -> 75 rpm
        0x56, 0x34, 0x12,  // distance 0x123456 = 1193046 m
        0xFB, 0xFF,        // resistance -5
        0x9C, 0xFF,        // inst power -100 W
        0x2C, 0x01,        // avg power 300 W
        0x64, 0x00,        // total energy 100 kcal
        0x58, 0x02,        // energy/h 600 kcal
        0x0A,              // energy/min 10 kcal
        0x8C,              // heart rate 140
        0x2D,              // MET 45 -> 4.5
        0x10, 0x0E,        // elapsed 3600 s
        0x2C, 0x01,        // remaining 300 s
    };
    const auto r = parse(b);
    CHECK(r.status == ParseStatus::Ok);
    CHECK_EQ(r.bytes_consumed, b.size());
    CHECK_EQ(b.size(), 30u);
    const auto& d = r.data;
    CHECK_NEAR(*d.instantaneous_speed_kmh, 100.0, 1e-3);
    CHECK_NEAR(*d.average_speed_kmh, 50.0, 1e-3);
    CHECK_NEAR(*d.instantaneous_cadence_rpm, 80.0, 1e-6);
    CHECK_NEAR(*d.average_cadence_rpm, 75.0, 1e-6);
    CHECK_EQ(*d.total_distance_m, 1193046u);
    CHECK_EQ(*d.resistance_level, -5);
    CHECK_EQ(*d.instantaneous_power_w, -100);
    CHECK_EQ(*d.average_power_w, 300);
    CHECK_EQ(*d.total_energy_kcal, 100);
    CHECK_EQ(*d.energy_per_hour_kcal, 600);
    CHECK_EQ(*d.energy_per_minute_kcal, 10);
    CHECK_EQ(*d.heart_rate_bpm, 140);
    CHECK_NEAR(*d.metabolic_equivalent, 4.5, 1e-5);
    CHECK_EQ(*d.elapsed_time_s, 3600);
    CHECK_EQ(*d.remaining_time_s, 300);
}

TEST("uint24 distance uses all three bytes (0xFFFFFF)") {
    // flags = bit0 | bit4 = 0x0011
    const auto r = parse({0x11, 0x00, 0xFF, 0xFF, 0xFF});
    CHECK(r.status == ParseStatus::Ok);
    CHECK_EQ(*r.data.total_distance_m, 0xFFFFFFu);
}

TEST("expended energy 'not available' markers become nullopt") {
    // flags = bit0 | bit8 = 0x0101
    const auto r = parse({0x01, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
    CHECK(r.status == ParseStatus::Ok);
    CHECK(!r.data.total_energy_kcal);
    CHECK(!r.data.energy_per_hour_kcal);
    CHECK(!r.data.energy_per_minute_kcal);
}

TEST("truncated inside power: earlier fields kept, power absent") {
    // flags 0x0044: speed, cadence, power. Power has only 1 of 2 bytes.
    const auto r = parse({0x44, 0x00, 0xE8, 0x03, 0xA0, 0x00, 0xC8});
    CHECK(r.status == ParseStatus::Truncated);
    CHECK_EQ(r.bytes_consumed, 6u);
    CHECK_NEAR(*r.data.instantaneous_speed_kmh, 10.0, 1e-4);
    CHECK_NEAR(*r.data.instantaneous_cadence_rpm, 80.0, 1e-6);
    CHECK(!r.data.instantaneous_power_w);
}

TEST("truncated inside uint24 distance") {
    const auto r = parse({0x11, 0x00, 0x01, 0x02});
    CHECK(r.status == ParseStatus::Truncated);
    CHECK(!r.data.total_distance_m);
}

TEST("truncated inside expended-energy group: whole group dropped") {
    const auto r = parse({0x01, 0x01, 0x64, 0x00, 0x58});
    CHECK(r.status == ParseStatus::Truncated);
    CHECK_EQ(r.bytes_consumed, 2u);
    CHECK(!r.data.total_energy_kcal);
}

TEST("flags present but no payload at all") {
    const auto r = parse({0x00, 0x00});  // speed announced, missing
    CHECK(r.status == ParseStatus::Truncated);
    CHECK(!r.data.instantaneous_speed_kmh);
}

TEST("too short for flags: 0 and 1 bytes, and nullptr") {
    CHECK(parse({}).status == ParseStatus::TooShort);
    CHECK(parse({0x44}).status == ParseStatus::TooShort);
    CHECK(parse_indoor_bike_data(nullptr, 10).status == ParseStatus::TooShort);
}

TEST("reserved flag bits reported, trailing bytes ignored") {
    const auto r = parse({0x01, 0x80, 0xAA, 0xBB});
    CHECK(r.status == ParseStatus::Ok);
    CHECK(r.reserved_flags_set);
    CHECK_EQ(r.bytes_consumed, 2u);
}

TEST("parser never reads past the buffer for any flags/length") {
    // Exhaustive over all 16-bit flags x lengths 0..32: the payload bytes after
    // `len` are poisoned; a read past the end would surface as consumed > len.
    uint8_t buf[64];
    for (uint32_t flags = 0; flags <= 0xFFFF; ++flags) {
        for (size_t len = 0; len <= 32; ++len) {
            for (size_t i = 0; i < sizeof(buf); ++i) buf[i] = 0x5A;
            buf[0] = static_cast<uint8_t>(flags);
            buf[1] = static_cast<uint8_t>(flags >> 8);
            const auto r = parse_indoor_bike_data(buf, len);
            if (r.bytes_consumed > len) {
                CHECK(r.bytes_consumed <= len);
                return;
            }
        }
    }
}

TEST("Fitness Machine Feature: two little-endian uint32") {
    const std::vector<uint8_t> b = {0x06, 0x44, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00};
    const auto mf = parse_machine_feature(b.data(), b.size());
    CHECK(mf.has_value());
    CHECK_EQ(mf->fitness_machine_features, 0x4406u);  // cadence, distance, resist, power
    CHECK_EQ(mf->target_setting_features, 0x4u);      // resistance target
    CHECK(!parse_machine_feature(b.data(), 7).has_value());
    CHECK(machine_feature_bit_name(14) != nullptr);
    CHECK(machine_feature_bit_name(31) == nullptr);
}
