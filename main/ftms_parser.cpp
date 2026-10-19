#include "ftms_parser.hpp"

namespace ftms {
namespace {

// Little-endian bounds-checked cursor. Each read either fully succeeds and
// advances, or fails and leaves the cursor untouched.
class ByteReader {
public:
    ByteReader(const uint8_t* buf, size_t len) : buf_(buf), len_(len) {}

    size_t pos() const { return pos_; }
    size_t remaining() const { return len_ - pos_; }

    bool u8(uint8_t& out) {
        if (remaining() < 1) return false;
        out = buf_[pos_++];
        return true;
    }
    bool u16(uint16_t& out) {
        if (remaining() < 2) return false;
        out = static_cast<uint16_t>(buf_[pos_] | (buf_[pos_ + 1] << 8));
        pos_ += 2;
        return true;
    }
    bool s16(int16_t& out) {
        uint16_t raw;
        if (!u16(raw)) return false;
        out = static_cast<int16_t>(raw);
        return true;
    }
    bool u24(uint32_t& out) {
        if (remaining() < 3) return false;
        out = static_cast<uint32_t>(buf_[pos_]) |
              (static_cast<uint32_t>(buf_[pos_ + 1]) << 8) |
              (static_cast<uint32_t>(buf_[pos_ + 2]) << 16);
        pos_ += 3;
        return true;
    }
    bool u32(uint32_t& out) {
        if (remaining() < 4) return false;
        out = static_cast<uint32_t>(buf_[pos_]) |
              (static_cast<uint32_t>(buf_[pos_ + 1]) << 8) |
              (static_cast<uint32_t>(buf_[pos_ + 2]) << 16) |
              (static_cast<uint32_t>(buf_[pos_ + 3]) << 24);
        pos_ += 4;
        return true;
    }

private:
    const uint8_t* buf_;
    size_t len_;
    size_t pos_ = 0;
};

}  // namespace

ParseResult parse_indoor_bike_data(const uint8_t* buf, size_t len) {
    ParseResult r;
    if (buf == nullptr) len = 0;
    ByteReader rd(buf, len);

    if (!rd.u16(r.flags)) {
        r.status = ParseStatus::TooShort;
        r.bytes_consumed = 0;
        return r;
    }
    r.reserved_flags_set = (r.flags & kFlagsReservedMask) != 0;
    const uint16_t f = r.flags;
    IndoorBikeData& d = r.data;

    // Fields must be read strictly in spec order; the first failed read means
    // the packet is truncated and nothing after it can be trusted.
    auto finish = [&](ParseStatus s) {
        r.status = s;
        r.bytes_consumed = rd.pos();
        return r;
    };

    uint16_t u16v = 0;
    int16_t s16v = 0;
    uint8_t u8v = 0;
    uint32_t u32v = 0;

    if (!(f & kFlagMoreData)) {  // inverted: present when bit 0 == 0
        if (!rd.u16(u16v)) return finish(ParseStatus::Truncated);
        d.instantaneous_speed_kmh = u16v * kSpeedKmhPerUnit;
    }
    if (f & kFlagAverageSpeed) {
        if (!rd.u16(u16v)) return finish(ParseStatus::Truncated);
        d.average_speed_kmh = u16v * kSpeedKmhPerUnit;
    }
    if (f & kFlagInstCadence) {
        if (!rd.u16(u16v)) return finish(ParseStatus::Truncated);
        d.instantaneous_cadence_rpm = u16v * kCadenceRpmPerUnit;
    }
    if (f & kFlagAverageCadence) {
        if (!rd.u16(u16v)) return finish(ParseStatus::Truncated);
        d.average_cadence_rpm = u16v * kCadenceRpmPerUnit;
    }
    if (f & kFlagTotalDistance) {
        if (!rd.u24(u32v)) return finish(ParseStatus::Truncated);
        d.total_distance_m = u32v;
    }
    if (f & kFlagResistanceLevel) {
        if (!rd.s16(s16v)) return finish(ParseStatus::Truncated);
        d.resistance_level = s16v;
    }
    if (f & kFlagInstPower) {
        if (!rd.s16(s16v)) return finish(ParseStatus::Truncated);
        d.instantaneous_power_w = s16v;
    }
    if (f & kFlagAveragePower) {
        if (!rd.s16(s16v)) return finish(ParseStatus::Truncated);
        d.average_power_w = s16v;
    }
    if (f & kFlagExpendedEnergy) {
        // The three sub-fields share one flag; take the group atomically.
        if (rd.remaining() < kExpendedEnergyGroupSize) return finish(ParseStatus::Truncated);
        uint16_t total = 0, per_hour = 0;
        uint8_t per_min = 0;
        rd.u16(total);  // cannot fail: length checked for the whole group above
        rd.u16(per_hour);
        rd.u8(per_min);
        if (total != kEnergyU16NotAvailable) d.total_energy_kcal = total;
        if (per_hour != kEnergyU16NotAvailable) d.energy_per_hour_kcal = per_hour;
        if (per_min != kEnergyU8NotAvailable) d.energy_per_minute_kcal = per_min;
    }
    if (f & kFlagHeartRate) {
        if (!rd.u8(u8v)) return finish(ParseStatus::Truncated);
        d.heart_rate_bpm = u8v;
    }
    if (f & kFlagMetabolicEquivalent) {
        if (!rd.u8(u8v)) return finish(ParseStatus::Truncated);
        d.metabolic_equivalent = u8v * kMetPerUnit;
    }
    if (f & kFlagElapsedTime) {
        if (!rd.u16(u16v)) return finish(ParseStatus::Truncated);
        d.elapsed_time_s = u16v;
    }
    if (f & kFlagRemainingTime) {
        if (!rd.u16(u16v)) return finish(ParseStatus::Truncated);
        d.remaining_time_s = u16v;
    }
    return finish(ParseStatus::Ok);
}

std::optional<MachineFeature> parse_machine_feature(const uint8_t* buf, size_t len) {
    if (buf == nullptr || len < kMachineFeatureSize) return std::nullopt;
    ByteReader rd(buf, len);
    MachineFeature mf;  // reads cannot fail: length checked above
    rd.u32(mf.fitness_machine_features);
    rd.u32(mf.target_setting_features);
    return mf;
}

const char* machine_feature_bit_name(unsigned bit) {
    // FTMS §4.3.1.1, Table 4.3
    static constexpr const char* kNames[] = {
        "Average Speed",                    // 0
        "Cadence",                          // 1
        "Total Distance",                   // 2
        "Inclination",                      // 3
        "Elevation Gain",                   // 4
        "Pace",                             // 5
        "Step Count",                       // 6
        "Resistance Level",                 // 7
        "Stride Count",                     // 8
        "Expended Energy",                  // 9
        "Heart Rate Measurement",           // 10
        "Metabolic Equivalent",             // 11
        "Elapsed Time",                     // 12
        "Remaining Time",                   // 13
        "Power Measurement",                // 14
        "Force on Belt and Power Output",   // 15
        "User Data Retention",              // 16
    };
    return bit < sizeof(kNames) / sizeof(kNames[0]) ? kNames[bit] : nullptr;
}

const char* target_setting_bit_name(unsigned bit) {
    // FTMS §4.3.1.2, Table 4.4
    static constexpr const char* kNames[] = {
        "Speed Target",                          // 0
        "Inclination Target",                    // 1
        "Resistance Target",                     // 2
        "Power Target",                          // 3
        "Heart Rate Target",                     // 4
        "Targeted Expended Energy",              // 5
        "Targeted Step Number",                  // 6
        "Targeted Stride Number",                // 7
        "Targeted Distance",                     // 8
        "Targeted Training Time",                // 9
        "Targeted Time in Two HR Zones",         // 10
        "Targeted Time in Three HR Zones",       // 11
        "Targeted Time in Five HR Zones",        // 12
        "Indoor Bike Simulation Parameters",     // 13
        "Wheel Circumference",                   // 14
        "Spin Down Control",                     // 15
        "Targeted Cadence",                      // 16
    };
    return bit < sizeof(kNames) / sizeof(kNames[0]) ? kNames[bit] : nullptr;
}

}  // namespace ftms
