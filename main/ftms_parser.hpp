// PURE module: FTMS byte payloads -> structs. No ESP-IDF / NimBLE dependencies,
// so it builds and is unit-tested on the host (see test/).
//
// References: Bluetooth SIG "Fitness Machine Service" v1.0,
//   §4.3   Fitness Machine Feature (0x2ACC)
//   §4.9   Indoor Bike Data (0x2AD2)
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

namespace ftms {

// ---- Indoor Bike Data (0x2AD2) flag bits, FTMS §4.9.1.1 -------------------
// Bit 0 is inverted: Instantaneous Speed is present when the bit is 0.
inline constexpr uint16_t kFlagMoreData            = 1u << 0;
inline constexpr uint16_t kFlagAverageSpeed        = 1u << 1;
inline constexpr uint16_t kFlagInstCadence         = 1u << 2;
inline constexpr uint16_t kFlagAverageCadence      = 1u << 3;
inline constexpr uint16_t kFlagTotalDistance       = 1u << 4;
inline constexpr uint16_t kFlagResistanceLevel     = 1u << 5;
inline constexpr uint16_t kFlagInstPower           = 1u << 6;
inline constexpr uint16_t kFlagAveragePower        = 1u << 7;
inline constexpr uint16_t kFlagExpendedEnergy      = 1u << 8;
inline constexpr uint16_t kFlagHeartRate           = 1u << 9;
inline constexpr uint16_t kFlagMetabolicEquivalent = 1u << 10;
inline constexpr uint16_t kFlagElapsedTime         = 1u << 11;
inline constexpr uint16_t kFlagRemainingTime       = 1u << 12;
inline constexpr uint16_t kFlagsReservedMask       = 0xE000;  // bits 13-15 RFU

// ---- Field resolutions, FTMS §4.9.1.2 - §4.9.1.14 -------------------------
inline constexpr float kSpeedKmhPerUnit   = 0.01f;  // speed fields: 0.01 km/h
inline constexpr float kCadenceRpmPerUnit = 0.5f;   // cadence fields: 0.5 1/min
inline constexpr float kMetPerUnit        = 0.1f;   // Metabolic Equivalent: 0.1

// FTMS §4.9.1.10: energy sub-fields use all-ones for "Data Not Available".
inline constexpr uint16_t kEnergyU16NotAvailable = 0xFFFF;
inline constexpr uint8_t  kEnergyU8NotAvailable  = 0xFF;

// Size in bytes of the Expended Energy group (uint16 total + uint16 per hour
// + uint8 per minute), FTMS §4.9.1.10 - §4.9.1.12.
inline constexpr size_t kExpendedEnergyGroupSize = 5;

// Decoded Indoor Bike Data. Every field is optional: a field is set only when
// its flag bit says it is present AND the packet actually contained it.
struct IndoorBikeData {
    std::optional<float>    instantaneous_speed_kmh;
    std::optional<float>    average_speed_kmh;
    std::optional<float>    instantaneous_cadence_rpm;
    std::optional<float>    average_cadence_rpm;
    std::optional<uint32_t> total_distance_m;
    std::optional<int16_t>  resistance_level;
    std::optional<int16_t>  instantaneous_power_w;
    std::optional<int16_t>  average_power_w;
    std::optional<uint16_t> total_energy_kcal;
    std::optional<uint16_t> energy_per_hour_kcal;
    std::optional<uint8_t>  energy_per_minute_kcal;
    std::optional<uint8_t>  heart_rate_bpm;
    std::optional<float>    metabolic_equivalent;
    std::optional<uint16_t> elapsed_time_s;
    std::optional<uint16_t> remaining_time_s;
};

enum class ParseStatus : uint8_t {
    Ok,         // every field announced by the flags was read
    Truncated,  // packet ended early; fields parsed before the cut are valid
    TooShort,   // fewer than 2 bytes: not even a flags field
};

struct ParseResult {
    IndoorBikeData data;
    ParseStatus status = ParseStatus::TooShort;
    uint16_t flags = 0;
    size_t bytes_consumed = 0;   // < len means trailing bytes were ignored
    bool reserved_flags_set = false;
};

// Parses an Indoor Bike Data notification. Never reads past buf[len-1].
ParseResult parse_indoor_bike_data(const uint8_t* buf, size_t len);

// ---- Fitness Machine Feature (0x2ACC), FTMS §4.3 --------------------------
struct MachineFeature {
    uint32_t fitness_machine_features = 0;  // FTMS §4.3.1.1
    uint32_t target_setting_features = 0;   // FTMS §4.3.1.2
};
inline constexpr size_t kMachineFeatureSize = 8;

std::optional<MachineFeature> parse_machine_feature(const uint8_t* buf, size_t len);

// Human-readable bit names for logging; nullptr for reserved bits.
const char* machine_feature_bit_name(unsigned bit);
const char* target_setting_bit_name(unsigned bit);

}  // namespace ftms
