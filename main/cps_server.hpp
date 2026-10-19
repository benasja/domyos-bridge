// NimBLE peripheral exposing the Cycling Power Service (0x1818) and Device
// Information Service (0x180A) so an Apple Watch can pair it as a power meter.
//
// The header is IDF-free so the payload encoder can be unit-tested on the host.
#pragma once

#include <cstddef>
#include <cstdint>

namespace cps {

// ---- Cycling Power Measurement (0x2A63) layout ----------------------------
// GATT Specification Supplement, "Cycling Power Measurement":
//   uint16 Flags
//   sint16 Instantaneous Power (W)                         - always present
//   uint16 Cumulative Crank Revolutions                    - if flag bit 5
//   uint16 Last Crank Event Time (1/1024 s)                - if flag bit 5
inline constexpr uint16_t kMeasFlagCrankRevPresent = 1u << 5;  // Flags bit 5
inline constexpr size_t kMeasurementSize = 8;

// Cycling Power Feature (0x2A65) bit 3: Crank Revolution Data Supported.
inline constexpr uint32_t kFeatureCrankRevSupported = 1u << 3;

// Sensor Location (0x2A5D) value 13: Rear Hub.
inline constexpr uint8_t kSensorLocationRearHub = 0x0D;

// GAP Appearance 0x0484: category Cycling (0x012) / sub-category Power Sensor (0x04).
inline constexpr uint16_t kAppearanceCyclingPowerSensor = 0x0484;

struct Measurement {
    int16_t power_w;
    uint16_t cumulative_crank_revs;
    uint16_t last_crank_event_time;  // 1/1024 s
};

// Serialises a measurement (little-endian) into exactly kMeasurementSize bytes.
inline void encode_measurement(const Measurement& m, uint8_t (&out)[kMeasurementSize]) {
    const uint16_t flags = kMeasFlagCrankRevPresent;
    const auto power = static_cast<uint16_t>(m.power_w);
    out[0] = static_cast<uint8_t>(flags);
    out[1] = static_cast<uint8_t>(flags >> 8);
    out[2] = static_cast<uint8_t>(power);
    out[3] = static_cast<uint8_t>(power >> 8);
    out[4] = static_cast<uint8_t>(m.cumulative_crank_revs);
    out[5] = static_cast<uint8_t>(m.cumulative_crank_revs >> 8);
    out[6] = static_cast<uint8_t>(m.last_crank_event_time);
    out[7] = static_cast<uint8_t>(m.last_crank_event_time >> 8);
}

// Registers the GATT services. Call after nimble_port_init() and before the
// host task starts. Returns 0 on success (NimBLE error code otherwise).
int register_services();

// Starts (or restarts) connectable advertising. Call from the host sync
// callback, i.e. on the NimBLE host task.
void start_advertising(uint8_t own_addr_type);

// Sends a measurement notification to every subscribed central. Safe to call
// from any task. Returns how many centrals were notified.
int notify(const Measurement& m);

// Number of centrals currently subscribed to 0x2A63 notifications.
int subscriber_count();

// Number of centrals connected to us (subscribed or not).
int connection_count();

}  // namespace cps
