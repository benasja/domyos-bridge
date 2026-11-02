#include "ftms_client.hpp"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"
#include "ftms_parser.hpp"
#include "nvs.h"
#include "sdkconfig.h"
#include "shared_state.hpp"
// Must come last (see header).
#include "nimble_includes.hpp"

namespace ftms_client {
namespace {

constexpr const char* TAG = "ftms";

// ---- Assigned numbers (Bluetooth SIG Assigned Numbers §3.4 / §3.8) --------
constexpr uint16_t kUuidFitnessMachineService = 0x1826;
constexpr uint16_t kUuidIndoorBikeData = 0x2AD2;
constexpr uint16_t kUuidFitnessMachineFeature = 0x2ACC;
constexpr uint16_t kUuidCccd = 0x2902;  // Client Characteristic Configuration descriptor

// CCCD value (Core Spec Vol 3 Part G §3.3.3.3): bit 0 = notifications, LE.
constexpr uint8_t kCccdEnableNotifications[2] = {0x01, 0x00};

// Scan 50 ms out of every 100 ms (units of 0.625 ms), leaving radio time for
// the Watch link and our own advertising. Active scan: the bike's name may
// only be in its scan response.
constexpr uint16_t kScanInterval = 160;
constexpr uint16_t kScanWindow = 80;

constexpr int32_t kConnectTimeoutMs = 10'000;
constexpr uint32_t kBackoffMinMs = 1'000;
constexpr uint32_t kBackoffMaxMs = 10'000;
// With a saved bike, other matching bikes are ignored for this long after a
// scan starts, so a neighbour's bike doesn't win the race.
constexpr int64_t kPreferSavedWindowUs = 5'000'000;
constexpr int64_t kDecodedInfoLogIntervalUs = 1'000'000;

// Largest Indoor Bike Data is 30 bytes (all fields); extra bytes are ignored.
constexpr size_t kMaxNotifyLen = 64;

constexpr const char* kNvsNamespace = "bridge";
constexpr const char* kNvsKeyBikeAddr = "bike_addr";

constexpr ble_uuid16_t make_uuid16(uint16_t value) {
    ble_uuid16_t u{};
    u.u.type = BLE_UUID_TYPE_16;
    u.value = value;
    return u;
}
const ble_uuid16_t kSvcFtms = make_uuid16(kUuidFitnessMachineService);

enum class State : uint8_t { Idle, Scanning, Connecting, Discovering, Subscribed };

// Everything we learn about the connected bike. Reset on every connection.
struct Peer {
    uint16_t conn = BLE_HS_CONN_HANDLE_NONE;
    ble_addr_t addr{};
    uint16_t svc_start = 0;
    uint16_t svc_end = 0;
    uint16_t ibd_val = 0;      // Indoor Bike Data value handle
    uint16_t ibd_end = 0;      // last handle that can hold its descriptors
    uint16_t feature_val = 0;  // Fitness Machine Feature value handle
    uint16_t cccd = 0;         // Indoor Bike Data CCCD handle
    bool security_requested = false;
};

// All of the below is touched only from the NimBLE host task (GAP/GATT
// callbacks and the retry callout run there), so no locking is needed.
State s_state = State::Idle;
uint8_t s_own_addr_type = BLE_OWN_ADDR_PUBLIC;
Peer s_peer;
ble_addr_t s_saved_addr{};
bool s_have_saved_addr = false;
int64_t s_scan_started_us = 0;
uint32_t s_backoff_ms = kBackoffMinMs;
ble_npl_callout s_retry_timer;
int64_t s_last_info_log_us = 0;

int gap_event(ble_gap_event* event, void* arg);
void start_scan();

// ---- Helpers ---------------------------------------------------------------

struct AddrStr {
    char s[18];
};
AddrStr fmt_addr(const ble_addr_t& a) {
    AddrStr out;
    snprintf(out.s, sizeof(out.s), "%02x:%02x:%02x:%02x:%02x:%02x", a.val[5], a.val[4],
             a.val[3], a.val[2], a.val[1], a.val[0]);
    return out;
}

// Fixed-size printf-append buffer for building one log line without heap.
class Line {
public:
    void add(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
        if (len_ >= sizeof(buf_)) return;
        va_list ap;
        va_start(ap, fmt);
        const int n = vsnprintf(buf_ + len_, sizeof(buf_) - len_, fmt, ap);
        va_end(ap);
        if (n > 0) len_ = std::min(sizeof(buf_), len_ + static_cast<size_t>(n));
    }
    const char* c_str() const { return buf_; }

private:
    char buf_[320] = {};
    size_t len_ = 0;
};

void save_bike_addr(const ble_addr_t& addr) {
    if (s_have_saved_addr && ble_addr_cmp(&addr, &s_saved_addr) == 0) return;
    // Rare (first connection to a new bike) and short; NimBLE's own bond
    // store writes NVS from this same task.
    nvs_handle_t h;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_blob(h, kNvsKeyBikeAddr, &addr, sizeof(addr));
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Saving bike address failed: %s", esp_err_to_name(err));
        return;
    }
    s_saved_addr = addr;
    s_have_saved_addr = true;
    ESP_LOGI(TAG, "Saved %s as the preferred bike", fmt_addr(addr).s);
}

void load_bike_addr() {
    nvs_handle_t h;
    if (nvs_open(kNvsNamespace, NVS_READONLY, &h) != ESP_OK) return;  // nothing saved yet
    size_t len = sizeof(s_saved_addr);
    s_have_saved_addr = nvs_get_blob(h, kNvsKeyBikeAddr, &s_saved_addr, &len) == ESP_OK &&
                        len == sizeof(s_saved_addr);
    nvs_close(h);
    if (s_have_saved_addr) {
        ESP_LOGI(TAG, "Preferred bike from NVS: %s (type %d)", fmt_addr(s_saved_addr).s,
                 s_saved_addr.type);
    }
}

// ---- Reconnect with backoff -------------------------------------------------

void schedule_rescan() {
    s_state = State::Idle;
    shared_state().set_link(BikeLink::Scanning);
    ESP_LOGI(TAG, "Rescanning in %u ms", static_cast<unsigned>(s_backoff_ms));
    ble_npl_callout_reset(&s_retry_timer, ble_npl_time_ms_to_ticks32(s_backoff_ms));
    s_backoff_ms = std::min(s_backoff_ms * 2, kBackoffMaxMs);
}

void retry_timer_cb(ble_npl_event*) { start_scan(); }

// Gives up on the current connection; the DISCONNECT event schedules a rescan.
void drop_connection(const char* why, int status) {
    ESP_LOGE(TAG, "%s (status=%d); disconnecting", why, status);
    ble_gap_terminate(s_peer.conn, BLE_ERR_REM_USER_CONN_TERM);
}

// ---- Indoor Bike Data handling ---------------------------------------------

void log_decoded(const ftms::ParseResult& r, esp_log_level_t level) {
    const ftms::IndoorBikeData& d = r.data;
    Line l;
    l.add("flags=0x%04x", r.flags);
    if (d.instantaneous_speed_kmh) l.add(" speed=%.2fkm/h", *d.instantaneous_speed_kmh);
    if (d.average_speed_kmh) l.add(" avg_speed=%.2fkm/h", *d.average_speed_kmh);
    if (d.instantaneous_cadence_rpm) l.add(" cadence=%.1frpm", *d.instantaneous_cadence_rpm);
    if (d.average_cadence_rpm) l.add(" avg_cadence=%.1frpm", *d.average_cadence_rpm);
    if (d.total_distance_m) l.add(" dist=%um", static_cast<unsigned>(*d.total_distance_m));
    if (d.resistance_level) l.add(" resistance=%d", *d.resistance_level);
    if (d.instantaneous_power_w) l.add(" power=%dW", *d.instantaneous_power_w);
    if (d.average_power_w) l.add(" avg_power=%dW", *d.average_power_w);
    if (d.total_energy_kcal) l.add(" energy=%ukcal", *d.total_energy_kcal);
    if (d.energy_per_hour_kcal) l.add(" energy/h=%u", *d.energy_per_hour_kcal);
    if (d.energy_per_minute_kcal) l.add(" energy/min=%u", *d.energy_per_minute_kcal);
    if (d.heart_rate_bpm) l.add(" hr=%ubpm", *d.heart_rate_bpm);
    if (d.metabolic_equivalent) l.add(" met=%.1f", *d.metabolic_equivalent);
    if (d.elapsed_time_s) l.add(" elapsed=%us", *d.elapsed_time_s);
    if (d.remaining_time_s) l.add(" remaining=%us", *d.remaining_time_s);
    ESP_LOG_LEVEL(level, TAG, "%s", l.c_str());
}

void on_indoor_bike_data(const uint8_t* buf, size_t len, size_t total_len) {
    const int64_t now_us = esp_timer_get_time();
    const bool debug = esp_log_level_get(TAG) >= ESP_LOG_DEBUG;

    if (debug) {
        char hex[kMaxNotifyLen * 3 + 1] = {};
        for (size_t i = 0; i < len; ++i) snprintf(hex + i * 3, 4, "%02X ", buf[i]);
        ESP_LOGD(TAG, "0x2AD2 [%u] %s", static_cast<unsigned>(total_len), hex);
    }
    if (total_len > len) {
        ESP_LOGW(TAG, "Notification of %u bytes truncated to %u", static_cast<unsigned>(total_len),
                 static_cast<unsigned>(len));
    }

    const ftms::ParseResult r = ftms::parse_indoor_bike_data(buf, len);
    if (r.status == ftms::ParseStatus::TooShort) {
        ESP_LOGW(TAG, "Indoor Bike Data too short (%u bytes); ignored", static_cast<unsigned>(len));
        return;
    }
    if (r.status == ftms::ParseStatus::Truncated) {
        ESP_LOGW(TAG, "Indoor Bike Data truncated: flags=0x%04x len=%u, kept first %u bytes",
                 r.flags, static_cast<unsigned>(len), static_cast<unsigned>(r.bytes_consumed));
    }
    if (r.reserved_flags_set) ESP_LOGD(TAG, "Reserved flag bits set: 0x%04x", r.flags);

    const power_filter::Result pf = shared_state().apply(r.data, now_us);
    switch (pf.event) {
        case power_filter::Event::SpikeStarted:
            ESP_LOGD(TAG, "Power jump to %d W suspected; holding %d W", pf.spike_peak_w, pf.held_w);
            break;
        case power_filter::Event::SpikeRejected:
            ESP_LOGI(TAG, "Dropped power spike: bike sent up to %d W, Watch got %d W",
                     pf.spike_peak_w, pf.held_w);
            break;
        case power_filter::Event::SpikeConfirmed:
            ESP_LOGI(TAG, "Power rise to %d W lasted; accepted (was %d W)", pf.power_w, pf.held_w);
            break;
        case power_filter::Event::None:
            break;
    }

    if (now_us - s_last_info_log_us >= kDecodedInfoLogIntervalUs) {
        s_last_info_log_us = now_us;
        log_decoded(r, ESP_LOG_INFO);
    } else if (debug) {
        log_decoded(r, ESP_LOG_DEBUG);
    }
}

// ---- GATT discovery chain ---------------------------------------------------
// MTU -> (first connection only: GATT table dump) -> FTMS service ->
// characteristics -> 0x2AD2 descriptors -> read 0x2ACC -> write 0x2AD2 CCCD.
// Each step starts the next from its completion callback.

void subscribe();

int on_subscribed(uint16_t conn, const ble_gatt_error* error, ble_gatt_attr*, void*) {
    if (conn != s_peer.conn) return 0;
    if (error->status == 0) {
        s_state = State::Subscribed;
        s_backoff_ms = kBackoffMinMs;
        shared_state().set_link(BikeLink::Connected);
        ESP_LOGI(TAG, "Subscribed to Indoor Bike Data. Pedal!");
        save_bike_addr(s_peer.addr);
        return 0;
    }
    const bool needs_security =
        error->status == BLE_HS_ATT_ERR(BLE_ATT_ERR_INSUFFICIENT_AUTHEN) ||
        error->status == BLE_HS_ATT_ERR(BLE_ATT_ERR_INSUFFICIENT_ENC);
    if (needs_security && !s_peer.security_requested) {
        ESP_LOGW(TAG, "Bike requires encryption for the CCCD; pairing");
        s_peer.security_requested = true;
        const int rc = ble_gap_security_initiate(conn);
        if (rc != 0) drop_connection("ble_gap_security_initiate failed", rc);
        return 0;  // ENC_CHANGE retries the subscribe
    }
    drop_connection("Enabling Indoor Bike Data notifications failed", error->status);
    return 0;
}

void subscribe() {
    const int rc = ble_gattc_write_flat(s_peer.conn, s_peer.cccd, kCccdEnableNotifications,
                                        sizeof(kCccdEnableNotifications), on_subscribed, nullptr);
    if (rc != 0) drop_connection("CCCD write could not start", rc);
}

void log_machine_features(const ftms::MachineFeature& mf) {
    ESP_LOGI(TAG, "Fitness Machine Feature: features=0x%08x target_settings=0x%08x",
             static_cast<unsigned>(mf.fitness_machine_features),
             static_cast<unsigned>(mf.target_setting_features));
    Line l;
    for (unsigned bit = 0; bit < 32; ++bit) {
        if (!(mf.fitness_machine_features & (1u << bit))) continue;
        const char* name = ftms::machine_feature_bit_name(bit);
        if (name) l.add(" [%s]", name); else l.add(" [bit%u?]", bit);
    }
    ESP_LOGI(TAG, "  bike claims:%s", l.c_str());
    Line t;
    for (unsigned bit = 0; bit < 32; ++bit) {
        if (!(mf.target_setting_features & (1u << bit))) continue;
        const char* name = ftms::target_setting_bit_name(bit);
        if (name) t.add(" [%s]", name); else t.add(" [bit%u?]", bit);
    }
    ESP_LOGI(TAG, "  target settings (unused, read-only bridge):%s", t.c_str());
}

int on_feature_read(uint16_t conn, const ble_gatt_error* error, ble_gatt_attr* attr, void*) {
    if (conn != s_peer.conn) return 0;
    if (error->status == 0 && attr != nullptr && attr->om != nullptr) {
        uint8_t buf[16] = {};
        const uint16_t len = std::min<uint16_t>(OS_MBUF_PKTLEN(attr->om), sizeof(buf));
        os_mbuf_copydata(attr->om, 0, len, buf);
        if (const auto mf = ftms::parse_machine_feature(buf, len)) {
            log_machine_features(*mf);
        } else {
            ESP_LOGW(TAG, "Fitness Machine Feature has unexpected length %u", len);
        }
    } else {
        ESP_LOGW(TAG, "Reading Fitness Machine Feature failed (status=%d); continuing",
                 error->status);
    }
    subscribe();  // the feature read is informational only
    return 0;
}

void read_feature_then_subscribe() {
    if (s_peer.feature_val == 0) {
        ESP_LOGW(TAG, "Bike has no Fitness Machine Feature characteristic");
        subscribe();
        return;
    }
    const int rc = ble_gattc_read(s_peer.conn, s_peer.feature_val, on_feature_read, nullptr);
    if (rc != 0) subscribe();
}

int on_descriptor(uint16_t conn, const ble_gatt_error* error, uint16_t, const ble_gatt_dsc* dsc,
                  void*) {
    if (conn != s_peer.conn) return 0;
    if (error->status == 0) {
        if (ble_uuid_u16(&dsc->uuid.u) == kUuidCccd) s_peer.cccd = dsc->handle;
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        drop_connection("Descriptor discovery failed", error->status);
        return 0;
    }
    if (s_peer.cccd == 0) {
        drop_connection("Indoor Bike Data has no CCCD", 0);
        return 0;
    }
    ESP_LOGD(TAG, "0x2AD2 CCCD handle=%u", s_peer.cccd);
    read_feature_then_subscribe();
    return 0;
}

int on_characteristic(uint16_t conn, const ble_gatt_error* error, const ble_gatt_chr* chr,
                      void*) {
    if (conn != s_peer.conn) return 0;
    if (error->status == 0) {
        // Descriptors of 0x2AD2 end just before the next characteristic.
        if (s_peer.ibd_val != 0 && s_peer.ibd_end == 0 && chr->def_handle > s_peer.ibd_val) {
            s_peer.ibd_end = chr->def_handle - 1;
        }
        switch (ble_uuid_u16(&chr->uuid.u)) {
            case kUuidIndoorBikeData:
                s_peer.ibd_val = chr->val_handle;
                if (!(chr->properties & BLE_GATT_CHR_PROP_NOTIFY)) {
                    ESP_LOGW(TAG, "0x2AD2 does not advertise the Notify property");
                }
                break;
            case kUuidFitnessMachineFeature:
                s_peer.feature_val = chr->val_handle;
                break;
            default:
                break;
        }
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        drop_connection("Characteristic discovery failed", error->status);
        return 0;
    }
    if (s_peer.ibd_val == 0) {
        drop_connection("Bike has no Indoor Bike Data (0x2AD2) characteristic", 0);
        return 0;
    }
    if (s_peer.ibd_end == 0) s_peer.ibd_end = s_peer.svc_end;
    ESP_LOGD(TAG, "0x2AD2 value handle=%u (descriptors up to %u), 0x2ACC handle=%u",
             s_peer.ibd_val, s_peer.ibd_end, s_peer.feature_val);
    const int rc = ble_gattc_disc_all_dscs(conn, s_peer.ibd_val, s_peer.ibd_end, on_descriptor,
                                           nullptr);
    if (rc != 0) drop_connection("Descriptor discovery could not start", rc);
    return 0;
}

int on_service(uint16_t conn, const ble_gatt_error* error, const ble_gatt_svc* svc, void*) {
    if (conn != s_peer.conn) return 0;
    if (error->status == 0) {
        s_peer.svc_start = svc->start_handle;
        s_peer.svc_end = svc->end_handle;
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        drop_connection("Service discovery failed", error->status);
        return 0;
    }
    if (s_peer.svc_start == 0) {
        drop_connection("Device has no Fitness Machine Service (0x1826)", 0);
        return 0;
    }
    ESP_LOGD(TAG, "FTMS service handles %u..%u", s_peer.svc_start, s_peer.svc_end);
    const int rc = ble_gattc_disc_all_chrs(conn, s_peer.svc_start, s_peer.svc_end,
                                           on_characteristic, nullptr);
    if (rc != 0) drop_connection("Characteristic discovery could not start", rc);
    return 0;
}

void discover_service() {
    const int rc = ble_gattc_disc_svc_by_uuid(s_peer.conn, &kSvcFtms.u, on_service, nullptr);
    if (rc != 0) drop_connection("Service discovery could not start", rc);
}

// ---- GATT table dump (once per boot) ----------------------------------------
// Lists every service and characteristic the bike offers, so a log shows
// whether it has more than FTMS (e.g. Domyos' own UART-style channel, which
// is what the console listens on for its display). Purely informational:
// any failure here just moves on to the FTMS discovery above.

struct DumpSvc {
    uint16_t start;
    uint16_t end;
};
constexpr size_t kMaxDumpSvcs = 16;
DumpSvc s_dump_svcs[kMaxDumpSvcs];
size_t s_dump_count = 0;
size_t s_dump_next = 0;
bool s_dump_done = false;

// Microchip ISSC "transparent UART" service, used by Domyos consoles.
constexpr const char* kUuidDomyosUart = "49535343-fe7d-4ae5-8fa9-9fafd205e455";

void dump_next_service();

int on_dump_characteristic(uint16_t conn, const ble_gatt_error* error, const ble_gatt_chr* chr,
                           void*) {
    if (conn != s_peer.conn) return 0;
    if (error->status == 0) {
        char uuid[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&chr->uuid.u, uuid);
        const uint8_t p = chr->properties;
        ESP_LOGI(TAG, "    chr %s handle=%u%s%s%s%s%s", uuid, chr->val_handle,
                 (p & BLE_GATT_CHR_PROP_READ) ? " read" : "",
                 (p & BLE_GATT_CHR_PROP_WRITE) ? " write" : "",
                 (p & BLE_GATT_CHR_PROP_WRITE_NO_RSP) ? " write-no-rsp" : "",
                 (p & BLE_GATT_CHR_PROP_NOTIFY) ? " notify" : "",
                 (p & BLE_GATT_CHR_PROP_INDICATE) ? " indicate" : "");
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        ESP_LOGW(TAG, "    characteristic listing failed (status=%d)", error->status);
    }
    dump_next_service();
    return 0;
}

void dump_next_service() {
    if (s_dump_next >= s_dump_count) {
        ESP_LOGI(TAG, "GATT table done");
        s_dump_done = true;
        discover_service();
        return;
    }
    const DumpSvc& s = s_dump_svcs[s_dump_next++];
    const int rc = ble_gattc_disc_all_chrs(s_peer.conn, s.start, s.end, on_dump_characteristic,
                                           nullptr);
    if (rc != 0) {
        ESP_LOGW(TAG, "    characteristic listing could not start (rc=%d)", rc);
        dump_next_service();
    }
}

int on_dump_service(uint16_t conn, const ble_gatt_error* error, const ble_gatt_svc* svc, void*) {
    if (conn != s_peer.conn) return 0;
    if (error->status == 0) {
        char uuid[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&svc->uuid.u, uuid);
        const bool domyos = strcmp(uuid, kUuidDomyosUart) == 0;
        ESP_LOGI(TAG, "  svc %s handles %u..%u%s", uuid, svc->start_handle, svc->end_handle,
                 domyos ? "  <-- Domyos console channel" : "");
        if (s_dump_count < kMaxDumpSvcs) {
            s_dump_svcs[s_dump_count++] = {svc->start_handle, svc->end_handle};
        }
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        ESP_LOGW(TAG, "Service listing failed (status=%d); skipping the GATT table",
                 error->status);
        discover_service();
        return 0;
    }
    dump_next_service();
    return 0;
}

// Runs the dump until one completes after boot (a connection that drops
// halfway tries again next time), then the normal FTMS discovery.
void dump_gatt_then_discover() {
    if (s_dump_done) {
        discover_service();
        return;
    }
    s_dump_count = 0;
    s_dump_next = 0;
    ESP_LOGI(TAG, "GATT table of %s:", fmt_addr(s_peer.addr).s);
    const int rc = ble_gattc_disc_all_svcs(s_peer.conn, on_dump_service, nullptr);
    if (rc != 0) {
        ESP_LOGW(TAG, "Service listing could not start (rc=%d)", rc);
        discover_service();
    }
}

int on_mtu(uint16_t conn, const ble_gatt_error* error, uint16_t mtu, void*) {
    if (conn != s_peer.conn) return 0;
    if (error->status == 0) {
        ESP_LOGI(TAG, "Bike MTU=%u", mtu);
    } else {
        ESP_LOGW(TAG, "MTU exchange failed (status=%d); continuing with 23", error->status);
    }
    dump_gatt_then_discover();
    return 0;
}

// ---- Scanning and connecting ------------------------------------------------

bool advertisement_matches(const ble_gap_disc_desc& d) {
    ble_hs_adv_fields f{};
    if (ble_hs_adv_parse_fields(&f, d.data, d.length_data) != 0) return false;

    for (int i = 0; i < f.num_uuids16; ++i) {
        if (ble_uuid_u16(&f.uuids16[i].u) == kUuidFitnessMachineService) return true;
    }
    const char* prefix = CONFIG_BRIDGE_BIKE_NAME_PREFIX;
    const size_t prefix_len = strlen(prefix);
    return prefix_len > 0 && f.name != nullptr && f.name_len >= prefix_len &&
           memcmp(f.name, prefix, prefix_len) == 0;
}

void connect_to(const ble_addr_t& addr, int8_t rssi) {
    int rc = ble_gap_disc_cancel();  // NimBLE refuses to connect while scanning
    if (rc != 0 && rc != BLE_HS_EALREADY) ESP_LOGW(TAG, "ble_gap_disc_cancel rc=%d", rc);

    s_state = State::Connecting;
    s_peer = Peer{};
    s_peer.addr = addr;
    shared_state().set_link(BikeLink::Connecting);
    ESP_LOGI(TAG, "Connecting to bike %s (type %d, rssi %d)", fmt_addr(addr).s, addr.type, rssi);

    rc = ble_gap_connect(s_own_addr_type, &addr, kConnectTimeoutMs, nullptr, gap_event, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect rc=%d", rc);
        schedule_rescan();
    }
}

void on_discovered(const ble_gap_disc_desc& d) {
    if (s_state != State::Scanning || !advertisement_matches(d)) return;

    const bool is_saved = s_have_saved_addr && ble_addr_cmp(&d.addr, &s_saved_addr) == 0;
    if (s_have_saved_addr && !is_saved &&
        esp_timer_get_time() - s_scan_started_us < kPreferSavedWindowUs) {
        return;  // give the saved bike a head start
    }
    connect_to(d.addr, d.rssi);
}

void start_scan() {
    if (s_state == State::Scanning || s_peer.conn != BLE_HS_CONN_HANDLE_NONE) return;

    ble_gap_disc_params params{};
    params.itvl = kScanInterval;
    params.window = kScanWindow;
    params.passive = 0;
    params.filter_duplicates = 0;  // keep reporting: the saved-bike window relies on it

    const int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &params, gap_event, nullptr);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "ble_gap_disc rc=%d", rc);
        schedule_rescan();
        return;
    }
    s_state = State::Scanning;
    s_scan_started_us = esp_timer_get_time();
    shared_state().set_link(BikeLink::Scanning);
    ESP_LOGI(TAG, "Scanning for an FTMS bike (0x1826 or name \"%s*\")%s",
             CONFIG_BRIDGE_BIKE_NAME_PREFIX, s_have_saved_addr ? ", preferring saved bike" : "");
}

// ---- GAP events: scan reports + the bike connection ------------------------

int gap_event(ble_gap_event* event, void*) {
    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            on_discovered(event->disc);
            return 0;

        case BLE_GAP_EVENT_DISC_COMPLETE:
            if (s_state == State::Scanning) {  // ended without us cancelling it
                ESP_LOGW(TAG, "Scan ended unexpectedly (reason=%d)", event->disc_complete.reason);
                schedule_rescan();
            }
            return 0;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status != 0) {
                ESP_LOGW(TAG, "Bike connection failed (status=%d)", event->connect.status);
                schedule_rescan();
                return 0;
            }
            s_peer.conn = event->connect.conn_handle;
            s_state = State::Discovering;
            ESP_LOGI(TAG, "Bike connected (handle=%d); discovering FTMS", s_peer.conn);
            if (ble_gattc_exchange_mtu(s_peer.conn, on_mtu, nullptr) != 0) discover_service();
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            if (event->disconnect.conn.conn_handle != s_peer.conn) return 0;
            ESP_LOGW(TAG, "Bike disconnected (reason=0x%03x)", event->disconnect.reason);
            s_peer = Peer{};
            shared_state().clear_values();
            schedule_rescan();
            return 0;

        case BLE_GAP_EVENT_NOTIFY_RX: {
            if (event->notify_rx.conn_handle != s_peer.conn ||
                event->notify_rx.attr_handle != s_peer.ibd_val) {
                return 0;
            }
            uint8_t buf[kMaxNotifyLen];
            const uint16_t total = OS_MBUF_PKTLEN(event->notify_rx.om);
            const uint16_t len = std::min<uint16_t>(total, sizeof(buf));
            os_mbuf_copydata(event->notify_rx.om, 0, len, buf);
            on_indoor_bike_data(buf, len, total);
            return 0;
        }

        case BLE_GAP_EVENT_ENC_CHANGE:
            if (event->enc_change.conn_handle != s_peer.conn) return 0;
            ESP_LOGI(TAG, "Bike link encryption status=%d", event->enc_change.status);
            if (event->enc_change.status == 0 && s_state == State::Discovering &&
                s_peer.security_requested && s_peer.cccd != 0) {
                subscribe();
            }
            return 0;

        case BLE_GAP_EVENT_REPEAT_PAIRING: {
            ble_gap_conn_desc desc;
            if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
                ble_store_util_delete_peer(&desc.peer_id_addr);
            }
            return BLE_GAP_REPEAT_PAIRING_RETRY;
        }

        case BLE_GAP_EVENT_MTU:
            ESP_LOGD(TAG, "MTU event handle=%d mtu=%d", event->mtu.conn_handle, event->mtu.value);
            return 0;

        default:
            return 0;
    }
}

}  // namespace

void init() {
    load_bike_addr();
    ble_npl_callout_init(&s_retry_timer, nimble_port_get_dflt_eventq(), retry_timer_cb, nullptr);
}

void start(uint8_t own_addr_type) {
    s_own_addr_type = own_addr_type;
    ble_npl_callout_stop(&s_retry_timer);
    s_peer = Peer{};
    s_state = State::Idle;
    s_backoff_ms = kBackoffMinMs;
    start_scan();
}

}  // namespace ftms_client
