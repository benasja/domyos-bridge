#include "cps_server.hpp"

#include <array>
#include <cstring>
#include <mutex>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "sdkconfig.h"
// Must come last (see header).
#include "nimble_includes.hpp"

namespace cps {
namespace {

constexpr const char* TAG = "cps";

// ---- Assigned numbers (Bluetooth SIG Assigned Numbers, §3.4 services, §3.8
// characteristics) ---------------------------------------------------------
constexpr uint16_t kUuidCyclingPowerService   = 0x1818;
constexpr uint16_t kUuidDeviceInfoService     = 0x180A;
constexpr uint16_t kUuidCyclingPowerMeas      = 0x2A63;
constexpr uint16_t kUuidCyclingPowerFeature   = 0x2A65;
constexpr uint16_t kUuidSensorLocation        = 0x2A5D;
constexpr uint16_t kUuidManufacturerName      = 0x2A29;
constexpr uint16_t kUuidModelNumber           = 0x2A24;
constexpr uint16_t kUuidFirmwareRevision      = 0x2A26;

// One link is kept free for the bike (central role); the rest may be used by
// Watch / phone centrals. Allowing more than one means a phone that grabs the
// sensor in the background cannot lock the Watch out.
constexpr int kMaxCentrals = CONFIG_BT_NIMBLE_MAX_CONNECTIONS - 1;
static_assert(kMaxCentrals >= 1, "BT_NIMBLE_MAX_CONNECTIONS must be >= 2 (bike + Watch)");

// AD structure overhead: 1 length byte + 1 type byte (Core Spec Vol 3 Part C §11).
constexpr size_t kAdHeader = 2;
constexpr size_t kAdFlagsSize = kAdHeader + 1;
constexpr size_t kAdUuid16ListSize = kAdHeader + 2;
constexpr size_t kAdAppearanceSize = kAdHeader + 2;

constexpr ble_uuid16_t make_uuid16(uint16_t value) {
    ble_uuid16_t u{};
    u.u.type = BLE_UUID_TYPE_16;
    u.value = value;
    return u;
}

const ble_uuid16_t kSvcCps = make_uuid16(kUuidCyclingPowerService);
const ble_uuid16_t kSvcDis = make_uuid16(kUuidDeviceInfoService);
const ble_uuid16_t kChrMeas = make_uuid16(kUuidCyclingPowerMeas);
const ble_uuid16_t kChrFeature = make_uuid16(kUuidCyclingPowerFeature);
const ble_uuid16_t kChrLocation = make_uuid16(kUuidSensorLocation);
const ble_uuid16_t kChrManufacturer = make_uuid16(kUuidManufacturerName);
const ble_uuid16_t kChrModel = make_uuid16(kUuidModelNumber);
const ble_uuid16_t kChrFirmware = make_uuid16(kUuidFirmwareRevision);

uint16_t s_meas_val_handle = 0;
uint8_t s_own_addr_type = BLE_OWN_ADDR_PUBLIC;
bool s_adv_allowed = false;  // set once the host has synced

struct Client {
    uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
    bool subscribed = false;
};
std::mutex s_clients_mutex;
std::array<Client, kMaxCentrals> s_clients;

int gap_event(ble_gap_event* event, void* arg);

// ---- GATT access ----------------------------------------------------------

int append(os_mbuf* om, const void* data, size_t len) {
    return os_mbuf_append(om, data, len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

int gatt_access(uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt* ctxt,
                void* arg) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;

    switch (ble_uuid_u16(ctxt->chr->uuid)) {
        case kUuidCyclingPowerFeature: {
            const uint32_t v = kFeatureCrankRevSupported;
            const uint8_t le[4] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8),
                                   static_cast<uint8_t>(v >> 16), static_cast<uint8_t>(v >> 24)};
            return append(ctxt->om, le, sizeof(le));
        }
        case kUuidSensorLocation:
            return append(ctxt->om, &kSensorLocationRearHub, 1);
        case kUuidManufacturerName:
            return append(ctxt->om, CONFIG_BRIDGE_DIS_MANUFACTURER,
                          strlen(CONFIG_BRIDGE_DIS_MANUFACTURER));
        case kUuidModelNumber:
            return append(ctxt->om, CONFIG_BRIDGE_DIS_MODEL, strlen(CONFIG_BRIDGE_DIS_MODEL));
        case kUuidFirmwareRevision: {
            const char* ver = esp_app_get_description()->version;
            return append(ctxt->om, ver, strlen(ver));
        }
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

ble_gatt_chr_def chr(const ble_uuid16_t& uuid, ble_gatt_chr_flags flags,
                     uint16_t* val_handle = nullptr) {
    ble_gatt_chr_def c{};
    c.uuid = &uuid.u;
    c.access_cb = gatt_access;
    c.flags = flags;
    c.val_handle = val_handle;
    return c;
}

// NimBLE keeps pointers into these tables for the lifetime of the host.
// Characteristics need no encryption so the Watch can also use it unbonded.
std::array<ble_gatt_chr_def, 4> s_cps_chrs;
std::array<ble_gatt_chr_def, 4> s_dis_chrs;
std::array<ble_gatt_svc_def, 3> s_services;

// ---- Client bookkeeping (host task writes, publish task reads) ------------

bool add_client(uint16_t conn_handle) {
    std::lock_guard<std::mutex> lock(s_clients_mutex);
    for (auto& c : s_clients) {
        if (c.conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            c = Client{conn_handle, false};
            return true;
        }
    }
    return false;
}

void remove_client(uint16_t conn_handle) {
    std::lock_guard<std::mutex> lock(s_clients_mutex);
    for (auto& c : s_clients) {
        if (c.conn_handle == conn_handle) c = Client{};
    }
}

bool is_client(uint16_t conn_handle) {
    std::lock_guard<std::mutex> lock(s_clients_mutex);
    for (const auto& c : s_clients) {
        if (c.conn_handle == conn_handle) return true;
    }
    return false;
}

void set_subscribed(uint16_t conn_handle, bool subscribed) {
    std::lock_guard<std::mutex> lock(s_clients_mutex);
    for (auto& c : s_clients) {
        if (c.conn_handle == conn_handle) c.subscribed = subscribed;
    }
}

// ---- Advertising ----------------------------------------------------------

void advertise() {
    if (!s_adv_allowed || ble_gap_adv_active()) return;
    if (connection_count() >= kMaxCentrals) {
        ESP_LOGI(TAG, "All %d central slots in use; not advertising", kMaxCentrals);
        return;
    }

    const char* name = ble_svc_gap_device_name();
    const size_t name_len = strlen(name);

    ble_hs_adv_fields adv{};
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv.uuids16 = &kSvcCps;
    adv.num_uuids16 = 1;
    adv.uuids16_is_complete = 1;
    adv.appearance = kAppearanceCyclingPowerSensor;
    adv.appearance_is_present = 1;

    ble_hs_adv_fields rsp{};
    const size_t used = kAdFlagsSize + kAdUuid16ListSize + kAdAppearanceSize;
    const bool name_in_adv = used + kAdHeader + name_len <= BLE_HS_ADV_MAX_SZ;
    ble_hs_adv_fields& name_fields = name_in_adv ? adv : rsp;
    name_fields.name = reinterpret_cast<const uint8_t*>(name);
    name_fields.name_len = static_cast<uint8_t>(name_len);
    name_fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&adv);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields rc=%d", rc);
        return;
    }
    if (!name_in_adv) {
        rc = ble_gap_adv_rsp_set_fields(&rsp);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields rc=%d", rc);
            return;
        }
    }

    ble_gap_adv_params params{};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    // Interval 0 = NimBLE defaults (30-60 ms), quick to find from the Watch.
    rc = ble_gap_adv_start(s_own_addr_type, nullptr, BLE_HS_FOREVER, &params, gap_event, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "Advertising as \"%s\" (name in %s)", name,
             name_in_adv ? "adv packet" : "scan response");
}

void log_conn(const char* what, uint16_t conn_handle) {
    ble_gap_conn_desc desc;
    if (ble_gap_conn_find(conn_handle, &desc) != 0) {
        ESP_LOGI(TAG, "%s handle=%d", what, conn_handle);
        return;
    }
    const uint8_t* a = desc.peer_id_addr.val;
    ESP_LOGI(TAG, "%s handle=%d peer=%02x:%02x:%02x:%02x:%02x:%02x itvl=%.2fms enc=%d bonded=%d",
             what, conn_handle, a[5], a[4], a[3], a[2], a[1], a[0],
             desc.conn_itvl * 1.25, desc.sec_state.encrypted, desc.sec_state.bonded);
}

// ---- GAP events for peripheral-role connections ---------------------------

int gap_event(ble_gap_event* event, void* arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                if (!add_client(event->connect.conn_handle)) {
                    ESP_LOGW(TAG, "No free central slot; dropping connection");
                    ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                } else {
                    log_conn("Central connected", event->connect.conn_handle);
                }
            } else {
                ESP_LOGW(TAG, "Central connect failed status=%d", event->connect.status);
            }
            advertise();  // keep a slot discoverable if one is free
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Central disconnected handle=%d reason=0x%03x",
                     event->disconnect.conn.conn_handle, event->disconnect.reason);
            remove_client(event->disconnect.conn.conn_handle);
            advertise();
            return 0;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGD(TAG, "Advertising complete reason=%d", event->adv_complete.reason);
            advertise();
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == s_meas_val_handle &&
                is_client(event->subscribe.conn_handle)) {
                set_subscribed(event->subscribe.conn_handle, event->subscribe.cur_notify);
                ESP_LOGI(TAG, "Power measurement notifications %s (handle=%d, reason=%d)",
                         event->subscribe.cur_notify ? "ON" : "OFF",
                         event->subscribe.conn_handle, event->subscribe.reason);
            }
            return 0;

        case BLE_GAP_EVENT_ENC_CHANGE:
            if (event->enc_change.status == 0) {
                log_conn("Encryption on", event->enc_change.conn_handle);
            } else {
                ESP_LOGW(TAG, "Encryption failed handle=%d status=%d",
                         event->enc_change.conn_handle, event->enc_change.status);
            }
            return 0;

        case BLE_GAP_EVENT_REPEAT_PAIRING: {
            // The peer lost its bond (e.g. "Forget device" on the Watch) but we
            // still have ours: drop our copy and let it pair again.
            ble_gap_conn_desc desc;
            if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
                ble_store_util_delete_peer(&desc.peer_id_addr);
            }
            ESP_LOGI(TAG, "Repeat pairing: deleted old bond, retrying");
            return BLE_GAP_REPEAT_PAIRING_RETRY;
        }

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU updated handle=%d mtu=%d", event->mtu.conn_handle,
                     event->mtu.value);
            return 0;

        case BLE_GAP_EVENT_CONN_UPDATE:
            log_conn("Connection params updated", event->conn_update.conn_handle);
            return 0;

        default:
            return 0;
    }
}

}  // namespace

int register_services() {
    s_cps_chrs = {
        chr(kChrMeas, BLE_GATT_CHR_F_NOTIFY, &s_meas_val_handle),
        chr(kChrFeature, BLE_GATT_CHR_F_READ),
        chr(kChrLocation, BLE_GATT_CHR_F_READ),
        ble_gatt_chr_def{},  // terminator
    };
    s_dis_chrs = {
        chr(kChrManufacturer, BLE_GATT_CHR_F_READ),
        chr(kChrModel, BLE_GATT_CHR_F_READ),
        chr(kChrFirmware, BLE_GATT_CHR_F_READ),
        ble_gatt_chr_def{},
    };
    s_services[0] = ble_gatt_svc_def{};
    s_services[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    s_services[0].uuid = &kSvcCps.u;
    s_services[0].characteristics = s_cps_chrs.data();
    s_services[1] = ble_gatt_svc_def{};
    s_services[1].type = BLE_GATT_SVC_TYPE_PRIMARY;
    s_services[1].uuid = &kSvcDis.u;
    s_services[1].characteristics = s_dis_chrs.data();
    s_services[2] = ble_gatt_svc_def{};  // terminator

    int rc = ble_svc_gap_device_name_set(CONFIG_BRIDGE_CPS_DEVICE_NAME);
    if (rc != 0) return rc;
    rc = ble_svc_gap_device_appearance_set(kAppearanceCyclingPowerSensor);
    if (rc != 0) return rc;
    rc = ble_gatts_count_cfg(s_services.data());
    if (rc != 0) return rc;
    return ble_gatts_add_svcs(s_services.data());
}

void start_advertising(uint8_t own_addr_type) {
    s_own_addr_type = own_addr_type;
    s_adv_allowed = true;
    advertise();
}

int notify(const Measurement& m) {
    uint8_t payload[kMeasurementSize];
    encode_measurement(m, payload);

    std::array<uint16_t, kMaxCentrals> targets;
    int count = 0;
    {
        std::lock_guard<std::mutex> lock(s_clients_mutex);
        for (const auto& c : s_clients) {
            if (c.conn_handle != BLE_HS_CONN_HANDLE_NONE && c.subscribed) {
                targets[count++] = c.conn_handle;
            }
        }
    }

    int sent = 0;
    for (int i = 0; i < count; ++i) {
        // mbufs come from NimBLE's fixed msys pool, not the heap. The notify
        // call takes ownership of om whether or not it succeeds.
        os_mbuf* om = ble_hs_mbuf_from_flat(payload, sizeof(payload));
        if (om == nullptr) {
            ESP_LOGW(TAG, "No mbuf for notification");
            continue;
        }
        const int rc = ble_gatts_notify_custom(targets[i], s_meas_val_handle, om);
        if (rc == 0) {
            ++sent;
        } else {
            ESP_LOGW(TAG, "notify handle=%d rc=%d", targets[i], rc);
        }
    }
    return sent;
}

int subscriber_count() {
    std::lock_guard<std::mutex> lock(s_clients_mutex);
    int n = 0;
    for (const auto& c : s_clients) {
        if (c.conn_handle != BLE_HS_CONN_HANDLE_NONE && c.subscribed) ++n;
    }
    return n;
}

int connection_count() {
    std::lock_guard<std::mutex> lock(s_clients_mutex);
    int n = 0;
    for (const auto& c : s_clients) {
        if (c.conn_handle != BLE_HS_CONN_HANDLE_NONE) ++n;
    }
    return n;
}

}  // namespace cps
