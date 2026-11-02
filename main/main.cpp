// Domyos FTMS -> Cycling Power bridge.
//
// The FTMS client (NimBLE host task) writes bike data into shared_state; the
// 1 Hz publish task turns it into Cycling Power notifications. With
// CONFIG_BRIDGE_FAKE_DATA the bike is ignored and a constant power/cadence is
// published instead (for testing the Watch side alone).

#include <initializer_list>

#include "cps_server.hpp"
#include "crank_synth.hpp"
#include "ftms_client.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "shared_state.hpp"
#include "status_led.hpp"
// Must come last (see header).
#include "nimble_includes.hpp"

namespace {

constexpr const char* TAG = "bridge";

constexpr TickType_t kPublishPeriod = pdMS_TO_TICKS(1000);  // CPS notify rate: 1 Hz
constexpr int64_t kStaleAfterUs = CONFIG_BRIDGE_STALE_TIMEOUT_MS * 1000LL;
constexpr int kInfoLogEveryN = 5;  // INFO summary every 5 s; per-sample at DEBUG

constexpr uint32_t kPublishTaskStack = 4096;
constexpr UBaseType_t kPublishTaskPriority = 5;

uint8_t s_own_addr_type = BLE_OWN_ADDR_PUBLIC;

void on_host_reset(int reason) { ESP_LOGE(TAG, "NimBLE host reset, reason=%d", reason); }

void on_host_sync() {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "No usable BLE address rc=%d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto rc=%d", rc);
        return;
    }
    uint8_t addr[6] = {};
    ble_hs_id_copy_addr(s_own_addr_type, addr, nullptr);
    ESP_LOGI(TAG, "BLE address %02x:%02x:%02x:%02x:%02x:%02x (type %d)", addr[5], addr[4],
             addr[3], addr[2], addr[1], addr[0], s_own_addr_type);

    cps::start_advertising(s_own_addr_type);
#if !CONFIG_BRIDGE_FAKE_DATA
    ftms_client::start(s_own_addr_type);
#endif
}

void host_task(void*) {
    nimble_port_run();  // returns only after nimble_port_stop()
    nimble_port_freertos_deinit();
}

status_led::Pattern led_pattern(const BikeSnapshot& s) {
    if (s.link != BikeLink::Connected) return status_led::Pattern::SlowBlink;
    return cps::subscriber_count() > 0 ? status_led::Pattern::DoubleBlink
                                       : status_led::Pattern::Solid;
}

void publish_task(void*) {
    crank::CrankSynth synth;
    TickType_t last_wake = xTaskGetTickCount();
    int tick = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, kPublishPeriod);
        const int64_t now_us = esp_timer_get_time();

#if CONFIG_BRIDGE_FAKE_DATA
        ftms::IndoorBikeData fake;
        fake.instantaneous_cadence_rpm = static_cast<float>(CONFIG_BRIDGE_FAKE_CADENCE_RPM);
        fake.instantaneous_power_w = static_cast<int16_t>(CONFIG_BRIDGE_FAKE_POWER_W);
        shared_state().apply(fake, now_us);
#endif

        const BikeSnapshot s = shared_state().snapshot();
        const bool cadence_fresh = s.cadence_rpm && s.fresh(s.cadence_updated_us, now_us, kStaleAfterUs);
        const bool power_fresh = s.power_w && s.fresh(s.power_updated_us, now_us, kStaleAfterUs);

        // Stale or missing cadence => 0 rpm, which freezes the crank counters.
        synth.update(now_us, publishable_cadence_rpm(s, now_us, kStaleAfterUs));
        const auto crank = synth.output();

        const cps::Measurement m{
            publishable_power_w(s, now_us, kStaleAfterUs, crank::kStoppedCadenceRpm),
            crank.cumulative_revs,
            crank.last_event_time,
        };
        const int sent = cps::notify(m);
        status_led::set(led_pattern(s));

        ESP_LOGD(TAG, "tx power=%d revs=%u t=%u (%d subscriber(s))", m.power_w,
                 m.cumulative_crank_revs, m.last_crank_event_time, sent);
        if (++tick % kInfoLogEveryN == 0) {
            ESP_LOGI(TAG,
                     "bike=%s cadence=%s%.1f power=%s%d%s | CPS: %d W, revs=%u, evt=%u/1024s | "
                     "centrals=%d subscribed=%d",
                     s.link == BikeLink::Connected ? "connected" : "not connected",
                     cadence_fresh ? "" : "(stale)", s.cadence_rpm.value_or(0.0f),
                     power_fresh ? "" : "(stale)", s.power_raw_w.value_or(0),
                     s.power_held ? " (spike, held)" : "", m.power_w,
                     m.cumulative_crank_revs, m.last_crank_event_time, cps::connection_count(),
                     sent);
        }
    }
}

void init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase (%s); bonds will be lost",
                 esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

}  // namespace

extern "C" void app_main(void) {
#if CONFIG_BRIDGE_VERBOSE_LOG
    for (const char* tag : {"bridge", "cps", "ftms"}) esp_log_level_set(tag, ESP_LOG_DEBUG);
#endif
    init_nvs();
    status_led::start();

#if CONFIG_BRIDGE_POWER_SPIKE_FILTER
    power_filter::Config spike_cfg;
    spike_cfg.enabled = true;
    spike_cfg.min_jump_w = CONFIG_BRIDGE_SPIKE_MIN_JUMP_W;
    spike_cfg.jump_percent = CONFIG_BRIDGE_SPIKE_JUMP_PERCENT;
    spike_cfg.confirm_us = int64_t{CONFIG_BRIDGE_SPIKE_CONFIRM_MS} * 1000;
    shared_state().configure_power_filter(spike_cfg);
    ESP_LOGI(TAG, "Power spike filter: rises > %d W and > %d%% held for %d ms",
             CONFIG_BRIDGE_SPIKE_MIN_JUMP_W, CONFIG_BRIDGE_SPIKE_JUMP_PERCENT,
             CONFIG_BRIDGE_SPIKE_CONFIRM_MS);
#endif

#if CONFIG_BRIDGE_FAKE_DATA
    ESP_LOGW(TAG, "*** FAKE DATA MODE: sending constant %d W / %d rpm ***",
             CONFIG_BRIDGE_FAKE_POWER_W, CONFIG_BRIDGE_FAKE_CADENCE_RPM);
    shared_state().set_link(BikeLink::Connected);
#else
    shared_state().set_link(BikeLink::Scanning);
#endif

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.reset_cb = on_host_reset;
    ble_hs_cfg.sync_cb = on_host_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Just Works bonding: no display/keyboard, no MITM protection, LE Secure
    // Connections preferred. Distribute encryption + identity keys so the Watch
    // (which uses a resolvable private address) is recognised on reconnect.
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    const int rc = cps::register_services();
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT registration failed rc=%d", rc);
        abort();
    }
    ble_store_config_init();
#if !CONFIG_BRIDGE_FAKE_DATA
    ftms_client::init();
#endif

    nimble_port_freertos_init(host_task);
    xTaskCreate(publish_task, "publish", kPublishTaskStack, nullptr, kPublishTaskPriority, nullptr);
}
