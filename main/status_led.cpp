#include "status_led.hpp"

#include "sdkconfig.h"

#if CONFIG_BRIDGE_STATUS_LED

#include <atomic>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace status_led {
namespace {

constexpr const char* TAG = "led";
constexpr gpio_num_t kGpio = static_cast<gpio_num_t>(CONFIG_BRIDGE_STATUS_LED_GPIO);
constexpr bool kActiveLow = CONFIG_BRIDGE_STATUS_LED_ACTIVE_LOW;

// Each pattern is a 1 s frame of 20 x 50 ms slots; bit i set = LED on in slot i.
constexpr TickType_t kSlot = pdMS_TO_TICKS(50);
constexpr int kSlotsPerFrame = 20;
constexpr uint32_t kMaskOff = 0;
constexpr uint32_t kMaskSlowBlink = 0x0000F;            // 200 ms on, 800 ms off
constexpr uint32_t kMaskSolid = (1u << kSlotsPerFrame) - 1;
constexpr uint32_t kMaskDoubleBlink = 0x00033;          // on-off-on, then off

std::atomic<Pattern> s_pattern{Pattern::Off};

uint32_t mask_for(Pattern p) {
    switch (p) {
        case Pattern::SlowBlink: return kMaskSlowBlink;
        case Pattern::Solid: return kMaskSolid;
        case Pattern::DoubleBlink: return kMaskDoubleBlink;
        case Pattern::Off:
        default: return kMaskOff;
    }
}

void write(bool on) { gpio_set_level(kGpio, (on != kActiveLow) ? 1 : 0); }

void led_task(void*) {
    for (;;) {
        const uint32_t mask = mask_for(s_pattern.load(std::memory_order_relaxed));
        for (int slot = 0; slot < kSlotsPerFrame; ++slot) {
            write((mask >> slot) & 1u);
            vTaskDelay(kSlot);
        }
    }
}

}  // namespace

void start() {
    gpio_config_t cfg{};
    cfg.pin_bit_mask = 1ULL << kGpio;
    cfg.mode = GPIO_MODE_OUTPUT;
    if (gpio_config(&cfg) != ESP_OK) {
        ESP_LOGW(TAG, "GPIO%d config failed; status LED disabled", kGpio);
        return;
    }
    write(false);
    xTaskCreate(led_task, "status_led", 2048, nullptr, 1, nullptr);
}

void set(Pattern p) { s_pattern.store(p, std::memory_order_relaxed); }

}  // namespace status_led

#else

namespace status_led {
void start() {}
void set(Pattern) {}
}  // namespace status_led

#endif
