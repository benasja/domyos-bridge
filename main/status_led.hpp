// Optional, board-specific status LED (default: ESP32-C3 Super Mini, GPIO8,
// active-low). Compiled to no-ops when CONFIG_BRIDGE_STATUS_LED is off.
#pragma once

namespace status_led {

enum class Pattern : unsigned char {
    Off,
    SlowBlink,    // scanning for the bike
    Solid,        // bike connected
    DoubleBlink,  // bike connected and a Watch subscribed
};

void start();
void set(Pattern p);

}  // namespace status_led
