// NimBLE central: finds the Domyos bike, subscribes to Indoor Bike Data
// (0x2AD2) and feeds decoded samples into shared_state. Read-only: never
// writes the FTMS Control Point.
#pragma once

#include <cstdint>

namespace ftms_client {

// Loads the preferred bike address from NVS and prepares timers. Call once
// after nimble_port_init() and nvs_flash_init(), before the host task starts.
void init();

// Starts scanning. Call from the host sync callback (NimBLE host task); safe
// to call again after a host reset.
void start(uint8_t own_addr_type);

}  // namespace ftms_client
