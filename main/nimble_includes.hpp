// Single entry point for NimBLE headers in C++ code.
//
// NimBLE's porting layer (os/os.h) defines function-like `min`/`max` macros,
// which break <limits>, <algorithm> etc. if any standard header is included
// after it. Include this last, after all standard headers.
#pragma once

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#undef min
#undef max

// Provided by NimBLE's store/config (NVS-backed with BT_NIMBLE_NVS_PERSIST),
// but not declared in any public header.
extern "C" void ble_store_config_init(void);
