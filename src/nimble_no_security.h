#pragma once

// Arduino ESP32 supplies NimBLE defaults through sdkconfig.h. Include it here
// so the forced include can override only the passive-scan settings below.
#if defined(ESP_PLATFORM)
#include "sdkconfig.h"
#endif

// The application only uses NimBLEDevice + NimBLEScan. Limit role pruning to
// C++ wrapper code so ESP-IDF's prebuilt Bluetooth host stays link-compatible.
#if defined(__cplusplus)
#undef CONFIG_BT_NIMBLE_ROLE_CENTRAL
#undef CONFIG_BT_NIMBLE_ROLE_PERIPHERAL
#undef CONFIG_BT_NIMBLE_ROLE_BROADCASTER
#undef CONFIG_BT_NIMBLE_ROLE_OBSERVER
#undef CONFIG_NIMBLE_ROLE_CENTRAL
#undef CONFIG_NIMBLE_ROLE_PERIPHERAL
#undef CONFIG_NIMBLE_ROLE_BROADCASTER
#undef CONFIG_NIMBLE_ROLE_OBSERVER
#define CONFIG_BT_NIMBLE_ROLE_CENTRAL 0
#define CONFIG_BT_NIMBLE_ROLE_PERIPHERAL 0
#define CONFIG_BT_NIMBLE_ROLE_BROADCASTER 0
#define CONFIG_BT_NIMBLE_ROLE_OBSERVER 1
#endif

// Passive scanning never pairs or opens encrypted BLE links.
#define MYNEWT_VAL_BLE_SM_LEGACY 0
#define MYNEWT_VAL_BLE_SM_SC 0
#define MYNEWT_VAL_BLE_SM_BONDING 0
#define MYNEWT_VAL_BLE_SM_OUR_KEY_DIST 0
#define MYNEWT_VAL_BLE_SM_THEIR_KEY_DIST 0

// We do not use controller or host whitelists for advertisement capture.
#define MYNEWT_VAL_BLE_WHITELIST 0

// No BLE mesh or host debug paths are used by iBeacon capture.
#define MYNEWT_VAL_BLE_MESH 0
#define MYNEWT_VAL_BLE_HS_DEBUG 0
