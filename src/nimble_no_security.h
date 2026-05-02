#pragma once

// Arduino ESP32 supplies NimBLE defaults through sdkconfig.h. Include it here
// so the forced include can override only the passive-scan settings below.
#if defined(ESP_PLATFORM)
#include "sdkconfig.h"
#endif

// mymota32 only uses IPv4 Wi-Fi and LAN MQTT/HTTP. Keep Arduino/lwIP wrapper
// code from referencing IPv6 and PPP paths from the prebuilt ESP32 libraries.
#undef CONFIG_LWIP_IPV6
#undef CONFIG_LWIP_IPV6_AUTOCONFIG
#undef CONFIG_LWIP_IPV6_DHCP6
#undef CONFIG_LWIP_IPV6_FORWARD

// No PPP/cellular transport is used; devices are Wi-Fi only.
#undef CONFIG_LWIP_PPP_SUPPORT
#undef CONFIG_LWIP_PPP_NOTIFY_PHASE_SUPPORT
#undef CONFIG_LWIP_PPP_PAP_SUPPORT
#undef CONFIG_LWIP_PPP_VJ_HEADER_COMPRESSION
#undef CONFIG_PPP_SUPPORT
#undef CONFIG_PPP_NOTIFY_PHASE_SUPPORT
#undef CONFIG_PPP_PAP_SUPPORT

// The C host must keep central enabled for ESP-IDF link compatibility, but
// mymota32 never acts as a BLE peripheral or advertiser.
#undef CONFIG_BT_NIMBLE_ROLE_PERIPHERAL
#undef CONFIG_BT_NIMBLE_ROLE_BROADCASTER
#undef CONFIG_NIMBLE_ROLE_PERIPHERAL
#undef CONFIG_NIMBLE_ROLE_BROADCASTER
#define CONFIG_BT_NIMBLE_ROLE_PERIPHERAL 0
#define CONFIG_BT_NIMBLE_ROLE_BROADCASTER 0

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
#define CONFIG_BT_NIMBLE_CRYPTO_STACK_MBEDTLS 1
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
