#pragma once

// myMota32 only passively scans BLE advertisements; it never pairs or opens
// encrypted BLE links.
#define MYNEWT_VAL_BLE_SM_LEGACY 0
#define MYNEWT_VAL_BLE_SM_SC 0
#define MYNEWT_VAL_BLE_SM_BONDING 0
#define MYNEWT_VAL_BLE_SM_OUR_KEY_DIST 0
#define MYNEWT_VAL_BLE_SM_THEIR_KEY_DIST 0
