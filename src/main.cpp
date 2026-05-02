#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <bootloader_common.h>
#include <cJSON.h>
#include <esp_chip_info.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <string.h>
#include <math.h>

#ifndef MYMOTA32_VERSION
#define MYMOTA32_VERSION "dev"
#endif

#ifndef MYMOTA32_TARGET
#define MYMOTA32_TARGET "esp32"
#endif

#ifndef MYMOTA32_ESP32_U4WDH
#define MYMOTA32_ESP32_U4WDH 0
#endif

#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ROLE_OBSERVER
#define MYMOTA32_IBEACON_SUPPORTED 1
#else
#define MYMOTA32_IBEACON_SUPPORTED 0
#endif

#if CONFIG_IDF_TARGET_ESP32C3
#define MYMOTA32_LIGHT_SUPPORTED 1
#else
#define MYMOTA32_LIGHT_SUPPORTED 0
#endif

namespace {

constexpr uint32_t kConnectTimeoutMs = 20000;
constexpr uint32_t kWifiReconnectBeginMs = 60000;
constexpr uint32_t kInitialFallbackApMs = 300000;
constexpr uint32_t kApRetryMs = 10000;
constexpr uint32_t kWifiDynamicPowerSettleMs = 30000;
constexpr uint32_t kWifiDynamicPowerSampleMs = 2000;
constexpr uint8_t kWifiDynamicPowerSampleCount = 5;
constexpr uint8_t kWifiDynamicPowerDisconnectLimit = 2;
constexpr uint8_t kWifiDynamicPowerDefault = 1;
constexpr int8_t kWifiTxPowerMaxQdbm = 82;     // 20.5 dBm
constexpr int8_t kWifiTxPowerMediumQdbm = 52;  // 13.0 dBm
constexpr int8_t kWifiTxPowerStrongQdbm = 40;  // 10.0 dBm
constexpr uint32_t kBootRecoveryStableMs = 30000;
constexpr uint8_t kBootRecoveryLimit = 5;
constexpr size_t kUpdateSectorSize = 4096;
constexpr size_t kUpdateHeaderHoldBytes = 16;
constexpr uint8_t kEspImageMagic = 0xE9;
constexpr uint8_t UPDATE_ERROR_OK = 0;
constexpr uint8_t UPDATE_ERROR_WRITE = 1;
constexpr uint8_t UPDATE_ERROR_ERASE = 2;
constexpr uint8_t UPDATE_ERROR_READ = 3;
constexpr uint8_t UPDATE_ERROR_SPACE = 4;
constexpr uint8_t UPDATE_ERROR_SIZE = 5;
constexpr uint8_t UPDATE_ERROR_STREAM = 6;
constexpr uint8_t UPDATE_ERROR_MD5 = 7;
constexpr uint8_t UPDATE_ERROR_MAGIC_BYTE = 8;
constexpr uint8_t UPDATE_ERROR_ACTIVATE = 9;
constexpr uint8_t UPDATE_ERROR_NO_PARTITION = 10;
constexpr uint8_t UPDATE_ERROR_BAD_ARGUMENT = 11;
constexpr uint8_t UPDATE_ERROR_ABORT = 12;
constexpr uint8_t kUpdateErrorTargetMismatch = 250;
constexpr uint8_t kPhyModeAuto = 0;
constexpr uint8_t kPhyModeB = 1;
constexpr uint8_t kPhyModeG = 2;
constexpr uint8_t kPhyModeN = 3;
constexpr uint8_t kPhyModeFailsafe = kPhyModeG;

constexpr size_t kSsidMaxLen = 32;
constexpr size_t kPasswordMaxLen = 64;
constexpr size_t kHostnameMaxLen = 32;

constexpr size_t kTemplateGpioCount = 36;
constexpr size_t kTemplateNameMaxLen = 32;
constexpr size_t kTemplateJsonMaxLen = 800;

constexpr uint8_t kInvalidPin = 0xff;
#if CONFIG_IDF_TARGET_ESP32C3
constexpr uint8_t kEspMaxGpio = 21;
constexpr uint8_t kEspFlashFirst = 11;
constexpr uint8_t kEspFlashLast = 17;
#else
constexpr uint8_t kEspMaxGpio = 39;
constexpr uint8_t kEspFlashFirst = 6;
constexpr uint8_t kEspFlashLast = 11;
#endif

constexpr uint8_t kMaxRelays = 4;
constexpr uint8_t kMaxButtons = 4;
constexpr uint8_t kMaxLeds = 4;
constexpr uint8_t kMaxLedOutputs = kMaxLeds + 1;
constexpr uint8_t kLightDimmerOff = 0;
constexpr uint8_t kLightDimmerMin = 1;
constexpr uint8_t kLightDimmerMax = 100;
constexpr uint8_t kLightPowerOnDimmerDefault = 50;
constexpr uint16_t kLightCtMin = 153;
constexpr uint16_t kLightCtMax = 500;
constexpr uint16_t kLightCtDefault = 326;
constexpr uint8_t kLightModeWhite = 0;
constexpr uint8_t kLightModeRgb = 1;
constexpr uint32_t kLightPersistDelayMs = 2000;
constexpr uint8_t kLightFadeDefault = 0;
constexpr uint8_t kLightSpeedDefault = 1;
constexpr uint8_t kLightSpeedMin = 1;
constexpr uint8_t kLightSpeedMax = 40;
constexpr uint8_t kLightChannelCount = 5;
constexpr uint16_t kLightChannelMax = 1023;
constexpr uint16_t kLightFadeStepMs = 25;
constexpr uint8_t kSm2335AddrStandby = 0xc0;
constexpr uint8_t kSm2335AddrStart5Ch = 0xd8;
constexpr uint8_t kSm2335DelayUs = 2;

constexpr uint16_t kButtonHoldDefaultMs = 500;
constexpr uint16_t kButtonHoldMinMs = 100;
constexpr uint16_t kButtonHoldMaxMs = 60000;
constexpr uint16_t kButtonDebounceDefaultMs = 50;
constexpr uint16_t kButtonDebounceMinMs = 5;
constexpr uint16_t kButtonDebounceMaxMs = 200;
constexpr uint32_t kLedUpdateMs = 50;
constexpr uint8_t kPowerSavingOff = 0;
constexpr uint8_t kPowerSavingLight = 1;
constexpr uint8_t kPowerSavingDeep = 2;
constexpr uint16_t kRelayEnforcementMinSeconds = 1;
constexpr uint16_t kRelayEnforcementMaxSeconds = 65535U;
constexpr uint16_t kRelayPulseMinSeconds = 1;
constexpr uint16_t kRelayPulseMaxSeconds = 65535U;
constexpr uint32_t kGracefulRelaySnapshotMagic = 0x4d523252UL;  // MR2R
constexpr uint16_t kGracefulRelaySnapshotVersion = 1;
constexpr const char *kGracefulRelayPrefsNamespace = "mymota32-rl";
constexpr const char *kGracefulRelayPrefsKey = "snap";
constexpr const char *kLastRelayPrefsKey = "last";

constexpr uint8_t kInputModeButton = 0;
constexpr uint8_t kInputModeSwitch = 1;
constexpr uint8_t kInputModeUnset = 255;
constexpr uint8_t kInputOnLevelLow = 0;
constexpr uint8_t kInputOnLevelHigh = 1;
constexpr uint8_t kInputOnLevelUnset = 255;
constexpr uint8_t kInputKindButton = 0;
constexpr uint8_t kInputKindSwitch = 1;
constexpr uint8_t kButtonRelayUnset = 255;

constexpr uint8_t kButtonActionNone = 0;
constexpr uint8_t kButtonActionRelayToggle = 1;
constexpr uint8_t kButtonActionMqtt = 2;
constexpr uint8_t kButtonActionWebhook = 3;
constexpr size_t kButtonActionTargetMaxLen = 128;
constexpr size_t kButtonActionPayloadMaxLen = 128;
constexpr uint16_t kApiSettingsVersion = 2;
constexpr const char *kDefaultButtonMqttTopic = "stat/{TOPIC}/RESULT";
constexpr const char *kDefaultButtonMqttPressPayload = "{\"Switch{BUTTONID}\":{\"Action\":\"{TYPE}\"}}";
constexpr const char *kDefaultButtonMqttHoldPayload = "{\"Switch{BUTTONID}\":{\"Action\":\"{TYPE}\"}}";

constexpr uint32_t kWebhookConnectTimeoutMs = 500;
constexpr uint32_t kWebhookFlushTimeoutMs = 50;
constexpr uint32_t kWebhookStopTimeoutMs = 25;

constexpr uint8_t kLedAttachNone = 0;
constexpr uint8_t kLedAttachRelayBase = 1;
constexpr uint8_t kLedAttachButtonBase = 33;

constexpr size_t kMqttHostMaxLen = 64;
constexpr size_t kMqttTopicMaxLen = 32;
constexpr uint16_t kMqttDefaultPort = 1883;
constexpr uint16_t kMqttKeepaliveMax = 65535U;
constexpr uint16_t kMqttProtocolKeepaliveSec = 30;
constexpr uint32_t kMqttReconnectMs = 5000;
constexpr uint32_t kMqttConnectTimeoutMs = 650;
constexpr uint32_t kMqttConnackTimeoutMs = 500;
constexpr uint32_t kMqttIoTimeoutMs = 250;
constexpr uint32_t kMqttInboundReadTimeoutMs = 20;
constexpr uint32_t kMqttBrokerSilenceTimeoutMs = static_cast<uint32_t>(kMqttProtocolKeepaliveSec) * 2000UL;
constexpr uint32_t kMqttConnackMaxRemainingLength = 2;
constexpr uint32_t kMqttSubackMaxRemainingLength = 16;
constexpr uint32_t kMqttInboundMaxRemainingLength = 512;
constexpr uint8_t kMqttInboundPacketLimit = 4;
constexpr uint16_t kMqttCommandPacketId = 1;
constexpr size_t kMqttCommandTopicMaxLen = 5 + kMqttTopicMaxLen + 2;
constexpr size_t kMqttInboundTopicMaxLen = kMqttCommandTopicMaxLen + 32;
constexpr size_t kMqttInboundPayloadMaxLen = 96;
constexpr uint8_t kMqttButtonQueueDepth = 4;
constexpr size_t kMqttButtonTopicMaxLen = kButtonActionTargetMaxLen + kMqttTopicMaxLen + 16;
constexpr size_t kMqttButtonPayloadMaxLen = kButtonActionPayloadMaxLen + kMqttTopicMaxLen + 24;
constexpr uint32_t kMqttButtonQueueMaxAgeMs = 5000;
constexpr uint8_t kMqttLightPendingDimmer = 0x01;
constexpr uint8_t kMqttLightPendingCt = 0x02;
constexpr uint8_t kMqttLightPendingColor = 0x04;
constexpr uint8_t kMqttLightPendingFade = 0x08;
constexpr uint8_t kMqttLightPendingSpeed = 0x10;
constexpr uint8_t kMqttLightPendingAll = kMqttLightPendingDimmer | kMqttLightPendingCt | kMqttLightPendingColor | kMqttLightPendingFade | kMqttLightPendingSpeed;
constexpr uint16_t kMqttEnergyIntervalMax = 65535U;
constexpr float kMqttEnergyChangeMaxPercent = 1000.0f;
constexpr uint16_t kMqttEnergyChangeMaxWatts = 65535U;
constexpr uint8_t kMqttPacketConnack = 0x20;
constexpr uint8_t kMqttPacketPublish = 0x30;
constexpr uint8_t kMqttPacketPuback = 0x40;
constexpr uint8_t kMqttPacketSubscribe = 0x82;
constexpr uint8_t kMqttPacketSuback = 0x90;
constexpr uint8_t kMqttPacketPingreq = 0xc0;
constexpr uint8_t kMqttPacketPingresp = 0xd0;

constexpr uint8_t kMqttConnectIdle = 0;
constexpr uint8_t kMqttConnectOk = 1;
constexpr uint8_t kMqttConnectTcpFailed = 2;
constexpr uint8_t kMqttConnectWriteFailed = 3;
constexpr uint8_t kMqttConnectConnackTimeout = 4;
constexpr uint8_t kMqttConnectConnackRejected = 5;
constexpr uint8_t kMqttConnectSubscribeFailed = 6;

constexpr uint32_t kEnergyIntegrateMs = 1000;
constexpr uint32_t kEnergyPersistMinMs = 600000;
constexpr uint64_t kEnergyPersistDeltaUkwh = 10000;
constexpr uint64_t kEnergyTotalMaxUkwh = 1000000000000ULL;
constexpr float kEnergyTotalOffsetMinKwh = 0.0f;
constexpr float kEnergyTotalOffsetMaxKwh = 1000000.0f;
constexpr float kEnergyZeroPowerThreshold = 0.001f;
constexpr uint8_t kEnergyDriverNone = 0;
constexpr uint8_t kEnergyDriverBl0939 = 1;
constexpr uint8_t kEnergyDriverHlw8012 = 2;
constexpr uint8_t kEnergyMaxChannels = 2;
constexpr uint32_t kHlwUpdateMs = 200;
constexpr uint32_t kHlwPowerProbeUs = 10000000UL;
constexpr uint8_t kHlwCf1CycleTicks = 8;
constexpr uint8_t kHlwCf1SampleStartTick = 2;
constexpr uint8_t kHlwCf1SampleEndTick = 8;
constexpr uint32_t kHlwPowerRatio = 10000;
constexpr uint32_t kHlwVoltageRatio = 2200;
constexpr uint32_t kHlwCurrentRatio = 4545;
constexpr uint32_t kHjlPowerRatio = 1362;
constexpr uint32_t kHjlVoltageRatio = 822;
constexpr uint32_t kHjlCurrentRatio = 3300;
constexpr uint32_t kHlwPowerCalibration = 12530;
constexpr uint32_t kHlwVoltageCalibration = 1950;
constexpr uint32_t kHlwCurrentCalibration = 3500;
constexpr uint32_t kBl0939PollMs = 1000;
constexpr uint32_t kBl0939StaleMs = 5000;
constexpr uint8_t kBl0939BufferSize = 35;
constexpr uint8_t kBl0939Address = 0x05;
constexpr uint8_t kBl09xxReadCommand = 0x50;
constexpr uint8_t kBl09xxWriteCommand = 0xa0;
constexpr uint8_t kBl09xxFullPacket = 0xaa;
constexpr uint8_t kBl09xxPacketHeader = 0x55;
constexpr uint32_t kBl0939PowerRef = 713;
constexpr uint32_t kBl0939VoltageRef = 17159;
constexpr uint32_t kBl0939CurrentRef = 266013;
constexpr uint8_t kBl09xxInit[][4] = {
  {0x19, 0x5a, 0x5a, 0x5a},
  {0x1a, 0x55, 0x00, 0x00},
  {0x18, 0x00, 0x10, 0x00},
  {0x1b, 0xff, 0x47, 0x00},
  {0x10, 0x1c, 0x18, 0x00}
};

constexpr uint8_t kMqttEnergyReportReasonNone = 0;
constexpr uint8_t kMqttEnergyReportReasonInitial = 1;
constexpr uint8_t kMqttEnergyReportReasonInterval = 2;
constexpr uint8_t kMqttEnergyReportReasonPowerChangePercent = 3;
constexpr uint8_t kMqttEnergyReportReasonIntervalPowerChangePercent = 4;
constexpr uint8_t kMqttEnergyReportReasonPowerChangeWatts = 5;
constexpr uint8_t kMqttEnergyReportReasonIntervalPowerChangeWatts = 6;
constexpr uint8_t kMqttEnergyReportReasonPowerChangePercentWatts = 7;
constexpr uint8_t kMqttEnergyReportReasonIntervalPowerChangePercentWatts = 8;
constexpr uint8_t kMqttEnergyReportReasonRelayOff = 9;
constexpr uint8_t kMqttEnergyReportReasonPowerZero = 10;

constexpr uint8_t kIBeaconQueueDepth = 12;
constexpr uint8_t kIBeaconMaxPacketBytes = 62;
constexpr uint8_t kIBeaconProcessLimit = 4;
constexpr uint8_t kIBeaconCacheSize = 32;
constexpr size_t kIBeaconFilterListMaxLen = 255;
constexpr size_t kIBeaconFilterInputMaxLen = 384;
constexpr uint32_t kIBeaconPruneIntervalMs = 60000;
constexpr uint32_t kIBeaconKeyfobCacheTtlMs = 1800000;
constexpr uint32_t kIBeaconClimateCacheTtlMs = 21600000;
constexpr uint16_t kIBeaconScanIntervalMs = 160;
constexpr uint16_t kIBeaconScanWindowMs = 80;
constexpr uint8_t kIBeaconKindKeyfob = 1;
constexpr uint8_t kIBeaconKindClimate = 2;
constexpr uint16_t kIBeaconFilter1DefaultSec = 60;
constexpr uint16_t kIBeaconFilter2DefaultSec = 10;
const uint16_t kIBeaconFilterIntervals[] = {1, 5, 10, 15, 30, 60, 120, 300, 600};

constexpr uint8_t kPowerStateOff = 0;
constexpr uint8_t kPowerStateOn = 1;
constexpr uint8_t kPowerStateToggle = 2;

constexpr uint16_t kTplNone = 0;
constexpr uint16_t kTplUser = 1;
constexpr uint16_t kTplKey1 = 32;
constexpr uint16_t kTplKey1Np = 64;
constexpr uint16_t kTplKey1Inv = 96;
constexpr uint16_t kTplKey1InvNp = 128;
constexpr uint16_t kTplSwt1 = 160;
constexpr uint16_t kTplSwt1Np = 192;
constexpr uint16_t kTplRel1 = 224;
constexpr uint16_t kTplRel1Inv = 256;
constexpr uint16_t kTplLed1 = 288;
constexpr uint16_t kTplLed1Inv = 320;
constexpr uint16_t kTplLedLnk = 544;
constexpr uint16_t kTplLedLnkInv = 576;
constexpr uint16_t kTplI2cScl = 608;
constexpr uint16_t kTplI2cSda = 640;
constexpr uint16_t kTplNrgSel = 2592;
constexpr uint16_t kTplNrgSelInv = 2624;
constexpr uint16_t kTplNrgCf1 = 2656;
constexpr uint16_t kTplHlwCf = 2688;
constexpr uint16_t kTplHjlCf = 2720;
constexpr uint16_t kTplTxd = 3200;
constexpr uint16_t kTplAde7953Irq = 3456;
constexpr uint16_t kTplAdcInput = 4704;
constexpr uint16_t kTplAdcTemp = 4736;
constexpr uint16_t kTplBl0939Rx = 8128;
constexpr uint16_t kTplSm2335Clk = 9088;
constexpr uint16_t kTplSm2335Dat = 9120;
constexpr uint16_t kTplOptionA = 9472;
constexpr uint16_t kTplSentinelEnd = 65504;
constexpr uint8_t kTplOptionACount = 9;

#if CONFIG_IDF_TARGET_ESP32C3
constexpr size_t kTemplateJsonMinGpioCount = 22;
#else
constexpr size_t kTemplateJsonMinGpioCount = kTemplateGpioCount;
const uint8_t kEsp32TemplateToPhy[kTemplateGpioCount] = {
  0, 1, 2, 3, 4, 5,
  9, 10,
  12, 13, 14, 15, 16, 17, 18, 19,
  20, 21, 22, 23, 24, 25, 26, 27,
  6, 7, 8, 11,
  32, 33, 34, 35, 36, 37, 38, 39
};
#endif

#if CONFIG_IDF_TARGET_ESP32C3
const char kTemplateSwitchbotW1401400Json[] PROGMEM =
  "{\"NAME\":\"Switchbot W1401400\",\"GPIO\":[0,0,0,0,9128,9088,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],\"FLAG\":0,\"BASE\":1,\"CMND\":\"SetOption37 25\"}";
const char kTemplateGenericC3RelayJson[] PROGMEM =
  "{\"NAME\":\"Generic C3 Relay\",\"GPIO\":[32,0,0,0,224,288,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],\"FLAG\":0,\"BASE\":1}";
#else
const char kTemplateShellyPlusPlugSJson[] PROGMEM =
  "{\"NAME\":\"Shelly Plus Plug S\",\"GPIO\":[0,0,0,0,224,0,32,2720,0,0,0,0,0,0,0,2624,0,0,2656,0,0,288,289,0,0,0,0,0,0,4736,0,0,0,0,0,0],\"FLAG\":0,\"BASE\":1}";
const char kTemplateShellyPlus2PmPcb019Json[] PROGMEM =
  "{\"NAME\":\"Shelly Plus 2PM PCB v0.1.9\",\"GPIO\":[320,0,0,0,34,192,0,0,225,224,0,0,0,0,193,0,0,0,0,0,0,608,640,3458,0,0,0,0,0,9472,0,4736,0,0,0,0],\"FLAG\":0,\"BASE\":1}";
const char kTemplateShellyPlus1PmJson[] PROGMEM =
  "{\"NAME\":\"Shelly Plus 1PM\",\"GPIO\":[0,0,0,0,192,2720,0,0,0,0,0,0,0,0,2656,0,0,0,0,2624,0,32,224,0,0,0,0,0,4736,0,0,0,0,0,0,0],\"FLAG\":0,\"BASE\":1}";
const char kTemplateShellyPlus1Json[] PROGMEM =
  "{\"NAME\":\"Shelly Plus 1 \",\"GPIO\":[288,0,0,0,192,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,32,224,0,0,0,0,0,4736,4705,0,0,0,0,0,0],\"FLAG\":0,\"BASE\":1}";
const char kTemplateShellyPlusI4Json[] PROGMEM =
  "{\"NAME\":\"Shelly Plus i4\",\"GPIO\":[0,0,0,0,0,0,0,0,192,0,193,0,0,0,0,0,0,0,0,0,0,0,195,194,0,0,0,0,0,0,0,0,0,0,0,0],\"FLAG\":0,\"BASE\":1}";
const char kTemplateNousA8tJson[] PROGMEM =
  "{\"NAME\":\"NOUS A8T\",\"GPIO\":[1,1,320,1,32,1,1,1,1,224,2624,1,1,1,1,1,0,1,1,1,0,1,2656,2720,0,0,0,0,1,1,1,1,1,0,0,1],\"FLAG\":0,\"BASE\":1}";
const char kTemplateNousB1tJson[] PROGMEM =
  "{\"NAME\":\"NOUS B1T\",\"GPIO\":[544,0,1,0,32,160,1,1,224,0,0,1,1,1,0,1,0,1,1,1,0,1,1,1,0,0,0,0,1,1,1,0,1,0,0,1],\"FLAG\":0,\"BASE\":1}";
const char kTemplateNousB3tJson[] PROGMEM =
  "{\"NAME\":\"NOUS B3T\",\"GPIO\":[544,3200,1,8128,32,160,1,1,224,225,0,1,1,1,161,1,0,1,1,1,0,1,1,1,0,0,0,0,1,1,1,0,1,0,0,1],\"FLAG\":0,\"BASE\":1}";
const char kTemplateSonoffDualR3V2Json[] PROGMEM =
  "{\"NAME\":\"Sonoff Dual R3 v2\",\"GPIO\":[32,0,0,0,0,0,0,0,0,576,225,0,0,0,0,0,0,0,0,0,0,3200,8128,224,0,0,0,0,160,161,0,0,0,0,0,0],\"FLAG\":0,\"BASE\":1}";
const char kTemplateSonoffMinir4Json[] PROGMEM =
  "{\"NAME\":\"Sonoff MINIR4\",\"GPIO\":[32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,576,0,0,0,0,0,0,224,160,0,0,0,0,0,0,0,0,0,0,0,0],\"FLAG\":0,\"BASE\":1}";
#endif

struct PinAssignment {
  uint8_t pin;
  bool inverted;
  bool no_pullup;
};

struct ButtonState {
  bool raw_pressed;
  bool stable_pressed;
  bool hold_emitted;
  uint32_t changed_at;
  uint32_t pressed_at;
};

struct MqttButtonPending {
  char topic[kMqttButtonTopicMaxLen + 1];
  char payload[kMqttButtonPayloadMaxLen + 1];
  uint32_t queued_at;
};

struct GracefulRelaySnapshot {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t chip_id;
  uint16_t relay_mask;
  uint8_t relay_count;
  uint8_t reserved;
  uint32_t relay_signature;
  uint32_t crc;
};

struct LightState {
  bool present;
  bool power;
  uint8_t dimmer;
  uint16_t ct;
  uint8_t mode;
  uint8_t rgb[3];
  uint16_t channels[kLightChannelCount];
  uint16_t fade_start[kLightChannelCount];
  uint16_t fade_end[kLightChannelCount];
  uint32_t fade_start_ms;
  uint16_t fade_duration_ms;
  uint32_t fade_next_ms;
  bool fade_running;
  bool fade_initialized;
  bool config_dirty;
  uint32_t config_save_at;
};

struct RuntimeTemplate {
  bool enabled;
  char name[kTemplateNameMaxLen + 1];
  uint16_t base;
  uint32_t flag;
  PinAssignment relays[kMaxRelays];
  PinAssignment buttons[kMaxButtons];
  uint8_t input_kind[kMaxButtons];
  uint8_t input_function_index[kMaxButtons];
  uint8_t input_default_relay[kMaxButtons];
  PinAssignment leds[kMaxLeds];
  PinAssignment link_led;
  uint8_t relay_count;
  uint8_t button_count;
  uint8_t led_count;
  uint8_t i2c_scl_pin;
  uint8_t i2c_sda_pin;
  uint8_t energy_cf_pin;
  uint8_t energy_cf1_pin;
  uint8_t energy_sel_pin;
  uint8_t energy_tx_pin;
  uint8_t energy_bl0939_rx_pin;
  uint8_t sm2335_clk_pin;
  uint8_t sm2335_dat_pin;
  uint8_t sm2335_current;
  bool energy_sel_inverted;
  bool energy_hjl;
  bool adc_temp;
  bool sm2335;
  uint8_t unsupported_count;
  uint8_t unsupported_pin[12];
  uint16_t unsupported_code[12];
};

struct StoredConfig {
  char ssid[kSsidMaxLen + 1];
  char password[kPasswordMaxLen + 1];
  char hostname[kHostnameMaxLen + 1];
  uint8_t phy_mode;

  uint8_t template_enabled;
  uint16_t template_base;
  uint32_t template_flag;
  char template_name[kTemplateNameMaxLen + 1];
  uint16_t template_gpio[kTemplateGpioCount];

  uint8_t input_mode[kMaxButtons];
  uint8_t input_relay[kMaxButtons];
  uint8_t input_on_level[kMaxButtons];

  uint8_t button_press_action[kMaxButtons];
  uint8_t button_hold_action[kMaxButtons];
  uint8_t button_press_relay[kMaxButtons];
  uint8_t button_hold_relay[kMaxButtons];
  char button_press_target[kMaxButtons][kButtonActionTargetMaxLen + 1];
  char button_press_payload[kMaxButtons][kButtonActionPayloadMaxLen + 1];
  char button_hold_target[kMaxButtons][kButtonActionTargetMaxLen + 1];
  char button_hold_payload[kMaxButtons][kButtonActionPayloadMaxLen + 1];

  uint16_t button_hold_ms;
  uint16_t button_debounce_ms;

  uint8_t led_attach[kMaxLedOutputs];

  char mqtt_host[kMqttHostMaxLen + 1];
  uint16_t mqtt_port;
  char mqtt_topic[kMqttTopicMaxLen + 1];
  uint16_t mqtt_keepalive;

  float energy_total_offset_kwh;
  uint16_t energy_mqtt_interval;
  uint16_t energy_mqtt_change_percent_x10;
  uint16_t energy_mqtt_change_watts;

  uint8_t relay_restore_boot[kMaxRelays];
  uint8_t relay_on_boot[kMaxRelays];
  uint8_t relay_time_enabled[kMaxRelays];
  uint16_t relay_time_seconds[kMaxRelays];
  uint8_t relay_pulse_enabled[kMaxRelays];
  uint16_t relay_pulse_seconds[kMaxRelays];

  uint8_t light_power;
  uint8_t light_dimmer;
  uint16_t light_ct;
  uint8_t light_mode;
  uint8_t light_rgb[3];
  uint8_t light_on_dimmer;
  uint8_t light_fade;
  uint8_t light_speed;
  uint8_t light_restore_boot;

  uint8_t ibeacon_enabled;
  uint16_t ibeacon_filter1_interval_sec;
  uint16_t ibeacon_filter2_interval_sec;
  char ibeacon_filter1_macs[kIBeaconFilterListMaxLen + 1];
  char ibeacon_filter2_macs[kIBeaconFilterListMaxLen + 1];

  uint16_t power_saving_mode;
  uint8_t wifi_dynamic_power;
};

struct IBeaconObservation {
  char mac[18];
  int8_t rssi;
  uint8_t payload[kIBeaconMaxPacketBytes];
  uint8_t payload_len;
  uint32_t seen_at;
};

struct IBeaconClimateReading {
  bool valid;
  uint32_t hash;
};

struct IBeaconBthomeReading {
  bool has_packet_id;
  uint8_t packet_id;
};

struct IBeaconCacheEntry {
  bool used;
  char mac[18];
  uint8_t kind;
  int8_t rssi;
  uint32_t climate_hash;
  uint8_t packet_id;
  bool has_packet_id;
  uint32_t sent_at;
};

struct EnergyChannelState {
  float voltage;
  float current;
  float power;
  uint32_t current_raw;
  int32_t power_raw;
};

struct EnergyState {
  bool present;
  uint8_t driver;
  uint8_t channel_count;
  uint8_t rx_pin;
  uint8_t tx_pin;
  uint8_t cf_pin;
  uint8_t cf1_pin;
  uint8_t sel_pin;
  bool sel_inverted;
  bool hjl;
  float voltage;
  float current;
  float power;
  float total_kwh;
  float temperature;
  EnergyChannelState channel[kEnergyMaxChannels];
  uint8_t rx_buffer[kBl0939BufferSize];
  uint8_t byte_counter;
  uint16_t tps1;
  uint32_t voltage_raw;
  bool received;
  uint32_t last_poll_ms;
  uint32_t last_success_ms;
  uint32_t last_integrate_ms;
  uint32_t last_hlw_update_ms;
  volatile uint32_t hlw_cf_pulse_length;
  volatile uint32_t hlw_cf_pulse_last_us;
  volatile uint32_t hlw_cf_summed_pulse_length;
  volatile uint32_t hlw_cf_pulse_counter;
  volatile uint32_t hlw_cf1_pulse_length;
  volatile uint32_t hlw_cf1_pulse_last_us;
  volatile uint32_t hlw_cf1_summed_pulse_length;
  volatile uint32_t hlw_cf1_pulse_counter;
  volatile uint8_t hlw_cf1_timer;
  volatile bool hlw_load_off;
  uint32_t hlw_cf_power_pulse_length;
  uint32_t hlw_cf1_voltage_pulse_length;
  uint32_t hlw_cf1_current_pulse_length;
  uint32_t hlw_power_ratio;
  uint32_t hlw_voltage_ratio;
  uint32_t hlw_current_ratio;
  uint8_t hlw_power_retry;
  bool hlw_select_ui_flag;
  bool hlw_voltage_on_selected;
};

StoredConfig config{};
RuntimeTemplate runtime_template{};
EnergyState energy{};
LightState light{};
bool relay_state[kMaxRelays] = {false};
bool graceful_relay_restore_valid = false;
uint16_t graceful_relay_restore_mask = 0;
bool last_relay_restore_valid = false;
uint16_t last_relay_restore_mask = 0;
bool relay_enforcement_pending[kMaxRelays] = {false};
uint32_t relay_enforcement_due[kMaxRelays] = {0};
bool relay_pulse_pending[kMaxRelays] = {false};
uint32_t relay_pulse_due[kMaxRelays] = {0};
ButtonState button_state[kMaxButtons] = {};
uint32_t last_led_update = 0;

bool config_ok = false;
bool ap_started = false;
bool sta_connected_once = false;
uint32_t boot_id = 0;
uint32_t last_ap_attempt = 0;
uint32_t last_wifi_begin_attempt = 0;
uint32_t disconnected_since = 0;
bool disconnected_timer_active = false;
uint32_t wifi_dynamic_power_connected_since = 0;
uint32_t wifi_dynamic_power_last_sample = 0;
uint8_t wifi_dynamic_power_samples = 0;
uint8_t wifi_dynamic_power_disconnects = 0;
int16_t wifi_dynamic_power_rssi_sum = 0;
int8_t wifi_tx_power_qdbm = kWifiTxPowerMaxQdbm;

uint32_t boot_recovery_count = 0;
bool boot_recovery_factory_reset = false;
bool boot_recovery_cleared = false;
uint32_t boot_started_ms = 0;

bool update_started = false;
bool update_ok = false;
uint8_t update_error = UPDATE_ERROR_OK;
const esp_partition_t *update_partition = nullptr;
size_t update_written = 0;
size_t update_erased_until = 0;
uint8_t update_header[kUpdateHeaderHoldBytes] = {0};
size_t update_header_len = 0;

uint32_t restart_due_ms = 0;
uint32_t restart_scheduled_ms = 0;
bool restart_preserve_relays = false;

uint32_t perf_loop_count = 0;
uint64_t perf_busy_us = 0;
uint32_t perf_max_loop_us = 0;
uint32_t perf_window_start_ms = 0;
bool perf_window_started = false;
uint32_t perf_last_loop_hz = 0;
uint8_t perf_last_loop_load = 0;
uint32_t perf_last_loop_max_us = 0;

WebServer server(80);
Preferences prefs;

WiFiClient mqtt_client;
HardwareSerial bl0939_serial(1);
uint32_t next_mqtt_reconnect = 0;
uint32_t last_mqtt_io = 0;
uint32_t last_mqtt_rx = 0;
uint32_t last_mqtt_ping = 0;
uint32_t last_mqtt_state_publish = 0;
uint32_t last_mqtt_energy_publish = 0;
uint32_t last_mqtt_connect_attempt = 0;
uint32_t last_mqtt_connect_duration = 0;
uint8_t last_mqtt_connect_result = kMqttConnectIdle;
uint8_t last_mqtt_energy_report_reason = kMqttEnergyReportReasonNone;
uint8_t mqtt_pending_relay_mask = 0;
uint8_t mqtt_pending_light_mask = 0;
uint16_t mqtt_pending_energy_zero_relay_mask = 0;
uint8_t mqtt_pending_energy_report_reason = kMqttEnergyReportReasonNone;
bool mqtt_ping_pending = false;
float last_mqtt_energy_power = NAN;
float last_observed_energy_power = NAN;

uint64_t energy_saved_ukwh = 0;
uint32_t last_energy_persist_ms = 0;
bool energy_persist_requested = false;

MqttButtonPending mqtt_button_queue[kMqttButtonQueueDepth]{};
uint8_t mqtt_button_queue_head = 0;
uint8_t mqtt_button_queue_count = 0;

IBeaconCacheEntry ibeacon_cache[kIBeaconCacheSize]{};
uint32_t last_ibeacon_prune = 0;
uint32_t next_ibeacon_start_attempt = 0;
bool ibeacon_stack_started = false;
bool ibeacon_scanning = false;
char ibeacon_status[24] = "idle";
uint32_t ibeacon_mqtt_rate_window_start = 0;
uint16_t ibeacon_mqtt_rate_window_count = 0;
uint16_t ibeacon_mqtt_reports_per_minute = 0;

#if MYMOTA32_IBEACON_SUPPORTED
IBeaconObservation ibeacon_queue[kIBeaconQueueDepth]{};
uint8_t ibeacon_queue_head = 0;
uint8_t ibeacon_queue_count = 0;
portMUX_TYPE ibeacon_queue_mux = portMUX_INITIALIZER_UNLOCKED;

class IBeaconScanCallbacks : public NimBLEScanCallbacks {
 public:
  void onResult(const NimBLEAdvertisedDevice *device) override;
};

IBeaconScanCallbacks ibeacon_scan_callbacks;
NimBLEScan *ibeacon_scan = nullptr;
#endif

void recordLoopPerf(uint32_t started_us, uint32_t ended_us) {
  const uint32_t now_ms = millis();
  const uint32_t elapsed_us = ended_us - started_us;
  if (!perf_window_started) {
    perf_window_start_ms = now_ms;
    perf_window_started = true;
  }
  perf_loop_count++;
  perf_busy_us += elapsed_us;
  if (elapsed_us > perf_max_loop_us) perf_max_loop_us = elapsed_us;
  const uint32_t window_ms = now_ms - perf_window_start_ms;
  if (window_ms < 1000) return;
  perf_last_loop_hz = (perf_loop_count * 1000UL) / window_ms;
  const uint64_t window_us = static_cast<uint64_t>(window_ms) * 1000ULL;
  uint32_t load = window_us ? static_cast<uint32_t>((perf_busy_us * 100ULL) / window_us) : 0;
  if (load > 100) load = 100;
  perf_last_loop_load = static_cast<uint8_t>(load);
  perf_last_loop_max_us = perf_max_loop_us;
  perf_window_start_ms = now_ms;
  perf_loop_count = 0;
  perf_busy_us = 0;
  perf_max_loop_us = 0;
}

uint8_t sanitizePhyMode(uint8_t mode) {
  if (mode > kPhyModeN) return kPhyModeAuto;
  return mode;
}

const __FlashStringHelper *phyModeName(uint8_t mode) {
  switch (mode) {
    case kPhyModeB: return F("11b");
    case kPhyModeG: return F("11g");
    case kPhyModeN: return F("11n");
    default: return F("auto");
  }
}

uint8_t sanitizePowerSavingMode(uint16_t mode) {
  return mode <= kPowerSavingDeep ? static_cast<uint8_t>(mode) : kPowerSavingOff;
}

bool parsePowerSavingMode(String value, uint8_t &mode) {
  value.trim();
  value.toLowerCase();
  if (value == F("0") || value == F("off")) {
    mode = kPowerSavingOff;
    return true;
  }
  if (value == F("1") || value == F("light")) {
    mode = kPowerSavingLight;
    return true;
  }
  if (value == F("2") || value == F("deep")) {
    mode = kPowerSavingDeep;
    return true;
  }
  return false;
}

uint8_t activePhyMode() {
  uint8_t protocol = 0;
  if (esp_wifi_get_protocol(WIFI_IF_STA, &protocol) != ESP_OK) return kPhyModeAuto;
  if (protocol & WIFI_PROTOCOL_11N) return kPhyModeN;
  if (protocol & WIFI_PROTOCOL_11G) return kPhyModeG;
  if (protocol & WIFI_PROTOCOL_11B) return kPhyModeB;
  return kPhyModeAuto;
}

uint32_t chipIdValue() {
  const uint64_t mac = ESP.getEfuseMac();
  uint32_t id = 0;
  for (uint8_t i = 0; i < 24; i += 8) {
    id |= static_cast<uint32_t>((mac >> (40 - i)) & 0xffU) << i;
  }
  return id;
}

String chipIdHex() {
  const uint32_t id = chipIdValue();
  char buf[7];
  snprintf(buf, sizeof(buf), "%06X", id);
  return String(buf);
}

String chipModelName() {
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  const uint32_t pkg_version = bootloader_common_get_chip_ver_pkg();
  const bool single_core = chip_info.cores == 1;
  const uint32_t full_revision = chip_info.revision < 100 ? chip_info.revision * 100 : chip_info.revision;
  const bool rev3 = full_revision >= 300;

  switch (chip_info.model) {
    case CHIP_ESP32:
      switch (pkg_version) {
        case 0: return single_core ? F("ESP32-S0WDQ6") : (rev3 ? F("ESP32-D0WDQ6-V3") : F("ESP32-D0WDQ6"));
        case 1: return single_core ? F("ESP32-S0WD") : (rev3 ? F("ESP32-D0WD-V3") : F("ESP32-D0WD"));
        case 2: return F("ESP32-D2WD");
        case 3: return single_core ? F("ESP32-S0WD-OEM") : F("ESP32-D0WD-OEM");
        case 4: return single_core ? F("ESP32-U4WDH-S") : F("ESP32-U4WDH-D");
        case 5: return rev3 ? F("ESP32-PICO-V3") : F("ESP32-PICO-D4");
        case 6: return F("ESP32-PICO-V3-02");
        case 7: return F("ESP32-D0WDR2-V3");
        default: return F("ESP32");
      }
    case CHIP_ESP32C3:
      switch (pkg_version) {
        case 1: return F("ESP8685");
        case 2: return F("ESP32-C3 AZ");
        case 3: return F("ESP8686");
        default: return F("ESP32-C3");
      }
    default:
      return ESP.getChipModel();
  }
}

String chipDisplayName() {
  return chipModelName() + F(" (") + chipIdHex() + F(")");
}

String defaultHostname() {
  return "mymota32-" + chipIdHex();
}

uint32_t makeBootId() {
  return static_cast<uint32_t>(esp_random());
}

String htmlEscape(const String &input) {
  String out;
  out.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); i++) {
    const char c = input[i];
    switch (c) {
      case '&': out += F("&amp;"); break;
      case '<': out += F("&lt;"); break;
      case '>': out += F("&gt;"); break;
      case '"': out += F("&quot;"); break;
      case '\'': out += F("&#39;"); break;
      default: out += c;
    }
  }
  return out;
}

String htmlEscape(const char *input) {
  return htmlEscape(String(input));
}

String jsonEscape(const char *input) {
  String out;
  if (!input) return out;
  for (const char *p = input; *p; p++) {
    const char c = *p;
    switch (c) {
      case '\\': out += F("\\\\"); break;
      case '"': out += F("\\\""); break;
      case '\b': out += F("\\b"); break;
      case '\f': out += F("\\f"); break;
      case '\n': out += F("\\n"); break;
      case '\r': out += F("\\r"); break;
      case '\t': out += F("\\t"); break;
      default:
        if (static_cast<uint8_t>(c) < 0x20) {
          char buf[8];
          snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

String ipToString(IPAddress ip) {
  return ip.toString();
}

String pinName(uint8_t pin) {
  if (pin == kInvalidPin) return F("-");
  return String(F("GPIO")) + String(pin);
}

bool digitalPinSupported(uint8_t pin) {
  if (pin == kInvalidPin) return false;
  if (pin > kEspMaxGpio) return false;
  if (pin >= kEspFlashFirst && pin <= kEspFlashLast) return false;
#if MYMOTA32_ESP32_U4WDH
  if (pin == 16 || pin == 17) return false;
#endif
  return true;
}

void resetPinAssignment(PinAssignment &assignment) {
  assignment.pin = kInvalidPin;
  assignment.inverted = false;
  assignment.no_pullup = false;
}

bool hasPin(const PinAssignment &assignment) {
  return assignment.pin != kInvalidPin;
}

void writeAssignedPin(const PinAssignment &assignment, bool on) {
  if (!digitalPinSupported(assignment.pin)) return;
  digitalWrite(assignment.pin, (on ^ assignment.inverted) ? HIGH : LOW);
}

bool isSwitchInput(uint8_t input) {
  return input < kMaxButtons && runtime_template.input_kind[input] == kInputKindSwitch;
}

uint8_t inputFunctionIndex(uint8_t input) {
  if (input >= kMaxButtons) return input;
  const uint8_t function_index = runtime_template.input_function_index[input];
  return function_index < kMaxButtons ? function_index : input;
}

bool defaultInputRelayTarget(uint8_t input, uint8_t &relay);

uint8_t defaultInputMode(uint8_t input) {
  uint8_t relay = 0;
  return isSwitchInput(input) && defaultInputRelayTarget(input, relay) ? kInputModeSwitch : kInputModeButton;
}

uint8_t effectiveInputMode(uint8_t input) {
  if (input >= kMaxButtons) return kInputModeButton;
  if (config.input_mode[input] == kInputModeButton || config.input_mode[input] == kInputModeSwitch) {
    return config.input_mode[input];
  }
  return defaultInputMode(input);
}

uint8_t defaultInputOnLevel(uint8_t input) {
  if (input >= kMaxButtons || !hasPin(runtime_template.buttons[input])) return kInputOnLevelLow;
  if (isSwitchInput(input)) return runtime_template.buttons[input].no_pullup ? kInputOnLevelHigh : kInputOnLevelLow;
  return runtime_template.buttons[input].inverted ? kInputOnLevelHigh : kInputOnLevelLow;
}

uint8_t effectiveInputOnLevel(uint8_t input) {
  if (input >= kMaxButtons) return kInputOnLevelLow;
  if (config.input_on_level[input] == kInputOnLevelLow || config.input_on_level[input] == kInputOnLevelHigh) {
    return config.input_on_level[input];
  }
  return defaultInputOnLevel(input);
}

bool readInputActive(uint8_t input) {
  if (input >= kMaxButtons || !digitalPinSupported(runtime_template.buttons[input].pin)) return false;
  const bool high = digitalRead(runtime_template.buttons[input].pin) == HIGH;
  return high == (effectiveInputOnLevel(input) == kInputOnLevelHigh);
}

bool relayAvailable(uint8_t relay) {
  return relay < runtime_template.relay_count && hasPin(runtime_template.relays[relay]);
}

bool defaultInputRelayTarget(uint8_t input, uint8_t &relay) {
  if (input < kMaxButtons) {
    const uint8_t preferred = runtime_template.input_default_relay[input];
    if (preferred < runtime_template.relay_count && hasPin(runtime_template.relays[preferred])) {
      relay = preferred;
      return true;
    }
  }
  if (input < runtime_template.relay_count && hasPin(runtime_template.relays[input])) {
    relay = input;
    return true;
  }
  for (uint8_t i = 0; i < runtime_template.relay_count; i++) {
    if (hasPin(runtime_template.relays[i])) {
      relay = i;
      return true;
    }
  }
  return false;
}

bool inputRelayTarget(uint8_t input, uint8_t &relay) {
  if (input >= kMaxButtons) return false;
  const uint8_t configured = config.input_relay[input];
  if (configured < runtime_template.relay_count && hasPin(runtime_template.relays[configured])) {
    relay = configured;
    return true;
  }
  return defaultInputRelayTarget(input, relay);
}

const PinAssignment *ledOutputAssignment(uint8_t led) {
  if (led < kMaxLeds) return &runtime_template.leds[led];
  if (led == kMaxLeds) return &runtime_template.link_led;
  return nullptr;
}

bool hasLedOutput(uint8_t led) {
  const PinAssignment *assignment = ledOutputAssignment(led);
  if (!assignment || !hasPin(*assignment)) return false;
  return led >= kMaxLeds || led < runtime_template.led_count;
}

String ledOutputName(uint8_t led) {
  if (led < kMaxLeds) return String(F("LED ")) + String(led + 1);
  return F("Link LED");
}

bool isLedAttachmentEncoding(uint8_t value) {
  if (value == kLedAttachNone) return true;
  if (value >= kLedAttachRelayBase && value < kLedAttachRelayBase + kMaxRelays) return true;
  if (value >= kLedAttachButtonBase && value < kLedAttachButtonBase + kMaxButtons) return true;
  return false;
}

bool isButtonActionEncoding(uint8_t value) {
  return value == kButtonActionNone ||
         value == kButtonActionRelayToggle ||
         value == kButtonActionMqtt ||
         value == kButtonActionWebhook;
}

void setDefaultButtonActionText(StoredConfig &target, uint8_t button) {
  if (button >= kMaxButtons) return;
  strlcpy(target.button_press_target[button], kDefaultButtonMqttTopic, sizeof(target.button_press_target[button]));
  strlcpy(target.button_press_payload[button], kDefaultButtonMqttPressPayload, sizeof(target.button_press_payload[button]));
  strlcpy(target.button_hold_target[button], kDefaultButtonMqttTopic, sizeof(target.button_hold_target[button]));
  strlcpy(target.button_hold_payload[button], kDefaultButtonMqttHoldPayload, sizeof(target.button_hold_payload[button]));
}

bool ledAttachmentRelayIndex(uint8_t value, uint8_t &index) {
  if (value < kLedAttachRelayBase || value >= kLedAttachRelayBase + kMaxRelays) return false;
  index = value - kLedAttachRelayBase;
  return true;
}

bool ledAttachmentButtonIndex(uint8_t value, uint8_t &index) {
  if (value < kLedAttachButtonBase || value >= kLedAttachButtonBase + kMaxButtons) return false;
  index = value - kLedAttachButtonBase;
  return true;
}

bool ledAttachmentAvailable(uint8_t value) {
  uint8_t index = 0;
  if (value == kLedAttachNone) return true;
  if (ledAttachmentRelayIndex(value, index)) {
    return index < runtime_template.relay_count && hasPin(runtime_template.relays[index]);
  }
  if (ledAttachmentButtonIndex(value, index)) {
    return index < runtime_template.button_count && hasPin(runtime_template.buttons[index]);
  }
  return false;
}

uint8_t defaultLedAttachment(uint8_t led) {
  if (led < runtime_template.relay_count && hasPin(runtime_template.relays[led])) {
    return kLedAttachRelayBase + led;
  }
  return kLedAttachNone;
}

bool ledOutputOn(uint8_t led) {
  if (!hasLedOutput(led) || led >= kMaxLedOutputs) return false;
  const uint8_t attachment = config.led_attach[led];
  uint8_t index = 0;
  if (ledAttachmentRelayIndex(attachment, index)) {
    return index < runtime_template.relay_count && hasPin(runtime_template.relays[index]) && relay_state[index];
  }
  if (ledAttachmentButtonIndex(attachment, index)) {
    return index < runtime_template.button_count && hasPin(runtime_template.buttons[index]) && button_state[index].stable_pressed;
  }
  return false;
}

bool hasConfigurableLedOutputs() {
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    if (hasLedOutput(i)) return true;
  }
  return false;
}

bool hasConfigurableRelays() {
  for (uint8_t i = 0; i < runtime_template.relay_count; i++) {
    if (relayAvailable(i)) return true;
  }
  return false;
}

bool hasConfigurableButtons() {
  for (uint8_t i = 0; i < runtime_template.button_count; i++) {
    if (hasPin(runtime_template.buttons[i])) return true;
  }
  return false;
}

void addUnsupportedTemplatePin(RuntimeTemplate &target, uint8_t pin, uint16_t code) {
  const uint8_t cap = sizeof(target.unsupported_code) / sizeof(target.unsupported_code[0]);
  if (target.unsupported_count >= cap) return;
  const uint8_t index = target.unsupported_count++;
  target.unsupported_pin[index] = pin;
  target.unsupported_code[index] = code;
}

uint8_t templateIndexToPin(uint8_t index) {
#if CONFIG_IDF_TARGET_ESP32C3
  return index <= kEspMaxGpio ? index : kInvalidPin;
#else
  return index < kTemplateGpioCount ? kEsp32TemplateToPhy[index] : kInvalidPin;
#endif
}

uint8_t templateInputSlot(RuntimeTemplate &target, uint8_t function_index, uint8_t kind) {
  if (function_index >= kMaxButtons) return kMaxButtons;
  if (!hasPin(target.buttons[function_index]) ||
      (target.input_kind[function_index] == kind &&
       target.input_function_index[function_index] == function_index)) {
    return function_index;
  }
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    if (!hasPin(target.buttons[i])) return i;
  }
  return kMaxButtons;
}

bool assignTemplateInput(RuntimeTemplate &target, uint8_t function_index, uint8_t kind,
                         const PinAssignment &assignment) {
  const uint8_t slot = templateInputSlot(target, function_index, kind);
  if (slot >= kMaxButtons) return false;
  target.buttons[slot] = assignment;
  target.input_kind[slot] = kind;
  target.input_function_index[slot] = function_index;
  target.input_default_relay[slot] = function_index < kMaxRelays ? function_index : kButtonRelayUnset;
  if (target.button_count <= slot) target.button_count = slot + 1;
  return true;
}

void parseTemplateFunction(RuntimeTemplate &target, uint8_t pin, uint16_t code) {
  if (code == kTplNone || code == kTplUser || code == kTplSentinelEnd) return;

  const uint16_t base = code & 0xffe0U;
  const uint8_t index = code & 0x1fU;

  if (code == kTplNrgSel || code == kTplNrgSelInv) {
    if (digitalPinSupported(pin)) {
      target.energy_sel_pin = pin;
      target.energy_sel_inverted = code == kTplNrgSelInv;
    }
    return;
  }
  if (code == kTplNrgCf1) {
    if (digitalPinSupported(pin)) target.energy_cf1_pin = pin;
    return;
  }
  if (code == kTplHlwCf || code == kTplHjlCf) {
    if (digitalPinSupported(pin)) {
      target.energy_cf_pin = pin;
      target.energy_hjl = code == kTplHjlCf;
    }
    return;
  }
  if (base == kTplAdcTemp) {
    if (digitalPinSupported(pin)) target.adc_temp = true;
    return;
  }
  if (base == kTplAdcInput) {
    return;
  }
  if (code == kTplTxd) {
    if (digitalPinSupported(pin)) target.energy_tx_pin = pin;
    return;
  }
  if (code == kTplBl0939Rx) {
    if (digitalPinSupported(pin)) target.energy_bl0939_rx_pin = pin;
    return;
  }
#if MYMOTA32_LIGHT_SUPPORTED
  if (code == kTplSm2335Clk) {
    if (digitalPinSupported(pin)) target.sm2335_clk_pin = pin;
    return;
  }
  if (base == kTplSm2335Dat) {
    if (digitalPinSupported(pin)) {
      target.sm2335_dat_pin = pin;
      target.sm2335_current = (index << 4) | index;
    }
    return;
  }
#endif
  if (base == kTplAde7953Irq) {
    return;
  }
  if (base == kTplOptionA && index < kTplOptionACount) {
    return;
  }

  if (!digitalPinSupported(pin)) {
    addUnsupportedTemplatePin(target, pin, code);
    return;
  }

  if (base == kTplKey1 || base == kTplKey1Np || base == kTplKey1Inv || base == kTplKey1InvNp) {
    if (index >= kMaxButtons) {
      addUnsupportedTemplatePin(target, pin, code);
      return;
    }
    const PinAssignment assignment = {
      pin,
      base == kTplKey1Inv || base == kTplKey1InvNp,
      base == kTplKey1Np || base == kTplKey1InvNp
    };
    if (!assignTemplateInput(target, index, kInputKindButton, assignment)) {
      addUnsupportedTemplatePin(target, pin, code);
    }
    return;
  }

  if (base == kTplSwt1 || base == kTplSwt1Np) {
    if (index >= kMaxButtons) {
      addUnsupportedTemplatePin(target, pin, code);
      return;
    }
    const PinAssignment assignment = {pin, false, base == kTplSwt1Np};
    if (!assignTemplateInput(target, index, kInputKindSwitch, assignment)) {
      addUnsupportedTemplatePin(target, pin, code);
    }
    return;
  }

  if (base == kTplRel1 || base == kTplRel1Inv) {
    if (index >= kMaxRelays) {
      addUnsupportedTemplatePin(target, pin, code);
      return;
    }
    target.relays[index] = {pin, base == kTplRel1Inv, false};
    if (target.relay_count <= index) target.relay_count = index + 1;
    return;
  }

  if (base == kTplLed1 || base == kTplLed1Inv) {
    if (index >= kMaxLeds) {
      addUnsupportedTemplatePin(target, pin, code);
      return;
    }
    target.leds[index] = {pin, base == kTplLed1Inv, false};
    if (target.led_count <= index) target.led_count = index + 1;
    return;
  }

  if (code == kTplLedLnk || code == kTplLedLnkInv) {
    target.link_led = {pin, code == kTplLedLnkInv, false};
    return;
  }

  if (code == kTplI2cScl) { target.i2c_scl_pin = pin; return; }
  if (code == kTplI2cSda) { target.i2c_sda_pin = pin; return; }

  addUnsupportedTemplatePin(target, pin, code);
}

void resetRuntimeTemplate(RuntimeTemplate &target) {
  memset(&target, 0, sizeof(target));
  for (uint8_t i = 0; i < kMaxRelays; i++) resetPinAssignment(target.relays[i]);
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    resetPinAssignment(target.buttons[i]);
    target.input_function_index[i] = kButtonRelayUnset;
    target.input_default_relay[i] = kButtonRelayUnset;
  }
  for (uint8_t i = 0; i < kMaxLeds; i++) resetPinAssignment(target.leds[i]);
  resetPinAssignment(target.link_led);
  target.i2c_scl_pin = kInvalidPin;
  target.i2c_sda_pin = kInvalidPin;
  target.energy_cf_pin = kInvalidPin;
  target.energy_cf1_pin = kInvalidPin;
  target.energy_sel_pin = kInvalidPin;
  target.energy_tx_pin = kInvalidPin;
  target.energy_bl0939_rx_pin = kInvalidPin;
  target.sm2335_clk_pin = kInvalidPin;
  target.sm2335_dat_pin = kInvalidPin;
  target.sm2335_current = 0x88;
}

void decodeTemplateConfigInto(const StoredConfig &source, RuntimeTemplate &target) {
  resetRuntimeTemplate(target);
  if (!source.template_enabled) return;
  target.enabled = true;
  strlcpy(target.name, source.template_name, sizeof(target.name));
  target.base = source.template_base;
  target.flag = source.template_flag;
  for (uint8_t i = 0; i < kTemplateGpioCount; i++) {
    parseTemplateFunction(target, templateIndexToPin(i), source.template_gpio[i]);
  }
#if MYMOTA32_LIGHT_SUPPORTED
  target.sm2335 = digitalPinSupported(target.sm2335_clk_pin) &&
                  digitalPinSupported(target.sm2335_dat_pin);
#else
  target.sm2335 = false;
#endif
}

void decodeTemplateConfig() {
  decodeTemplateConfigInto(config, runtime_template);
}

bool cjsonIsType(const cJSON *value, int type) {
  return value && ((value->type & 0xff) == type);
}

const cJSON *cjsonObjectItem(const cJSON *object, const char *key) {
  if (!cjsonIsType(object, cJSON_Object)) return nullptr;
  for (const cJSON *item = object->child; item; item = item->next) {
    if (item->string && strcmp(item->string, key) == 0) return item;
  }
  return nullptr;
}

uint8_t cjsonArraySize(const cJSON *array) {
  if (!cjsonIsType(array, cJSON_Array)) return 0;
  uint8_t count = 0;
  for (const cJSON *item = array->child; item && count < UINT8_MAX; item = item->next) count++;
  return count;
}

const cJSON *cjsonArrayItem(const cJSON *array, uint8_t index) {
  const cJSON *item = cjsonIsType(array, cJSON_Array) ? array->child : nullptr;
  while (item && index--) item = item->next;
  return item;
}

bool cjsonUintInRange(const cJSON *value, uint32_t max_value, uint32_t &out) {
  if (!cjsonIsType(value, cJSON_Number)) return false;
  const double number = value->valuedouble;
  if (number < 0 || number > static_cast<double>(max_value)) return false;
  const uint32_t integer = static_cast<uint32_t>(number);
  if (static_cast<double>(integer) != number) return false;
  out = integer;
  return true;
}

bool parseTemplateJson(const String &json, StoredConfig &target, String &error) {
  if (json.length() < 9 || json.length() > kTemplateJsonMaxLen) {
    error = F("Template JSON length is invalid");
    return false;
  }

  cJSON *doc = cJSON_ParseWithLengthOpts(json.c_str(), json.length() + 1, nullptr, true);
  if (!doc) {
    error = F("Template JSON parse failed");
    return false;
  }
  if (!cjsonIsType(doc, cJSON_Object)) {
    cJSON_Delete(doc);
    error = F("Template must be a JSON object");
    return false;
  }

  const cJSON *name_value = cjsonObjectItem(doc, "NAME");
  const char *name = (cjsonIsType(name_value, cJSON_String) && name_value->valuestring) ? name_value->valuestring : "";
  if (name[0] == '\0') {
    cJSON_Delete(doc);
    error = F("Template NAME is empty");
    return false;
  }
  if (strlen(name) >= sizeof(target.template_name)) {
    cJSON_Delete(doc);
    error = F("Template NAME is too long");
    return false;
  }

  const cJSON *gpio_values = cjsonObjectItem(doc, "GPIO");
  const uint8_t gpio_count = cjsonArraySize(gpio_values);
  if (!cjsonIsType(gpio_values, cJSON_Array) ||
      (gpio_count != kTemplateGpioCount && gpio_count != kTemplateJsonMinGpioCount)) {
    cJSON_Delete(doc);
    error = F("GPIO entry count is invalid for this target");
    return false;
  }

  uint16_t gpio[kTemplateGpioCount]{};
  for (uint8_t i = 0; i < gpio_count; i++) {
    uint32_t parsed = 0;
    if (!cjsonUintInRange(cjsonArrayItem(gpio_values, i), UINT16_MAX, parsed)) {
      cJSON_Delete(doc);
      error = F("Invalid GPIO value");
      return false;
    }
    gpio[i] = static_cast<uint16_t>(parsed);
  }

  uint32_t base = 0;
  if (!cjsonUintInRange(cjsonObjectItem(doc, "BASE"), UINT16_MAX, base) || base == 0) {
    cJSON_Delete(doc);
    error = F("Template BASE is invalid");
    return false;
  }
  uint32_t flag = 0;
  const cJSON *flag_value = cjsonObjectItem(doc, "FLAG");
  if (flag_value && !cjsonUintInRange(flag_value, UINT32_MAX, flag)) {
    cJSON_Delete(doc);
    error = F("Template FLAG is invalid");
    return false;
  }

  target.template_enabled = 1;
  target.template_base = static_cast<uint16_t>(base);
  target.template_flag = flag;
  strlcpy(target.template_name, name, sizeof(target.template_name));
  memcpy(target.template_gpio, gpio, sizeof(target.template_gpio));
  cJSON_Delete(doc);
  return true;
}

String currentTemplateJson() {
  if (!config.template_enabled) return String();
  String out;
  out.reserve(kTemplateJsonMaxLen);
  out += F("{\"NAME\":\"");
  out += jsonEscape(config.template_name);
  out += F("\",\"GPIO\":[");
  for (uint8_t i = 0; i < kTemplateGpioCount; i++) {
    if (i) out += ',';
    out += String(config.template_gpio[i]);
  }
  out += F("],\"FLAG\":");
  out += String(config.template_flag);
  out += F(",\"BASE\":");
  out += String(config.template_base);
  out += F("}");
  return out;
}

void scheduleRestart(uint32_t delay_ms, bool preserve_relays = false) {
  restart_due_ms = millis() + delay_ms;
  restart_scheduled_ms = millis();
  restart_preserve_relays = preserve_relays;
}

bool restartDue() {
  return restart_due_ms != 0 && (int32_t)(millis() - restart_due_ms) >= 0;
}

void clearTemplateConfig(StoredConfig &target) {
  target.template_enabled = 0;
  target.template_base = 0;
  target.template_flag = 0;
  memset(target.template_name, 0, sizeof(target.template_name));
  memset(target.template_gpio, 0, sizeof(target.template_gpio));
}

String defaultMqttTopic() {
  return "tasmota_" + chipIdHex();
}

uint8_t sanitizeLightDimmerValue(uint16_t value) {
  if (value < kLightDimmerMin) return kLightDimmerMin;
  if (value > kLightDimmerMax) return kLightDimmerMax;
  return static_cast<uint8_t>(value);
}

uint16_t sanitizeLightCtValue(uint16_t value) {
  if (value < kLightCtMin) return kLightCtMin;
  if (value > kLightCtMax) return kLightCtMax;
  return value;
}

uint8_t sanitizeLightSpeedValue(uint16_t value) {
  if (value < kLightSpeedMin || value > kLightSpeedMax) return kLightSpeedDefault;
  return static_cast<uint8_t>(value);
}

bool isIBeaconFilterInterval(uint16_t value) {
  for (uint8_t i = 0; i < sizeof(kIBeaconFilterIntervals) / sizeof(kIBeaconFilterIntervals[0]); i++) {
    if (kIBeaconFilterIntervals[i] == value) return true;
  }
  return false;
}

uint16_t sanitizeIBeaconFilterInterval(uint16_t value, uint16_t fallback) {
  return isIBeaconFilterInterval(value) ? value : fallback;
}

bool isHexChar(char c) {
  return (c >= '0' && c <= '9') ||
         (c >= 'a' && c <= 'f') ||
         (c >= 'A' && c <= 'F');
}

char uppercaseHexChar(char c) {
  if (c >= 'a' && c <= 'f') return static_cast<char>(c - 'a' + 'A');
  return c;
}

bool appendNormalizedIBeaconMacToken(const char *token, size_t token_len, char *out, size_t out_size, size_t &out_len) {
  char hex[12]{};
  uint8_t count = 0;
  for (size_t i = 0; i < token_len; i++) {
    const char c = token[i];
    if (c == ':' || c == '-' || c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
    if (!isHexChar(c) || count >= 12) {
      return false;
    }
    hex[count++] = uppercaseHexChar(c);
  }
  if (count == 0) return true;
  if (count != 12) {
    return false;
  }
  const size_t needed = out_len + (out_len ? 1 : 0) + 12 + 1;
  if (needed > out_size) {
    return false;
  }
  if (out_len) out[out_len++] = ',';
  memcpy(out + out_len, hex, sizeof(hex));
  out_len += sizeof(hex);
  out[out_len] = '\0';
  return true;
}

bool normalizeIBeaconMacList(const String &input, char *out, size_t out_size) {
  if (out_size == 0) return false;
  out[0] = '\0';
  size_t out_len = 0;
  size_t start = 0;
  const size_t input_len = input.length();
  const char *raw = input.c_str();
  for (size_t i = 0; i <= input_len; i++) {
    if (i == input_len || raw[i] == ',') {
      if (!appendNormalizedIBeaconMacToken(raw + start, i - start, out, out_size, out_len)) {
        return false;
      }
      start = i + 1;
    }
  }
  return true;
}

bool clearGracefulRelaySnapshot();
bool clearLastRelaySnapshot();
void loadLastRelaySnapshot();
bool saveLastRelaySnapshotIfNeeded();
void refreshRelayEnforcementRuntime(bool schedule_off_relays);
void refreshRelayPulseRuntime(bool schedule_on_relays);
void setupLightRuntime();
void maintainLight();
void scheduleMqttLightPublish(uint8_t mask);
bool persistLightConfig(bool force = false);

void setDefaultConfig() {
  memset(&config, 0, sizeof(config));
  strlcpy(config.hostname, defaultHostname().c_str(), sizeof(config.hostname));
  config.phy_mode = kPhyModeAuto;
  config.button_hold_ms = kButtonHoldDefaultMs;
  config.button_debounce_ms = kButtonDebounceDefaultMs;
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    config.input_mode[i] = kInputModeUnset;
    config.input_relay[i] = kButtonRelayUnset;
    config.input_on_level[i] = kInputOnLevelUnset;
    config.button_press_action[i] = kButtonActionRelayToggle;
    config.button_hold_action[i] = kButtonActionNone;
    config.button_press_relay[i] = kButtonRelayUnset;
    config.button_hold_relay[i] = kButtonRelayUnset;
    strlcpy(config.button_press_target[i], kDefaultButtonMqttTopic, sizeof(config.button_press_target[i]));
    strlcpy(config.button_press_payload[i], kDefaultButtonMqttPressPayload, sizeof(config.button_press_payload[i]));
    strlcpy(config.button_hold_target[i], kDefaultButtonMqttTopic, sizeof(config.button_hold_target[i]));
    strlcpy(config.button_hold_payload[i], kDefaultButtonMqttHoldPayload, sizeof(config.button_hold_payload[i]));
  }
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    config.led_attach[i] = kLedAttachNone;
  }
  config.mqtt_port = kMqttDefaultPort;
  strlcpy(config.mqtt_topic, defaultMqttTopic().c_str(), sizeof(config.mqtt_topic));
  config.mqtt_keepalive = 0;
  config.energy_total_offset_kwh = 0.0f;
  config.energy_mqtt_interval = 0;
  config.energy_mqtt_change_percent_x10 = 0;
  config.energy_mqtt_change_watts = 0;
  memset(config.relay_restore_boot, 1, sizeof(config.relay_restore_boot));
  memset(config.relay_on_boot, 0, sizeof(config.relay_on_boot));
  memset(config.relay_time_enabled, 0, sizeof(config.relay_time_enabled));
  memset(config.relay_time_seconds, 0, sizeof(config.relay_time_seconds));
  memset(config.relay_pulse_enabled, 0, sizeof(config.relay_pulse_enabled));
  memset(config.relay_pulse_seconds, 0, sizeof(config.relay_pulse_seconds));
  config.light_power = 0;
  config.light_dimmer = kLightDimmerOff;
  config.light_ct = kLightCtDefault;
  config.light_mode = kLightModeWhite;
  memset(config.light_rgb, 0, sizeof(config.light_rgb));
  config.light_on_dimmer = kLightPowerOnDimmerDefault;
  config.light_fade = kLightFadeDefault;
  config.light_speed = kLightSpeedDefault;
  config.light_restore_boot = 1;
  config.ibeacon_enabled = 0;
  config.ibeacon_filter1_interval_sec = kIBeaconFilter1DefaultSec;
  config.ibeacon_filter2_interval_sec = kIBeaconFilter2DefaultSec;
  config.ibeacon_filter1_macs[0] = '\0';
  config.ibeacon_filter2_macs[0] = '\0';
  config.power_saving_mode = kPowerSavingOff;
  config.wifi_dynamic_power = kWifiDynamicPowerDefault;
}

template <size_t N>
void readByteArray(Preferences &p, const char *key, uint8_t (&out)[N], uint8_t fill) {
  uint8_t buf[N];
  memset(buf, fill, sizeof(buf));
  size_t got = p.getBytesLength(key);
  if (got == sizeof(buf)) {
    p.getBytes(key, buf, sizeof(buf));
  }
  memcpy(out, buf, sizeof(out));
}

void readGpioArray(Preferences &p, const char *key, uint16_t (&out)[kTemplateGpioCount]) {
  uint16_t buf[kTemplateGpioCount];
  memset(buf, 0, sizeof(buf));
  size_t got = p.getBytesLength(key);
  if (got == sizeof(buf)) {
    p.getBytes(key, buf, sizeof(buf));
  }
  memcpy(out, buf, sizeof(out));
}

template <size_t N>
void readUShortArray(Preferences &p, const char *key, uint16_t (&out)[N], uint16_t fill) {
  uint16_t buf[N];
  for (uint8_t i = 0; i < N; i++) buf[i] = fill;
  size_t got = p.getBytesLength(key);
  if (got == sizeof(buf)) {
    p.getBytes(key, buf, sizeof(buf));
  }
  memcpy(out, buf, sizeof(out));
}

bool loadConfig() {
  setDefaultConfig();
  if (!prefs.begin("mymota32", true)) {
    config_ok = false;
    return false;
  }
  String ssid = prefs.getString("ssid", "");
  String password = prefs.getString("password", "");
  String hostname = prefs.getString("hostname", "");
  uint8_t phy = prefs.getUChar("phy", kPhyModeAuto);

  uint8_t tpl_en = prefs.getUChar("tpl_en", 0);
  uint16_t tpl_base = prefs.getUShort("tpl_base", 0);
  uint32_t tpl_flag = prefs.getUInt("tpl_flag", 0);
  String tpl_name = prefs.getString("tpl_name", "");
  uint16_t tpl_gpio[kTemplateGpioCount];
  readGpioArray(prefs, "tpl_gpio", tpl_gpio);

  uint8_t in_mode[kMaxButtons];
  uint8_t in_relay[kMaxButtons];
  uint8_t in_level[kMaxButtons];
  readByteArray(prefs, "in_mode", in_mode, kInputModeUnset);
  readByteArray(prefs, "in_relay", in_relay, kButtonRelayUnset);
  readByteArray(prefs, "in_level", in_level, kInputOnLevelUnset);

  uint8_t bp_act[kMaxButtons];
  uint8_t bh_act[kMaxButtons];
  uint8_t bp_rel[kMaxButtons];
  uint8_t bh_rel[kMaxButtons];
  readByteArray(prefs, "bp_act", bp_act, kButtonActionRelayToggle);
  readByteArray(prefs, "bh_act", bh_act, kButtonActionNone);
  readByteArray(prefs, "bp_rel", bp_rel, kButtonRelayUnset);
  readByteArray(prefs, "bh_rel", bh_rel, kButtonRelayUnset);

  char bp_tgt_buf[kMaxButtons][kButtonActionTargetMaxLen + 1];
  char bp_pld_buf[kMaxButtons][kButtonActionPayloadMaxLen + 1];
  char bh_tgt_buf[kMaxButtons][kButtonActionTargetMaxLen + 1];
  char bh_pld_buf[kMaxButtons][kButtonActionPayloadMaxLen + 1];
  bool bp_tgt_have = prefs.getBytesLength("bp_tgt") == sizeof(bp_tgt_buf);
  bool bp_pld_have = prefs.getBytesLength("bp_pld") == sizeof(bp_pld_buf);
  bool bh_tgt_have = prefs.getBytesLength("bh_tgt") == sizeof(bh_tgt_buf);
  bool bh_pld_have = prefs.getBytesLength("bh_pld") == sizeof(bh_pld_buf);
  if (bp_tgt_have) prefs.getBytes("bp_tgt", bp_tgt_buf, sizeof(bp_tgt_buf));
  if (bp_pld_have) prefs.getBytes("bp_pld", bp_pld_buf, sizeof(bp_pld_buf));
  if (bh_tgt_have) prefs.getBytes("bh_tgt", bh_tgt_buf, sizeof(bh_tgt_buf));
  if (bh_pld_have) prefs.getBytes("bh_pld", bh_pld_buf, sizeof(bh_pld_buf));

  uint16_t btn_hold = prefs.getUShort("btn_hold", kButtonHoldDefaultMs);
  uint16_t btn_db = prefs.getUShort("btn_db", kButtonDebounceDefaultMs);

  uint8_t leds[kMaxLedOutputs];
  readByteArray(prefs, "leds", leds, kLedAttachNone);

  String mqtt_host = prefs.getString("mqtt_host", "");
  uint16_t mqtt_port = prefs.getUShort("mqtt_port", kMqttDefaultPort);
  String mqtt_topic = prefs.getString("mqtt_topic", "");
  uint16_t mqtt_keepalive = prefs.getUShort("mqtt_keep", 0);
  float energy_total_offset_kwh = prefs.getFloat("en_offset", 0.0f);
  uint16_t energy_mqtt_interval = prefs.getUShort("en_int", 0);
  uint16_t energy_mqtt_change_percent_x10 = prefs.getUShort("en_pct", 0);
  uint16_t energy_mqtt_change_watts = prefs.getUShort("en_watts", 0);
  energy_saved_ukwh = prefs.getULong64("en_total", 0);

  uint8_t relay_restore_boot[kMaxRelays];
  uint8_t relay_on_boot[kMaxRelays];
  uint8_t relay_time_enabled[kMaxRelays];
  uint16_t relay_time_seconds[kMaxRelays];
  uint8_t relay_pulse_enabled[kMaxRelays];
  uint16_t relay_pulse_seconds[kMaxRelays];
  readByteArray(prefs, "rel_restore", relay_restore_boot, 1);
  readByteArray(prefs, "rel_on_boot", relay_on_boot, 0);
  readByteArray(prefs, "rel_time_en", relay_time_enabled, 0);
  readUShortArray(prefs, "rel_time_s", relay_time_seconds, 0);
  readByteArray(prefs, "rel_pulse_en", relay_pulse_enabled, 0);
  readUShortArray(prefs, "rel_pulse_s", relay_pulse_seconds, 0);

  uint8_t light_power = prefs.getUChar("lt_power", 0);
  uint8_t light_dimmer = prefs.getUChar("lt_dim", kLightDimmerOff);
  uint16_t light_ct = prefs.getUShort("lt_ct", kLightCtDefault);
  uint8_t light_mode = prefs.getUChar("lt_mode", kLightModeWhite);
  uint8_t light_rgb[3];
  readByteArray(prefs, "lt_rgb", light_rgb, 0);
  uint8_t light_on_dimmer = prefs.getUChar("lt_on_dim", kLightPowerOnDimmerDefault);
  uint8_t light_fade = prefs.getUChar("lt_fade", kLightFadeDefault);
  uint8_t light_speed = prefs.getUChar("lt_speed", kLightSpeedDefault);
  uint8_t light_restore_boot = prefs.getUChar("lt_restore", 1);

  uint8_t ibeacon_enabled = prefs.getUChar("ibeacon", 0);
  uint16_t ibeacon_filter1_interval = prefs.getUShort("ib_f1_int", kIBeaconFilter1DefaultSec);
  uint16_t ibeacon_filter2_interval = prefs.getUShort("ib_f2_int", kIBeaconFilter2DefaultSec);
  String ibeacon_filter1_macs = prefs.getString("ib_f1_mac", "");
  String ibeacon_filter2_macs = prefs.getString("ib_f2_mac", "");
  uint16_t power_saving_mode = prefs.getUShort("pwr_save", kPowerSavingOff);
  uint8_t wifi_dynamic_power = prefs.getUChar("wifi_dyn", kWifiDynamicPowerDefault);
  prefs.end();

  strlcpy(config.ssid, ssid.c_str(), sizeof(config.ssid));
  strlcpy(config.password, password.c_str(), sizeof(config.password));
  if (hostname.length() > 0) {
    strlcpy(config.hostname, hostname.c_str(), sizeof(config.hostname));
  }
  config.phy_mode = sanitizePhyMode(phy);

  config.template_enabled = tpl_en ? 1 : 0;
  config.template_base = tpl_base;
  config.template_flag = tpl_flag;
  strlcpy(config.template_name, tpl_name.c_str(), sizeof(config.template_name));
  memcpy(config.template_gpio, tpl_gpio, sizeof(config.template_gpio));

  memcpy(config.input_mode, in_mode, sizeof(config.input_mode));
  memcpy(config.input_relay, in_relay, sizeof(config.input_relay));
  memcpy(config.input_on_level, in_level, sizeof(config.input_on_level));

  for (uint8_t i = 0; i < kMaxButtons; i++) {
    config.button_press_action[i] = isButtonActionEncoding(bp_act[i]) ? bp_act[i] : kButtonActionRelayToggle;
    config.button_hold_action[i] = isButtonActionEncoding(bh_act[i]) ? bh_act[i] : kButtonActionNone;
    config.button_press_relay[i] = bp_rel[i];
    config.button_hold_relay[i] = bh_rel[i];
    if (bp_tgt_have) {
      bp_tgt_buf[i][kButtonActionTargetMaxLen] = '\0';
      strlcpy(config.button_press_target[i], bp_tgt_buf[i], sizeof(config.button_press_target[i]));
    }
    if (bp_pld_have) {
      bp_pld_buf[i][kButtonActionPayloadMaxLen] = '\0';
      strlcpy(config.button_press_payload[i], bp_pld_buf[i], sizeof(config.button_press_payload[i]));
    }
    if (bh_tgt_have) {
      bh_tgt_buf[i][kButtonActionTargetMaxLen] = '\0';
      strlcpy(config.button_hold_target[i], bh_tgt_buf[i], sizeof(config.button_hold_target[i]));
    }
    if (bh_pld_have) {
      bh_pld_buf[i][kButtonActionPayloadMaxLen] = '\0';
      strlcpy(config.button_hold_payload[i], bh_pld_buf[i], sizeof(config.button_hold_payload[i]));
    }
  }

  if (btn_hold < kButtonHoldMinMs || btn_hold > kButtonHoldMaxMs) btn_hold = kButtonHoldDefaultMs;
  if (btn_db < kButtonDebounceMinMs || btn_db > kButtonDebounceMaxMs) btn_db = kButtonDebounceDefaultMs;
  config.button_hold_ms = btn_hold;
  config.button_debounce_ms = btn_db;

  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    config.led_attach[i] = isLedAttachmentEncoding(leds[i]) ? leds[i] : kLedAttachNone;
  }

  strlcpy(config.mqtt_host, mqtt_host.c_str(), sizeof(config.mqtt_host));
  config.mqtt_port = mqtt_port == 0 ? kMqttDefaultPort : mqtt_port;
  if (mqtt_topic.length() == 0) {
    strlcpy(config.mqtt_topic, defaultMqttTopic().c_str(), sizeof(config.mqtt_topic));
  } else {
    strlcpy(config.mqtt_topic, mqtt_topic.c_str(), sizeof(config.mqtt_topic));
  }
  config.mqtt_keepalive = mqtt_keepalive;
  if (isnan(energy_total_offset_kwh) ||
      energy_total_offset_kwh < kEnergyTotalOffsetMinKwh ||
      energy_total_offset_kwh > kEnergyTotalOffsetMaxKwh) {
    energy_total_offset_kwh = 0.0f;
  }
  config.energy_total_offset_kwh = energy_total_offset_kwh;
  config.energy_mqtt_interval = energy_mqtt_interval;
  config.energy_mqtt_change_percent_x10 =
    energy_mqtt_change_percent_x10 > static_cast<uint16_t>(kMqttEnergyChangeMaxPercent * 10.0f) ? 0 : energy_mqtt_change_percent_x10;
  config.energy_mqtt_change_watts = energy_mqtt_change_watts;
  if (energy_saved_ukwh > kEnergyTotalMaxUkwh) energy_saved_ukwh = 0;
  for (uint8_t i = 0; i < kMaxRelays; i++) {
    config.relay_restore_boot[i] = relay_restore_boot[i] ? 1 : 0;
    config.relay_on_boot[i] = relay_on_boot[i] ? 1 : 0;
    if (config.relay_restore_boot[i]) config.relay_on_boot[i] = 0;
    config.relay_time_enabled[i] = relay_time_enabled[i] ? 1 : 0;
    config.relay_time_seconds[i] = relay_time_seconds[i];
    if (config.relay_time_enabled[i] &&
        (config.relay_time_seconds[i] < kRelayEnforcementMinSeconds ||
         config.relay_time_seconds[i] > kRelayEnforcementMaxSeconds)) {
      config.relay_time_enabled[i] = 0;
      config.relay_time_seconds[i] = 0;
    }
    config.relay_pulse_enabled[i] = relay_pulse_enabled[i] ? 1 : 0;
    config.relay_pulse_seconds[i] = relay_pulse_seconds[i];
    if (config.relay_pulse_enabled[i] &&
        config.relay_pulse_seconds[i] < kRelayPulseMinSeconds) {
      config.relay_pulse_enabled[i] = 0;
      config.relay_pulse_seconds[i] = 0;
    }
  }
  config.light_power = light_power ? 1 : 0;
  config.light_on_dimmer = sanitizeLightDimmerValue(light_on_dimmer);
  config.light_ct = sanitizeLightCtValue(light_ct);
  config.light_mode = light_mode == kLightModeRgb ? kLightModeRgb : kLightModeWhite;
  memcpy(config.light_rgb, light_rgb, sizeof(config.light_rgb));
  config.light_fade = light_fade ? 1 : 0;
  config.light_speed = sanitizeLightSpeedValue(light_speed);
  config.light_restore_boot = light_restore_boot ? 1 : 0;
  if (!config.light_power) {
    config.light_dimmer = kLightDimmerOff;
  } else if (light_dimmer < kLightDimmerMin) {
    config.light_dimmer = config.light_on_dimmer;
  } else {
    config.light_dimmer = sanitizeLightDimmerValue(light_dimmer);
  }
  config.ibeacon_enabled = ibeacon_enabled ? 1 : 0;
  config.ibeacon_filter1_interval_sec = sanitizeIBeaconFilterInterval(ibeacon_filter1_interval, kIBeaconFilter1DefaultSec);
  config.ibeacon_filter2_interval_sec = sanitizeIBeaconFilterInterval(ibeacon_filter2_interval, kIBeaconFilter2DefaultSec);
  if (!normalizeIBeaconMacList(ibeacon_filter1_macs, config.ibeacon_filter1_macs, sizeof(config.ibeacon_filter1_macs))) {
    config.ibeacon_filter1_macs[0] = '\0';
  }
  if (!normalizeIBeaconMacList(ibeacon_filter2_macs, config.ibeacon_filter2_macs, sizeof(config.ibeacon_filter2_macs))) {
    config.ibeacon_filter2_macs[0] = '\0';
  }
  config.power_saving_mode = sanitizePowerSavingMode(power_saving_mode);
  config.wifi_dynamic_power = wifi_dynamic_power ? 1 : 0;

  config_ok = config.ssid[0] != '\0';
  return config_ok;
}

bool saveWifiConfig(const char *ssid, const char *password, const char *hostname, uint8_t phy_mode,
                    bool dynamic_power) {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putString("ssid", ssid);
  prefs.putString("password", password);
  if (hostname && hostname[0]) prefs.putString("hostname", hostname);
  else prefs.putString("hostname", defaultHostname());
  prefs.putUChar("phy", sanitizePhyMode(phy_mode));
  prefs.putUChar("wifi_dyn", dynamic_power ? 1 : 0);
  prefs.end();
  return loadConfig();
}

bool saveTemplateConfig(const StoredConfig &source) {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putUChar("tpl_en", source.template_enabled);
  prefs.putUShort("tpl_base", source.template_base);
  prefs.putUInt("tpl_flag", source.template_flag);
  prefs.putString("tpl_name", source.template_name);
  prefs.putBytes("tpl_gpio", source.template_gpio, sizeof(source.template_gpio));
  prefs.end();
  return loadConfig();
}

bool saveLedAttachments(const uint8_t (&leds)[kMaxLedOutputs]) {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putBytes("leds", leds, sizeof(leds));
  prefs.end();
  return loadConfig();
}

void resetMqttRuntimeState() {
  next_mqtt_reconnect = 0;
  last_mqtt_io = 0;
  last_mqtt_rx = 0;
  last_mqtt_ping = 0;
  last_mqtt_state_publish = 0;
  last_mqtt_energy_publish = 0;
  last_mqtt_connect_attempt = 0;
  last_mqtt_connect_duration = 0;
  last_mqtt_connect_result = kMqttConnectIdle;
  mqtt_pending_relay_mask = 0;
  mqtt_pending_light_mask = 0;
  mqtt_pending_energy_zero_relay_mask = 0;
  mqtt_pending_energy_report_reason = kMqttEnergyReportReasonNone;
  mqtt_ping_pending = false;
  last_mqtt_energy_power = NAN;
  last_observed_energy_power = NAN;
  last_mqtt_energy_report_reason = kMqttEnergyReportReasonNone;
}

bool saveMqttConfig(const char *host, uint16_t port, const char *topic, uint16_t keepalive) {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putString("mqtt_host", host ? host : "");
  prefs.putUShort("mqtt_port", port);
  prefs.putString("mqtt_topic", topic ? topic : "");
  prefs.putUShort("mqtt_keep", keepalive);
  prefs.end();
  resetMqttRuntimeState();
  if (mqtt_client.connected()) mqtt_client.stop();
  return loadConfig();
}

bool saveEnergyConfig(float total_offset_kwh, uint16_t mqtt_interval, uint16_t mqtt_change_percent_x10,
                      uint16_t mqtt_change_watts) {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putFloat("en_offset", total_offset_kwh);
  prefs.putUShort("en_int", mqtt_interval);
  prefs.putUShort("en_pct", mqtt_change_percent_x10);
  prefs.putUShort("en_watts", mqtt_change_watts);
  prefs.end();
  last_mqtt_energy_publish = 0;
  last_mqtt_energy_power = NAN;
  last_observed_energy_power = NAN;
  last_mqtt_energy_report_reason = kMqttEnergyReportReasonNone;
  mqtt_pending_energy_zero_relay_mask = 0;
  mqtt_pending_energy_report_reason = kMqttEnergyReportReasonNone;
  return loadConfig();
}

bool saveIBeaconConfig(bool enabled, uint16_t filter1_interval, const char *filter1_macs,
                       uint16_t filter2_interval, const char *filter2_macs) {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putUChar("ibeacon", enabled ? 1 : 0);
  prefs.putUShort("ib_f1_int", sanitizeIBeaconFilterInterval(filter1_interval, kIBeaconFilter1DefaultSec));
  prefs.putString("ib_f1_mac", filter1_macs ? filter1_macs : "");
  prefs.putUShort("ib_f2_int", sanitizeIBeaconFilterInterval(filter2_interval, kIBeaconFilter2DefaultSec));
  prefs.putString("ib_f2_mac", filter2_macs ? filter2_macs : "");
  prefs.end();
  return loadConfig();
}

bool savePowerSavingConfig(uint8_t mode) {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putUShort("pwr_save", sanitizePowerSavingMode(mode));
  prefs.end();
  return loadConfig();
}

bool saveDeviceStateEnforcementConfig(const uint8_t *restore_boot, const uint8_t *on_boot,
                                      const uint8_t *time_enabled, const uint16_t *time_seconds,
                                      uint8_t light_restore_boot) {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putBytes("rel_restore", restore_boot, sizeof(config.relay_restore_boot));
  prefs.putBytes("rel_on_boot", on_boot, sizeof(config.relay_on_boot));
  prefs.putBytes("rel_time_en", time_enabled, sizeof(config.relay_time_enabled));
  prefs.putBytes("rel_time_s", time_seconds, sizeof(config.relay_time_seconds));
  prefs.putUChar("lt_restore", light_restore_boot ? 1 : 0);
  prefs.end();
  if (!loadConfig()) return false;
  refreshRelayEnforcementRuntime(true);
  saveLastRelaySnapshotIfNeeded();
  return true;
}

bool saveRelayPulseConfig(const uint8_t *pulse_enabled, const uint16_t *pulse_seconds) {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putBytes("rel_pulse_en", pulse_enabled, sizeof(config.relay_pulse_enabled));
  prefs.putBytes("rel_pulse_s", pulse_seconds, sizeof(config.relay_pulse_seconds));
  prefs.end();
  if (!loadConfig()) return false;
  refreshRelayPulseRuntime(true);
  return true;
}

bool saveLightConfig() {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putUChar("lt_power", light.power ? 1 : 0);
  prefs.putUChar("lt_dim", light.dimmer);
  prefs.putUShort("lt_ct", light.ct);
  prefs.putUChar("lt_mode", light.mode);
  prefs.putBytes("lt_rgb", light.rgb, sizeof(light.rgb));
  prefs.putUChar("lt_on_dim", config.light_on_dimmer);
  prefs.putUChar("lt_fade", config.light_fade);
  prefs.putUChar("lt_speed", config.light_speed);
  prefs.putUChar("lt_restore", config.light_restore_boot ? 1 : 0);
  prefs.end();
  config.light_power = light.power ? 1 : 0;
  config.light_dimmer = light.dimmer;
  config.light_ct = light.ct;
  config.light_mode = light.mode;
  memcpy(config.light_rgb, light.rgb, sizeof(config.light_rgb));
  light.config_dirty = false;
  return true;
}

bool saveInputConfig(const StoredConfig &source) {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putUShort("btn_hold", source.button_hold_ms);
  prefs.putUShort("btn_db", source.button_debounce_ms);
  prefs.putBytes("in_mode", source.input_mode, sizeof(source.input_mode));
  prefs.putBytes("in_relay", source.input_relay, sizeof(source.input_relay));
  prefs.putBytes("in_level", source.input_on_level, sizeof(source.input_on_level));
  prefs.putBytes("bp_act", source.button_press_action, sizeof(source.button_press_action));
  prefs.putBytes("bh_act", source.button_hold_action, sizeof(source.button_hold_action));
  prefs.putBytes("bp_rel", source.button_press_relay, sizeof(source.button_press_relay));
  prefs.putBytes("bh_rel", source.button_hold_relay, sizeof(source.button_hold_relay));
  prefs.putBytes("bp_tgt", source.button_press_target, sizeof(source.button_press_target));
  prefs.putBytes("bp_pld", source.button_press_payload, sizeof(source.button_press_payload));
  prefs.putBytes("bh_tgt", source.button_hold_target, sizeof(source.button_hold_target));
  prefs.putBytes("bh_pld", source.button_hold_payload, sizeof(source.button_hold_payload));
  prefs.end();
  return loadConfig();
}

bool factoryResetConfig() {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.clear();
  prefs.end();
  Preferences boot_prefs;
  if (boot_prefs.begin("mymota32-boot", false)) {
    boot_prefs.clear();
    boot_prefs.end();
  }
  clearGracefulRelaySnapshot();
  clearLastRelaySnapshot();
  setDefaultConfig();
  config_ok = false;
  return true;
}

void loadBootRecoveryState() {
  Preferences boot_prefs;
  if (!boot_prefs.begin("mymota32-boot", false)) {
    boot_recovery_count = 0;
    boot_recovery_factory_reset = false;
    return;
  }
  boot_recovery_count = boot_prefs.getUInt("count", 0) + 1;
  boot_prefs.putUInt("count", boot_recovery_count);
  boot_prefs.end();

  if (boot_recovery_count >= kBootRecoveryLimit) {
    boot_recovery_factory_reset = true;
    factoryResetConfig();
    boot_recovery_count = 0;
  }
}

void clearBootRecoveryState() {
  Preferences boot_prefs;
  if (!boot_prefs.begin("mymota32-boot", false)) return;
  boot_prefs.putUInt("count", 0);
  boot_prefs.end();
  boot_recovery_count = 0;
  boot_recovery_cleared = true;
}

void maintainBootRecovery() {
  if (boot_recovery_cleared) return;
  if (millis() - boot_started_ms >= kBootRecoveryStableMs) {
    clearBootRecoveryState();
  }
}

bool wifiTxPowerIsMax() {
  return wifi_tx_power_qdbm >= kWifiTxPowerMaxQdbm;
}

bool wifiDynamicPowerApplied() {
  return wifi_dynamic_power_samples >= kWifiDynamicPowerSampleCount;
}

void appendWifiTxPowerDbm(String &out) {
  out += String(wifi_tx_power_qdbm / 4);
  out += '.';
  out += static_cast<char>('0' + (((wifi_tx_power_qdbm & 3) * 10 + 2) / 4));
}

void setWifiTxPowerQdbm(int8_t qdbm) {
  if (esp_wifi_set_max_tx_power(qdbm) == ESP_OK) {
    wifi_tx_power_qdbm = qdbm;
  }
}

void resetWifiDynamicPowerRuntime(bool restore_max) {
  wifi_dynamic_power_connected_since = WiFi.status() == WL_CONNECTED ? millis() : 0;
  wifi_dynamic_power_last_sample = 0;
  wifi_dynamic_power_samples = 0;
  wifi_dynamic_power_disconnects = 0;
  wifi_dynamic_power_rssi_sum = 0;
  if (restore_max) setWifiTxPowerQdbm(kWifiTxPowerMaxQdbm);
}

int8_t wifiDynamicPowerTargetQdbm(int16_t rssi) {
  if (rssi >= -40) return kWifiTxPowerStrongQdbm;
  if (rssi >= -50) return kWifiTxPowerMediumQdbm;
  return kWifiTxPowerMaxQdbm;
}

void prepareWifiTxPowerForConnect() {
  setWifiTxPowerQdbm(kWifiTxPowerMaxQdbm);
  wifi_dynamic_power_connected_since = 0;
  wifi_dynamic_power_last_sample = 0;
  wifi_dynamic_power_samples = 0;
  wifi_dynamic_power_rssi_sum = 0;
}

void maintainWifiDynamicPower() {
  const bool connected = WiFi.status() == WL_CONNECTED;
  const uint32_t now = millis();

  if (!config.wifi_dynamic_power) {
    if (!wifiTxPowerIsMax()) setWifiTxPowerQdbm(kWifiTxPowerMaxQdbm);
    if (!connected) wifi_dynamic_power_connected_since = 0;
    wifi_dynamic_power_samples = 0;
    return;
  }

  if (!connected) {
    if (wifi_dynamic_power_connected_since && wifiDynamicPowerApplied() && !wifiTxPowerIsMax()) {
      wifi_dynamic_power_disconnects++;
      setWifiTxPowerQdbm(kWifiTxPowerMaxQdbm);
    }
    wifi_dynamic_power_connected_since = 0;
    wifi_dynamic_power_last_sample = 0;
    wifi_dynamic_power_samples = 0;
    wifi_dynamic_power_rssi_sum = 0;
    return;
  }

  if (!wifi_dynamic_power_connected_since) {
    wifi_dynamic_power_connected_since = now;
    wifi_dynamic_power_last_sample = 0;
    wifi_dynamic_power_samples = 0;
    wifi_dynamic_power_rssi_sum = 0;
    setWifiTxPowerQdbm(kWifiTxPowerMaxQdbm);
    return;
  }

  if (wifi_dynamic_power_disconnects >= kWifiDynamicPowerDisconnectLimit || wifiDynamicPowerApplied()) return;
  if (now - wifi_dynamic_power_connected_since < kWifiDynamicPowerSettleMs) return;
  if (wifi_dynamic_power_last_sample && now - wifi_dynamic_power_last_sample < kWifiDynamicPowerSampleMs) return;

  const int16_t rssi = static_cast<int16_t>(WiFi.RSSI());
  wifi_dynamic_power_last_sample = now;
  wifi_dynamic_power_rssi_sum += rssi;
  wifi_dynamic_power_samples++;
  if (wifi_dynamic_power_samples < kWifiDynamicPowerSampleCount) return;

  const int16_t avg_rssi = wifi_dynamic_power_rssi_sum / static_cast<int16_t>(wifi_dynamic_power_samples);
  setWifiTxPowerQdbm(wifiDynamicPowerTargetQdbm(avg_rssi));
}

void applyPhyMode(uint8_t phy_mode) {
  phy_mode = sanitizePhyMode(phy_mode);
  uint8_t protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
  switch (phy_mode) {
    case kPhyModeB: protocol = WIFI_PROTOCOL_11B; break;
    case kPhyModeG: protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G; break;
    case kPhyModeN: protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N; break;
    default: break;
  }
  esp_wifi_set_protocol(WIFI_IF_STA, protocol);
}

bool waitForWifi(uint32_t timeout_ms) {
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeout_ms) {
    delay(100);
  }
  return WiFi.status() == WL_CONNECTED;
}

bool connectWifiWithPhy(uint8_t phy_mode, uint32_t timeout_ms) {
  WiFi.disconnect(false, true);
  delay(100);
  WiFi.mode(WIFI_STA);
  applyPhyMode(phy_mode);
  prepareWifiTxPowerForConnect();
  WiFi.begin(config.ssid, config.password);
  last_wifi_begin_attempt = millis();
  return waitForWifi(timeout_ms);
}

void startAp() {
  if (ap_started) return;
  const uint32_t now = millis();
  if (last_ap_attempt && (now - last_ap_attempt < kApRetryMs)) return;
  last_ap_attempt = now;
  const String ap_name = defaultHostname();
  WiFi.mode(WIFI_AP_STA);
  ap_started = WiFi.softAP(ap_name.c_str());
}

void stopAp() {
  if (!ap_started) return;
  if (WiFi.softAPdisconnect(true)) {
    ap_started = false;
    last_ap_attempt = 0;
  }
}

void beginWifiReconnect(uint32_t now) {
  if (!config_ok || WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(ap_started ? WIFI_AP_STA : WIFI_STA);
  applyPhyMode(config.phy_mode);
  prepareWifiTxPowerForConnect();
  WiFi.begin(config.ssid, config.password);
  last_wifi_begin_attempt = now;
}

void prepareWifi() {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(config.hostname);
  WiFi.setSleep(false);
  resetWifiDynamicPowerRuntime(false);
}

void connectWifi() {
  prepareWifi();
  if (!config_ok) {
    WiFi.mode(WIFI_AP);
    startAp();
    return;
  }
  disconnected_since = millis();
  disconnected_timer_active = true;
  if (connectWifiWithPhy(config.phy_mode, kConnectTimeoutMs)) {
    sta_connected_once = true;
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    if (config.phy_mode != kPhyModeAuto && config.phy_mode != kPhyModeFailsafe) {
      if (connectWifiWithPhy(kPhyModeFailsafe, kConnectTimeoutMs)) {
        sta_connected_once = true;
        return;
      }
    }
  }
}

void maintainWifi() {
  if (!config_ok) {
    startAp();
    return;
  }
  if (WiFi.status() == WL_CONNECTED) {
    sta_connected_once = true;
    stopAp();
    disconnected_since = 0;
    disconnected_timer_active = false;
    last_wifi_begin_attempt = 0;
    return;
  }
  const uint32_t now = millis();
  if (!disconnected_timer_active) {
    disconnected_since = now;
    disconnected_timer_active = true;
    last_wifi_begin_attempt = now;
  }
  if (now - last_wifi_begin_attempt >= kWifiReconnectBeginMs) {
    beginWifiReconnect(now);
  }
  if (!sta_connected_once && !ap_started && now - disconnected_since >= kInitialFallbackApMs) {
    startAp();
  }
}

void updateDeviceLeds(bool force = false) {
  const uint32_t now = millis();
  if (!force && now - last_led_update < kLedUpdateMs) return;
  last_led_update = now;
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    const PinAssignment *assignment = ledOutputAssignment(i);
    if (!assignment || !hasLedOutput(i)) continue;
    writeAssignedPin(*assignment, ledOutputOn(i));
  }
}

void scheduleMqttRelayPublish(uint8_t relay);
void scheduleMqttRelayOffEnergyReport(uint8_t relay);
bool persistEnergyTotal(bool force);
bool mqttConfigured();
bool parseUint16Input(const String &input, uint16_t min_value, uint16_t max_value, uint16_t &out);
bool executeDeviceCommand(const char *raw, size_t cmd_len, const char *arg, size_t arg_len, String &out, String &error);

uint32_t relaySnapshotHashByte(uint32_t hash, uint8_t value) {
  hash ^= value;
  return hash * 16777619UL;
}

uint32_t relayTemplateSignature() {
  uint32_t hash = 2166136261UL;
  hash = relaySnapshotHashByte(hash, runtime_template.relay_count);
  for (uint8_t i = 0; i < kMaxRelays; i++) {
    const PinAssignment &relay = runtime_template.relays[i];
    hash = relaySnapshotHashByte(hash, relay.pin);
    hash = relaySnapshotHashByte(hash, relay.inverted ? 1 : 0);
    hash = relaySnapshotHashByte(hash, relay.no_pullup ? 1 : 0);
    hash = relaySnapshotHashByte(hash, hasPin(relay) ? 1 : 0);
  }
  return hash;
}

uint16_t relayStateMask() {
  uint16_t mask = 0;
  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (!relayAvailable(i) || !relay_state[i]) continue;
    mask |= (1U << i);
  }
  return mask;
}

uint32_t gracefulRelaySnapshotCrc(const GracefulRelaySnapshot &snapshot) {
  const uint8_t *data = reinterpret_cast<const uint8_t *>(&snapshot);
  const size_t crc_offset = offsetof(GracefulRelaySnapshot, crc);
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < sizeof(snapshot); i++) {
    const bool crc_byte = i >= crc_offset && i < crc_offset + sizeof(snapshot.crc);
    hash = relaySnapshotHashByte(hash, crc_byte ? 0 : data[i]);
  }
  return hash;
}

bool readRelaySnapshot(const char *key, GracefulRelaySnapshot &snapshot) {
  memset(&snapshot, 0, sizeof(snapshot));
  Preferences relay_prefs;
  if (!relay_prefs.begin(kGracefulRelayPrefsNamespace, true)) return false;
  const size_t got = relay_prefs.getBytesLength(key);
  if (got == sizeof(snapshot)) {
    relay_prefs.getBytes(key, &snapshot, sizeof(snapshot));
  }
  relay_prefs.end();
  return got == sizeof(snapshot);
}

bool clearRelaySnapshot(const char *key) {
  Preferences relay_prefs;
  if (!relay_prefs.begin(kGracefulRelayPrefsNamespace, false)) return false;
  relay_prefs.remove(key);
  relay_prefs.end();
  return true;
}

bool clearGracefulRelaySnapshot() {
  return clearRelaySnapshot(kGracefulRelayPrefsKey);
}

bool clearLastRelaySnapshot() {
  return clearRelaySnapshot(kLastRelayPrefsKey);
}

bool relaySnapshotValid(const GracefulRelaySnapshot &snapshot) {
  if (snapshot.magic != kGracefulRelaySnapshotMagic) return false;
  if (snapshot.version != kGracefulRelaySnapshotVersion) return false;
  if (snapshot.size != sizeof(GracefulRelaySnapshot)) return false;
  if (snapshot.chip_id != chipIdValue()) return false;
  if (snapshot.relay_count != runtime_template.relay_count) return false;
  if (snapshot.relay_signature != relayTemplateSignature()) return false;
  return snapshot.crc == gracefulRelaySnapshotCrc(snapshot);
}

void loadGracefulRelaySnapshot() {
  graceful_relay_restore_valid = false;
  graceful_relay_restore_mask = 0;

  GracefulRelaySnapshot snapshot{};
  if (esp_reset_reason() == ESP_RST_SW &&
      readRelaySnapshot(kGracefulRelayPrefsKey, snapshot) &&
      relaySnapshotValid(snapshot)) {
    graceful_relay_restore_mask = snapshot.relay_mask;
    graceful_relay_restore_valid = true;
  }
  clearGracefulRelaySnapshot();
}

void loadLastRelaySnapshot() {
  last_relay_restore_valid = false;
  last_relay_restore_mask = 0;

  GracefulRelaySnapshot snapshot{};
  if (readRelaySnapshot(kLastRelayPrefsKey, snapshot) &&
      relaySnapshotValid(snapshot)) {
    last_relay_restore_mask = snapshot.relay_mask;
    last_relay_restore_valid = true;
  }
}

bool saveRelaySnapshot(const char *key) {
  if (runtime_template.relay_count == 0) return clearRelaySnapshot(key);

  GracefulRelaySnapshot snapshot{};
  snapshot.magic = kGracefulRelaySnapshotMagic;
  snapshot.version = kGracefulRelaySnapshotVersion;
  snapshot.size = sizeof(GracefulRelaySnapshot);
  snapshot.chip_id = chipIdValue();
  snapshot.relay_mask = relayStateMask();
  snapshot.relay_count = runtime_template.relay_count;
  snapshot.relay_signature = relayTemplateSignature();
  snapshot.crc = gracefulRelaySnapshotCrc(snapshot);

  Preferences relay_prefs;
  if (!relay_prefs.begin(kGracefulRelayPrefsNamespace, false)) return false;
  const size_t written = relay_prefs.putBytes(key, &snapshot, sizeof(snapshot));
  relay_prefs.end();
  return written == sizeof(snapshot);
}

bool saveGracefulRelaySnapshot() {
  return saveRelaySnapshot(kGracefulRelayPrefsKey);
}

bool saveLastRelaySnapshot() {
  if (!saveRelaySnapshot(kLastRelayPrefsKey)) return false;
  last_relay_restore_mask = relayStateMask();
  last_relay_restore_valid = runtime_template.relay_count > 0;
  return true;
}

bool gracefulRelayRestoreState(uint8_t relay) {
  return graceful_relay_restore_valid &&
         relay < kMaxRelays &&
         relay < runtime_template.relay_count &&
         (graceful_relay_restore_mask & (1U << relay));
}

bool lastRelayRestoreState(uint8_t relay) {
  return last_relay_restore_valid &&
         relay < kMaxRelays &&
         relay < runtime_template.relay_count &&
         (last_relay_restore_mask & (1U << relay));
}

bool anyRelayRestoreBootEnabled() {
  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (relayAvailable(i) && config.relay_restore_boot[i]) return true;
  }
  return false;
}

bool saveLastRelaySnapshotIfNeeded() {
  if (!anyRelayRestoreBootEnabled()) return true;
  return saveLastRelaySnapshot();
}

bool relayBootState(uint8_t relay) {
  if (relay < kMaxRelays && config.relay_restore_boot[relay]) {
    if (last_relay_restore_valid) return lastRelayRestoreState(relay);
    if (graceful_relay_restore_valid) return gracefulRelayRestoreState(relay);
    return false;
  }
  if (graceful_relay_restore_valid) return gracefulRelayRestoreState(relay);
  return relay < kMaxRelays && config.relay_on_boot[relay];
}

bool relayTimeEnforcementActive(uint8_t relay) {
  return relayAvailable(relay) &&
         config.relay_time_enabled[relay] &&
         config.relay_time_seconds[relay] >= kRelayEnforcementMinSeconds;
}

bool relayPulseActive(uint8_t relay) {
  return relayAvailable(relay) &&
         config.relay_pulse_enabled[relay] &&
         config.relay_pulse_seconds[relay] >= kRelayPulseMinSeconds;
}

void cancelRelayEnforcement(uint8_t relay) {
  if (relay >= kMaxRelays) return;
  relay_enforcement_pending[relay] = false;
  relay_enforcement_due[relay] = 0;
}

void scheduleRelayEnforcement(uint8_t relay) {
  if (relay >= kMaxRelays || !relayTimeEnforcementActive(relay)) {
    cancelRelayEnforcement(relay);
    return;
  }
  relay_enforcement_due[relay] = millis() + (static_cast<uint32_t>(config.relay_time_seconds[relay]) * 1000UL);
  relay_enforcement_pending[relay] = true;
}

void refreshRelayEnforcementRuntime(bool schedule_off_relays) {
  for (uint8_t i = 0; i < kMaxRelays; i++) {
    if (!relayTimeEnforcementActive(i) || relay_state[i]) {
      cancelRelayEnforcement(i);
    } else if (schedule_off_relays) {
      scheduleRelayEnforcement(i);
    }
  }
}

void cancelRelayPulse(uint8_t relay) {
  if (relay >= kMaxRelays) return;
  relay_pulse_pending[relay] = false;
  relay_pulse_due[relay] = 0;
}

void scheduleRelayPulse(uint8_t relay) {
  if (relay >= kMaxRelays || !relayPulseActive(relay)) {
    cancelRelayPulse(relay);
    return;
  }
  relay_pulse_due[relay] = millis() + (static_cast<uint32_t>(config.relay_pulse_seconds[relay]) * 1000UL);
  relay_pulse_pending[relay] = true;
}

void refreshRelayPulseRuntime(bool schedule_on_relays) {
  for (uint8_t i = 0; i < kMaxRelays; i++) {
    if (!relayPulseActive(i) || !relay_state[i]) {
      cancelRelayPulse(i);
    } else if (schedule_on_relays) {
      scheduleRelayPulse(i);
    }
  }
}

void setRelay(uint8_t relay, bool on, bool suppress_off_enforcement = false) {
  if (relay >= kMaxRelays || !hasPin(runtime_template.relays[relay])) return;
  const bool changed = relay_state[relay] != on;
  const bool was_on = relay_state[relay];
  relay_state[relay] = on;
  writeAssignedPin(runtime_template.relays[relay], on);
  if (on) {
    cancelRelayEnforcement(relay);
    scheduleRelayPulse(relay);
  } else {
    cancelRelayPulse(relay);
  }
  if (!on && changed && !suppress_off_enforcement) {
    scheduleRelayEnforcement(relay);
  }
  if (changed) {
    updateDeviceLeds(true);
    saveLastRelaySnapshotIfNeeded();
    scheduleMqttRelayPublish(relay);
    if (was_on && !on) {
      energy_persist_requested = true;
      scheduleMqttRelayOffEnergyReport(relay);
    }
  }
}

void toggleRelay(uint8_t relay) {
  if (relay >= kMaxRelays) return;
  setRelay(relay, !relay_state[relay]);
}

void setupDevicePins() {
  memset(relay_enforcement_pending, 0, sizeof(relay_enforcement_pending));
  memset(relay_enforcement_due, 0, sizeof(relay_enforcement_due));
  memset(relay_pulse_pending, 0, sizeof(relay_pulse_pending));
  memset(relay_pulse_due, 0, sizeof(relay_pulse_due));

  for (uint8_t i = 0; i < kMaxRelays; i++) {
    relay_state[i] = relayBootState(i);
    if (!hasPin(runtime_template.relays[i])) continue;
    writeAssignedPin(runtime_template.relays[i], relay_state[i]);
    pinMode(runtime_template.relays[i].pin, OUTPUT);
    writeAssignedPin(runtime_template.relays[i], relay_state[i]);
    if (relay_state[i]) scheduleRelayPulse(i);
  }
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    const PinAssignment *assignment = ledOutputAssignment(i);
    if (!assignment || !hasLedOutput(i)) continue;
    writeAssignedPin(*assignment, false);
    pinMode(assignment->pin, OUTPUT);
    writeAssignedPin(*assignment, false);
  }
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    if (!hasPin(runtime_template.buttons[i])) continue;
    pinMode(runtime_template.buttons[i].pin, runtime_template.buttons[i].no_pullup ? INPUT : INPUT_PULLUP);
    const bool active = readInputActive(i);
    button_state[i] = {active, active, false, millis(), millis()};
  }
  graceful_relay_restore_valid = false;
  graceful_relay_restore_mask = 0;
  saveLastRelaySnapshotIfNeeded();
  updateDeviceLeds(true);
}

bool defaultButtonRelayTarget(uint8_t button, uint8_t &relay) {
  return defaultInputRelayTarget(button, relay);
}

uint8_t configuredButtonRelayTarget(uint8_t button, bool hold) {
  if (button >= kMaxButtons) return kButtonRelayUnset;
  return hold ? config.button_hold_relay[button] : config.button_press_relay[button];
}

bool buttonRelayTarget(uint8_t button, bool hold, uint8_t &relay) {
  const uint8_t configured = configuredButtonRelayTarget(button, hold);
  if (relayAvailable(configured)) {
    relay = configured;
    return true;
  }
  return defaultButtonRelayTarget(button, relay);
}

bool buttonActionAvailable(uint8_t button, uint8_t action) {
  if (action == kButtonActionNone) return true;
  if (action == kButtonActionRelayToggle) {
    uint8_t relay = 0;
    return defaultButtonRelayTarget(button, relay);
  }
  if (action == kButtonActionMqtt || action == kButtonActionWebhook) return true;
  return false;
}

const char *buttonEventType(bool hold) {
  return hold ? "HOLD" : "TOGGLE";
}

const char *buttonActionTarget(uint8_t button, bool hold) {
  if (button >= kMaxButtons) return "";
  return hold ? config.button_hold_target[button] : config.button_press_target[button];
}

const char *buttonActionPayload(uint8_t button, bool hold) {
  if (button >= kMaxButtons) return "";
  return hold ? config.button_hold_payload[button] : config.button_press_payload[button];
}

bool parseRelayStateToken(const char *token, size_t len, uint8_t &relay) {
  if (len < 14 || strncmp(token, "{RELAY", 6) != 0) return false;
  uint16_t number = 0;
  size_t index = 6;
  while (index < len && token[index] >= '0' && token[index] <= '9') {
    number = (number * 10U) + static_cast<uint16_t>(token[index] - '0');
    index++;
  }
  if (number == 0 || number > kMaxRelays) return false;
  if (index + 7 != len || strncmp(token + index, "_STATE}", 7) != 0) return false;
  relay = static_cast<uint8_t>(number - 1);
  return true;
}

void appendRelayStateTokenValue(String &out, uint8_t relay) {
  const bool available = relay < runtime_template.relay_count && hasPin(runtime_template.relays[relay]);
  out += available ? (relay_state[relay] ? F("ON") : F("OFF")) : F("UNKNOWN");
}

String expandButtonActionText(const char *input, uint8_t button, bool hold) {
  String out;
  if (!input) return out;
  out.reserve(strlen(input) + strlen(config.mqtt_topic) + 16);

  const char *p = input;
  while (*p) {
    if (*p != '{') {
      out += *p++;
      continue;
    }
    const char *end = strchr(p, '}');
    if (!end) {
      out += *p++;
      continue;
    }
    const size_t len = static_cast<size_t>(end - p + 1);
    uint8_t relay = 0;
    if (len == 10 && strncmp(p, "{BUTTONID}", len) == 0) {
      out += String(inputFunctionIndex(button) + 1);
    } else if (len == 6 && strncmp(p, "{TYPE}", len) == 0) {
      out += buttonEventType(hold);
    } else if (len == 7 && strncmp(p, "{TOPIC}", len) == 0) {
      out += config.mqtt_topic;
    } else if (parseRelayStateToken(p, len, relay)) {
      appendRelayStateTokenValue(out, relay);
    } else {
      for (const char *copy = p; copy <= end; copy++) {
        out += *copy;
      }
    }
    p = end + 1;
  }
  return out;
}

bool hasControlChar(const String &value, bool allow_multiline = false) {
  for (size_t i = 0; i < value.length(); i++) {
    const char c = value[i];
    if (allow_multiline && (c == '\n' || c == '\r' || c == '\t')) continue;
    if (static_cast<uint8_t>(c) < 0x20 || c == 0x7f) return true;
  }
  return false;
}

bool isValidButtonActionText(const String &value, size_t max_len, bool allow_empty, bool allow_multiline = false) {
  if (!allow_empty && value.length() == 0) return false;
  if (value.length() > max_len) return false;
  return !hasControlChar(value, allow_multiline);
}

bool isValidMqttPublishTopicTemplate(const String &topic) {
  if (!isValidButtonActionText(topic, kButtonActionTargetMaxLen, false)) return false;
  for (size_t i = 0; i < topic.length(); i++) {
    if (topic[i] == '#' || topic[i] == '+') return false;
  }
  return true;
}

void clearMqttButtonQueue() {
  mqtt_button_queue_head = 0;
  mqtt_button_queue_count = 0;
}

uint8_t mqttButtonQueueIndex(uint8_t offset) {
  return (mqtt_button_queue_head + offset) % kMqttButtonQueueDepth;
}

void dropMqttButtonQueueHead() {
  if (mqtt_button_queue_count == 0) return;
  mqtt_button_queue_head = mqttButtonQueueIndex(1);
  mqtt_button_queue_count--;
}

bool mqttButtonQueueExpired(const MqttButtonPending &item, uint32_t now) {
  return static_cast<uint32_t>(now - item.queued_at) > kMqttButtonQueueMaxAgeMs;
}

void expireMqttButtonQueue(uint32_t now) {
  while (mqtt_button_queue_count > 0 && mqttButtonQueueExpired(mqtt_button_queue[mqtt_button_queue_head], now)) {
    dropMqttButtonQueueHead();
  }
}

bool pushMqttButtonQueue(const String &topic, const String &payload) {
  if (topic.length() == 0 || topic.length() > kMqttButtonTopicMaxLen) return false;
  if (payload.length() == 0 || payload.length() > kMqttButtonPayloadMaxLen) return false;
  if (mqtt_button_queue_count >= kMqttButtonQueueDepth) {
    dropMqttButtonQueueHead();
  }
  MqttButtonPending &slot = mqtt_button_queue[mqttButtonQueueIndex(mqtt_button_queue_count)];
  slot.queued_at = millis();
  strlcpy(slot.topic, topic.c_str(), sizeof(slot.topic));
  strlcpy(slot.payload, payload.c_str(), sizeof(slot.payload));
  mqtt_button_queue_count++;
  return true;
}

bool mqttQueueButtonAction(uint8_t button, bool hold) {
  if (button >= kMaxButtons) return false;
  if (!mqttConfigured()) return false;
  String topic = expandButtonActionText(buttonActionTarget(button, hold), button, hold);
  String payload = expandButtonActionText(buttonActionPayload(button, hold), button, hold);
  topic.trim();
  if (topic.length() == 0 || payload.length() == 0) return false;
  return pushMqttButtonQueue(topic, payload);
}

bool parseHttpUrl(const String &url, String &host, uint16_t &port, String &path) {
  String value = url;
  value.trim();
  if (!value.startsWith(F("http://"))) return false;
  const int host_start = 7;
  const int path_start = value.indexOf('/', host_start);
  String host_port = path_start < 0 ? value.substring(host_start) : value.substring(host_start, path_start);
  host_port.trim();
  if (host_port.length() == 0) return false;
  port = 80;
  const int colon = host_port.lastIndexOf(':');
  if (colon >= 0) {
    const String port_text = host_port.substring(colon + 1);
    uint16_t parsed_port = 0;
    if (!parseUint16Input(port_text, 1, 65535U, parsed_port)) return false;
    port = parsed_port;
    host = host_port.substring(0, colon);
  } else {
    host = host_port;
  }
  host.trim();
  if (host.length() == 0 || host.indexOf(' ') >= 0) return false;
  path = path_start < 0 ? String(F("/")) : value.substring(path_start);
  if (path.length() == 0) path = F("/");
  return true;
}

uint8_t hexNibble(char c) {
  if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
  if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(c - 'a' + 10);
  if (c >= 'A' && c <= 'F') return static_cast<uint8_t>(c - 'A' + 10);
  return 0xff;
}

bool urlDecodeComponent(const String &input, String &out) {
  out = "";
  out.reserve(input.length());
  for (size_t i = 0; i < input.length(); i++) {
    const char c = input[i];
    if (c == '+') {
      out += ' ';
      continue;
    }
    if (c != '%') {
      out += c;
      continue;
    }
    if (i + 2 >= input.length()) return false;
    const uint8_t hi = hexNibble(input[i + 1]);
    const uint8_t lo = hexNibble(input[i + 2]);
    if (hi > 0x0f || lo > 0x0f) return false;
    const char decoded = static_cast<char>((hi << 4) | lo);
    if (decoded == '\0' || (static_cast<uint8_t>(decoded) < 0x20 && decoded != '\t')) return false;
    out += decoded;
    i += 2;
  }
  return true;
}

bool executeCmndString(const String &cmnd_str, String &out, String &error) {
  const char *raw = cmnd_str.c_str();
  size_t total_len = cmnd_str.length();

  while (total_len > 0) {
    const char c = raw[0];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    raw++;
    total_len--;
  }
  while (total_len > 0) {
    const char c = raw[total_len - 1];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    total_len--;
  }

  size_t cmd_len = 0;
  while (cmd_len < total_len) {
    const char c = raw[cmd_len];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') break;
    cmd_len++;
  }
  size_t arg_start = cmd_len;
  while (arg_start < total_len) {
    const char c = raw[arg_start];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    arg_start++;
  }

  if (cmd_len == 0) {
    error = F("Invalid cmnd");
    return false;
  }

  const size_t arg_len = arg_start < total_len ? total_len - arg_start : 0;
  return executeDeviceCommand(raw, cmd_len, raw + arg_start, arg_len, out, error);
}

bool webhookHostIsLocal(const String &host) {
  if (WiFi.status() == WL_CONNECTED && host == ipToString(WiFi.localIP())) return true;
  if (host.equalsIgnoreCase(config.hostname)) return true;
  const int dot = String(config.hostname).indexOf('.');
  if (dot > 0 && host.equalsIgnoreCase(String(config.hostname).substring(0, dot))) return true;
  return strcasecmp(host.c_str(), "localhost") == 0 || host == F("127.0.0.1");
}

bool runLocalCmndWebhookPath(const String &path) {
  const int query_start = path.indexOf('?');
  if (query_start < 0) return false;
  if (!path.substring(0, query_start).equalsIgnoreCase(F("/cm"))) return false;

  size_t start = static_cast<size_t>(query_start + 1);
  while (start < path.length()) {
    int next = path.indexOf('&', start);
    if (next < 0) next = path.length();
    const int eq = path.indexOf('=', start);
    if (eq >= 0 && eq < next) {
      String name;
      if (!urlDecodeComponent(path.substring(start, eq), name)) return false;
      if (name.equalsIgnoreCase(F("cmnd"))) {
        String cmnd;
        if (!urlDecodeComponent(path.substring(eq + 1, next), cmnd)) return false;
        String out;
        String error;
        return executeCmndString(cmnd, out, error);
      }
    }
    start = static_cast<size_t>(next + 1);
  }
  return false;
}

void drainWebhookResponse(WiFiClient &client) {
  const uint32_t started = millis();
  while (client.connected() && static_cast<uint32_t>(millis() - started) < kWebhookFlushTimeoutMs) {
    while (client.available()) {
      client.read();
    }
    delay(1);
  }
  while (client.available()) {
    client.read();
  }
}

bool runWebhookAction(uint8_t button, bool hold) {
  if (WiFi.status() != WL_CONNECTED) return false;
  const String url = expandButtonActionText(buttonActionTarget(button, hold), button, hold);
  String host;
  uint16_t port = 80;
  String path;
  if (!parseHttpUrl(url, host, port, path)) return false;
  if (port == 80 && webhookHostIsLocal(host)) return runLocalCmndWebhookPath(path);
  WiFiClient client;
  client.setTimeout(kWebhookConnectTimeoutMs);
  if (!client.connect(host.c_str(), port)) return false;
  String request;
  request.reserve(path.length() + host.length() + 90);
  request += F("GET ");
  request += path;
  request += F(" HTTP/1.1\r\nHost: ");
  request += host;
  request += F("\r\nConnection: close\r\nUser-Agent: myMota32/");
  request += F(MYMOTA32_VERSION);
  request += F("\r\n\r\n");
  if (client.print(request) != request.length()) {
    client.stop();
    return false;
  }
  drainWebhookResponse(client);
  client.stop();
  return true;
}

bool runButtonAction(uint8_t button, uint8_t action, bool hold) {
  if (action == kButtonActionRelayToggle) {
    uint8_t relay = 0;
    if (buttonRelayTarget(button, hold, relay)) {
      toggleRelay(relay);
      return true;
    }
  } else if (action == kButtonActionMqtt) {
    return mqttQueueButtonAction(button, hold);
  } else if (action == kButtonActionWebhook) {
    return runWebhookAction(button, hold);
  }
  return false;
}

void maintainButtons() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < runtime_template.button_count; i++) {
    if (!hasPin(runtime_template.buttons[i])) continue;
    const bool raw = readInputActive(i);
    if (raw != button_state[i].raw_pressed) {
      button_state[i].raw_pressed = raw;
      button_state[i].changed_at = now;
    }
    if ((now - button_state[i].changed_at) >= config.button_debounce_ms && raw != button_state[i].stable_pressed) {
      button_state[i].stable_pressed = raw;
      if (effectiveInputMode(i) == kInputModeSwitch) {
        uint8_t relay = 0;
        if (inputRelayTarget(i, relay)) setRelay(relay, raw);
        button_state[i].hold_emitted = false;
      } else if (raw) {
        button_state[i].pressed_at = now;
        button_state[i].hold_emitted = false;
      } else if (!button_state[i].hold_emitted) {
        const uint8_t action = config.button_press_action[i];
        if (action != kButtonActionNone) runButtonAction(i, action, false);
      }
    }
    if (effectiveInputMode(i) == kInputModeButton && button_state[i].stable_pressed && !button_state[i].hold_emitted) {
      if ((now - button_state[i].pressed_at) >= config.button_hold_ms) {
        button_state[i].hold_emitted = true;
        const uint8_t action = config.button_hold_action[i];
        if (action != kButtonActionNone) runButtonAction(i, action, true);
      }
    }
  }
}

void maintainRelayEnforcement() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (!relay_enforcement_pending[i]) continue;
    if (!relayTimeEnforcementActive(i) || relay_state[i]) {
      cancelRelayEnforcement(i);
      continue;
    }
    if (static_cast<int32_t>(now - relay_enforcement_due[i]) >= 0) {
      setRelay(i, true);
    }
  }
}

void maintainRelayPulsing() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (!relay_pulse_pending[i]) continue;
    if (!relayPulseActive(i) || !relay_state[i]) {
      cancelRelayPulse(i);
      continue;
    }
    if (static_cast<int32_t>(now - relay_pulse_due[i]) >= 0) {
      setRelay(i, false, true);
    }
  }
}

void maintainDevice() {
  maintainButtons();
  maintainRelayEnforcement();
  maintainRelayPulsing();
  updateDeviceLeds();
}

uint64_t kwhToUkwh(float value) {
  if (isnan(value) || value <= 0.0f) return 0;
  if (value >= static_cast<float>(kEnergyTotalMaxUkwh) / 1000000.0f) return kEnergyTotalMaxUkwh;
  return static_cast<uint64_t>((value * 1000000.0f) + 0.5f);
}

float ukwhToKwh(uint64_t value) {
  return static_cast<float>(value) / 1000000.0f;
}

float reportedEnergyTotalKwh() {
  return energy.total_kwh + config.energy_total_offset_kwh;
}

bool persistEnergyTotal(bool force) {
  if (!energy.present) return true;
  const uint64_t total = kwhToUkwh(energy.total_kwh);
  const uint64_t saved = energy_saved_ukwh;
  const uint64_t delta = total > saved ? total - saved : saved - total;
  const uint32_t now = millis();
  if (!force) {
    if (now - last_energy_persist_ms < kEnergyPersistMinMs) return true;
    if (!energy_persist_requested && delta < kEnergyPersistDeltaUkwh) return true;
  }
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putULong64("en_total", total);
  prefs.end();
  energy_saved_ukwh = total;
  last_energy_persist_ms = now;
  energy_persist_requested = false;
  return true;
}

void updateEnergyAggregateFromChannels() {
  energy.voltage = 0.0f;
  energy.current = 0.0f;
  energy.power = 0.0f;
  for (uint8_t i = 0; i < energy.channel_count && i < kEnergyMaxChannels; i++) {
    if (energy.channel[i].voltage > energy.voltage) energy.voltage = energy.channel[i].voltage;
    energy.current += energy.channel[i].current;
    energy.power += energy.channel[i].power;
  }
}

const __FlashStringHelper *energyDriverName() {
  if (energy.driver == kEnergyDriverBl0939) return F("BL0939");
  if (energy.driver == kEnergyDriverHlw8012) return energy.hjl ? F("BL0937/HJL-01") : F("HLW8012");
  return F("none");
}

bool energyRelayHasChannel(uint8_t relay) {
  if (!energy.present || relay >= runtime_template.relay_count || relay >= kMaxRelays) return false;
  if (energy.channel_count <= 1) return relay == 0;
  return relay < energy.channel_count && relay < kEnergyMaxChannels;
}

void updateEnergyRelayOffZero(uint8_t relay) {
  if (!energyRelayHasChannel(relay)) return;
  energy.channel[relay].power = 0.0f;
  energy.channel[relay].current = 0.0f;
  updateEnergyAggregateFromChannels();
}

void scheduleMqttEnergyReport(uint8_t reason) {
  if (!energy.present || !mqttConfigured() || reason == kMqttEnergyReportReasonNone) return;
  if (mqtt_pending_energy_report_reason == kMqttEnergyReportReasonRelayOff &&
      reason == kMqttEnergyReportReasonPowerZero) {
    return;
  }
  mqtt_pending_energy_report_reason = reason;
}

void scheduleMqttRelayOffEnergyReport(uint8_t relay) {
  if (!energyRelayHasChannel(relay)) return;
  updateEnergyRelayOffZero(relay);
  mqtt_pending_energy_zero_relay_mask |= (1U << relay);
  scheduleMqttEnergyReport(kMqttEnergyReportReasonRelayOff);
}

bool energyDevicePowerOn() {
  bool has_relay = false;
  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (!relayAvailable(i)) continue;
    has_relay = true;
    if (relay_state[i]) return true;
  }
  return !has_relay;
}

void IRAM_ATTR hlwCfInterrupt() {
  const uint32_t us = micros();
  if (energy.hlw_load_off) {
    energy.hlw_cf_pulse_last_us = us;
    energy.hlw_load_off = false;
    return;
  }
  energy.hlw_cf_pulse_length = us - energy.hlw_cf_pulse_last_us;
  energy.hlw_cf_pulse_last_us = us;
  energy.hlw_cf_summed_pulse_length += energy.hlw_cf_pulse_length;
  energy.hlw_cf_pulse_counter = energy.hlw_cf_pulse_counter + 1;
}

void IRAM_ATTR hlwCf1Interrupt() {
  const uint32_t us = micros();
  energy.hlw_cf1_pulse_length = us - energy.hlw_cf1_pulse_last_us;
  energy.hlw_cf1_pulse_last_us = us;
  if (energy.hlw_cf1_timer > kHlwCf1SampleStartTick &&
      energy.hlw_cf1_timer < kHlwCf1SampleEndTick) {
    energy.hlw_cf1_summed_pulse_length += energy.hlw_cf1_pulse_length;
    energy.hlw_cf1_pulse_counter = energy.hlw_cf1_pulse_counter + 1;
  }
}

void processHlwPulseEnergy(uint32_t now) {
  if (now - energy.last_hlw_update_ms < kHlwUpdateMs) return;
  energy.last_hlw_update_ms = now;

  uint32_t cf_pulse_length = 0;
  uint32_t cf_summed_pulse_length = 0;
  uint32_t cf_pulse_counter = 0;
  bool load_off = false;

  if (micros() - energy.hlw_cf_pulse_last_us > kHlwPowerProbeUs) {
    energy.hlw_cf_pulse_length = 0;
    energy.hlw_load_off = true;
  }

  noInterrupts();
  cf_pulse_length = energy.hlw_cf_pulse_length;
  cf_summed_pulse_length = energy.hlw_cf_summed_pulse_length;
  cf_pulse_counter = energy.hlw_cf_pulse_counter;
  load_off = energy.hlw_load_off;
  energy.hlw_cf_summed_pulse_length = 0;
  energy.hlw_cf_pulse_counter = 0;
  interrupts();

  energy.hlw_cf_power_pulse_length = cf_pulse_length;
  if (cf_pulse_counter && !load_off) {
    energy.hlw_cf_power_pulse_length = cf_summed_pulse_length / cf_pulse_counter;
  }

  EnergyChannelState &channel = energy.channel[0];
  const bool power_on = energyDevicePowerOn();
  if (energy.hlw_cf_power_pulse_length && power_on && !load_off) {
    const uint32_t watts_x10 = (energy.hlw_power_ratio * kHlwPowerCalibration) / energy.hlw_cf_power_pulse_length;
    channel.power = static_cast<float>(watts_x10) / 10.0f;
    energy.hlw_power_retry = 1;
    energy.last_success_ms = now;
  } else if (energy.hlw_power_retry) {
    energy.hlw_power_retry--;
  } else {
    channel.power = 0.0f;
  }

  if (digitalPinSupported(energy.cf1_pin)) {
    const uint8_t cf1_timer = energy.hlw_cf1_timer + 1;
    energy.hlw_cf1_timer = cf1_timer;
    if (cf1_timer >= kHlwCf1CycleTicks) {
      energy.hlw_cf1_timer = 0;
      energy.hlw_select_ui_flag = !energy.hlw_select_ui_flag;
      if (digitalPinSupported(energy.sel_pin)) {
        digitalWrite(energy.sel_pin, energy.hlw_select_ui_flag ? HIGH : LOW);
      }

      uint32_t cf1_summed_pulse_length = 0;
      uint32_t cf1_pulse_counter = 0;
      noInterrupts();
      cf1_summed_pulse_length = energy.hlw_cf1_summed_pulse_length;
      cf1_pulse_counter = energy.hlw_cf1_pulse_counter;
      energy.hlw_cf1_summed_pulse_length = 0;
      energy.hlw_cf1_pulse_counter = 0;
      interrupts();

      const uint32_t cf1_pulse_length = cf1_pulse_counter ? cf1_summed_pulse_length / cf1_pulse_counter : 0;
      if (energy.hlw_select_ui_flag == energy.hlw_voltage_on_selected) {
        energy.hlw_cf1_voltage_pulse_length = cf1_pulse_length;
        energy.voltage_raw = cf1_pulse_length;
        if (cf1_pulse_length && power_on) {
          const uint32_t volts_x10 = (energy.hlw_voltage_ratio * kHlwVoltageCalibration) / cf1_pulse_length;
          channel.voltage = static_cast<float>(volts_x10) / 10.0f;
          energy.last_success_ms = now;
        } else {
          channel.voltage = 0.0f;
        }
      } else {
        energy.hlw_cf1_current_pulse_length = cf1_pulse_length;
        channel.current_raw = cf1_pulse_length;
        if (cf1_pulse_length && channel.power > 0.0f) {
          const uint32_t milliamps = (energy.hlw_current_ratio * kHlwCurrentCalibration) / cf1_pulse_length;
          channel.current = static_cast<float>(milliamps) / 1000.0f;
          energy.last_success_ms = now;
        } else {
          channel.current = 0.0f;
        }
      }
    }
  } else {
    channel.voltage = 0.0f;
    channel.current = 0.0f;
  }

  channel.power_raw = static_cast<int32_t>(energy.hlw_cf_power_pulse_length);
  updateEnergyAggregateFromChannels();
}

uint32_t bl09xxRead24(uint8_t index) {
  return (static_cast<uint32_t>(energy.rx_buffer[index + 2]) << 16) |
         (static_cast<uint32_t>(energy.rx_buffer[index + 1]) << 8) |
         energy.rx_buffer[index];
}

int32_t bl09xxReadSigned24(uint8_t index) {
  int32_t value = static_cast<int32_t>(bl09xxRead24(index));
  if (value & 0x00800000L) value |= 0xff000000L;
  return value;
}

bool decodeBl0939Packet() {
  const uint16_t tps1 = (static_cast<uint16_t>(energy.rx_buffer[29]) << 8) | energy.rx_buffer[28];
  const bool tps_jump = energy.tps1 &&
                        (abs(static_cast<int>(tps1) - static_cast<int>(energy.tps1)) > 10);
  if (energy.rx_buffer[0] != kBl09xxPacketHeader ||
      tps_jump) {
    return false;
  }

  energy.tps1 = tps1;
  energy.temperature = ((170.0f / 448.0f) * ((static_cast<float>(tps1) / 2.0f) - 32.0f)) - 45.0f;
  energy.voltage_raw = bl09xxRead24(10);
  const float voltage = static_cast<float>(energy.voltage_raw) / static_cast<float>(kBl0939VoltageRef);

  for (uint8_t i = 0; i < kEnergyMaxChannels; i++) {
    const uint8_t current_index = i == 0 ? 4 : 7;
    const uint8_t power_index = i == 0 ? 16 : 19;
    EnergyChannelState &channel = energy.channel[i];
    channel.voltage = voltage;
    channel.current_raw = bl09xxRead24(current_index);
    channel.power_raw = bl09xxReadSigned24(power_index);
    if (channel.power_raw > static_cast<int32_t>(kBl0939PowerRef)) {
      channel.power = static_cast<float>(channel.power_raw) / static_cast<float>(kBl0939PowerRef);
      channel.current = static_cast<float>(channel.current_raw) / static_cast<float>(kBl0939CurrentRef);
    } else {
      channel.power = 0.0f;
      channel.current = 0.0f;
    }
  }

  updateEnergyAggregateFromChannels();
  energy.last_success_ms = millis();
  return true;
}

void processBl0939Serial() {
  if (!energy.present || energy.driver != kEnergyDriverBl0939) return;
  while (bl0939_serial.available()) {
    const int raw = bl0939_serial.read();
    if (raw < 0) break;
    const uint8_t value = static_cast<uint8_t>(raw);
    if (!energy.received && value == kBl09xxPacketHeader) {
      energy.received = true;
      energy.byte_counter = 0;
    }
    if (!energy.received) continue;
    energy.rx_buffer[energy.byte_counter++] = value;
    if (energy.byte_counter < kBl0939BufferSize) continue;

    uint8_t checksum = kBl09xxReadCommand | kBl0939Address;
    for (uint8_t i = 0; i < kBl0939BufferSize - 1; i++) checksum += energy.rx_buffer[i];
    checksum ^= 0xff;
    if (checksum == energy.rx_buffer[kBl0939BufferSize - 1] && decodeBl0939Packet()) {
      energy.received = false;
      energy.byte_counter = 0;
      return;
    }

    memmove(energy.rx_buffer, energy.rx_buffer + 1, kBl0939BufferSize - 1);
    energy.byte_counter = kBl0939BufferSize - 1;
    while (energy.byte_counter > 0 && energy.rx_buffer[0] != kBl09xxPacketHeader) {
      memmove(energy.rx_buffer, energy.rx_buffer + 1, energy.byte_counter - 1);
      energy.byte_counter--;
    }
    energy.received = energy.byte_counter > 0;
  }
}

void sendBl0939Init() {
  for (uint8_t i = 0; i < sizeof(kBl09xxInit) / sizeof(kBl09xxInit[0]); i++) {
    uint8_t checksum = kBl09xxWriteCommand | kBl0939Address;
    bl0939_serial.write(checksum);
    for (uint8_t j = 0; j < 4; j++) {
      checksum += kBl09xxInit[i][j];
      bl0939_serial.write(kBl09xxInit[i][j]);
    }
    bl0939_serial.write(0xff ^ checksum);
    delay(1);
  }
}

void pollBl0939(uint32_t now) {
  if (now - energy.last_poll_ms < kBl0939PollMs) return;
  energy.last_poll_ms = now;
  bl0939_serial.write(kBl09xxReadCommand | kBl0939Address);
  bl0939_serial.write(kBl09xxFullPacket);
}

void setupEnergyMonitor() {
  memset(&energy, 0, sizeof(energy));
  energy.driver = kEnergyDriverNone;
  energy.rx_pin = kInvalidPin;
  energy.tx_pin = kInvalidPin;
  energy.cf_pin = kInvalidPin;
  energy.cf1_pin = kInvalidPin;
  energy.sel_pin = kInvalidPin;
  energy.total_kwh = ukwhToKwh(energy_saved_ukwh);
  last_energy_persist_ms = millis();
  energy_persist_requested = false;

  if (digitalPinSupported(runtime_template.energy_bl0939_rx_pin) &&
      digitalPinSupported(runtime_template.energy_tx_pin)) {
    energy.present = true;
    energy.driver = kEnergyDriverBl0939;
    energy.channel_count = 2;
    energy.rx_pin = runtime_template.energy_bl0939_rx_pin;
    energy.tx_pin = runtime_template.energy_tx_pin;
    energy.last_integrate_ms = millis();
    energy.last_success_ms = millis();
    bl0939_serial.setRxBufferSize(128);
    bl0939_serial.begin(4800, SERIAL_8N1, energy.rx_pin, energy.tx_pin);
    sendBl0939Init();
    return;
  }

  if (!digitalPinSupported(runtime_template.energy_cf_pin)) {
    return;
  }

  energy.present = true;
  energy.driver = kEnergyDriverHlw8012;
  energy.channel_count = 1;
  energy.cf_pin = runtime_template.energy_cf_pin;
  energy.cf1_pin = runtime_template.energy_cf1_pin;
  energy.sel_pin = runtime_template.energy_sel_pin;
  energy.sel_inverted = runtime_template.energy_sel_inverted;
  energy.hjl = runtime_template.energy_hjl;
  energy.hlw_power_ratio = energy.hjl ? kHjlPowerRatio : kHlwPowerRatio;
  energy.hlw_voltage_ratio = energy.hjl ? kHjlVoltageRatio : kHlwVoltageRatio;
  energy.hlw_current_ratio = energy.hjl ? kHjlCurrentRatio : kHlwCurrentRatio;
  energy.hlw_voltage_on_selected = !energy.sel_inverted;
  energy.hlw_select_ui_flag = false;
  energy.hlw_load_off = true;
  energy.last_integrate_ms = millis();
  energy.last_success_ms = millis();
  energy.last_hlw_update_ms = millis();

  if (digitalPinSupported(energy.sel_pin)) {
    pinMode(energy.sel_pin, OUTPUT);
    digitalWrite(energy.sel_pin, energy.hlw_select_ui_flag ? HIGH : LOW);
  }
  if (digitalPinSupported(energy.cf1_pin)) {
    pinMode(energy.cf1_pin, INPUT_PULLUP);
    attachInterrupt(energy.cf1_pin, hlwCf1Interrupt, FALLING);
  }
  pinMode(energy.cf_pin, INPUT_PULLUP);
  attachInterrupt(energy.cf_pin, hlwCfInterrupt, FALLING);
}

void observeEnergyPowerForZeroReport() {
  if (!energy.present) {
    last_observed_energy_power = NAN;
    return;
  }
  const bool had_positive_power = !isnan(last_observed_energy_power) &&
                                  fabsf(last_observed_energy_power) > kEnergyZeroPowerThreshold &&
                                  last_observed_energy_power > 0.0f;
  const bool has_zero_power = fabsf(energy.power) <= kEnergyZeroPowerThreshold;
  if (had_positive_power && has_zero_power) scheduleMqttEnergyReport(kMqttEnergyReportReasonPowerZero);
  last_observed_energy_power = energy.power;
}

void maintainEnergy() {
  if (!energy.present) return;
  const uint32_t now = millis();
  if (energy.driver == kEnergyDriverBl0939) {
    processBl0939Serial();
    pollBl0939(now);
    if (now - energy.last_success_ms >= kBl0939StaleMs) {
      for (uint8_t i = 0; i < energy.channel_count && i < kEnergyMaxChannels; i++) {
        energy.channel[i].power = 0.0f;
        energy.channel[i].current = 0.0f;
      }
      updateEnergyAggregateFromChannels();
    }
  } else if (energy.driver == kEnergyDriverHlw8012) {
    processHlwPulseEnergy(now);
  }
  if (now - energy.last_integrate_ms >= kEnergyIntegrateMs) {
    const uint32_t elapsed = now - energy.last_integrate_ms;
    energy.last_integrate_ms = now;
    if (energy.power > 0.0f) {
      energy.total_kwh += (energy.power * static_cast<float>(elapsed)) / 3600000000.0f;
      if (kwhToUkwh(energy.total_kwh) > kEnergyTotalMaxUkwh) {
        energy.total_kwh = ukwhToKwh(kEnergyTotalMaxUkwh);
      }
    }
  }
  persistEnergyTotal(false);
  observeEnergyPowerForZeroReport();
}

bool parseUint16Input(const String &input, uint16_t min_value, uint16_t max_value, uint16_t &out) {
  if (input.length() == 0) return false;
  for (size_t i = 0; i < input.length(); i++) {
    if (input[i] < '0' || input[i] > '9') return false;
  }
  long parsed = input.toInt();
  if (parsed < min_value || parsed > max_value) return false;
  out = static_cast<uint16_t>(parsed);
  return true;
}

uint32_t decimalScale(uint8_t decimals) {
  uint32_t scale = 1;
  while (decimals--) scale *= 10;
  return scale;
}

int64_t floatToScaledDecimal(float value, uint8_t decimals) {
  if (isnan(value)) return 0;
  const float scale = static_cast<float>(decimalScale(decimals));
  const float scaled = value * scale;
  return static_cast<int64_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

void appendScaledDecimal(String &out, int64_t scaled, uint8_t decimals) {
  if (scaled < 0) {
    out += '-';
    scaled = -scaled;
  }
  const uint32_t scale = decimalScale(decimals);
  const uint64_t whole = decimals ? static_cast<uint64_t>(scaled) / scale : static_cast<uint64_t>(scaled);
  out += static_cast<unsigned long>(whole);
  if (!decimals) return;

  uint32_t fraction = static_cast<uint32_t>(static_cast<uint64_t>(scaled) % scale);
  out += '.';
  for (uint32_t pad = scale / 10; pad > 1 && fraction < pad; pad /= 10) out += '0';
  out += static_cast<unsigned long>(fraction);
}

void appendFloatDecimal(String &out, float value, uint8_t decimals) {
  appendScaledDecimal(out, floatToScaledDecimal(value, decimals), decimals);
}

bool parseUnsignedScaledDecimalInput(const String &input, uint8_t decimals, uint64_t max_scaled, uint64_t &out) {
  const uint32_t scale = decimalScale(decimals);
  uint64_t whole = 0;
  uint64_t fraction = 0;
  uint8_t fraction_digits = 0;
  bool have_digit = false;
  bool have_decimal = false;
  size_t index = 0;
  while (index < input.length() && (input[index] == ' ' || input[index] == '\t')) index++;
  for (; index < input.length(); index++) {
    const char c = input[index];
    if (c >= '0' && c <= '9') {
      have_digit = true;
      const uint8_t digit = static_cast<uint8_t>(c - '0');
      if (have_decimal) {
        if (fraction_digits >= decimals) return false;
        fraction = (fraction * 10U) + digit;
        fraction_digits++;
      } else {
        whole = (whole * 10U) + digit;
        if (whole > max_scaled / scale) return false;
      }
      continue;
    }
    if (c == '.' && !have_decimal && decimals > 0) {
      have_decimal = true;
      continue;
    }
    while (index < input.length()) {
      const char tail = input[index++];
      if (tail != ' ' && tail != '\t' && tail != '\r' && tail != '\n') return false;
    }
    break;
  }
  if (!have_digit) return false;
  while (fraction_digits++ < decimals) fraction *= 10U;
  const uint64_t scaled = (whole * scale) + fraction;
  if (scaled > max_scaled) return false;
  out = scaled;
  return true;
}

bool commandEquals(const char *p, size_t len, const char *name) {
  return name && strlen(name) == len && strncasecmp(p, name, len) == 0;
}

bool parseUint16Token(const char *p, size_t len, uint16_t min_value, uint16_t max_value, uint16_t &out) {
  if (!p || len == 0) return false;
  uint32_t value = 0;
  for (size_t i = 0; i < len; i++) {
    const char c = p[i];
    if (c < '0' || c > '9') return false;
    value = (value * 10U) + static_cast<uint32_t>(c - '0');
    if (value > max_value) return false;
  }
  if (value < min_value) return false;
  out = static_cast<uint16_t>(value);
  return true;
}

#if MYMOTA32_LIGHT_SUPPORTED
uint16_t scalePercentTo10(uint8_t percent) {
  return static_cast<uint16_t>((static_cast<uint32_t>(percent) * 1023U + 50U) / 100U);
}

uint16_t scaleRgbTo10(uint8_t value, uint8_t dimmer) {
  return static_cast<uint16_t>((static_cast<uint32_t>(value) * 1023U * dimmer + 12750U) / 25500U);
}

bool lightAvailable() {
  return runtime_template.sm2335;
}

void loadLightStateFromConfig() {
  light.power = config.light_restore_boot && config.light_power != 0;
  light.dimmer = light.power ? sanitizeLightDimmerValue(config.light_dimmer) : kLightDimmerOff;
  light.ct = sanitizeLightCtValue(config.light_ct);
  light.mode = config.light_mode == kLightModeRgb ? kLightModeRgb : kLightModeWhite;
  memcpy(light.rgb, config.light_rgb, sizeof(light.rgb));
  light.config_dirty = false;
  light.config_save_at = 0;
}

void sm2335WriteByte(uint8_t value) {
  for (int8_t bit = 7; bit >= 0; bit--) {
    digitalWrite(runtime_template.sm2335_dat_pin, (value >> bit) & 0x01);
    delayMicroseconds(kSm2335DelayUs);
    digitalWrite(runtime_template.sm2335_clk_pin, HIGH);
    delayMicroseconds(kSm2335DelayUs);
    digitalWrite(runtime_template.sm2335_clk_pin, LOW);
    delayMicroseconds(kSm2335DelayUs);
  }
  pinMode(runtime_template.sm2335_dat_pin, INPUT);
  digitalWrite(runtime_template.sm2335_clk_pin, HIGH);
  delayMicroseconds(kSm2335DelayUs);
  digitalWrite(runtime_template.sm2335_clk_pin, LOW);
  delayMicroseconds(kSm2335DelayUs);
  pinMode(runtime_template.sm2335_dat_pin, OUTPUT);
}

void sm2335Start(uint8_t addr) {
  digitalWrite(runtime_template.sm2335_dat_pin, LOW);
  delayMicroseconds(kSm2335DelayUs);
  digitalWrite(runtime_template.sm2335_clk_pin, LOW);
  delayMicroseconds(kSm2335DelayUs);
  sm2335WriteByte(addr);
}

void sm2335Stop() {
  digitalWrite(runtime_template.sm2335_clk_pin, HIGH);
  delayMicroseconds(kSm2335DelayUs);
  digitalWrite(runtime_template.sm2335_dat_pin, HIGH);
  delayMicroseconds(kSm2335DelayUs);
}

void sm2335WriteChannels(const uint16_t channels[5]) {
  bool all_zero = true;
  for (uint8_t i = 0; i < kLightChannelCount; i++) {
    if (channels[i] != 0) {
      all_zero = false;
      break;
    }
  }
  if (all_zero) {
    sm2335Start(kSm2335AddrStandby);
    for (uint8_t i = 0; i < 11; i++) sm2335WriteByte(0);
    sm2335Stop();
    return;
  }

  sm2335Start(kSm2335AddrStart5Ch);
  sm2335WriteByte(runtime_template.sm2335_current);
  for (uint8_t i = 0; i < kLightChannelCount; i++) {
    sm2335WriteByte(static_cast<uint8_t>(channels[i] >> 8));
    sm2335WriteByte(static_cast<uint8_t>(channels[i] & 0xff));
  }
  sm2335Stop();
}

void lightSm2335Channels(uint16_t out[5]) {
  uint16_t original[5] = {0, 0, 0, 0, 0};  // R,G,B,C,W before Tasmota SetOption37 25 remap.
  if (light.present && light.power && light.dimmer > 0) {
    if (light.mode == kLightModeRgb) {
      original[0] = scaleRgbTo10(light.rgb[0], light.dimmer);
      original[1] = scaleRgbTo10(light.rgb[1], light.dimmer);
      original[2] = scaleRgbTo10(light.rgb[2], light.dimmer);
    } else {
      const uint16_t brightness = scalePercentTo10(light.dimmer);
      const uint16_t ct = sanitizeLightCtValue(light.ct);
      const uint16_t range = kLightCtMax - kLightCtMin;
      const uint16_t warm = static_cast<uint16_t>(((static_cast<uint32_t>(ct - kLightCtMin) * brightness) + (range / 2U)) / range);
      original[3] = brightness - warm;
      original[4] = warm;
    }
  }
  out[0] = original[1];
  out[1] = original[0];
  out[2] = original[2];
  out[3] = original[4];
  out[4] = original[3];
}

bool lightChannelsAny(const uint16_t channels[kLightChannelCount]) {
  for (uint8_t i = 0; i < kLightChannelCount; i++) {
    if (channels[i] != 0) return true;
  }
  return false;
}

bool lightChannelsEqual(const uint16_t a[kLightChannelCount], const uint16_t b[kLightChannelCount]) {
  for (uint8_t i = 0; i < kLightChannelCount; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

void copyLightChannels(uint16_t out[kLightChannelCount], const uint16_t in[kLightChannelCount]) {
  memcpy(out, in, sizeof(uint16_t) * kLightChannelCount);
}

void writeLightChannelsImmediate(const uint16_t channels[kLightChannelCount]) {
  sm2335WriteChannels(channels);
  copyLightChannels(light.channels, channels);
  light.fade_initialized = true;
}

bool advanceLightFade(bool force) {
  if (!light.fade_running) return false;
  const uint32_t now = millis();
  if (!force && static_cast<int32_t>(now - light.fade_next_ms) < 0) return true;

  const uint32_t elapsed = now - light.fade_start_ms;
  if (elapsed >= light.fade_duration_ms) {
    light.fade_running = false;
    writeLightChannelsImmediate(light.fade_end);
    return true;
  }

  uint16_t channels[kLightChannelCount];
  for (uint8_t i = 0; i < kLightChannelCount; i++) {
    const int32_t start = light.fade_start[i];
    const int32_t delta = static_cast<int32_t>(light.fade_end[i]) - start;
    const int32_t value = start + ((delta * static_cast<int32_t>(elapsed)) / light.fade_duration_ms);
    channels[i] = static_cast<uint16_t>(value < 0 ? 0 : (value > kLightChannelMax ? kLightChannelMax : value));
  }
  sm2335WriteChannels(channels);
  copyLightChannels(light.channels, channels);
  light.fade_next_ms = now + kLightFadeStepMs;
  return true;
}

void cancelLightFade() {
  light.fade_running = false;
  light.fade_next_ms = 0;
}

void updateLightOutputs() {
  if (!light.present) return;
  uint16_t target[kLightChannelCount];
  lightSm2335Channels(target);
  if (light.fade_running) advanceLightFade(true);

  const bool power_off = !lightChannelsAny(target);
  if (!config.light_fade || !light.fade_initialized || power_off) {
    cancelLightFade();
    writeLightChannelsImmediate(target);
    return;
  }
  if (lightChannelsEqual(light.channels, target)) {
    cancelLightFade();
    return;
  }

  uint16_t max_delta = 0;
  for (uint8_t i = 0; i < kLightChannelCount; i++) {
    const uint16_t delta = light.channels[i] > target[i] ? light.channels[i] - target[i] : target[i] - light.channels[i];
    if (delta > max_delta) max_delta = delta;
  }
  if (max_delta == 0) return;

  copyLightChannels(light.fade_start, light.channels);
  copyLightChannels(light.fade_end, target);
  light.fade_start_ms = millis();
  const uint32_t base_ms = static_cast<uint32_t>(sanitizeLightSpeedValue(config.light_speed)) * 500UL;
  light.fade_duration_ms = static_cast<uint16_t>((static_cast<uint32_t>(max_delta) * base_ms) / kLightChannelMax + 1U);
  light.fade_next_ms = light.fade_start_ms;
  light.fade_running = true;
  advanceLightFade(true);
}

void scheduleLightConfigPersist() {
  if (!light.present) return;
  config.light_power = light.power ? 1 : 0;
  config.light_dimmer = light.dimmer;
  config.light_ct = light.ct;
  config.light_mode = light.mode;
  memcpy(config.light_rgb, light.rgb, sizeof(config.light_rgb));
  light.config_dirty = true;
  light.config_save_at = millis() + kLightPersistDelayMs;
}

bool persistLightConfig(bool force) {
  if (!light.config_dirty) return true;
  if (!force && static_cast<int32_t>(millis() - light.config_save_at) < 0) return true;
  return saveLightConfig();
}

void setLightPower(bool on, bool persist = true) {
  if (!light.present) return;
  const bool changed = light.power != on;
  const uint8_t target_dimmer = on ? sanitizeLightDimmerValue(config.light_on_dimmer) : kLightDimmerOff;
  const bool dimmer_changed = light.dimmer != target_dimmer;
  light.power = on;
  light.dimmer = target_dimmer;
  updateLightOutputs();
  if (changed || dimmer_changed) scheduleMqttLightPublish(kMqttLightPendingDimmer);
  if (persist && (changed || dimmer_changed)) scheduleLightConfigPersist();
}

void toggleLightPower(bool persist = true) {
  setLightPower(!light.power, persist);
}

void setLightDimmer(uint16_t dimmer, bool persist = true) {
  if (!light.present) return;
  if (dimmer == 0) {
    setLightPower(false, persist);
    return;
  }
  const uint8_t sanitized = sanitizeLightDimmerValue(dimmer);
  const bool changed = light.dimmer != sanitized || !light.power;
  light.power = true;
  light.dimmer = sanitized;
  updateLightOutputs();
  if (changed) scheduleMqttLightPublish(kMqttLightPendingDimmer);
  if (persist && changed) scheduleLightConfigPersist();
}

void setLightCt(uint16_t ct, bool persist = true) {
  if (!light.present) return;
  const uint16_t sanitized = sanitizeLightCtValue(ct);
  const bool changed = light.ct != sanitized || light.mode != kLightModeWhite;
  light.ct = sanitized;
  light.mode = kLightModeWhite;
  updateLightOutputs();
  if (changed) scheduleMqttLightPublish(kMqttLightPendingCt);
  if (persist && changed) scheduleLightConfigPersist();
}

bool parseLightColor(const char *p, size_t len, uint8_t rgb[3]) {
  if (!p || !rgb) return false;
  if (len == 6) {
    for (uint8_t i = 0; i < 3; i++) {
      const uint8_t hi = hexNibble(p[i * 2]);
      const uint8_t lo = hexNibble(p[i * 2 + 1]);
      if (hi > 0x0f || lo > 0x0f) return false;
      rgb[i] = static_cast<uint8_t>((hi << 4) | lo);
    }
    return true;
  }

  uint8_t index = 0;
  size_t start = 0;
  while (start <= len) {
    if (index >= 3) return false;
    size_t end = start;
    while (end < len && p[end] != ',') end++;
    uint16_t value = 0;
    if (!parseUint16Token(p + start, end - start, 0, 255, value)) return false;
    rgb[index++] = static_cast<uint8_t>(value);
    if (end == len) return index == 3;
    start = end + 1;
  }
  return false;
}

bool parseLightHsb(const char *p, size_t len, uint16_t &hue, uint8_t &sat, uint8_t &bri) {
  if (!p || len == 0) return false;
  uint16_t values[3]{};
  uint8_t index = 0;
  size_t start = 0;
  while (start <= len) {
    if (index >= 3) return false;
    size_t end = start;
    while (end < len && p[end] != ',') end++;
    const char *token = p + start;
    size_t token_len = end - start;
    while (token_len > 0 && (*token == ' ' || *token == '\t')) {
      token++;
      token_len--;
    }
    while (token_len > 0) {
      const char c = token[token_len - 1];
      if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
      token_len--;
    }
    uint16_t value = 0;
    if (!parseUint16Token(token, token_len, 0, 360, value)) return false;
    values[index++] = value;
    if (end == len) break;
    start = end + 1;
  }
  if (index != 3 || values[1] > 100 || values[2] > 100) return false;
  hue = values[0];
  sat = static_cast<uint8_t>(values[1]);
  bri = static_cast<uint8_t>(values[2]);
  return true;
}

void lightHsbToRgb(uint16_t hue, uint8_t sat, uint8_t rgb[3]) {
  const uint16_t h = hue >= 360 ? 0 : hue;
  const uint8_t s = static_cast<uint8_t>((static_cast<uint16_t>(sat) * 255U + 50U) / 100U);
  const uint8_t region = h / 60U;
  const uint8_t fraction = static_cast<uint8_t>(((h % 60U) * 255U + 30U) / 60U);
  const uint8_t p = 255U - s;
  const uint8_t q = 255U - static_cast<uint8_t>((static_cast<uint16_t>(s) * fraction) / 255U);
  const uint8_t t = 255U - static_cast<uint8_t>((static_cast<uint16_t>(s) * (255U - fraction)) / 255U);
  switch (region) {
    case 0: rgb[0] = 255; rgb[1] = t; rgb[2] = p; break;
    case 1: rgb[0] = q; rgb[1] = 255; rgb[2] = p; break;
    case 2: rgb[0] = p; rgb[1] = 255; rgb[2] = t; break;
    case 3: rgb[0] = p; rgb[1] = q; rgb[2] = 255; break;
    case 4: rgb[0] = t; rgb[1] = p; rgb[2] = 255; break;
    default: rgb[0] = 255; rgb[1] = p; rgb[2] = q; break;
  }
}

void lightRgbToHsb(uint16_t &hue, uint8_t &sat, uint8_t &bri) {
  const uint8_t r = light.rgb[0];
  const uint8_t g = light.rgb[1];
  const uint8_t b = light.rgb[2];
  const uint8_t max_value = max(r, max(g, b));
  const uint8_t min_value = min(r, min(g, b));
  const uint8_t delta = max_value - min_value;
  bri = light.power ? light.dimmer : 0;
  if (max_value == 0 || delta == 0) {
    hue = 0;
    sat = max_value == 0 ? 0 : static_cast<uint8_t>((static_cast<uint16_t>(delta) * 100U + (max_value / 2U)) / max_value);
    return;
  }
  sat = static_cast<uint8_t>((static_cast<uint16_t>(delta) * 100U + (max_value / 2U)) / max_value);
  int16_t h = 0;
  if (max_value == r) {
    h = static_cast<int16_t>((60 * (static_cast<int16_t>(g) - static_cast<int16_t>(b))) / delta);
  } else if (max_value == g) {
    h = static_cast<int16_t>(120 + (60 * (static_cast<int16_t>(b) - static_cast<int16_t>(r))) / delta);
  } else {
    h = static_cast<int16_t>(240 + (60 * (static_cast<int16_t>(r) - static_cast<int16_t>(g))) / delta);
  }
  if (h < 0) h += 360;
  hue = static_cast<uint16_t>(h);
}

void setLightColor(const uint8_t rgb[3], bool persist = true) {
  if (!light.present || !rgb) return;
  const bool any = rgb[0] || rgb[1] || rgb[2];
  const bool changed = light.mode != kLightModeRgb || memcmp(light.rgb, rgb, sizeof(light.rgb)) != 0 || light.power != any;
  memcpy(light.rgb, rgb, sizeof(light.rgb));
  light.mode = kLightModeRgb;
  if (any && light.dimmer == 0) light.dimmer = sanitizeLightDimmerValue(config.light_on_dimmer);
  light.power = any;
  if (!any) light.dimmer = kLightDimmerOff;
  updateLightOutputs();
  if (changed) scheduleMqttLightPublish(kMqttLightPendingColor | kMqttLightPendingDimmer);
  if (persist && changed) scheduleLightConfigPersist();
}

void setLightHsb(uint16_t hue, uint8_t sat, uint8_t bri, bool persist = true) {
  if (!light.present) return;
  uint8_t rgb[3];
  lightHsbToRgb(hue, sat, rgb);
  const uint8_t dimmer = bri == 0 ? kLightDimmerOff : sanitizeLightDimmerValue(bri);
  const bool any = dimmer > 0 && (rgb[0] || rgb[1] || rgb[2]);
  const bool changed = light.mode != kLightModeRgb ||
                       memcmp(light.rgb, rgb, sizeof(light.rgb)) != 0 ||
                       light.power != any ||
                       light.dimmer != dimmer;
  memcpy(light.rgb, rgb, sizeof(light.rgb));
  light.mode = kLightModeRgb;
  light.power = any;
  light.dimmer = dimmer;
  updateLightOutputs();
  if (changed) scheduleMqttLightPublish(kMqttLightPendingColor | kMqttLightPendingDimmer);
  if (persist && changed) scheduleLightConfigPersist();
}

void setLightFadeEnabled(bool enabled, bool persist = true) {
  if (!light.present) return;
  const uint8_t value = enabled ? 1 : 0;
  if (config.light_fade == value) return;
  config.light_fade = value;
  if (!enabled) updateLightOutputs();
  scheduleMqttLightPublish(kMqttLightPendingFade);
  if (persist) {
    light.config_dirty = true;
    light.config_save_at = millis() + kLightPersistDelayMs;
  }
}

void setLightSpeed(uint16_t speed, bool persist = true) {
  if (!light.present) return;
  const uint8_t value = sanitizeLightSpeedValue(speed);
  if (config.light_speed == value) return;
  config.light_speed = value;
  scheduleMqttLightPublish(kMqttLightPendingSpeed);
  if (persist) {
    light.config_dirty = true;
    light.config_save_at = millis() + kLightPersistDelayMs;
  }
}

void setupLightRuntime() {
  memset(&light, 0, sizeof(light));
  light.present = lightAvailable();
  loadLightStateFromConfig();
  if (!light.present) return;
  pinMode(runtime_template.sm2335_dat_pin, OUTPUT);
  pinMode(runtime_template.sm2335_clk_pin, OUTPUT);
  sm2335Stop();
  updateLightOutputs();
}

void maintainLight() {
  advanceLightFade(false);
  persistLightConfig(false);
}
#else
void setupLightRuntime() {}
void maintainLight() {}
void scheduleMqttLightPublish(uint8_t) {}
bool persistLightConfig(bool) {
  return true;
}
#endif

bool isValidMqttHost(const String &host) {
  if (host.length() > kMqttHostMaxLen) return false;
  for (size_t i = 0; i < host.length(); i++) {
    const char c = host[i];
    if (c <= ' ' || c == '/' || c == '\\') return false;
  }
  return true;
}

bool isValidMqttTopic(const String &topic) {
  if (topic.length() == 0 || topic.length() > kMqttTopicMaxLen) return false;
  for (size_t i = 0; i < topic.length(); i++) {
    const uint8_t c = static_cast<uint8_t>(topic[i]);
    if (c < 0x20 || c == 0x7f) return false;
  }
  return true;
}

bool mqttConfigured() {
  return config.mqtt_host[0] != '\0' && config.mqtt_topic[0] != '\0' && config.mqtt_port != 0;
}

const __FlashStringHelper *mqttConnectResultName(uint8_t result) {
  switch (result) {
    case kMqttConnectOk: return F("ok");
    case kMqttConnectTcpFailed: return F("tcp_failed");
    case kMqttConnectWriteFailed: return F("connect_write_failed");
    case kMqttConnectConnackTimeout: return F("connack_timeout");
    case kMqttConnectConnackRejected: return F("connack_rejected");
    case kMqttConnectSubscribeFailed: return F("subscribe_failed");
    default: return F("idle");
  }
}

bool parsePowerCommand(const char *p, size_t len, uint8_t &relay, char *response_key, size_t key_size) {
  if (len < 5 || strncasecmp(p, "power", 5) != 0) return false;
  if (len == 5) {
    relay = 0;
    strlcpy(response_key, "POWER", key_size);
    return true;
  }
  uint16_t relay_number = 0;
  for (size_t i = 5; i < len; i++) {
    const char c = p[i];
    if (c < '0' || c > '9') return false;
    relay_number = (relay_number * 10U) + static_cast<uint16_t>(c - '0');
    if (relay_number > kMaxRelays) return false;
  }
  if (relay_number == 0) return false;
  relay = static_cast<uint8_t>(relay_number - 1);
  if (relay_number == 1 && runtime_template.relay_count <= 1) {
    strlcpy(response_key, "POWER", key_size);
    return true;
  }
  if (snprintf(response_key, key_size, "POWER%u", static_cast<unsigned>(relay_number)) >= static_cast<int>(key_size)) {
    return false;
  }
  return true;
}

bool parsePowerState(const char *p, size_t len, uint8_t &state) {
  if (len == 1 && p[0] == '0') { state = kPowerStateOff; return true; }
  if (len == 1 && p[0] == '1') { state = kPowerStateOn; return true; }
  if (len == 1 && p[0] == '2') { state = kPowerStateToggle; return true; }
  if (len == 2 && (p[0] | 0x20) == 'o' && (p[1] | 0x20) == 'n') {
    state = kPowerStateOn;
    return true;
  }
  if (len == 3 && (p[0] | 0x20) == 'o' && (p[1] | 0x20) == 'f' && (p[2] | 0x20) == 'f') {
    state = kPowerStateOff;
    return true;
  }
  if (len == 6 &&
      (p[0] | 0x20) == 't' &&
      (p[1] | 0x20) == 'o' &&
      (p[2] | 0x20) == 'g' &&
      (p[3] | 0x20) == 'g' &&
      (p[4] | 0x20) == 'l' &&
      (p[5] | 0x20) == 'e') {
    state = kPowerStateToggle;
    return true;
  }
  return false;
}

void recordMqttConnectResult(uint8_t result, uint32_t started) {
  last_mqtt_connect_result = result;
  last_mqtt_connect_duration = millis() - started;
}

bool mqttReadByteUntil(uint8_t &value, uint32_t deadline_ms) {
  while (!mqtt_client.available()) {
    if (!mqtt_client.connected() || static_cast<int32_t>(millis() - deadline_ms) >= 0) {
      return false;
    }
    delay(1);
  }
  const int read_value = mqtt_client.read();
  if (read_value < 0) return false;
  value = static_cast<uint8_t>(read_value);
  last_mqtt_rx = millis();
  return true;
}

bool mqttReadBytesUntil(uint8_t *buffer, uint32_t length, uint32_t deadline_ms) {
  for (uint32_t i = 0; i < length; i++) {
    if (!mqttReadByteUntil(buffer[i], deadline_ms)) return false;
  }
  return true;
}

bool mqttSkipBytesUntil(uint32_t length, uint32_t deadline_ms) {
  uint8_t ignored = 0;
  for (uint32_t i = 0; i < length; i++) {
    if (!mqttReadByteUntil(ignored, deadline_ms)) return false;
  }
  return true;
}

bool mqttReadRemainingLengthUntil(uint32_t &length, uint32_t max_length, uint32_t deadline_ms) {
  length = 0;
  uint32_t multiplier = 1;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t encoded = 0;
    if (!mqttReadByteUntil(encoded, deadline_ms)) return false;
    length += static_cast<uint32_t>(encoded & 0x7fU) * multiplier;
    if (length > max_length) return true;
    if ((encoded & 0x80U) == 0) return true;
    multiplier *= 128U;
  }
  return false;
}

bool mqttWriteByte(uint8_t value) {
  return mqtt_client.write(&value, 1) == 1;
}

bool mqttWriteRemainingLength(uint32_t length) {
  do {
    uint8_t encoded = length % 128U;
    length /= 128U;
    if (length) encoded |= 0x80U;
    if (!mqttWriteByte(encoded)) return false;
  } while (length);
  return true;
}

bool mqttWriteString(const char *value) {
  const uint16_t len = value ? strlen(value) : 0;
  uint8_t header[2] = {
    static_cast<uint8_t>(len >> 8),
    static_cast<uint8_t>(len & 0xffU)
  };
  if (mqtt_client.write(header, sizeof(header)) != sizeof(header)) return false;
  return len == 0 || mqtt_client.write(reinterpret_cast<const uint8_t *>(value), len) == len;
}

String mqttClientId() {
  return "mymota32_" + chipIdHex();
}

String mqttCommandTopicFilter() {
  String topic;
  topic.reserve(strlen(config.mqtt_topic) + 8);
  topic += F("cmnd/");
  topic += config.mqtt_topic;
  topic += F("/#");
  return topic;
}

void mqttStop() {
  mqtt_client.stop();
  last_mqtt_io = 0;
  last_mqtt_rx = 0;
  last_mqtt_ping = 0;
  mqtt_ping_pending = false;
}

void queueMqttConnectHeal() {
  for (uint8_t i = 0; i < runtime_template.relay_count; i++) {
    if (hasPin(runtime_template.relays[i])) {
      mqtt_pending_relay_mask |= (1U << i);
    }
  }
#if MYMOTA32_LIGHT_SUPPORTED
  if (light.present) mqtt_pending_light_mask |= kMqttLightPendingAll;
#endif
}

bool mqttReadSuback(uint16_t packet_id, uint32_t deadline_ms) {
  uint8_t packet_type = 0;
  uint32_t remaining = 0;
  if (!mqttReadByteUntil(packet_type, deadline_ms)) return false;
  if (packet_type != kMqttPacketSuback) return false;
  if (!mqttReadRemainingLengthUntil(remaining, kMqttSubackMaxRemainingLength, deadline_ms)) return false;
  if (remaining < 3) return false;

  uint8_t id_bytes[2];
  if (!mqttReadBytesUntil(id_bytes, sizeof(id_bytes), deadline_ms)) return false;
  remaining -= sizeof(id_bytes);
  const uint16_t received_id = (static_cast<uint16_t>(id_bytes[0]) << 8) | id_bytes[1];
  if (received_id != packet_id) {
    if (remaining) mqttSkipBytesUntil(remaining, deadline_ms);
    return false;
  }

  uint8_t return_code = 0x80;
  if (!mqttReadByteUntil(return_code, deadline_ms)) return false;
  remaining--;
  if (remaining && !mqttSkipBytesUntil(remaining, deadline_ms)) return false;
  return return_code != 0x80;
}

bool mqttSubscribeCommandTopic() {
  const String filter = mqttCommandTopicFilter();
  if (filter.length() == 0 || filter.length() > kMqttCommandTopicMaxLen) return false;

  const uint32_t remaining_length = 2U + 2U + filter.length() + 1U;
  const bool ok = mqttWriteByte(kMqttPacketSubscribe) &&
                  mqttWriteRemainingLength(remaining_length) &&
                  mqttWriteByte(static_cast<uint8_t>(kMqttCommandPacketId >> 8)) &&
                  mqttWriteByte(static_cast<uint8_t>(kMqttCommandPacketId & 0xffU)) &&
                  mqttWriteString(filter.c_str()) &&
                  mqttWriteByte(0x00);
  if (!ok) return false;
  last_mqtt_io = millis();
  return mqttReadSuback(kMqttCommandPacketId, millis() + kMqttConnackTimeoutMs);
}

bool mqttConnect() {
  if (!mqttConfigured() || WiFi.status() != WL_CONNECTED) return false;

  const uint32_t started = millis();
  last_mqtt_connect_attempt = started;
  mqttStop();
  mqtt_client.setTimeout(kMqttConnectTimeoutMs);
  if (!mqtt_client.connect(config.mqtt_host, config.mqtt_port)) {
    recordMqttConnectResult(kMqttConnectTcpFailed, started);
    return false;
  }
  mqtt_client.setTimeout(kMqttIoTimeoutMs);

  const String client_id = mqttClientId();
  const uint32_t remaining_length = 10U + 2U + client_id.length();
  bool ok = mqttWriteByte(0x10) &&
            mqttWriteRemainingLength(remaining_length) &&
            mqttWriteString("MQTT") &&
            mqttWriteByte(0x04) &&
            mqttWriteByte(0x02) &&
            mqttWriteByte(static_cast<uint8_t>(kMqttProtocolKeepaliveSec >> 8)) &&
            mqttWriteByte(static_cast<uint8_t>(kMqttProtocolKeepaliveSec & 0xffU)) &&
            mqttWriteString(client_id.c_str());
  if (!ok) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectWriteFailed, started);
    return false;
  }
  last_mqtt_io = millis();

  uint8_t packet_type = 0;
  uint32_t remaining = 0;
  uint8_t flags = 0;
  uint8_t return_code = 0;
  const uint32_t connack_deadline = millis() + kMqttConnackTimeoutMs;
  ok = mqttReadByteUntil(packet_type, connack_deadline);
  if (!ok) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectConnackTimeout, started);
    return false;
  }
  if (packet_type != kMqttPacketConnack) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectConnackRejected, started);
    return false;
  }
  ok = mqttReadRemainingLengthUntil(remaining, kMqttConnackMaxRemainingLength, connack_deadline);
  if (!ok || remaining != 0x02) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectConnackTimeout, started);
    return false;
  }
  ok = mqttReadByteUntil(flags, connack_deadline) &&
       mqttReadByteUntil(return_code, connack_deadline);
  if (!ok) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectConnackTimeout, started);
    return false;
  }
  if (flags != 0x00 || return_code != 0x00) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectConnackRejected, started);
    return false;
  }
  last_mqtt_io = millis();
  last_mqtt_rx = last_mqtt_io;
  last_mqtt_ping = 0;
  mqtt_ping_pending = false;
  if (!mqttSubscribeCommandTopic()) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectSubscribeFailed, started);
    return false;
  }
  recordMqttConnectResult(kMqttConnectOk, started);
  queueMqttConnectHeal();
  return true;
}

bool mqttEnsureConnected() {
  if (mqtt_client.connected()) return true;
  const uint32_t now = millis();
  if (next_mqtt_reconnect && now - next_mqtt_reconnect < kMqttReconnectMs) {
    return false;
  }
  next_mqtt_reconnect = now;
  return mqttConnect();
}

bool mqttPublish(const char *topic, const char *payload) {
  if (!mqttEnsureConnected()) return false;

  const uint16_t topic_len = topic ? strlen(topic) : 0;
  const uint16_t payload_len = payload ? strlen(payload) : 0;
  if (topic_len == 0) return false;

  const uint32_t remaining_length = 2U + topic_len + payload_len;
  const bool ok = mqttWriteByte(0x30) &&
                  mqttWriteRemainingLength(remaining_length) &&
                  mqttWriteString(topic) &&
                  (payload_len == 0 || mqtt_client.write(reinterpret_cast<const uint8_t *>(payload), payload_len) == payload_len);
  if (!ok) {
    mqttStop();
    return false;
  }
  last_mqtt_io = millis();
  return true;
}

bool iBeaconCaptureSupported() {
  return MYMOTA32_IBEACON_SUPPORTED != 0;
}

void setIBeaconStatus(const char *status) {
  strlcpy(ibeacon_status, status ? status : "unknown", sizeof(ibeacon_status));
}

void updateIBeaconMqttReportRate(uint32_t now) {
  if (ibeacon_mqtt_rate_window_start == 0) {
    ibeacon_mqtt_rate_window_start = now;
    return;
  }
  const uint32_t elapsed = now - ibeacon_mqtt_rate_window_start;
  if (elapsed < 60000UL) return;
  ibeacon_mqtt_reports_per_minute = elapsed < 120000UL ? ibeacon_mqtt_rate_window_count : 0;
  ibeacon_mqtt_rate_window_count = 0;
  ibeacon_mqtt_rate_window_start = now;
}

void recordIBeaconMqttReport(uint32_t now) {
  updateIBeaconMqttReportRate(now);
  if (ibeacon_mqtt_rate_window_count < UINT16_MAX) ibeacon_mqtt_rate_window_count++;
}

uint32_t fnv1aUpdate(uint32_t hash, uint8_t value) {
  hash ^= value;
  return hash * 16777619UL;
}

bool readAdStructure(const uint8_t *payload, uint8_t payload_len, size_t &offset, uint8_t &type, const uint8_t *&data, uint8_t &data_len) {
  while (offset < payload_len && payload[offset] == 0) {
    offset++;
  }
  if (offset >= payload_len) return false;
  const uint8_t ad_len = payload[offset];
  if (ad_len == 0) return false;
  if (offset + 1U + ad_len > payload_len) return false;
  type = payload[offset + 1];
  data = &payload[offset + 2];
  data_len = ad_len - 1;
  offset += 1U + ad_len;
  return true;
}

bool parseIBeaconClimate(const uint8_t *payload, uint8_t payload_len, IBeaconClimateReading &reading) {
  reading.valid = false;
  reading.hash = 0;
  size_t offset = 0;
  uint8_t type = 0;
  const uint8_t *data = nullptr;
  uint8_t data_len = 0;
  while (readAdStructure(payload, payload_len, offset, type, data, data_len)) {
    if (type != 0x16 || data_len < 2 || data[0] != 0x1a || data[1] != 0x18) continue;
    const uint8_t *service = data + 2;
    const uint8_t service_len = data_len - 2;
    if (service_len == 13) {
      const uint8_t *r = service + 6;
      int16_t temp10 = static_cast<int16_t>((static_cast<uint16_t>(r[0]) << 8) | r[1]);
      const uint8_t hum = r[2];
      const uint8_t bat = r[3];
      if (temp10 < -100 || temp10 > 400 || hum < 1 || hum > 100 || bat < 1 || bat > 100) return false;
      uint32_t hash = fnv1aUpdate(2166136261UL, 0xa1);
      hash = fnv1aUpdate(hash, r[0]);
      hash = fnv1aUpdate(hash, r[1]);
      hash = fnv1aUpdate(hash, hum);
      hash = fnv1aUpdate(hash, bat);
      reading.valid = true;
      reading.hash = hash;
      return true;
    }
    if (service_len == 15) {
      const uint8_t *r = service + 6;
      int16_t temp100 = static_cast<int16_t>(static_cast<uint16_t>(r[0]) | (static_cast<uint16_t>(r[1]) << 8));
      const uint16_t hum100 = static_cast<uint16_t>(r[2]) | (static_cast<uint16_t>(r[3]) << 8);
      const uint8_t bat = r[6];
      if (temp100 < -1000 || temp100 > 4000 || hum100 < 100 || hum100 > 10000 || bat < 1 || bat > 100) return false;
      uint32_t hash = fnv1aUpdate(2166136261UL, 0xa2);
      hash = fnv1aUpdate(hash, r[0]);
      hash = fnv1aUpdate(hash, r[1]);
      hash = fnv1aUpdate(hash, r[2]);
      hash = fnv1aUpdate(hash, r[3]);
      hash = fnv1aUpdate(hash, bat);
      reading.valid = true;
      reading.hash = hash;
      return true;
    }
  }
  return false;
}

IBeaconBthomeReading parseIBeaconBthome(const uint8_t *payload, uint8_t payload_len) {
  IBeaconBthomeReading reading{};
  size_t offset = 0;
  uint8_t type = 0;
  const uint8_t *data = nullptr;
  uint8_t data_len = 0;
  while (readAdStructure(payload, payload_len, offset, type, data, data_len)) {
    if (type != 0x16 || data_len < 3 || data[0] != 0xd2 || data[1] != 0xfc) continue;
    const uint8_t *service = data + 2;
    const uint8_t service_len = data_len - 2;
    const uint8_t dev_info = service[0];
    if (((dev_info >> 5) & 0x7) != 2 || (dev_info & 0x01)) continue;
    uint8_t index = 1;
    while (index < service_len) {
      const uint8_t obj = service[index++];
      if (obj == 0x00) {
        if (index >= service_len) break;
        reading.has_packet_id = true;
        reading.packet_id = service[index];
        return reading;
      } else if (obj == 0x01 || obj == 0x0a || obj == 0x3a) {
        index += 1;
      } else if (obj == 0x02 || obj == 0x03 || obj == 0x0c) {
        index += 2;
      } else {
        break;
      }
    }
  }
  return reading;
}

IBeaconCacheEntry *findIBeaconCacheEntry(const char *mac, uint8_t kind) {
  for (uint8_t i = 0; i < kIBeaconCacheSize; i++) {
    if (ibeacon_cache[i].used && ibeacon_cache[i].kind == kind && strcmp(ibeacon_cache[i].mac, mac) == 0) {
      return &ibeacon_cache[i];
    }
  }
  return nullptr;
}

IBeaconCacheEntry *allocateIBeaconCacheEntry(uint32_t now) {
  IBeaconCacheEntry *oldest = &ibeacon_cache[0];
  for (uint8_t i = 0; i < kIBeaconCacheSize; i++) {
    if (!ibeacon_cache[i].used) {
      return &ibeacon_cache[i];
    }
    if (static_cast<uint32_t>(ibeacon_cache[i].sent_at - oldest->sent_at) > 0x7fffffffUL) {
      oldest = &ibeacon_cache[i];
    }
  }
  memset(oldest, 0, sizeof(*oldest));
  oldest->sent_at = now;
  return oldest;
}

void pruneIBeaconCache(uint32_t now) {
  if (last_ibeacon_prune && now - last_ibeacon_prune < kIBeaconPruneIntervalMs) return;
  last_ibeacon_prune = now;
  for (uint8_t i = 0; i < kIBeaconCacheSize; i++) {
    if (!ibeacon_cache[i].used) continue;
    const uint32_t ttl = ibeacon_cache[i].kind == kIBeaconKindClimate ? kIBeaconClimateCacheTtlMs : kIBeaconKeyfobCacheTtlMs;
    if (now - ibeacon_cache[i].sent_at > ttl) {
      memset(&ibeacon_cache[i], 0, sizeof(ibeacon_cache[i]));
    }
  }
}

bool compactIBeaconMac(const char *mac, char (&out)[13]) {
  uint8_t count = 0;
  for (const char *p = mac; p && *p; p++) {
    if (*p == ':' || *p == '-') continue;
    if (!isHexChar(*p) || count >= 12) return false;
    out[count++] = uppercaseHexChar(*p);
  }
  out[count] = '\0';
  return count == 12;
}

bool iBeaconMacInFilterList(const char *list, const char *mac) {
  if (!list || !mac || !list[0]) return false;
  char compact[13]{};
  if (!compactIBeaconMac(mac, compact)) return false;
  const char *p = list;
  while (*p) {
    while (*p == ',' || *p == ' ' || *p == '\t') p++;
    const char *start = p;
    while (*p && *p != ',') p++;
    if (static_cast<size_t>(p - start) == 12 && strncmp(start, compact, 12) == 0) return true;
    if (*p == ',') p++;
  }
  return false;
}

bool iBeaconFiltersConfigured() {
  return config.ibeacon_filter1_macs[0] != '\0' || config.ibeacon_filter2_macs[0] != '\0';
}

bool iBeaconMacAllowedByFilters(const char *mac) {
  if (!iBeaconFiltersConfigured()) return true;
  return iBeaconMacInFilterList(config.ibeacon_filter1_macs, mac) ||
         iBeaconMacInFilterList(config.ibeacon_filter2_macs, mac);
}

uint16_t iBeaconThrottleIntervalSec(const char *mac) {
  uint16_t interval = 0;
  if (iBeaconMacInFilterList(config.ibeacon_filter1_macs, mac)) {
    interval = config.ibeacon_filter1_interval_sec;
  }
  if (iBeaconMacInFilterList(config.ibeacon_filter2_macs, mac) &&
      config.ibeacon_filter2_interval_sec > interval) {
    interval = config.ibeacon_filter2_interval_sec;
  }
  return interval;
}

bool iBeaconThrottleAllows(const IBeaconObservation &obs, const IBeaconCacheEntry *entry, uint32_t now) {
  if (!entry) return true;
  const uint16_t interval_sec = iBeaconThrottleIntervalSec(obs.mac);
  if (interval_sec == 0) return true;
  return now - entry->sent_at >= static_cast<uint32_t>(interval_sec) * 1000UL;
}

bool shouldPublishIBeacon(const IBeaconObservation &obs, const IBeaconClimateReading &climate, const IBeaconBthomeReading &bthome, uint32_t now) {
  if (!iBeaconMacAllowedByFilters(obs.mac)) {
    return false;
  }

  const uint8_t kind = climate.valid ? kIBeaconKindClimate : kIBeaconKindKeyfob;
  IBeaconCacheEntry *entry = findIBeaconCacheEntry(obs.mac, kind);
  if (!entry) {
    return true;
  }
  if (!iBeaconThrottleAllows(obs, entry, now)) {
    return false;
  }
  if (kind == kIBeaconKindClimate) {
    return entry->climate_hash != climate.hash;
  }
  if (bthome.has_packet_id && (!entry->has_packet_id || entry->packet_id != bthome.packet_id)) {
    return true;
  }
  if (entry->rssi != obs.rssi) {
    return true;
  }
  return false;
}

void rememberPublishedIBeacon(const IBeaconObservation &obs, const IBeaconClimateReading &climate, const IBeaconBthomeReading &bthome, uint32_t now) {
  const uint8_t kind = climate.valid ? kIBeaconKindClimate : kIBeaconKindKeyfob;
  IBeaconCacheEntry *entry = findIBeaconCacheEntry(obs.mac, kind);
  if (!entry) {
    entry = allocateIBeaconCacheEntry(now);
  }
  entry->used = true;
  strlcpy(entry->mac, obs.mac, sizeof(entry->mac));
  entry->kind = kind;
  entry->rssi = obs.rssi;
  entry->climate_hash = climate.valid ? climate.hash : 0;
  entry->has_packet_id = bthome.has_packet_id;
  entry->packet_id = bthome.packet_id;
  entry->sent_at = now;
}

void bytesToHex(const uint8_t *data, uint8_t len, char *out, size_t out_size) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  if (out_size == 0) return;
  size_t pos = 0;
  for (uint8_t i = 0; i < len && pos + 2 < out_size; i++) {
    out[pos++] = kHex[(data[i] >> 4) & 0x0f];
    out[pos++] = kHex[data[i] & 0x0f];
  }
  out[pos] = '\0';
}

bool mqttPublishIBeacon(const IBeaconObservation &obs) {
  if (!mqttConfigured()) return false;
  char packet_hex[(kIBeaconMaxPacketBytes * 2) + 1];
  bytesToHex(obs.payload, obs.payload_len, packet_hex, sizeof(packet_hex));

  String topic;
  topic.reserve(strlen(config.mqtt_topic) + 14);
  topic += F("tele/");
  topic += config.mqtt_topic;
  topic += F("/SENSOR");

  String payload;
  payload.reserve(100 + strlen(packet_hex));
  payload += F("{\"IBEACON\":{\"MAC\":\"");
  payload += obs.mac;
  payload += F("\",\"RSSI\":");
  payload += String(static_cast<int>(obs.rssi));
  if (obs.payload_len > 0) {
    payload += F(",\"PACKET\":\"");
    payload += packet_hex;
    payload += F("\"");
  }
  payload += F("}}");

  return mqttPublish(topic.c_str(), payload.c_str());
}

#if MYMOTA32_IBEACON_SUPPORTED
void normalizeIBeaconMac(char *mac) {
  for (char *p = mac; *p; p++) {
    if (*p >= 'a' && *p <= 'f') *p = static_cast<char>(*p - 'a' + 'A');
  }
}

void pushIBeaconObservation(const IBeaconObservation &obs) {
  portENTER_CRITICAL(&ibeacon_queue_mux);
  if (ibeacon_queue_count >= kIBeaconQueueDepth) {
    ibeacon_queue_head = (ibeacon_queue_head + 1) % kIBeaconQueueDepth;
    ibeacon_queue_count--;
  }
  const uint8_t index = (ibeacon_queue_head + ibeacon_queue_count) % kIBeaconQueueDepth;
  ibeacon_queue[index] = obs;
  ibeacon_queue_count++;
  portEXIT_CRITICAL(&ibeacon_queue_mux);
}

bool popIBeaconObservation(IBeaconObservation &obs) {
  bool have = false;
  portENTER_CRITICAL(&ibeacon_queue_mux);
  if (ibeacon_queue_count > 0) {
    obs = ibeacon_queue[ibeacon_queue_head];
    ibeacon_queue_head = (ibeacon_queue_head + 1) % kIBeaconQueueDepth;
    ibeacon_queue_count--;
    have = true;
  }
  portEXIT_CRITICAL(&ibeacon_queue_mux);
  return have;
}

void resetIBeaconObservationQueue() {
  portENTER_CRITICAL(&ibeacon_queue_mux);
  ibeacon_queue_head = 0;
  ibeacon_queue_count = 0;
  portEXIT_CRITICAL(&ibeacon_queue_mux);
}

void IBeaconScanCallbacks::onResult(const NimBLEAdvertisedDevice *device) {
  if (!device || !config.ibeacon_enabled) return;
  IBeaconObservation obs{};
  std::string mac = device->getAddress().toString();
  strlcpy(obs.mac, mac.c_str(), sizeof(obs.mac));
  normalizeIBeaconMac(obs.mac);
  if (!iBeaconMacAllowedByFilters(obs.mac)) return;
  obs.rssi = device->getRSSI();
  const std::vector<uint8_t> &payload = device->getPayload();
  obs.payload_len = static_cast<uint8_t>(min(payload.size(), static_cast<size_t>(kIBeaconMaxPacketBytes)));
  if (obs.payload_len > 0) {
    memcpy(obs.payload, payload.data(), obs.payload_len);
  }
  obs.seen_at = millis();
  pushIBeaconObservation(obs);
}
#endif

void resetIBeaconRuntimeState() {
  memset(ibeacon_cache, 0, sizeof(ibeacon_cache));
  last_ibeacon_prune = 0;
  ibeacon_mqtt_rate_window_start = millis();
  ibeacon_mqtt_rate_window_count = 0;
  ibeacon_mqtt_reports_per_minute = 0;
#if MYMOTA32_IBEACON_SUPPORTED
  resetIBeaconObservationQueue();
#endif
}

void stopIBeaconCapture() {
#if MYMOTA32_IBEACON_SUPPORTED
  if (ibeacon_scan && ibeacon_scan->isScanning()) {
    ibeacon_scan->stop();
  }
#endif
  ibeacon_scanning = false;
  setIBeaconStatus(config.ibeacon_enabled ? "stopped" : "disabled");
}

void startIBeaconCapture() {
  if (!config.ibeacon_enabled) {
    stopIBeaconCapture();
    return;
  }
  if (!iBeaconCaptureSupported()) {
    ibeacon_scanning = false;
    setIBeaconStatus("unsupported");
    return;
  }
#if MYMOTA32_IBEACON_SUPPORTED
  const uint32_t now = millis();
  if (next_ibeacon_start_attempt && now - next_ibeacon_start_attempt < 30000UL) return;
  if (!ibeacon_stack_started) {
    next_ibeacon_start_attempt = now;
    setIBeaconStatus("starting");
    if (!NimBLEDevice::init("mymota32")) {
      setIBeaconStatus("init_failed");
      return;
    }
    ibeacon_stack_started = true;
    ibeacon_scan = NimBLEDevice::getScan();
    if (!ibeacon_scan) {
      setIBeaconStatus("scan_missing");
      return;
    }
    ibeacon_scan->setScanCallbacks(&ibeacon_scan_callbacks, true);
    ibeacon_scan->setActiveScan(false);
    ibeacon_scan->setInterval(kIBeaconScanIntervalMs);
    ibeacon_scan->setWindow(kIBeaconScanWindowMs);
    ibeacon_scan->setMaxResults(0);
  }
  if (!ibeacon_scan) {
    ibeacon_scanning = false;
    setIBeaconStatus("scan_missing");
    return;
  }
  if (!ibeacon_scan->isScanning()) {
    if (!ibeacon_scan->start(0, true, false)) {
      ibeacon_scanning = false;
      setIBeaconStatus("scan_failed");
      return;
    }
  }
  next_ibeacon_start_attempt = 0;
  ibeacon_scanning = true;
  setIBeaconStatus("scanning");
#endif
}

void processIBeaconObservation(const IBeaconObservation &obs) {
  IBeaconClimateReading climate{};
  parseIBeaconClimate(obs.payload, obs.payload_len, climate);
  const IBeaconBthomeReading bthome = parseIBeaconBthome(obs.payload, obs.payload_len);
  const uint32_t now = millis();
  if (!shouldPublishIBeacon(obs, climate, bthome, now)) {
    return;
  }
  if (mqttPublishIBeacon(obs)) {
    recordIBeaconMqttReport(now);
    rememberPublishedIBeacon(obs, climate, bthome, now);
  }
}

void maintainIBeacon() {
  const uint32_t now = millis();
  updateIBeaconMqttReportRate(now);
  pruneIBeaconCache(now);
  if (!config.ibeacon_enabled) {
    if (ibeacon_scanning) stopIBeaconCapture();
    return;
  }
  startIBeaconCapture();
#if MYMOTA32_IBEACON_SUPPORTED
  IBeaconObservation obs{};
  uint8_t processed = 0;
  while (processed < kIBeaconProcessLimit && popIBeaconObservation(obs)) {
    processIBeaconObservation(obs);
    processed++;
  }
#endif
}

bool mqttPublishCommandResult(const String &payload) {
  if (payload.length() == 0) return true;
  String topic;
  topic.reserve(strlen(config.mqtt_topic) + 14);
  topic += F("stat/");
  topic += config.mqtt_topic;
  topic += F("/RESULT");
  return mqttPublish(topic.c_str(), payload.c_str());
}

#if MYMOTA32_LIGHT_SUPPORTED
void scheduleMqttLightPublish(uint8_t mask) {
  if (!light.present || !mqttConfigured()) return;
  mqtt_pending_light_mask |= mask & kMqttLightPendingAll;
}

void appendLightColorHex(String &payload) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 3; i++) {
    payload += kHex[(light.rgb[i] >> 4) & 0x0f];
    payload += kHex[light.rgb[i] & 0x0f];
  }
}

bool mqttPublishLightState(uint8_t mask) {
  if (!light.present) return true;
  mask &= kMqttLightPendingAll;
  if (!mask) return true;

  String payload;
  payload.reserve(76);
  payload += '{';
  bool comma = false;
  if (mask & kMqttLightPendingDimmer) {
    payload += F("\"Dimmer\":");
    payload += light.dimmer;
    comma = true;
  }
  if (mask & kMqttLightPendingCt) {
    if (comma) payload += ',';
    payload += F("\"CT\":");
    payload += light.ct;
    comma = true;
  }
  if (mask & kMqttLightPendingColor) {
    if (comma) payload += ',';
    payload += F("\"Color\":\"");
    appendLightColorHex(payload);
    payload += '"';
    comma = true;
  }
  if (mask & kMqttLightPendingFade) {
    if (comma) payload += ',';
    payload += F("\"Fade\":\"");
    payload += config.light_fade ? F("ON") : F("OFF");
    payload += '"';
    comma = true;
  }
  if (mask & kMqttLightPendingSpeed) {
    if (comma) payload += ',';
    payload += F("\"Speed\":");
    payload += config.light_speed;
  }
  payload += '}';

  const bool ok = mqttPublishCommandResult(payload);
  if (ok) last_mqtt_state_publish = millis();
  return ok;
}
#endif

String mqttRelayTopic(uint8_t relay) {
  String topic;
  topic.reserve(strlen(config.mqtt_topic) + 16);
  topic += F("stat/");
  topic += config.mqtt_topic;
  topic += F("/");
  if (runtime_template.relay_count <= 1) {
    topic += F("POWER");
  } else {
    topic += F("POWER");
    topic += String(relay + 1);
  }
  return topic;
}

void scheduleMqttRelayPublish(uint8_t relay) {
  if (!mqttConfigured()) return;
  if (relay >= kMaxRelays) return;
  mqtt_pending_relay_mask |= (1U << relay);
}

bool mqttPublishRelayState(uint8_t relay) {
  if (relay >= runtime_template.relay_count || !hasPin(runtime_template.relays[relay])) return true;
  const String topic = mqttRelayTopic(relay);
  const bool ok = mqttPublish(topic.c_str(), relay_state[relay] ? "ON" : "OFF");
  if (ok) {
    last_mqtt_state_publish = millis();
  }
  return ok;
}

bool mqttPublishAllRelayStates() {
  bool ok = true;
  bool published = false;
  for (uint8_t i = 0; i < runtime_template.relay_count; i++) {
    if (!hasPin(runtime_template.relays[i])) continue;
    published = true;
    if (!mqttPublishRelayState(i)) {
      ok = false;
      break;
    }
  }
  if (ok && published) {
    last_mqtt_state_publish = millis();
  }
  return ok;
}

String mqttEnergyTopic() {
  String topic;
  topic.reserve(strlen(config.mqtt_topic) + 16);
  topic += F("stat/");
  topic += config.mqtt_topic;
  topic += F("/STATUS8");
  return topic;
}

bool mqttEnergyReportingEnabled() {
  return energy.present && (config.energy_mqtt_interval > 0 ||
                            config.energy_mqtt_change_percent_x10 > 0 ||
                            config.energy_mqtt_change_watts > 0);
}

bool mqttEnergyPowerChangedPercentEnough() {
  if (config.energy_mqtt_change_percent_x10 == 0) return false;
  if (isnan(last_mqtt_energy_power)) return true;
  const float delta = fabsf(energy.power - last_mqtt_energy_power);
  const float baseline = fabsf(last_mqtt_energy_power);
  if (baseline < 0.001f) return delta > 0.0f;
  return (delta * 1000.0f) >= (baseline * static_cast<float>(config.energy_mqtt_change_percent_x10));
}

bool mqttEnergyPowerChangedWattsEnough() {
  if (config.energy_mqtt_change_watts == 0) return false;
  if (isnan(last_mqtt_energy_power)) return true;
  return fabsf(energy.power - last_mqtt_energy_power) >= static_cast<float>(config.energy_mqtt_change_watts);
}

const __FlashStringHelper *mqttEnergyReportReasonName(uint8_t reason) {
  switch (reason) {
    case kMqttEnergyReportReasonInitial: return F("initial");
    case kMqttEnergyReportReasonInterval: return F("interval");
    case kMqttEnergyReportReasonPowerChangePercent: return F("power change %");
    case kMqttEnergyReportReasonIntervalPowerChangePercent: return F("interval + power change %");
    case kMqttEnergyReportReasonPowerChangeWatts: return F("power change W");
    case kMqttEnergyReportReasonIntervalPowerChangeWatts: return F("interval + power change W");
    case kMqttEnergyReportReasonPowerChangePercentWatts: return F("power change % + W");
    case kMqttEnergyReportReasonIntervalPowerChangePercentWatts: return F("interval + power change % + W");
    case kMqttEnergyReportReasonRelayOff: return F("relay off");
    case kMqttEnergyReportReasonPowerZero: return F("power zero");
    default: return F("none");
  }
}

uint8_t mqttEnergyReportReason(uint32_t now) {
  if (last_mqtt_energy_publish == 0 || isnan(last_mqtt_energy_power)) return kMqttEnergyReportReasonInitial;
  bool interval_due = false;
  if (config.energy_mqtt_interval > 0) {
    interval_due = now - last_mqtt_energy_publish >= static_cast<uint32_t>(config.energy_mqtt_interval) * 1000UL;
  }
  const bool percent_due = mqttEnergyPowerChangedPercentEnough();
  const bool watts_due = mqttEnergyPowerChangedWattsEnough();
  if (interval_due && percent_due && watts_due) return kMqttEnergyReportReasonIntervalPowerChangePercentWatts;
  if (interval_due && percent_due) return kMqttEnergyReportReasonIntervalPowerChangePercent;
  if (interval_due && watts_due) return kMqttEnergyReportReasonIntervalPowerChangeWatts;
  if (interval_due) return kMqttEnergyReportReasonInterval;
  if (percent_due && watts_due) return kMqttEnergyReportReasonPowerChangePercentWatts;
  if (percent_due) return kMqttEnergyReportReasonPowerChangePercent;
  if (watts_due) return kMqttEnergyReportReasonPowerChangeWatts;
  return kMqttEnergyReportReasonNone;
}

bool mqttPublishEnergyStatus(uint8_t reason, uint16_t zero_relay_mask = 0) {
  if (!energy.present) return true;

  float payload_power = energy.power;
  float payload_current = energy.current;
  if (zero_relay_mask) {
    if (energy.channel_count > 1) {
      for (uint8_t i = 0; i < energy.channel_count && i < kEnergyMaxChannels; i++) {
        if (!(zero_relay_mask & (1U << i))) continue;
        payload_power -= energy.channel[i].power;
        payload_current -= energy.channel[i].current;
      }
      if (payload_power < 0.0f && fabsf(payload_power) <= kEnergyZeroPowerThreshold) payload_power = 0.0f;
      if (payload_current < 0.0f && fabsf(payload_current) <= kEnergyZeroPowerThreshold) payload_current = 0.0f;
    } else if (zero_relay_mask & 0x01U) {
      payload_power = 0.0f;
      payload_current = 0.0f;
    }
  }

  String payload;
  payload.reserve(260);
  payload += F("{\"StatusSNS\":{\"ENERGY\":{\"Total\":");
  appendFloatDecimal(payload, reportedEnergyTotalKwh(), 4);
  payload += F(",\"Power\":");
  appendFloatDecimal(payload, payload_power, 2);
  payload += F(",\"Voltage\":");
  appendFloatDecimal(payload, energy.voltage, 1);
  payload += F(",\"Current\":");
  appendFloatDecimal(payload, payload_current, 3);
  if (energy.channel_count > 1) {
    for (uint8_t i = 0; i < energy.channel_count && i < kEnergyMaxChannels; i++) {
      const bool zero_channel = zero_relay_mask & (1U << i);
      payload += F(",\"Power");
      payload += String(i + 1);
      payload += F("\":");
      appendFloatDecimal(payload, zero_channel ? 0.0f : energy.channel[i].power, 2);
      payload += F(",\"Current");
      payload += String(i + 1);
      payload += F("\":");
      appendFloatDecimal(payload, zero_channel ? 0.0f : energy.channel[i].current, 3);
    }
  }
  payload += F("}}}");

  const String topic = mqttEnergyTopic();
  const bool ok = mqttPublish(topic.c_str(), payload.c_str());
  if (ok) {
    last_mqtt_energy_publish = millis();
    last_mqtt_energy_power = payload_power;
    last_mqtt_energy_report_reason = reason;
  }
  return ok;
}

void maintainMqttEnergyReports(uint32_t now) {
  if (!energy.present) {
    last_mqtt_energy_publish = 0;
    last_mqtt_energy_power = NAN;
    last_mqtt_energy_report_reason = kMqttEnergyReportReasonNone;
    mqtt_pending_energy_zero_relay_mask = 0;
    mqtt_pending_energy_report_reason = kMqttEnergyReportReasonNone;
    return;
  }
  if (mqtt_pending_energy_report_reason != kMqttEnergyReportReasonNone) {
    const uint8_t reason = mqtt_pending_energy_report_reason;
    const uint16_t zero_relay_mask = mqtt_pending_energy_zero_relay_mask;
    if (mqttPublishEnergyStatus(reason, zero_relay_mask)) {
      mqtt_pending_energy_zero_relay_mask = 0;
      mqtt_pending_energy_report_reason = kMqttEnergyReportReasonNone;
    }
    return;
  }
  if (!mqttEnergyReportingEnabled()) return;
  const uint8_t reason = mqttEnergyReportReason(now);
  if (reason != kMqttEnergyReportReasonNone) mqttPublishEnergyStatus(reason);
}

bool mqttCommandFromTopic(const char *topic, size_t topic_len, const char *&command, size_t &command_len) {
  constexpr size_t prefix_len = 5;
  if (!topic || topic_len <= prefix_len) return false;
  if (strncmp(topic, "cmnd/", prefix_len) != 0) return false;

  const size_t configured_len = strlen(config.mqtt_topic);
  if (configured_len == 0) return false;
  if (topic_len <= prefix_len + configured_len + 1) return false;
  if (memcmp(topic + prefix_len, config.mqtt_topic, configured_len) != 0) return false;
  if (topic[prefix_len + configured_len] != '/') return false;

  command = topic + prefix_len + configured_len + 1;
  command_len = topic_len - prefix_len - configured_len - 1;
  return command_len > 0;
}

void trimCommandSpan(const char *&p, size_t &len) {
  while (len > 0) {
    const char c = p[0];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    p++;
    len--;
  }
  while (len > 0) {
    const char c = p[len - 1];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    len--;
  }
}

bool parseBacklogCommand(const char *p, size_t len) {
  constexpr size_t prefix_len = 7;
  if (!p || len < prefix_len || strncasecmp(p, "backlog", prefix_len) != 0) return false;
  for (size_t i = prefix_len; i < len; i++) {
    const char c = p[i];
    if (c < '0' || c > '9') return false;
  }
  return true;
}

void stripLeadingBacklogTokens(const char *&p, size_t &len) {
  for (;;) {
    trimCommandSpan(p, len);
    size_t token_len = 0;
    while (token_len < len) {
      const char c = p[token_len];
      if (c == ' ' || c == '\t' || c == '\r' || c == '\n') break;
      token_len++;
    }
    if (token_len == 0 || !parseBacklogCommand(p, token_len)) return;
    p += token_len;
    len -= token_len;
  }
}

bool executeBacklogCommands(const char *arg, size_t arg_len, String &out, String &error) {
  const char *p = arg;
  size_t len = arg_len;
  stripLeadingBacklogTokens(p, len);
  bool ran = false;
  while (len > 0) {
    size_t segment_len = 0;
    while (segment_len < len && p[segment_len] != ';') segment_len++;
    const char *segment = p;
    size_t command_len = segment_len;
    stripLeadingBacklogTokens(segment, command_len);
    if (command_len > 0) {
      String command;
      command.reserve(command_len);
      for (size_t i = 0; i < command_len; i++) command += segment[i];
      String response;
      if (!executeCmndString(command, response, error)) return false;
      out = response;
      ran = true;
    }
    if (segment_len == len) break;
    p += segment_len + 1;
    len -= segment_len + 1;
  }
  if (!ran) {
    out = F("{\"Backlog\":\"Empty\"}");
  } else if (out.length() == 0) {
    out = F("{\"Backlog\":\"Done\"}");
  }
  return true;
}

#if MYMOTA32_LIGHT_SUPPORTED
bool parseLightDimmerCommand(const char *p, size_t len, uint8_t &index, char *response_key, size_t key_size) {
  constexpr size_t prefix_len = 6;
  if (!p || len < prefix_len || strncasecmp(p, "dimmer", prefix_len) != 0) return false;
  if (len == prefix_len) {
    index = 0;
    strlcpy(response_key, "Dimmer", key_size);
    return true;
  }
  uint16_t dimmer_index = 0;
  for (size_t i = prefix_len; i < len; i++) {
    const char c = p[i];
    if (c < '0' || c > '9') return false;
    dimmer_index = (dimmer_index * 10U) + static_cast<uint16_t>(c - '0');
    if (dimmer_index > 9) return false;
  }
  index = dimmer_index > 4 ? 1 : static_cast<uint8_t>(dimmer_index);
  if (snprintf(response_key, key_size, "Dimmer%u", static_cast<unsigned>(dimmer_index)) >= static_cast<int>(key_size)) {
    return false;
  }
  return dimmer_index > 0;
}
#endif

bool executeDeviceCommand(const char *raw, size_t cmd_len, const char *arg, size_t arg_len, String &out, String &error) {
  if (!raw || !arg || cmd_len == 0) {
    error = F("Invalid cmnd");
    return false;
  }

  while (arg_len > 0) {
    const char c = arg[0];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    arg++;
    arg_len--;
  }
  while (arg_len > 0) {
    const char c = arg[arg_len - 1];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    arg_len--;
  }

  if (parseBacklogCommand(raw, cmd_len)) {
    return executeBacklogCommands(arg, arg_len, out, error);
  }

  uint8_t relay = 0;
  char response_key[12];
  if (parsePowerCommand(raw, cmd_len, relay, response_key, sizeof(response_key))) {
    if (relay < kMaxRelays && hasPin(runtime_template.relays[relay])) {
      bool on = relay_state[relay];
      if (arg_len > 0) {
        uint8_t state = kPowerStateOff;
        if (!parsePowerState(arg, arg_len, state)) {
          error = F("Invalid power state");
          return false;
        }
        on = state == kPowerStateToggle ? !relay_state[relay] : state == kPowerStateOn;
        setRelay(relay, on);
        updateDeviceLeds(true);
      }
      out.reserve(24);
      out += F("{\"");
      out += response_key;
      out += F("\":\"");
      out += (on ? F("ON") : F("OFF"));
      out += F("\"}");
      return true;
    }
#if MYMOTA32_LIGHT_SUPPORTED
    if (relay != 0 || !light.present) {
      error = F("Invalid relay");
      return false;
    }
    bool on = light.power;
    if (arg_len > 0) {
      uint8_t state = kPowerStateOff;
      if (!parsePowerState(arg, arg_len, state)) {
        error = F("Invalid power state");
        return false;
      }
      on = state == kPowerStateToggle ? !light.power : state == kPowerStateOn;
      setLightPower(on);
    }
    out.reserve(24);
    out += F("{\"");
    out += response_key;
    out += F("\":\"");
    out += (on ? F("ON") : F("OFF"));
    out += F("\"}");
    return true;
#else
    error = F("Invalid relay");
    return false;
#endif
  }

#if MYMOTA32_LIGHT_SUPPORTED
  uint8_t dimmer_index = 0;
  if (parseLightDimmerCommand(raw, cmd_len, dimmer_index, response_key, sizeof(response_key))) {
    if (!light.present) {
      error = F("No light output is configured");
      return false;
    }
    if (arg_len > 0) {
      uint16_t dimmer = 0;
      if (!parseUint16Token(arg, arg_len, kLightDimmerOff, kLightDimmerMax, dimmer)) {
        error = F("Invalid dimmer");
        return false;
      }
      setLightDimmer(dimmer);
    }
    out.reserve(20);
    out += F("{\"");
    out += response_key;
    out += F("\":");
    out += light.dimmer;
    out += F("}");
    return true;
  }

  if (commandEquals(raw, cmd_len, "ct") || commandEquals(raw, cmd_len, "colortemperature")) {
    if (!light.present) {
      error = F("No light output is configured");
      return false;
    }
    if (arg_len > 0) {
      uint16_t ct = 0;
      if (!parseUint16Token(arg, arg_len, kLightCtMin, kLightCtMax, ct)) {
        error = F("Invalid color temperature");
        return false;
      }
      setLightCt(ct);
    }
    out.reserve(16);
    out += F("{\"CT\":");
    out += light.ct;
    out += F("}");
    return true;
  }

  if (commandEquals(raw, cmd_len, "color")) {
    if (!light.present) {
      error = F("No light output is configured");
      return false;
    }
    if (arg_len > 0) {
      uint8_t rgb[3];
      if (!parseLightColor(arg, arg_len, rgb)) {
        error = F("Invalid color");
        return false;
      }
      setLightColor(rgb);
    }
    out.reserve(24);
    out += F("{\"Color\":\"");
    appendLightColorHex(out);
    out += F("\"}");
    return true;
  }

  if (commandEquals(raw, cmd_len, "hsbcolor")) {
    if (!light.present) {
      error = F("No light output is configured");
      return false;
    }
    uint16_t hue = 0;
    uint8_t sat = 0;
    uint8_t bri = 0;
    if (arg_len > 0) {
      if (!parseLightHsb(arg, arg_len, hue, sat, bri)) {
        error = F("Invalid HSB color");
        return false;
      }
      setLightHsb(hue, sat, bri);
    } else {
      lightRgbToHsb(hue, sat, bri);
    }
    out.reserve(28);
    out += F("{\"HSBColor\":\"");
    out += hue;
    out += ',';
    out += sat;
    out += ',';
    out += bri;
    out += F("\"}");
    return true;
  }

  if (commandEquals(raw, cmd_len, "fade")) {
    if (!light.present) {
      error = F("No light output is configured");
      return false;
    }
    if (arg_len > 0) {
      uint8_t state = kPowerStateOff;
      if (!parsePowerState(arg, arg_len, state)) {
        error = F("Invalid fade state");
        return false;
      }
      const bool enabled = state == kPowerStateToggle ? !config.light_fade : state == kPowerStateOn;
      setLightFadeEnabled(enabled);
    }
    out.reserve(16);
    out += F("{\"Fade\":\"");
    out += config.light_fade ? F("ON") : F("OFF");
    out += F("\"}");
    return true;
  }

  if (commandEquals(raw, cmd_len, "speed")) {
    if (!light.present) {
      error = F("No light output is configured");
      return false;
    }
    if (arg_len > 0) {
      uint16_t speed = 0;
      if (!parseUint16Token(arg, arg_len, kLightSpeedMin, kLightSpeedMax, speed)) {
        error = F("Invalid speed");
        return false;
      }
      setLightSpeed(speed);
    }
    out.reserve(16);
    out += F("{\"Speed\":");
    out += config.light_speed;
    out += F("}");
    return true;
  }
#endif

  error = F("Unsupported command");
  return false;
}

bool mqttSendPuback(uint16_t packet_id) {
  const bool ok = mqttWriteByte(kMqttPacketPuback) &&
                  mqttWriteByte(0x02) &&
                  mqttWriteByte(static_cast<uint8_t>(packet_id >> 8)) &&
                  mqttWriteByte(static_cast<uint8_t>(packet_id & 0xffU));
  if (ok) {
    last_mqtt_io = millis();
  }
  return ok;
}

bool mqttProcessPublish(uint8_t packet_type, uint32_t remaining, uint32_t deadline) {
  const uint8_t qos = (packet_type >> 1) & 0x03U;
  if (qos == 3 || remaining < 2) return false;
  if (remaining > kMqttInboundMaxRemainingLength) {
    return mqttSkipBytesUntil(remaining, deadline);
  }

  uint8_t topic_len_bytes[2];
  if (!mqttReadBytesUntil(topic_len_bytes, sizeof(topic_len_bytes), deadline)) return false;
  remaining -= sizeof(topic_len_bytes);
  const uint16_t topic_len = (static_cast<uint16_t>(topic_len_bytes[0]) << 8) | topic_len_bytes[1];
  if (topic_len == 0 || topic_len > remaining) return false;
  if (topic_len > kMqttInboundTopicMaxLen) {
    return mqttSkipBytesUntil(remaining, deadline);
  }

  char topic[kMqttInboundTopicMaxLen + 1];
  if (!mqttReadBytesUntil(reinterpret_cast<uint8_t *>(topic), topic_len, deadline)) return false;
  topic[topic_len] = '\0';
  remaining -= topic_len;

  uint16_t packet_id = 0;
  if (qos > 0) {
    if (remaining < 2) return false;
    uint8_t id_bytes[2];
    if (!mqttReadBytesUntil(id_bytes, sizeof(id_bytes), deadline)) return false;
    remaining -= sizeof(id_bytes);
    packet_id = (static_cast<uint16_t>(id_bytes[0]) << 8) | id_bytes[1];
  }

  if (qos > 1 || remaining > kMqttInboundPayloadMaxLen) {
    if (!mqttSkipBytesUntil(remaining, deadline)) return false;
    if (qos == 1 && !mqttSendPuback(packet_id)) return false;
    return true;
  }

  char payload[kMqttInboundPayloadMaxLen + 1];
  if (remaining && !mqttReadBytesUntil(reinterpret_cast<uint8_t *>(payload), remaining, deadline)) return false;
  payload[remaining] = '\0';

  if (qos == 1 && !mqttSendPuback(packet_id)) return false;

  const char *command = nullptr;
  size_t command_len = 0;
  if (!mqttCommandFromTopic(topic, topic_len, command, command_len)) return true;

  String response;
  String error;
  if (!executeDeviceCommand(command, command_len, payload, remaining, response, error)) {
    return true;
  }
  return mqttPublishCommandResult(response);
}

bool mqttProcessInboundPacket() {
  uint8_t packet_type = 0;
  uint32_t remaining = 0;
  const uint32_t deadline = millis() + kMqttInboundReadTimeoutMs;

  if (!mqttReadByteUntil(packet_type, deadline)) return false;
  if (!mqttReadRemainingLengthUntil(remaining, kMqttInboundMaxRemainingLength, deadline)) return false;

  if (packet_type == kMqttPacketPingresp) {
    if (remaining != 0) return false;
    last_mqtt_ping = 0;
    mqtt_ping_pending = false;
    return true;
  }

  if ((packet_type & 0xf0U) == kMqttPacketPublish) {
    return mqttProcessPublish(packet_type, remaining, deadline);
  }

  return mqttSkipBytesUntil(remaining, deadline);
}

bool mqttProcessInbound() {
  uint8_t packet_count = 0;
  while (mqtt_client.available() && packet_count < kMqttInboundPacketLimit) {
    if (!mqttProcessInboundPacket()) {
      mqttStop();
      return false;
    }
    packet_count++;
  }
  return true;
}

void maintainMqtt() {
  if (!mqttConfigured() || WiFi.status() != WL_CONNECTED) {
    if (mqtt_client.connected()) mqttStop();
    clearMqttButtonQueue();
    return;
  }

  uint32_t now_pre = millis();
  expireMqttButtonQueue(now_pre);

  if (!mqttEnsureConnected()) return;

  if (!mqttProcessInbound()) return;

  uint32_t now = millis();
  if ((last_mqtt_rx && now - last_mqtt_rx >= kMqttBrokerSilenceTimeoutMs) ||
      (mqtt_ping_pending && last_mqtt_ping && now - last_mqtt_ping >= kMqttBrokerSilenceTimeoutMs)) {
    mqttStop();
    return;
  }

  if (now - last_mqtt_io >= (static_cast<uint32_t>(kMqttProtocolKeepaliveSec) * 1000UL)) {
    if (mqttWriteByte(kMqttPacketPingreq) && mqttWriteByte(0x00)) {
      last_mqtt_io = now;
      last_mqtt_ping = now;
      mqtt_ping_pending = true;
    } else {
      mqttStop();
      return;
    }
  }

  for (uint8_t i = 0; i < runtime_template.relay_count; i++) {
    const uint8_t mask = 1U << i;
    if (!(mqtt_pending_relay_mask & mask)) continue;
    if (!mqttPublishRelayState(i)) return;
    mqtt_pending_relay_mask &= ~mask;
  }

  now = millis();
#if MYMOTA32_LIGHT_SUPPORTED
  if (mqtt_pending_light_mask && light.present) {
    const uint8_t mask = mqtt_pending_light_mask;
    if (!mqttPublishLightState(mask)) return;
    mqtt_pending_light_mask &= ~mask;
  }
  now = millis();
#endif

  while (mqtt_button_queue_count > 0) {
    MqttButtonPending &slot = mqtt_button_queue[mqtt_button_queue_head];
    if (mqttButtonQueueExpired(slot, now)) {
      dropMqttButtonQueueHead();
      continue;
    }
    if (!mqttPublish(slot.topic, slot.payload)) return;
    dropMqttButtonQueueHead();
  }
  now = millis();

  maintainMqttEnergyReports(now);
  now = millis();

  bool keepalive_has_state = runtime_template.relay_count > 0;
#if MYMOTA32_LIGHT_SUPPORTED
  keepalive_has_state = keepalive_has_state || light.present;
#endif
  if (config.mqtt_keepalive > 0 && keepalive_has_state) {
    const uint32_t interval_ms = static_cast<uint32_t>(config.mqtt_keepalive) * 1000UL;
    if (now - last_mqtt_state_publish >= interval_ms) {
      if (runtime_template.relay_count > 0 && !mqttPublishAllRelayStates()) return;
#if MYMOTA32_LIGHT_SUPPORTED
      if (light.present) mqttPublishLightState(kMqttLightPendingAll);
#endif
    }
  }
}

const __FlashStringHelper *updateErrorName(uint8_t err) {
  switch (err) {
    case UPDATE_ERROR_OK: return F("ok");
    case UPDATE_ERROR_WRITE: return F("flash write failed");
    case UPDATE_ERROR_ERASE: return F("flash erase failed");
    case UPDATE_ERROR_READ: return F("flash read failed");
    case UPDATE_ERROR_SPACE: return F("not enough space");
    case UPDATE_ERROR_SIZE: return F("bad size");
    case UPDATE_ERROR_STREAM: return F("stream timeout");
    case UPDATE_ERROR_MD5: return F("md5 mismatch");
    case UPDATE_ERROR_MAGIC_BYTE: return F("wrong magic byte");
    case UPDATE_ERROR_ACTIVATE: return F("activate failed");
    case UPDATE_ERROR_NO_PARTITION: return F("partition not found");
    case UPDATE_ERROR_BAD_ARGUMENT: return F("bad argument");
    case UPDATE_ERROR_ABORT: return F("aborted");
    case kUpdateErrorTargetMismatch: return F("filename target mismatch");
    default: return F("unknown");
  }
}

bool truthyUpdateVerifyArg() {
  if (!server.hasArg(F("verify"))) return true;
  String value = server.arg(F("verify"));
  value.trim();
  value.toLowerCase();
  return !(value == F("0") || value == F("false") || value == F("off") || value == F("no"));
}

bool firmwareFilenameMatchesTarget(String filename) {
  filename.toLowerCase();
  String target = F(MYMOTA32_TARGET);
  target.toLowerCase();
  return filename.indexOf(target) >= 0;
}

void appendHeader(String &page, const __FlashStringHelper *title, bool show_spinner = false) {
  (void)title;
  page += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>myMota32");
  if (config_ok && config.hostname[0] != '\0') {
    page += F(" &middot; ");
    page += htmlEscape(config.hostname);
  }
  page += F("</title><style>:root{--bg:#f6f7f9;--panel:#fff;--line:#d8dee8;--text:#17202a;--muted:#687386;--ok:#177245;--bad:#a23a36;--accent:#1f7a5f;--accent2:#205c8a}");
  page += F("*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:Arial,sans-serif;font-size:15px;line-height:1.4}");
  page += F(".top{background:#17202a;color:#fff;border-bottom:4px solid var(--accent);padding:18px 16px}.topin{max-width:1080px;margin:0 auto;display:flex;align-items:end;justify-content:space-between;gap:12px;flex-wrap:wrap}");
  page += F(".brand{font-size:28px;font-weight:700;letter-spacing:0;color:inherit;text-decoration:none}.brand span{color:#7dd3aa}.sub{color:#c7d0dc;font-size:13px}.meta{display:flex;align-items:center;gap:8px}");
  page += F(".spin{width:13px;height:13px;border:2px solid rgba(255,255,255,.35);border-top-color:#7dd3aa;border-radius:50%;opacity:.55}.spin.active{opacity:1;animation:rot .7s linear infinite}@keyframes rot{to{transform:rotate(360deg)}}main{max-width:1080px;margin:18px auto 28px;padding:0 14px}");
  page += F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px;box-shadow:0 1px 2px rgba(0,0,0,.04)}.wide{grid-column:1/-1}");
  page += F(".panel h2{font-size:17px;margin:0 0 12px}.panel h3{font-size:14px;margin:0 0 10px}.panel-title{display:flex;align-items:center;justify-content:space-between;gap:12px;margin:0 0 12px}.panel-title h2{margin:0}.kv{display:grid;grid-template-columns:minmax(110px,42%) 1fr;gap:8px 12px}.kv span,.hint{color:var(--muted)}.kv div{min-width:0}");
  page += F("code{background:#eef2f6;border:1px solid #dce3ea;border-radius:4px;padding:1px 4px;word-break:break-word}.pill{display:inline-block;border-radius:999px;padding:2px 8px;background:#eef2f6;color:#364152}.pill.ok{background:var(--ok);color:#fff}.pill.bad{background:var(--bad);color:#fff}.panel h2 .pill{font-size:13px;font-weight:400;vertical-align:1px}.ok{color:var(--ok)}.bad{color:var(--bad)}.muted{color:var(--muted)}");
  page += F(".tokens{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:8px}.tokens div{display:flex;flex-direction:column;gap:3px}.help{position:relative;margin-left:auto}.help-q{display:inline-flex;align-items:center;justify-content:center;width:24px;height:24px;border:1px solid var(--line);border-radius:50%;background:#eef2f6;color:var(--accent2);font-size:14px;font-weight:700;cursor:help}.help-box{display:none;position:absolute;right:0;top:30px;z-index:30;width:520px;max-width:calc(100vw - 48px);background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:12px;box-shadow:0 8px 24px rgba(0,0,0,.18);color:var(--text);font-size:14px;font-weight:400;line-height:1.4}.help:hover .help-box,.help:focus-within .help-box{display:block}.help-box p{margin:0 0 8px}.bb{border-top:1px solid var(--line);margin-top:12px;padding-top:12px}.ae,.me{display:none}.ae.show,.me.show{display:block}.hidden{display:none}");
  page += F("form{margin:0}.row{margin:10px 0}label{display:block;font-weight:600;color:#344054}input,button,select,textarea{font:inherit}input,select,textarea{width:100%;margin-top:4px;padding:9px;border:1px solid #b9c4d0;border-radius:6px;background:#fff}input[type=checkbox]{width:auto;margin:0 6px 0 0;padding:0;vertical-align:-1px}textarea{min-height:92px;resize:vertical}");
  page += F("button,.btn{display:inline-block;margin:4px 4px 0 0;padding:8px 12px;border:1px solid var(--accent);border-radius:6px;background:var(--accent);color:#fff;text-decoration:none;cursor:pointer}.secondary{background:#fff;color:var(--accent2);border-color:#9eb7cf}.danger{background:#fff;color:var(--bad);border-color:#d4aaa7}.inline{display:inline}.actions{display:flex;flex-wrap:wrap;gap:8px;margin-top:8px}.inline button{margin:0 4px 0 0}.list{margin:0;padding-left:18px}@media(max-width:520px){.kv{grid-template-columns:1fr}.brand{font-size:24px}}</style></head><body>");
  page += F("<header class='top'><div class='topin'><div><a class='brand' href='/'>my<span>Mota32</span></a><div class='sub'>ESP32 firmware</div></div><div class='sub meta'><span>");
  page += F(MYMOTA32_VERSION);
  page += F(" / ");
  page += F(MYMOTA32_TARGET);
  page += F("</span>");
  if (show_spinner) page += F("<span id='poll-spin' class='spin active'></span>");
  page += F("</div></div></header><main>");
}

void appendFooter(String &page, bool live_poll = true, bool reboot_wait = false) {
  page += F("<script>var ls=Date.now();function ok(){ls=Date.now();var e=document.getElementById('poll-spin');if(e)e.className='spin active';}");
  page += F("function ck(){var e=document.getElementById('poll-spin');if(e&&Date.now()-ls>5000)e.className='spin';}");
  page += F("function fh(){return fetch('/health',{cache:'no-store'}).then(function(r){if(!r.ok)throw Error();return r.json();}).then(function(d){ok();return d;});}");
  page += F("function t(i,v){var e=document.getElementById(i);if(e)e.textContent=v;}");
  page += F("function p(i,v,c){var e=document.getElementById(i);if(e){e.textContent=v;e.className=c;}}");
  page += F("function sd(e,d){if(!e)return;var q=e.querySelectorAll('input,select,textarea,button');for(var i=0;i<q.length;i++)q[i].disabled=d;}");
  page += F("function live(){fh().then(function(d){");
  page += F("t('live-heap',d.heap+' bytes');t('live-uptime',d.uptime+'s');t('live-active-phy',d.active_phy);");
  page += F("if(d.perf){t('live-loop-load',d.perf.loop_load+'%');t('live-loop-hz',d.perf.loop_hz+'/s');t('live-loop-max',Number(d.perf.loop_max_us/1000).toFixed(1)+' ms');}");
  page += F("t('live-recovery',d.recovery.fast_boot_count+'/'+d.recovery.limit);");
  page += F("p('live-wifi',d.wifi?'connected':'disconnected',d.wifi?'pill ok':'pill bad');t('live-ssid',d.wifi_ssid||'n/a');t('live-ip',d.ip||'n/a');t('live-rssi',d.rssi==null?'n/a':d.rssi+' dBm');t('live-txp',d.wifi_tx_power+' dBm');");
  page += F("p('live-mqtt',d.mqtt.enabled?(d.mqtt.connected?'connected':'disconnected'):'not configured',d.mqtt.enabled?(d.mqtt.connected?'pill ok':'pill bad'):'pill');");
  page += F("if(d.mqtt){t('live-mqtt-pending',d.mqtt.pending);t('live-mqtt-result',d.mqtt.last_connect_result);t('live-mqtt-connect-ms',d.mqtt.last_connect_ms+' ms');t('live-mqtt-attempt',d.mqtt.last_attempt_ms_ago==null?'n/a':d.mqtt.last_attempt_ms_ago+' ms ago');}");
#if MYMOTA32_LIGHT_SUPPORTED
  page += F("if(d.light){p('live-light-power',d.light.power?'on':'off',d.light.power?'pill ok':'pill bad');t('live-light-dimmer',d.light.dimmer+'%');t('live-light-ct',d.light.ct+' mired');t('live-light-color',d.light.color||'000000');t('live-light-on-dimmer',d.light.on_dimmer+'%');t('live-light-fade',d.light.fade?'on':'off');t('live-light-speed',d.light.speed);}");
#endif
  page += F("if(d.ibeacon){t('live-ibeacon-mqtt-rpm',d.ibeacon.mqtt_reports_per_minute+'/min');}");
  page += F("if(d.power){for(var i=0;i<d.power.length;i++){if(d.power[i]!==null)p('live-relay-'+i,d.power[i]?'on':'off',d.power[i]?'pill ok':'pill bad');}}");
  page += F("if(d.buttons){for(var b=0;b<d.buttons.length;b++){if(d.buttons[b])p('live-button-'+b,d.buttons[b].state||(d.buttons[b].pressed?'pressed':'released'),d.buttons[b].pressed?'pill ok':'pill bad');}}");
  page += F("if(d.leds){for(var l=0;l<d.leds.length;l++){if(d.leds[l])p('live-led-'+l,d.leds[l].on?'on':'off',d.leds[l].on?'pill ok':'pill bad');}}");
  page += F("function fmt(v,d,s){return v==null?'n/a':Number(v).toFixed(d)+s;}if(d.energy){t('live-energy-power',fmt(d.energy.power,1,' W'));t('live-energy-voltage',fmt(d.energy.voltage,1,' V'));t('live-energy-current',fmt(d.energy.current,3,' A'));t('live-energy-total',fmt(d.energy.total_kwh,4,' kWh'));t('live-energy-offset',fmt(d.energy.offset_kwh,4,' kWh'));t('live-energy-mqtt-age',d.energy.last_mqtt_report_ms_ago==null?'n/a':d.energy.last_mqtt_report_ms_ago+' ms ago');t('live-energy-mqtt-reason',d.energy.last_mqtt_report_reason||'n/a');if(d.energy.channels){for(var e=0;e<d.energy.channels.length;e++){t('live-energy-ch'+e+'-power',fmt(d.energy.channels[e].power,1,' W'));t('live-energy-ch'+e+'-current',fmt(d.energy.channels[e].current,3,' A'));}}}");
  page += F("}).catch(function(){});}");
  page += F("function ba(s){var k=s.getAttribute('data-key'),v=s.value,b=document.getElementById('extra-'+k);if(!b)return;var t=b.querySelector('.ti'),p=b.querySelector('.pi'),rr=b.querySelector('.rr'),tr=b.querySelector('.tr'),pr=b.querySelector('.pr'),tl=b.querySelector('.tl'),off=s.disabled;b.className=(v=='1'||v=='2'||v=='3')?'ae show':'ae';if(rr)rr.className=v=='1'?'row rr':'row rr hidden';if(tr)tr.className=(v=='2'||v=='3')?'row tr':'row tr hidden';if(pr)pr.className=v=='2'?'row pr':'row pr hidden';sd(rr,off||v!='1');sd(tr,off||!(v=='2'||v=='3'));sd(pr,off||v!='2');if(v=='2'){if(t&&(!t.value||t.value.indexOf('http://')==0))t.value=t.getAttribute('data-default-topic');if(p&&!p.value)p.value=p.getAttribute('data-default-payload');if(tl)tl.textContent='MQTT topic';}else if(v=='3'&&tl)tl.textContent='Webhook URL';}");
  page += F("function im(s){var k=s.getAttribute('data-input'),v=s.value,b=document.getElementById('input-button-'+k),w=document.getElementById('input-switch-'+k);if(b)b.className=v=='0'?'me show':'me';if(w)w.className=v=='1'?'me show':'me';sd(b,v!='0');sd(w,v!='1');if(b){var a=b.querySelectorAll('.ba');for(var i=0;i<a.length;i++)ba(a[i]);}}");
  page += F("function rb(s){var k=s.getAttribute('data-relay'),o=document.getElementById('relay_on_boot'+k),r=document.getElementById('relay_restore_boot'+k);if(!o||!r||!s.checked)return;if(s==r)o.checked=false;else if(s==o)r.checked=false;}");
#if MYMOTA32_LIGHT_SUPPORTED
  page += F("function lv(i){return i.type=='checkbox'?(i.checked?'1':'0'):i.value;}function lu(i){var e=i.getAttribute('data-live'),s=i.getAttribute('data-suffix')||'',v=lv(i);if(i.type=='checkbox')v=i.checked?(i.getAttribute('data-on')||'on'):(i.getAttribute('data-off')||'off');if(e)t(e,v+s);}function la(i){lu(i);var body=new URLSearchParams();body.append(i.name,lv(i));body.append('_inline','1');fetch('/light',{method:'POST',body:body,cache:'no-store'}).then(function(r){if(!r.ok)return r.text().then(function(x){throw Error(x||r.statusText)});live();}).catch(function(x){alert(x.message||x);});}");
#endif
  page += F("function ts(){var s=document.getElementById('known-template'),t=document.getElementById('template-json');if(!s||!t)return;var v=t.value.trim(),m=0;for(var i=1;i<s.options.length;i++){if(s.options[i].getAttribute('data-json')==v){m=i;break;}}s.selectedIndex=m;}");
  page += F("function tp(s){var o=s.options[s.selectedIndex],t=document.getElementById('template-json');if(o&&t&&o.getAttribute('data-json')){t.value=o.getAttribute('data-json');ts();}}");
  page += F("function vf(f){var t=(f.getAttribute('data-target')||'').toLowerCase(),i=f.querySelector('input[type=file]'),c=f.querySelector('.fv');if(!i||!t)return true;if(c&&!c.checked){i.setCustomValidity('');return true;}var n=i.files&&i.files[0]?i.files[0].name.toLowerCase():'';var o=!n||n.indexOf(t)>=0;i.setCustomValidity(o?'':'Firmware file name must include '+t);return o;}");
  page += F("function fu(f){var c=f.querySelector('.fv');f.action='/update?verify='+(!c||c.checked?'1':'0');}");
  page += F("function fw(){var a=document.querySelectorAll('.fu');for(var i=0;i<a.length;i++){(function(f){var x=f.querySelector('input[type=file]'),c=f.querySelector('.fv');fu(f);if(x)x.onchange=function(){vf(f);this.reportValidity();};if(c)c.onchange=function(){fu(f);vf(f);if(x)x.reportValidity();};f.addEventListener('submit',function(e){fu(f);if(!vf(f)){e.preventDefault();if(x)x.reportValidity();}},true);})(a[i]);}}");
#if MYMOTA32_LIGHT_SUPPORTED
  page += F("function bi(){var a=document.querySelectorAll('.ba');for(var i=0;i<a.length;i++){a[i].onchange=function(){ba(this)};ba(a[i]);}var m=document.querySelectorAll('.im');for(var j=0;j<m.length;j++){m[j].onchange=function(){im(this)};im(m[j]);}var c=document.querySelectorAll('.rbc');for(var k=0;k<c.length;k++){c[k].onchange=function(){rb(this)};rb(c[k]);}var l=document.querySelectorAll('.la');for(var n=0;n<l.length;n++){l[n].oninput=function(){lu(this)};l[n].onchange=function(){la(this)};}var t=document.getElementById('template-json');if(t){t.oninput=ts;t.onchange=ts;}ts();}bi();fw();");
#else
  page += F("function bi(){var a=document.querySelectorAll('.ba');for(var i=0;i<a.length;i++){a[i].onchange=function(){ba(this)};ba(a[i]);}var m=document.querySelectorAll('.im');for(var j=0;j<m.length;j++){m[j].onchange=function(){im(this)};im(m[j]);}var c=document.querySelectorAll('.rbc');for(var k=0;k<c.length;k++){c[k].onchange=function(){rb(this)};rb(c[k]);}var t=document.getElementById('template-json');if(t){t.oninput=ts;t.onchange=ts;}ts();}bi();fw();");
#endif
  page += F("document.addEventListener('click',function(e){var b=e.target;while(b&&b.tagName!='BUTTON'&&b.tagName!='INPUT')b=b.parentNode;if(!b||!b.form)return;var t=(b.type||'').toLowerCase();if(t=='submit'||t=='image')b.form._s=b;},true);");
  page += F("document.addEventListener('submit',function(e){var f=e.target;if(!f||f.getAttribute('data-inline')!='1')return;e.preventDefault();var fd=new FormData(f),b=e.submitter||f._s;if(b&&b.name)fd.append(b.name,b.value);fd.append('_inline','1');var body=new URLSearchParams();fd.forEach(function(v,k){body.append(k,v);});fetch(f.getAttribute('action')||location.pathname,{method:(f.method||'POST').toUpperCase(),body:body,cache:'no-store'}).then(function(r){if(!r.ok)return r.text().then(function(x){throw Error(x||r.statusText)});live();}).catch(function(x){alert(x.message||x);});},true);");
  if (live_poll) page += F("setInterval(live,1000);setInterval(ck,1000);live();");
  if (reboot_wait) {
    page += F("var rb=");
    page += String(boot_id);
    page += F(";function rw(){fh().then(function(d){if(d.boot_id==null||d.boot_id!=rb)location.href='/';else setTimeout(rw,1000);}).catch(function(){setTimeout(rw,1000);});}setTimeout(rw,4000);");
  }
  page += F("</script></main></body></html>");
}

void sendHtml(String &page) {
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("text/html"), page);
}

void __attribute__((noinline)) sendPlain(uint16_t status, const __FlashStringHelper *message) {
  server.send(status, F("text/plain"), message);
}

void __attribute__((noinline)) sendPlain(uint16_t status, const char *message) {
  server.send(status, F("text/plain"), message);
}

void __attribute__((noinline)) sendPlain(uint16_t status, const String &message) {
  server.send(status, F("text/plain"), message);
}

void sendInlineOkOrHome() {
  if (server.hasArg("_inline")) {
    sendPlain(204, "");
    return;
  }
  server.sendHeader(F("Location"), F("/"), true);
  sendPlain(303, "");
}

void beginStreamedResponse(const char *content_type) {
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, content_type, String());
}

void flushStreamChunk(String &chunk) {
  if (chunk.length() == 0) return;
  server.sendContent(chunk);
  chunk.remove(0);
  delay(0);
}

void appendMillisTenthsFromMicros(String &out, uint32_t micros_value) {
  const uint32_t tenths = (micros_value + 50) / 100;
  out += String(tenths / 10);
  out += '.';
  out += static_cast<char>('0' + (tenths % 10));
}

void appendStatusBlock(String &page) {
  page += F("<section class='panel wide'><h2>System Status</h2><div class='kv'>");
  page += F("<span>Version</span><div><code>");
  page += F(MYMOTA32_VERSION);
  page += F("</code> <code>");
  page += F(MYMOTA32_TARGET);
  page += F("</code></div><span>Chip</span><div><code>");
  page += chipDisplayName();
  page += F("</code></div><span>Hostname</span><div><code>");
  page += htmlEscape(config.hostname);
  page += F("</code></div><span>Heap</span><div><code id='live-heap'>");
  page += String(ESP.getFreeHeap());
  page += F(" bytes</code></div><span>Uptime</span><div><code id='live-uptime'>");
  page += String(millis() / 1000);
  page += F("s</code></div><span>Loop load</span><div><code id='live-loop-load'>");
  page += String(perf_last_loop_load);
  page += F("%</code> app busy</div><span>Loop rate</span><div><code id='live-loop-hz'>");
  page += String(perf_last_loop_hz);
  page += F("/s</code></div><span>Slowest loop</span><div><code id='live-loop-max'>");
  appendMillisTenthsFromMicros(page, perf_last_loop_max_us);
  page += F(" ms</code></div><span>PHY mode</span><div><code>");
  page += phyModeName(config.phy_mode);
  page += F("</code> configured <code id='live-active-phy'>");
  page += phyModeName(activePhyMode());
  page += F("</code> active</div><span>Recovery guard</span><div><code id='live-recovery'>");
  page += String(boot_recovery_count);
  page += F("/");
  page += String(kBootRecoveryLimit);
  page += F("</code> clears after <code>");
  page += String(kBootRecoveryStableMs / 1000);
  page += F("s</code>");
  if (boot_recovery_factory_reset) {
    page += F(" <span class='pill bad'>factory reset</span>");
  }
  page += F("</div>");

  if (WiFi.status() == WL_CONNECTED) {
    page += F("<span>Wi-Fi</span><div><span id='live-wifi' class='pill ok'>connected</span> <code id='live-ssid'>");
    page += htmlEscape(WiFi.SSID());
    page += F("</code></div><span>IP</span><div><code id='live-ip'>");
    page += ipToString(WiFi.localIP());
    page += F("</code></div><span>RSSI</span><div><code id='live-rssi'>");
    page += String(WiFi.RSSI());
    page += F(" dBm</code></div>");
  } else {
    page += F("<span>Wi-Fi</span><div><span id='live-wifi' class='pill bad'>disconnected</span> <code id='live-ssid'>n/a</code></div>");
    page += F("<span>IP</span><div><code id='live-ip'>n/a</code></div><span>RSSI</span><div><code id='live-rssi'>n/a</code></div>");
  }
  page += F("<span>Tx power</span><div><code id='live-txp'>");
  appendWifiTxPowerDbm(page);
  page += F(" dBm</code></div>");
  if (ap_started) {
    page += F("<span>Setup AP</span><div><code>");
    page += htmlEscape(WiFi.softAPSSID());
    page += F("</code> <span class='pill ok'>open</span> at <code>");
    page += ipToString(WiFi.softAPIP());
    page += F("</code></div>");
  }

  page += F("<span>MQTT</span><div>");
  if (config.mqtt_host[0] == '\0') {
    page += F("<span id='live-mqtt' class='pill'>not configured</span>");
  } else if (mqtt_client.connected()) {
    page += F("<span id='live-mqtt' class='pill ok'>connected</span>");
  } else {
    page += F("<span id='live-mqtt' class='pill bad'>disconnected</span>");
  }
  page += F("</div><span>MQTT iBeacon Reports / minute</span><div><code id='live-ibeacon-mqtt-rpm'>");
  page += String(ibeacon_mqtt_reports_per_minute);
  page += F("/min</code></div><span>MQTT broker</span><div>");
  if (config.mqtt_host[0] == '\0') {
    page += F("<span class='muted'>not configured</span>");
  } else {
    page += F("<code>");
    page += htmlEscape(config.mqtt_host);
    page += F(":");
    page += String(config.mqtt_port);
    page += F("</code>");
  }
  page += F("</div><span>MQTT topic</span><div><code>");
  page += htmlEscape(config.mqtt_topic);
  page += F("</code></div><span>MQTT keepalive</span><div><code>");
  if (config.mqtt_keepalive == 0) {
    page += F("disabled");
  } else {
    page += String(config.mqtt_keepalive);
    page += F("s");
  }
  page += F("</code></div><span>MQTT pending</span><div><code id='live-mqtt-pending'>");
  page += String(static_cast<unsigned>(mqtt_pending_relay_mask)
#if MYMOTA32_LIGHT_SUPPORTED
                 + static_cast<unsigned>(mqtt_pending_light_mask)
#endif
  );
  page += F("</code></div><span>MQTT last connect</span><div><code id='live-mqtt-result'>");
  page += mqttConnectResultName(last_mqtt_connect_result);
  page += F("</code> in <code id='live-mqtt-connect-ms'>");
  page += String(last_mqtt_connect_duration);
  page += F(" ms</code></div><span>MQTT last attempt</span><div><code id='live-mqtt-attempt'>");
  if (last_mqtt_connect_attempt == 0) {
    page += F("n/a");
  } else {
    page += String(millis() - last_mqtt_connect_attempt);
    page += F(" ms ago");
  }
  page += F("</code></div>");
  page += F("</div></section>");
}

void appendTemplateStatus(String &page) {
  page += F("<section class='panel'><h2>Template</h2>");
  if (!runtime_template.enabled) {
    page += F("<p class='muted'>No template configured.</p>");
  } else {
    page += F("<div class='kv'><span>Name</span><div><code>");
    page += htmlEscape(runtime_template.name);
    page += F("</code></div><span>Base</span><div><code>");
    page += String(runtime_template.base);
    page += F("</code> flag <code>");
    page += String(runtime_template.flag);
    page += F("</code></div><span>GPIO roles</span><div><code>");
    page += String(runtime_template.relay_count);
    page += F("</code> relays <code>");
    page += String(runtime_template.button_count);
    page += F("</code> inputs <code>");
    page += String(runtime_template.led_count);
    page += F("</code> LEDs</div>");
#if MYMOTA32_LIGHT_SUPPORTED
    if (light.present) {
      page += F("<span>Light</span><div><code>SM2335</code> DAT <code>");
      page += pinName(runtime_template.sm2335_dat_pin);
      page += F("</code>, CLK <code>");
      page += pinName(runtime_template.sm2335_clk_pin);
      page += F("</code></div>");
    } else if (runtime_template.sm2335_clk_pin != kInvalidPin || runtime_template.sm2335_dat_pin != kInvalidPin) {
      page += F("<span>Light</span><div><span class='bad'>SM2335 pins invalid</span></div>");
    }
#endif
    if (runtime_template.i2c_scl_pin != kInvalidPin || runtime_template.i2c_sda_pin != kInvalidPin) {
      page += F("<span>I2C</span><div>SCL <code>");
      page += pinName(runtime_template.i2c_scl_pin);
      page += F("</code>, SDA <code>");
      page += pinName(runtime_template.i2c_sda_pin);
      page += F("</code></div>");
    }
    if (energy.present) {
      page += F("<span>Energy</span><div><code>");
      page += energyDriverName();
      if (energy.driver == kEnergyDriverBl0939) {
        page += F("</code> RX <code>");
        page += pinName(energy.rx_pin);
        page += F("</code>, TX <code>");
        page += pinName(energy.tx_pin);
      } else if (energy.driver == kEnergyDriverHlw8012) {
        page += F("</code> CF <code>");
        page += pinName(energy.cf_pin);
        page += F("</code>, CF1 <code>");
        page += pinName(energy.cf1_pin);
        page += F("</code>, SEL <code>");
        page += pinName(energy.sel_pin);
      }
      page += F("</code>, channels <code>");
      page += String(energy.channel_count);
      page += F("</code></div>");
    }
    if (hasPin(runtime_template.link_led)) {
      page += F("<span>Link LED</span><div><code>");
      page += pinName(runtime_template.link_led.pin);
      page += F("</code></div>");
    }
    page += F("</div>");
    if (runtime_template.unsupported_count) {
      page += F("<p class='bad'>Unsupported GPIO functions:");
      for (uint8_t i = 0; i < runtime_template.unsupported_count; i++) {
        page += F(" <code>");
        page += pinName(runtime_template.unsupported_pin[i]);
        page += F("=");
        page += String(runtime_template.unsupported_code[i]);
        page += F("</code>");
      }
      page += F("</p>");
    }
  }
  page += F("</section>");
}

void appendDeviceControls(String &page) {
  bool has_device = runtime_template.relay_count > 0 || energy.present;
#if MYMOTA32_LIGHT_SUPPORTED
  has_device = has_device || light.present;
#endif
  if (!runtime_template.enabled || !has_device) return;
  page += F("<section class='panel'><h2>Device</h2>");
#if MYMOTA32_LIGHT_SUPPORTED
  if (light.present) {
    page += F("<div class='row'><strong>Light</strong> <span id='live-light-power' class='pill ");
    page += light.power ? F("ok'>on") : F("bad'>off");
    page += F("</span><div class='kv'><span>Dimmer</span><div><code id='live-light-dimmer'>");
    page += String(light.dimmer);
    page += F("%</code></div><span>Color temp</span><div><code id='live-light-ct'>");
    page += String(light.ct);
    page += F(" mired</code></div><span>Color</span><div><code id='live-light-color'>");
    appendLightColorHex(page);
    page += F("</code></div><span>ON dimmer</span><div><code id='live-light-on-dimmer'>");
    page += String(config.light_on_dimmer);
    page += F("%</code></div><span>Fade</span><div><code id='live-light-fade'>");
    page += config.light_fade ? F("on") : F("off");
    page += F("</code></div><span>Speed</span><div><code id='live-light-speed'>");
    page += String(config.light_speed);
    page += F("</code></div></div>");
    page += F("<form class='inline' data-inline='1' method='post' action='/light'><span class='actions'><button name='power' value='toggle'>Toggle</button><button name='power' value='on'>On</button><button class='secondary' name='power' value='off'>Off</button></span></form>");
    page += F("<div class='row'><label>Dimmer<br><input class='la' data-live='live-light-dimmer' data-suffix='%' name='dimmer' type='range' min='0' max='100' step='1' value='");
    page += String(light.dimmer);
    page += F("'></label></div><div class='row'><label>Color temperature<br><input name='ct' type='range' min='");
    page += String(kLightCtMin);
    page += F("' max='");
    page += String(kLightCtMax);
    page += F("' step='1' value='");
    page += String(light.ct);
    page += F("' class='la' data-live='live-light-ct' data-suffix=' mired'></label></div><div class='row'><label>Color RGB<br><input class='la' data-live='live-light-color' name='color' maxlength='6' value='");
    appendLightColorHex(page);
    page += F("'></label></div><div class='row'><label>ON dimmer<br><input class='la' data-live='live-light-on-dimmer' data-suffix='%' name='on_dimmer' type='number' min='1' max='100' step='1' value='");
    page += String(config.light_on_dimmer);
    page += F("'></label></div><div class='row'><label><input class='la' data-live='live-light-fade' data-on='on' data-off='off' name='fade' type='checkbox' value='1'");
    if (config.light_fade) page += F(" checked");
    page += F(">Fade</label></div><div class='row'><label>Speed<br><input class='la' data-live='live-light-speed' name='speed' type='number' min='");
    page += String(kLightSpeedMin);
    page += F("' max='");
    page += String(kLightSpeedMax);
    page += F("' step='1' value='");
    page += String(config.light_speed);
    page += F("'></label></div></div>");
  }
#endif
  for (uint8_t i = 0; i < runtime_template.relay_count; i++) {
    if (!hasPin(runtime_template.relays[i])) continue;
    page += F("<div class='row'><strong>Relay ");
    page += String(i + 1);
    page += F("</strong> <span class='hint'>on</span> <code>");
    page += pinName(runtime_template.relays[i].pin);
    page += F("</code> ");
    page += F("<span id='live-relay-");
    page += String(i);
    page += F("' class='pill ");
    page += relay_state[i] ? F("ok'>on") : F("bad'>off");
    page += F("</span>");
    page += F("<form class='inline' data-inline='1' method='post' action='/power'><input type='hidden' name='relay' value='");
    page += String(i + 1);
    page += F("'><span class='actions'><button name='state' value='toggle'>Toggle</button><button name='state' value='on'>On</button><button class='secondary' name='state' value='off'>Off</button></span></form></div>");
  }
  if (energy.present) {
    page += F("<div class='bb'><strong>Energy</strong><div class='kv'><span>Power</span><div><code id='live-energy-power'>");
    appendFloatDecimal(page, energy.power, 1);
    page += F(" W</code></div><span>Voltage</span><div><code id='live-energy-voltage'>");
    appendFloatDecimal(page, energy.voltage, 1);
    page += F(" V</code></div><span>Current</span><div><code id='live-energy-current'>");
    appendFloatDecimal(page, energy.current, 3);
    page += F(" A</code></div><span>Total</span><div><code id='live-energy-total'>");
    appendFloatDecimal(page, reportedEnergyTotalKwh(), 4);
    page += F(" kWh</code></div><span>Total offset</span><div><code id='live-energy-offset'>");
    appendFloatDecimal(page, config.energy_total_offset_kwh, 4);
    page += F(" kWh</code></div><span>Last MQTT report</span><div><code id='live-energy-mqtt-age'>");
    if (last_mqtt_energy_publish == 0) {
      page += F("n/a");
    } else {
      page += String(millis() - last_mqtt_energy_publish);
      page += F(" ms ago");
    }
    page += F("</code></div><span>MQTT report reason</span><div><code id='live-energy-mqtt-reason'>");
    page += mqttEnergyReportReasonName(last_mqtt_energy_report_reason);
    page += F("</code></div></div>");
    if (energy.channel_count > 1) {
      page += F("<div class='kv'>");
      for (uint8_t i = 0; i < energy.channel_count && i < kEnergyMaxChannels; i++) {
        page += F("<span>Channel ");
        page += String(i + 1);
        page += F("</span><div><code id='live-energy-ch");
        page += String(i);
        page += F("-power'>");
        appendFloatDecimal(page, energy.channel[i].power, 1);
        page += F(" W</code> <code id='live-energy-ch");
        page += String(i);
        page += F("-current'>");
        appendFloatDecimal(page, energy.channel[i].current, 3);
        page += F(" A</code></div>");
      }
      page += F("</div>");
    }
    page += F("<form data-inline='1' method='post' action='/energy'><div class='row'><label>Total kWh offset<br><input name='total_offset_kwh' type='number' min='");
    appendFloatDecimal(page, kEnergyTotalOffsetMinKwh, 0);
    page += F("' max='");
    appendFloatDecimal(page, kEnergyTotalOffsetMaxKwh, 0);
    page += F("' step='0.0001' value='");
    appendFloatDecimal(page, config.energy_total_offset_kwh, 4);
    page += F("'></label></div><div class='row'><label>MQTT report interval seconds<br><input name='energy_report_interval' type='number' min='0' max='");
    page += String(kMqttEnergyIntervalMax);
    page += F("' step='1' value='");
    page += String(config.energy_mqtt_interval);
    page += F("'></label></div><div class='row'><label>MQTT report power change percent<br><input name='energy_report_change_percent' type='number' min='0' max='");
    appendFloatDecimal(page, kMqttEnergyChangeMaxPercent, 1);
    page += F("' step='0.1' value='");
    appendScaledDecimal(page, config.energy_mqtt_change_percent_x10, 1);
    page += F("'></label></div><div class='row'><label>MQTT report power change watts<br><input name='energy_report_change_watts' type='number' min='0' max='");
    page += String(kMqttEnergyChangeMaxWatts);
    page += F("' step='1' value='");
    page += String(config.energy_mqtt_change_watts);
    page += F("'></label></div><button type='submit'>Save energy</button></form></div>");
  }
  page += F("</section>");
}

void appendLedAttachmentOption(String &page, uint8_t value, const String &label, uint8_t selected) {
  page += F("<option value='");
  page += String(value);
  page += F("'");
  if (selected == value) page += F(" selected");
  page += F(">");
  page += htmlEscape(label);
  page += F("</option>");
}

void appendLedSettings(String &page) {
  if (!runtime_template.enabled || !hasConfigurableLedOutputs()) return;
  page += F("<section class='panel'><h2>LEDs</h2><form data-inline='1' method='post' action='/leds'>");
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    const PinAssignment *assignment = ledOutputAssignment(i);
    if (!assignment || !hasLedOutput(i)) continue;
    const uint8_t selected = config.led_attach[i];
    page += F("<div class='row'><label>");
    page += htmlEscape(ledOutputName(i));
    page += F(" <span class='hint'>");
    page += pinName(assignment->pin);
    page += F("</span> <span id='live-led-");
    page += String(i);
    page += F("' class='pill ");
    page += ledOutputOn(i) ? F("ok'>on") : F("bad'>off");
    page += F("</span><br><select name='led");
    page += String(i);
    page += F("'>");
    appendLedAttachmentOption(page, kLedAttachNone, F("Nothing"), selected);
    for (uint8_t relay = 0; relay < runtime_template.relay_count; relay++) {
      if (!hasPin(runtime_template.relays[relay])) continue;
      appendLedAttachmentOption(page, kLedAttachRelayBase + relay, String(F("Relay ")) + String(relay + 1), selected);
    }
    for (uint8_t button = 0; button < runtime_template.button_count; button++) {
      if (!hasPin(runtime_template.buttons[button])) continue;
      appendLedAttachmentOption(page, kLedAttachButtonBase + button, String(F("Input ")) + String(button + 1), selected);
    }
    page += F("</select></label></div>");
  }
  page += F("<button type='submit'>Save LEDs</button></form></section>");
}

bool deviceStateEnforcementAvailable() {
  if (!runtime_template.enabled) return false;
  if (hasConfigurableRelays()) return true;
#if MYMOTA32_LIGHT_SUPPORTED
  if (light.present) return true;
#endif
  return false;
}

void appendDeviceStateEnforcementSettings(String &page) {
  if (!deviceStateEnforcementAvailable()) return;

  page += F("<section class='panel'><h2>Device State Enforcement</h2><form data-inline='1' method='post' action='/relay-enforcement'>");
  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (!relayAvailable(i)) continue;
    page += F("<div class='bb'><strong>Relay ");
    page += String(i + 1);
    page += F("</strong> <span class='hint'>");
    page += pinName(runtime_template.relays[i].pin);
    page += F("</span><div class='row'><label><input class='rbc' id='relay_on_boot");
    page += String(i);
    page += F("' data-relay='");
    page += String(i);
    page += F("' type='checkbox' name='relay_on_boot");
    page += String(i);
    page += F("' value='1'");
    if (config.relay_on_boot[i]) page += F(" checked");
    page += F(">Turn on at boot</label></div><div class='row'><label><input class='rbc' id='relay_restore_boot");
    page += String(i);
    page += F("' data-relay='");
    page += String(i);
    page += F("' type='checkbox' name='relay_restore_boot");
    page += String(i);
    page += F("' value='1'");
    if (config.relay_restore_boot[i]) page += F(" checked");
    page += F(">Restore last state at boot</label></div><div class='row'><label><input type='checkbox' name='relay_time_enabled");
    page += String(i);
    page += F("' value='1'");
    if (config.relay_time_enabled[i]) page += F(" checked");
    page += F(">Restore after OFF</label><input name='relay_time_seconds");
    page += String(i);
    page += F("' type='number' min='");
    page += String(kRelayEnforcementMinSeconds);
    page += F("' max='");
    page += String(kRelayEnforcementMaxSeconds);
    page += F("' step='1' placeholder='seconds' value='");
    if (config.relay_time_seconds[i] >= kRelayEnforcementMinSeconds) {
      page += String(config.relay_time_seconds[i]);
    }
    page += F("'></div></div>");
  }
#if MYMOTA32_LIGHT_SUPPORTED
  if (light.present) {
    page += F("<div class='bb'><strong>Light</strong><div class='row'><label><input type='checkbox' name='light_restore_boot' value='1'");
    if (config.light_restore_boot) page += F(" checked");
    page += F(">Restore last state at boot</label><span class='hint'>Power, dimmer, color temperature, and color</span></div></div>");
  }
#endif
  page += F("<button type='submit'>Save device state enforcement</button></form></section>");
}

void appendRelayPulseSettings(String &page) {
  if (!runtime_template.enabled || !hasConfigurableRelays()) return;

  page += F("<section class='panel'><h2>Relay Pulsing</h2><form data-inline='1' method='post' action='/relay-pulsing'>");
  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (!relayAvailable(i)) continue;
    page += F("<div class='bb'><strong>Relay ");
    page += String(i + 1);
    page += F("</strong> <span class='hint'>");
    page += pinName(runtime_template.relays[i].pin);
    page += F("</span><div class='row'><label><input type='checkbox' name='relay_pulse_enabled");
    page += String(i);
    page += F("' value='1'");
    if (config.relay_pulse_enabled[i]) page += F(" checked");
    page += F(">Pulse when ON</label><input name='relay_pulse_seconds");
    page += String(i);
    page += F("' type='number' min='0' max='");
    page += String(kRelayPulseMaxSeconds);
    page += F("' step='1' placeholder='seconds' value='");
    if (config.relay_pulse_seconds[i] > 0) {
      page += String(config.relay_pulse_seconds[i]);
    }
    page += F("'></div></div>");
  }
  page += F("<button type='submit'>Save relay pulsing</button></form></section>");
}

void appendInputModeOption(String &page, uint8_t value, const String &label, uint8_t selected) {
  page += F("<option value='");
  page += String(value);
  page += F("'");
  if (selected == value) page += F(" selected");
  page += F(">");
  page += htmlEscape(label);
  page += F("</option>");
}

void appendInputRelayOption(String &page, uint8_t value, uint8_t selected) {
  page += F("<option value='");
  page += String(value);
  page += F("'");
  if (selected == value) page += F(" selected");
  page += F(">Relay ");
  page += String(value + 1);
  page += F("</option>");
}

void appendButtonActionOption(String &page, uint8_t value, const __FlashStringHelper *label, uint8_t selected) {
  page += F("<option value='");
  page += String(value);
  page += F("'");
  if (selected == value) page += F(" selected");
  page += F(">");
  page += label;
  page += F("</option>");
}

void appendButtonActionSelect(String &page, uint8_t button, const char *name, uint8_t selected) {
  page += F("<select class='ba' data-key='");
  page += name;
  page += String(button);
  page += F("' name='");
  page += name;
  page += String(button);
  page += F("'>");
  appendButtonActionOption(page, kButtonActionNone, F("Nothing"), selected);
  if (buttonActionAvailable(button, kButtonActionRelayToggle)) {
    appendButtonActionOption(page, kButtonActionRelayToggle, F("Relay toggle"), selected);
  }
  appendButtonActionOption(page, kButtonActionMqtt, F("MQTT broadcast"), selected);
  appendButtonActionOption(page, kButtonActionWebhook, F("Webhook exec"), selected);
  page += F("</select>");
}

void appendButtonActionExtra(String &page, uint8_t button, const char *name, bool hold) {
  uint8_t selected_relay = 0;
  const bool has_relay_target = buttonRelayTarget(button, hold, selected_relay);
  page += F("<div id='extra-");
  page += name;
  page += String(button);
  page += F("' class='ae'>");
  if (has_relay_target) {
    page += F("<div class='row rr'><label>Target relay<br><select name='");
    page += name;
    page += F("_relay");
    page += String(button);
    page += F("'>");
    for (uint8_t relay = 0; relay < runtime_template.relay_count; relay++) {
      if (!hasPin(runtime_template.relays[relay])) continue;
      appendInputRelayOption(page, relay, selected_relay);
    }
    page += F("</select></label></div>");
  } else {
    page += F("<div class='row rr'><span class='hint'>No relay available.</span></div>");
  }
  page += F("<div class='row tr'><label><span class='tl'>MQTT topic</span><br><input class='ti' name='");
  page += name;
  page += F("_target");
  page += String(button);
  page += F("' maxlength='");
  page += String(kButtonActionTargetMaxLen);
  page += F("' data-default-topic='");
  page += htmlEscape(kDefaultButtonMqttTopic);
  page += F("' value='");
  page += htmlEscape(buttonActionTarget(button, hold));
  page += F("'></label></div><div class='row pr'><label>MQTT payload<br><textarea class='pi' name='");
  page += name;
  page += F("_payload");
  page += String(button);
  page += F("' maxlength='");
  page += String(kButtonActionPayloadMaxLen);
  page += F("' data-default-payload='");
  page += htmlEscape(hold ? kDefaultButtonMqttHoldPayload : kDefaultButtonMqttPressPayload);
  page += F("'>");
  page += htmlEscape(buttonActionPayload(button, hold));
  page += F("</textarea></label></div></div>");
}

bool inputCanFollowOutput(uint8_t input) {
  uint8_t relay = 0;
  return defaultButtonRelayTarget(input, relay);
}

String inputDisplayName(uint8_t input) {
  String name = isSwitchInput(input) ? F("Switch ") : F("Button ");
  name += String(inputFunctionIndex(input) + 1);
  return name;
}

String inputKindName(uint8_t input) {
  return isSwitchInput(input) ? F("switch") : F("button");
}

String inputStateName(uint8_t input, bool active) {
  if (effectiveInputMode(input) == kInputModeSwitch) return active ? F("on") : F("off");
  return active ? F("pressed") : F("released");
}

void appendButtonSettings(String &page) {
  if (!runtime_template.enabled || !hasConfigurableButtons()) return;

  page += F("<section class='panel'><div class='panel-title'><h2>Inputs</h2><div class='help' tabindex='0'><span class='help-q'>?</span><div class='help-box'><p><strong>Action placeholders</strong></p><div class='tokens'>");
  page += F("<div><code>{BUTTONID}</code><span class='hint'>input number, starting at 1</span></div>");
  page += F("<div><code>{TYPE}</code><span class='hint'>TOGGLE on press, HOLD on hold</span></div>");
  page += F("<div><code>{TOPIC}</code><span class='hint'>current MQTT topic</span></div>");
  page += F("<div><code>{RELAYX_STATE}</code><span class='hint'>relay state, for example {RELAY1_STATE}</span></div>");
  page += F("</div><p class='hint'>MQTT broadcast sends a topic and payload through the configured broker.</p></div></div></div><form data-inline='1' method='post' action='/buttons'>");
  page += F("<div class='row'><label>Hold time ms<br><input name='hold_ms' type='number' min='");
  page += String(kButtonHoldMinMs);
  page += F("' max='");
  page += String(kButtonHoldMaxMs);
  page += F("' step='1' value='");
  page += String(config.button_hold_ms);
  page += F("'></label><label>Debounce ms<br><input name='debounce_ms' type='number' min='");
  page += String(kButtonDebounceMinMs);
  page += F("' max='");
  page += String(kButtonDebounceMaxMs);
  page += F("' step='1' value='");
  page += String(config.button_debounce_ms);
  page += F("'></label></div>");

  for (uint8_t i = 0; i < runtime_template.button_count; i++) {
    if (!hasPin(runtime_template.buttons[i])) continue;
    const uint8_t mode = effectiveInputMode(i);
    const uint8_t on_level = effectiveInputOnLevel(i);
    uint8_t target_relay = 0;
    inputRelayTarget(i, target_relay);
    page += F("<div class='bb'><strong>");
    page += htmlEscape(inputDisplayName(i));
    page += F("</strong> <span class='hint'>");
    page += pinName(runtime_template.buttons[i].pin);
    page += F(" ");
    page += htmlEscape(inputKindName(i));
    page += F("</span> <span id='live-button-");
    page += String(i);
    page += F("' class='pill ");
    page += button_state[i].stable_pressed ? F("ok'>") : F("bad'>");
    page += htmlEscape(inputStateName(i, button_state[i].stable_pressed));
    page += F("</span>");

    page += F("<div class='row'><label>Kind<br><select class='im' data-input='");
    page += String(i);
    page += F("' name='mode");
    page += String(i);
    page += F("'>");
    appendInputModeOption(page, kInputModeButton, F("Button actions"), mode);
    if (inputCanFollowOutput(i)) {
      appendInputModeOption(page, kInputModeSwitch, F("Switch follows output"), mode);
    }
    page += F("</select></label></div>");

    page += F("<div id='input-switch-");
    page += String(i);
    page += F("' class='me");
    if (mode == kInputModeSwitch) page += F(" show");
    page += F("'>");
    uint8_t unused_relay = 0;
    if (defaultButtonRelayTarget(i, unused_relay)) {
      page += F("<div class='row'><label>Target relay<br><select name='relay");
      page += String(i);
      page += F("'>");
      for (uint8_t relay = 0; relay < runtime_template.relay_count; relay++) {
        if (!hasPin(runtime_template.relays[relay])) continue;
        appendInputRelayOption(page, relay, target_relay);
      }
      page += F("</select></label></div>");
    }
    page += F("<div class='row'><label>Reverse<br><select name='reverse");
    page += String(i);
    page += F("'><option value='0'");
    if (on_level == kInputOnLevelHigh) page += F(" selected");
    page += F(">No, GPIO high is ON</option><option value='1'");
    if (on_level == kInputOnLevelLow) page += F(" selected");
    page += F(">Yes, GPIO low is ON</option></select></label></div></div>");

    page += F("<div id='input-button-");
    page += String(i);
    page += F("' class='me");
    if (mode == kInputModeButton) page += F(" show");
    page += F("'><div class='row'><label>Press<br>");
    appendButtonActionSelect(page, i, "press", config.button_press_action[i]);
    page += F("</label>");
    appendButtonActionExtra(page, i, "press", false);
    page += F("</div><div class='row'><label>Hold<br>");
    appendButtonActionSelect(page, i, "hold", config.button_hold_action[i]);
    page += F("</label>");
    appendButtonActionExtra(page, i, "hold", true);
    page += F("</div></div></div>");
  }
  page += F("<button type='submit'>Save inputs</button></form></section>");
}

void appendTemplateForm(String &page) {
  page += F("<section class='panel wide'><h2>Template Selection</h2><form method='post' action='/template'>");
  page += F("<div class='row'><label>Known template<br><select id='known-template' onchange='tp(this)'><option value=''>Select a template</option>");
#if CONFIG_IDF_TARGET_ESP32C3
  page += F("<option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateGenericC3RelayJson)));
  page += F("'>Generic C3 Relay</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateSwitchbotW1401400Json)));
  page += F("'>Switchbot W1401400</option>");
#else
  page += F("<option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateNousA8tJson)));
  page += F("'>NOUS A8T</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateNousB1tJson)));
  page += F("'>NOUS B1T</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateNousB3tJson)));
  page += F("'>NOUS B3T</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateShellyPlus2PmPcb019Json)));
  page += F("'>Shelly Plus 2PM PCB v0.1.9</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateShellyPlus1PmJson)));
  page += F("'>Shelly Plus 1PM</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateShellyPlus1Json)));
  page += F("'>Shelly Plus 1</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateShellyPlusI4Json)));
  page += F("'>Shelly Plus i4</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateShellyPlusPlugSJson)));
  page += F("'>Shelly Plus Plug S</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateSonoffDualR3V2Json)));
  page += F("'>Sonoff Dual R3 v2</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateSonoffMinir4Json)));
  page += F("'>Sonoff MINIR4</option>");
#endif
  page += F("</select></label></div>");
  page += F("<div class='row'><label>Tasmota ESP32 template JSON<br><textarea id='template-json' name='template' rows='6' maxlength='");
  page += String(kTemplateJsonMaxLen);
  page += F("'>");
  page += htmlEscape(currentTemplateJson());
  page += F("</textarea></label></div>");
  page += F("<button type='submit'>Save template</button> <button class='danger' type='submit' name='clear' value='1'>Clear template</button></form></section>");
}

void appendMqttForm(String &page) {
  page += F("<section class='panel'><h2>MQTT Settings</h2><form data-inline='1' method='post' action='/mqtt'>");
  page += F("<div class='row'><label>Host<br><input name='host' maxlength='");
  page += String(kMqttHostMaxLen);
  page += F("' value='");
  page += htmlEscape(config.mqtt_host);
  page += F("'></label></div><div class='row'><label>Port<br><input name='port' type='number' min='1' max='65535' value='");
  page += String(config.mqtt_port);
  page += F("'></label></div><div class='row'><label>Topic<br><input name='topic' maxlength='");
  page += String(kMqttTopicMaxLen);
  page += F("' required value='");
  page += htmlEscape(config.mqtt_topic);
  page += F("'></label></div><div class='row'><label>State keepalive seconds<br><input name='keepalive' type='number' min='0' max='");
  page += String(kMqttKeepaliveMax);
  page += F("' value='");
  page += String(config.mqtt_keepalive);
  page += F("'></label></div><button type='submit'>Save MQTT</button></form></section>");
}

void appendIBeaconIntervalSelect(String &page, const char *name, uint16_t selected, bool disabled) {
  page += F("<select name='");
  page += name;
  page += F("'");
  if (disabled) page += F(" disabled");
  page += F(">");
  for (uint8_t i = 0; i < sizeof(kIBeaconFilterIntervals) / sizeof(kIBeaconFilterIntervals[0]); i++) {
    const uint16_t value = kIBeaconFilterIntervals[i];
    page += F("<option value='");
    page += String(value);
    page += F("'");
    if (selected == value) page += F(" selected");
    page += F(">");
    page += String(value);
    page += F("s</option>");
  }
  page += F("</select>");
}

void appendIBeaconForm(String &page) {
  const bool unsupported = !iBeaconCaptureSupported();
  page += F("<section class='panel'><div class='panel-title'><h2>iBeacon</h2>");
  if (unsupported) {
    page += F("<span class='pill bad'>unsupported</span>");
  } else if (config.ibeacon_enabled && ibeacon_scanning) {
    page += F("<span id='live-ibeacon' class='pill ok'>scanning</span>");
  } else if (config.ibeacon_enabled) {
    page += F("<span id='live-ibeacon' class='pill bad'>");
    page += htmlEscape(ibeacon_status);
    page += F("</span>");
  } else {
    page += F("<span id='live-ibeacon' class='pill'>disabled</span>");
  }
  page += F("</div><form data-inline='1' method='post' action='/ibeacon'>");
  page += F("<div class='row'><label><input type='checkbox' name='enabled' value='1'");
  if (config.ibeacon_enabled) page += F(" checked");
  if (unsupported) page += F(" disabled");
  page += F(">Enable</label></div>");
  page += F("<div class='row'><label>G1 MACs<br><input name='f1' maxlength='");
  page += String(kIBeaconFilterInputMaxLen);
  page += F("' value='");
  page += htmlEscape(config.ibeacon_filter1_macs);
  page += F("'");
  if (unsupported) page += F(" disabled");
  page += F("></label><label>Max<br>");
  appendIBeaconIntervalSelect(page, "i1", config.ibeacon_filter1_interval_sec, unsupported);
  page += F("</label></div>");
  page += F("<div class='row'><label>G2 MACs<br><input name='f2' maxlength='");
  page += String(kIBeaconFilterInputMaxLen);
  page += F("' value='");
  page += htmlEscape(config.ibeacon_filter2_macs);
  page += F("'");
  if (unsupported) page += F(" disabled");
  page += F("></label><label>Max<br>");
  appendIBeaconIntervalSelect(page, "i2", config.ibeacon_filter2_interval_sec, unsupported);
  page += F("</label></div><button type='submit'");
  if (unsupported) page += F(" disabled");
  page += F(">Save iBeacon</button></form></section>");
}

void appendPowerSavingOption(String &page, uint8_t mode, const __FlashStringHelper *label) {
  page += F("<option value='");
  page += mode;
  page += F("'");
  if (sanitizePowerSavingMode(config.power_saving_mode) == mode) page += F(" selected");
  page += F(">");
  page += label;
  page += F("</option>");
}

void appendPowerSavingSelect(String &page) {
  page += F("<div class='row'><select name='power_saving'>");
  appendPowerSavingOption(page, kPowerSavingOff, F("Off"));
  appendPowerSavingOption(page, kPowerSavingLight, F("Light"));
  appendPowerSavingOption(page, kPowerSavingDeep, F("Deep"));
  page += F("</select></div>");
}

void appendPhyModeSelect(String &page) {
  page += F("<div class='row'><label>PHY mode<br><select name='phy_mode'>");
  for (uint8_t mode = 0; mode <= kPhyModeN; mode++) {
    page += F("<option value='");
    page += String(mode);
    page += F("'");
    if (config.phy_mode == mode) page += F(" selected");
    page += F(">");
    page += phyModeName(mode);
    page += F("</option>");
  }
  page += F("</select></label></div>");
}

void appendWifiDynamicPowerCheckbox(String &page) {
  page += F("<div class='row'><label><input type='checkbox' name='wifi_dynamic_power' value='1'");
  if (config.wifi_dynamic_power) page += F(" checked");
  page += F(">Dynamic Wi-Fi power</label></div>");
}

void handleRoot() {
  String page;
  page.reserve(10800);
  beginStreamedResponse("text/html");
  appendHeader(page, F("myMota32"), true);
  page += F("<div class='grid'>");
  flushStreamChunk(page);
  appendStatusBlock(page);
  flushStreamChunk(page);
  appendTemplateStatus(page);
  flushStreamChunk(page);
  appendDeviceControls(page);
  flushStreamChunk(page);
  appendButtonSettings(page);
  flushStreamChunk(page);
  appendLedSettings(page);
  flushStreamChunk(page);
  appendDeviceStateEnforcementSettings(page);
  flushStreamChunk(page);
  appendRelayPulseSettings(page);
  flushStreamChunk(page);
  appendIBeaconForm(page);
  flushStreamChunk(page);

  page += F("<section class='panel'><h2>Wi-Fi</h2><form method='post' action='/wifi'>");
  page += F("<div class='row'><label>SSID<br><input name='ssid' maxlength='32' required value='");
  page += htmlEscape(config.ssid);
  page += F("'></label></div><div class='row'><label>Password<br><input id='wifi-password' type='password' name='password' maxlength='64' autocomplete='current-password' value='");
  page += htmlEscape(config.password);
  page += F("' onfocus=\"this.type='text'\" onclick=\"this.type='text'\"></label></div>");
  page += F("<div class='row'><label>Hostname<br><input name='hostname' maxlength='32' value='");
  page += htmlEscape(config.hostname);
  page += F("'></label></div>");
  appendPhyModeSelect(page);
  appendWifiDynamicPowerCheckbox(page);
  page += F("<button type='submit'>Save Wi-Fi</button></form>");
  page += F("<p><a class='btn secondary' href='/scan'>Scan networks</a></p></section>");
  flushStreamChunk(page);

  appendMqttForm(page);
  flushStreamChunk(page);

  page += F("<section class='panel'><h2>System</h2><h3>Firmware</h3><form class='fu' method='post' action='/update?verify=1' enctype='multipart/form-data' data-target='");
  page += F(MYMOTA32_TARGET);
  page += F("'>");
  page += F("<input type='file' name='firmware' accept='.bin' required>");
  page += F("<div class='row'><label><input class='fv' type='checkbox' checked>Verify target</label></div>");
  page += F("<button type='submit'>Upload</button></form>");
  page += F("<div class='bb'><h3>Power Saving</h3><form method='post' action='/system'>");
  appendPowerSavingSelect(page);
  page += F("<button type='submit'>Save</button></form></div>");
  page += F("<div class='bb'><h3>Reboot</h3><div class='actions'><a class='btn secondary' href='/reboot'>Reboot</a></div>");
  page += F("<div class='actions'><form class='inline' method='post' action='/factory-reset' onsubmit=\"return confirm('Factory reset?')\"><button class='danger' type='submit'>Factory Reset</button></form></div></div></section>");
  flushStreamChunk(page);

  appendTemplateForm(page);
  flushStreamChunk(page);

  page += F("</div>");
  flushStreamChunk(page);
  appendFooter(page);
  flushStreamChunk(page);
  server.sendContent(F(""));
}

void handleScan() {
  String page;
  page.reserve(2600);
  appendHeader(page, F("myMota32 Scan"));
  page += F("<section class='panel'><h2>Networks</h2>");

  int count = WiFi.scanComplete();
  if (count == WIFI_SCAN_FAILED) {
    WiFi.scanDelete();
    WiFi.scanNetworks(true);
    count = WiFi.scanComplete();
  }
  if (count == WIFI_SCAN_RUNNING) {
    page += F("<p>Scanning for Wi-Fi networks...</p>");
    page += F("<p class='muted'>This page will refresh when results are ready.</p>");
    page += F("<script>setTimeout(function(){location.reload()},1000)</script>");
    page += F("<p><a class='btn secondary' href='/'>Back</a></p></section>");
    appendFooter(page, false);
    sendHtml(page);
    return;
  }
  if (count <= 0) {
    page += F("<p>No networks found.</p>");
  } else {
    page += F("<form method='post' action='/wifi'><div class='row'><label>Password<br><input type='password' name='password' maxlength='64'></label></div>");
    page += F("<div class='row'><label>Hostname<br><input name='hostname' maxlength='32' value='");
    page += htmlEscape(config.hostname);
    page += F("'></label></div>");
    appendPhyModeSelect(page);
    appendWifiDynamicPowerCheckbox(page);
    page += F("<ul class='list'>");
    for (int i = 0; i < count; i++) {
      page += F("<li><label><input type='radio' name='ssid' required value='");
      page += htmlEscape(WiFi.SSID(i));
      page += F("'> ");
      page += htmlEscape(WiFi.SSID(i));
      page += F(" <span class='muted'>");
      page += String(WiFi.RSSI(i));
      page += F(" dBm ch ");
      page += String(WiFi.channel(i));
      page += F("</span></label></li>");
    }
    page += F("</ul><button type='submit'>Save Wi-Fi</button></form>");
  }
  WiFi.scanDelete();
  page += F("<p><a class='btn secondary' href='/'>Back</a></p></section>");
  appendFooter(page);
  sendHtml(page);
}

void handleWifiSave() {
  const String ssid = server.arg("ssid");
  const String password = server.arg("password");
  const String hostname = server.arg("hostname");
  uint8_t phy_mode = config.phy_mode;
  const bool dynamic_power = server.hasArg("wifi_dynamic_power");
  char password_to_save[sizeof(config.password)];

  if (ssid.length() == 0 || ssid.length() > 32 || password.length() > 64 || hostname.length() > 32) {
    sendPlain(400, F("Invalid Wi-Fi settings"));
    return;
  }
  if (server.hasArg("phy_mode")) {
    phy_mode = sanitizePhyMode(static_cast<uint8_t>(server.arg("phy_mode").toInt()));
  }
  if (password.length() == 0 && ssid == config.ssid && config.password[0] != '\0') {
    strlcpy(password_to_save, config.password, sizeof(password_to_save));
  } else {
    strlcpy(password_to_save, password.c_str(), sizeof(password_to_save));
  }
  if (!saveWifiConfig(ssid.c_str(), password_to_save, hostname.c_str(), phy_mode, dynamic_power)) {
    sendPlain(500, F("Could not save Wi-Fi settings"));
    return;
  }
  String page;
  page.reserve(800);
  appendHeader(page, F("myMota32 Wi-Fi"));
  page += F("<p class='ok'>Saved. Rebooting.</p>");
  page += F("<p>Returning when reachable.</p>");
  page += F("<p class='muted'>If IP changed, reconnect manually.</p>");
  appendFooter(page, false, true);
  sendHtml(page);
  scheduleRestart(1200, true);
}

void handleTemplateSave() {
  if (server.hasArg("clear")) {
    StoredConfig candidate = config;
    clearTemplateConfig(candidate);
    if (!saveTemplateConfig(candidate)) {
      sendPlain(500, F("Could not clear template"));
      return;
    }
    decodeTemplateConfig();
    String page;
    page.reserve(700);
    appendHeader(page, F("myMota32 Template"));
    page += F("<p class='ok'>Template cleared. Rebooting.</p>");
    page += F("<p>The page will return to the dashboard when the device is reachable again.</p>");
    appendFooter(page, false, true);
    sendHtml(page);
    scheduleRestart(1200);
    return;
  }

  String template_json = server.arg("template");
  template_json.trim();
  if (template_json.length() == 0) {
    sendPlain(400, F("Template JSON is empty"));
    return;
  }
  StoredConfig candidate = config;
  String error;
  if (!parseTemplateJson(template_json, candidate, error)) {
    String msg = F("Invalid template: ");
    msg += error;
    msg += '\n';
    sendPlain(400, msg);
    return;
  }
  if (!saveTemplateConfig(candidate)) {
    sendPlain(500, F("Could not save template"));
    return;
  }
  decodeTemplateConfig();

  String page;
  page.reserve(900);
  appendHeader(page, F("myMota32 Template"));
  page += F("<p class='ok'>Template saved. Rebooting.</p>");
  if (runtime_template.unsupported_count) {
    page += F("<p class='bad'>The template contains unsupported GPIO functions. Check the Template card after reboot.</p>");
  }
  page += F("<p>The page will return to the dashboard when the device is reachable again.</p>");
  appendFooter(page, false, true);
  sendHtml(page);
  scheduleRestart(1200);
}

void handlePowerSave() {
  if (!server.hasArg("relay") || !server.hasArg("state")) {
    sendPlain(400, F("Missing relay or state"));
    return;
  }
  const int relay = server.arg("relay").toInt();
  const String state = server.arg("state");
  if (relay < 1 || relay > kMaxRelays || !hasPin(runtime_template.relays[relay - 1])) {
    sendPlain(400, F("Invalid relay"));
    return;
  }
  if (state == "on") setRelay(relay - 1, true);
  else if (state == "off") setRelay(relay - 1, false);
  else if (state == "toggle") toggleRelay(relay - 1);
  else { sendPlain(400, F("Invalid relay state")); return; }
  updateDeviceLeds(true);
  sendInlineOkOrHome();
}

#if MYMOTA32_LIGHT_SUPPORTED
void handleLightSave() {
  if (!light.present) {
    sendPlain(400, F("No light output is configured"));
    return;
  }

  if (server.hasArg("power")) {
    const String state = server.arg("power");
    if (state == "on") setLightPower(true);
    else if (state == "off") setLightPower(false);
    else if (state == "toggle") toggleLightPower();
    else { sendPlain(400, F("Invalid light power state")); return; }
  }
  if (server.hasArg("dimmer")) {
    uint16_t dimmer = 0;
    if (!parseUint16Input(server.arg("dimmer"), kLightDimmerOff, kLightDimmerMax, dimmer)) {
      sendPlain(400, F("Invalid dimmer"));
      return;
    }
    setLightDimmer(dimmer);
  }
  if (server.hasArg("ct")) {
    uint16_t ct = 0;
    if (!parseUint16Input(server.arg("ct"), kLightCtMin, kLightCtMax, ct)) {
      sendPlain(400, F("Invalid color temperature"));
      return;
    }
    setLightCt(ct);
  }
  if (server.hasArg("color")) {
    String color = server.arg("color");
    color.trim();
    uint8_t rgb[3];
    if (!parseLightColor(color.c_str(), color.length(), rgb)) {
      sendPlain(400, F("Invalid color"));
      return;
    }
    setLightColor(rgb);
  }
  if (server.hasArg("on_dimmer")) {
    uint16_t on_dimmer = 0;
    if (!parseUint16Input(server.arg("on_dimmer"), kLightDimmerMin, kLightDimmerMax, on_dimmer)) {
      sendPlain(400, F("Invalid ON dimmer"));
      return;
    }
    const uint8_t next_on_dimmer = static_cast<uint8_t>(on_dimmer);
    if (config.light_on_dimmer != next_on_dimmer) {
      config.light_on_dimmer = next_on_dimmer;
      light.config_dirty = true;
      light.config_save_at = millis();
    }
  }
  if (server.hasArg("fade")) {
    const String value = server.arg("fade");
    if (value != "0" && value != "1") {
      sendPlain(400, F("Invalid fade"));
      return;
    }
    setLightFadeEnabled(value == "1");
  }
  if (server.hasArg("speed")) {
    uint16_t speed = 0;
    if (!parseUint16Input(server.arg("speed"), kLightSpeedMin, kLightSpeedMax, speed)) {
      sendPlain(400, F("Invalid speed"));
      return;
    }
    setLightSpeed(speed);
  }
  persistLightConfig(true);

  sendInlineOkOrHome();
}
#endif

void handleLedSave() {
  if (!hasConfigurableLedOutputs()) {
    sendPlain(400, F("No configurable LEDs are available"));
    return;
  }
  uint8_t attachments[kMaxLedOutputs];
  memcpy(attachments, config.led_attach, sizeof(attachments));
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    if (!hasLedOutput(i)) continue;
    String arg_name = F("led");
    arg_name += String(i);
    if (!server.hasArg(arg_name)) {
      sendPlain(400, F("Missing LED setting"));
      return;
    }
    const long raw_value = server.arg(arg_name).toInt();
    if (raw_value < 0 || raw_value > 255) {
      sendPlain(400, F("Invalid LED setting"));
      return;
    }
    const uint8_t attachment = static_cast<uint8_t>(raw_value);
    if (!ledAttachmentAvailable(attachment)) {
      sendPlain(400, F("Invalid LED attachment"));
      return;
    }
    attachments[i] = attachment;
  }
  if (!saveLedAttachments(attachments)) {
    sendPlain(500, F("Could not save LED settings"));
    return;
  }
  updateDeviceLeds(true);
  sendInlineOkOrHome();
}

void handleDeviceStateEnforcementSave() {
  if (!deviceStateEnforcementAvailable()) {
    sendPlain(400, F("No configurable device state settings are available"));
    return;
  }

  uint8_t restore_boot[kMaxRelays];
  uint8_t on_boot[kMaxRelays];
  uint8_t time_enabled[kMaxRelays];
  uint16_t time_seconds[kMaxRelays];
  uint8_t light_restore_boot = config.light_restore_boot;
  memcpy(restore_boot, config.relay_restore_boot, sizeof(restore_boot));
  memcpy(on_boot, config.relay_on_boot, sizeof(on_boot));
  memcpy(time_enabled, config.relay_time_enabled, sizeof(time_enabled));
  memcpy(time_seconds, config.relay_time_seconds, sizeof(time_seconds));

  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (!relayAvailable(i)) continue;

    String restore_boot_arg = F("relay_restore_boot");
    restore_boot_arg += String(i);
    String on_boot_arg = F("relay_on_boot");
    on_boot_arg += String(i);
    String time_enabled_arg = F("relay_time_enabled");
    time_enabled_arg += String(i);
    String seconds_arg = F("relay_time_seconds");
    seconds_arg += String(i);

    restore_boot[i] = server.hasArg(restore_boot_arg) ? 1 : 0;
    on_boot[i] = server.hasArg(on_boot_arg) ? 1 : 0;
    if (on_boot[i]) restore_boot[i] = 0;
    else if (restore_boot[i]) on_boot[i] = 0;
    time_enabled[i] = server.hasArg(time_enabled_arg) ? 1 : 0;

    String seconds_text = server.hasArg(seconds_arg) ? server.arg(seconds_arg) : String();
    seconds_text.trim();
    if (time_enabled[i]) {
      uint16_t seconds = 0;
      if (!parseUint16Input(seconds_text, kRelayEnforcementMinSeconds, kRelayEnforcementMaxSeconds, seconds)) {
        sendPlain(400, F("Invalid relay enforcement seconds"));
        return;
      }
      time_seconds[i] = seconds;
    } else if (seconds_text.length() == 0) {
      time_seconds[i] = 0;
    } else {
      uint16_t seconds = 0;
      if (parseUint16Input(seconds_text, kRelayEnforcementMinSeconds, kRelayEnforcementMaxSeconds, seconds)) {
        time_seconds[i] = seconds;
      }
    }
  }

#if MYMOTA32_LIGHT_SUPPORTED
  if (light.present) {
    light_restore_boot = server.hasArg("light_restore_boot") ? 1 : 0;
  }
#endif

  if (!saveDeviceStateEnforcementConfig(restore_boot, on_boot, time_enabled, time_seconds, light_restore_boot)) {
    sendPlain(500, F("Could not save device state enforcement settings"));
    return;
  }

  sendInlineOkOrHome();
}

void handleRelayPulseSave() {
  if (!hasConfigurableRelays()) {
    sendPlain(400, F("No configurable relays are available"));
    return;
  }

  uint8_t pulse_enabled[kMaxRelays];
  uint16_t pulse_seconds[kMaxRelays];
  memcpy(pulse_enabled, config.relay_pulse_enabled, sizeof(pulse_enabled));
  memcpy(pulse_seconds, config.relay_pulse_seconds, sizeof(pulse_seconds));

  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (!relayAvailable(i)) continue;

    String enabled_arg = F("relay_pulse_enabled");
    enabled_arg += String(i);
    String seconds_arg = F("relay_pulse_seconds");
    seconds_arg += String(i);

    uint16_t seconds = 0;
    String seconds_text = server.hasArg(seconds_arg) ? server.arg(seconds_arg) : String();
    seconds_text.trim();
    if (seconds_text.length() > 0 &&
        !parseUint16Input(seconds_text, 0, kRelayPulseMaxSeconds, seconds)) {
      sendPlain(400, F("Invalid relay pulse seconds"));
      return;
    }

    pulse_seconds[i] = seconds;
    pulse_enabled[i] = (server.hasArg(enabled_arg) && seconds >= kRelayPulseMinSeconds) ? 1 : 0;
  }

  if (!saveRelayPulseConfig(pulse_enabled, pulse_seconds)) {
    sendPlain(500, F("Could not save relay pulsing settings"));
    return;
  }

  sendInlineOkOrHome();
}

bool isValidWebhookUrlTemplate(const String &url) {
  if (!isValidButtonActionText(url, kButtonActionTargetMaxLen, false)) return false;
  if (!url.startsWith(F("http://"))) return false;
  const int host_start = 7;
  const int path_start = url.indexOf('/', host_start);
  const String host_port = path_start < 0 ? url.substring(host_start) : url.substring(host_start, path_start);
  return host_port.length() > 0 && host_port.indexOf(' ') < 0;
}

bool readButtonRelayTargetInput(uint8_t button, const char *prefix, uint8_t action,
                                uint8_t relays[], String &error) {
  if (button >= kMaxButtons) return false;
  String relay_arg = prefix;
  relay_arg += F("_relay");
  relay_arg += String(button);

  if (action != kButtonActionRelayToggle) {
    if (server.hasArg(relay_arg)) {
      uint16_t relay_value = 0;
      if (parseUint16Input(server.arg(relay_arg), 0, kMaxRelays - 1, relay_value)) {
        relays[button] = static_cast<uint8_t>(relay_value);
      }
    }
    return true;
  }

  uint8_t default_relay = 0;
  if (!defaultButtonRelayTarget(button, default_relay)) {
    relays[button] = kButtonRelayUnset;
    return true;
  }

  if (!server.hasArg(relay_arg)) {
    error = F("Missing relay target");
    return false;
  }

  uint16_t relay_value = 0;
  if (!parseUint16Input(server.arg(relay_arg), 0, kMaxRelays - 1, relay_value)) {
    error = F("Invalid relay target");
    return false;
  }

  const uint8_t relay = static_cast<uint8_t>(relay_value);
  if (!relayAvailable(relay)) {
    error = F("Invalid relay target");
    return false;
  }

  relays[button] = relay;
  return true;
}

bool readButtonEventText(uint8_t button, const char *prefix, bool hold, uint8_t action,
                         char targets[][kButtonActionTargetMaxLen + 1],
                         char payloads[][kButtonActionPayloadMaxLen + 1],
                         String &error) {
  String target_arg = prefix;
  target_arg += F("_target");
  target_arg += String(button);
  String payload_arg = prefix;
  payload_arg += F("_payload");
  payload_arg += String(button);

  String target = server.hasArg(target_arg) ? server.arg(target_arg) : String(targets[button]);
  String payload = server.hasArg(payload_arg) ? server.arg(payload_arg) : String(payloads[button]);
  target.trim();

  if (action == kButtonActionMqtt) {
    if (target.length() == 0) target = kDefaultButtonMqttTopic;
    if (payload.length() == 0) payload = hold ? kDefaultButtonMqttHoldPayload : kDefaultButtonMqttPressPayload;
    if (!isValidMqttPublishTopicTemplate(target)) { error = F("Invalid MQTT button topic"); return false; }
    if (!isValidButtonActionText(payload, kButtonActionPayloadMaxLen, false, true)) { error = F("Invalid MQTT button payload"); return false; }
  } else if (action == kButtonActionWebhook) {
    if (!isValidWebhookUrlTemplate(target)) { error = F("Invalid webhook URL"); return false; }
  } else {
    if (!isValidButtonActionText(target, kButtonActionTargetMaxLen, true)) { error = F("Invalid button action target"); return false; }
    if (!isValidButtonActionText(payload, kButtonActionPayloadMaxLen, true, true)) { error = F("Invalid button action payload"); return false; }
  }

  strlcpy(targets[button], target.c_str(), kButtonActionTargetMaxLen + 1);
  strlcpy(payloads[button], payload.c_str(), kButtonActionPayloadMaxLen + 1);
  return true;
}

void handleButtonSave() {
  if (!hasConfigurableButtons()) {
    sendPlain(400, F("No configurable inputs are available"));
    return;
  }

  uint16_t hold_ms = kButtonHoldDefaultMs;
  if (!parseUint16Input(server.arg("hold_ms"), kButtonHoldMinMs, kButtonHoldMaxMs, hold_ms)) {
    sendPlain(400, F("Invalid input hold time"));
    return;
  }
  uint16_t debounce_ms = kButtonDebounceDefaultMs;
  if (!parseUint16Input(server.arg("debounce_ms"), kButtonDebounceMinMs, kButtonDebounceMaxMs, debounce_ms)) {
    sendPlain(400, F("Invalid input debounce time"));
    return;
  }

  StoredConfig candidate = config;
  candidate.button_hold_ms = hold_ms;
  candidate.button_debounce_ms = debounce_ms;

  for (uint8_t i = 0; i < runtime_template.button_count; i++) {
    if (!hasPin(runtime_template.buttons[i])) continue;

    String mode_arg = F("mode");
    mode_arg += String(i);
    if (!server.hasArg(mode_arg)) {
      sendPlain(400, F("Missing input mode"));
      return;
    }
    uint16_t mode_value = 0;
    if (!parseUint16Input(server.arg(mode_arg), 0, 1, mode_value)) {
      sendPlain(400, F("Invalid input mode"));
      return;
    }
    const uint8_t input_mode = static_cast<uint8_t>(mode_value);
    candidate.input_mode[i] = input_mode;

    if (input_mode == kInputModeSwitch) {
      String relay_arg = F("relay");
      relay_arg += String(i);
      String reverse_arg = F("reverse");
      reverse_arg += String(i);
      uint8_t unused_relay = 0;
      const bool has_relay_target = defaultButtonRelayTarget(i, unused_relay);
      if ((has_relay_target && !server.hasArg(relay_arg)) || !server.hasArg(reverse_arg)) {
        sendPlain(400, F("Missing switch setting"));
        return;
      }
      uint16_t relay_value = 0;
      uint16_t reverse_value = 0;
      if ((has_relay_target && !parseUint16Input(server.arg(relay_arg), 0, kMaxRelays - 1, relay_value)) ||
          !parseUint16Input(server.arg(reverse_arg), 0, 1, reverse_value)) {
        sendPlain(400, F("Invalid switch setting"));
        return;
      }
      if (has_relay_target) {
        const uint8_t relay = static_cast<uint8_t>(relay_value);
        if (!relayAvailable(relay)) {
          sendPlain(400, F("Invalid switch relay"));
          return;
        }
        candidate.input_relay[i] = relay;
      } else {
        sendPlain(400, F("Invalid switch target"));
        return;
      }
      candidate.input_on_level[i] = reverse_value ? kInputOnLevelLow : kInputOnLevelHigh;
      continue;
    }

    candidate.input_relay[i] = i;
    candidate.input_on_level[i] = kInputOnLevelUnset;

    String press_arg = F("press");
    press_arg += String(i);
    String hold_arg = F("hold");
    hold_arg += String(i);
    if (!server.hasArg(press_arg) || !server.hasArg(hold_arg)) {
      sendPlain(400, F("Missing button action setting"));
      return;
    }
    uint16_t press_value = 0;
    uint16_t hold_value = 0;
    if (!parseUint16Input(server.arg(press_arg), 0, 255, press_value) ||
        !parseUint16Input(server.arg(hold_arg), 0, 255, hold_value)) {
      sendPlain(400, F("Invalid button action setting"));
      return;
    }

    const uint8_t press_action = static_cast<uint8_t>(press_value);
    const uint8_t hold_action = static_cast<uint8_t>(hold_value);
    if (!isButtonActionEncoding(press_action) || !isButtonActionEncoding(hold_action) ||
        !buttonActionAvailable(i, press_action) || !buttonActionAvailable(i, hold_action)) {
      sendPlain(400, F("Invalid button action"));
      return;
    }
    candidate.button_press_action[i] = press_action;
    candidate.button_hold_action[i] = hold_action;

    String error;
    if (!readButtonRelayTargetInput(i, "press", press_action, candidate.button_press_relay, error) ||
        !readButtonRelayTargetInput(i, "hold", hold_action, candidate.button_hold_relay, error) ||
        !readButtonEventText(i, "press", false, press_action, candidate.button_press_target, candidate.button_press_payload, error) ||
        !readButtonEventText(i, "hold", true, hold_action, candidate.button_hold_target, candidate.button_hold_payload, error)) {
      sendPlain(400, error);
      return;
    }
  }

  if (!saveInputConfig(candidate)) {
    sendPlain(500, F("Could not save input settings"));
    return;
  }
  for (uint8_t i = 0; i < runtime_template.button_count; i++) {
    if (effectiveInputMode(i) == kInputModeSwitch && hasPin(runtime_template.buttons[i])) {
      uint8_t target = 0;
      if (inputRelayTarget(i, target)) setRelay(target, readInputActive(i));
    }
  }
  updateDeviceLeds(true);
  sendInlineOkOrHome();
}

struct ApiSettingsStats {
  uint16_t applied;
  uint16_t skipped;
};

void recordApiSettingsApplied(ApiSettingsStats &stats) {
  stats.applied++;
}

void recordApiSettingsSkipped(ApiSettingsStats &stats) {
  stats.skipped++;
}

const __FlashStringHelper *apiSettingsActionName(uint8_t action) {
  switch (action) {
    case kButtonActionRelayToggle: return F("relay_toggle");
    case kButtonActionMqtt: return F("mqtt");
    case kButtonActionWebhook: return F("webhook");
    default: return F("none");
  }
}

const __FlashStringHelper *apiSettingsInputModeName(uint8_t mode) {
  return mode == kInputModeSwitch ? F("switch") : F("button");
}

bool apiSettingsButtonAvailable(uint8_t input) {
  return input < runtime_template.button_count && hasPin(runtime_template.buttons[input]);
}

void appendApiSettingsJson(String &out) {
  out += F("{\"format\":\"mymota-api-settings\",\"api_version\":");
  out += kApiSettingsVersion;
  out += F(",\"hold_ms\":");
  out += config.button_hold_ms;
  out += F(",\"inputs\":[");
  bool first = true;
  for (uint8_t i = 0; i < runtime_template.button_count && i < kMaxButtons; i++) {
    if (!first) out += ',';
    first = false;
    if (!hasPin(runtime_template.buttons[i])) {
      out += F("null");
      continue;
    }
    out += F("{\"input\":");
    out += i + 1;
    out += F(",\"mode\":\"");
    out += apiSettingsInputModeName(effectiveInputMode(i));
    out += F("\",\"press\":{\"action\":\"");
    out += apiSettingsActionName(config.button_press_action[i]);
    out += F("\",\"mqtt_topic\":\"");
    out += jsonEscape(config.button_press_target[i]);
    out += F("\",\"mqtt_payload\":\"");
    out += jsonEscape(config.button_press_payload[i]);
    out += F("\"}}");
  }
  out += F("]}");
}

bool applyApiInputPressMqttValues(uint16_t input_number, bool has_topic, const String &topic_value,
                                  bool has_payload, const String &payload_value, StoredConfig &target,
                                  ApiSettingsStats &stats) {
  if (input_number < 1 || input_number > kMaxButtons) {
    recordApiSettingsSkipped(stats);
    return false;
  }
  const uint8_t input = static_cast<uint8_t>(input_number - 1);
  if (!apiSettingsButtonAvailable(input)) {
    recordApiSettingsSkipped(stats);
    return false;
  }
  if (!has_topic && !has_payload) {
    recordApiSettingsSkipped(stats);
    return false;
  }

  String topic = target.button_press_target[input];
  topic.trim();
  if (topic.length() == 0 || !isValidMqttPublishTopicTemplate(topic)) {
    topic = kDefaultButtonMqttTopic;
  }
  String payload = target.button_press_payload[input];
  if (payload.length() == 0 || !isValidButtonActionText(payload, kButtonActionPayloadMaxLen, false, true)) {
    payload = kDefaultButtonMqttPressPayload;
  }

  if (has_topic) {
    topic = topic_value;
    topic.trim();
  }
  if (has_topic && !isValidMqttPublishTopicTemplate(topic)) {
    recordApiSettingsSkipped(stats);
    return false;
  }
  if (has_payload) {
    payload = payload_value;
  }
  if (has_payload && !isValidButtonActionText(payload, kButtonActionPayloadMaxLen, false, true)) {
    recordApiSettingsSkipped(stats);
    return false;
  }

  target.input_mode[input] = kInputModeButton;
  target.input_relay[input] = input;
  target.input_on_level[input] = kInputOnLevelUnset;
  target.button_press_action[input] = kButtonActionMqtt;
  strlcpy(target.button_press_target[input], topic.c_str(), sizeof(target.button_press_target[input]));
  strlcpy(target.button_press_payload[input], payload.c_str(), sizeof(target.button_press_payload[input]));
  if (has_topic) recordApiSettingsApplied(stats);
  if (has_payload) recordApiSettingsApplied(stats);
  return true;
}

bool apiSettingsGetArg(const String &primary, const String &fallback, String &out) {
  if (server.hasArg(primary)) {
    out = server.arg(primary);
    return true;
  }
  if (fallback.length() && server.hasArg(fallback)) {
    out = server.arg(fallback);
    return true;
  }
  return false;
}

bool apiSettingsIndexedArg(uint8_t input_number, const char *primary_suffix, const char *fallback_suffix, String &out) {
  String primary = F("input");
  primary += input_number;
  primary += primary_suffix;
  String fallback = F("input");
  fallback += input_number;
  fallback += fallback_suffix;
  return apiSettingsGetArg(primary, fallback, out);
}

bool apiSettingsIndexedArgPresent(uint8_t input_number, const char *primary_suffix, const char *fallback_suffix) {
  String primary = F("input");
  primary += input_number;
  primary += primary_suffix;
  if (server.hasArg(primary)) return true;

  String fallback = F("input");
  fallback += input_number;
  fallback += fallback_suffix;
  return server.hasArg(fallback);
}

bool apiSettingsGetHasUpdateArgs() {
  if (server.hasArg("hold_ms")) return true;
  if (server.hasArg("input") || server.hasArg("id")) return true;
  for (uint8_t input_number = 1; input_number <= kMaxButtons; input_number++) {
    if (apiSettingsIndexedArgPresent(input_number, "_mqtt_topic", "_topic") ||
        apiSettingsIndexedArgPresent(input_number, "_mqtt_payload", "_payload")) {
      return true;
    }
  }
  return false;
}

bool applyApiSettingsGetArgs(StoredConfig &target, ApiSettingsStats &stats) {
  bool saw_setting_arg = false;

  if (server.hasArg("hold_ms")) {
    saw_setting_arg = true;
    uint16_t hold_ms = kButtonHoldDefaultMs;
    if (parseUint16Input(server.arg("hold_ms"), kButtonHoldMinMs, kButtonHoldMaxMs, hold_ms)) {
      target.button_hold_ms = hold_ms;
      recordApiSettingsApplied(stats);
    } else {
      recordApiSettingsSkipped(stats);
    }
  }

  if (server.hasArg("input") || server.hasArg("id")) {
    saw_setting_arg = true;
    const String input_text = server.hasArg("input") ? server.arg("input") : server.arg("id");
    uint16_t input_number = 0;
    if (!parseUint16Input(input_text, 1, kMaxButtons, input_number)) {
      recordApiSettingsSkipped(stats);
    } else {
      String topic;
      String payload;
      const bool has_topic = apiSettingsGetArg(F("mqtt_topic"), F("topic"), topic);
      const bool has_payload = apiSettingsGetArg(F("mqtt_payload"), F("payload"), payload);
      applyApiInputPressMqttValues(input_number, has_topic, topic, has_payload, payload, target, stats);
    }
  }

  for (uint8_t input_number = 1; input_number <= kMaxButtons; input_number++) {
    String topic;
    String payload;
    const bool has_topic = apiSettingsIndexedArg(input_number, "_mqtt_topic", "_topic", topic);
    const bool has_payload = apiSettingsIndexedArg(input_number, "_mqtt_payload", "_payload", payload);
    if (!has_topic && !has_payload) continue;
    saw_setting_arg = true;
    applyApiInputPressMqttValues(input_number, has_topic, topic, has_payload, payload, target, stats);
  }

  return saw_setting_arg;
}

void sendApiSettingsError(uint16_t status, const __FlashStringHelper *message) {
  String out;
  out.reserve(120);
  out += F("{\"ok\":false,\"error\":\"");
  out += message;
  out += F("\"}");
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(status, F("application/json"), out);
}

void finishApiSettingsUpdate(const StoredConfig &candidate, const ApiSettingsStats &stats) {
  if (stats.applied == 0) {
    String out;
    out.reserve(260);
    out += F("{\"ok\":false,\"error\":\"No API settings applied\",\"skipped\":");
    out += stats.skipped;
    out += F("}");
    server.sendHeader(F("Cache-Control"), F("no-store"));
    server.send(400, F("application/json"), out);
    return;
  }

  if (!saveInputConfig(candidate)) {
    sendApiSettingsError(500, F("Could not save settings"));
    return;
  }
  updateDeviceLeds(true);

  String out;
  out.reserve(2200);
  out += F("{\"ok\":true,\"applied\":");
  out += stats.applied;
  out += F(",\"skipped\":");
  out += stats.skipped;
  out += F(",\"settings\":");
  appendApiSettingsJson(out);
  out += F("}");
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("application/json"), out);
}

void handleApiSettingsGet() {
  if (server.hasArg("power_saving")) {
    uint8_t mode = kPowerSavingOff;
    if (!parsePowerSavingMode(server.arg("power_saving"), mode)) {
      sendApiSettingsError(400, F("Invalid power saving"));
      return;
    }
    if (!savePowerSavingConfig(mode)) {
      sendApiSettingsError(500, F("Could not save settings"));
      return;
    }
    server.send(200, F("application/json"), F("{\"ok\":true}"));
    return;
  }

  if (!apiSettingsGetHasUpdateArgs()) {
    String out;
    out.reserve(1800);
    appendApiSettingsJson(out);
    server.sendHeader(F("Cache-Control"), F("no-store"));
    server.send(200, F("application/json"), out);
    return;
  }

  StoredConfig candidate = config;
  ApiSettingsStats stats = {0, 0};
  if (applyApiSettingsGetArgs(candidate, stats)) {
    finishApiSettingsUpdate(candidate, stats);
    return;
  }
  sendApiSettingsError(400, F("No API settings supplied"));
}

void handleMqttSave() {
  String host = server.arg("host");
  String port_arg = server.arg("port");
  String topic = server.arg("topic");
  String keepalive_arg = server.arg("keepalive");
  host.trim();
  port_arg.trim();
  topic.trim();
  keepalive_arg.trim();

  uint16_t port = kMqttDefaultPort;
  uint16_t keepalive = 0;
  if (!isValidMqttHost(host)) {
    sendPlain(400, F("Invalid MQTT host"));
    return;
  }
  if (!parseUint16Input(port_arg, 1, 65535U, port)) {
    sendPlain(400, F("Invalid MQTT port"));
    return;
  }
  if (!isValidMqttTopic(topic)) {
    sendPlain(400, F("Invalid MQTT topic"));
    return;
  }
  if (!parseUint16Input(keepalive_arg, 0, kMqttKeepaliveMax, keepalive)) {
    sendPlain(400, F("Invalid MQTT keepalive"));
    return;
  }

  if (!saveMqttConfig(host.c_str(), port, topic.c_str(), keepalive)) {
    sendPlain(500, F("Could not save MQTT settings"));
    return;
  }

  sendInlineOkOrHome();
}

void handleEnergySave() {
  if (!energy.present) {
    sendPlain(400, F("No energy monitor is configured"));
    return;
  }

  float total_offset_kwh = 0.0f;
  uint16_t energy_report_interval = config.energy_mqtt_interval;
  uint16_t energy_report_change_percent_x10 = config.energy_mqtt_change_percent_x10;
  uint16_t energy_report_change_watts = config.energy_mqtt_change_watts;
  uint64_t total_offset_scaled = 0;
  if (!parseUnsignedScaledDecimalInput(server.arg("total_offset_kwh"), 4,
                                       static_cast<uint64_t>(kEnergyTotalOffsetMaxKwh) * 10000ULL,
                                       total_offset_scaled)) {
    sendPlain(400, F("Invalid total kWh offset"));
    return;
  }
  total_offset_kwh = static_cast<float>(total_offset_scaled) / 10000.0f;
  if (server.hasArg("energy_report_interval") &&
      !parseUint16Input(server.arg("energy_report_interval"), 0, kMqttEnergyIntervalMax, energy_report_interval)) {
    sendPlain(400, F("Invalid energy report interval"));
    return;
  }
  if (server.hasArg("energy_report_change_percent")) {
    uint64_t percent_scaled = 0;
    if (!parseUnsignedScaledDecimalInput(server.arg("energy_report_change_percent"), 1,
                                         static_cast<uint64_t>(kMqttEnergyChangeMaxPercent * 10.0f),
                                         percent_scaled)) {
      sendPlain(400, F("Invalid energy report change percent"));
      return;
    }
    energy_report_change_percent_x10 = static_cast<uint16_t>(percent_scaled);
  }
  if (server.hasArg("energy_report_change_watts") &&
      !parseUint16Input(server.arg("energy_report_change_watts"), 0, kMqttEnergyChangeMaxWatts, energy_report_change_watts)) {
    sendPlain(400, F("Invalid energy report change watts"));
    return;
  }

  if (!saveEnergyConfig(total_offset_kwh, energy_report_interval, energy_report_change_percent_x10,
                        energy_report_change_watts)) {
    sendPlain(500, F("Could not save energy settings"));
    return;
  }

  sendInlineOkOrHome();
}

void handleIBeaconSave() {
  const bool enabled = server.hasArg("enabled") && server.arg("enabled") == "1";
  if (enabled && !iBeaconCaptureSupported()) {
    sendPlain(400, F("unsupported"));
    return;
  }

  uint16_t filter1_interval = kIBeaconFilter1DefaultSec;
  uint16_t filter2_interval = kIBeaconFilter2DefaultSec;
  if (!parseUint16Input(server.hasArg("i1") ? server.arg("i1") : String(kIBeaconFilter1DefaultSec),
                        1, 600, filter1_interval) ||
      !isIBeaconFilterInterval(filter1_interval) ||
      !parseUint16Input(server.hasArg("i2") ? server.arg("i2") : String(kIBeaconFilter2DefaultSec),
                        1, 600, filter2_interval) ||
      !isIBeaconFilterInterval(filter2_interval)) {
    sendPlain(400, F("invalid interval"));
    return;
  }

  char filter1_macs[kIBeaconFilterListMaxLen + 1]{};
  char filter2_macs[kIBeaconFilterListMaxLen + 1]{};
  if (!normalizeIBeaconMacList(server.hasArg("f1") ? server.arg("f1") : String(),
                               filter1_macs, sizeof(filter1_macs)) ||
      !normalizeIBeaconMacList(server.hasArg("f2") ? server.arg("f2") : String(),
                               filter2_macs, sizeof(filter2_macs))) {
    sendPlain(400, F("invalid mac"));
    return;
  }

  if (!saveIBeaconConfig(enabled, filter1_interval, filter1_macs, filter2_interval, filter2_macs)) {
    sendPlain(500, F("save failed"));
    return;
  }

  resetIBeaconRuntimeState();
  if (config.ibeacon_enabled) startIBeaconCapture();
  else stopIBeaconCapture();

  sendInlineOkOrHome();
}

void handleSystemSave() {
  uint8_t mode = kPowerSavingOff;
  if (!parsePowerSavingMode(server.arg("power_saving"), mode)) {
    sendPlain(400, F("Invalid power saving"));
    return;
  }

  if (!savePowerSavingConfig(mode)) {
    sendPlain(500, F("Save failed"));
    return;
  }

  sendInlineOkOrHome();
}

void handleReboot() {
  String page;
  page.reserve(700);
  appendHeader(page, F("myMota32 Reboot"));
  page += F("<p class='ok'>Rebooting.</p>");
  page += F("<p>The page will return to the dashboard when the device is reachable again.</p>");
  appendFooter(page, false, true);
  sendHtml(page);
  scheduleRestart(500, true);
}

void handleFactoryReset() {
  if (!factoryResetConfig()) {
    sendPlain(500, F("Could not factory reset settings"));
    return;
  }
  energy.present = false;
  clearBootRecoveryState();
  String page;
  page.reserve(900);
  appendHeader(page, F("myMota32 Factory Reset"));
  page += F("<p class='ok'>Factory reset complete. Rebooting.</p>");
  page += F("<p>All saved settings have been cleared. After reboot, use the setup AP if the device does not return on this address.</p>");
  appendFooter(page, false, true);
  sendHtml(page);
  scheduleRestart(800);
}

void handleHealth() {
  updateIBeaconMqttReportRate(millis());
  String out;
  out.reserve(1900);
  beginStreamedResponse("application/json");
  out += F("{\"name\":\"myMota32\",\"version\":\"");
  out += F(MYMOTA32_VERSION);
  out += F("\",\"target\":\"");
  out += F(MYMOTA32_TARGET);
  out += F("\",\"chip_model\":\"");
  out += chipModelName();
  out += F("\",\"chip_id\":\"");
  out += chipIdHex();
  out += F("\",\"chip\":\"");
  out += chipIdHex();
  out += F("\",\"boot_id\":");
  out += boot_id;
  out += F(",\"heap\":");
  out += ESP.getFreeHeap();
  out += F(",\"uptime\":");
  out += millis() / 1000;
  out += F(",\"perf\":{\"loop_hz\":");
  out += perf_last_loop_hz;
  out += F(",\"loop_load\":");
  out += perf_last_loop_load;
  out += F(",\"loop_max_us\":");
  out += perf_last_loop_max_us;
  out += F("},\"wifi\":");
  out += ((WiFi.status() == WL_CONNECTED) ? F("true") : F("false"));
  out += F(",\"wifi_ssid\":\"");
  out += (WiFi.status() == WL_CONNECTED ? jsonEscape(WiFi.SSID().c_str()) : String());
  out += F("\",\"ip\":\"");
  out += (WiFi.status() == WL_CONNECTED ? ipToString(WiFi.localIP()) : String());
  out += F("\",\"rssi\":");
  if (WiFi.status() == WL_CONNECTED) out += WiFi.RSSI();
  else out += F("null");
  out += F(",\"wifi_tx_power\":");
  appendWifiTxPowerDbm(out);
  out += F(",\"ap\":");
  out += (ap_started ? F("true") : F("false"));
  out += F(",\"configured_phy_mode\":");
  out += config.phy_mode;
  out += F(",\"configured_phy\":\"");
  out += phyModeName(config.phy_mode);
  out += F("\",\"active_phy_mode\":");
  out += activePhyMode();
  out += F(",\"active_phy\":\"");
  out += phyModeName(activePhyMode());
  out += F("\",\"recovery\":{\"fast_boot_count\":");
  out += boot_recovery_count;
  out += F(",\"limit\":");
  out += kBootRecoveryLimit;
  out += F(",\"stable_seconds\":");
  out += kBootRecoveryStableMs / 1000;
  out += F(",\"factory_reset\":");
  out += (boot_recovery_factory_reset ? F("true") : F("false"));
  out += F("},\"template\":{\"enabled\":");
  out += (runtime_template.enabled ? F("true") : F("false"));
  if (runtime_template.enabled) {
    out += F(",\"name\":\"");
    out += jsonEscape(runtime_template.name);
    out += F("\",\"base\":");
    out += runtime_template.base;
    out += F(",\"flag\":");
    out += runtime_template.flag;
    out += F(",\"relays\":");
    out += runtime_template.relay_count;
    out += F(",\"buttons\":");
    out += runtime_template.button_count;
    out += F(",\"leds\":");
    out += runtime_template.led_count;
    out += F(",\"unsupported\":");
    out += runtime_template.unsupported_count;
  }
  out += F("},\"power\":[");
  for (uint8_t i = 0; i < kMaxRelays; i++) {
    if (i) out += ',';
    if (relayAvailable(i)) out += relay_state[i] ? F("true") : F("false");
    else out += F("null");
  }
  out += F("],\"buttons\":[");
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    if (i) out += ',';
    if (i < runtime_template.button_count && hasPin(runtime_template.buttons[i])) {
      out += F("{\"pressed\":");
      out += button_state[i].stable_pressed ? F("true") : F("false");
      out += F(",\"state\":\"");
      out += inputStateName(i, button_state[i].stable_pressed);
      out += F("\"}");
    } else {
      out += F("null");
    }
  }
  out += F("],\"leds\":[");
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    if (i) out += ',';
    if (hasLedOutput(i)) {
      out += F("{\"on\":");
      out += ledOutputOn(i) ? F("true") : F("false");
      out += F("}");
    } else {
      out += F("null");
    }
  }
  out += F("]");
#if MYMOTA32_LIGHT_SUPPORTED
  if (light.present) {
    out += F(",\"light\":{\"power\":");
    out += light.power ? F("true") : F("false");
    out += F(",\"dimmer\":");
    out += light.dimmer;
    out += F(",\"ct\":");
    out += light.ct;
    out += F(",\"mode\":\"");
    out += light.mode == kLightModeRgb ? F("rgb") : F("white");
    out += F("\",\"color\":\"");
    appendLightColorHex(out);
    out += F("\",\"on_dimmer\":");
    out += config.light_on_dimmer;
    out += F(",\"fade\":");
    out += config.light_fade ? F("true") : F("false");
    out += F(",\"speed\":");
    out += config.light_speed;
    out += F(",\"fading\":");
    out += light.fade_running ? F("true") : F("false");
    out += F(",\"driver\":\"sm2335\"}");
  }
#endif
  if (energy.present) {
    out += F(",\"energy\":{\"driver\":\"");
    if (energy.driver == kEnergyDriverBl0939) out += F("bl0939");
    else if (energy.driver == kEnergyDriverHlw8012) out += energy.hjl ? F("bl0937") : F("hlw8012");
    else out += F("unknown");
    out += F("\",\"voltage\":");
    appendFloatDecimal(out, energy.voltage, 1);
    out += F(",\"current\":");
    appendFloatDecimal(out, energy.current, 3);
    out += F(",\"power\":");
    appendFloatDecimal(out, energy.power, 1);
    out += F(",\"total_kwh\":");
    appendFloatDecimal(out, reportedEnergyTotalKwh(), 4);
    out += F(",\"recorded_total_kwh\":");
    appendFloatDecimal(out, energy.total_kwh, 4);
    out += F(",\"offset_kwh\":");
    appendFloatDecimal(out, config.energy_total_offset_kwh, 4);
    out += F(",\"report_interval\":");
    out += config.energy_mqtt_interval;
    out += F(",\"report_change_percent\":");
    appendScaledDecimal(out, config.energy_mqtt_change_percent_x10, 1);
    out += F(",\"report_change_watts\":");
    out += config.energy_mqtt_change_watts;
    out += F(",\"last_mqtt_report_ms_ago\":");
    if (last_mqtt_energy_publish == 0) out += F("null");
    else out += millis() - last_mqtt_energy_publish;
    out += F(",\"last_mqtt_report_reason\":\"");
    out += mqttEnergyReportReasonName(last_mqtt_energy_report_reason);
    out += F("\",\"temperature\":");
    appendFloatDecimal(out, energy.temperature, 1);
    if (energy.channel_count > 1) {
      out += F(",\"channels\":[");
      for (uint8_t i = 0; i < energy.channel_count && i < kEnergyMaxChannels; i++) {
        if (i) out += ',';
        out += F("{\"voltage\":");
        appendFloatDecimal(out, energy.channel[i].voltage, 1);
        out += F(",\"current\":");
        appendFloatDecimal(out, energy.channel[i].current, 3);
        out += F(",\"power\":");
        appendFloatDecimal(out, energy.channel[i].power, 1);
        out += F("}");
      }
      out += F("]");
    }
    out += F(",\"debug\":{");
    if (energy.driver == kEnergyDriverBl0939) {
      out += F("\"rx_pin\":");
      out += energy.rx_pin;
      out += F(",\"tx_pin\":");
      out += energy.tx_pin;
    } else if (energy.driver == kEnergyDriverHlw8012) {
      out += F("\"cf_pin\":");
      out += energy.cf_pin;
      out += F(",\"cf1_pin\":");
      out += energy.cf1_pin;
      out += F(",\"sel_pin\":");
      out += energy.sel_pin;
      out += F(",\"sel_inverted\":");
      out += energy.sel_inverted ? F("true") : F("false");
      out += F(",\"load_off\":");
      out += energy.hlw_load_off ? F("true") : F("false");
      out += F(",\"cf_power_pulse_us\":");
      out += energy.hlw_cf_power_pulse_length;
      out += F(",\"cf1_voltage_pulse_us\":");
      out += energy.hlw_cf1_voltage_pulse_length;
      out += F(",\"cf1_current_pulse_us\":");
      out += energy.hlw_cf1_current_pulse_length;
    } else {
      out += F("\"driver_id\":");
      out += energy.driver;
    }
    out += F(",\"last_success_ms_ago\":");
    out += millis() - energy.last_success_ms;
    out += F(",\"voltage_raw\":");
    out += energy.voltage_raw;
    out += F(",\"raw\":[");
    for (uint8_t i = 0; i < energy.channel_count && i < kEnergyMaxChannels; i++) {
      if (i) out += ',';
      out += F("{\"current\":");
      out += energy.channel[i].current_raw;
      out += F(",\"power\":");
      out += energy.channel[i].power_raw;
      out += F("}");
    }
    out += F("]}}");
  }
  out += F(",\"ibeacon\":{\"enabled\":");
  out += config.ibeacon_enabled ? F("true") : F("false");
  out += F(",\"scanning\":");
  out += ibeacon_scanning ? F("true") : F("false");
  out += F(",\"status\":\"");
  out += jsonEscape(ibeacon_status);
  out += F("\",\"mqtt_reports_per_minute\":");
  out += ibeacon_mqtt_reports_per_minute;
  out += F("}");
  out += F(",\"mqtt\":{\"enabled\":");
  out += (mqttConfigured() ? F("true") : F("false"));
  out += F(",\"connected\":");
  out += (mqtt_client.connected() ? F("true") : F("false"));
  out += F(",\"host\":\"");
  out += jsonEscape(config.mqtt_host);
  out += F("\",\"port\":");
  out += config.mqtt_port;
  out += F(",\"topic\":\"");
  out += jsonEscape(config.mqtt_topic);
  out += F("\",\"keepalive\":");
  out += config.mqtt_keepalive;
  out += F(",\"pending\":");
  out += static_cast<unsigned>(mqtt_pending_relay_mask) + static_cast<unsigned>(mqtt_pending_light_mask);
  out += F(",\"last_connect_result\":\"");
  out += mqttConnectResultName(last_mqtt_connect_result);
  out += F("\",\"last_connect_ms\":");
  out += last_mqtt_connect_duration;
  out += F(",\"last_attempt_ms_ago\":");
  if (last_mqtt_connect_attempt == 0) out += F("null");
  else out += millis() - last_mqtt_connect_attempt;
  out += F("}}");
  flushStreamChunk(out);
  server.sendContent(F(""));
}

void handleCmnd() {
  if (!server.hasArg("cmnd")) {
    sendPlain(400, F("Missing cmnd"));
    return;
  }

  String out;
  String error;
  if (!executeCmndString(server.arg("cmnd"), out, error)) {
    sendPlain(400, error);
    return;
  }

  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("application/json"), out);
}

void clearUpdateRuntime() {
  update_started = false;
  update_partition = nullptr;
  update_written = 0;
  update_erased_until = 0;
  update_header_len = 0;
  memset(update_header, 0, sizeof(update_header));
}

bool updateEraseUntil(size_t end_offset) {
  if (!update_partition) return false;
  while (update_erased_until < end_offset) {
    if (esp_partition_erase_range(update_partition, update_erased_until, kUpdateSectorSize) != ESP_OK) {
      update_error = UPDATE_ERROR_ERASE;
      return false;
    }
    update_erased_until += kUpdateSectorSize;
  }
  return true;
}

bool updateBeginUpload() {
  update_partition = esp_ota_get_next_update_partition(nullptr);
  if (!update_partition) {
    update_error = UPDATE_ERROR_NO_PARTITION;
    return false;
  }
  update_written = 0;
  update_erased_until = 0;
  update_header_len = 0;
  update_started = true;
  return true;
}

bool updateWritePayload(const uint8_t *data, size_t len) {
  if (!update_started || !update_partition) {
    update_error = UPDATE_ERROR_BAD_ARGUMENT;
    return false;
  }
  if (len == 0) return true;
  if (update_written + len > update_partition->size) {
    update_error = UPDATE_ERROR_SPACE;
    return false;
  }

  size_t pos = 0;
  if (update_header_len < kUpdateHeaderHoldBytes) {
    const size_t need = kUpdateHeaderHoldBytes - update_header_len;
    const size_t take = len < need ? len : need;
    memcpy(update_header + update_header_len, data, take);
    update_header_len += take;
    update_written += take;
    pos += take;
  }

  if (pos < len) {
    const size_t write_offset = update_written;
    const size_t write_len = len - pos;
    const size_t erase_end = ((write_offset + write_len + kUpdateSectorSize - 1) / kUpdateSectorSize) * kUpdateSectorSize;
    if (!updateEraseUntil(erase_end)) return false;
    if (esp_partition_write(update_partition, write_offset, data + pos, write_len) != ESP_OK) {
      update_error = UPDATE_ERROR_WRITE;
      return false;
    }
    update_written += write_len;
  }
  return true;
}

bool updateFinishUpload() {
  if (!update_started || !update_partition || update_header_len < kUpdateHeaderHoldBytes) {
    update_error = UPDATE_ERROR_SIZE;
    clearUpdateRuntime();
    return false;
  }
  if (!updateEraseUntil(kUpdateSectorSize)) {
    clearUpdateRuntime();
    return false;
  }
  if (esp_partition_write(update_partition, 0, update_header, sizeof(update_header)) != ESP_OK) {
    update_error = UPDATE_ERROR_WRITE;
    clearUpdateRuntime();
    return false;
  }

  uint8_t check = 0;
  if (esp_partition_read(update_partition, 0, &check, sizeof(check)) != ESP_OK || check != kEspImageMagic) {
    update_error = UPDATE_ERROR_READ;
    clearUpdateRuntime();
    return false;
  }
  if (esp_ota_set_boot_partition(update_partition) != ESP_OK) {
    update_error = UPDATE_ERROR_ACTIVATE;
    clearUpdateRuntime();
    return false;
  }

  clearUpdateRuntime();
  update_ok = true;
  return true;
}

void updateAbortUpload(uint8_t err) {
  clearUpdateRuntime();
  update_ok = false;
  update_error = err;
}

void handleUpdateDone() {
  if (update_ok && update_error == UPDATE_ERROR_OK) {
    String page;
    page.reserve(700);
    appendHeader(page, F("myMota32 Update"));
    page += F("<p class='ok'>Firmware uploaded. Rebooting.</p>");
    page += F("<p>The page will return to the dashboard when the device is reachable again.</p>");
    appendFooter(page, false, true);
    sendHtml(page);
    scheduleRestart(1200, true);
    return;
  }
  String page;
  page.reserve(800);
  appendHeader(page, F("myMota32 Update Failed"));
  page += F("<p class='bad'>Firmware upload failed: ");
  page += updateErrorName(update_error);
  page += F("</p><p><a href='/'>Back</a></p>");
  appendFooter(page);
  sendHtml(page);
}

void handleUpdateUpload() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    clearUpdateRuntime();
    update_ok = false;
    update_error = UPDATE_ERROR_OK;
    if (upload.filename.length() == 0) {
      update_error = UPDATE_ERROR_SIZE;
      return;
    }
    if (truthyUpdateVerifyArg() && !firmwareFilenameMatchesTarget(upload.filename)) {
      update_error = kUpdateErrorTargetMismatch;
      return;
    }
    persistEnergyTotal(true);
    persistLightConfig(true);
    return;
  }
  if (upload.status == UPLOAD_FILE_WRITE && update_error != UPDATE_ERROR_OK) return;
  if (upload.status == UPLOAD_FILE_WRITE) {
    if (!update_started && upload.totalSize == 0) {
      if (upload.currentSize < 4) { update_error = UPDATE_ERROR_SIZE; return; }
      if (upload.buf[0] != kEspImageMagic) { update_error = UPDATE_ERROR_MAGIC_BYTE; return; }
      if (!updateBeginUpload()) return;
    }
    updateWritePayload(upload.buf, upload.currentSize);
    return;
  }
  if (upload.status == UPLOAD_FILE_END) {
    if (update_error != UPDATE_ERROR_OK) {
      clearUpdateRuntime();
    } else if (!update_started) {
      update_error = UPDATE_ERROR_SIZE;
    } else if (!updateFinishUpload()) {
      update_ok = false;
    }
    return;
  }
  if (upload.status == UPLOAD_FILE_ABORTED) {
    updateAbortUpload(UPDATE_ERROR_STREAM);
  }
}

void handleNotFound() {
  server.sendHeader(F("Location"), F("/"), true);
  sendPlain(302, "");
}

void setupRoutes() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/scan", HTTP_GET, handleScan);
  server.on("/wifi", HTTP_POST, handleWifiSave);
  server.on("/template", HTTP_POST, handleTemplateSave);
  server.on("/power", HTTP_POST, handlePowerSave);
#if MYMOTA32_LIGHT_SUPPORTED
  server.on("/light", HTTP_POST, handleLightSave);
#endif
  server.on("/leds", HTTP_POST, handleLedSave);
  server.on("/relay-enforcement", HTTP_POST, handleDeviceStateEnforcementSave);
  server.on("/relay-pulsing", HTTP_POST, handleRelayPulseSave);
  server.on("/buttons", HTTP_POST, handleButtonSave);
  server.on("/mqtt", HTTP_POST, handleMqttSave);
  server.on("/energy", HTTP_POST, handleEnergySave);
  server.on("/ibeacon", HTTP_POST, handleIBeaconSave);
  server.on("/system", HTTP_POST, handleSystemSave);
  server.on("/reboot", HTTP_GET, handleReboot);
  server.on("/factory-reset", HTTP_POST, handleFactoryReset);
  server.on("/health", HTTP_GET, handleHealth);
  server.on("/cm", HTTP_GET, handleCmnd);
  server.on("/api/settings", HTTP_GET, handleApiSettingsGet);
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  server.onNotFound(handleNotFound);
}

}  // namespace

void idleAfterLoopWork() {
  uint8_t delay_ms = sanitizePowerSavingMode(config.power_saving_mode);
  if (delay_ms == kPowerSavingDeep) delay_ms = 10;
  if (delay_ms > 0 && restart_due_ms == 0 && !update_started) {
    delay(delay_ms);
  } else {
    yield();
  }
}

void setup() {
  delay(20);
  boot_started_ms = millis();
  loadBootRecoveryState();
  loadConfig();
  decodeTemplateConfig();
  loadGracefulRelaySnapshot();
  loadLastRelaySnapshot();
  setupDevicePins();
  setupLightRuntime();
  setupEnergyMonitor();
  connectWifi();
  boot_id = makeBootId();
  setupRoutes();
  server.begin();
}

void loop() {
  const uint32_t loop_started_us = micros();
  server.handleClient();
  maintainBootRecovery();
  maintainWifi();
  maintainWifiDynamicPower();
  server.handleClient();
  maintainDevice();
  maintainLight();
  maintainEnergy();
  server.handleClient();
  maintainMqtt();
  maintainIBeacon();
  server.handleClient();

  if (restartDue()) {
    if (restart_preserve_relays) {
      saveGracefulRelaySnapshot();
    } else {
      clearGracefulRelaySnapshot();
    }
    persistEnergyTotal(true);
    persistLightConfig(true);
    delay(50);
    ESP.restart();
  }
  recordLoopPerf(loop_started_us, micros());
  idleAfterLoopWork();
}
