#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <bootloader_common.h>
#include <esp_chip_info.h>
#include <esp_system.h>
#include <esp_wifi.h>

#ifndef MYMOTA32_VERSION
#define MYMOTA32_VERSION "dev"
#endif

#ifndef MYMOTA32_TARGET
#define MYMOTA32_TARGET "esp32"
#endif

#ifndef MYMOTA32_ESP32_U4WDH
#define MYMOTA32_ESP32_U4WDH 0
#endif

namespace {

constexpr uint32_t kConnectTimeoutMs = 20000;
constexpr uint32_t kWifiReconnectBeginMs = 60000;
constexpr uint32_t kInitialFallbackApMs = 300000;
constexpr uint32_t kApRetryMs = 10000;
constexpr uint32_t kBootRecoveryStableMs = 30000;
constexpr uint8_t kBootRecoveryLimit = 5;
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
constexpr size_t kTemplateJsonDocCapacity = 2048;

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

constexpr uint16_t kButtonHoldDefaultMs = 500;
constexpr uint16_t kButtonHoldMinMs = 100;
constexpr uint16_t kButtonHoldMaxMs = 60000;
constexpr uint16_t kButtonDebounceDefaultMs = 50;
constexpr uint16_t kButtonDebounceMinMs = 5;
constexpr uint16_t kButtonDebounceMaxMs = 200;
constexpr uint32_t kLedUpdateMs = 50;

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
constexpr uint32_t kMqttConnackTimeoutMs = 250;
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
constexpr uint16_t kTplAde7953Irq = 3456;
constexpr uint16_t kTplAdcInput = 4704;
constexpr uint16_t kTplAdcTemp = 4736;
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

const char kTemplateShellyPlusPlugSJson[] PROGMEM =
  "{\"NAME\":\"Shelly Plus Plug S\",\"GPIO\":[0,0,0,0,224,0,32,2720,0,0,0,0,0,0,0,2624,0,0,2656,0,0,288,289,0,0,0,0,0,0,4736,0,0,0,0,0,0],\"FLAG\":0,\"BASE\":1}";
const char kTemplateShellyPlus2PmPcb019Json[] PROGMEM =
  "{\"NAME\":\"Shelly Plus 2PM PCB v0.1.9\",\"GPIO\":[320,0,0,0,34,192,0,0,225,224,0,0,0,0,193,0,0,0,0,0,0,608,640,3458,0,0,0,0,0,9472,0,4736,0,0,0,0],\"FLAG\":0,\"BASE\":1}";
const char kTemplateNousA8tJson[] PROGMEM =
  "{\"NAME\":\"NOUS A8T\",\"GPIO\":[1,1,320,1,32,1,1,1,1,224,2624,1,1,1,1,1,0,1,1,1,0,1,2656,2720,0,0,0,0,1,1,1,1,1,0,0,1],\"FLAG\":0,\"BASE\":1}";
const char kTemplateGenericC3RelayJson[] PROGMEM =
  "{\"NAME\":\"Generic C3 Relay\",\"GPIO\":[32,0,0,0,224,288,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],\"FLAG\":0,\"BASE\":1}";

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

struct RuntimeTemplate {
  bool enabled;
  char name[kTemplateNameMaxLen + 1];
  uint16_t base;
  uint32_t flag;
  PinAssignment relays[kMaxRelays];
  PinAssignment buttons[kMaxButtons];
  uint8_t input_kind[kMaxButtons];
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
  bool energy_sel_inverted;
  bool energy_hjl;
  bool adc_temp;
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
};

StoredConfig config{};
RuntimeTemplate runtime_template{};
bool relay_state[kMaxRelays] = {false};
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

uint32_t boot_recovery_count = 0;
bool boot_recovery_factory_reset = false;
bool boot_recovery_cleared = false;
uint32_t boot_started_ms = 0;

bool update_started = false;
bool update_ok = false;
uint8_t update_error = UPDATE_ERROR_OK;

uint32_t restart_due_ms = 0;
uint32_t restart_scheduled_ms = 0;

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
uint32_t next_mqtt_reconnect = 0;
uint32_t last_mqtt_io = 0;
uint32_t last_mqtt_rx = 0;
uint32_t last_mqtt_ping = 0;
uint32_t last_mqtt_state_publish = 0;
uint32_t last_mqtt_connect_attempt = 0;
uint32_t last_mqtt_connect_duration = 0;
uint8_t last_mqtt_connect_result = kMqttConnectIdle;
uint8_t mqtt_pending_relay_mask = 0;
bool mqtt_ping_pending = false;

MqttButtonPending mqtt_button_queue[kMqttButtonQueueDepth]{};
uint8_t mqtt_button_queue_head = 0;
uint8_t mqtt_button_queue_count = 0;

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

uint8_t activePhyMode() {
  uint8_t protocol = 0;
  if (esp_wifi_get_protocol(WIFI_IF_STA, &protocol) != ESP_OK) return kPhyModeAuto;
  if (protocol & WIFI_PROTOCOL_11N) return kPhyModeN;
  if (protocol & WIFI_PROTOCOL_11G) return kPhyModeG;
  if (protocol & WIFI_PROTOCOL_11B) return kPhyModeB;
  return kPhyModeAuto;
}

String chipIdHex() {
  uint64_t mac = ESP.getEfuseMac();
  uint32_t low24 = static_cast<uint32_t>(mac & 0xFFFFFFULL);
  char buf[7];
  snprintf(buf, sizeof(buf), "%06X", low24);
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

uint8_t defaultInputMode(uint8_t input) {
  return isSwitchInput(input) ? kInputModeSwitch : kInputModeButton;
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
  if (isSwitchInput(input)) return kInputOnLevelHigh;
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
  if (code == kTplAdcTemp || code == kTplAdcInput) {
    if (digitalPinSupported(pin)) target.adc_temp = code == kTplAdcTemp;
    return;
  }
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
    target.buttons[index] = {
      pin,
      base == kTplKey1Inv || base == kTplKey1InvNp,
      base == kTplKey1Np || base == kTplKey1InvNp
    };
    target.input_kind[index] = kInputKindButton;
    if (target.button_count <= index) target.button_count = index + 1;
    return;
  }

  if (base == kTplSwt1 || base == kTplSwt1Np) {
    if (index >= kMaxButtons) {
      addUnsupportedTemplatePin(target, pin, code);
      return;
    }
    target.buttons[index] = {pin, false, base == kTplSwt1Np};
    target.input_kind[index] = kInputKindSwitch;
    if (target.button_count <= index) target.button_count = index + 1;
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
  for (uint8_t i = 0; i < kMaxButtons; i++) resetPinAssignment(target.buttons[i]);
  for (uint8_t i = 0; i < kMaxLeds; i++) resetPinAssignment(target.leds[i]);
  resetPinAssignment(target.link_led);
  target.i2c_scl_pin = kInvalidPin;
  target.i2c_sda_pin = kInvalidPin;
  target.energy_cf_pin = kInvalidPin;
  target.energy_cf1_pin = kInvalidPin;
  target.energy_sel_pin = kInvalidPin;
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
}

void decodeTemplateConfig() {
  decodeTemplateConfigInto(config, runtime_template);
}

bool isJsonSpace(char c) {
  return c == ' ' || c == '\n' || c == '\r' || c == '\t';
}

bool templateJsonHasSingleRootObject(const String &json) {
  const char *p = json.c_str();
  while (isJsonSpace(*p)) p++;
  if (*p != '{') return false;

  uint16_t depth = 0;
  bool in_string = false;
  bool escaped = false;
  for (; *p; p++) {
    const char c = *p;
    if (in_string) {
      if (escaped) escaped = false;
      else if (c == '\\') escaped = true;
      else if (c == '"') in_string = false;
      continue;
    }
    if (c == '"') in_string = true;
    else if (c == '{' || c == '[') depth++;
    else if (c == '}' || c == ']') {
      if (depth == 0) return false;
      depth--;
      if (depth == 0) {
        p++;
        break;
      }
    }
  }
  if (depth != 0 || in_string || escaped) return false;
  while (isJsonSpace(*p)) p++;
  return *p == '\0';
}

bool parseTemplateJson(const String &json, StoredConfig &target, String &error) {
  if (json.length() < 9 || json.length() > kTemplateJsonMaxLen) {
    error = F("Template JSON length is invalid");
    return false;
  }
  if (!templateJsonHasSingleRootObject(json)) {
    error = F("Template must be one complete JSON object");
    return false;
  }

  DynamicJsonDocument doc(kTemplateJsonDocCapacity);
  const DeserializationError json_error = deserializeJson(doc, json);
  if (json_error) {
    error = F("Template JSON parse failed: ");
    error += json_error.c_str();
    return false;
  }
  if (!doc.is<JsonObject>()) {
    error = F("Template must be a JSON object");
    return false;
  }

  const char *name = doc["NAME"] | "";
  if (name[0] == '\0') {
    error = F("Template NAME is empty");
    return false;
  }
  if (strlen(name) >= sizeof(target.template_name)) {
    error = F("Template NAME is too long");
    return false;
  }

  JsonArray gpio_values = doc["GPIO"].as<JsonArray>();
  const size_t gpio_count = gpio_values.size();
  if (gpio_values.isNull() ||
      (gpio_count != kTemplateGpioCount && gpio_count != kTemplateJsonMinGpioCount)) {
    error = F("GPIO entry count is invalid for this target");
    return false;
  }

  uint16_t gpio[kTemplateGpioCount]{};
  for (uint8_t i = 0; i < gpio_count; i++) {
    JsonVariant value = gpio_values[i];
    if (!value.is<uint16_t>()) {
      error = F("Invalid GPIO value");
      return false;
    }
    gpio[i] = value.as<uint16_t>();
  }

  JsonVariant base_value = doc["BASE"];
  if (!base_value.is<uint16_t>() || base_value.as<uint16_t>() == 0) {
    error = F("Template BASE is invalid");
    return false;
  }
  JsonVariant flag_value = doc["FLAG"];
  if (!flag_value.isNull() && !flag_value.is<uint32_t>()) {
    error = F("Template FLAG is invalid");
    return false;
  }

  target.template_enabled = 1;
  target.template_base = base_value.as<uint16_t>();
  target.template_flag = flag_value.isNull() ? 0 : flag_value.as<uint32_t>();
  strlcpy(target.template_name, name, sizeof(target.template_name));
  memcpy(target.template_gpio, gpio, sizeof(target.template_gpio));
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

void scheduleRestart(uint32_t delay_ms) {
  restart_due_ms = millis() + delay_ms;
  restart_scheduled_ms = millis();
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

  config_ok = config.ssid[0] != '\0';
  return config_ok;
}

bool saveWifiConfig(const char *ssid, const char *password, const char *hostname, uint8_t phy_mode) {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putString("ssid", ssid);
  prefs.putString("password", password);
  if (hostname && hostname[0]) prefs.putString("hostname", hostname);
  else prefs.putString("hostname", defaultHostname());
  prefs.putUChar("phy", sanitizePhyMode(phy_mode));
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
  last_mqtt_connect_attempt = 0;
  last_mqtt_connect_duration = 0;
  last_mqtt_connect_result = kMqttConnectIdle;
  mqtt_pending_relay_mask = 0;
  mqtt_ping_pending = false;
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
  WiFi.begin(config.ssid, config.password);
  last_wifi_begin_attempt = now;
}

void prepareWifi() {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(config.hostname);
  WiFi.setSleep(false);
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
bool mqttConfigured();
bool parseUint16Input(const String &input, uint16_t min_value, uint16_t max_value, uint16_t &out);

void setRelay(uint8_t relay, bool on) {
  if (relay >= kMaxRelays || !hasPin(runtime_template.relays[relay])) return;
  const bool changed = relay_state[relay] != on;
  relay_state[relay] = on;
  writeAssignedPin(runtime_template.relays[relay], on);
  if (changed) {
    updateDeviceLeds(true);
    scheduleMqttRelayPublish(relay);
  }
}

void toggleRelay(uint8_t relay) {
  if (relay >= kMaxRelays) return;
  setRelay(relay, !relay_state[relay]);
}

void setupDevicePins() {
  for (uint8_t i = 0; i < kMaxRelays; i++) {
    relay_state[i] = false;
    if (!hasPin(runtime_template.relays[i])) continue;
    writeAssignedPin(runtime_template.relays[i], false);
    pinMode(runtime_template.relays[i].pin, OUTPUT);
    writeAssignedPin(runtime_template.relays[i], false);
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
    if (effectiveInputMode(i) == kInputModeSwitch) {
      uint8_t relay = 0;
      if (inputRelayTarget(i, relay)) setRelay(relay, active);
    }
  }
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
      out += String(button + 1);
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

bool runWebhookAction(uint8_t button, bool hold) {
  if (WiFi.status() != WL_CONNECTED) return false;
  const String url = expandButtonActionText(buttonActionTarget(button, hold), button, hold);
  String host;
  uint16_t port = 80;
  String path;
  if (!parseHttpUrl(url, host, port, path)) return false;
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
  client.flush();
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
    mqttQueueButtonAction(button, hold);
  } else if (action == kButtonActionWebhook) {
    runWebhookAction(button, hold);
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

void maintainDevice() {
  maintainButtons();
  updateDeviceLeds();
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

bool mqttPublishCommandResult(const String &payload) {
  if (payload.length() == 0) return true;
  String topic;
  topic.reserve(strlen(config.mqtt_topic) + 14);
  topic += F("stat/");
  topic += config.mqtt_topic;
  topic += F("/RESULT");
  return mqttPublish(topic.c_str(), payload.c_str());
}

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

  uint8_t relay = 0;
  char response_key[12];
  if (parsePowerCommand(raw, cmd_len, relay, response_key, sizeof(response_key))) {
    if (relay >= kMaxRelays || !hasPin(runtime_template.relays[relay])) {
      error = F("Invalid relay");
      return false;
    }
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

  if (config.mqtt_keepalive > 0 && runtime_template.relay_count > 0) {
    const uint32_t interval_ms = static_cast<uint32_t>(config.mqtt_keepalive) * 1000UL;
    if (now - last_mqtt_state_publish >= interval_ms) {
      mqttPublishAllRelayStates();
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
    default: return F("unknown");
  }
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
  page += F(".panel h2{font-size:17px;margin:0 0 12px}.panel-title{display:flex;align-items:center;justify-content:space-between;gap:12px;margin:0 0 12px}.panel-title h2{margin:0}.kv{display:grid;grid-template-columns:minmax(110px,42%) 1fr;gap:8px 12px}.kv span,.hint{color:var(--muted)}.kv div{min-width:0}");
  page += F("code{background:#eef2f6;border:1px solid #dce3ea;border-radius:4px;padding:1px 4px;word-break:break-word}.pill{display:inline-block;border-radius:999px;padding:2px 8px;background:#eef2f6;color:#364152}.pill.ok{background:var(--ok);color:#fff}.pill.bad{background:var(--bad);color:#fff}.panel h2 .pill{font-size:13px;font-weight:400;vertical-align:1px}.ok{color:var(--ok)}.bad{color:var(--bad)}.muted{color:var(--muted)}");
  page += F(".tokens{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:8px}.tokens div{display:flex;flex-direction:column;gap:3px}.help{position:relative;margin-left:auto}.help-q{display:inline-flex;align-items:center;justify-content:center;width:24px;height:24px;border:1px solid var(--line);border-radius:50%;background:#eef2f6;color:var(--accent2);font-size:14px;font-weight:700;cursor:help}.help-box{display:none;position:absolute;right:0;top:30px;z-index:30;width:520px;max-width:calc(100vw - 48px);background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:12px;box-shadow:0 8px 24px rgba(0,0,0,.18);color:var(--text);font-size:14px;font-weight:400;line-height:1.4}.help:hover .help-box,.help:focus-within .help-box{display:block}.help-box p{margin:0 0 8px}.button-block{border-top:1px solid var(--line);margin-top:12px;padding-top:12px}.action-extra,.mode-extra{display:none}.action-extra.show,.mode-extra.show{display:block}.hidden{display:none}");
  page += F("form{margin:0}.row{margin:10px 0}label{display:block;font-weight:600;color:#344054}input,button,select,textarea{font:inherit}input,select,textarea{width:100%;margin-top:4px;padding:9px;border:1px solid #b9c4d0;border-radius:6px;background:#fff}input[type=checkbox]{width:auto;margin:0 6px 0 0;padding:0;vertical-align:-1px}textarea{min-height:92px;resize:vertical}");
  page += F("button,.btn{display:inline-block;margin:4px 4px 0 0;padding:8px 12px;border:1px solid var(--accent);border-radius:6px;background:var(--accent);color:#fff;text-decoration:none;cursor:pointer}.secondary{background:#fff;color:var(--accent2);border-color:#9eb7cf}.danger{background:#fff;color:var(--bad);border-color:#d4aaa7}.inline{display:inline}.actions{display:flex;flex-wrap:wrap;gap:6px}.inline button{margin:0 4px 0 0}.list{margin:0;padding-left:18px}@media(max-width:520px){.kv{grid-template-columns:1fr}.brand{font-size:24px}}</style></head><body>");
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
  page += F("function live(){fh().then(function(d){");
  page += F("t('live-heap',d.heap+' bytes');t('live-uptime',d.uptime+'s');t('live-active-phy',d.active_phy);");
  page += F("if(d.perf){t('live-loop-load',d.perf.loop_load+'%');t('live-loop-hz',d.perf.loop_hz+'/s');t('live-loop-max',Number(d.perf.loop_max_us/1000).toFixed(1)+' ms');}");
  page += F("t('live-recovery',d.recovery.fast_boot_count+'/'+d.recovery.limit);");
  page += F("p('live-wifi',d.wifi?'connected':'disconnected',d.wifi?'pill ok':'pill bad');t('live-ssid',d.wifi_ssid||'n/a');t('live-ip',d.ip||'n/a');t('live-rssi',d.rssi==null?'n/a':d.rssi+' dBm');");
  page += F("p('live-mqtt',d.mqtt.enabled?(d.mqtt.connected?'connected':'disconnected'):'not configured',d.mqtt.enabled?(d.mqtt.connected?'pill ok':'pill bad'):'pill');");
  page += F("if(d.mqtt){t('live-mqtt-pending',d.mqtt.pending);t('live-mqtt-result',d.mqtt.last_connect_result);t('live-mqtt-connect-ms',d.mqtt.last_connect_ms+' ms');t('live-mqtt-attempt',d.mqtt.last_attempt_ms_ago==null?'n/a':d.mqtt.last_attempt_ms_ago+' ms ago');}");
  page += F("if(d.power){for(var i=0;i<d.power.length;i++){if(d.power[i]!==null)p('live-relay-'+i,d.power[i]?'on':'off',d.power[i]?'pill ok':'pill bad');}}");
  page += F("if(d.buttons){for(var b=0;b<d.buttons.length;b++){if(d.buttons[b])p('live-button-'+b,d.buttons[b].state||(d.buttons[b].pressed?'pressed':'released'),d.buttons[b].pressed?'pill ok':'pill bad');}}");
  page += F("if(d.leds){for(var l=0;l<d.leds.length;l++){if(d.leds[l])p('live-led-'+l,d.leds[l].on?'on':'off',d.leds[l].on?'pill ok':'pill bad');}}");
  page += F("}).catch(function(){});}");
  page += F("function ba(s){var k=s.getAttribute('data-key'),v=s.value,b=document.getElementById('extra-'+k);if(!b)return;var t=b.querySelector('.target-input'),p=b.querySelector('.payload-input'),rr=b.querySelector('.relay-row'),tr=b.querySelector('.target-row'),pr=b.querySelector('.payload-row'),tl=b.querySelector('.target-label'),h=b.querySelector('.action-hint');b.className=(v=='1'||v=='2'||v=='3')?'action-extra show':'action-extra';if(rr)rr.className=v=='1'?'row relay-row':'row relay-row hidden';if(tr)tr.className=(v=='2'||v=='3')?'row target-row':'row target-row hidden';if(pr)pr.className=(v=='2')?'row payload-row':'row payload-row hidden';if(v=='1'){if(h)h.textContent='Toggles the configured relay.';}else if(v=='2'){if(t&&(!t.value||t.value.indexOf('http://')==0))t.value=t.getAttribute('data-default-topic');if(p&&!p.value)p.value=p.getAttribute('data-default-payload');if(tl)tl.textContent='MQTT topic';if(h)h.textContent='Publishes this topic and payload through the configured MQTT broker.';}else if(v=='3'){if(tl)tl.textContent='Webhook URL';if(h)h.textContent='Executes an HTTP GET request; only http:// URLs are supported.';}}");
  page += F("function im(s){var k=s.getAttribute('data-input'),v=s.value,b=document.getElementById('input-button-'+k),w=document.getElementById('input-switch-'+k);if(b)b.className=v=='0'?'mode-extra show':'mode-extra';if(w)w.className=v=='1'?'mode-extra show':'mode-extra';}");
  page += F("function ts(){var s=document.getElementById('known-template'),t=document.getElementById('template-json');if(!s||!t)return;var v=t.value.trim(),m=0;for(var i=1;i<s.options.length;i++){if(s.options[i].getAttribute('data-json')==v){m=i;break;}}s.selectedIndex=m;}");
  page += F("function tp(s){var o=s.options[s.selectedIndex],t=document.getElementById('template-json');if(o&&t&&o.getAttribute('data-json')){t.value=o.getAttribute('data-json');ts();}}");
  page += F("function bi(){var a=document.querySelectorAll('.button-action');for(var i=0;i<a.length;i++){a[i].onchange=function(){ba(this)};ba(a[i]);}var m=document.querySelectorAll('.input-mode');for(var j=0;j<m.length;j++){m[j].onchange=function(){im(this)};im(m[j]);}var t=document.getElementById('template-json');if(t){t.oninput=ts;t.onchange=ts;}ts();}bi();");
  page += F("document.addEventListener('click',function(e){var b=e.target;while(b&&b.tagName!='BUTTON'&&b.tagName!='INPUT')b=b.parentNode;if(!b||!b.form)return;var t=(b.type||'').toLowerCase();if(t=='submit'||t=='image')b.form._s=b;},true);");
  page += F("document.addEventListener('submit',function(e){var f=e.target;if(!f||f.getAttribute('data-inline')!='1')return;e.preventDefault();var fd=new FormData(f),b=e.submitter||f._s;if(b&&b.name)fd.append(b.name,b.value);fd.append('_inline','1');fetch(f.getAttribute('action')||location.pathname,{method:(f.method||'POST').toUpperCase(),body:fd,cache:'no-store'}).then(function(r){if(!r.ok)return r.text().then(function(x){throw Error(x||r.statusText)});live();}).catch(function(x){alert(x.message||x);});},true);");
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
  page += String(static_cast<float>(perf_last_loop_max_us) / 1000.0f, 1);
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
  page += F("</div><span>MQTT broker</span><div>");
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
  page += String(mqtt_pending_relay_mask);
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
    if (runtime_template.i2c_scl_pin != kInvalidPin || runtime_template.i2c_sda_pin != kInvalidPin) {
      page += F("<span>I2C</span><div>SCL <code>");
      page += pinName(runtime_template.i2c_scl_pin);
      page += F("</code>, SDA <code>");
      page += pinName(runtime_template.i2c_sda_pin);
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
  if (!runtime_template.enabled || runtime_template.relay_count == 0) return;
  page += F("<section class='panel'><h2>Device</h2>");
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
  page += F("<select class='button-action' data-key='");
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
  page += F("' class='action-extra'>");
  if (has_relay_target) {
    page += F("<div class='row relay-row'><label>Target relay<br><select name='");
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
    page += F("<div class='row relay-row'><span class='hint'>No relay available.</span></div>");
  }
  page += F("<div class='row target-row'><label><span class='target-label'>MQTT topic</span><br><input class='target-input' name='");
  page += name;
  page += F("_target");
  page += String(button);
  page += F("' maxlength='");
  page += String(kButtonActionTargetMaxLen);
  page += F("' data-default-topic='");
  page += htmlEscape(kDefaultButtonMqttTopic);
  page += F("' value='");
  page += htmlEscape(buttonActionTarget(button, hold));
  page += F("'></label></div><div class='row payload-row'><label>MQTT payload<br><textarea class='payload-input' name='");
  page += name;
  page += F("_payload");
  page += String(button);
  page += F("' maxlength='");
  page += String(kButtonActionPayloadMaxLen);
  page += F("' data-default-payload='");
  page += htmlEscape(hold ? kDefaultButtonMqttHoldPayload : kDefaultButtonMqttPressPayload);
  page += F("'>");
  page += htmlEscape(buttonActionPayload(button, hold));
  page += F("</textarea></label></div><p class='hint action-hint'></p></div>");
}

bool inputCanFollowOutput(uint8_t input) {
  uint8_t relay = 0;
  return defaultButtonRelayTarget(input, relay);
}

String inputDisplayName(uint8_t input) {
  String name = isSwitchInput(input) ? F("Switch ") : F("Button ");
  name += String(input + 1);
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
    page += F("<div class='button-block'><strong>");
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

    page += F("<div class='row'><label>Kind<br><select class='input-mode' data-input='");
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
    page += F("' class='mode-extra");
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
    page += F("' class='mode-extra");
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
  page += F("<option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateGenericC3RelayJson)));
  page += F("'>Generic C3 Relay</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateNousA8tJson)));
  page += F("'>NOUS A8T</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateShellyPlus2PmPcb019Json)));
  page += F("'>Shelly Plus 2PM PCB v0.1.9</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateShellyPlusPlugSJson)));
  page += F("'>Shelly Plus Plug S</option></select></label></div>");
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

void handleRoot() {
  String page;
  page.reserve(6500);
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
  appendMqttForm(page);
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
  page += F("<button type='submit'>Save Wi-Fi</button></form>");
  page += F("<p><a class='btn secondary' href='/scan'>Scan networks</a></p></section>");
  flushStreamChunk(page);

  page += F("<section class='panel'><h2>Firmware</h2><form method='post' action='/update' enctype='multipart/form-data'>");
  page += F("<input type='file' name='firmware' accept='.bin' required><br><button type='submit'>Upload firmware</button></form>");
  page += F("<p><a class='btn secondary' href='/reboot'>Reboot</a></p>");
  page += F("<form method='post' action='/factory-reset' onsubmit=\"return confirm('Factory reset will delete Wi-Fi settings. Continue?')\"><button class='danger' type='submit'>Factory reset</button></form></section>");
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
  char password_to_save[sizeof(config.password)];

  if (ssid.length() == 0 || ssid.length() > 32 || password.length() > 64 || hostname.length() > 32) {
    server.send(400, F("text/plain"), F("Invalid Wi-Fi settings"));
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
  if (!saveWifiConfig(ssid.c_str(), password_to_save, hostname.c_str(), phy_mode)) {
    server.send(500, F("text/plain"), F("Could not save Wi-Fi settings"));
    return;
  }
  String page;
  page.reserve(800);
  appendHeader(page, F("myMota32 Wi-Fi"));
  page += F("<p class='ok'>Wi-Fi settings saved. Rebooting.</p>");
  page += F("<p>The page will return to the dashboard when the device is reachable again.</p>");
  page += F("<p class='muted'>If Wi-Fi or IP changed, reconnect to the device manually.</p>");
  appendFooter(page, false, true);
  sendHtml(page);
  scheduleRestart(1200);
}

void handleTemplateSave() {
  if (server.hasArg("clear")) {
    StoredConfig candidate = config;
    clearTemplateConfig(candidate);
    if (!saveTemplateConfig(candidate)) {
      server.send(500, F("text/plain"), F("Could not clear template"));
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
    server.send(400, F("text/plain"), F("Template JSON is empty"));
    return;
  }
  StoredConfig candidate = config;
  String error;
  if (!parseTemplateJson(template_json, candidate, error)) {
    String msg = F("Invalid template: ");
    msg += error;
    msg += '\n';
    server.send(400, F("text/plain"), msg);
    return;
  }
  if (!saveTemplateConfig(candidate)) {
    server.send(500, F("text/plain"), F("Could not save template"));
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
    server.send(400, F("text/plain"), F("Missing relay or state"));
    return;
  }
  const int relay = server.arg("relay").toInt();
  const String state = server.arg("state");
  if (relay < 1 || relay > kMaxRelays || !hasPin(runtime_template.relays[relay - 1])) {
    server.send(400, F("text/plain"), F("Invalid relay"));
    return;
  }
  if (state == "on") setRelay(relay - 1, true);
  else if (state == "off") setRelay(relay - 1, false);
  else if (state == "toggle") toggleRelay(relay - 1);
  else { server.send(400, F("text/plain"), F("Invalid relay state")); return; }
  updateDeviceLeds(true);
  if (server.hasArg("_inline")) { server.send(204, F("text/plain"), ""); return; }
  server.sendHeader(F("Location"), F("/"), true);
  server.send(303, F("text/plain"), "");
}

void handleLedSave() {
  if (!hasConfigurableLedOutputs()) {
    server.send(400, F("text/plain"), F("No configurable LEDs are available"));
    return;
  }
  uint8_t attachments[kMaxLedOutputs];
  memcpy(attachments, config.led_attach, sizeof(attachments));
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    if (!hasLedOutput(i)) continue;
    String arg_name = F("led");
    arg_name += String(i);
    if (!server.hasArg(arg_name)) {
      server.send(400, F("text/plain"), F("Missing LED setting"));
      return;
    }
    const long raw_value = server.arg(arg_name).toInt();
    if (raw_value < 0 || raw_value > 255) {
      server.send(400, F("text/plain"), F("Invalid LED setting"));
      return;
    }
    const uint8_t attachment = static_cast<uint8_t>(raw_value);
    if (!ledAttachmentAvailable(attachment)) {
      server.send(400, F("text/plain"), F("Invalid LED attachment"));
      return;
    }
    attachments[i] = attachment;
  }
  if (!saveLedAttachments(attachments)) {
    server.send(500, F("text/plain"), F("Could not save LED settings"));
    return;
  }
  updateDeviceLeds(true);
  if (server.hasArg("_inline")) { server.send(204, F("text/plain"), ""); return; }
  server.sendHeader(F("Location"), F("/"), true);
  server.send(303, F("text/plain"), "");
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
    server.send(400, F("text/plain"), F("No configurable inputs are available"));
    return;
  }

  uint16_t hold_ms = kButtonHoldDefaultMs;
  if (!parseUint16Input(server.arg("hold_ms"), kButtonHoldMinMs, kButtonHoldMaxMs, hold_ms)) {
    server.send(400, F("text/plain"), F("Invalid input hold time"));
    return;
  }
  uint16_t debounce_ms = kButtonDebounceDefaultMs;
  if (!parseUint16Input(server.arg("debounce_ms"), kButtonDebounceMinMs, kButtonDebounceMaxMs, debounce_ms)) {
    server.send(400, F("text/plain"), F("Invalid input debounce time"));
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
      server.send(400, F("text/plain"), F("Missing input mode"));
      return;
    }
    uint16_t mode_value = 0;
    if (!parseUint16Input(server.arg(mode_arg), 0, 1, mode_value)) {
      server.send(400, F("text/plain"), F("Invalid input mode"));
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
        server.send(400, F("text/plain"), F("Missing switch setting"));
        return;
      }
      uint16_t relay_value = 0;
      uint16_t reverse_value = 0;
      if ((has_relay_target && !parseUint16Input(server.arg(relay_arg), 0, kMaxRelays - 1, relay_value)) ||
          !parseUint16Input(server.arg(reverse_arg), 0, 1, reverse_value)) {
        server.send(400, F("text/plain"), F("Invalid switch setting"));
        return;
      }
      if (has_relay_target) {
        const uint8_t relay = static_cast<uint8_t>(relay_value);
        if (!relayAvailable(relay)) {
          server.send(400, F("text/plain"), F("Invalid switch relay"));
          return;
        }
        candidate.input_relay[i] = relay;
      } else {
        server.send(400, F("text/plain"), F("Invalid switch target"));
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
      server.send(400, F("text/plain"), F("Missing button action setting"));
      return;
    }
    uint16_t press_value = 0;
    uint16_t hold_value = 0;
    if (!parseUint16Input(server.arg(press_arg), 0, 255, press_value) ||
        !parseUint16Input(server.arg(hold_arg), 0, 255, hold_value)) {
      server.send(400, F("text/plain"), F("Invalid button action setting"));
      return;
    }

    const uint8_t press_action = static_cast<uint8_t>(press_value);
    const uint8_t hold_action = static_cast<uint8_t>(hold_value);
    if (!isButtonActionEncoding(press_action) || !isButtonActionEncoding(hold_action) ||
        !buttonActionAvailable(i, press_action) || !buttonActionAvailable(i, hold_action)) {
      server.send(400, F("text/plain"), F("Invalid button action"));
      return;
    }
    candidate.button_press_action[i] = press_action;
    candidate.button_hold_action[i] = hold_action;

    String error;
    if (!readButtonRelayTargetInput(i, "press", press_action, candidate.button_press_relay, error) ||
        !readButtonRelayTargetInput(i, "hold", hold_action, candidate.button_hold_relay, error) ||
        !readButtonEventText(i, "press", false, press_action, candidate.button_press_target, candidate.button_press_payload, error) ||
        !readButtonEventText(i, "hold", true, hold_action, candidate.button_hold_target, candidate.button_hold_payload, error)) {
      server.send(400, F("text/plain"), error);
      return;
    }
  }

  if (!saveInputConfig(candidate)) {
    server.send(500, F("text/plain"), F("Could not save input settings"));
    return;
  }
  for (uint8_t i = 0; i < runtime_template.button_count; i++) {
    if (effectiveInputMode(i) == kInputModeSwitch && hasPin(runtime_template.buttons[i])) {
      uint8_t target = 0;
      if (inputRelayTarget(i, target)) setRelay(target, readInputActive(i));
    }
  }
  updateDeviceLeds(true);
  if (server.hasArg("_inline")) { server.send(204, F("text/plain"), ""); return; }
  server.sendHeader(F("Location"), F("/"), true);
  server.send(303, F("text/plain"), "");
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
    server.send(400, F("text/plain"), F("Invalid MQTT host"));
    return;
  }
  if (!parseUint16Input(port_arg, 1, 65535U, port)) {
    server.send(400, F("text/plain"), F("Invalid MQTT port"));
    return;
  }
  if (!isValidMqttTopic(topic)) {
    server.send(400, F("text/plain"), F("Invalid MQTT topic"));
    return;
  }
  if (!parseUint16Input(keepalive_arg, 0, kMqttKeepaliveMax, keepalive)) {
    server.send(400, F("text/plain"), F("Invalid MQTT keepalive"));
    return;
  }

  if (!saveMqttConfig(host.c_str(), port, topic.c_str(), keepalive)) {
    server.send(500, F("text/plain"), F("Could not save MQTT settings"));
    return;
  }

  if (server.hasArg("_inline")) { server.send(204, F("text/plain"), ""); return; }
  String page;
  page.reserve(700);
  appendHeader(page, F("myMota32 MQTT"));
  page += F("<p class='ok'>MQTT settings saved.</p>");
  page += F("<p><a href='/'>Back</a></p>");
  appendFooter(page);
  sendHtml(page);
}

void handleReboot() {
  String page;
  page.reserve(700);
  appendHeader(page, F("myMota32 Reboot"));
  page += F("<p class='ok'>Rebooting.</p>");
  page += F("<p>The page will return to the dashboard when the device is reachable again.</p>");
  appendFooter(page, false, true);
  sendHtml(page);
  scheduleRestart(500);
}

void handleFactoryReset() {
  if (!factoryResetConfig()) {
    server.send(500, F("text/plain"), F("Could not factory reset settings"));
    return;
  }
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
  String out;
  out.reserve(1400);
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
  out += F("],\"mqtt\":{\"enabled\":");
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
  out += mqtt_pending_relay_mask;
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
    server.send(400, F("text/plain"), F("Missing cmnd"));
    return;
  }

  const String cmnd_str = server.arg("cmnd");
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
    server.send(400, F("text/plain"), F("Invalid cmnd"));
    return;
  }

  const size_t arg_len = arg_start < total_len ? total_len - arg_start : 0;
  String out;
  String error;
  if (!executeDeviceCommand(raw, cmd_len, raw + arg_start, arg_len, out, error)) {
    server.send(400, F("text/plain"), error);
    return;
  }

  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("application/json"), out);
}

void handleUpdateDone() {
  if (update_ok && !Update.hasError()) {
    String page;
    page.reserve(700);
    appendHeader(page, F("myMota32 Update"));
    page += F("<p class='ok'>Firmware uploaded. Rebooting.</p>");
    page += F("<p>The page will return to the dashboard when the device is reachable again.</p>");
    appendFooter(page, false, true);
    sendHtml(page);
    scheduleRestart(1200);
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
    update_started = false;
    update_ok = false;
    update_error = UPDATE_ERROR_OK;
    if (upload.filename.length() == 0) update_error = UPDATE_ERROR_SIZE;
    return;
  }
  if (upload.status == UPLOAD_FILE_WRITE && update_error != UPDATE_ERROR_OK) return;
  if (upload.status == UPLOAD_FILE_WRITE) {
    if (!update_started && upload.totalSize == 0) {
      if (upload.currentSize < 4) { update_error = UPDATE_ERROR_SIZE; return; }
      if (upload.buf[0] != 0xE9) { update_error = UPDATE_ERROR_MAGIC_BYTE; return; }
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { update_error = Update.getError(); return; }
      update_started = true;
    }
    if (Update.hasError()) { update_error = Update.getError(); return; }
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) update_error = Update.getError();
    return;
  }
  if (upload.status == UPLOAD_FILE_END) {
    if (update_error != UPDATE_ERROR_OK) {
      if (update_started) { Update.abort(); update_started = false; }
    } else if (!update_started) {
      update_error = UPDATE_ERROR_SIZE;
    } else if (Update.end(true)) {
      update_ok = true;
      update_started = false;
    } else {
      update_error = Update.getError();
      update_started = false;
    }
    return;
  }
  if (upload.status == UPLOAD_FILE_ABORTED) {
    if (update_started) Update.abort();
    update_started = false;
    update_ok = false;
    update_error = UPDATE_ERROR_STREAM;
  }
}

void handleNotFound() {
  server.sendHeader(F("Location"), F("/"), true);
  server.send(302, F("text/plain"), "");
}

void setupRoutes() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/scan", HTTP_GET, handleScan);
  server.on("/wifi", HTTP_POST, handleWifiSave);
  server.on("/template", HTTP_POST, handleTemplateSave);
  server.on("/power", HTTP_POST, handlePowerSave);
  server.on("/leds", HTTP_POST, handleLedSave);
  server.on("/buttons", HTTP_POST, handleButtonSave);
  server.on("/mqtt", HTTP_POST, handleMqttSave);
  server.on("/reboot", HTTP_GET, handleReboot);
  server.on("/factory-reset", HTTP_POST, handleFactoryReset);
  server.on("/health", HTTP_GET, handleHealth);
  server.on("/cm", HTTP_GET, handleCmnd);
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  server.onNotFound(handleNotFound);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(20);
  Serial.println();
  boot_started_ms = millis();
  loadBootRecoveryState();
  loadConfig();
  decodeTemplateConfig();
  setupDevicePins();
  Serial.printf("myMota32 %s %s chip %s\n", MYMOTA32_VERSION, MYMOTA32_TARGET, chipDisplayName().c_str());
  if (runtime_template.enabled) {
    Serial.printf("Template '%s' base %u relays %u buttons %u leds %u unsupported %u\n",
                  runtime_template.name, runtime_template.base, runtime_template.relay_count,
                  runtime_template.button_count, runtime_template.led_count,
                  runtime_template.unsupported_count);
  }
  connectWifi();
  boot_id = makeBootId();
  setupRoutes();
  server.begin();
  Serial.printf("HTTP server started; STA %s AP %s\n",
                WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "not-connected",
                ap_started ? WiFi.softAPIP().toString().c_str() : "off");
}

void loop() {
  const uint32_t loop_started_us = micros();
  server.handleClient();
  maintainBootRecovery();
  maintainWifi();
  server.handleClient();
  maintainDevice();
  server.handleClient();
  maintainMqtt();
  server.handleClient();

  if (restartDue()) {
    delay(50);
    ESP.restart();
  }
  recordLoopPerf(loop_started_us, micros());
  yield();
}
