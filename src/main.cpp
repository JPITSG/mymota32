#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_system.h>
#include <esp_wifi.h>

#ifndef MYMOTA32_VERSION
#define MYMOTA32_VERSION "dev"
#endif

#ifndef MYMOTA32_TARGET
#define MYMOTA32_TARGET "esp32"
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

struct StoredConfig {
  char ssid[kSsidMaxLen + 1];
  char password[kPasswordMaxLen + 1];
  char hostname[kHostnameMaxLen + 1];
  uint8_t phy_mode;
};

StoredConfig config{};
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

void scheduleRestart(uint32_t delay_ms) {
  restart_due_ms = millis() + delay_ms;
  restart_scheduled_ms = millis();
}

bool restartDue() {
  return restart_due_ms != 0 && (int32_t)(millis() - restart_due_ms) >= 0;
}

void setDefaultConfig() {
  memset(&config, 0, sizeof(config));
  strlcpy(config.hostname, defaultHostname().c_str(), sizeof(config.hostname));
  config.phy_mode = kPhyModeAuto;
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
  prefs.end();

  strlcpy(config.ssid, ssid.c_str(), sizeof(config.ssid));
  strlcpy(config.password, password.c_str(), sizeof(config.password));
  if (hostname.length() > 0) {
    strlcpy(config.hostname, hostname.c_str(), sizeof(config.hostname));
  }
  config.phy_mode = sanitizePhyMode(phy);
  config_ok = config.ssid[0] != '\0';
  return config_ok;
}

bool saveWifiConfig(const char *ssid, const char *password, const char *hostname, uint8_t phy_mode) {
  if (!prefs.begin("mymota32", false)) return false;
  prefs.putString("ssid", ssid);
  prefs.putString("password", password);
  if (hostname && hostname[0]) {
    prefs.putString("hostname", hostname);
  } else {
    prefs.putString("hostname", defaultHostname());
  }
  prefs.putUChar("phy", sanitizePhyMode(phy_mode));
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
  page += F(".panel h2{font-size:17px;margin:0 0 12px}.kv{display:grid;grid-template-columns:minmax(110px,42%) 1fr;gap:8px 12px}.kv span,.hint{color:var(--muted)}.kv div{min-width:0}");
  page += F("code{background:#eef2f6;border:1px solid #dce3ea;border-radius:4px;padding:1px 4px;word-break:break-word}.pill{display:inline-block;border-radius:999px;padding:2px 8px;background:#eef2f6;color:#364152}.pill.ok{background:var(--ok);color:#fff}.pill.bad{background:var(--bad);color:#fff}.ok{color:var(--ok)}.bad{color:var(--bad)}.muted{color:var(--muted)}");
  page += F("form{margin:0}.row{margin:10px 0}label{display:block;font-weight:600;color:#344054}input,button,select,textarea{font:inherit}input,select,textarea{width:100%;margin-top:4px;padding:9px;border:1px solid #b9c4d0;border-radius:6px;background:#fff}input[type=checkbox]{width:auto;margin:0 6px 0 0;padding:0;vertical-align:-1px}");
  page += F("button,.btn{display:inline-block;margin:4px 4px 0 0;padding:8px 12px;border:1px solid var(--accent);border-radius:6px;background:var(--accent);color:#fff;text-decoration:none;cursor:pointer}.secondary{background:#fff;color:var(--accent2);border-color:#9eb7cf}.danger{background:#fff;color:var(--bad);border-color:#d4aaa7}.list{margin:0;padding-left:18px}@media(max-width:520px){.kv{grid-template-columns:1fr}.brand{font-size:24px}}</style></head><body>");
  page += F("<header class='top'><div class='topin'><div><a class='brand' href='/'>my<span>Mota32</span></a><div class='sub'>ESP32 firmware</div></div><div class='sub meta'><span>");
  page += F(MYMOTA32_VERSION);
  page += F(" / ");
  page += F(MYMOTA32_TARGET);
  page += F("</span>");
  if (show_spinner) {
    page += F("<span id='poll-spin' class='spin active'></span>");
  }
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
  page += F("}).catch(function(){});}");
  if (live_poll) {
    page += F("setInterval(live,1000);setInterval(ck,1000);live();");
  }
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
  page += chipIdHex();
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
  page += F("</div><span>Wi-Fi</span><div><span id='live-wifi' class='pill");
  page += (WiFi.status() == WL_CONNECTED) ? F(" ok'>connected") : F(" bad'>disconnected");
  page += F("</span></div><span>SSID</span><div><code id='live-ssid'>");
  page += (WiFi.status() == WL_CONNECTED) ? htmlEscape(WiFi.SSID()) : String(F("n/a"));
  page += F("</code></div><span>IP</span><div><code id='live-ip'>");
  page += (WiFi.status() == WL_CONNECTED) ? ipToString(WiFi.localIP()) : String(F("n/a"));
  page += F("</code></div><span>RSSI</span><div><code id='live-rssi'>");
  if (WiFi.status() == WL_CONNECTED) {
    page += String(WiFi.RSSI());
    page += F(" dBm");
  } else {
    page += F("n/a");
  }
  page += F("</code></div>");
  if (ap_started) {
    page += F("<span>AP</span><div><code>");
    page += ipToString(WiFi.softAPIP());
    page += F("</code></div>");
  }
  page += F("</div></section>");
}

void appendPhyModeOption(String &page, uint8_t mode) {
  page += F("<option value='");
  page += String(mode);
  page += F("'");
  if (config.phy_mode == mode) {
    page += F(" selected");
  }
  page += F(">");
  page += phyModeName(mode);
  page += F("</option>");
}

void appendPhyModeSelect(String &page) {
  page += F("<div class='row'><label>PHY mode<br><select name='phy_mode'>");
  appendPhyModeOption(page, kPhyModeAuto);
  appendPhyModeOption(page, kPhyModeB);
  appendPhyModeOption(page, kPhyModeG);
  appendPhyModeOption(page, kPhyModeN);
  page += F("</select></label></div>");
}

void handleRoot() {
  String page;
  page.reserve(5200);
  beginStreamedResponse("text/html");
  appendHeader(page, F("myMota32"), true);
  page += F("<div class='grid'>");
  flushStreamChunk(page);
  appendStatusBlock(page);
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
  page += F("<button type='submit'>Save Wi-Fi</button></form>");
  page += F("<p><a class='btn secondary' href='/scan'>Scan networks</a></p></section>");
  flushStreamChunk(page);

  page += F("<section class='panel'><h2>Firmware</h2><form method='post' action='/update' enctype='multipart/form-data'>");
  page += F("<input type='file' name='firmware' accept='.bin' required><br><button type='submit'>Upload firmware</button></form>");
  page += F("<p><a class='btn secondary' href='/reboot'>Reboot</a></p>");
  page += F("<form method='post' action='/factory-reset' onsubmit=\"return confirm('Factory reset will delete Wi-Fi settings. Continue?')\"><button class='danger' type='submit'>Factory reset</button></form></section>");
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
  out.reserve(900);
  beginStreamedResponse("application/json");
  out += F("{\"name\":\"myMota32\",\"version\":\"");
  out += F(MYMOTA32_VERSION);
  out += F("\",\"target\":\"");
  out += F(MYMOTA32_TARGET);
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
  if (WiFi.status() == WL_CONNECTED) {
    out += WiFi.RSSI();
  } else {
    out += F("null");
  }
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
  out += F("}}");
  flushStreamChunk(out);
  server.sendContent(F(""));
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
    if (upload.filename.length() == 0) {
      update_error = UPDATE_ERROR_SIZE;
    }
    return;
  }

  if (upload.status == UPLOAD_FILE_WRITE && update_error != UPDATE_ERROR_OK) {
    return;
  }

  if (upload.status == UPLOAD_FILE_WRITE) {
    if (!update_started && upload.totalSize == 0) {
      if (upload.currentSize < 4) {
        update_error = UPDATE_ERROR_SIZE;
        return;
      }
      if (upload.buf[0] != 0xE9) {
        update_error = UPDATE_ERROR_MAGIC_BYTE;
        return;
      }
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        update_error = Update.getError();
        return;
      }
      update_started = true;
    }
    if (Update.hasError()) {
      update_error = Update.getError();
      return;
    }
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      update_error = Update.getError();
    }
    return;
  }

  if (upload.status == UPLOAD_FILE_END) {
    if (update_error != UPDATE_ERROR_OK) {
      if (update_started) {
        Update.abort();
        update_started = false;
      }
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
    if (update_started) {
      Update.abort();
    }
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
  server.on("/reboot", HTTP_GET, handleReboot);
  server.on("/factory-reset", HTTP_POST, handleFactoryReset);
  server.on("/health", HTTP_GET, handleHealth);
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
  Serial.printf("myMota32 %s %s chip %s\n", MYMOTA32_VERSION, MYMOTA32_TARGET, chipIdHex().c_str());
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

  if (restartDue()) {
    delay(50);
    ESP.restart();
  }

  recordLoopPerf(loop_started_us, micros());
  yield();
}
