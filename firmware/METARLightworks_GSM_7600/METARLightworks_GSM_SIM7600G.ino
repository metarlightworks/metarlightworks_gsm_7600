// ============================================================
// METARLightworks_GSM
// Full integrated version
// - Plain AVWX METAR endpoint: /api/metar/<ICAO>
// - Separate nearby lookup fallback
// - Uses Adafruit_NeoPixel
// - Hardened AP startup
// - Styled setup/admin UI
// - WiFi-only OTA
// - JSON METAR parsing
// - GSM retry + reconnect on failed fetch
// - Boot = blinking yellow
// - No signal before first valid METAR = solid yellow
// - Keep last METAR color on later fetch failures
// - WiFi fetch every 20 min, cellular every 60 min
// - Auto mode uses WiFi rules on WiFi and cellular rules on fallback
// - Refreshable WiFi scan list in setup UI
// - Connection mode selector: Auto, Cellular only, WiFi only
// - Forget Saved WiFi button
// - Admin page supports loading a new API key without showing current one
// - Flight Pulse is WiFi-only
// - v1.0.12: SIM7600 ready check uses cached AT registration/IP instead of TinyGSM network/GPRS checks
// - v1.0.13: Boot normal attach first; operator rescue only if needed
// - v1.0.14: Hidden /admintest page for accelerated cellular data testing
// - v1.0.15: Customer graphical cellular signal display + admin LTE detail panel
// - v1.0.16: Persistent NO SERVICE modem recovery + corrected carrier label
// - v1.0.17: Registered-but-data-not-open recovery + PDP diagnostics + chronological event log
// - v1.0.18: Read-only event log on normal Admin page; hidden test menu no longer linked
// ============================================================

// LilyGO T-SIM7600G-H / T-SIM7600X ESP32 target
// Board/Product: https://lilygo.cc/products/t-sim7600
#define LILYGO_SIM7600X
#define TINY_GSM_RX_BUFFER 1024

#include "utilities.h"

#ifndef MODEM_POWERON_PULSE_WIDTH_MS
#define MODEM_POWERON_PULSE_WIDTH_MS 1000
#endif

// SIM7600 does not use the A76XX secure-client wrapper.
// LilyGO's current SIM7600 examples use the modem built-in HTTPS API.
#if defined(TINY_GSM_MODEM_SIM7600)
#define MLW_CELLULAR_BUILTIN_HTTPS 1
#else
#define MLW_CELLULAR_BUILTIN_HTTPS 0
#ifdef TINY_GSM_MODEM_A7670
#undef TINY_GSM_MODEM_A7670
#endif
#ifdef TINY_GSM_MODEM_A7608
#undef TINY_GSM_MODEM_A7608
#endif
#ifndef TINY_GSM_MODEM_A76XXSSL
#define TINY_GSM_MODEM_A76XXSSL
#endif
#endif

#define TINY_GSM_DEBUG Serial

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <time.h>
#include <math.h>
#include "TimeSync.h"

#if MLW_CELLULAR_BUILTIN_HTTPS && !defined(TINY_GSM_FORK_LIBRARY)
#error "SIM7600G build requires the LilyGO TinyGSM fork from Xinyuan-LilyGO/LilyGo-Modem-Series/lib. Mainline TinyGSM does not provide the built-in HTTPS API used here."
#endif


// ====================
// VERSION
// ====================
#define FW_VERSION "1.0.18"

// ====================
// API SETTINGS
// ====================
const char *DEFAULT_AVWX_API_KEY = "StgdUrn7KMjUQo7hBFb99prBEdEfHkh6U_9TDWFul-A";
const char *AVWX_HOST = "avwx.rest";
const int AVWX_PORT = 443;

// ====================
// OTA SETTINGS
// ====================
String otaManifestHost = "raw.githubusercontent.com";
int otaManifestPort = 443;
String otaManifestPath = "/metarlightworks/metar-gsm-firmware/main/ota.json";

// ====================
// DEFAULTS
// ====================
#define DEFAULT_LED_PIN 2
#define DEFAULT_NUM_LEDS 1
#define DEFAULT_BRIGHTNESS 15
#define DEFAULT_LED_ORDER 0 // 0=GRB, 1=RGB, 2=BRG

// ====================
// FETCH INTERVALS
// ====================
const unsigned long WIFI_FETCH_INTERVAL_MS = 20UL * 60UL * 1000UL;
const unsigned long CELL_FETCH_INTERVAL_MS = 60UL * 60UL * 1000UL;

// Cellular hardening
const unsigned long CELL_NETWORK_TIMEOUT_MS = 180UL * 1000UL;
const unsigned long CELL_DATA_TIMEOUT_MS    = 75UL  * 1000UL;
const unsigned long CELL_SETTLE_MS          = 15UL  * 1000UL;
const unsigned long WIFI_RETRY_INTERVAL_MS  = 5UL   * 60UL * 1000UL;

// ====================
// GLOBAL CONFIG
// ====================
uint8_t brightness = DEFAULT_BRIGHTNESS;
String airportCode = "KTIX";
String apn = "us.simplex.iot";
String wifiSsid = "";
String wifiPassword = "";
String connectionMode = "auto"; // auto, cellular, wifi
String avwxApiKey = DEFAULT_AVWX_API_KEY;

String ledOnTime = "06:00";
String ledOffTime = "22:00";
int timezone = -5;
bool ledScheduleEnabled = true;

int ledPin = DEFAULT_LED_PIN;
int numLeds = DEFAULT_NUM_LEDS;
int ledOrder = DEFAULT_LED_ORDER;
bool otaEnabled = false;

bool useWiFi = false;
bool cellularInitialized = false;
bool cellularDataReady = false;
bool lookupInProgress = false;
bool scheduleCurrentlyOff = false;
unsigned long lastWiFiReconnectAttempt = 0;

// Cellular diagnostics shown on the web page
String cellDiagOperator = "";
String cellDiagReg = "";
String cellDiagCereg = "";
String cellDiagCpsi = "";
String cellDiagIP = "";
String cellDiagIpAddrRaw = "";
String cellDiagCgatt = "";
String cellDiagCgact = "";
String cellDiagCgpaddr = "";
String cellDiagNetopen = "";
String cellDiagCeer = "";
String cellLastError = "";
int cellDiagSignal = -1;
unsigned long cellDiagUpdatedMs = 0;

// Cellular recovery. This is intentionally conservative enough to avoid thrash,
// but v1.0.17 also handles the real-world case where the modem is registered
// on LTE yet the data/PDP session will not open.
const uint8_t CELL_NOSERVICE_RECOVERY_THRESHOLD = 5;
const uint8_t CELL_DATA_FAIL_RECOVERY_THRESHOLD = 1; // one full 3-attempt attach failure cycle
const uint8_t CELL_REGISTERED_NO_IP_RECOVERY_THRESHOLD = 3;
const unsigned long CELL_NOSERVICE_RECOVERY_COOLDOWN_MS = 15UL * 60UL * 1000UL;
const unsigned long CELL_PASSIVE_DIAG_INTERVAL_MS = 60UL * 1000UL;
uint8_t cellNoServiceStreak = 0;
uint8_t cellRegisteredNoIpStreak = 0;
uint8_t cellDataOpenFailCycles = 0;
uint16_t cellUnstableServiceEvents = 0;
unsigned long lastCellNoServiceRecoveryMs = 0;
unsigned long lastPassiveCellDiagMs = 0;
uint16_t cellHardRecoveryCount = 0;
bool cellHardRecoveryInProgress = false;
String cellLastHardRecoveryReason = "";
String cellLastDataFailureReason = "";

// Chronological admin event log. RAM-only to avoid flash/NVS wear.
const uint8_t EVENT_LOG_MAX = 80;
String eventLogLines[EVENT_LOG_MAX];
uint8_t eventLogStart = 0;
uint8_t eventLogCount = 0;
uint32_t eventLogSeq = 0;
String cellLastLoggedState = "";

// Passive fetch diagnostics only. These do not change cellular behavior.
String lastFetchTransport = "none";
String lastFetchDetail = "Not yet fetched";
String lastFetchTarget = "";
int lastHttpStatus = 0;
unsigned long lastFetchDurationMs = 0;
unsigned long bootCellConnectDurationMs = 0;
unsigned long lastCellConnectStartMs = 0;

// Hidden /admintest cellular data usage test settings.
// Active test mode is intentionally runtime-only so a reboot cannot accidentally keep burning data.
const char *ADMIN_TEST_PASSWORD = "north";
bool cellDataTestEnabled = false;
String cellDataTestPath = "cellular"; // cellular, wifi, auto
uint16_t cellDataTestIntervalSec = 60;
uint8_t cellDataOverheadMultiplier = 4;
String forcedFetchPath = ""; // one-shot override: cellular, wifi, or empty

// Cellular data estimate counters. These reset on reboot or from /admintest.
uint32_t cellMeterRequestCount = 0;
uint32_t cellMeterSuccessCount = 0;
uint32_t cellMeterFailCount = 0;
uint32_t cellMeterLastBodyBytes = 0;
uint32_t cellMeterLastHeaderBytes = 0;
uint32_t cellMeterLastEstimatedBytes = 0;
int cellMeterLastStatus = 0;
String cellMeterLastTag = "none";
uint64_t cellMeterBodyBytes = 0;
uint64_t cellMeterHeaderBytes = 0;
uint64_t cellMeterEstimatedBytes = 0;
unsigned long cellMeterStartedMs = 0;


// Flight Pulse config
bool fpEnabled = false;
String fpTail = "";
String fpHex = "";

// ====================
// GLOBAL STATE
// ====================
Preferences preferences;
WebServer server(80);

unsigned long previousMillis = 0;
unsigned long currentFetchInterval = CELL_FETCH_INTERVAL_MS;

String lastFlightRules = "UNKNOWN";
String lastRawMetar = "";
String lastStatusLine = "Booting...";
unsigned long lastFetchMillis = 0;
bool hasValidMetar = false;

// Boot / signal state
bool bootBlinkActive = true;
bool noSignalState = false;
unsigned long bootBlinkPrev = 0;
bool bootBlinkOn = false;

// Admin test state
bool adminTestActive = false;
unsigned long adminTestUntil = 0;

// Flight Pulse runtime
bool fpIsFlying = false;
unsigned long fpLastCheckMs = 0;
const unsigned long fpCheckIntervalMs = 180000UL; // 3 min, WiFi only
int fpFlyingStreak = 0;
bool fpPulseActive = false;
unsigned long fpPulseStartMs = 0;
int fpBaseBrightness = 100;
const float FP_PERIOD_MS = 3500.0f;
const float FP_MIN_FRACTION = 0.15f;

// ====================
// LED
// ====================
Adafruit_NeoPixel *strip = nullptr;

// ====================
// MODEM / NET
// ====================
TinyGsm modem(SerialAT);
#if MLW_CELLULAR_BUILTIN_HTTPS
TinyGsmClient gsmClient(modem);   // SIM7600 cellular HTTPS uses modem.https_*; this plain client is kept only for compatibility.
#else
TinyGsmClientSecure gsmClient(modem);
#endif
WiFiClientSecure wifiClient;

// ====================
// FORWARD DECLARATIONS
// ====================
void loadConfig();
void saveMainConfig();
void saveAdminConfig();

void setupStrip();
neoPixelType getNeoTypeForOrder(int order);
uint32_t makeColor(uint8_t r, uint8_t g, uint8_t b);
void setAllPixels(uint32_t color);
void updateLED(String vfrStatus);
void applyCurrentLedColor();

void initCellular();
bool connectCellularData(bool forceAutoSearch, const String& reason);
bool waitForCellularNetwork(unsigned long timeoutMs);
bool waitForCellularDataReady(unsigned long timeoutMs);
void forceAutomaticOperatorSelection();
String modemAT(const String& cmd, unsigned long timeoutMs = 3000);
void updateCellularDiagnostics(const String& reason);
bool cellularNoServiceDetected();
bool cellularRegisteredButNoIpDetected();
bool cellularRecoveryShouldMonitor();
void resetCellularDataFailureCounters();
void resetCellularDataSessionOnly(const String& reason);
bool noteCellularDataFailureCycle(const String& reason);
bool waitForModemATAfterReset(unsigned long timeoutMs);
void configureModemRuntimeAfterReset(const String& reason);
bool hardRestartCellularModem(const String& reason);
void evaluateCellularRecovery(const String& reason);
void serviceCellularRecovery();
void serviceDelay(unsigned long ms);
void maintainWiFiConnection();
bool wifiModeAllowed();
bool cellularModeAllowed();
bool cellularRulesActive();
String connectionModeLabel();
String activeConnectionLabel();
String uptimeText();
void noteFetchStart(const String& transport, const String& detail);
void noteFetchFinish(int statusCode, const String& detail, unsigned long startMs);
unsigned long effectiveFetchIntervalMs();
String formatBytes(uint64_t bytes);
String dataTestPathLabel();
String dataTestStatusLine();
void resetCellDataCounters();
void recordCellularDataSample(const char* tag, int statusCode, uint32_t bodyBytes, uint32_t headerBytes);
bool admintestAuthorized();
String admintestAuthHidden();
void applyAdminTestFetchOverride();
String htmlEscape(const String& in);
String eventTimestampText();
void logEvent(const String& category, const String& message);
void clearEventLog();
String getEventLogHtml(bool includeControls = true);
String buildWiFiScanOptions(int& networkCount);
void fetchMETAR();
void findNearbyAirport();
void parseMETAR(String response);
void updateLEDSchedule();

bool reconnectCellularData();
String urlEncode(const String& value);
String avwxPathWithToken(const String& path);
int doCellularHttpsGetBuiltIn(const String& path, String& responseOut, const char* logTag);
int doCellularMetarRequest(const String& path, String& responseOut);
int doCellularLookupRequest(const String& path, String& responseOut);

void setBootBlink(bool active);
void updateBootBlink();
void setNoSignalState(bool active);
void updateAdminTestState();
void applyAdminTestColor(const String& c);

// Flight Pulse
bool isHex6(String s);
int usN_to_icao_int(String tail);
String usN_to_hex6(String tail);
String resolveTailOrHex(String tailIn, String hexIn);
bool fpFetchIsFlying_ADSBlol(const String& icaoHex6);
void fpStartPulse();
void fpStopPulseRestore();
void fpUpdatePulseOverlay();
void updateFlightPulse();

String htmlHeader(const String &title);
String htmlFooter();
String getStatusBlock();
String sinceLastFetchText();
String maskApiKeyStatus();
String cellDiagAgeText();
String cellLastHardRecoveryAgeText();
String cellCarrierLabel();
String cellNetworkLabel();
bool cachedCellularRegistrationReady();
bool getCellularLteMetrics(float& rsrpDbm, float& rsrqDb, float& rssiDbm, float& sinrDb, String& bandOut);
int cellularSignalBars();
String cellularSignalLabel();
String cellularSignalEmoji();
String cellularSignalSummaryText();
String cellularSignalBarsHtml();
String cellularSignalBoxHtml();
String getCustomerCellSignalHtml();
String getAdminCellularDetailsHtml();

bool otaParseRawUrl(const String& url, String& host, int& port, String& path);
bool otaGetLatest(String& outVersion, String& outUrl, int& outSize);
bool otaInstallNow();

void handleRoot();
void handleSave();
void handleAdmin();
void handleSaveAdmin();
void handleAdminTest();
void handleSaveAdminTest();
void handleAdminTestResetCounters();
void handleAdminTestClearLog();
void handleAdminTestFetchNow();
void handleRefresh();
void handleForgetWifi();
void handleTest();
void handleOtaCheck();
void handleOtaInstall();
void handleFlightPulseSave();

// ============================================================
// HTML HELPERS
// ============================================================

bool wifiModeAllowed() {
  return connectionMode != "cellular";
}

bool cellularModeAllowed() {
  return connectionMode != "wifi";
}

bool cellularRulesActive() {
  return cellularModeAllowed() && !(useWiFi && WiFi.status() == WL_CONNECTED);
}

String connectionModeLabel() {
  if (connectionMode == "cellular") return "Cellular only";
  if (connectionMode == "wifi") return "WiFi only";
  return "Auto: WiFi preferred, cellular fallback";
}

String activeConnectionLabel() {
  if (useWiFi && WiFi.status() == WL_CONNECTED) return "WiFi";
  if (connectionMode == "cellular") return "Cellular only";
  if (connectionMode == "auto") return "Cellular fallback";
  return "WiFi not connected";
}

String uptimeText() {
  unsigned long sec = millis() / 1000UL;
  unsigned long days = sec / 86400UL;
  sec %= 86400UL;
  unsigned long hrs = sec / 3600UL;
  sec %= 3600UL;
  unsigned long mins = sec / 60UL;

  if (days > 0) return String(days) + "d " + String(hrs) + "h " + String(mins) + "m";
  if (hrs > 0) return String(hrs) + "h " + String(mins) + "m";
  return String(mins) + "m";
}

void noteFetchStart(const String& transport, const String& detail) {
  lastFetchTransport = transport;
  lastFetchDetail = detail;
  lastFetchTarget = airportCode;
  lastHttpStatus = 0;
  lastFetchDurationMs = 0;
  Serial.println("[FETCH] Start " + transport + " | " + detail + " | airport " + airportCode);
  logEvent("FETCH", "Start " + transport + " | " + detail + " | airport " + airportCode);
}

void noteFetchFinish(int statusCode, const String& detail, unsigned long startMs) {
  lastHttpStatus = statusCode;
  lastFetchDurationMs = millis() - startMs;
  lastFetchDetail = detail;
  Serial.println("[FETCH] Finish HTTP/status " + String(statusCode) + " | " + detail + " | " + String(lastFetchDurationMs) + " ms");
  logEvent("FETCH", "Finish status " + String(statusCode) + " | " + detail + " | " + String(lastFetchDurationMs) + " ms");
}

unsigned long effectiveFetchIntervalMs() {
  if (!cellDataTestEnabled) return currentFetchInterval;
  uint16_t sec = cellDataTestIntervalSec;
  if (sec < 30) sec = 30;
  if (sec > 3600) sec = 3600;
  return (unsigned long)sec * 1000UL;
}

String formatBytes(uint64_t bytes) {
  char buf[32];
  double v = (double)bytes;
  if (bytes < 1024ULL) {
    snprintf(buf, sizeof(buf), "%lu B", (unsigned long)bytes);
  } else if (bytes < 1024ULL * 1024ULL) {
    snprintf(buf, sizeof(buf), "%.1f KB", v / 1024.0);
  } else {
    snprintf(buf, sizeof(buf), "%.2f MB", v / 1048576.0);
  }
  return String(buf);
}

String dataTestPathLabel() {
  if (cellDataTestPath == "wifi") return "WiFi only";
  if (cellDataTestPath == "auto") return "Normal auto/device logic";
  return "Cellular only";
}

String dataTestStatusLine() {
  String s = String(cellDataTestEnabled ? "ON" : "OFF");
  s += " | path " + dataTestPathLabel();
  s += " | interval " + String(cellDataTestIntervalSec) + " sec";
  return s;
}

void resetCellDataCounters() {
  cellMeterRequestCount = 0;
  cellMeterSuccessCount = 0;
  cellMeterFailCount = 0;
  cellMeterLastBodyBytes = 0;
  cellMeterLastHeaderBytes = 0;
  cellMeterLastEstimatedBytes = 0;
  cellMeterLastStatus = 0;
  cellMeterLastTag = "none";
  cellMeterBodyBytes = 0;
  cellMeterHeaderBytes = 0;
  cellMeterEstimatedBytes = 0;
  cellMeterStartedMs = millis();
}

void recordCellularDataSample(const char* tag, int statusCode, uint32_t bodyBytes, uint32_t headerBytes) {
  cellMeterRequestCount++;
  if (statusCode == 200) cellMeterSuccessCount++;
  else cellMeterFailCount++;
  uint32_t baseBytes = bodyBytes + headerBytes;
  if (baseBytes == 0) baseBytes = 1;
  uint32_t estimated = baseBytes * (uint32_t)cellDataOverheadMultiplier;
  cellMeterLastTag = String(tag ? tag : "cellular");
  cellMeterLastStatus = statusCode;
  cellMeterLastBodyBytes = bodyBytes;
  cellMeterLastHeaderBytes = headerBytes;
  cellMeterLastEstimatedBytes = estimated;
  cellMeterBodyBytes += bodyBytes;
  cellMeterHeaderBytes += headerBytes;
  cellMeterEstimatedBytes += estimated;
  Serial.println("[DATA] " + cellMeterLastTag + " status " + String(statusCode) + " body " + String(bodyBytes) + " header " + String(headerBytes) + " est " + String(estimated) + " bytes");
}

bool admintestAuthorized() { return server.hasArg("p") && server.arg("p") == ADMIN_TEST_PASSWORD; }
String admintestAuthHidden() { return "<input type='hidden' name='p' value='" + htmlEscape(String(ADMIN_TEST_PASSWORD)) + "'>"; }
void applyAdminTestFetchOverride() {
  if (!cellDataTestEnabled) return;
  if (cellDataTestPath == "cellular") forcedFetchPath = "cellular";
  else if (cellDataTestPath == "wifi") forcedFetchPath = "wifi";
  else forcedFetchPath = "";
}

String htmlEscape(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else if (c == '\'') out += "&#39;";
    else out += c;
  }
  return out;
}


String eventTimestampText() {
  unsigned long sec = millis() / 1000UL;
  unsigned long days = sec / 86400UL;
  unsigned long rem = sec % 86400UL;
  unsigned long hrs = rem / 3600UL;
  rem %= 3600UL;
  unsigned long mins = rem / 60UL;
  unsigned long secs = rem % 60UL;

  char uptimeBuf[32];
  if (days > 0) snprintf(uptimeBuf, sizeof(uptimeBuf), "%lud %02lu:%02lu:%02lu", days, hrs, mins, secs);
  else snprintf(uptimeBuf, sizeof(uptimeBuf), "%02lu:%02lu:%02lu", hrs, mins, secs);

  time_t now = time(nullptr);
  if (now > 1700000000) {
    struct tm ti;
    localtime_r(&now, &ti);
    char timeBuf[32];
    strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", &ti);
    return String(timeBuf) + " | up " + String(uptimeBuf);
  }
  return String("up ") + String(uptimeBuf);
}

void logEvent(const String& category, const String& message) {
  String cat = category;
  cat.trim();
  if (!cat.length()) cat = "INFO";

  String msg = message;
  msg.replace("\r", " ");
  msg.replace("\n", " ");
  msg.trim();
  if (!msg.length()) msg = "event";

  eventLogSeq++;
  String line = String("#") + String(eventLogSeq) + " | " + eventTimestampText() + " | " + cat + " | " + msg;

  uint8_t idx;
  if (eventLogCount < EVENT_LOG_MAX) {
    idx = (eventLogStart + eventLogCount) % EVENT_LOG_MAX;
    eventLogCount++;
  } else {
    idx = eventLogStart;
    eventLogStart = (eventLogStart + 1) % EVENT_LOG_MAX;
  }
  eventLogLines[idx] = line;
  Serial.println("[EVENT] " + line);
}

void clearEventLog() {
  for (uint8_t i = 0; i < EVENT_LOG_MAX; i++) eventLogLines[i] = "";
  eventLogStart = 0;
  eventLogCount = 0;
  eventLogSeq = 0;
}

String getEventLogHtml(bool includeControls) {
  String html;
  html += "<div class='card'><h2>Chronological Event Log</h2>";
  html += "<div class='muted'>RAM-only log of the last " + String(EVENT_LOG_MAX) + " modem, carrier, data, recovery, WiFi, and METAR events. Newest is at the bottom.</div>";
  html += "<div class='eventlog'>";
  if (eventLogCount == 0) {
    html += "<div class='eventrow muted'>No events logged yet.</div>";
  } else {
    for (uint8_t i = 0; i < eventLogCount; i++) {
      uint8_t idx = (eventLogStart + i) % EVENT_LOG_MAX;
      html += "<div class='eventrow'>" + htmlEscape(eventLogLines[idx]) + "</div>";
    }
  }
  html += "</div>";
  if (includeControls) {
    html += "<form method='POST' action='/admintestClearLog' style='margin-top:10px;'>" + admintestAuthHidden();
    html += "<button class='btn btn4' type='submit'>Clear Event Log</button></form>";
  } else {
    html += "<div class='muted' style='margin-top:10px;'>Read-only troubleshooting log. Data-test controls and fetch-rate controls are not shown on the normal Admin page.</div>";
  }
  html += "</div>";
  return html;
}

String buildWiFiScanOptions(int& networkCount) {
  networkCount = WiFi.scanNetworks(false, true);
  String options;

  if (networkCount <= 0) {
    options += "<option value=''>No networks found</option>";
    return options;
  }

  options += "<option value=''>Select a network...</option>";
  for (int i = 0; i < networkCount; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) continue;

    bool duplicate = false;
    for (int j = 0; j < i; j++) {
      if (WiFi.SSID(j) == ssid) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) continue;

    String enc = htmlEscape(ssid);
    String auth = WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "open" : "locked";
    int rssi = WiFi.RSSI(i);
    options += "<option value='" + enc + "'";
    if (ssid == wifiSsid) options += " selected";
    options += ">" + enc + " (" + String(rssi) + " dBm, " + auth + ")</option>";
  }
  return options;
}


String htmlHeader(const String &title) {
  String html;
  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>" + title + "</title>";
  html += "<style>";
  html += "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,Arial,sans-serif;background:#0f172a;color:#e2e8f0;margin:0;padding:0;}";
  html += ".wrap{max-width:920px;margin:0 auto;padding:18px;}";
  html += ".card{background:#111827;border:1px solid #334155;border-radius:16px;padding:18px;margin-bottom:16px;box-shadow:0 4px 18px rgba(0,0,0,.25);}";
  html += "h1,h2,h3{margin:0 0 12px 0;}";
  html += "h1{font-size:28px;} h2{font-size:20px;} h3{font-size:16px;color:#cbd5e1;}";
  html += "label{display:block;margin:10px 0 6px;color:#cbd5e1;font-weight:600;}";
  html += "input,select{width:100%;padding:12px;border-radius:10px;border:1px solid #475569;background:#0b1220;color:#e2e8f0;box-sizing:border-box;}";
  html += ".row{display:grid;grid-template-columns:1fr 1fr;gap:14px;}";
  html += ".row3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px;}";
  html += ".btn{display:inline-block;background:#2563eb;color:white;padding:12px 16px;border:none;border-radius:10px;text-decoration:none;font-weight:700;cursor:pointer;margin:4px 6px 4px 0;}";
  html += ".btn2{background:#475569;}";
  html += ".btn3{background:#059669;}";
  html += ".btn4{background:#dc2626;}";
  html += ".pill{display:inline-block;padding:5px 10px;border-radius:999px;background:#1e293b;border:1px solid #334155;margin:0 6px 6px 0;font-size:13px;}";
  html += ".muted{color:#94a3b8;font-size:14px;}";
  html += ".good{color:#86efac;}.warn{color:#fbbf24;}.bad{color:#fca5a5;}";
  html += ".mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;word-break:break-word;}";
  html += ".divider{height:1px;background:#334155;margin:14px 0;}";
  html += ".signalbox{display:flex;align-items:center;justify-content:space-between;gap:14px;background:#0b1220;border:1px solid #334155;border-radius:14px;padding:14px;margin-top:12px;}";
  html += ".signalmain{display:flex;align-items:center;gap:12px;}";
  html += ".signalemoji{font-size:34px;line-height:1;}";
  html += ".signaltitle{font-size:18px;font-weight:800;}";
  html += ".bars{display:flex;align-items:flex-end;gap:4px;height:34px;min-width:72px;}";
  html += ".bar{width:10px;border-radius:4px 4px 2px 2px;background:#1e293b;border:1px solid #475569;}";
  html += ".bar.on{background:#86efac;border-color:#86efac;}";
  html += ".bar.b1{height:9px}.bar.b2{height:15px}.bar.b3{height:21px}.bar.b4{height:27px}.bar.b5{height:33px}";
  html += ".eventlog{max-height:360px;overflow:auto;background:#0b1220;border:1px solid #334155;border-radius:12px;padding:10px;}";
  html += ".eventrow{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:13px;border-bottom:1px solid rgba(51,65,85,.55);padding:5px 0;white-space:pre-wrap;word-break:break-word;}";
  html += ".eventrow:last-child{border-bottom:none;}";
  html += "@media(max-width:720px){.row,.row3{grid-template-columns:1fr;}}";
  html += "</style></head><body><div class='wrap'>";
  return html;
}

String htmlFooter() {
  return "</div></body></html>";
}

String sinceLastFetchText() {
  if (lastFetchMillis == 0) return "Never";
  unsigned long sec = (millis() - lastFetchMillis) / 1000UL;
  if (sec < 60) return String(sec) + " sec ago";
  if (sec < 3600) return String(sec / 60UL) + " min ago";
  return String(sec / 3600UL) + " hr ago";
}

String maskApiKeyStatus() {
  if (avwxApiKey == DEFAULT_AVWX_API_KEY) return "Default key in use";
  if (avwxApiKey.length() < 8) return "Custom key saved";
  return "Custom key saved";
}

String cellDiagAgeText() {
  if (cellDiagUpdatedMs == 0) return "not yet updated";
  unsigned long sec = (millis() - cellDiagUpdatedMs) / 1000UL;
  if (sec < 60) return String(sec) + " sec ago";
  if (sec < 3600) return String(sec / 60UL) + " min ago";
  return String(sec / 3600UL) + " hr ago";
}

String cellLastHardRecoveryAgeText() {
  if (lastCellNoServiceRecoveryMs == 0) return "never";
  unsigned long sec = (millis() - lastCellNoServiceRecoveryMs) / 1000UL;
  if (sec < 60) return String(sec) + " sec ago";
  if (sec < 3600) return String(sec / 60UL) + " min ago";
  return String(sec / 3600UL) + " hr ago";
}

String cellCarrierLabel() {
  String r = cellDiagOperator;
  r.trim();

  int q1 = r.indexOf('"');
  int q2 = r.indexOf('"', q1 + 1);
  if (q1 >= 0 && q2 > q1) {
    String carrier = r.substring(q1 + 1, q2);
    carrier.trim();
    if (carrier.length()) return carrier;
  }

  if (cachedCellularRegistrationReady()) return "Registered, carrier not reported";
  if (r.indexOf("+COPS:") >= 0 || r.indexOf("COPS") >= 0) return "Auto carrier selection";
  return "No carrier reported";
}

String cpsiToken(int index) {
  String data = cellDiagCpsi;
  data.replace("\r", " ");
  data.replace("\n", " ");
  data.replace(" OK", "");
  data.replace("OK", "");
  int colon = data.indexOf(':');
  if (colon >= 0) data = data.substring(colon + 1);
  data.trim();

  int start = 0;
  for (int i = 0; i <= index; i++) {
    int comma = data.indexOf(',', start);
    String tok;
    if (comma < 0) tok = data.substring(start);
    else tok = data.substring(start, comma);
    tok.trim();
    if (i == index) return tok;
    if (comma < 0) break;
    start = comma + 1;
  }
  return "";
}

int cpsiBandTokenIndex() {
  for (int i = 0; i < 24; i++) {
    String tok = cpsiToken(i);
    if (tok.startsWith("EUTRAN-BAND")) return i;
  }
  return -1;
}

bool getCellularLteMetrics(float& rsrpDbm, float& rsrqDb, float& rssiDbm, float& sinrDb, String& bandOut) {
  rsrpDbm = 0;
  rsrqDb = 0;
  rssiDbm = 0;
  sinrDb = 0;
  bandOut = "";

  int bi = cpsiBandTokenIndex();
  if (bi < 0) return false;

  bandOut = cpsiToken(bi);
  String rsrqRaw = cpsiToken(bi + 4);
  String rsrpRaw = cpsiToken(bi + 5);
  String rssiRaw = cpsiToken(bi + 6);
  String sinrRaw = cpsiToken(bi + 7);
  if (!rsrqRaw.length() || !rsrpRaw.length() || !rssiRaw.length() || !sinrRaw.length()) return false;

  rsrqDb = rsrqRaw.toFloat() / 10.0f;
  rsrpDbm = rsrpRaw.toFloat() / 10.0f;
  rssiDbm = rssiRaw.toFloat() / 10.0f;
  sinrDb = sinrRaw.toFloat() / 10.0f;
  return true;
}

String cellNetworkLabel() {
  float rsrp, rsrq, rssi, sinr;
  String band;
  if (getCellularLteMetrics(rsrp, rsrq, rssi, sinr, band) && band.length()) {
    band.replace("EUTRAN-", "");
    return "LTE " + band;
  }
  if (cellDiagCpsi.indexOf("LTE") >= 0) return "LTE";
  return "Unknown network";
}

int barsFromRsrp(float rsrpDbm) {
  if (rsrpDbm >= -90.0f) return 5;
  if (rsrpDbm >= -100.0f) return 4;
  if (rsrpDbm >= -105.0f) return 3;
  if (rsrpDbm >= -110.0f) return 2;
  if (rsrpDbm >= -118.0f) return 1;
  return 0;
}

int barsFromSinr(float sinrDb) {
  if (sinrDb >= 20.0f) return 5;
  if (sinrDb >= 13.0f) return 4;
  if (sinrDb >= 5.0f) return 3;
  if (sinrDb >= 0.0f) return 2;
  if (sinrDb >= -3.0f) return 1;
  return 0;
}

int barsFromCsq(int csq) {
  if (csq < 0 || csq == 99) return 0;
  if (csq >= 25) return 5;
  if (csq >= 20) return 4;
  if (csq >= 15) return 3;
  if (csq >= 10) return 2;
  if (csq >= 2) return 1;
  return 0;
}

int cellularSignalBars() {
  float rsrp, rsrq, rssi, sinr;
  String band;
  if (getCellularLteMetrics(rsrp, rsrq, rssi, sinr, band)) {
    int a = barsFromRsrp(rsrp);
    int b = barsFromSinr(sinr);
    return (a < b) ? a : b;
  }
  return barsFromCsq(cellDiagSignal);
}

String cellularSignalLabel() {
  int b = cellularSignalBars();
  if (b >= 5) return "Excellent";
  if (b == 4) return "Good";
  if (b == 3) return "Fair";
  if (b == 2) return "Weak";
  if (b == 1) return "Very weak";
  return "No usable signal";
}

String cellularSignalEmoji() {
  int b = cellularSignalBars();
  if (cellularDataReady) return "📶";
  if (cachedCellularRegistrationReady()) return "📡";
  if (b > 0) return "📡";
  return "⚠️";
}

String cellularSignalSummaryText() {
  String s;
  if (cellularDataReady) s += "Connected";
  else if (cachedCellularRegistrationReady()) s += "Registered, data not ready";
  else if (cellularInitialized) s += "Searching";
  else s += "Not started";

  int b = cellularSignalBars();
  s += " — " + String(b) + "/5 bars, " + cellularSignalLabel();
  return s;
}

String cellularSignalBarsHtml() {
  int bars = cellularSignalBars();
  String h = "<div class='bars' aria-label='" + String(bars) + " out of 5 cellular bars'>";
  for (int i = 1; i <= 5; i++) {
    h += "<span class='bar b" + String(i) + String(i <= bars ? " on" : "") + "'></span>";
  }
  h += "</div>";
  return h;
}

String cellularSignalBoxHtml() {
  String html;
  html += "<div class='signalbox'>";
  html += "<div class='signalmain'><div class='signalemoji'>" + cellularSignalEmoji() + "</div><div>";
  html += "<div class='signaltitle'>" + cellularSignalSummaryText() + "</div>";
  html += "<div class='muted'>" + htmlEscape(cellCarrierLabel()) + " · " + htmlEscape(cellNetworkLabel()) + "</div>";
  html += "</div></div>";
  html += cellularSignalBarsHtml();
  html += "</div>";
  return html;
}

String getCustomerCellSignalHtml() {
  String html;
  html += "<div class='card'><h2>Cellular Signal</h2>";
  html += cellularSignalBoxHtml();

  if (!cellularDataReady && cachedCellularRegistrationReady()) {
    html += "<div class='warn' style='margin-top:10px;'>Cell service is present, but cellular data has not opened yet. This can take longer with weak signal or roaming SIMs.</div>";
  } else if (cellularSignalBars() <= 2 && cellularInitialized) {
    html += "<div class='warn' style='margin-top:10px;'>Weak cellular signal may slow startup after a power outage.</div>";
  }

  if (cellNoServiceStreak > 0 && cellularNoServiceDetected()) {
    html += "<div class='warn' style='margin-top:10px;'>Cellular service dropped. The modem recovery counter is " + String(cellNoServiceStreak) + "/" + String(CELL_NOSERVICE_RECOVERY_THRESHOLD) + ". The lamp will try a modem restart if this persists.</div>";
  }

  html += "<div class='muted'>Signal checked " + cellDiagAgeText() + ". Use Refresh Now or reload this page after the modem connects.</div>";
  html += "</div>";
  return html;
}

String getAdminCellularDetailsHtml() {
  float rsrp, rsrq, rssi, sinr;
  String band;
  bool haveLte = getCellularLteMetrics(rsrp, rsrq, rssi, sinr, band);

  String html;
  html += "<div class='card'><h2>Cellular Diagnostics</h2>";
  html += cellularSignalBoxHtml();
  if (!cellularDataReady && cachedCellularRegistrationReady()) {
    html += "<div class='warn' style='margin-top:10px;'>Cell service is present, but cellular data has not opened yet.</div>";
  }
  html += "<div class='divider'></div>";
  html += "<div class='row3'>";
  html += "<div><span class='pill'>Data " + String(cellularDataReady ? "ready" : "not ready") + "</span></div>";
  html += "<div><span class='pill'>IP " + htmlEscape(cellDiagIP.length() ? cellDiagIP : String("unknown")) + "</span></div>";
  html += "<div><span class='pill'>RSSI/CSQ " + String(cellDiagSignal) + "</span></div>";
  html += "</div>";
  if (bootCellConnectDurationMs > 0) html += "<div class='muted'>Last cellular connect duration: " + String(bootCellConnectDurationMs) + " ms</div>";
  html += "<div class='muted'>No-service streak: " + String(cellNoServiceStreak) + "/" + String(CELL_NOSERVICE_RECOVERY_THRESHOLD) + " | registered-no-IP: " + String(cellRegisteredNoIpStreak) + "/" + String(CELL_REGISTERED_NO_IP_RECOVERY_THRESHOLD) + " | data-fail cycles: " + String(cellDataOpenFailCycles) + " | hard recoveries: " + String(cellHardRecoveryCount) + " | last hard recovery: " + cellLastHardRecoveryAgeText() + "</div>";
  if (cellLastDataFailureReason.length()) html += "<div class='warn'>Last data failure: " + htmlEscape(cellLastDataFailureReason) + "</div>";
  if (cellLastHardRecoveryReason.length()) html += "<div class='muted'>Last hard recovery reason: " + htmlEscape(cellLastHardRecoveryReason) + "</div>";
  if (cellLastError.length()) html += "<div class='warn'>Cell last error: " + htmlEscape(cellLastError) + "</div>";

  if (haveLte) {
    html += "<div class='divider'></div>";
    html += "<div class='row3'>";
    html += "<div><label>RSRP</label><div class='mono'>" + String(rsrp, 1) + " dBm</div></div>";
    html += "<div><label>RSRQ</label><div class='mono'>" + String(rsrq, 1) + " dB</div></div>";
    html += "<div><label>SINR</label><div class='mono'>" + String(sinr, 1) + " dB</div></div>";
    html += "</div>";
    html += "<div class='row3'>";
    html += "<div><label>LTE RSSI</label><div class='mono'>" + String(rssi, 1) + " dBm</div></div>";
    html += "<div><label>Band</label><div class='mono'>" + htmlEscape(band) + "</div></div>";
    html += "<div><label>Parsed Quality</label><div class='mono'>" + String(cellularSignalBars()) + "/5 " + htmlEscape(cellularSignalLabel()) + "</div></div>";
    html += "</div>";
    html += "<div class='muted'>Bars use the weaker of RSRP and SINR, because low SINR can break data even when RSSI looks acceptable.</div>";
  }

  html += "<div class='divider'></div>";
  if (cellDiagIpAddrRaw.length()) html += "<div class='muted mono'>IPADDR: " + htmlEscape(cellDiagIpAddrRaw) + "</div>";
  if (cellDiagNetopen.length()) html += "<div class='muted mono'>NETOPEN: " + htmlEscape(cellDiagNetopen) + "</div>";
  if (cellDiagCgatt.length()) html += "<div class='muted mono'>CGATT: " + htmlEscape(cellDiagCgatt) + "</div>";
  if (cellDiagCgact.length()) html += "<div class='muted mono'>CGACT: " + htmlEscape(cellDiagCgact) + "</div>";
  if (cellDiagCgpaddr.length()) html += "<div class='muted mono'>CGPADDR: " + htmlEscape(cellDiagCgpaddr) + "</div>";
  if (cellDiagCeer.length()) html += "<div class='muted mono'>CEER: " + htmlEscape(cellDiagCeer) + "</div>";
  if (cellDiagOperator.length()) html += "<div class='muted mono'>COPS: " + htmlEscape(cellDiagOperator) + "</div>";
  if (cellDiagReg.length()) html += "<div class='muted mono'>CREG: " + htmlEscape(cellDiagReg) + "</div>";
  if (cellDiagCereg.length()) html += "<div class='muted mono'>CEREG: " + htmlEscape(cellDiagCereg) + "</div>";
  if (cellDiagCpsi.length()) html += "<div class='muted mono'>CPSI: " + htmlEscape(cellDiagCpsi) + "</div>";
  html += "<div class='muted'>Diagnostics checked " + cellDiagAgeText() + ".</div>";
  html += "</div>";
  return html;
}

String getStatusBlock() {
  String s;
  s += "<div>";
  s += "<span class='pill'>FW " + String(FW_VERSION) + "</span>";
  s += "<span class='pill'>Airport " + airportCode + "</span>";
  s += "<span class='pill'>Rules " + lastFlightRules + "</span>";
  s += "<span class='pill'>Last fetch " + sinceLastFetchText() + "</span>";
  if (cellDataTestEnabled) s += "<span class='pill warn'>TEST every " + String(cellDataTestIntervalSec) + " sec</span>";
  else s += "<span class='pill'>Fetch every " + String((currentFetchInterval / 60000UL)) + " min</span>";
  s += "<span class='pill'>Mode " + connectionModeLabel() + "</span>";
  s += "<span class='pill'>Active " + activeConnectionLabel() + "</span>";
  s += "<span class='pill'>Uptime " + uptimeText() + "</span>";
  s += "</div>";

  if (cellDataTestEnabled) {
    s += "<div class='warn'>Admin Cellular Data Test Mode ACTIVE — " + dataTestStatusLine() + ". Web UI can stay on WiFi while test fetches use the selected path.</div>";
  }
  s += "<div class='muted'>Heap free " + String(ESP.getFreeHeap()) + " | min " + String(ESP.getMinFreeHeap()) + "</div>";
  s += "<div class='muted'>Last request: " + htmlEscape(lastFetchTransport) + " | target " + htmlEscape(lastFetchTarget) + " | status " + String(lastHttpStatus) + " | " + String(lastFetchDurationMs) + " ms | " + htmlEscape(lastFetchDetail) + "</div>";

  if (WiFi.status() == WL_CONNECTED) {
    s += "<div class='good'>WiFi connected: " + WiFi.localIP().toString() + "</div>";
  } else {
    s += "<div class='warn'>WiFi not connected</div>";
  }

  IPAddress apIP = WiFi.softAPIP();
  s += "<div class='muted'>AP SSID: METARLightworks_GSM | AP IP: " + apIP.toString() + "</div>";

  if (cellularRulesActive()) {
    if (modem.isNetworkConnected()) {
      s += "<div class='good'>Cellular network connected</div>";
    } else {
      s += "<div class='warn'>Cellular network not connected</div>";
    }
  }

  if (fpEnabled) {
    String fpStatus = "WiFi only";
    if (cellDataTestEnabled && cellDataTestPath == "cellular") {
      fpStatus = "Paused during cellular data test";
    } else if (cellularRulesActive()) {
      fpStatus = "Paused on cellular";
    } else if (useWiFi && WiFi.status() == WL_CONNECTED) {
      fpStatus = fpIsFlying ? "Flying" : "Not flying";
      if (fpHex.length() != 6) fpStatus = "Enabled, no aircraft set";
    }
    s += "<div class='muted'>Flight Pulse: " + fpStatus + "</div>";
  } else {
    s += "<div class='muted'>Flight Pulse: Off</div>";
  }

  if (cellDataTestEnabled && cellDataTestPath == "cellular") {
    s += "<div class='muted'>Admin test cellular fetch path active: WiFi web UI stays connected, METAR test fetches use cellular, Flight Pulse paused.</div>";
  } else if (cellularRulesActive()) {
    s += "<div class='muted'>Cellular rules active: METAR hourly, Flight Pulse paused, OTA WiFi required.</div>";
  } else if (useWiFi && WiFi.status() == WL_CONNECTED) {
    s += "<div class='muted'>WiFi rules active: normal METAR interval, Flight Pulse allowed, OTA allowed if enabled.</div>";
  }

  s += "<div class='muted'>LED pin " + String(ledPin) + " | count " + String(numLeds) + " | brightness " + String(brightness) + "</div>";
  s += "<div class='muted'>API key: " + maskApiKeyStatus() + "</div>";
  s += "<div class='muted'>Backend status: " + lastStatusLine + "</div>";
  if (cellularRulesActive() || cellularInitialized || (cellDataTestEnabled && cellDataTestPath == "cellular")) {
    s += "<div class='muted'>Cellular: " + cellularSignalSummaryText() + " | IP " + (cellDiagIP.length() ? cellDiagIP : String("unknown")) + "</div>";
    if (bootCellConnectDurationMs > 0) s += "<div class='muted'>Last cellular connect duration: " + String(bootCellConnectDurationMs) + " ms</div>";
    if (cellNoServiceStreak > 0 || cellRegisteredNoIpStreak > 0 || cellDataOpenFailCycles > 0 || cellHardRecoveryCount > 0) s += "<div class='muted'>No-service streak " + String(cellNoServiceStreak) + "/" + String(CELL_NOSERVICE_RECOVERY_THRESHOLD) + " | registered-no-IP " + String(cellRegisteredNoIpStreak) + "/" + String(CELL_REGISTERED_NO_IP_RECOVERY_THRESHOLD) + " | data-fail cycles " + String(cellDataOpenFailCycles) + " | hard recoveries " + String(cellHardRecoveryCount) + "</div>";
    if (cellLastError.length()) s += "<div class='warn'>Cell last error: " + htmlEscape(cellLastError) + "</div>";
  }
  if (lastRawMetar.length()) {
    s += "<div class='muted mono'>Last METAR: " + lastRawMetar + "</div>";
  }
  return s;
}

// ============================================================
// CONFIG STORAGE
// ============================================================

void loadConfig() {
  preferences.begin("metar", true);

  brightness         = preferences.getUInt("brightness", DEFAULT_BRIGHTNESS);
  airportCode        = preferences.getString("airport", "KTIX");
  wifiSsid           = preferences.getString("wifiSsid", "");
  wifiPassword       = preferences.getString("wifiPassword", "");
  connectionMode     = preferences.getString("connMode", "auto");
  apn                = preferences.getString("apn", "us.simplex.iot");
  avwxApiKey         = preferences.getString("apiKey", DEFAULT_AVWX_API_KEY);
  ledOnTime          = preferences.getString("ledOnTime", "06:00");
  ledOffTime         = preferences.getString("ledOffTime", "22:00");
  timezone           = preferences.getInt("timezone", -5);
  ledScheduleEnabled = preferences.getBool("schedEn", true);

  ledPin             = preferences.getInt("ledPin", DEFAULT_LED_PIN);
  numLeds            = preferences.getInt("numLeds", DEFAULT_NUM_LEDS);
  ledOrder           = preferences.getInt("ledOrder", DEFAULT_LED_ORDER);

  otaEnabled         = preferences.getBool("otaEnabled", false);

  cellDataTestPath           = preferences.getString("testPath", "cellular");
  cellDataTestIntervalSec    = preferences.getUInt("testSec", 60);
  cellDataOverheadMultiplier = preferences.getUInt("testMult", 4);
  cellDataTestEnabled        = false; // safety: never resume accelerated data mode after reboot

  fpEnabled          = preferences.getBool("fpEnabled", false);
  fpTail             = preferences.getString("fpTail", "");
  fpHex              = preferences.getString("fpHex", "");

  preferences.end();

  connectionMode.trim(); connectionMode.toLowerCase();
  if (connectionMode != "auto" && connectionMode != "cellular" && connectionMode != "wifi") connectionMode = "auto";

  if (avwxApiKey.length() == 0) avwxApiKey = DEFAULT_AVWX_API_KEY;
  if (brightness > 255) brightness = 255;
  if (numLeds < 1) numLeds = 1;
  if (numLeds > 300) numLeds = 300;
  if (ledOrder < 0 || ledOrder > 2) ledOrder = 0;
  cellDataTestPath.trim(); cellDataTestPath.toLowerCase();
  if (cellDataTestPath != "cellular" && cellDataTestPath != "wifi" && cellDataTestPath != "auto") cellDataTestPath = "cellular";
  if (cellDataTestIntervalSec < 30) cellDataTestIntervalSec = 30;
  if (cellDataTestIntervalSec > 3600) cellDataTestIntervalSec = 3600;
  if (cellDataOverheadMultiplier < 1) cellDataOverheadMultiplier = 1;
  if (cellDataOverheadMultiplier > 10) cellDataOverheadMultiplier = 10;

  airportCode.trim(); airportCode.toUpperCase();
  fpTail.trim(); fpTail.toUpperCase();
  fpHex.trim(); fpHex.toUpperCase();
}

void saveMainConfig() {
  preferences.begin("metar", false);
  preferences.putUInt("brightness", brightness);
  preferences.putString("airport", airportCode);
  preferences.putString("wifiSsid", wifiSsid);
  preferences.putString("wifiPassword", wifiPassword);
  preferences.putString("connMode", connectionMode);
  preferences.putString("apn", apn);
  preferences.putString("ledOnTime", ledOnTime);
  preferences.putString("ledOffTime", ledOffTime);
  preferences.putInt("timezone", timezone);
  preferences.putBool("schedEn", ledScheduleEnabled);
  preferences.putBool("fpEnabled", fpEnabled);
  preferences.putString("fpTail", fpTail);
  preferences.putString("fpHex", fpHex);
  preferences.end();
}

void saveAdminConfig() {
  preferences.begin("metar", false);
  preferences.putInt("ledPin", ledPin);
  preferences.putInt("numLeds", numLeds);
  preferences.putInt("ledOrder", ledOrder);
  preferences.putBool("otaEnabled", otaEnabled);
  preferences.putString("apiKey", avwxApiKey);
  preferences.end();
}

// ============================================================
// LED HELPERS
// ============================================================

neoPixelType getNeoTypeForOrder(int order) {
  switch (order) {
    case 1: return NEO_RGB + NEO_KHZ800;
    case 2: return NEO_BRG + NEO_KHZ800;
    case 0:
    default: return NEO_GRB + NEO_KHZ800;
  }
}

void setupStrip() {
  if (strip != nullptr) {
    delete strip;
    strip = nullptr;
  }

  if (numLeds < 1) numLeds = 1;
  if (numLeds > 300) numLeds = 300;

  strip = new Adafruit_NeoPixel(numLeds, ledPin, getNeoTypeForOrder(ledOrder));
  strip->begin();
  strip->clear();
  strip->show();
}

uint32_t makeColor(uint8_t r, uint8_t g, uint8_t b) {
  if (!strip) return 0;
  return strip->Color(r, g, b);
}

void setAllPixels(uint32_t color) {
  if (!strip) return;
  for (int i = 0; i < numLeds; i++) {
    strip->setPixelColor(i, color);
  }
  strip->show();
}

void applyCurrentLedColor() {
  updateLED(lastFlightRules);
}

void updateLED(String vfrStatus) {
  lastFlightRules = vfrStatus;

  uint8_t r = 0, g = 0, b = 0;

  if (vfrStatus == "VFR") {
    r = 0; g = 255; b = 0;
  } else if (vfrStatus == "MVFR") {
    r = 0; g = 0; b = 255;
  } else if (vfrStatus == "IFR") {
    r = 255; g = 0; b = 0;
  } else if (vfrStatus == "LIFR") {
    r = 255; g = 0; b = 255;
  }

  uint8_t sr = (uint8_t)((r * brightness) / 255);
  uint8_t sg = (uint8_t)((g * brightness) / 255);
  uint8_t sb = (uint8_t)((b * brightness) / 255);

  setAllPixels(makeColor(sr, sg, sb));
}

void setBootBlink(bool active) {
  bootBlinkActive = active;
  if (!active) bootBlinkOn = false;
}

void updateBootBlink() {
  if (!bootBlinkActive || !strip) return;
  if (hasValidMetar) {
    bootBlinkActive = false;
    return;
  }
  if (millis() - bootBlinkPrev >= 450) {
    bootBlinkPrev = millis();
    bootBlinkOn = !bootBlinkOn;
    if (bootBlinkOn) setAllPixels(makeColor(brightness, brightness, 0));
    else setAllPixels(makeColor(0, 0, 0));
  }
}

void setNoSignalState(bool active) {
  noSignalState = active;
  if (active && !hasValidMetar) {
    setBootBlink(false);
    setAllPixels(makeColor(brightness, brightness, 0));
  }
}

void updateAdminTestState() {
  if (adminTestActive && millis() > adminTestUntil) {
    adminTestActive = false;
    if (noSignalState && !hasValidMetar) {
      setAllPixels(makeColor(brightness, brightness, 0));
    } else if (hasValidMetar) {
      applyCurrentLedColor();
    }
  }
}

void applyAdminTestColor(const String& c) {
  uint32_t color = 0;
  if (c == "green") color = makeColor(0, brightness, 0);
  else if (c == "blue") color = makeColor(0, 0, brightness);
  else if (c == "red") color = makeColor(brightness, 0, 0);
  else if (c == "magenta") color = makeColor(brightness, 0, brightness);
  else if (c == "white") color = makeColor(brightness, brightness, brightness);
  else if (c == "yellow") color = makeColor(brightness, brightness, 0);
  else color = makeColor(0, 0, 0);

  adminTestActive = true;
  adminTestUntil = millis() + 15000UL;
  setAllPixels(color);
}

// ============================================================
// FLIGHT PULSE HELPERS
// ============================================================

bool isHex6(String s) {
  s.trim();
  s.toUpperCase();
  if (s.length() != 6) return false;
  for (int i = 0; i < 6; i++) {
    char c = s[i];
    bool ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F');
    if (!ok) return false;
  }
  return true;
}

int usN_to_icao_int(String tail) {
  tail.trim();
  tail.toUpperCase();

  const String base9  = "123456789";
  const String base10 = "0123456789";
  const String base34 = "ABCDEFGHJKLMNPQRSTUVWXYZ0123456789";

  const int icaooffset = 0xA00001;
  const int b1 = 101711;
  const int b2 = 10111;

  if (tail.length() < 2) return -1;
  if (tail[0] != 'N') return -1;

  int d1 = base9.indexOf(tail[1]);
  if (d1 < 0) return -1;

  int icao = icaooffset + d1 * b1;
  if (tail.length() == 2) return icao;

  auto enc_suffix = [&](String suf) -> int {
    if (suf.length() == 0) return 0;
    suf.toUpperCase();
    int r0 = base34.indexOf(suf[0]);
    if (r0 < 0) return -9999;

    int r1;
    if (suf.length() == 1) r1 = 0;
    else {
      int idx = base34.indexOf(suf[1]);
      if (idx < 0) return -9999;
      r1 = idx + 1;
    }

    if (r0 < 24) return r0 * 25 + r1 + 1;
    return r0 * 35 + r1 - 239;
  };

  int d2 = base10.indexOf(tail[2]);
  if (d2 == -1) {
    String suf = tail.substring(2);
    if (suf.length() > 2) suf = suf.substring(0, 2);
    int enc = enc_suffix(suf);
    if (enc < 0) return -1;
    icao += enc;
    return icao;
  }

  icao += d2 * b2 + 601;
  if (tail.length() == 3) return icao;

  int d3 = base10.indexOf(tail[3]);
  if (d3 > -1) {
    icao += d3 * 951 + 601;
    String suf = "";
    if (tail.length() > 4) suf = tail.substring(4);
    if (suf.length() > 2) suf = suf.substring(0, 2);
    int enc = enc_suffix(suf);
    if (enc < 0) return -1;
    icao += enc;
    return icao;
  } else {
    String suf = tail.substring(3);
    if (suf.length() > 2) suf = suf.substring(0, 2);
    int enc = enc_suffix(suf);
    if (enc < 0) return -1;
    icao += enc;
    return icao;
  }
}

String usN_to_hex6(String tail) {
  int icao = usN_to_icao_int(tail);
  if (icao < 0) return "";
  char out[7];
  snprintf(out, sizeof(out), "%06X", icao);
  return String(out);
}

String resolveTailOrHex(String tailIn, String hexIn) {
  tailIn.trim(); tailIn.toUpperCase();
  hexIn.trim(); hexIn.toUpperCase();

  if (tailIn.length()) {
    String h = usN_to_hex6(tailIn);
    if (h.length() == 6) return h;
  }
  if (hexIn.length() && isHex6(hexIn)) return hexIn;
  return "";
}

bool fpFetchIsFlying_ADSBlol(const String& icaoHex6) {
  if (!useWiFi) return false;
  if (WiFi.status() != WL_CONNECTED) return false;
  if (!fpEnabled) return false;
  if (!isHex6(icaoHex6)) return false;

  WiFiClientSecure client;
  client.setInsecure();

  HttpClient http(client, "api.adsb.lol", 443);
  http.setTimeout(10000);

  String path = "/v2/icao/" + String(icaoHex6);
  path.toLowerCase();

  http.beginRequest();
  http.get(path);
  http.sendHeader("Accept", "application/json");
  http.endRequest();

  int code = http.responseStatusCode();
  if (code != 200) {
    Serial.print("[FP] ADSB HTTP Fail: ");
    Serial.println(code);
    http.stop();
    return false;
  }

  String body = http.responseBody();
  http.stop();



 

  DynamicJsonDocument doc(8192);
  if (deserializeJson(doc, body)) return false;

  JsonArray ac = doc["ac"].as<JsonArray>();
  if (ac.isNull() || ac.size() == 0) return false;

  JsonObject a = ac[0].as<JsonObject>();

  float gs = a["gs"] | 0.0;
  float alt_baro = a["alt_baro"] | 0.0;
  float alt_geom = a["alt_geom"] | 0.0;
  int gnd = a["gnd"] | 0;

  float alt = (alt_baro > 0.0f) ? alt_baro : alt_geom;

  if (gnd == 1) return false;
  if (alt > 500.0f || gs > 35.0f) return true;
  return false;
}

void fpStartPulse() {
  fpPulseActive = true;
  fpPulseStartMs = millis();
  fpBaseBrightness = brightness;
}

void fpStopPulseRestore() {
  fpPulseActive = false;
  if (!strip) return;
  strip->setBrightness((uint8_t)brightness);
  strip->show();
}

void fpUpdatePulseOverlay() {
  if (!fpPulseActive || !strip) return;

  unsigned long now = millis();
  float t = fmod((float)(now - fpPulseStartMs), FP_PERIOD_MS) / FP_PERIOD_MS;
  float wave = 0.5f - 0.5f * cosf(2.0f * PI * t);

  int minB = (int)(fpBaseBrightness * FP_MIN_FRACTION);
  if (minB < 3) minB = 3;
  int maxB = fpBaseBrightness;
  if (maxB < 3) maxB = 3;

  int b = (int)(minB + (maxB - minB) * wave);
  strip->setBrightness((uint8_t)b);
  strip->show();
}

void updateFlightPulse() {
  if (scheduleCurrentlyOff) {
    if (fpPulseActive) fpStopPulseRestore();
    return;
  }

  if (adminTestActive || bootBlinkActive || (noSignalState && !hasValidMetar)) {
    if (fpPulseActive) fpStopPulseRestore();
    return;
  }

  if (!fpEnabled || !useWiFi || WiFi.status() != WL_CONNECTED || fpHex.length() != 6) {
    if (fpPulseActive) fpStopPulseRestore();
    return;
  }

  unsigned long now = millis();

  if (now - fpLastCheckMs >= fpCheckIntervalMs) {
    fpLastCheckMs = now;

    bool flyingNow = fpFetchIsFlying_ADSBlol(fpHex);

    if (flyingNow) fpFlyingStreak = min(fpFlyingStreak + 1, 3);
    else fpFlyingStreak = max(fpFlyingStreak - 1, -3);

    bool flyingDebounced = (fpFlyingStreak >= 2);

    if (flyingDebounced != fpIsFlying) {
      fpIsFlying = flyingDebounced;
      if (fpIsFlying && !fpPulseActive) fpStartPulse();
      if (!fpIsFlying && fpPulseActive) fpStopPulseRestore();
    }
  }

  fpUpdatePulseOverlay();
}

// ============================================================
// OTA HELPERS
// ============================================================

bool otaParseRawUrl(const String& url, String& host, int& port, String& path) {
  host = "";
  port = 443;
  path = "";

  if (!url.startsWith("https://")) return false;

  String s = url.substring(8);
  int slash = s.indexOf('/');
  if (slash < 0) return false;

  host = s.substring(0, slash);
  path = s.substring(slash);
  return host.length() && path.length();
}

bool otaGetLatest(String& outVersion, String& outUrl, int& outSize) {
  outVersion = "";
  outUrl = "";
  outSize = 0;

  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client;
  client.setInsecure();

  HttpClient http(client, otaManifestHost.c_str(), otaManifestPort);
  http.setTimeout(12000);
  http.connectionKeepAlive();

  http.beginRequest();
  http.get(otaManifestPath.c_str());
  http.sendHeader("Accept", "application/json");
  http.endRequest();

  int statusCode = http.responseStatusCode();
  if (statusCode != 200) {
    http.stop();
    return false;
  }

  String payload = http.responseBody();
  http.stop();

  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) return false;

  outVersion = doc["version"] | "";
  outUrl     = doc["url"] | "";
  outSize    = doc["size"] | 0;

  return outVersion.length() > 0 && outUrl.length() > 0;
}

bool otaInstallNow() {
  if (WiFi.status() != WL_CONNECTED) return false;

  String newVersion, binUrl;
  int expectedSize = 0;

  if (!otaGetLatest(newVersion, binUrl, expectedSize)) return false;
  if (newVersion == String(FW_VERSION)) return false;

  String host, path;
  int port = 443;
  if (!otaParseRawUrl(binUrl, host, port, path)) return false;

  WiFiClientSecure client;
  client.setInsecure();

  HttpClient http(client, host.c_str(), port);
  http.setTimeout(30000);
  http.connectionKeepAlive();

  http.beginRequest();
  http.get(path.c_str());
  http.sendHeader("Accept", "application/octet-stream");
  http.endRequest();

  int statusCode = http.responseStatusCode();
  if (statusCode != 200) {
    http.stop();
    return false;
  }

  int contentLength = http.contentLength();
  if (contentLength <= 0 && expectedSize > 0) contentLength = expectedSize;

  if (!Update.begin(contentLength > 0 ? contentLength : UPDATE_SIZE_UNKNOWN)) {
    http.stop();
    return false;
  }

  size_t written = Update.writeStream(http);
  http.stop();

  if (written == 0) return false;
  if (!Update.end()) return false;
  if (!Update.isFinished()) return false;

  delay(1000);
  ESP.restart();
  return true;
}

// ============================================================
// MODEM / CELL RECONNECT
// ============================================================

void serviceDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    server.handleClient();
    if (bootBlinkActive) updateBootBlink();
    delay(20);
  }
}

String cleanAT(String r) {
  r.replace("\r", " ");
  r.replace("\n", " ");
  r.replace("  ", " ");
  r.trim();
  return r;
}

String modemAT(const String& cmd, unsigned long timeoutMs) {
  while (SerialAT.available()) SerialAT.read();

  Serial.print("[AT] ");
  Serial.println(cmd);
  SerialAT.println(cmd);

  String resp;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (SerialAT.available()) {
      char c = (char)SerialAT.read();
      resp += c;
      if (resp.indexOf("\r\nOK\r\n") >= 0 || resp.indexOf("\nOK\r") >= 0 ||
          resp.indexOf("\r\nERROR\r\n") >= 0 || resp.indexOf("\nERROR\r") >= 0) {
        Serial.print("[AT<-] ");
        Serial.println(cleanAT(resp));
        return cleanAT(resp);
      }
    }
    server.handleClient();
    if (bootBlinkActive) updateBootBlink();
    delay(10);
  }

  Serial.print("[AT timeout<-] ");
  Serial.println(cleanAT(resp));
  return cleanAT(resp);
}

void updateCellularDiagnostics(const String& reason) {
  Serial.println("[CELL] Diagnostics: " + reason);

  cellDiagSignal = modem.getSignalQuality();
  cellDiagIP = modem.getLocalIP();
  cellDiagIpAddrRaw = modemAT("AT+IPADDR", 3000);
  cellDiagNetopen = modemAT("AT+NETOPEN?", 3000);
  cellDiagCgatt = modemAT("AT+CGATT?", 3000);
  cellDiagCgact = modemAT("AT+CGACT?", 3000);
  cellDiagCgpaddr = modemAT("AT+CGPADDR=1", 3000);
  cellDiagOperator = modemAT("AT+COPS?", 2500);
  cellDiagReg = modemAT("AT+CREG?", 2000);
  cellDiagCereg = modemAT("AT+CEREG?", 2000);
  cellDiagCpsi = modemAT("AT+CPSI?", 3000);
  cellDiagCeer = modemAT("AT+CEER", 3000);
  cellDiagUpdatedMs = millis();

  Serial.print("[CELL] Signal: "); Serial.println(cellDiagSignal);
  Serial.print("[CELL] IP: "); Serial.println(cellDiagIP);

  String state = cellularSignalSummaryText() + " | " + cellCarrierLabel() + " | " + cellNetworkLabel() + " | IP " + (cellDiagIP.length() ? cellDiagIP : String("unknown"));
  if (state != cellLastLoggedState) {
    logEvent("CELL", reason + " -> " + state);
    cellLastLoggedState = state;
  }
}


bool isValidCellularIP(const String& ipIn) {
  String ip = ipIn;
  ip.trim();
  if (ip.length() == 0) return false;
  if (ip == "0.0.0.0") return false;
  if (ip.indexOf("NOT") >= 0) return false;
  if (ip.indexOf("ERROR") >= 0) return false;
  return true;
}

bool regResponseShowsRegistered(String r) {
  r.replace(" ", "");
  // Registered home = 1, registered roaming = 5. 1NCE roaming usually reports 5.
  return (r.indexOf(",1") >= 0 || r.indexOf(",5") >= 0);
}

bool cachedCellularRegistrationReady() {
  return regResponseShowsRegistered(cellDiagReg) || regResponseShowsRegistered(cellDiagCereg);
}

bool cellularNoServiceDetected() {
  String cpsi = cellDiagCpsi;
  cpsi.toUpperCase();

  bool saysNoService = (cpsi.indexOf("NO SERVICE") >= 0);
  bool csqUnknown = (cellDiagSignal == 99 || cellDiagSignal < 0);
  bool notRegistered = !cachedCellularRegistrationReady();
  bool badIp = !isValidCellularIP(cellDiagIP);

  return saysNoService && csqUnknown && notRegistered && badIp;
}

bool cellularRegisteredButNoIpDetected() {
  String cpsi = cellDiagCpsi;
  cpsi.toUpperCase();

  bool registered = cachedCellularRegistrationReady();
  bool lteOnline = (cpsi.indexOf("LTE") >= 0 && cpsi.indexOf("ONLINE") >= 0);
  bool badIp = !isValidCellularIP(cellDiagIP);
  bool notNoService = (cpsi.indexOf("NO SERVICE") < 0);

  return registered && lteOnline && badIp && notNoService;
}

void resetCellularDataFailureCounters() {
  cellRegisteredNoIpStreak = 0;
  cellDataOpenFailCycles = 0;
  cellLastDataFailureReason = "";
}

void resetCellularDataSessionOnly(const String& reason) {
  Serial.println("[CELL] Data-session reset only: " + reason);
  logEvent("CELL", "Data-session reset only: " + reason);
  lastStatusLine = "Cellular data-session reset";

#if MLW_CELLULAR_BUILTIN_HTTPS
  modem.https_end();
  modem.setNetworkDeactivate();
  modemAT("AT+NETCLOSE", 15000);
#else
  modem.gprsDisconnect();
#endif
  serviceDelay(2000);

  String cgdc = "AT+CGDCONT=1,\"IP\",\"" + apn + "\"";
  modemAT(cgdc, 3000);
  modemAT("AT+CGAUTH=1,0", 3000); // no PAP/CHAP auth unless provider explicitly requires it
#if MLW_CELLULAR_BUILTIN_HTTPS
  modem.setNetworkAPN(apn.c_str());
#endif
  cellDiagIpAddrRaw = modemAT("AT+IPADDR", 3000);
}

bool noteCellularDataFailureCycle(const String& reason) {
  cellLastDataFailureReason = reason;

  if (!cellularRecoveryShouldMonitor()) return false;
  if (cellHardRecoveryInProgress) {
    Serial.println("[CELL] Data failure noted during hard recovery; not nesting another hard recovery");
    return false;
  }

  if (cellDataOpenFailCycles < 250) cellDataOpenFailCycles++;

  if (cellularRegisteredButNoIpDetected()) {
    if (cellRegisteredNoIpStreak < 250) cellRegisteredNoIpStreak++;
    Serial.println("[CELL] Registered but no IP streak " + String(cellRegisteredNoIpStreak) + "/" + String(CELL_REGISTERED_NO_IP_RECOVERY_THRESHOLD));
  } else if (cellularNoServiceDetected()) {
    if (cellNoServiceStreak < 250) cellNoServiceStreak++;
    if (cellUnstableServiceEvents < 65000) cellUnstableServiceEvents++;
    Serial.println("[CELL] Data failure ended in NO SERVICE; no-service streak " + String(cellNoServiceStreak));
  } else {
    if (cellUnstableServiceEvents < 65000) cellUnstableServiceEvents++;
  }

  Serial.println("[CELL] Data-open failure cycle " + String(cellDataOpenFailCycles) + "/" + String(CELL_DATA_FAIL_RECOVERY_THRESHOLD) + " | " + reason);
  logEvent("CELL", "Data-open failure cycle " + String(cellDataOpenFailCycles) + "/" + String(CELL_DATA_FAIL_RECOVERY_THRESHOLD) + " | " + reason);

  bool triggerDataFail = cellDataOpenFailCycles >= CELL_DATA_FAIL_RECOVERY_THRESHOLD;
  bool triggerRegisteredNoIp = cellRegisteredNoIpStreak >= CELL_REGISTERED_NO_IP_RECOVERY_THRESHOLD;

  if (triggerDataFail || triggerRegisteredNoIp) {
    String why = "data/PDP did not open";
    why += " | cycles " + String(cellDataOpenFailCycles);
    why += " | registered-no-IP " + String(cellRegisteredNoIpStreak);
    why += " | " + reason;
    return hardRestartCellularModem(why);
  }

  return false;
}

bool cellularRecoveryShouldMonitor() {
  if (!cellularInitialized) return false;
  if (!cellularModeAllowed()) return false;

  // In Auto mode with WiFi connected, do not keep poking the modem unless the
  // admin cellular test is active. Customer web UI and OTA can keep using WiFi.
  if (useWiFi && WiFi.status() == WL_CONNECTED) {
    return cellDataTestEnabled && cellDataTestPath == "cellular";
  }

  return true;
}

bool waitForModemATAfterReset(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (modem.testAT(1000)) {
      Serial.println("[CELL] Modem responded after hard recovery reset");
      return true;
    }
    server.handleClient();
    if (bootBlinkActive) updateBootBlink();
    delay(500);
  }
  return false;
}

void configureModemRuntimeAfterReset(const String& reason) {
  Serial.println("[CELL] Reconfiguring modem after reset: " + reason);
  cellularInitialized = true;
  cellularDataReady = false;

  modemAT("AT", 2000);
  modemAT("ATE0", 2000);
  modemAT("AT+CMEE=2", 2000);
  modemAT("AT+CREG=2", 2000);
  modemAT("AT+CEREG=2", 2000);

  unsigned long simStart = millis();
  while (millis() - simStart < 30000UL) {
    SimStatus sim = modem.getSimStatus();
    if (sim == SIM_READY) {
      Serial.println("[CELL] SIM ready after modem reset");
      break;
    }
    Serial.println("[CELL] Waiting for SIM ready after modem reset...");
    serviceDelay(1000);
  }

#ifdef TINY_GSM_MODEM_HAS_NETWORK_MODE
  modem.setNetworkMode(MODEM_NETWORK_AUTO);
#endif
#if MLW_CELLULAR_BUILTIN_HTTPS
  modem.setNetworkAPN(apn.c_str());
  modemAT("AT+SIMCOMATI", 10000);
#endif

  updateCellularDiagnostics("after hard modem reset");
}

bool hardRestartCellularModem(const String& reason) {
  if (cellHardRecoveryInProgress) {
    Serial.println("[CELL] Hard modem recovery already in progress");
    return false;
  }

  unsigned long now = millis();
  if (lastCellNoServiceRecoveryMs > 0 && now - lastCellNoServiceRecoveryMs < CELL_NOSERVICE_RECOVERY_COOLDOWN_MS) {
    Serial.println("[CELL] Hard modem recovery suppressed by cooldown");
    logEvent("RECOVERY", "Hard modem recovery suppressed by cooldown: " + reason);
    cellLastError = "Hard recovery suppressed by cooldown";
    return false;
  }

  cellHardRecoveryInProgress = true;
  lastCellNoServiceRecoveryMs = now;
  cellHardRecoveryCount++;
  cellNoServiceStreak = 0;
  cellRegisteredNoIpStreak = 0;
  cellDataOpenFailCycles = 0;
  cellLastHardRecoveryReason = reason;
  cellLastError = "Hard modem recovery: " + reason;
  lastStatusLine = "Hard modem recovery";

  Serial.println("[CELL] HARD MODEM RECOVERY START: " + reason);
  logEvent("RECOVERY", "HARD MODEM RECOVERY START: " + reason);

  if (fpPulseActive) fpStopPulseRestore();

#if MLW_CELLULAR_BUILTIN_HTTPS
  modem.https_end();
  modem.setNetworkDeactivate();
#else
  modem.gprsDisconnect();
#endif
  serviceDelay(1000);

  // Reset only the modem side. This preserves ESP32 uptime, saved settings,
  // customer web UI, and the last good METAR LED color.
  modemAT("AT+CFUN=1,1", 8000);
  serviceDelay(12000);

  if (!waitForModemATAfterReset(60000UL)) {
    Serial.println("[CELL] No AT after CFUN reset; trying hardware reset pin fallback if available");
#ifdef MODEM_RESET_PIN
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    delay(100);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
    delay(2600);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    serviceDelay(8000);
#endif
    if (!waitForModemATAfterReset(45000UL)) {
      cellLastError = "Hard modem recovery failed: no AT response after reset";
      lastStatusLine = "Hard modem recovery failed";
      cellularInitialized = false;
      cellularDataReady = false;
      Serial.println("[CELL] HARD MODEM RECOVERY FAILED: no AT response");
      logEvent("RECOVERY", "HARD MODEM RECOVERY FAILED: no AT response");
      cellHardRecoveryInProgress = false;
      return false;
    }
  }

  configureModemRuntimeAfterReset(reason);

  bool ok = connectCellularData(true, "hard modem recovery");
  if (ok) {
    cellNoServiceStreak = 0;
    lastStatusLine = "Cellular recovered after modem restart";
    cellLastError = "";
    resetCellularDataFailureCounters();
    Serial.println("[CELL] HARD MODEM RECOVERY OK");
    logEvent("RECOVERY", "HARD MODEM RECOVERY OK");
    cellHardRecoveryInProgress = false;
    return true;
  }

  lastStatusLine = "Hard modem recovery did not restore data";
  if (!cellLastError.length()) cellLastError = "Hard modem recovery did not restore data";
  Serial.println("[CELL] HARD MODEM RECOVERY DID NOT RESTORE DATA");
  logEvent("RECOVERY", "HARD MODEM RECOVERY DID NOT RESTORE DATA");
  cellHardRecoveryInProgress = false;
  return false;
}

void evaluateCellularRecovery(const String& reason) {
  if (!cellularRecoveryShouldMonitor()) return;

  if (cellularNoServiceDetected()) {
    if (cellNoServiceStreak < 250) cellNoServiceStreak++;
    if (cellRegisteredNoIpStreak > 0) cellUnstableServiceEvents++;
    cellRegisteredNoIpStreak = 0;
    Serial.println("[CELL] Persistent NO SERVICE streak " + String(cellNoServiceStreak) + "/" + String(CELL_NOSERVICE_RECOVERY_THRESHOLD) + " | " + reason);
    logEvent("CELL", "NO SERVICE streak " + String(cellNoServiceStreak) + "/" + String(CELL_NOSERVICE_RECOVERY_THRESHOLD) + " | " + reason);
  } else if (cellularRegisteredButNoIpDetected()) {
    if (cellNoServiceStreak > 0) cellUnstableServiceEvents++;
    cellNoServiceStreak = 0;
    if (cellRegisteredNoIpStreak < 250) cellRegisteredNoIpStreak++;
    Serial.println("[CELL] Registered but no IP streak " + String(cellRegisteredNoIpStreak) + "/" + String(CELL_REGISTERED_NO_IP_RECOVERY_THRESHOLD) + " | " + reason);
    logEvent("CELL", "Registered but no IP streak " + String(cellRegisteredNoIpStreak) + "/" + String(CELL_REGISTERED_NO_IP_RECOVERY_THRESHOLD) + " | " + reason);
  } else {
    if (cellNoServiceStreak > 0) Serial.println("[CELL] NO SERVICE streak cleared");
    if (cellRegisteredNoIpStreak > 0) Serial.println("[CELL] Registered-no-IP streak cleared");
    cellNoServiceStreak = 0;
    cellRegisteredNoIpStreak = 0;
    if (cellularDataReady && isValidCellularIP(cellDiagIP)) resetCellularDataFailureCounters();
    return;
  }

  if (cellNoServiceStreak >= CELL_NOSERVICE_RECOVERY_THRESHOLD) {
    hardRestartCellularModem("persistent NO SERVICE / CSQ99 detected during " + reason);
  } else if (cellRegisteredNoIpStreak >= CELL_REGISTERED_NO_IP_RECOVERY_THRESHOLD) {
    hardRestartCellularModem("registered LTE but IP/data session stayed closed during " + reason);
  }
}

void serviceCellularRecovery() {
  if (!cellularRecoveryShouldMonitor()) return;

  unsigned long now = millis();
  if (now - lastPassiveCellDiagMs < CELL_PASSIVE_DIAG_INTERVAL_MS) return;
  lastPassiveCellDiagMs = now;

  updateCellularDiagnostics("passive no-service monitor");
  evaluateCellularRecovery("passive monitor");
}

bool sim7600BuiltInHttpsReadyCached(const char* context) {
#if MLW_CELLULAR_BUILTIN_HTTPS
  String ip = modem.getLocalIP();
  ip.trim();
  if (isValidCellularIP(ip)) cellDiagIP = ip;

  bool ipReady = isValidCellularIP(cellDiagIP);
  bool regReady = cachedCellularRegistrationReady();
  bool ready = cellularDataReady && ipReady && regReady;

  Serial.print("[CELL] SIM7600 ready check ");
  Serial.print(context ? context : "");
  Serial.print(": dataReady="); Serial.print(cellularDataReady ? "yes" : "no");
  Serial.print(" ipReady="); Serial.print(ipReady ? "yes" : "no");
  Serial.print(" regReady="); Serial.print(regReady ? "yes" : "no");
  Serial.print(" ip="); Serial.println(cellDiagIP.length() ? cellDiagIP : String("unknown"));

  return ready;
#else
  (void)context;
  return modem.isNetworkConnected() && modem.isGprsConnected() && cellularDataReady;
#endif
}

bool sim7600BuiltInHttpsReadyWithRecheck(const char* context) {
#if MLW_CELLULAR_BUILTIN_HTTPS
  if (sim7600BuiltInHttpsReadyCached(context)) return true;
  updateCellularDiagnostics(String("ready recheck: ") + (context ? context : "cellular"));
  return sim7600BuiltInHttpsReadyCached("after diagnostics");
#else
  (void)context;
  return modem.isNetworkConnected() && modem.isGprsConnected() && cellularDataReady;
#endif
}

void forceAutomaticOperatorSelection() {
  lastStatusLine = "Forcing automatic cellular operator";
  cellLastError = "";

  // Make modem errors more descriptive and make sure registration reporting is enabled.
  modemAT("AT+CMEE=2", 2000);
  modemAT("AT+CREG=2", 2000);
  modemAT("AT+CEREG=2", 2000);

  // Full functionality and automatic operator selection.
  modemAT("AT+CFUN=1", 10000);
  String r = modemAT("AT+COPS=0", 90000);
  if (r.indexOf("ERROR") >= 0) {
    cellLastError = "COPS auto select returned ERROR";
  }

  updateCellularDiagnostics("after COPS=0");
}

bool waitForCellularNetwork(unsigned long timeoutMs) {
  lastStatusLine = "Waiting for cellular network";
  unsigned long start = millis();
  unsigned long lastDiag = 0;

  while (millis() - start < timeoutMs) {
    if (modem.isNetworkConnected()) {
      updateCellularDiagnostics("network registered");
      lastStatusLine = "Cellular network registered";
      return true;
    }

    if (millis() - lastDiag > 10000UL) {
      lastDiag = millis();
      updateCellularDiagnostics("waiting for network");
    }
    serviceDelay(1000);
  }

  cellLastError = "No cellular registration before timeout";
  lastStatusLine = "Cellular network timeout";
  updateCellularDiagnostics("network timeout");
  evaluateCellularRecovery("network timeout");
  return false;
}

bool waitForCellularDataReady(unsigned long timeoutMs) {
  lastStatusLine = "Waiting for cellular data/IP";
  unsigned long start = millis();
  unsigned long lastDiag = 0;

  while (millis() - start < timeoutMs) {
#if MLW_CELLULAR_BUILTIN_HTTPS
    bool dataSession = true;
#else
    bool dataSession = modem.isGprsConnected();
#endif
    String ip = modem.getLocalIP();
    ip.trim();

    if (dataSession && ip.length() > 0 && ip != "0.0.0.0") {
      cellDiagIP = ip;
      cellularDataReady = true;
      updateCellularDiagnostics("data ready");
      lastStatusLine = "Cellular data ready";
      return true;
    }

    if (millis() - lastDiag > 10000UL) {
      lastDiag = millis();
      updateCellularDiagnostics("waiting for data/IP");
    }
    serviceDelay(1000);
  }

  cellularDataReady = false;
  cellLastError = "No cellular IP/data before timeout";
  lastStatusLine = "Cellular data timeout";
  updateCellularDiagnostics("data timeout");
  evaluateCellularRecovery("data timeout");
  return false;
}

void initCellular() {
  lastStatusLine = "Powering modem";
  Serial.println("\n[MODEM] Powering modem...");

#ifdef BOARD_POWERON_PIN
  pinMode(BOARD_POWERON_PIN, OUTPUT);
  digitalWrite(BOARD_POWERON_PIN, HIGH);
#endif

#ifdef MODEM_RESET_PIN
  pinMode(MODEM_RESET_PIN, OUTPUT);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
  delay(100);
  digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
  delay(2600);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
#endif

#ifdef MODEM_FLIGHT_PIN
  // SIM7600 boards have a flight/airplane-mode control pin. HIGH exits airplane mode.
  pinMode(MODEM_FLIGHT_PIN, OUTPUT);
  digitalWrite(MODEM_FLIGHT_PIN, HIGH);
#endif

#ifdef MODEM_DTR_PIN
  // Keep DTR low so the modem does not remain in sleep.
  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, LOW);
#endif

  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(MODEM_POWERON_PULSE_WIDTH_MS);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  serviceDelay(5000);

  Serial.println("[MODEM] Checking modem connection...");
  int retry = 0;
  while (!modem.testAT(1000)) {
    Serial.print(".");
    server.handleClient();
    updateBootBlink();

    if (retry++ > 10) {
      Serial.println("\n[MODEM] Restarting modem...");
      digitalWrite(BOARD_PWRKEY_PIN, LOW);
      delay(100);
      digitalWrite(BOARD_PWRKEY_PIN, HIGH);
      delay(MODEM_POWERON_PULSE_WIDTH_MS);
      digitalWrite(BOARD_PWRKEY_PIN, LOW);
      retry = 0;
      serviceDelay(5000);
    }
  }

  Serial.println("\n[MODEM] Modem is ready.");
  cellularInitialized = true;
  lastStatusLine = "Modem ready";

  modemAT("AT", 2000);
  modemAT("ATE0", 2000);
  modemAT("AT+CMEE=2", 2000);

  // Helpful on SIM7600: verify the SIM is ready before attempting network attach.
  unsigned long simStart = millis();
  while (millis() - simStart < 30000UL) {
    SimStatus sim = modem.getSimStatus();
    if (sim == SIM_READY) {
      Serial.println("[MODEM] SIM ready");
      break;
    }
    Serial.println("[MODEM] Waiting for SIM ready...");
    serviceDelay(1000);
  }

#ifdef TINY_GSM_MODEM_HAS_NETWORK_MODE
  modem.setNetworkMode(MODEM_NETWORK_AUTO);
#endif
#if MLW_CELLULAR_BUILTIN_HTTPS
  modemAT("AT+SIMCOMATI", 10000);
#endif

  updateCellularDiagnostics("modem ready");
}

bool connectCellularData(bool forceAutoSearch, const String& reason) {
  if (!cellularInitialized) initCellular();

  lastCellConnectStartMs = millis();
  Serial.println("[CELL] Connect cellular data: " + reason);
  logEvent("CELL", "Connect cellular data: " + reason);
  lastStatusLine = "Cellular connect: " + reason;
  cellLastError = "";
  cellularDataReady = false;

  if (forceAutoSearch) {
    Serial.println("[CELL] Operator rescue enabled for this connect; forcing automatic operator selection");
    forceAutomaticOperatorSelection();
  } else {
    Serial.println("[CELL] Normal connect; leaving current operator/radio registration alone");
  }

  // Make sure the 1NCE APN is in the PDP profile before enabling data.
  String cgdc = "AT+CGDCONT=1,\"IP\",\"" + apn + "\"";
  modemAT(cgdc, 3000);
#if MLW_CELLULAR_BUILTIN_HTTPS
  modem.setNetworkAPN(apn.c_str());
#endif

  if (!waitForCellularNetwork(CELL_NETWORK_TIMEOUT_MS)) {
    return false;
  }

  for (int attempt = 1; attempt <= 3; attempt++) {
#if MLW_CELLULAR_BUILTIN_HTTPS
    Serial.println("[CELL] SIM7600 network-active attempt " + String(attempt));
    logEvent("CELL", "SIM7600 data attempt " + String(attempt));
    lastStatusLine = "SIM7600 data attempt " + String(attempt);
    modem.setNetworkDeactivate();
    modemAT("AT+NETCLOSE", 15000);
    serviceDelay(2500);
    bool ok = modem.setNetworkActive(apn, false);
#else
    Serial.println("[CELL] GPRS connect attempt " + String(attempt));
    lastStatusLine = "GPRS connect attempt " + String(attempt);
    modem.gprsDisconnect();
    serviceDelay(1500);
    bool ok = modem.gprsConnect(apn.c_str(), "", "");
#endif

    if (ok) {
      Serial.println("[CELL] data attach returned true");
      logEvent("CELL", "Data attach returned true");
      if (waitForCellularDataReady(CELL_DATA_TIMEOUT_MS)) {
        Serial.println("[CELL] Settling cellular data before first HTTPS request...");
        lastStatusLine = "Cellular data settling";
        serviceDelay(CELL_SETTLE_MS);
        lastStatusLine = "Cellular connected";
        bootCellConnectDurationMs = millis() - lastCellConnectStartMs;
        resetCellularDataFailureCounters();
        cellNoServiceStreak = 0;
        Serial.println("[CELL] Cellular connected in " + String(bootCellConnectDurationMs) + " ms");
        logEvent("CELL", "Cellular connected with IP " + cellDiagIP + " in " + String(bootCellConnectDurationMs) + " ms");
        return true;
      }
    } else {
      Serial.println("[CELL] data attach returned false");
      logEvent("CELL", "Data attach returned false on attempt " + String(attempt));
      cellLastError = "data attach failed attempt " + String(attempt);
      updateCellularDiagnostics("data attach failed");
    }

    if (attempt == 2) {
      // If we are still registered on LTE, do not immediately force operator reselection;
      // that can kick the SIM off a usable T-Mobile/AT&T roaming registration.
      updateCellularDiagnostics("before final data attach attempt");
      if (cachedCellularRegistrationReady()) {
        resetCellularDataSessionOnly("registered but data not open before final attempt");
      } else {
        forceAutomaticOperatorSelection();
      }
    }

    serviceDelay(5000);
  }

  cellularDataReady = false;
  lastStatusLine = "Cellular data connect failed";
  if (!cellLastError.length()) cellLastError = "Cellular data connect failed";
  updateCellularDiagnostics("data connect failed");
  bootCellConnectDurationMs = millis() - lastCellConnectStartMs;
  Serial.println("[CELL] Cellular connect failed after " + String(bootCellConnectDurationMs) + " ms");
  logEvent("CELL", "Cellular connect failed after " + String(bootCellConnectDurationMs) + " ms | " + cellLastError);

  if (noteCellularDataFailureCycle("connectCellularData failed after 3 attempts; last error: " + cellLastError)) {
    return cellularDataReady && isValidCellularIP(cellDiagIP);
  }

  return false;
}

bool reconnectCellularData() {
  Serial.println("[CELL] Reconnecting cellular data...");
  lastStatusLine = "Reconnecting cellular data";
  cellularDataReady = false;

#if MLW_CELLULAR_BUILTIN_HTTPS
  modem.setNetworkDeactivate();
#else
  modem.gprsDisconnect();
#endif
  serviceDelay(1500);

  if (connectCellularData(false, "reconnect")) {
    lastStatusLine = "Cellular reconnected";
    return true;
  }

  Serial.println("[CELL] Normal reconnect failed; trying automatic operator rescue...");
  cellLastError = "Normal reconnect failed; trying COPS=0 rescue";

  if (connectCellularData(true, "operator rescue")) {
    lastStatusLine = "Cellular reconnected after operator rescue";
    return true;
  }

  Serial.println("[CELL] Cellular data reconnect failed");
  lastStatusLine = "Cellular reconnect failed";
  return false;
}

void maintainWiFiConnection() {
  if (!wifiModeAllowed()) {
    if (useWiFi || WiFi.status() == WL_CONNECTED) {
      Serial.println("[WiFi] Cellular-only mode; disconnecting station WiFi");
      WiFi.disconnect(false, true);
    }
    useWiFi = false;
    currentFetchInterval = CELL_FETCH_INTERVAL_MS;
#if MLW_CELLULAR_BUILTIN_HTTPS
    bool maintainNeedsReconnect = !sim7600BuiltInHttpsReadyCached("maintain");
#else
    bool maintainNeedsReconnect = !modem.isNetworkConnected() || !modem.isGprsConnected() || !cellularDataReady;
#endif
    if (cellularModeAllowed() && maintainNeedsReconnect) {
      unsigned long now = millis();
      if (now - lastWiFiReconnectAttempt >= WIFI_RETRY_INTERVAL_MS) {
        lastWiFiReconnectAttempt = now;
        Serial.println("[CELL] Maintain path reconnect needed");
        connectCellularData(false, "cellular-only maintain");
      }
    }
    return;
  }

  if (wifiSsid.length() == 0) {
    useWiFi = false;
    currentFetchInterval = cellularModeAllowed() ? CELL_FETCH_INTERVAL_MS : WIFI_FETCH_INTERVAL_MS;
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!useWiFi) {
      useWiFi = true;
      currentFetchInterval = WIFI_FETCH_INTERVAL_MS;
      configTime(timezone * 3600, 0, "pool.ntp.org", "time.nist.gov");
      lastStatusLine = "WiFi connected; using WiFi";
      logEvent("WIFI", "Connected STA " + WiFi.localIP().toString());
      previousMillis = millis();
      if (fpPulseActive) fpStopPulseRestore();
    }
    return;
  }

  if (useWiFi) {
    useWiFi = false;
    if (cellularModeAllowed()) {
      currentFetchInterval = CELL_FETCH_INTERVAL_MS;
      lastStatusLine = "WiFi lost; using cellular fallback";
      if (!connectCellularData(false, "WiFi lost fallback")) {
        if (!hasValidMetar) setNoSignalState(true);
      }
    } else {
      currentFetchInterval = WIFI_FETCH_INTERVAL_MS;
      lastStatusLine = "WiFi lost; WiFi-only mode";
      logEvent("WIFI", "Lost STA WiFi in WiFi-only mode");
      if (!hasValidMetar) setNoSignalState(true);
    }
    if (fpPulseActive) fpStopPulseRestore();
  }

  unsigned long now = millis();
  if (now - lastWiFiReconnectAttempt >= WIFI_RETRY_INTERVAL_MS) {
    lastWiFiReconnectAttempt = now;
    Serial.println("[WiFi] Periodic reconnect attempt...");
    WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
  }
}

// ============================================================
// METAR FETCH / PARSE
// ============================================================

void parseMETAR(String response) {
  Serial.println("\n[METAR] Decoding response...");

  DynamicJsonDocument doc(12288);
  DeserializationError err = deserializeJson(doc, response);

  if (err) {
    Serial.print("[METAR] JSON parse failed: ");
    Serial.println(err.c_str());
    Serial.println("[METAR] First 500 chars:");
    Serial.println(response.substring(0, 500));
    lastStatusLine = "JSON parse failed";
    return;
  }

  if (!doc["raw"].isNull()) {
    lastRawMetar = doc["raw"].as<String>();
    Serial.print("[METAR] Raw: ");
    Serial.println(lastRawMetar);
  } else {
    lastRawMetar = "";
  }

  if (!doc["flight_rules"].isNull()) {
    String vfrStatus = doc["flight_rules"].as<String>();
    Serial.print("[METAR] Flight Rules: ");
    Serial.println(vfrStatus);
    hasValidMetar = true;
    setNoSignalState(false);
    setBootBlink(false);
    if (!adminTestActive) updateLED(vfrStatus);
    lastStatusLine = "METAR parsed successfully";
    logEvent("METAR", "Parsed " + airportCode + " flight rules " + vfrStatus);
  } else {
    Serial.println("[METAR] flight_rules not found");
    lastStatusLine = "flight_rules missing";
  }

  lastFetchMillis = millis();
}

String urlEncode(const String& value) {
  String encoded;
  const char *hex = "0123456789ABCDEF";
  for (size_t i = 0; i < value.length(); i++) {
    char c = value.charAt(i);
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
      encoded += c;
    } else {
      encoded += '%';
      encoded += hex[(c >> 4) & 0x0F];
      encoded += hex[c & 0x0F];
    }
  }
  return encoded;
}

String avwxPathWithToken(const String& path) {
  String out = path;
  out += (out.indexOf('?') >= 0) ? "&token=" : "?token=";
  out += urlEncode(avwxApiKey);
  return out;
}

int doCellularHttpsGetBuiltIn(const String& path, String& responseOut, const char* logTag) {
  responseOut = "";

#if !MLW_CELLULAR_BUILTIN_HTTPS
  (void)path;
  (void)logTag;
  return -799;
#else
  if (!sim7600BuiltInHttpsReadyWithRecheck(logTag)) {
    cellLastError = String(logTag) + " attempted before SIM7600 data ready";
    return -701;
  }

  String fullUrl = String("https://") + AVWX_HOST + avwxPathWithToken(path);
  Serial.print("["); Serial.print(logTag); Serial.print("] SIM7600 HTTPS URL: ");
  Serial.println(fullUrl.substring(0, fullUrl.indexOf("token=") + 6) + "***");

  modem.https_end();
  serviceDelay(250);
  modem.https_begin();

  if (!modem.https_set_url(fullUrl.c_str(), TINYGSM_SSL_AUTO)) {
    cellLastError = String(logTag) + " https_set_url failed";
    Serial.print("["); Serial.print(logTag); Serial.println("] https_set_url failed");
    modem.https_end();
    return -711;
  }

  int statusCode = modem.https_get();
  Serial.print("["); Serial.print(logTag); Serial.print("] SIM7600 HTTPS: ");
  Serial.println(statusCode);

  String header = modem.https_header();
  if (statusCode == 200) {
    responseOut = modem.https_body();
    Serial.print("["); Serial.print(logTag); Serial.print("] Response length: ");
    Serial.println(responseOut.length());
    if (responseOut.length() > 0) {
      Serial.print("["); Serial.print(logTag); Serial.println("] First 300 chars:");
      Serial.println(responseOut.substring(0, 300));
    }
  } else {
    if (header.length()) {
      Serial.print("["); Serial.print(logTag); Serial.print("] HTTPS header: ");
      Serial.println(header);
    }
  }

  recordCellularDataSample(logTag, statusCode, responseOut.length(), header.length());
  modem.https_end();
  return statusCode;
#endif
}

int doCellularMetarRequest(const String& path, String& responseOut) {
  responseOut = "";

#if MLW_CELLULAR_BUILTIN_HTTPS
  return doCellularHttpsGetBuiltIn(path, responseOut, "METAR");
#else
  noteFetchStart("Cellular", "METAR request");

  if (!modem.isNetworkConnected() || !modem.isGprsConnected() || !cellularDataReady) {
    cellLastError = "Cellular request attempted before data ready";
    return -701;
  }

  // A7670 SSL compatibility mode:
  // Match the 1.0.1 request behavior that was known to work.
  // Do not pre-stop gsmClient here and do not force Connection: close;
  // some A7670/TinyGSM firmware combinations close the TLS socket early.
  HttpClient http(gsmClient, AVWX_HOST, AVWX_PORT);
  http.setTimeout(30000);
  http.connectionKeepAlive();

  http.beginRequest();
  http.get(path.c_str());
  http.sendHeader("Authorization", avwxApiKey.c_str());
  http.sendHeader("Accept", "application/json");
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("[METAR] Cellular HTTP: ");
  Serial.println(statusCode);

  if (statusCode == 200) {
    responseOut = http.responseBody();
    Serial.println("[METAR] Response length: " + String(responseOut.length()));
    if (responseOut.length() > 0) {
      Serial.println("[METAR] First 300 chars:");
      Serial.println(responseOut.substring(0, 300));
    }
  }

  recordCellularDataSample("METAR", statusCode, responseOut.length(), 0);
  http.stop();
  return statusCode;
#endif
}

int doCellularLookupRequest(const String& path, String& responseOut) {
  responseOut = "";

#if MLW_CELLULAR_BUILTIN_HTTPS
  return doCellularHttpsGetBuiltIn(path, responseOut, "LOOKUP");
#else
  if (!modem.isNetworkConnected() || !modem.isGprsConnected() || !cellularDataReady) {
    cellLastError = "Lookup attempted before cellular data ready";
    return -701;
  }

  // A7670 SSL compatibility mode. Keep this the same as doCellularMetarRequest().
  HttpClient http(gsmClient, AVWX_HOST, AVWX_PORT);
  http.setTimeout(30000);
  http.connectionKeepAlive();

  http.beginRequest();
  http.get(path.c_str());
  http.sendHeader("Authorization", avwxApiKey.c_str());
  http.sendHeader("Accept", "application/json");
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("[LOOKUP] Cellular HTTP: ");
  Serial.println(statusCode);

  if (statusCode == 200) {
    responseOut = http.responseBody();
    Serial.println("[LOOKUP] Response length: " + String(responseOut.length()));
  }

  recordCellularDataSample("LOOKUP", statusCode, responseOut.length(), 0);
  http.stop();
  return statusCode;
#endif
}

void findNearbyAirport() {
  if (lookupInProgress) {
    Serial.println("[LOOKUP] Already in lookup fallback; skipping recursion.");
    return;
  }

  Serial.println("\n[LOOKUP] Looking up nearby airport for " + airportCode);
  String originalAirport = airportCode;
  String lookupPath = "/api/station/" + originalAirport + "/lookup?format=json";
  String response;
  int statusCode = -999;

  if (useWiFi && WiFi.status() == WL_CONNECTED) {
    wifiClient.stop();
    wifiClient.setInsecure();
    HttpClient http(wifiClient, AVWX_HOST, AVWX_PORT);
    http.setTimeout(30000);

    http.beginRequest();
    http.get(lookupPath.c_str());
    http.sendHeader("Authorization", avwxApiKey.c_str());
    http.sendHeader("Accept", "application/json");
    http.sendHeader("Connection", "close");
    http.endRequest();

    statusCode = http.responseStatusCode();
    Serial.print("[LOOKUP] WiFi HTTP: ");
    Serial.println(statusCode);

    if (statusCode == 200) {
      response = http.responseBody();
      Serial.println("[LOOKUP] Response length: " + String(response.length()));
    }
    http.stop();
    wifiClient.stop();
  } else {
    if (!cellularModeAllowed()) {
      lastStatusLine = "Nearby lookup skipped; WiFi-only mode";
      return;
    }

#if MLW_CELLULAR_BUILTIN_HTTPS
    bool lookupNeedsReconnect = !sim7600BuiltInHttpsReadyWithRecheck("nearby lookup");
#else
    bool lookupNeedsReconnect = !modem.isNetworkConnected() || !modem.isGprsConnected() || !cellularDataReady;
#endif
    if (lookupNeedsReconnect) {
      Serial.println("[LOOKUP] Cellular data not ready; reconnecting before nearby lookup");
      if (!reconnectCellularData()) return;
    }

    statusCode = doCellularLookupRequest(lookupPath, response);

    if (statusCode == 502 || statusCode < 0) {
      Serial.println("[LOOKUP] Retry after reconnect...");
      if (reconnectCellularData()) {
        statusCode = doCellularLookupRequest(lookupPath, response);
      }
    }
  }

  if (statusCode != 200) {
    lastStatusLine = "Nearby lookup failed HTTP " + String(statusCode);
    return;
  }

  int icaoIndex = response.indexOf("\"icao\":\"");
  if (icaoIndex == -1) {
    lastStatusLine = "Nearby lookup no ICAO";
    return;
  }

  int startIcao = icaoIndex + 8;
  int endIcao = response.indexOf("\"", startIcao);
  if (endIcao == -1) {
    lastStatusLine = "Nearby lookup parse failed";
    return;
  }

  String newAirport = response.substring(startIcao, endIcao);
  newAirport.trim();
  newAirport.toUpperCase();

  if (newAirport.length() == 0 || newAirport == originalAirport) {
    lastStatusLine = "Nearby lookup returned same airport";
    return;
  }

  // Safety change: do not overwrite the customer's configured airport on a transient fetch failure.
  // Use the nearby station as a temporary fallback only.
  Serial.print("[LOOKUP] Temporary nearby airport: ");
  Serial.println(newAirport);
  lastStatusLine = "Temporary nearby airport " + newAirport;

  lookupInProgress = true;
  airportCode = newAirport;
  fetchMETAR();
  airportCode = originalAirport;
  lookupInProgress = false;
}

void fetchMETAR() {
  unsigned long fetchStart = millis();
  Serial.println("\n[METAR] Fetching METAR");
  String metarPath = "/api/metar/" + airportCode;

  String fetchOverride = forcedFetchPath;
  forcedFetchPath = "";
  bool forceCellularThisFetch = (fetchOverride == "cellular");
  bool forceWiFiThisFetch = (fetchOverride == "wifi");
  if (forceCellularThisFetch) Serial.println("[ADMTEST] This METAR fetch is forced over cellular; WiFi web access remains untouched.");
  if (forceWiFiThisFetch) Serial.println("[ADMTEST] This METAR fetch is forced over WiFi.");

  if (forceWiFiThisFetch && !(useWiFi && WiFi.status() == WL_CONNECTED)) {
    noteFetchStart("WiFi", "Admin test WiFi-only request");
    noteFetchFinish(-902, "Admin test WiFi-only requested but WiFi not connected", fetchStart);
    lastStatusLine = "Admin test WiFi-only fetch skipped; WiFi not connected";
    return;
  }

  if (!forceCellularThisFetch && useWiFi && WiFi.status() == WL_CONNECTED) {
    noteFetchStart("WiFi", "METAR request");
    lastStatusLine = "Fetching METAR over WiFi";
    wifiClient.stop();
    wifiClient.setInsecure();

    HttpClient http(wifiClient, AVWX_HOST, AVWX_PORT);
    http.setTimeout(30000);

    http.beginRequest();
    http.get(metarPath.c_str());
    http.sendHeader("Authorization", avwxApiKey.c_str());
    http.sendHeader("Accept", "application/json");
    http.sendHeader("Connection", "close");
    http.endRequest();

    int statusCode = http.responseStatusCode();
    Serial.print("[METAR] WiFi HTTP: ");
    Serial.println(statusCode);

    if (statusCode == 200) {
      String response = http.responseBody();
      Serial.println("[METAR] Response length: " + String(response.length()));
      if (response.length() > 0) {
        Serial.println("[METAR] First 300 chars:");
        Serial.println(response.substring(0, 300));
      }
      noteFetchFinish(statusCode, "WiFi METAR OK", fetchStart);
      parseMETAR(response);
      http.stop();
      wifiClient.stop();
      return;
    }

    http.stop();
    wifiClient.stop();
    noteFetchFinish(statusCode, "WiFi METAR failed", fetchStart);
    lastStatusLine = "WiFi METAR fetch failed";
    if (!hasValidMetar) setNoSignalState(true);
    if (!lookupInProgress) findNearbyAirport();
    return;
  }

  if (!forceCellularThisFetch && !cellularModeAllowed()) {
    noteFetchStart("none", "WiFi-only mode skip");
    noteFetchFinish(-901, "WiFi-only mode; cellular fetch skipped", fetchStart);
    lastStatusLine = "WiFi-only mode; cellular fetch skipped";
    if (!hasValidMetar) setNoSignalState(true);
    return;
  }

  if (forceCellularThisFetch && !cellularInitialized) {
    Serial.println("[ADMTEST] Cellular modem not initialized yet; connecting for admin cellular test.");
    if (!connectCellularData(false, "admin cellular test")) {
      if (!connectCellularData(true, "admin cellular test rescue")) {
        noteFetchStart("Cellular", "Admin test cellular request");
        noteFetchFinish(-703, "Admin test cellular connect failed", fetchStart);
        lastStatusLine = "Admin test cellular connect failed";
        return;
      }
    }
  }

  noteFetchStart("Cellular", forceCellularThisFetch ? "Admin test cellular METAR request" : "METAR request");

#if MLW_CELLULAR_BUILTIN_HTTPS
  bool metarNeedsReconnect = !sim7600BuiltInHttpsReadyWithRecheck("pre-METAR");
#else
  bool metarNeedsReconnect = !modem.isNetworkConnected() || !modem.isGprsConnected() || !cellularDataReady;
#endif
  if (metarNeedsReconnect) {
    Serial.println("[CELL] Cellular data not ready for METAR; reconnecting");
    if (!reconnectCellularData()) {
      noteFetchFinish(-702, "Cellular reconnect before METAR failed", fetchStart);
      if (!hasValidMetar) setNoSignalState(true);
      return;
    }
  } else {
#if MLW_CELLULAR_BUILTIN_HTTPS
    Serial.println("[CELL] SIM7600 cellular session already ready; skipping pre-METAR reconnect");
#endif
  }

  lastStatusLine = "Fetching METAR over cellular";

  String response;
  int statusCode = doCellularMetarRequest(metarPath, response);

  if (statusCode == 200) {
    noteFetchFinish(statusCode, "Cellular METAR OK", fetchStart);
    parseMETAR(response);
    return;
  }

  if (statusCode == 713 || statusCode == -701 || statusCode < 0) {
#if MLW_CELLULAR_BUILTIN_HTTPS
    Serial.println("[METAR] SIM7600 HTTPS stack not ready; soft retry before reconnect...");
    lastStatusLine = "SIM7600 HTTPS soft retry";
    serviceDelay(10000);
    updateCellularDiagnostics("before METAR soft retry");
    if (sim7600BuiltInHttpsReadyCached("METAR soft retry")) {
      statusCode = doCellularMetarRequest(metarPath, response);
      if (statusCode == 200) {
        noteFetchFinish(statusCode, "Cellular METAR OK after soft retry", fetchStart);
        parseMETAR(response);
        return;
      }
    }
#endif
    Serial.println("[METAR] Retry after reconnect...");
    lastStatusLine = "Retrying after cellular reconnect";

    if (reconnectCellularData()) {
      statusCode = doCellularMetarRequest(metarPath, response);
      if (statusCode == 200) {
        noteFetchFinish(statusCode, "Cellular METAR OK after reconnect", fetchStart);
        parseMETAR(response);
        return;
      }
    }
  } else if (statusCode == 502 || statusCode == 400) {
    Serial.println("[METAR] Server/modem HTTP error; retry after reconnect...");
    lastStatusLine = "Retrying after cellular reconnect";
    if (reconnectCellularData()) {
      statusCode = doCellularMetarRequest(metarPath, response);
      if (statusCode == 200) {
        noteFetchFinish(statusCode, "Cellular METAR OK after reconnect", fetchStart);
        parseMETAR(response);
        return;
      }
    }
  }

  noteFetchFinish(statusCode, "Cellular METAR failed", fetchStart);
  lastStatusLine = "Cellular METAR fetch failed";
  cellLastError = "METAR HTTP " + String(statusCode);
#if MLW_CELLULAR_BUILTIN_HTTPS
  cellularDataReady = sim7600BuiltInHttpsReadyCached("post-failure state");
#else
  cellularDataReady = modem.isGprsConnected();
#endif
  if (!hasValidMetar) setNoSignalState(true);
  if (!lookupInProgress) findNearbyAirport();
}

// ============================================================
// SCHEDULE
// ============================================================

void updateLEDSchedule() {
  if (adminTestActive || bootBlinkActive || (noSignalState && !hasValidMetar)) return;
  if (!ledScheduleEnabled) {
    scheduleCurrentlyOff = false;
    return;
  }
  if (!hasValidMetar) return;

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    ensureTime(timezone);
    if (!getLocalTime(&timeinfo)) return;
  }

  int currentMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;

  int onHour, onMin, offHour, offMin;
  sscanf(ledOnTime.c_str(), "%d:%d", &onHour, &onMin);
  sscanf(ledOffTime.c_str(), "%d:%d", &offHour, &offMin);

  int onTimeMinutes = onHour * 60 + onMin;
  int offTimeMinutes = offHour * 60 + offMin;

  bool shouldBeOn;
  if (onTimeMinutes < offTimeMinutes) {
    shouldBeOn = (currentMinutes >= onTimeMinutes && currentMinutes < offTimeMinutes);
  } else {
    shouldBeOn = (currentMinutes >= onTimeMinutes || currentMinutes < offTimeMinutes);
  }

  scheduleCurrentlyOff = !shouldBeOn;
  if (!shouldBeOn) {
    if (fpPulseActive) fpStopPulseRestore();
    setAllPixels(makeColor(0, 0, 0));
  } else {
    applyCurrentLedColor();
  }
}

// ============================================================
// WEB HANDLERS
// ============================================================

void handleRoot() {
  int scannedNetworks = -1;
  String scannedOptions = "";
  if (server.hasArg("scan")) {
    lastStatusLine = "Scanning WiFi networks";
    scannedOptions = buildWiFiScanOptions(scannedNetworks);
    lastStatusLine = "WiFi scan complete: " + String(scannedNetworks < 0 ? 0 : scannedNetworks) + " found";
  }

  String html = htmlHeader("METARLightworks GSM");

  html += "<div class='card'><h1>METARLightworks GSM</h1>";
  html += "<div class='muted'>Cellular METAR light with selectable Auto, Cellular-only, or WiFi-only connection mode.</div>";
  html += "<div class='divider'></div>";
  html += getStatusBlock();
  html += "<div class='divider'></div>";
  html += "<a class='btn' href='/'>Setup</a>";
  html += "<a class='btn btn2' href='/admin'>Admin</a>";
  html += "<a class='btn btn3' href='/refresh'>Refresh Now</a>";
  html += "</div>";

  html += getCustomerCellSignalHtml();

  html += "<div class='card'><h2>Main Setup</h2>";
  html += "<form method='POST' action='/save'>";

  html += "<div class='row'>";
  html += "<div><label>Brightness (0-255)</label><input type='number' min='0' max='255' name='brightness' value='" + String(brightness) + "'></div>";
  html += "<div><label>Airport Code</label><input type='text' name='airport' value='" + airportCode + "'></div>";
  html += "</div>";

  html += "<label>Connection Mode</label><select name='connectionMode'>";
  html += "<option value='auto'" + String(connectionMode == "auto" ? " selected" : "") + ">Auto: WiFi preferred, cellular fallback</option>";
  html += "<option value='cellular'" + String(connectionMode == "cellular" ? " selected" : "") + ">Cellular only</option>";
  html += "<option value='wifi'" + String(connectionMode == "wifi" ? " selected" : "") + ">WiFi only</option>";
  html += "</select>";
  html += "<div class='muted'>Auto uses WiFi rules when connected. If it falls back to cellular, METAR updates hourly and Flight Pulse/OTA pause.</div>";

  html += "<div id='wifi' class='divider'></div>";
  html += "<h3>WiFi Setup</h3>";
  html += "<a class='btn btn2' href='/?scan=1#wifi'>Refresh WiFi List</a>";
  if (scannedNetworks >= 0) {
    html += "<div class='muted'>Found " + String(scannedNetworks) + " network(s). Select one below or type the SSID manually.</div>";
    html += "<label>Detected WiFi Networks</label><select name='wifiSsidSelect' onchange=\"document.getElementById('wifiSsid').value=this.value\">";
    html += scannedOptions;
    html += "</select>";
  } else {
    html += "<div class='muted'>Press Refresh WiFi List to scan nearby networks.</div>";
  }
  html += "<label>WiFi SSID / Manual Entry</label><input id='wifiSsid' type='text' name='wifiSsid' value='" + htmlEscape(wifiSsid) + "'>";
  html += "<label>WiFi Password</label><input type='password' name='wifiPassword' value='" + htmlEscape(wifiPassword) + "'>";
  html += "<label>APN</label><input type='text' name='apn' value='" + htmlEscape(apn) + "'>";

  html += "<button class='btn btn4' type='submit' formaction='/forgetWifi' formmethod='POST' onclick=\"return confirm('Forget saved WiFi SSID and password?');\">Forget Saved WiFi</button>";
  html += "<span class='muted'>Clears only WiFi SSID/password. Airport, APN, API key, brightness, and Flight Pulse settings stay saved.</span>";

  html += "<div class='row'>";
  html += "<div><label>LED On Time</label><input type='time' name='ledOnTime' value='" + ledOnTime + "'></div>";
  html += "<div><label>LED Off Time</label><input type='time' name='ledOffTime' value='" + ledOffTime + "'></div>";
  html += "</div>";

  html += "<label>Time Zone</label><select name='timezone'>";
  html += "<option value='-5'" + String(timezone == -5 ? " selected" : "") + ">Eastern (GMT-5)</option>";
  html += "<option value='-6'" + String(timezone == -6 ? " selected" : "") + ">Central (GMT-6)</option>";
  html += "<option value='-7'" + String(timezone == -7 ? " selected" : "") + ">Mountain (GMT-7)</option>";
  html += "<option value='-8'" + String(timezone == -8 ? " selected" : "") + ">Pacific (GMT-8)</option>";
  html += "</select>";

  html += "<label>LED Schedule</label><select name='ledScheduleEnabled'>";
  html += "<option value='1'" + String(ledScheduleEnabled ? " selected" : "") + ">Enabled</option>";
  html += "<option value='0'" + String(!ledScheduleEnabled ? " selected" : "") + ">Disabled</option>";
  html += "</select>";

  html += "<div style='margin-top:16px;'><button class='btn' type='submit'>Save & Reboot</button></div>";
  html += "</form></div>";

  html += "<div class='card'><h2>Flight Pulse</h2>";
  html += "<div class='muted'>WiFi only. No cellular data is used.</div>";
  html += "<form method='POST' action='/flightpulse'>";
  html += "<label>Enable Flight Pulse</label><select name='fpEnabled'>";
  html += "<option value='1'" + String(fpEnabled ? " selected" : "") + ">Enabled</option>";
  html += "<option value='0'" + String(!fpEnabled ? " selected" : "") + ">Disabled</option>";
  html += "</select>";
  html += "<div class='row'>";
  html += "<div><label>Tail Number</label><input type='text' name='fpTail' value='" + fpTail + "' placeholder='N247AP'></div>";
  html += "<div><label>ICAO Hex</label><input type='text' name='fpHex' value='" + fpHex + "' placeholder='A24A0E'></div>";
  html += "</div>";
  html += "<div style='margin-top:16px;'><button class='btn btn3' type='submit'>Save Flight Pulse</button></div>";
  html += "</form></div>";

  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleSave() {
  if (server.hasArg("brightness")) brightness = (uint8_t)server.arg("brightness").toInt();
  if (server.hasArg("airport")) airportCode = server.arg("airport");
  if (server.hasArg("connectionMode")) connectionMode = server.arg("connectionMode");
  if (server.hasArg("wifiSsid")) wifiSsid = server.arg("wifiSsid");
  if (server.hasArg("wifiPassword")) wifiPassword = server.arg("wifiPassword");
  if (server.hasArg("apn")) apn = server.arg("apn");
  if (server.hasArg("ledOnTime")) ledOnTime = server.arg("ledOnTime");
  if (server.hasArg("ledOffTime")) ledOffTime = server.arg("ledOffTime");
  if (server.hasArg("timezone")) timezone = server.arg("timezone").toInt();
  if (server.hasArg("ledScheduleEnabled")) ledScheduleEnabled = (server.arg("ledScheduleEnabled").toInt() == 1);

  if (brightness > 255) brightness = 255;

  connectionMode.trim(); connectionMode.toLowerCase();
  if (connectionMode != "auto" && connectionMode != "cellular" && connectionMode != "wifi") connectionMode = "auto";
  wifiSsid.trim();
  apn.trim();
  airportCode.trim(); airportCode.toUpperCase();
  saveMainConfig();

  String html = htmlHeader("Saved");
  html += "<div class='card'><h2>Settings Saved</h2><p>Rebooting now.</p></div>";
  html += htmlFooter();
  server.send(200, "text/html", html);

  delay(1500);
  ESP.restart();
}

void handleForgetWifi() {
  wifiSsid = "";
  wifiPassword = "";
  if (connectionMode == "wifi") connectionMode = "auto";
  saveMainConfig();

  WiFi.disconnect(false, true);
  useWiFi = false;
  currentFetchInterval = CELL_FETCH_INTERVAL_MS;
  lastStatusLine = "Saved WiFi cleared";

  String html = htmlHeader("WiFi Cleared");
  html += "<div class='card'><h2>Saved WiFi Cleared</h2><p>The saved WiFi SSID and password were cleared. Rebooting now.</p></div>";
  html += htmlFooter();
  server.send(200, "text/html", html);

  delay(1500);
  ESP.restart();
}

void handleFlightPulseSave() {
  fpEnabled = server.hasArg("fpEnabled") && server.arg("fpEnabled").toInt() == 1;

  String tailIn = server.hasArg("fpTail") ? server.arg("fpTail") : "";
  String hexIn  = server.hasArg("fpHex") ? server.arg("fpHex") : "";
  tailIn.trim(); tailIn.toUpperCase();
  hexIn.trim(); hexIn.toUpperCase();

  if (tailIn.length() > 0 || hexIn.length() > 0) {
    String resolved = resolveTailOrHex(tailIn, hexIn);
    if (resolved.length() != 6) {
      String html = htmlHeader("Flight Pulse Error");
      html += "<div class='card'><h2>Invalid Tail or Hex</h2><p>Use a valid US tail number or a valid 6-character ICAO hex.</p><a class='btn btn2' href='/'>Back</a></div>";
      html += htmlFooter();
      server.send(400, "text/html", html);
      return;
    }
    fpTail = tailIn.length() ? tailIn : "";
    fpHex = resolved;
  }

  if (!fpEnabled) {
    fpIsFlying = false;
    fpFlyingStreak = 0;
    if (fpPulseActive) fpStopPulseRestore();
  }

  saveMainConfig();

  String html = htmlHeader("Saved");
  html += "<div class='card'><h2>Flight Pulse Saved</h2><a class='btn btn2' href='/'>Back</a></div>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleAdmin() {
  String html = htmlHeader("Admin");

  html += "<div class='card'><h1>Admin</h1>";
  html += "<a class='btn btn2' href='/'>Back</a>";
  html += "<a class='btn btn3' href='/refresh'>Refresh Now</a>";
  html += "<div class='muted'>Remote troubleshooting view with cellular diagnostics and a read-only chronological event log.</div>";
  html += "</div>";

  html += getAdminCellularDetailsHtml();
  html += getEventLogHtml(false);

  html += "<div class='card'><h2>LED Settings</h2>";
  html += "<form method='POST' action='/saveAdmin'>";

  html += "<div class='row3'>";
  html += "<div><label>LED Pin</label><input type='number' name='ledPin' value='" + String(ledPin) + "'></div>";
  html += "<div><label>LED Count</label><input type='number' min='1' max='300' name='numLeds' value='" + String(numLeds) + "'></div>";
  html += "<div><label>LED Order</label><select name='ledOrder'>";
  html += "<option value='0'" + String(ledOrder == 0 ? " selected" : "") + ">GRB</option>";
  html += "<option value='1'" + String(ledOrder == 1 ? " selected" : "") + ">RGB</option>";
  html += "<option value='2'" + String(ledOrder == 2 ? " selected" : "") + ">BRG</option>";
  html += "</select></div>";
  html += "</div>";

  html += "<label>New AVWX API Key</label>";
  html += "<input type='text' name='apiKey' value='' placeholder='Leave blank to keep current key'>";

  html += "<label>OTA</label><select name='otaEnabled'>";
  html += "<option value='1'" + String(otaEnabled ? " selected" : "") + ">Enabled</option>";
  html += "<option value='0'" + String(!otaEnabled ? " selected" : "") + ">Disabled</option>";
  html += "</select>";

  html += "<div style='margin-top:16px;'><button class='btn' type='submit'>Save Admin & Reboot</button></div>";
  html += "</form></div>";

  html += "<div class='card'><h2>LED Test</h2>";
  html += "<a class='btn btn3' href='/test?c=green'>Green</a>";
  html += "<a class='btn' href='/test?c=blue'>Blue</a>";
  html += "<a class='btn btn4' href='/test?c=red'>Red</a>";
  html += "<a class='btn btn2' href='/test?c=magenta'>Magenta</a>";
  html += "<a class='btn btn2' href='/test?c=white'>White</a>";
  html += "<a class='btn btn2' href='/test?c=yellow'>Yellow</a>";
  html += "<a class='btn btn4' href='/test?c=off'>Off</a>";
  html += "</div>";

  html += "<div class='card'><h2>OTA</h2>";
  if (connectionMode == "cellular") {
    html += "<div class='warn'>OTA requires WiFi. Current mode is Cellular Only.</div>";
  } else if (WiFi.status() == WL_CONNECTED) {
    html += "<div class='good'>WiFi connected. OTA available.</div>";
    html += "<div class='muted'>Current version: " + String(FW_VERSION) + "</div>";
    html += "<a class='btn' href='/otaCheck'>Check for Update</a>";
    html += "<a class='btn btn3' href='/otaInstall'>Install Update</a>";
  } else {
    html += "<div class='warn'>OTA disabled until WiFi is connected.</div>";
  }
  html += "</div>";

  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleSaveAdmin() {
  if (server.hasArg("ledPin")) ledPin = server.arg("ledPin").toInt();
  if (server.hasArg("numLeds")) numLeds = server.arg("numLeds").toInt();
  if (server.hasArg("ledOrder")) ledOrder = server.arg("ledOrder").toInt();
  if (server.hasArg("otaEnabled")) otaEnabled = (server.arg("otaEnabled").toInt() == 1);

  if (server.hasArg("apiKey")) {
    String newKey = server.arg("apiKey");
    newKey.trim();
    if (newKey.length() > 0) avwxApiKey = newKey;
  }

  if (numLeds < 1) numLeds = 1;
  if (numLeds > 300) numLeds = 300;
  if (ledOrder < 0 || ledOrder > 2) ledOrder = 0;

  saveAdminConfig();

  String html = htmlHeader("Saved");
  html += "<div class='card'><h2>Admin Settings Saved</h2><p>Rebooting now.</p></div>";
  html += htmlFooter();
  server.send(200, "text/html", html);

  delay(1500);
  ESP.restart();
}

void handleAdminTest() {
  if (!admintestAuthorized()) {
    String html = htmlHeader("Admin Test Login");
    html += "<div class='card'><h1>Admin Test</h1><form method='GET' action='/admintest'><label>Password</label><input type='password' name='p'><button class='btn' type='submit'>Unlock</button><a class='btn btn2' href='/'>Back</a></form></div>";
    html += htmlFooter();
    server.send(200, "text/html", html);
    return;
  }
  double avgEstimated = cellMeterRequestCount ? ((double)cellMeterEstimatedBytes / (double)cellMeterRequestCount) : 0.0;
  double projectedDay = avgEstimated * (86400.0 / (double)cellDataTestIntervalSec);
  double projectedMonth = projectedDay * 30.0;
  String html = htmlHeader("Admin Data Test");
  html += "<div class='card'><h1>🔒 Admin Cellular Data Test</h1><a class='btn btn2' href='/'>Back</a><a class='btn btn2' href='/admin'>Admin</a></div>";
  html += "<div class='card'><h2>Status</h2>" + getStatusBlock() + "<div class='divider'></div>";
  html += "<div class='muted'>Admin test: " + dataTestStatusLine() + "</div>";
  html += "<div class='muted'>STA WiFi: " + String(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "not connected") + "</div>";
  html += "<div class='muted'>SoftAP: " + WiFi.softAPIP().toString() + "</div></div>";
  html += getAdminCellularDetailsHtml();
  html += getEventLogHtml(true);
  html += "<div class='card'><h2>Test Controls</h2><form method='POST' action='/saveAdminTest'>" + admintestAuthHidden();
  html += "<label>Admin Data Test Mode</label><select name='enabled'><option value='0'" + String(!cellDataTestEnabled ? " selected" : "") + ">Off</option><option value='1'" + String(cellDataTestEnabled ? " selected" : "") + ">On</option></select>";
  html += "<label>Test Fetch Path</label><select name='path'><option value='cellular'" + String(cellDataTestPath == "cellular" ? " selected" : "") + ">Cellular only - keep WiFi web UI connected</option><option value='wifi'" + String(cellDataTestPath == "wifi" ? " selected" : "") + ">WiFi only</option><option value='auto'" + String(cellDataTestPath == "auto" ? " selected" : "") + ">Normal auto/device logic</option></select>";
  html += "<div class='row'><div><label>Test interval seconds</label><input type='number' min='30' max='3600' name='interval' value='" + String(cellDataTestIntervalSec) + "'></div><div><label>Estimate multiplier</label><input type='number' min='1' max='10' name='mult' value='" + String(cellDataOverheadMultiplier) + "'></div></div>";
  html += "<div class='muted'>Minimum interval is 30 seconds. Active mode does not change WiFi management access.</div><button class='btn' type='submit'>Save Test Settings</button></form>";
  html += "<form method='POST' action='/admintestFetchNow' style='display:inline-block'>" + admintestAuthHidden() + "<button class='btn btn3' type='submit'>Fetch Now Using Selected Path</button></form>";
  html += "<form method='POST' action='/admintestReset' style='display:inline-block'>" + admintestAuthHidden() + "<button class='btn btn4' type='submit'>Reset Counters</button></form></div>";
  html += "<div class='card'><h2>Cellular Data Estimate</h2>";
  html += "<div class='muted'>Requests " + String(cellMeterRequestCount) + " | OK " + String(cellMeterSuccessCount) + " | Fail " + String(cellMeterFailCount) + "</div>";
  html += "<div class='muted'>Last: " + htmlEscape(cellMeterLastTag) + " status " + String(cellMeterLastStatus) + " body " + formatBytes(cellMeterLastBodyBytes) + " header " + formatBytes(cellMeterLastHeaderBytes) + " est " + formatBytes(cellMeterLastEstimatedBytes) + "</div>";
  html += "<div class='muted'>Totals estimated: " + formatBytes(cellMeterEstimatedBytes) + " | avg " + formatBytes((uint64_t)avgEstimated) + " | projected " + formatBytes((uint64_t)projectedDay) + "/day, " + formatBytes((uint64_t)projectedMonth) + "/30 days</div>";
  html += "<div class='muted'>Estimate only; not carrier-billed bytes.</div></div>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleSaveAdminTest() {
  if (!admintestAuthorized()) { server.send(403, "text/plain", "Forbidden"); return; }
  cellDataTestEnabled = server.hasArg("enabled") && server.arg("enabled").toInt() == 1;
  if (server.hasArg("path")) cellDataTestPath = server.arg("path");
  cellDataTestPath.trim(); cellDataTestPath.toLowerCase();
  if (cellDataTestPath != "cellular" && cellDataTestPath != "wifi" && cellDataTestPath != "auto") cellDataTestPath = "cellular";
  if (server.hasArg("interval")) cellDataTestIntervalSec = server.arg("interval").toInt();
  if (cellDataTestIntervalSec < 30) cellDataTestIntervalSec = 30; if (cellDataTestIntervalSec > 3600) cellDataTestIntervalSec = 3600;
  if (server.hasArg("mult")) cellDataOverheadMultiplier = server.arg("mult").toInt();
  if (cellDataOverheadMultiplier < 1) cellDataOverheadMultiplier = 1; if (cellDataOverheadMultiplier > 10) cellDataOverheadMultiplier = 10;
  preferences.begin("metar", false); preferences.putString("testPath", cellDataTestPath); preferences.putUInt("testSec", cellDataTestIntervalSec); preferences.putUInt("testMult", cellDataOverheadMultiplier); preferences.end();
  previousMillis = millis(); lastStatusLine = cellDataTestEnabled ? "Admin data test mode enabled" : "Admin data test mode disabled"; logEvent("ADMIN", String("Data test ") + (cellDataTestEnabled ? "ON" : "OFF") + " | path " + cellDataTestPath + " | interval " + String(cellDataTestIntervalSec));
  String html = htmlHeader("Admin Test Saved"); html += "<div class='card'><h2>Admin Test Settings Saved</h2><div class='muted'>" + dataTestStatusLine() + "</div><a class='btn btn2' href='/admintest?p=" + String(ADMIN_TEST_PASSWORD) + "'>Back</a></div>" + htmlFooter(); server.send(200, "text/html", html);
}

void handleAdminTestResetCounters() { if (!admintestAuthorized()) { server.send(403, "text/plain", "Forbidden"); return; } resetCellDataCounters(); String html = htmlHeader("Counters Reset"); html += "<div class='card'><h2>Cellular Data Counters Reset</h2><a class='btn btn2' href='/admintest?p=" + String(ADMIN_TEST_PASSWORD) + "'>Back</a></div>" + htmlFooter(); server.send(200, "text/html", html); }

void handleAdminTestClearLog() { if (!admintestAuthorized()) { server.send(403, "text/plain", "Forbidden"); return; } clearEventLog(); logEvent("ADMIN", "Event log cleared"); String html = htmlHeader("Event Log Cleared"); html += "<div class='card'><h2>Event Log Cleared</h2><a class='btn btn2' href='/admintest?p=" + String(ADMIN_TEST_PASSWORD) + "'>Back</a></div>" + htmlFooter(); server.send(200, "text/html", html); }

void handleAdminTestFetchNow() { if (!admintestAuthorized()) { server.send(403, "text/plain", "Forbidden"); return; } logEvent("ADMIN", "Manual admin test fetch requested using path " + cellDataTestPath); applyAdminTestFetchOverride(); fetchMETAR(); previousMillis = millis(); String html = htmlHeader("Fetch Complete"); html += "<div class='card'><h2>Admin Test Fetch Complete</h2><div class='muted'>Last request: " + htmlEscape(lastFetchTransport) + " | status " + String(lastHttpStatus) + " | " + String(lastFetchDurationMs) + " ms | " + htmlEscape(lastFetchDetail) + "</div><a class='btn btn2' href='/admintest?p=" + String(ADMIN_TEST_PASSWORD) + "'>Back</a></div>" + htmlFooter(); server.send(200, "text/html", html); }

void handleTest() {
  String c = server.arg("c");
  applyAdminTestColor(c);

  String html = htmlHeader("LED Test");
  html += "<div class='card'><h2>LED Test Applied</h2><div class='muted'>Test color will hold for about 15 seconds.</div><a class='btn btn2' href='/admin'>Back</a></div>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleRefresh() {
  fetchMETAR();
  String html = htmlHeader("Refresh");
  html += "<div class='card'><h2>Refresh Requested</h2><a class='btn btn2' href='/'>Back</a></div>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleOtaCheck() {
  String html = htmlHeader("OTA Check");
  html += "<div class='card'><h2>OTA Check</h2>";

  if (connectionMode == "cellular") {
    html += "<div class='warn'>OTA requires WiFi. Current mode is Cellular Only.</div>";
  } else if (WiFi.status() != WL_CONNECTED) {
    html += "<div class='warn'>WiFi not connected.</div>";
  } else if (!otaEnabled) {
    html += "<div class='warn'>OTA is disabled in admin.</div>";
  } else {
    String latestVersion, latestUrl;
    int latestSize = 0;
    if (otaGetLatest(latestVersion, latestUrl, latestSize)) {
      html += "<div class='good'>Latest version: " + latestVersion + "</div>";
      html += "<div class='muted'>Current version: " + String(FW_VERSION) + "</div>";
      html += "<div class='muted'>Size: " + String(latestSize) + " bytes</div>";
      html += "<div class='muted mono'>URL: " + latestUrl + "</div>";
    } else {
      html += "<div class='bad'>Failed to fetch OTA metadata.</div>";
    }
  }

  html += "<a class='btn btn2' href='/admin'>Back</a>";
  html += "</div>" + htmlFooter();
  server.send(200, "text/html", html);
}

void handleOtaInstall() {
  String html = htmlHeader("OTA Install");
  html += "<div class='card'><h2>OTA Install</h2>";

  if (connectionMode == "cellular") {
    html += "<div class='warn'>OTA requires WiFi. Current mode is Cellular Only.</div>";
    html += "<a class='btn btn2' href='/admin'>Back</a>";
    html += "</div>" + htmlFooter();
    server.send(200, "text/html", html);
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    html += "<div class='warn'>WiFi not connected.</div>";
    html += "<a class='btn btn2' href='/admin'>Back</a>";
    html += "</div>" + htmlFooter();
    server.send(200, "text/html", html);
    return;
  }

  if (!otaEnabled) {
    html += "<div class='warn'>OTA disabled in admin.</div>";
    html += "<a class='btn btn2' href='/admin'>Back</a>";
    html += "</div>" + htmlFooter();
    server.send(200, "text/html", html);
    return;
  }

  html += "<div class='muted'>Starting OTA. Device may reboot if successful.</div>";
  html += "</div>" + htmlFooter();
  server.send(200, "text/html", html);

  delay(500);
  otaInstallNow();
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("METARLightworks_GSM");
  Serial.println("FW Version: " FW_VERSION);
  Serial.println("========================================");

  SerialAT.begin(MODEM_BAUDRATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  loadConfig();
  setupStrip();
  resetCellDataCounters();
  logEvent("BOOT", String("Boot FW ") + FW_VERSION + " airport " + airportCode + " APN " + apn);

  Serial.println("[CFG] Airport: " + airportCode);
  Serial.println("[CFG] APN: " + apn);
  Serial.println("[CFG] Connection mode: " + connectionModeLabel());
  Serial.println("[CFG] WiFi saved: " + String(wifiSsid.length() ? "yes" : "no"));
  Serial.println("[CFG] OTA enabled: " + String(otaEnabled ? "yes" : "no"));

  setBootBlink(true);
  setNoSignalState(false);
  updateBootBlink();

  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(500);

  WiFi.mode(WIFI_AP_STA);
  delay(200);

  bool apOk = WiFi.softAP("METARLightworks_GSM", "metar123", 1, 0, 4);
  delay(500);

  Serial.print("[WiFi] softAP result: ");
  Serial.println(apOk ? "OK" : "FAILED");
  Serial.print("[WiFi] AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("[WiFi] AP MAC: ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.println("[WiFi] AP started: METARLightworks_GSM");
  logEvent("WIFI", "SoftAP started METARLightworks_GSM at " + WiFi.softAPIP().toString());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/admin", HTTP_GET, handleAdmin);
  server.on("/saveAdmin", HTTP_POST, handleSaveAdmin);
  server.on("/admintest", HTTP_GET, handleAdminTest);
  server.on("/saveAdminTest", HTTP_POST, handleSaveAdminTest);
  server.on("/admintestReset", HTTP_POST, handleAdminTestResetCounters);
  server.on("/admintestClearLog", HTTP_POST, handleAdminTestClearLog);
  server.on("/admintestFetchNow", HTTP_POST, handleAdminTestFetchNow);
  server.on("/refresh", HTTP_GET, handleRefresh);
  server.on("/forgetWifi", HTTP_POST, handleForgetWifi);
  server.on("/test", HTTP_GET, handleTest);
  server.on("/otaCheck", HTTP_GET, handleOtaCheck);
  server.on("/otaInstall", HTTP_GET, handleOtaInstall);
  server.on("/flightpulse", HTTP_POST, handleFlightPulseSave);
  server.onNotFound([](){ server.send(404, "text/plain", "Not Found"); });
  server.begin();

  delay(1500);

  if (!wifiModeAllowed()) {
    Serial.println("[WiFi] Cellular-only mode selected; skipping station WiFi");
    useWiFi = false;
    currentFetchInterval = CELL_FETCH_INTERVAL_MS;
    lastStatusLine = "Cellular-only mode";
  } else if (wifiSsid.length() > 0) {
    Serial.print("[WiFi] Connecting to ");
    Serial.println(wifiSsid);

    WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());

    int wifi_timeout = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_timeout < 30) {
      delay(1000);
      Serial.print(".");
      wifi_timeout++;
      server.handleClient();
      updateBootBlink();
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.print("[WiFi] Connected IP: ");
      Serial.println(WiFi.localIP());
      useWiFi = true;
      currentFetchInterval = WIFI_FETCH_INTERVAL_MS;
      configTime(timezone * 3600, 0, "pool.ntp.org", "time.nist.gov");
      lastStatusLine = "WiFi connected";
      logEvent("WIFI", "Boot STA connected " + WiFi.localIP().toString());
    } else {
      Serial.println();
      useWiFi = false;
      if (cellularModeAllowed()) {
        Serial.println("[WiFi] Failed, using cellular fallback");
        currentFetchInterval = CELL_FETCH_INTERVAL_MS;
        lastStatusLine = "WiFi failed, using cellular fallback";
        logEvent("WIFI", "Boot STA connect failed; using cellular fallback");
      } else {
        Serial.println("[WiFi] Failed, WiFi-only mode");
        currentFetchInterval = WIFI_FETCH_INTERVAL_MS;
        lastStatusLine = "WiFi failed; WiFi-only mode";
      }
    }
  } else {
    Serial.println("[WiFi] No WiFi credentials stored");
    useWiFi = false;
    if (cellularModeAllowed()) {
      currentFetchInterval = CELL_FETCH_INTERVAL_MS;
      lastStatusLine = "No WiFi creds, using cellular";
    } else {
      currentFetchInterval = WIFI_FETCH_INTERVAL_MS;
      lastStatusLine = "No WiFi creds; WiFi-only mode";
      setNoSignalState(true);
    }
  }

  if (!useWiFi && cellularModeAllowed()) {
    Serial.println("[CELL] Boot cellular connect: normal attach first; operator rescue only if needed");
    if (!connectCellularData(false, "boot")) {
      Serial.println("[CELL] Normal boot cellular connect failed; trying operator rescue");
      if (!connectCellularData(true, "boot operator rescue")) {
        Serial.println("[CELL] Cellular connect failed at boot");
        setNoSignalState(true);
      }
    }
  }

  ensureTime(timezone);

  fetchMETAR();
  previousMillis = millis();
}

void loop() {
  server.handleClient();
  updateAdminTestState();

  if (bootBlinkActive) updateBootBlink();

  maintainWiFiConnection();
  serviceCellularRecovery();

  if (!adminTestActive) updateLEDSchedule();

  if (!(cellDataTestEnabled && cellDataTestPath == "cellular")) {
    updateFlightPulse();
  } else if (fpPulseActive) {
    fpStopPulseRestore();
  }

  unsigned long currentMillis = millis();
  unsigned long intervalMs = effectiveFetchIntervalMs();
  if (currentMillis - previousMillis >= intervalMs) {
    previousMillis = currentMillis;
    applyAdminTestFetchOverride();
    fetchMETAR();
  }
}