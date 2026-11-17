#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_timer.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static const uint8_t PIN_SDA = 21;
static const uint8_t PIN_SCL = 22;
static const uint8_t PIN_RELAY = 25;
static const uint8_t PIN_BOOT = 0;

static const uint8_t SHT31_ADDR_A = 0x44;
static const uint8_t SHT31_ADDR_B = 0x45;

static const uint32_t CONFIG_MAGIC = 0x53485433UL;
static const uint16_t CONFIG_VERSION = 1;
static const char CONFIG_NS[] = "cfg";
static const char DEFAULT_SSID[] = "ESP32_SHT31";
static const char DEFAULT_PASS[] = "12345678";

static const uint8_t MAX_AP_CLIENTS = 4;
static const uint64_t FACTORY_HOLD_US = 5000000ULL;
static const uint64_t SHT31_MEASUREMENT_WAIT_US = 16000ULL;
static const uint64_t SENSOR_RETRY_US = 5000000ULL;
static const uint64_t WIFI_APPLY_DELAY_US = 1500000ULL;
static const uint64_t RESTART_DELAY_US = 750000ULL;
static const uint64_t FACTORY_RESET_DELAY_US = 1000000ULL;

struct Config {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint64_t seq;
  int64_t tempOffsetCenti;
  int64_t humOffsetCenti;
  int64_t relayThresholdCenti;
  int64_t relayHysteresisCenti;
  uint64_t relayMinSwitchMs;
  uint64_t sampleIntervalMs;
  char ssid[33];
  char password[65];
  uint8_t channel;
  uint8_t hidden;
  uint8_t relayEnabled;
  uint8_t relaySource;
  uint8_t relayComparator;
  uint8_t relayActiveLow;
  uint8_t relayManual;
  uint8_t relayFailSafe;
  uint8_t reserved[2];
  uint32_t crc32;
};

static Config gConfig;
static uint8_t gActiveSlot = 0;
static uint64_t gConfigSeq = 0;

static WebServer server(80);
static DNSServer dnsServer;

static int64_t tempCenti = 0;
static int64_t humCenti = 0;
static uint8_t sensorAddr = SHT31_ADDR_A;
static bool sensorOk = false;
static uint8_t sensorFailCount = 0;
static uint64_t sensorErrorCount = 0;
static uint64_t sensorReadCount = 0;
static uint64_t lastSensorReadUs = 0;
static uint64_t lastValidSensorUs = 0;
static uint64_t nextSensorTriggerUs = 0;
static uint64_t sensorMeasureStartUs = 0;

enum SensorState {
  SENSOR_IDLE = 0,
  SENSOR_WAIT = 1
};

static SensorState sensorState = SENSOR_IDLE;

static bool relayOn = false;
static uint8_t relayPhysical = 255;
static uint64_t lastRelaySwitchUs = 0;

static bool bootButtonPressed = false;
static bool bootResetTriggered = false;
static uint64_t bootButtonStartUs = 0;

static bool wifiApplyPending = false;
static bool restartPending = false;
static bool factoryResetPending = false;
static uint64_t wifiApplyAtUs = 0;
static uint64_t restartAtUs = 0;
static uint64_t factoryResetAtUs = 0;

static inline uint64_t nowUs() {
  int64_t v = esp_timer_get_time();
  return v > 0 ? (uint64_t)v : 0ULL;
}

static size_t boundedStrLen(const char* s, size_t maxLen) {
  size_t n = 0;
  while (n < maxLen && s[n] != 0) {
    ++n;
  }
  return n;
}

static void copyStr(char* dst, size_t dstSize, const char* src) {
  if (!dst || dstSize == 0) {
    return;
  }
  size_t i = 0;
  while (i + 1 < dstSize && src[i] != 0) {
    dst[i] = src[i];
    ++i;
  }
  dst[i] = 0;
}

static int clampValue(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static uint64_t clampValue(uint64_t v, uint64_t lo, uint64_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static int64_t clampValue(int64_t v, int64_t lo, int64_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static uint32_t crc32Calc(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      uint32_t mask = (uint32_t)-(int32_t)(crc & 1UL);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

static uint8_t sht31Crc(const uint8_t* data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x31);
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}

static void jsonEscape(char* dst, size_t dstSize, const char* src) {
  if (!dst || dstSize == 0) {
    return;
  }
  size_t j = 0;
  for (size_t i = 0; src[i] != 0 && j + 1 < dstSize; ++i) {
    unsigned char c = (unsigned char)src[i];
    if (c == '"' || c == '\\') {
      if (j + 2 >= dstSize) {
        break;
      }
      dst[j++] = '\\';
      dst[j++] = (char)c;
    } else if (c == '\n') {
      if (j + 2 >= dstSize) {
        break;
      }
      dst[j++] = '\\';
      dst[j++] = 'n';
    } else if (c == '\r') {
      if (j + 2 >= dstSize) {
        break;
      }
      dst[j++] = '\\';
      dst[j++] = 'r';
    } else if (c == '\t') {
      if (j + 2 >= dstSize) {
        break;
      }
      dst[j++] = '\\';
      dst[j++] = 't';
    } else if (c < 0x20) {
      dst[j++] = '?';
    } else {
      dst[j++] = (char)c;
    }
  }
  dst[j] = 0;
}

static void setDefaultConfig(Config& c) {
  memset(&c, 0, sizeof(c));
  c.magic = CONFIG_MAGIC;
  c.version = CONFIG_VERSION;
  c.size = sizeof(Config);
  c.seq = 0;
  c.tempOffsetCenti = 0;
  c.humOffsetCenti = 0;
  c.relayThresholdCenti = 3000;
  c.relayHysteresisCenti = 100;
  c.relayMinSwitchMs = 1000ULL;
  c.sampleIntervalMs = 2000ULL;
  copyStr(c.ssid, sizeof(c.ssid), DEFAULT_SSID);
  copyStr(c.password, sizeof(c.password), DEFAULT_PASS);
  c.channel = 1;
  c.hidden = 0;
  c.relayEnabled = 0;
  c.relaySource = 0;
  c.relayComparator = 0;
  c.relayActiveLow = 0;
  c.relayManual = 0;
  c.relayFailSafe = 0;
  c.reserved[0] = 0;
  c.reserved[1] = 0;
  c.crc32 = crc32Calc((const uint8_t*)&c, offsetof(Config, crc32));
}

static bool validateAndSanitizeConfig(Config& c) {
  if (c.magic != CONFIG_MAGIC || c.version != CONFIG_VERSION || c.size != sizeof(Config)) {
    return false;
  }

  size_t ssidLen = boundedStrLen(c.ssid, sizeof(c.ssid));
  if (ssidLen == 0 || ssidLen > 32) {
    return false;
  }

  size_t passLen = boundedStrLen(c.password, sizeof(c.password));
  if (passLen < 8 || passLen > 63) {
    return false;
  }

  for (size_t i = 0; i < ssidLen; ++i) {
    uint8_t ch = (uint8_t)c.ssid[i];
    if (ch < 0x20 || ch > 0x7E) {
      return false;
    }
  }

  for (size_t i = 0; i < passLen; ++i) {
    uint8_t ch = (uint8_t)c.password[i];
    if (ch < 0x20 || ch > 0x7E) {
      return false;
    }
  }

  c.channel = (uint8_t)clampValue((int)c.channel, 1, 13);
  c.hidden = c.hidden ? 1 : 0;
  c.relayEnabled = c.relayEnabled ? 1 : 0;
  c.relaySource = c.relaySource > 1 ? 0 : c.relaySource;
  c.relayComparator = c.relayComparator > 1 ? 0 : c.relayComparator;
  c.relayActiveLow = c.relayActiveLow ? 1 : 0;
  c.relayManual = c.relayManual > 2 ? 0 : c.relayManual;
  c.relayFailSafe = c.relayFailSafe ? 1 : 0;

  c.relayMinSwitchMs = clampValue(c.relayMinSwitchMs, (uint64_t)0, (uint64_t)600000);
  c.sampleIntervalMs = clampValue(c.sampleIntervalMs, (uint64_t)1000, (uint64_t)600000);
  c.tempOffsetCenti = clampValue(c.tempOffsetCenti, (int64_t)-1000, (int64_t)1000);
  c.humOffsetCenti = clampValue(c.humOffsetCenti, (int64_t)-2000, (int64_t)2000);
  if (c.relaySource == 1) {
    c.relayThresholdCenti = clampValue(c.relayThresholdCenti, (int64_t)0, (int64_t)10000);
  } else {
    c.relayThresholdCenti = clampValue(c.relayThresholdCenti, (int64_t)-10000, (int64_t)20000);
  }
  c.relayHysteresisCenti = clampValue(c.relayHysteresisCenti, (int64_t)0, (int64_t)5000);
  c.reserved[0] = 0;
  c.reserved[1] = 0;
  return true;
}

static bool configFieldsEqual(const Config& a, const Config& b) {
  Config x = a;
  Config y = b;
  x.seq = 0;
  y.seq = 0;
  x.crc32 = 0;
  y.crc32 = 0;
  return memcmp(&x, &y, sizeof(Config)) == 0;
}

static bool readConfigSlot(const char* key, Config& out) {
  Preferences p;
  if (!p.begin(CONFIG_NS, true)) {
    return false;
  }
  size_t len = p.getBytesLength(key);
  if (len != sizeof(Config)) {
    p.end();
    return false;
  }
  size_t got = p.getBytes(key, &out, sizeof(out));
  p.end();
  if (got != sizeof(out)) {
    return false;
  }
  if (out.magic != CONFIG_MAGIC || out.version != CONFIG_VERSION || out.size != sizeof(Config)) {
    return false;
  }
  uint32_t stored = out.crc32;
  uint32_t calc = crc32Calc((const uint8_t*)&out, offsetof(Config, crc32));
  if (stored != calc) {
    return false;
  }
  return validateAndSanitizeConfig(out);
}

static void loadConfig() {
  Config a;
  Config b;
  bool va = readConfigSlot("slotA", a);
  bool vb = readConfigSlot("slotB", b);

  if (va && vb) {
    if (a.seq >= b.seq) {
      gConfig = a;
      gActiveSlot = 1;
      gConfigSeq = a.seq;
    } else {
      gConfig = b;
      gActiveSlot = 2;
      gConfigSeq = b.seq;
    }
  } else if (va) {
    gConfig = a;
    gActiveSlot = 1;
    gConfigSeq = a.seq;
  } else if (vb) {
    gConfig = b;
    gActiveSlot = 2;
    gConfigSeq = b.seq;
  } else {
    setDefaultConfig(gConfig);
    gActiveSlot = 0;
    gConfigSeq = gConfig.seq;
  }
}

static bool saveConfigIfChanged(Config newCfg, bool& changed) {
  changed = false;
  if (!validateAndSanitizeConfig(newCfg)) {
    return false;
  }
  if (configFieldsEqual(newCfg, gConfig)) {
    return true;
  }

  Config c = newCfg;
  c.magic = CONFIG_MAGIC;
  c.version = CONFIG_VERSION;
  c.size = sizeof(Config);
  c.seq = (gConfigSeq == UINT64_MAX) ? 1ULL : (gConfigSeq + 1ULL);
  c.reserved[0] = 0;
  c.reserved[1] = 0;
  c.crc32 = 0;
  c.crc32 = crc32Calc((const uint8_t*)&c, offsetof(Config, crc32));

  uint8_t target = (gActiveSlot == 1) ? 2 : 1;
  const char* key = (target == 1) ? "slotA" : "slotB";

  Preferences p;
  if (!p.begin(CONFIG_NS, false)) {
    return false;
  }
  size_t written = p.putBytes(key, &c, sizeof(c));
  p.end();

  if (written != sizeof(c)) {
    return false;
  }

  gConfig = c;
  gActiveSlot = target;
  gConfigSeq = c.seq;
  changed = true;
  return true;
}

static bool sensorDataFresh(uint64_t now) {
  if (lastValidSensorUs == 0) {
    return false;
  }
  uint64_t staleMs = gConfig.sampleIntervalMs * 3ULL;
  if (staleMs < 10000ULL) {
    staleMs = 10000ULL;
  }
  return (now >= lastValidSensorUs) && ((now - lastValidSensorUs) <= (staleMs * 1000ULL));
}

static void sensorReadFailed(uint64_t now) {
  sensorOk = false;
  sensorFailCount++;
  sensorErrorCount++;
  if (sensorFailCount >= 3) {
    sensorAddr = (sensorAddr == SHT31_ADDR_A) ? SHT31_ADDR_B : SHT31_ADDR_A;
    sensorFailCount = 0;
  }
  uint64_t retry = gConfig.sampleIntervalMs * 1000ULL;
  if (retry < SENSOR_RETRY_US) {
    retry = SENSOR_RETRY_US;
  }
  nextSensorTriggerUs = now + retry;
}

static void triggerSensor(uint64_t now) {
  Wire.beginTransmission(sensorAddr);
  Wire.write((uint8_t)0x24);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() != 0) {
    sensorState = SENSOR_IDLE;
    sensorReadFailed(now);
    return;
  }
  sensorMeasureStartUs = nowUs();
  sensorState = SENSOR_WAIT;
}

static void updateSensor(uint64_t now) {
  if (sensorState == SENSOR_IDLE) {
    if (now >= nextSensorTriggerUs) {
      triggerSensor(now);
    }
    return;
  }

  if (sensorState != SENSOR_WAIT) {
    sensorState = SENSOR_IDLE;
    return;
  }

  if (now < sensorMeasureStartUs) {
    return;
  }

  uint64_t elapsed = now - sensorMeasureStartUs;
  if (elapsed < SHT31_MEASUREMENT_WAIT_US) {
    return;
  }

  bool success = false;

  if (Wire.requestFrom((int)sensorAddr, 6) == 6) {
    uint8_t b[6];
    for (uint8_t i = 0; i < 6; ++i) {
      b[i] = (uint8_t)Wire.read();
    }

    if (sht31Crc(b, 2) == b[2] && sht31Crc(&b[3], 2) == b[5]) {
      uint16_t traw = ((uint16_t)b[0] << 8) | b[1];
      uint16_t hraw = ((uint16_t)b[3] << 8) | b[4];

      int64_t t = -4500LL + (int64_t)((17500ULL * (uint64_t)traw) / 65535ULL);
      int64_t h = (int64_t)((10000ULL * (uint64_t)hraw) / 65535ULL);

      t += gConfig.tempOffsetCenti;
      h += gConfig.humOffsetCenti;

      if (h < 0) {
        h = 0;
      }
      if (h > 10000) {
        h = 10000;
      }

      tempCenti = t;
      humCenti = h;
      lastSensorReadUs = now;
      lastValidSensorUs = now;
      sensorOk = true;
      sensorFailCount = 0;
      sensorReadCount++;
      success = true;
    }
  }

  sensorState = SENSOR_IDLE;

  if (success) {
    nextSensorTriggerUs = now + (gConfig.sampleIntervalMs * 1000ULL);
  } else {
    sensorReadFailed(now);
  }
}

static void updateRelay(bool force, uint64_t now) {
  bool fresh = sensorDataFresh(now);
  bool desired = relayOn;

  if (!gConfig.relayEnabled) {
    desired = false;
  } else if (gConfig.relayManual == 1) {
    desired = false;
  } else if (gConfig.relayManual == 2) {
    desired = true;
  } else if (!fresh) {
    desired = (gConfig.relayFailSafe != 0);
  } else {
    int64_t value = (gConfig.relaySource == 0) ? tempCenti : humCenti;
    int64_t threshold = gConfig.relayThresholdCenti;
    int64_t hyst = gConfig.relayHysteresisCenti;

    if (gConfig.relayComparator == 0) {
      if (!relayOn) {
        desired = (value >= threshold);
      } else {
        desired = (value >= (threshold - hyst));
      }
    } else {
      if (!relayOn) {
        desired = (value <= threshold);
      } else {
        desired = (value <= (threshold + hyst));
      }
    }
  }

  bool hard = force || !gConfig.relayEnabled || gConfig.relayManual != 0 || !fresh;

  if (desired != relayOn) {
    bool allow = hard || lastRelaySwitchUs == 0;
    if (!allow) {
      uint64_t minUs = gConfig.relayMinSwitchMs * 1000ULL;
      allow = (now - lastRelaySwitchUs) >= minUs;
    }
    if (allow) {
      relayOn = desired;
      lastRelaySwitchUs = now;
    }
  }

  bool high = gConfig.relayActiveLow ? !relayOn : relayOn;
  uint8_t level = high ? HIGH : LOW;
  if (level != relayPhysical) {
    digitalWrite(PIN_RELAY, level);
    relayPhysical = level;
  }
}

static void applyAPSettings() {
  dnsServer.stop();

  IPAddress local(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.persistent(false);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(local, gateway, subnet);

  bool ok = WiFi.softAP(
    gConfig.ssid,
    gConfig.password,
    gConfig.channel,
    gConfig.hidden ? 1 : 0,
    MAX_AP_CLIENTS
  );

  if (!ok) {
    ok = WiFi.softAP(DEFAULT_SSID, DEFAULT_PASS, 1, 0, MAX_AP_CLIENTS);
    if (!ok) {
      ok = WiFi.softAP(DEFAULT_SSID, DEFAULT_PASS);
    }
    if (ok) {
      Config nc = gConfig;
      copyStr(nc.ssid, sizeof(nc.ssid), DEFAULT_SSID);
      copyStr(nc.password, sizeof(nc.password), DEFAULT_PASS);
      nc.channel = 1;
      nc.hidden = 0;
      bool changed = false;
      if (!saveConfigIfChanged(nc, changed)) {
        gConfig = nc;
      }
    }
  }

  dnsServer.start(53, "*", WiFi.softAPIP());
}

static void factoryResetNow() {
  digitalWrite(PIN_RELAY, gConfig.relayActiveLow ? HIGH : LOW);
  WiFi.softAPdisconnect(true);

  Preferences p;
  if (p.begin(CONFIG_NS, false)) {
    p.clear();
    p.end();
  }

  ESP.restart();
}

static void scheduleWifiApply() {
  wifiApplyPending = true;
  wifiApplyAtUs = nowUs() + WIFI_APPLY_DELAY_US;
}

static void scheduleRestart() {
  restartPending = true;
  restartAtUs = nowUs() + RESTART_DELAY_US;
  factoryResetPending = false;
  wifiApplyPending = false;
}

static void scheduleFactoryReset() {
  factoryResetPending = true;
  factoryResetAtUs = nowUs() + FACTORY_RESET_DELAY_US;
  restartPending = false;
  wifiApplyPending = false;
}

static void updatePendingActions(uint64_t now) {
  if (factoryResetPending) {
    if (now >= factoryResetAtUs) {
      factoryResetPending = false;
      factoryResetNow();
      return;
    }
  }

  if (restartPending) {
    if (now >= restartAtUs) {
      restartPending = false;
      ESP.restart();
      return;
    }
  }

  if (wifiApplyPending) {
    if (now >= wifiApplyAtUs) {
      wifiApplyPending = false;
      applyAPSettings();
    }
  }
}

static void updateBootButton(uint64_t now) {
  bool pressed = (digitalRead(PIN_BOOT) == LOW);

  if (pressed && !bootButtonPressed) {
    bootButtonPressed = true;
    bootResetTriggered = false;
    bootButtonStartUs = now;
  } else if (!pressed && bootButtonPressed) {
    bootButtonPressed = false;
    bootResetTriggered = false;
    bootButtonStartUs = 0;
  }

  if (pressed && bootButtonPressed && !bootResetTriggered && bootButtonStartUs != 0) {
    if ((now - bootButtonStartUs) >= FACTORY_HOLD_US) {
      bootResetTriggered = true;
      factoryResetNow();
    }
  }
}

static bool parseUnsigned(const char* s, uint64_t minV, uint64_t maxV, uint64_t& out) {
  if (!s || !*s || *s == '-') {
    return false;
  }
  char* end = nullptr;
  unsigned long long v = strtoull(s, &end, 10);
  if (end == s || *end != '\0') {
    return false;
  }
  if ((uint64_t)v < minV || (uint64_t)v > maxV) {
    return false;
  }
  out = (uint64_t)v;
  return true;
}

static bool parseDecimalCenti(const char* s, int64_t minCenti, int64_t maxCenti, int64_t& outCenti) {
  if (!s || !*s) {
    return false;
  }
  char* end = nullptr;
  double v = strtod(s, &end);
  if (end == s || *end != '\0' || !isfinite(v)) {
    return false;
  }
  double c = v * 100.0;
  if (c < (double)minCenti - 0.5 || c > (double)maxCenti + 0.5) {
    return false;
  }
  long long ci = llround(c);
  if (ci < minCenti || ci > maxCenti) {
    return false;
  }
  outCenti = (int64_t)ci;
  return true;
}

static bool parseBoolArg(const char* s, bool& out) {
  if (!s || !*s) {
    return false;
  }
  if (!strcmp(s, "1") || !strcmp(s, "true") || !strcmp(s, "on") || !strcmp(s, "yes")) {
    out = true;
    return true;
  }
  if (!strcmp(s, "0") || !strcmp(s, "false") || !strcmp(s, "off") || !strcmp(s, "no")) {
    out = false;
    return true;
  }
  return false;
}

static void sendJsonError(int code, const char* msg) {
  char buf[192];
  snprintf(buf, sizeof(buf), "{\"ok\":false,\"error\":\"%s\"}", msg);
  server.send(code, "application/json", buf);
}

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="color-scheme" content="dark light">
<title>ESP32 SHT31</title>
<style>
:root{--bg:#080d16;--panel:#111a29;--panel2:#0b1422;--text:#e7eefc;--muted:#93a4c3;--line:#263650;--accent:#4cc9f0;--accent2:#7c5cff;--good:#39d98a;--warn:#ffb020;--bad:#ff5c7a;--shadow:0 18px 50px rgba(0,0,0,.35)}
*{box-sizing:border-box}
html,body{min-height:100%}
body{margin:0;background:radial-gradient(circle at 15% 0%,rgba(76,201,240,.16),transparent 28%),radial-gradient(circle at 90% 10%,rgba(124,92,255,.14),transparent 25%),var(--bg);color:var(--text);font-family:ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif}
.app{max-width:1180px;margin:0 auto;padding:18px}
.topbar{display:flex;align-items:center;gap:14px;flex-wrap:wrap;justify-content:space-between;padding:14px 0}
.brand{font-weight:900;letter-spacing:.04em;font-size:1.12rem}
.brand span{color:var(--accent)}
.tabs{display:flex;gap:6px;padding:5px;background:rgba(17,26,41,.82);border:1px solid var(--line);border-radius:14px;backdrop-filter:blur(10px)}
.tab{border:0;border-radius:10px;padding:10px 14px;background:transparent;color:var(--muted);font-weight:800;cursor:pointer;white-space:nowrap}
.tab.active{background:linear-gradient(135deg,rgba(76,201,240,.2),rgba(124,92,255,.22));color:var(--text);box-shadow:inset 0 0 0 1px rgba(76,201,240,.2)}
.badge{padding:8px 12px;border-radius:999px;font-size:.78rem;font-weight:800;letter-spacing:.04em;border:1px solid var(--line);background:#0b1422;color:var(--muted)}
.badge.online{color:var(--good);border-color:rgba(57,217,138,.45);background:rgba(57,217,138,.09)}
.badge.offline{color:var(--bad);border-color:rgba(255,92,122,.45);background:rgba(255,92,122,.09)}
main{margin-top:18px}
.tab-panel.hidden{display:none}
.grid{display:grid;grid-template-columns:repeat(12,1fr);gap:14px}
.card{background:linear-gradient(180deg,rgba(17,26,41,.97),rgba(11,20,34,.97));border:1px solid var(--line);border-radius:18px;padding:16px;box-shadow:var(--shadow)}
.span-3{grid-column:span 3}
.span-4{grid-column:span 4}
.span-6{grid-column:span 6}
.span-12{grid-column:span 12}
.card h3{margin:0 0 10px;color:var(--muted);font-size:.76rem;text-transform:uppercase;letter-spacing:.14em}
.value{font-size:2.6rem;font-weight:900;line-height:1;margin:8px 0}
.value.on{color:var(--good)}
.value.off{color:var(--muted)}
.unit{font-size:1rem;color:var(--muted);font-weight:700}
.sub{color:var(--muted);font-size:.88rem;line-height:1.45}
.mini{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px}
.mini>div{background:rgba(255,255,255,.03);border:1px solid rgba(255,255,255,.05);border-radius:12px;padding:10px;min-width:0}
.label{font-size:.7rem;color:var(--muted);text-transform:uppercase;letter-spacing:.1em;margin-bottom:4px}
.status-strip{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:14px}
.chip{background:#0b1422;border:1px solid var(--line);border-radius:999px;padding:8px 12px;color:var(--muted);font-size:.82rem}
.chip b{color:var(--text)}
.form-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px}
label{display:flex;flex-direction:column;gap:7px;color:var(--muted);font-size:.82rem;font-weight:700}
input,select{width:100%;padding:12px;border-radius:12px;border:1px solid var(--line);background:#0b1422;color:var(--text);outline:none;font-size:1rem}
input:focus,select:focus{border-color:var(--accent);box-shadow:0 0 0 3px rgba(76,201,240,.14)}
input[type=checkbox]{width:auto;accent-color:var(--accent);transform:scale(1.2)}
.switch{flex-direction:row;align-items:center;gap:10px;background:#0b1422;border:1px solid var(--line);border-radius:12px;padding:12px;color:var(--text)}
.actions{display:flex;gap:10px;flex-wrap:wrap;margin-top:16px}
button{border:0;border-radius:12px;padding:12px 18px;background:linear-gradient(135deg,var(--accent),var(--accent2));color:#04101a;font-weight:900;cursor:pointer;transition:transform .15s,opacity .15s}
button:hover{transform:translateY(-1px)}
button:disabled{opacity:.55;cursor:wait;transform:none}
button.secondary{background:#1a2940;color:var(--text);border:1px solid var(--line)}
button.danger{background:linear-gradient(135deg,#ff5c7a,#ff8a4c);color:#1a0610}
.hint{color:var(--muted);font-size:.86rem;line-height:1.55}
.toast{position:fixed;left:50%;bottom:22px;transform:translate(-50%,20px);background:#17263d;color:var(--text);border:1px solid var(--line);padding:13px 17px;border-radius:14px;box-shadow:var(--shadow);opacity:0;pointer-events:none;transition:.25s;z-index:50;max-width:min(92vw,520px)}
.toast.show{opacity:1;transform:translate(-50%,0)}
.toast.error{border-color:rgba(255,92,122,.65)}
footer{display:flex;justify-content:space-between;gap:12px;flex-wrap:wrap;color:var(--muted);font-size:.8rem;padding:22px 4px 4px}
@media(max-width:900px){.span-3,.span-4,.span-6{grid-column:span 6}.form-grid{grid-template-columns:1fr}.mini{grid-template-columns:1fr}.value{font-size:2.3rem}}
@media(max-width:620px){.app{padding:12px}.topbar{align-items:flex-start}.tabs{width:100%;overflow:auto}.tab{flex:1;min-width:112px}.span-3,.span-4,.span-6{grid-column:span 12}.value{font-size:2.15rem}footer{flex-direction:column}}
</style>
</head>
<body>
<div class="app">
<header class="topbar">
<div class="brand">ESP32 <span>SHT31</span></div>
<nav class="tabs" aria-label="Navigation">
<button class="tab active" data-tab="dashboard" type="button">Dashboard</button>
<button class="tab" data-tab="settings" type="button">Sensor &amp; Relay</button>
<button class="tab" data-tab="wifi" type="button">Wi-Fi AP</button>
</nav>
<div id="onlineBadge" class="badge offline">Offline</div>
</header>
<main>
<section id="tab-dashboard" class="tab-panel">
<div class="status-strip">
<div class="chip">Sensor <b id="dashSensorChip">--</b></div>
<div class="chip">Relay <b id="dashRelayChip">--</b></div>
<div class="chip">AP <b id="dashSsidChip">--</b></div>
</div>
<div class="grid">
<div class="card span-3"><h3>Temperature</h3><div class="value"><span id="dashTemp">--</span><span class="unit"> °C</span></div><div class="sub" id="dashTempSub">Waiting</div></div>
<div class="card span-3"><h3>Humidity</h3><div class="value"><span id="dashHum">--</span><span class="unit"> %RH</span></div><div class="sub" id="dashHumSub">Waiting</div></div>
<div class="card span-3"><h3>Relay GPIO25</h3><div id="dashRelay" class="value off">OFF</div><div class="sub" id="dashRelaySub">Waiting</div></div>
<div class="card span-3"><h3>Sensor State</h3><div id="dashSensor" class="value off">--</div><div class="sub" id="dashSensorSub">Waiting</div></div>
<div class="card span-6"><h3>Access Point</h3><div class="mini"><div><div class="label">SSID</div><div id="dashSsid">--</div></div><div><div class="label">IP</div><div id="dashIp">--</div></div><div><div class="label">Clients</div><div id="dashClients">--</div></div><div><div class="label">Password</div><div id="dashPasswordSet">--</div></div></div></div>
<div class="card span-6"><h3>System</h3><div class="mini"><div><div class="label">Uptime</div><div id="dashUptime">--</div></div><div><div class="label">Free Heap</div><div id="dashHeap">--</div></div><div><div class="label">Minimum Heap</div><div id="dashMinHeap">--</div></div><div><div class="label">Sensor Errors</div><div id="dashErrors">--</div></div><div><div class="label">Last Valid Read</div><div id="dashLastRead">--</div></div><div><div class="label">Config Slot / Seq</div><div id="dashConfig">--</div></div></div></div>
</div>
</section>
<section id="tab-settings" class="tab-panel hidden">
<div class="card">
<h3>Live Sensor</h3>
<div class="status-strip">
<div class="chip">Temperature <b id="setCurrentTemp">--</b></div>
<div class="chip">Humidity <b id="setCurrentHum">--</b></div>
<div class="chip">Relay <b id="setCurrentRelay">--</b></div>
</div>
<form id="settingsForm" autocomplete="off">
<div class="form-grid">
<label>Sample interval (seconds)<input id="sampleIntervalSec" type="number" min="1" max="600" step="0.1" required></label>
<label>Temperature offset (°C)<input id="tempOffset" type="number" min="-10" max="10" step="0.01" required></label>
<label>Humidity offset (%RH)<input id="humOffset" type="number" min="-20" max="20" step="0.01" required></label>
<label>Relay minimum switch interval (ms)<input id="relayMinSwitchMs" type="number" min="0" max="600000" step="1" required></label>
<label>Relay source<select id="relaySource"><option value="0">Temperature</option><option value="1">Humidity</option></select></label>
<label>Trigger condition<select id="relayComparator"><option value="0">Value ≥ threshold</option><option value="1">Value ≤ threshold</option></select></label>
<label>Threshold <span id="thresholdUnit">(°C)</span><input id="relayThreshold" type="number" step="0.01" required></label>
<label>Hysteresis <span id="hysteresisUnit">(°C)</span><input id="relayHysteresis" type="number" min="0" max="50" step="0.01" required></label>
<label>Manual override<select id="relayManual"><option value="0">Automatic</option><option value="1">Force off</option><option value="2">Force on</option></select></label>
<label>Fail-safe on sensor error<select id="relayFailSafe"><option value="0">Relay off</option><option value="1">Relay on</option></select></label>
<label class="switch"><input id="relayEnabled" type="checkbox"> Relay enabled</label>
<label class="switch"><input id="relayActiveLow" type="checkbox"> Relay active low</label>
</div>
<div class="actions">
<button type="submit">Save settings</button>
<button type="button" id="reloadSettings" class="secondary">Reload current</button>
</div>
</form>
</div>
</section>
<section id="tab-wifi" class="tab-panel hidden">
<div class="card">
<h3>Current Access Point</h3>
<div class="status-strip">
<div class="chip">SSID <b id="wifiCurrentSsid">--</b></div>
<div class="chip">IP <b id="wifiCurrentIp">--</b></div>
<div class="chip">Clients <b id="wifiCurrentClients">--</b></div>
<div class="chip">Password <b id="wifiCurrentPasswordSet">--</b></div>
</div>
<form id="wifiForm" autocomplete="off">
<div class="form-grid">
<label>SSID<input id="wifiSsid" type="text" maxlength="32" required></label>
<label>Password<input id="wifiPassword" type="password" minlength="8" maxlength="63" placeholder="Leave blank to keep current"></label>
<label>Channel<input id="wifiChannel" type="number" min="1" max="13" step="1" required></label>
<label class="switch"><input id="wifiHidden" type="checkbox"> Hidden SSID</label>
</div>
<div class="actions"><button type="submit">Save Wi-Fi AP</button></div>
</form>
<p class="hint">Saving restarts the access point. If the SSID changes, reconnect to the new SSID. The captive portal redirects all HTTP requests to http://192.168.4.1/.</p>
<div class="actions">
<button type="button" id="restartBtn" class="secondary">Restart ESP32</button>
<button type="button" id="factoryBtn" class="danger">Factory reset</button>
</div>
</div>
</section>
</main>
<footer><span id="lastUpdate">Waiting for data</span><span>Captive portal: 192.168.4.1</span></footer>
</div>
<div id="toast" class="toast"></div>
<script>
const $=id=>document.getElementById(id);
const paths={dashboard:'/',settings:'/settings',wifi:'/wifi'};
let activeTab='dashboard';
let statusData=null;
let statusBusy=false;
let settingsDirty=false;
let wifiDirty=false;
let toastTimer=null;
let wasOnline=null;

function tabFromPath(path){
  if(path.startsWith('/settings'))return 'settings';
  if(path.startsWith('/wifi'))return 'wifi';
  return 'dashboard';
}

function showTab(tab,push){
  if(!paths[tab])tab='dashboard';
  activeTab=tab;
  document.querySelectorAll('.tab').forEach(b=>b.classList.toggle('active',b.dataset.tab===tab));
  document.querySelectorAll('.tab-panel').forEach(p=>p.classList.toggle('hidden',p.id!=='tab-'+tab));
  if(push&&location.pathname!==paths[tab]){
    history.pushState({tab},'',paths[tab]);
  }
  if(statusData){
    if(tab==='settings'&&!settingsDirty)fillSettings(statusData);
    if(tab==='wifi'&&!wifiDirty)fillWifi(statusData);
  }
}

function setOnline(online){
  const b=$('onlineBadge');
  b.textContent=online?'Online':'Offline';
  b.className='badge '+(online?'online':'offline');
  if(wasOnline===true&&!online)toast('Connection lost',true);
  wasOnline=online;
}

function toast(msg,isError){
  const el=$('toast');
  el.textContent=msg;
  el.className='toast show'+(isError?' error':'');
  clearTimeout(toastTimer);
  toastTimer=setTimeout(()=>{el.className='toast';},3200);
}

function formatDuration(ms){
  ms=Number(ms)||0;
  let s=Math.floor(ms/1000);
  const d=Math.floor(s/86400);s%=86400;
  const h=Math.floor(s/3600);s%=3600;
  const m=Math.floor(s/60);s%=60;
  if(d>0)return d+'d '+h+'h '+m+'m';
  if(h>0)return h+'h '+m+'m '+s+'s';
  if(m>0)return m+'m '+s+'s';
  return s+'s';
}

function formatAge(ms){
  ms=Number(ms)||0;
  if(ms<1000)return '<1s';
  if(ms<60000)return Math.floor(ms/1000)+'s';
  if(ms<3600000)return Math.floor(ms/60000)+'m';
  return Math.floor(ms/3600000)+'h';
}

function formatBytes(b){
  b=Number(b)||0;
  const kb=b/1024;
  if(kb<1024)return kb.toFixed(1)+' KB';
  return (kb/1024).toFixed(2)+' MB';
}

function sourceName(v){return Number(v)===1?'Humidity':'Temperature';}
function unitForSource(v){return Number(v)===1?'%RH':'°C';}
function comparatorName(v){return Number(v)===1?'≤':'≥';}
function manualName(v){return Number(v)===1?'Force off':Number(v)===2?'Force on':'Automatic';}

async function apiPost(path,data){
  const body=new URLSearchParams(data||{}).toString();
  const r=await fetch(path,{
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded;charset=UTF-8'},
    body,
    cache:'no-store'
  });
  let j;
  try{
    j=await r.json();
  }catch(e){
    throw new Error('Invalid response');
  }
  if(!r.ok||!j.ok)throw new Error(j.error||('HTTP '+r.status));
  return j;
}

async function refreshStatus(){
  if(statusBusy)return;
  statusBusy=true;
  try{
    const r=await fetch('/api/status',{cache:'no-store'});
    if(!r.ok)throw new Error('HTTP '+r.status);
    const d=await r.json();
    statusData=d;
    renderStatus(d);
    if(activeTab==='settings'&&!settingsDirty)fillSettings(d);
    if(activeTab==='wifi'&&!wifiDirty)fillWifi(d);
    setOnline(true);
    $('lastUpdate').textContent='Updated '+new Date().toLocaleTimeString();
  }catch(e){
    setOnline(false);
  }finally{
    statusBusy=false;
  }
}

function renderStatus(d){
  const valid=!!d.sensor.valid;
  const fresh=!!d.sensor.fresh;
  const relay=d.relay.on;
  const src=d.relay.source;
  const unit=unitForSource(src);

  $('dashTemp').textContent=valid?Number(d.sensor.tempC).toFixed(2):'--';
  $('dashHum').textContent=valid?Number(d.sensor.hum).toFixed(2):'--';
  $('dashTempSub').textContent=valid?(fresh?'Fresh reading':'Stale reading'):'No valid reading';
  $('dashHumSub').textContent=valid?(fresh?'Fresh reading':'Stale reading'):'No valid reading';

  $('dashRelay').textContent=relay?'ON':'OFF';
  $('dashRelay').className='value '+(relay?'on':'off');
  $('dashRelaySub').textContent=sourceName(src)+' '+comparatorName(d.relay.comparator)+' '+Number(d.relay.threshold).toFixed(2)+' '+unit+' · hysteresis '+Number(d.relay.hysteresis).toFixed(2)+' '+unit+' · '+manualName(d.relay.manual);

  $('dashSensor').textContent=d.sensor.ok?'OK':'ERROR';
  $('dashSensor').className='value '+(d.sensor.ok?'on':'off');
  $('dashSensorSub').textContent='Address 0x'+Number(d.sensor.addr).toString(16).toUpperCase()+' · errors '+d.sensor.errors+' · reads '+d.sensor.reads;

  $('dashSensorChip').textContent=d.sensor.ok?'OK':'ERROR';
  $('dashRelayChip').textContent=relay?'ON':'OFF';
  $('dashSsidChip').textContent=d.wifi.ssid;

  $('dashSsid').textContent=d.wifi.ssid;
  $('dashIp').textContent=d.wifi.ip;
  $('dashClients').textContent=d.wifi.clients;
  $('dashPasswordSet').textContent=d.wifi.passwordSet?'Set':'Open';

  $('dashUptime').textContent=formatDuration(d.uptimeMs);
  $('dashHeap').textContent=formatBytes(d.system.freeHeap);
  $('dashMinHeap').textContent=formatBytes(d.system.minFreeHeap);
  $('dashErrors').textContent=d.sensor.errors;
  $('dashLastRead').textContent=valid?formatAge(d.sensor.lastReadAgeMs)+' ago':'--';
  $('dashConfig').textContent='#'+d.system.configSlot+' / '+d.system.configSeq;

  $('setCurrentTemp').textContent=valid?Number(d.sensor.tempC).toFixed(2)+' °C':'--';
  $('setCurrentHum').textContent=valid?Number(d.sensor.hum).toFixed(2)+' %RH':'--';
  $('setCurrentRelay').textContent=relay?'ON':'OFF';

  $('wifiCurrentSsid').textContent=d.wifi.ssid;
  $('wifiCurrentIp').textContent=d.wifi.ip;
  $('wifiCurrentClients').textContent=d.wifi.clients;
  $('wifiCurrentPasswordSet').textContent=d.wifi.passwordSet?'Set':'Open';
}

function updateUnits(){
  const src=$('relaySource').value;
  const unit=unitForSource(src);
  $('thresholdUnit').textContent='('+unit+')';
  $('hysteresisUnit').textContent='('+unit+')';
  const t=$('relayThreshold');
  if(src==='1'){
    t.min='0';
    t.max='100';
  }else{
    t.min='-100';
    t.max='200';
  }
}

function fillSettings(d){
  const c=d.config;
  $('sampleIntervalSec').value=(Number(c.sampleIntervalMs)/1000).toFixed(1);
  $('tempOffset').value=Number(c.tempOffset).toFixed(2);
  $('humOffset').value=Number(c.humOffset).toFixed(2);
  $('relayMinSwitchMs').value=String(c.relayMinSwitchMs);
  $('relaySource').value=String(c.relaySource);
  $('relayComparator').value=String(c.relayComparator);
  $('relayThreshold').value=Number(c.relayThreshold).toFixed(2);
  $('relayHysteresis').value=Number(c.relayHysteresis).toFixed(2);
  $('relayManual').value=String(c.relayManual);
  $('relayFailSafe').value=c.relayFailSafe?'1':'0';
  $('relayEnabled').checked=!!c.relayEnabled;
  $('relayActiveLow').checked=!!c.relayActiveLow;
  updateUnits();
}

function fillWifi(d){
  if(!wifiDirty){
    $('wifiSsid').value=d.wifi.ssid;
    $('wifiPassword').value='';
    $('wifiChannel').value=String(d.wifi.channel);
    $('wifiHidden').checked=!!d.wifi.hidden;
  }
  $('wifiCurrentSsid').textContent=d.wifi.ssid;
  $('wifiCurrentIp').textContent=d.wifi.ip;
  $('wifiCurrentClients').textContent=d.wifi.clients;
  $('wifiCurrentPasswordSet').textContent=d.wifi.passwordSet?'Set':'Open';
}

document.querySelectorAll('.tab').forEach(b=>{
  b.addEventListener('click',()=>showTab(b.dataset.tab,true));
});

window.addEventListener('popstate',()=>showTab(tabFromPath(location.pathname),false));

['input','change'].forEach(ev=>{
  $('settingsForm').addEventListener(ev,()=>{settingsDirty=true;});
});

['input','change'].forEach(ev=>{
  $('wifiForm').addEventListener(ev,()=>{wifiDirty=true;});
});

$('relaySource').addEventListener('change',()=>{
  updateUnits();
  settingsDirty=true;
});

$('settingsForm').addEventListener('submit',async e=>{
  e.preventDefault();
  const form=e.target;
  if(!form.reportValidity())return;
  const btn=form.querySelector('button[type="submit"]');
  btn.disabled=true;
  try{
    const data={
      sampleIntervalMs:String(Math.round(parseFloat($('sampleIntervalSec').value)*1000)),
      tempOffset:$('tempOffset').value,
      humOffset:$('humOffset').value,
      relayEnabled:$('relayEnabled').checked?'1':'0',
      relaySource:$('relaySource').value,
      relayComparator:$('relayComparator').value,
      relayThreshold:$('relayThreshold').value,
      relayHysteresis:$('relayHysteresis').value,
      relayMinSwitchMs:$('relayMinSwitchMs').value,
      relayManual:$('relayManual').value,
      relayFailSafe:$('relayFailSafe').value,
      relayActiveLow:$('relayActiveLow').checked?'1':'0'
    };
    const j=await apiPost('/api/settings',data);
    settingsDirty=false;
    toast(j.changed?'Settings saved':'No changes');
    await refreshStatus();
  }catch(err){
    toast(err.message,true);
  }finally{
    btn.disabled=false;
  }
});

$('reloadSettings').addEventListener('click',async()=>{
  settingsDirty=false;
  if(statusData)fillSettings(statusData);
  await refreshStatus();
});

$('wifiForm').addEventListener('submit',async e=>{
  e.preventDefault();
  const form=e.target;
  if(!form.reportValidity())return;
  const btn=form.querySelector('button[type="submit"]');
  btn.disabled=true;
  try{
    const data={
      ssid:$('wifiSsid').value,
      password:$('wifiPassword').value,
      channel:$('wifiChannel').value,
      hidden:$('wifiHidden').checked?'1':'0'
    };
    const j=await apiPost('/api/wifi',data);
    wifiDirty=false;
    $('wifiPassword').value='';
    toast(j.changed?'Wi-Fi settings saved. Access point restarting...':'No changes');
    setTimeout(()=>refreshStatus(),2500);
  }catch(err){
    toast(err.message,true);
  }finally{
    btn.disabled=false;
  }
});

$('restartBtn').addEventListener('click',async()=>{
  if(!confirm('Restart ESP32?'))return;
  $('restartBtn').disabled=true;
  try{
    await apiPost('/api/restart',{});
    toast('Restart scheduled');
  }catch(err){
    toast(err.message,true);
  }finally{
    setTimeout(()=>{$('restartBtn').disabled=false;},3000);
  }
});

$('factoryBtn').addEventListener('click',async()=>{
  if(!confirm('Factory reset will erase all settings and restart. Continue?'))return;
  $('factoryBtn').disabled=true;
  try{
    await apiPost('/api/factory-reset',{});
    toast('Factory reset scheduled');
  }catch(err){
    toast(err.message,true);
  }finally{
    setTimeout(()=>{$('factoryBtn').disabled=false;},3000);
  }
});

document.addEventListener('visibilitychange',()=>{
  if(!document.hidden)refreshStatus();
});

showTab(tabFromPath(location.pathname),false);
refreshStatus();
setInterval(()=>{
  if(!document.hidden)refreshStatus();
},2000);
</script>
</body>
</html>
)rawliteral";

static void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Content-Type-Options", "nosniff");
  server.send_P(200, "text/html", INDEX_HTML);
}

static void handleCaptive() {
  server.sendHeader("Location", "http://192.168.4.1/");
  server.sendHeader("Cache-Control", "no-store");
  server.send(302, "text/plain", "");
}

static void handleNotFound() {
  if (server.uri().startsWith("/api/")) {
    sendJsonError(404, "Not found");
    return;
  }
  handleCaptive();
}

static void handleStatus() {
  uint64_t now = nowUs();
  bool fresh = sensorDataFresh(now);

  char ssidEsc[96];
  jsonEscape(ssidEsc, sizeof(ssidEsc), gConfig.ssid);

  IPAddress ip = WiFi.softAPIP();
  char ipStr[16];
  snprintf(ipStr, sizeof(ipStr), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

  char buf[2048];
  int n = snprintf(
    buf,
    sizeof(buf),
    "{\"ok\":true,\"uptimeMs\":%llu,"
    "\"sensor\":{\"ok\":%s,\"valid\":%s,\"fresh\":%s,\"tempC\":%.2f,\"hum\":%.2f,\"lastReadAgeMs\":%llu,\"errors\":%llu,\"reads\":%llu,\"addr\":%u},"
    "\"relay\":{\"on\":%s,\"physical\":%u,\"enabled\":%s,\"source\":%u,\"comparator\":%u,\"threshold\":%.2f,\"hysteresis\":%.2f,\"activeLow\":%s,\"manual\":%u,\"failSafe\":%s,\"minSwitchMs\":%llu},"
    "\"wifi\":{\"ssid\":\"%s\",\"ip\":\"%s\",\"clients\":%u,\"channel\":%u,\"hidden\":%s,\"passwordSet\":%s},"
    "\"config\":{\"sampleIntervalMs\":%llu,\"tempOffset\":%.2f,\"humOffset\":%.2f,\"relayThreshold\":%.2f,\"relayHysteresis\":%.2f,\"relaySource\":%u,\"relayComparator\":%u,\"relayEnabled\":%s,\"relayActiveLow\":%s,\"relayManual\":%u,\"relayFailSafe\":%s,\"relayMinSwitchMs\":%llu},"
    "\"system\":{\"freeHeap\":%lu,\"minFreeHeap\":%lu,\"maxAllocHeap\":%lu,\"configSeq\":%llu,\"configSlot\":%u}}",
    (unsigned long long)(now / 1000ULL),
    sensorOk ? "true" : "false",
    (lastValidSensorUs != 0) ? "true" : "false",
    fresh ? "true" : "false",
    (double)tempCenti / 100.0,
    (double)humCenti / 100.0,
    (unsigned long long)(lastSensorReadUs == 0 ? 0 : (now - lastSensorReadUs) / 1000ULL),
    (unsigned long long)sensorErrorCount,
    (unsigned long long)sensorReadCount,
    (unsigned)sensorAddr,
    relayOn ? "true" : "false",
    (unsigned)relayPhysical,
    gConfig.relayEnabled ? "true" : "false",
    (unsigned)gConfig.relaySource,
    (unsigned)gConfig.relayComparator,
    (double)gConfig.relayThresholdCenti / 100.0,
    (double)gConfig.relayHysteresisCenti / 100.0,
    gConfig.relayActiveLow ? "true" : "false",
    (unsigned)gConfig.relayManual,
    gConfig.relayFailSafe ? "true" : "false",
    (unsigned long long)gConfig.relayMinSwitchMs,
    ssidEsc,
    ipStr,
    (unsigned)WiFi.softAPgetStationNum(),
    (unsigned)gConfig.channel,
    gConfig.hidden ? "true" : "false",
    (boundedStrLen(gConfig.password, sizeof(gConfig.password)) > 0) ? "true" : "false",
    (unsigned long long)gConfig.sampleIntervalMs,
    (double)gConfig.tempOffsetCenti / 100.0,
    (double)gConfig.humOffsetCenti / 100.0,
    (double)gConfig.relayThresholdCenti / 100.0,
    (double)gConfig.relayHysteresisCenti / 100.0,
    (unsigned)gConfig.relaySource,
    (unsigned)gConfig.relayComparator,
    gConfig.relayEnabled ? "true" : "false",
    gConfig.relayActiveLow ? "true" : "false",
    (unsigned)gConfig.relayManual,
    gConfig.relayFailSafe ? "true" : "false",
    (unsigned long long)gConfig.relayMinSwitchMs,
    (unsigned long)ESP.getFreeHeap(),
    (unsigned long)ESP.getMinFreeHeap(),
    (unsigned long)ESP.getMaxAllocHeap(),
    (unsigned long long)gConfigSeq,
    (unsigned)gActiveSlot
  );

  if (n < 0 || n >= (int)sizeof(buf)) {
    sendJsonError(500, "Status build failed");
    return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

static void handleSettingsSave() {
  if (server.method() != HTTP_POST) {
    sendJsonError(405, "Method not allowed");
    return;
  }

  uint64_t sampleIntervalMs;
  uint64_t minSwitchMs;
  uint64_t source;
  uint64_t comparator;
  uint64_t manual;
  bool enabled;
  bool activeLow;
  bool failSafe;
  int64_t tempOff;
  int64_t humOff;
  int64_t threshold;
  int64_t hysteresis;

  if (!parseUnsigned(server.arg("sampleIntervalMs").c_str(), 1000ULL, 600000ULL, sampleIntervalMs)) {
    sendJsonError(400, "Invalid sampleIntervalMs");
    return;
  }
  if (!parseDecimalCenti(server.arg("tempOffset").c_str(), -1000LL, 1000LL, tempOff)) {
    sendJsonError(400, "Invalid tempOffset");
    return;
  }
  if (!parseDecimalCenti(server.arg("humOffset").c_str(), -2000LL, 2000LL, humOff)) {
    sendJsonError(400, "Invalid humOffset");
    return;
  }
  if (!parseBoolArg(server.arg("relayEnabled").c_str(), enabled)) {
    sendJsonError(400, "Invalid relayEnabled");
    return;
  }
  if (!parseUnsigned(server.arg("relaySource").c_str(), 0ULL, 1ULL, source)) {
    sendJsonError(400, "Invalid relaySource");
    return;
  }
  if (!parseUnsigned(server.arg("relayComparator").c_str(), 0ULL, 1ULL, comparator)) {
    sendJsonError(400, "Invalid relayComparator");
    return;
  }
  if (!parseDecimalCenti(server.arg("relayThreshold").c_str(), -10000LL, 20000LL, threshold)) {
    sendJsonError(400, "Invalid relayThreshold");
    return;
  }
  if (!parseDecimalCenti(server.arg("relayHysteresis").c_str(), 0LL, 5000LL, hysteresis)) {
    sendJsonError(400, "Invalid relayHysteresis");
    return;
  }
  if (!parseUnsigned(server.arg("relayMinSwitchMs").c_str(), 0ULL, 600000ULL, minSwitchMs)) {
    sendJsonError(400, "Invalid relayMinSwitchMs");
    return;
  }
  if (!parseUnsigned(server.arg("relayManual").c_str(), 0ULL, 2ULL, manual)) {
    sendJsonError(400, "Invalid relayManual");
    return;
  }
  if (!parseBoolArg(server.arg("relayFailSafe").c_str(), failSafe)) {
    sendJsonError(400, "Invalid relayFailSafe");
    return;
  }
  if (!parseBoolArg(server.arg("relayActiveLow").c_str(), activeLow)) {
    sendJsonError(400, "Invalid relayActiveLow");
    return;
  }

  if (source == 0ULL) {
    if (threshold < -10000LL || threshold > 20000LL) {
      sendJsonError(400, "Invalid relayThreshold");
      return;
    }
  } else {
    if (threshold < 0LL || threshold > 10000LL) {
      sendJsonError(400, "Invalid relayThreshold");
      return;
    }
  }

  Config nc = gConfig;
  nc.sampleIntervalMs = sampleIntervalMs;
  nc.tempOffsetCenti = tempOff;
  nc.humOffsetCenti = humOff;
  nc.relayEnabled = enabled ? 1 : 0;
  nc.relaySource = (uint8_t)source;
  nc.relayComparator = (uint8_t)comparator;
  nc.relayThresholdCenti = threshold;
  nc.relayHysteresisCenti = hysteresis;
  nc.relayMinSwitchMs = minSwitchMs;
  nc.relayManual = (uint8_t)manual;
  nc.relayFailSafe = failSafe ? 1 : 0;
  nc.relayActiveLow = activeLow ? 1 : 0;

  bool changed = false;
  if (!saveConfigIfChanged(nc, changed)) {
    sendJsonError(500, "Save failed");
    return;
  }

  if (changed) {
    uint64_t now = nowUs();
    nextSensorTriggerUs = now;
    updateRelay(true, now);
  }

  char buf[96];
  snprintf(buf, sizeof(buf), "{\"ok\":true,\"changed\":%s}", changed ? "true" : "false");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

static bool isPrintableAscii(const String& s) {
  for (size_t i = 0; i < s.length(); ++i) {
    uint8_t c = (uint8_t)s.charAt(i);
    if (c < 0x20 || c > 0x7E) {
      return false;
    }
  }
  return true;
}

static void handleWifiSave() {
  if (server.method() != HTTP_POST) {
    sendJsonError(405, "Method not allowed");
    return;
  }

  String ssidArg = server.arg("ssid");
  if (ssidArg.length() < 1 || ssidArg.length() > 32) {
    sendJsonError(400, "Invalid SSID");
    return;
  }
  if (!isPrintableAscii(ssidArg)) {
    sendJsonError(400, "SSID must be printable ASCII");
    return;
  }

  String passArg = server.arg("password");
  if (passArg.length() > 0 && (passArg.length() < 8 || passArg.length() > 63)) {
    sendJsonError(400, "Invalid password");
    return;
  }
  if (passArg.length() > 0 && !isPrintableAscii(passArg)) {
    sendJsonError(400, "Password must be printable ASCII");
    return;
  }

  uint64_t channel;
  bool hidden;

  if (!parseUnsigned(server.arg("channel").c_str(), 1ULL, 13ULL, channel)) {
    sendJsonError(400, "Invalid channel");
    return;
  }
  if (!parseBoolArg(server.arg("hidden").c_str(), hidden)) {
    sendJsonError(400, "Invalid hidden");
    return;
  }

  Config nc = gConfig;
  copyStr(nc.ssid, sizeof(nc.ssid), ssidArg.c_str());
  if (passArg.length() > 0) {
    copyStr(nc.password, sizeof(nc.password), passArg.c_str());
  }
  nc.channel = (uint8_t)channel;
  nc.hidden = hidden ? 1 : 0;

  bool changed = false;
  if (!saveConfigIfChanged(nc, changed)) {
    sendJsonError(500, "Save failed");
    return;
  }

  if (changed) {
    scheduleWifiApply();
  }

  char buf[128];
  snprintf(
    buf,
    sizeof(buf),
    "{\"ok\":true,\"changed\":%s,\"applyScheduled\":%s}",
    changed ? "true" : "false",
    changed ? "true" : "false"
  );

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

static void handleRestart() {
  if (server.method() != HTTP_POST) {
    sendJsonError(405, "Method not allowed");
    return;
  }
  scheduleRestart();
  server.send(200, "application/json", "{\"ok\":true,\"scheduled\":true}");
}

static void handleFactoryReset() {
  if (server.method() != HTTP_POST) {
    sendJsonError(405, "Method not allowed");
    return;
  }
  scheduleFactoryReset();
  server.send(200, "application/json", "{\"ok\":true,\"scheduled\":true}");
}

static void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleRoot);
  server.on("/settings", HTTP_GET, handleRoot);
  server.on("/wifi", HTTP_GET, handleRoot);

  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/settings", HTTP_POST, handleSettingsSave);
  server.on("/api/wifi", HTTP_POST, handleWifiSave);
  server.on("/api/restart", HTTP_POST, handleRestart);
  server.on("/api/factory-reset", HTTP_POST, handleFactoryReset);

  server.on("/generate_204", HTTP_GET, handleCaptive);
  server.on("/gen_204", HTTP_GET, handleCaptive);
  server.on("/hotspot-detect.html", HTTP_GET, handleCaptive);
  server.on("/library/test/success.html", HTTP_GET, handleCaptive);
  server.on("/connecttest.txt", HTTP_GET, handleCaptive);
  server.on("/redirect", HTTP_GET, handleCaptive);
  server.on("/canonical.html", HTTP_GET, handleCaptive);
  server.on("/success.txt", HTTP_GET, handleCaptive);
  server.on("/ncsi.txt", HTTP_GET, handleCaptive);
  server.on("/fwlink", HTTP_GET, handleCaptive);

  server.onNotFound(handleNotFound);
  server.begin();
}

void setup() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  loadConfig();

  pinMode(PIN_RELAY, OUTPUT);
  relayOn = false;
  relayPhysical = 255;
  digitalWrite(PIN_RELAY, gConfig.relayActiveLow ? HIGH : LOW);

  pinMode(PIN_BOOT, INPUT_PULLUP);

  applyAPSettings();
  setupWebServer();

  uint64_t now = nowUs();
  nextSensorTriggerUs = now;
  lastRelaySwitchUs = 0;
  updateRelay(true, now);
}

void loop() {
  uint64_t now = nowUs();

  updateBootButton(now);
  updateSensor(now);
  updateRelay(false, now);
  updatePendingActions(now);

  server.handleClient();
  dnsServer.processNextRequest();

  yield();
}
