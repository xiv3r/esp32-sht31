#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <EEPROM.h>
#include <stddef.h>
#include <ctype.h>
#include <math.h>

#define SDA_PIN 5
#define SCL_PIN 4
#define RELAY_PIN 12
#define BUTTON_PIN 0

#define AP_SSID "ESP8266_SHT31"
#define AP_PASS "12345678"

#define EEPROM_SIZE 512
#define EEPROM_MAGIC 0x5A5A5A5AUL

static const int64_t DEFAULT_TEMP_MM = 30000;
static const int64_t DEFAULT_HUM_MM  = 70000;
static const int64_t DEFAULT_HYST_MM = 1000;

struct Config {
  uint32_t magic;
  int64_t tempThresholdMilli;
  int64_t humThresholdMilli;
  int64_t hystMilli;
  uint32_t crc;
} __attribute__((packed));

static Config cfg;
static Adafruit_SHT31 sht31;
static ESP8266WebServer server(80);
static DNSServer dnsServer;

static bool shtOK = false;
static bool relayState = false;
static bool sensorValid = false;
static bool eepromDirty = false;
static bool pendingReset = false;
static bool pendingFactory = false;
static bool pendingResetFromButton = false;
static uint32_t pendingResetAt = 0;

static int64_t lastTempMilli = 0;
static int64_t lastHumMilli = 0;
static uint32_t lastSensorRead = 0;
static uint32_t lastDnsProcess = 0;
static uint32_t bootButtonPressStart = 0;
static bool bootButtonHeld = false;
static uint32_t eepromDirtyAt = 0;
static uint32_t sensorInvalidSince = 0;

static uint32_t crc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
  }
  return ~crc;
}

static void saveConfig() {
  cfg.magic = EEPROM_MAGIC;
  cfg.crc = crc32((uint8_t*)&cfg, offsetof(Config, crc));
  EEPROM.put(0, cfg);
  if (EEPROM.commit()) { eepromDirty = false; }
}

static void applyDefaults() {
  cfg.tempThresholdMilli = DEFAULT_TEMP_MM;
  cfg.humThresholdMilli  = DEFAULT_HUM_MM;
  cfg.hystMilli          = DEFAULT_HYST_MM;
}

static void loadConfig() {
  EEPROM.get(0, cfg);
  if (cfg.magic != EEPROM_MAGIC) { applyDefaults(); saveConfig(); return; }
  uint32_t expected = crc32((uint8_t*)&cfg, offsetof(Config, crc));
  if (cfg.crc != expected) { applyDefaults(); saveConfig(); return; }
  int64_t ct = cfg.tempThresholdMilli;
  int64_t ch = cfg.humThresholdMilli;
  int64_t cy = cfg.hystMilli;
  if (ct < -50000) ct = -50000;
  if (ct > 150000) ct = 150000;
  if (ch < 0)      ch = 0;
  if (ch > 100000) ch = 100000;
  if (cy < 0)      cy = 0;
  if (cy > 50000)  cy = 50000;
  if (ct != cfg.tempThresholdMilli || ch != cfg.humThresholdMilli || cy != cfg.hystMilli) {
    cfg.tempThresholdMilli = ct;
    cfg.humThresholdMilli  = ch;
    cfg.hystMilli          = cy;
    saveConfig();
  }
}

static void scheduleReset() {
  pendingFactory = false;
  pendingReset = true;
  pendingResetFromButton = false;
  pendingResetAt = millis();
}

static void scheduleFactoryReset(bool fromButton) {
  pendingFactory = true;
  pendingReset = true;
  pendingResetFromButton = fromButton;
  pendingResetAt = millis();
}

static void readSensor() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastSensorRead) < 2000UL) return;
  lastSensorRead = now;

  if (!shtOK) {
    sensorValid = false;
    Wire.beginTransmission(0x44);
    if (Wire.endTransmission() == 0) shtOK = sht31.begin(0x44);
    else {
      Wire.beginTransmission(0x45);
      if (Wire.endTransmission() == 0) shtOK = sht31.begin(0x45);
    }
    if (shtOK) {
      Wire.setClock(100000);
      Wire.setClockStretchLimit(1500);
    }
    return;
  }

  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  if (isnan(t) || isnan(h)) { shtOK = false; sensorValid = false; return; }

  int64_t tm = (int64_t)(t * 1000.0f);
  int64_t hm = (int64_t)(h * 1000.0f);
  if (tm < -100000) tm = -100000;
  if (tm >  150000) tm =  150000;
  if (hm < 0)       hm = 0;
  if (hm > 100000)  hm = 100000;

  lastTempMilli = tm;
  lastHumMilli  = hm;
  sensorValid = true;
}

static void updateRelay() {
  if (!sensorValid) {
    if (sensorInvalidSince == 0) sensorInvalidSince = millis();
    else if ((uint32_t)(millis() - sensorInvalidSince) >= 60000UL) {
      if (relayState) { relayState = false; digitalWrite(RELAY_PIN, LOW); }
    }
    return;
  }
  sensorInvalidSince = 0;
  int64_t hyst = cfg.hystMilli;
  int64_t tTh  = cfg.tempThresholdMilli;
  int64_t hTh  = cfg.humThresholdMilli;
  if (hyst < 0) hyst = 0;
  if (hyst > tTh) hyst = (tTh > 0) ? tTh : 0;
  if (hyst > hTh) hyst = (hTh > 0) ? hTh : 0;
  if (!relayState) {
    if (lastTempMilli >= tTh || lastHumMilli >= hTh) {
      relayState = true; digitalWrite(RELAY_PIN, HIGH);
    }
  } else {
    if (lastTempMilli <= (tTh - hyst) && lastHumMilli <= (hTh - hyst)) {
      relayState = false; digitalWrite(RELAY_PIN, LOW);
    }
  }
}

static void checkBootButton() {
  uint32_t now = millis();
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (!bootButtonHeld) { bootButtonHeld = true; bootButtonPressStart = now; }
    else if ((uint32_t)(now - bootButtonPressStart) >= 5000UL && !pendingFactory) {
      scheduleFactoryReset(true);
    }
  } else bootButtonHeld = false;
}

static void processDns() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastDnsProcess) < 30UL) return;
  lastDnsProcess = now;
  dnsServer.processNextRequest();
}

static void flushEeprom() {
  if (!eepromDirty) return;
  uint32_t now = millis();
  if ((uint32_t)(now - eepromDirtyAt) < 2000UL) return;
  eepromDirtyAt = now;
  saveConfig();
  if (eepromDirty) eepromDirtyAt = millis();
}

static void handlePendingReset() {
  if (!pendingReset) return;
  if ((uint32_t)(millis() - pendingResetAt) < 200UL) return;
  if (pendingResetFromButton) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      if ((uint32_t)(millis() - pendingResetAt) < 10000UL) return;
    }
  }
  pendingReset = false;
  pendingResetFromButton = false;
  if (pendingFactory) {
    pendingFactory = false;
    cfg.magic = 0;
    cfg.crc = 0;
    EEPROM.put(0, cfg);
    EEPROM.commit();
  } else if (eepromDirty) {
    saveConfig();
  }
  ESP.restart();
}

static bool parseDoubleStrict(const String &s, double &out) {
  if (s.length() == 0 || s.length() > 16) return false;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (!(isdigit((unsigned char)c) || c=='.' || c=='-' || c=='+')) return false;
  }
  char *end = nullptr;
  double v = strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != 0) return false;
  if (isnan(v) || isinf(v)) return false;
  out = v;
  return true;
}

static void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8", "");

  static const char P_HEAD[] PROGMEM =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP8266 SHT31</title><style>"
    "body{font-family:Arial;margin:0;padding:16px;background:#f0f0f0}"
    "h1{font-size:20px;color:#333}"
    ".card{background:#fff;border-radius:8px;padding:16px;margin:8px 0;"
    "box-shadow:0 1px 3px rgba(0,0,0,.2)}"
    "label{display:block;margin:8px 0 4px;font-size:14px;color:#555}"
    "input{width:100%;padding:8px;box-sizing:border-box;"
    "border:1px solid #ccc;border-radius:4px}"
    "button{padding:10px 16px;margin:8px 4px 0 0;background:#2196F3;"
    "color:#fff;border:none;border-radius:4px;cursor:pointer;font-size:14px}"
    ".val{font-size:28px;font-weight:bold;color:#2196F3}"
    ".relay-on{color:#4CAF50}.relay-off{color:#F44336}"
    ".grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
    "@media(max-width:600px){.grid{grid-template-columns:1fr}}"
    "</style></head><body><h1>ESP8266 SHT31 Dashboard</h1>"
    "<div class='card'><div class='grid'>"
    "<div><label>Temperature (&deg;C)</label><div id='temp' class='val'>--</div></div>"
    "<div><label>Humidity (%)</label><div id='hum' class='val'>--</div></div>"
    "<div><label>Relay State</label><div id='relay' class='val'>--</div></div>"
    "<div><label>Sensor Status</label><div id='sensor' class='val'>--</div></div>"
    "</div></div><div class='card'><form action='/set' method='get'>";
  server.sendContent_P(P_HEAD);

  {
    char buf[128];
    {
      int64_t v = cfg.tempThresholdMilli;
      bool neg = v < 0;
      uint64_t av = neg ? (uint64_t)(-v) : (uint64_t)v;
      snprintf(buf, sizeof(buf),
        "<label>Temp Threshold (&deg;C)</label><input name='t' value='%s%lu.%03lu' step='0.1' inputmode='decimal'>",
        neg ? "-" : "", (unsigned long)(av / 1000), (unsigned long)(av % 1000));
      server.sendContent(buf);
    }
    {
      int64_t v = cfg.humThresholdMilli;
      bool neg = v < 0;
      uint64_t av = neg ? (uint64_t)(-v) : (uint64_t)v;
      snprintf(buf, sizeof(buf),
        "<label>Hum Threshold (%%)</label><input name='h' value='%s%lu.%03lu' step='0.1' inputmode='decimal'>",
        neg ? "-" : "", (unsigned long)(av / 1000), (unsigned long)(av % 1000));
      server.sendContent(buf);
    }
    {
      int64_t v = cfg.hystMilli;
      bool neg = v < 0;
      uint64_t av = neg ? (uint64_t)(-v) : (uint64_t)v;
      snprintf(buf, sizeof(buf),
        "<label>Hysteresis</label><input name='y' value='%s%lu.%03lu' step='0.1' inputmode='decimal'>",
        neg ? "-" : "", (unsigned long)(av / 1000), (unsigned long)(av % 1000));
      server.sendContent(buf);
    }
  }

  static const char P_TAIL[] PROGMEM =
    "<button type='submit'>Save</button></form></div>"
    "<div class='card'><button onclick='rst()'>Factory Reset</button>"
    "<button onclick='rbt()'>Restart</button></div><script>"
    "async function st(){try{const r=await fetch('/status');const j=await r.json();"
    "document.getElementById('temp').textContent=(j.t/1000).toFixed(2);"
    "document.getElementById('hum').textContent=(j.h/1000).toFixed(2);"
    "const e=document.getElementById('relay');e.textContent=j.r?'ON':'OFF';"
    "e.className='val '+(j.r?'relay-on':'relay-off');"
    "document.getElementById('sensor').textContent=j.s?'OK':'ERROR';"
    "}catch(e){}}setInterval(st,2000);st();"
    "function rst(){if(confirm('Factory reset?')){fetch('/factory').catch(function(){});setTimeout(function(){location.reload();},3000);}}"
    "function rbt(){if(confirm('Restart?')){fetch('/restart').catch(function(){});setTimeout(function(){location.reload();},3000);}}"
    "</script></body></html>";
  server.sendContent_P(P_TAIL);
  server.sendContent("");
}

static void handleStatus() {
  char buf[128];
  snprintf(buf, sizeof(buf),
    "{\"t\":%lld,\"h\":%lld,\"r\":%d,\"s\":%d,\"m\":%lu}",
    (long long)lastTempMilli, (long long)lastHumMilli,
    relayState?1:0, sensorValid?1:0,
    (unsigned long)ESP.getFreeHeap());
  server.send(200, "application/json", buf);
}

static void handleSet() {
  double t, h, y;
  bool ok = parseDoubleStrict(server.arg("t"), t)
         && parseDoubleStrict(server.arg("h"), h)
         && parseDoubleStrict(server.arg("y"), y);
  bool applied = false;
  if (ok && t>=-50.0 && t<=150.0 && h>=0.0 && h<=100.0 && y>=0.0 && y<=50.0) {
    int64_t nt = (int64_t)llround(t * 1000.0);
    int64_t nh = (int64_t)llround(h * 1000.0);
    int64_t ny = (int64_t)llround(y * 1000.0);
    if (ny <= nt && ny <= nh) {
      applied = true;
      if (nt != cfg.tempThresholdMilli || nh != cfg.humThresholdMilli || ny != cfg.hystMilli) {
        cfg.tempThresholdMilli = nt;
        cfg.humThresholdMilli  = nh;
        cfg.hystMilli          = ny;
        eepromDirty = true;
        eepromDirtyAt = millis();
      }
    }
  }
  server.sendHeader("Location", applied ? "/" : "/?err=1");
  server.send(302, "text/plain", "");
}

static void handleFactory() {
  server.send(200, "text/plain", "OK");
  scheduleFactoryReset(false);
}

static void handleRestart() {
  server.send(200, "text/plain", "OK");
  scheduleReset();
}

static void handleCaptive() {
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

static void setupAP() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  WiFi.softAP(AP_SSID, AP_PASS);
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", IPAddress(192,168,4,1));
}

static void setupServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/set", HTTP_GET, handleSet);
  server.on("/factory", HTTP_GET, handleFactory);
  server.on("/restart", HTTP_GET, handleRestart);
  server.on("/generate_204", HTTP_GET, handleCaptive);
  server.on("/gen_204", HTTP_GET, handleCaptive);
  server.on("/hotspot-detect.html", HTTP_GET, handleCaptive);
  server.on("/connecttest.txt", HTTP_GET, handleCaptive);
  server.on("/redirect", HTTP_GET, handleCaptive);
  server.on("/canonical.html", HTTP_GET, handleCaptive);
  server.on("/ncsi.txt", HTTP_GET, handleCaptive);
  server.on("/fwlink", HTTP_GET, handleCaptive);
  server.on("/library/test/success.html", HTTP_GET, handleCaptive);
  server.onNotFound(handleCaptive);
  server.begin();
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  EEPROM.begin(EEPROM_SIZE);
  loadConfig();
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setClockStretchLimit(1500);
  shtOK = sht31.begin(0x44);
  if (!shtOK) shtOK = sht31.begin(0x45);
  Wire.setClock(100000);
  Wire.setClockStretchLimit(1500);
  setupAP();
  setupServer();
  lastSensorRead = millis() - 2000UL;
}

void loop() {
  checkBootButton();
  readSensor();
  updateRelay();
  processDns();
  server.handleClient();
  flushEeprom();
  handlePendingReset();
  yield();
}
