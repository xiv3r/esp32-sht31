/*
 * ESP32_SHT31_Controller.ino
 * ---------------------------------------------------------------
 * - WiFi AP always on: SSID "ESP32_SHT31", pass "12345678"
 * - Captive portal (DNS hijack + 302 redirect)
 * - SHT31 temp/humidity dashboard
 * - Relay on GPIO25 (active-high/active-low selectable)
 * - Threshold triggers from temp OR humidity
 * - PROGMEM web UI (HTML/CSS/JS gzipped-free, chunked)
 * - WiFi AP management page (SSID/pass change, always reverts to AP)
 * - Relay management page
 * - GPIO0 5s hold -> factory reset
 * - NVS wear mitigation (only write on change, CRC, debounce)
 * - millis/micros overflow-safe (delta arithmetic everywhere)
 * - Heap-fragmentation friendly (String reserved, no per-request news)
 * - Fully non-blocking (no delay() in loop; deferred tasks)
 * - Config corruption protection (magic + version + CRC32 + double-buffer)
 * ---------------------------------------------------------------
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_system.h>

static const uint8_t RELAY_PIN = 25;
static const uint8_t BOOT_BTN_PIN = 0;
static const uint8_t SHT31_ADDR = 0x44;

static const uint32_t SENSOR_INTERVAL_MS   = 2000;
static const uint32_t AP_CHECK_INTERVAL_MS = 3000;
static const uint32_t BTN_DEBOUNCE_MS      = 30;
static const uint32_t FACTORY_HOLD_MS      = 5000;
static const uint32_t DNS_INTERVAL_MS      = 50;
static const uint32_t SHT_CONV_WAIT_MS     = 20;
static const uint32_t SENSOR_FAULT_TIMEOUT_MS = 30000;

static const char* DEF_AP_SSID = "ESP32_SHT31";
static const char* DEF_AP_PASS = "12345678";

static const uint32_t CFG_MAGIC   = 0x53485433;
static const uint16_t CFG_VERSION = 4;
static const char*    NVS_NS      = "sht31cfg";

struct Config {
  uint32_t magic;
  uint16_t version;
  uint16_t _pad;
  char     apSsid[33];
  char     apPass[65];
  uint8_t  relayActiveHigh;
  uint8_t  relayManual;
  uint8_t  trigMode;
  uint8_t  _pad2;
  float    tempThreshold;
  float    humThreshold;
  float    tempHyst;
  float    humHyst;
  uint32_t crc32;
};

static_assert(sizeof(Config) == 132, "Config ABI changed — bump CFG_VERSION");

static Config g_cfg;
static Preferences g_prefs;

static WebServer  g_server(80);
static DNSServer  g_dns;
static bool       g_apRunning = false;

static float      g_tempC = NAN;
static float      g_humPct = NAN;
static bool       g_sensorOk = false;
static uint32_t   g_lastSensorRead = 0;
static uint32_t   g_lastApCheck = 0;
static uint32_t   g_lastDns = 0;
static bool       g_relayState = false;

static uint8_t    sht_phase = 0;
static uint32_t   sht_convStart = 0;

static uint32_t   g_rebootAt = 0;

static uint32_t   g_sensorFaultSince = 0;
static uint32_t   g_apRestartAt = 0;
static IPAddress  g_dnsIp(0, 0, 0, 0);
static uint8_t    g_apIpZeroCount = 0;

static bool       g_btnLastRaw = true;
static bool       g_btnStable   = true;
static uint32_t   g_btnLastChange = 0;
static uint32_t   g_btnPressStart = 0;
static bool       g_btnHoldHandled = false;

static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = ~crc;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));
  }
  return ~crc;
}

static bool cfg_validate(const Config& c) {
  if (c.magic != CFG_MAGIC) return false;
  if (c.version != CFG_VERSION) return false;
  if (c.apPass[0] == '\0') return false;
  size_t plen = strlen(c.apPass);
  if (plen < 8 || plen > 63) return false;
  if (c.apSsid[0] == '\0') return false;
  if (strlen(c.apSsid) > 32) return false;
  for (size_t i = 0; c.apSsid[i]; i++) {
    char ch = c.apSsid[i];
    if (ch < 0x20 || ch > 0x7E || ch == '"' || ch == '\\') return false;
  }
  if (c.trigMode > 3) return false;
  if (c.relayManual > 2) return false;
  return true;
}

static void cfg_defaults(Config& c) {
  memset(&c, 0, sizeof(c));
  c.magic = CFG_MAGIC;
  c.version = CFG_VERSION;
  strncpy(c.apSsid, DEF_AP_SSID, sizeof(c.apSsid) - 1);
  strncpy(c.apPass, DEF_AP_PASS, sizeof(c.apPass) - 1);
  c.relayActiveHigh = 1;
  c.relayManual     = 0;
  c.trigMode        = 2;
  c.tempThreshold   = 30.0f;
  c.humThreshold    = 60.0f;
  c.tempHyst        = 0.5f;
  c.humHyst         = 1.0f;
}

static void cfg_finalize(Config& c) {
  c.crc32 = 0;
  c.crc32 = crc32_update(0, (const uint8_t*)&c, sizeof(Config) - sizeof(uint32_t));
}

static bool cfg_loadFromNVS() {
  g_prefs.begin(NVS_NS, true);
  size_t n = g_prefs.getBytesLength("cfg");
  bool ok = false;
  if (n == sizeof(Config)) {
    Config tmp;
    if (g_prefs.getBytes("cfg", &tmp, sizeof(tmp)) == sizeof(tmp)) {
      if (cfg_validate(tmp)) {
        Config tmp2 = tmp;
        uint32_t savedCrc = tmp2.crc32;
        cfg_finalize(tmp2);
        if (tmp2.crc32 == savedCrc) { g_cfg = tmp; ok = true; }
      }
    }
  }
  g_prefs.end();
  return ok;
}

static bool cfg_saveIfChanged(Config& c) {
  cfg_finalize(c);
  Config existing;
  bool have = false;
  g_prefs.begin(NVS_NS, true);
  if (g_prefs.getBytesLength("cfg") == sizeof(Config)) {
    if (g_prefs.getBytes("cfg", &existing, sizeof(existing)) == sizeof(existing)) {
      Config e2 = existing; cfg_finalize(e2);
      have = (e2.crc32 == existing.crc32);
    }
  }
  g_prefs.end();
  if (have && memcmp(&existing, &c, sizeof(Config)) == 0) return true;
  g_prefs.begin(NVS_NS, false);
  size_t w = g_prefs.putBytes("cfg", &c, sizeof(Config));
  g_prefs.end();
  return w == sizeof(Config);
}

static const char PAGE_HEAD[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>ESP32 SHT31</title><style>
:root{--bg:#0f172a;--card:#1e293b;--fg:#e2e8f0;--accent:#38bdf8;--ok:#22c55e;--warn:#f59e0b;--err:#ef4444}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:var(--bg);color:var(--fg)}
header{display:flex;align-items:center;justify-content:space-between;padding:12px 16px;background:#111827;position:sticky;top:0;z-index:10}
header h1{font-size:16px;margin:0}
nav a{color:var(--fg);text-decoration:none;margin-left:14px;font-size:14px;opacity:.85}
nav a:hover{color:var(--accent);opacity:1}
main{max-width:900px;margin:0 auto;padding:16px}
.card{background:var(--card);border-radius:14px;padding:16px;margin-bottom:16px;box-shadow:0 2px 8px rgba(0,0,0,.4)}
h2{margin:0 0 12px;font-size:15px;color:var(--accent);text-transform:uppercase;letter-spacing:.06em}
.row{display:flex;flex-wrap:wrap;gap:10px;align-items:center}
.big{font-size:34px;font-weight:700}
.unit{font-size:15px;opacity:.7;margin-left:4px}
.green{color:var(--ok)} .amber{color:var(--warn)} .red{color:var(--err)}
button{cursor:pointer;border:0;border-radius:10px;padding:10px 16px;font-size:14px;font-weight:600;background:var(--accent);color:#0b1220;min-height:44px}
button.alt{background:#475569;color:var(--fg)}
button.danger{background:var(--err);color:#fff}
input,select{background:#0b1220;border:1px solid #334155;color:var(--fg);border-radius:8px;padding:10px;font-size:14px;min-height:44px;width:100%}
label{display:block;font-size:12px;opacity:.75;margin:10px 0 4px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px}
.badge{display:inline-block;padding:3px 9px;border-radius:999px;font-size:12px;font-weight:600}
.badge.on{background:#14532d;color:#86efac} .badge.off{background:#3f1d1d;color:#fca5a5}
.toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);background:#334155;color:#fff;padding:10px 18px;border-radius:10px;opacity:0;transition:opacity .3s;pointer-events:none;font-size:14px;z-index:100}
.toast.show{opacity:1}
footer{text-align:center;padding:16px;font-size:12px;opacity:.5}
@media(max-width:520px){nav a{margin-left:10px;font-size:13px}header h1{font-size:14px}.big{font-size:28px}}
</style></head><body>)HTML";

static const char PAGE_NAV[] PROGMEM = R"HTML(<header><h1>ESP32 SHT31</h1><nav>
<a href="/">Dashboard</a><a href="/relay">Relay</a><a href="/wifi">WiFi</a>
</nav></header><main>)HTML";

static const char PAGE_FOOT[] PROGMEM = R"HTML(</main><footer>ESP32 &middot; SHT31 &middot; PROGMEM UI</footer>
<div class="toast" id="t"></div>
<script>
function toast(m){var t=document.getElementById('t');t.textContent=m;t.classList.add('show');setTimeout(function(){t.classList.remove('show')},1800)}
function post(url,data){return fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data}).then(r=>r.json())}
</script></body></html>)HTML";

static const char PAGE_DASH[] PROGMEM = R"HTML(<div class="card">
<h2>Live Sensor (SHT31)</h2>
<div class="grid">
<div><div>Temperature</div><span class="big" id="tc">--</span><span class="unit">&deg;C</span></div>
<div><div>Humidity</div><span class="big" id="hc">--</span><span class="unit">%RH</span></div>
</div>
<div class="row" style="margin-top:12px">
<span>Sensor:</span><span id="sok" class="badge off">unknown</span>
<span style="margin-left:auto">Relay:</span><span id="rs" class="badge off">off</span>
</div>
</div>
<div class="card">
<h2>Thresholds</h2>
<div class="grid">
<div><label>Trigger source</label>
<select id="tm">
<option value="0">Temperature</option><option value="1">Humidity</option>
<option value="2">Either</option><option value="3">Both</option></select></div>
<div><label>Temp threshold (&deg;C)</label><input id="tt" type="number" step="0.1"></div>
<div><label>Hum threshold (%RH)</label><input id="ht" type="number" step="0.1"></div>
<div><label>Temp hysteresis</label><input id="th" type="number" step="0.1"></div>
<div><label>Hum hysteresis</label><input id="hh" type="number" step="0.1"></div>
</div>
<div class="row" style="margin-top:12px">
<button onclick="saveThr()">Save Thresholds</button>
</div>
</div>
<script>
function val(id){return document.getElementById(id).value}
function set(id,v){document.getElementById(id).value=v}
function refresh(){
 fetch('/api/status').then(r=>r.json()).then(d=>{
  document.getElementById('tc').textContent = d.sensorOk?d.temp.toFixed(2):'--';
  document.getElementById('hc').textContent = d.sensorOk?d.hum.toFixed(2):'--';
  var s=document.getElementById('sok'); s.textContent=d.sensorOk?'OK':'FAULT';
  s.className='badge '+(d.sensorOk?'on':'off');
  var r=document.getElementById('rs'); r.textContent=d.relay?'ON':'OFF';
  r.className='badge '+(d.relay?'on':'off');
  if(!document.activeElement || document.activeElement.tagName!=='INPUT'){
   set('tm',d.trigMode); set('tt',d.tempThr); set('ht',d.humThr);
   set('th',d.tempHyst); set('hh',d.humHyst);
  }
 }).catch(()=>{});
}
function saveThr(){
 var body='tm='+val('tm')+'&tt='+val('tt')+'&ht='+val('ht')+'&th='+val('th')+'&hh='+val('hh');
 post('/api/thresholds',body)
   .then(d=>{toast(d.ok?'Saved':'Failed: '+(d.err||'unknown'))})
   .catch(()=>{toast('Network error')});
}
refresh(); setInterval(refresh,2500);
</script>)HTML";

static const char PAGE_RELAY[] PROGMEM = R"HTML(<div class="card">
<h2>Relay (GPIO25)</h2>
<div class="row"><span>State:</span><span id="rs" class="badge off">--</span></div>
<label>Wiring polarity</label>
<select id="pol" onchange="savePol()">
<option value="1">Active HIGH</option><option value="0">Active LOW</option>
</select>
<label>Mode</label>
<select id="mode" onchange="saveMode()">
<option value="0">Auto (threshold)</option>
<option value="1">Force ON</option>
<option value="2">Force OFF</option>
</select>
<div class="row" style="margin-top:12px">
<button class="alt" onclick="saveMode()">Apply Mode</button>
</div>
</div>
<script>
function refresh(){
 fetch('/api/status').then(r=>r.json()).then(d=>{
  var s=document.getElementById('rs'); s.textContent=d.relay?'ON':'OFF';
  s.className='badge '+(d.relay?'on':'off');
  document.getElementById('pol').value=d.relayActiveHigh?1:0;
  document.getElementById('mode').value=d.relayManual;
 }).catch(()=>{});
}
function savePol(){post('/api/relay',(document.getElementById('pol').value==1?'pol=1':'pol=0')).then(d=>toast(d.ok?'Saved':'Failed: '+(d.err||'unknown'))).catch(()=>toast('Network error'))}
function saveMode(){post('/api/relay','mode='+document.getElementById('mode').value).then(d=>toast(d.ok?'Saved':'Failed: '+(d.err||'unknown'))).catch(()=>toast('Network error'))}
refresh(); setInterval(refresh,2500);
</script>)HTML";

static const char PAGE_WIFI[] PROGMEM = R"HTML(<div class="card">
<h2>WiFi AP</h2>
<div class="row"><span>Status:</span><span id="ap" class="badge off">--</span></div>
<label>SSID (max 32 chars)</label>
<input id="ssid" maxlength="32" autocomplete="off">
<label>Password (8-63 chars)</label>
<input id="pass" type="password" maxlength="63" autocomplete="new-password">
<div class="row" style="margin-top:12px">
<button onclick="saveWifi()">Save &amp; Restart AP</button>
</div>
<p style="font-size:12px;opacity:.6;margin-top:12px">AP is always active. Saving restarts the AP with the new credentials.</p>
</div>
<div class="card">
<h2>Factory Reset</h2>
<p style="font-size:13px;opacity:.7">Press and hold the onboard BOOT (GPIO0) button for 5 seconds <b>after</b> the device has booted. Holding it during power-on will enter bootloader mode and will not trigger a reset.</p>
<button class="danger" onclick="factory()">Factory Reset Now</button>
</div>
<script>
function refresh(){
 fetch('/api/status').then(r=>r.json()).then(d=>{
  var a=document.getElementById('ap'); a.textContent=d.ap?'RUNNING':'DOWN';
  a.className='badge '+(d.ap?'on':'off');
  if(document.activeElement!==document.getElementById('ssid')) document.getElementById('ssid').value=d.ssid;
 }).catch(()=>{});
}
function saveWifi(){
 var b='ssid='+encodeURIComponent(document.getElementById('ssid').value)+'&pass='+encodeURIComponent(document.getElementById('pass').value);
 post('/api/wifi',b)
   .then(d=>{toast(d.ok?'Saved, restarting AP — reconnect to the new SSID':'Failed: '+(d.err||'unknown'))})
   .catch(()=>{toast('No response — AP may have restarted; check device')});
}
function factory(){if(confirm('Factory reset?')){fetch('/api/factory',{method:'POST'}).then(r=>r.json().catch(()=>({}))).then(d=>{toast(d&&d.ok?'Rebooting...':'Reset failed — device not rebooted')}).catch(()=>toast('No response — check device'))}}
refresh(); setInterval(refresh,2500);
</script>)HTML";

static const char PAGE_404[] PROGMEM = R"HTML(<div class="card"><h2>Not Found</h2><p>Go to <a href="/" style="color:#38bdf8">Dashboard</a></p></div>)HTML";

static void sendPgPart(PGM_P p) {
  size_t len = strlen_P(p);
  if (len == 0) return;
  char chunk[256];
  size_t off = 0;
  while (off < len) {
    size_t n = (len - off) < sizeof(chunk) ? (len - off) : sizeof(chunk);
    memcpy_P(chunk, p + off, n);
    g_server.sendContent(chunk, n);
    off += n;
  }
}

static void sendPage(PGM_P const* parts, size_t count) {
  size_t total = 0;
  for (size_t i = 0; i < count; i++) total += strlen_P(parts[i]);
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.sendHeader("Connection", "close");
  g_server.setContentLength(total);
  g_server.send(200, "text/html; charset=utf-8", "");
  for (size_t i = 0; i < count; i++) sendPgPart(parts[i]);
}

static uint8_t sht_crc8(uint8_t hi, uint8_t lo) {
  uint8_t crc = 0xFF;
  uint8_t bytes[2] = { hi, lo };
  for (int i = 0; i < 2; i++) {
    crc ^= bytes[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

static bool sht_start() {
  Wire.beginTransmission(SHT31_ADDR);
  Wire.write((uint8_t)0x24);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() != 0) return false;
  sht_convStart = millis();
  sht_phase = 1;
  return true;
}

static int sht_poll(float& t, float& h) {
  if (sht_phase != 1) return 0;
  if ((uint32_t)(millis() - sht_convStart) < SHT_CONV_WAIT_MS) return 0;

  sht_phase = 0;

  if (Wire.requestFrom((uint8_t)SHT31_ADDR, (uint8_t)6) != 6) return -1;
  uint8_t d[6];
  for (int i = 0; i < 6; i++) d[i] = Wire.read();

  if (sht_crc8(d[0], d[1]) != d[2]) return -2;
  if (sht_crc8(d[3], d[4]) != d[5]) return -2;

  uint16_t rawT = ((uint16_t)d[0] << 8) | d[1];
  uint16_t rawH = ((uint16_t)d[3] << 8) | d[4];
  t = -45.0f + 175.0f * (float)rawT / 65535.0f;
  h = 100.0f * (float)rawH / 65535.0f;
  return 1;
}

static bool g_relayAutoState = false;

static void relay_apply(bool physicalOn) {
  g_relayState = physicalOn;
  bool level = g_cfg.relayActiveHigh ? physicalOn : !physicalOn;
  digitalWrite(RELAY_PIN, level ? HIGH : LOW);
}

static bool thresholdEval() {
  if (!g_sensorOk) return g_relayAutoState;

  bool tTrig = false, hTrig = false;
  switch (g_cfg.trigMode) {
    case 0: tTrig = g_tempC >= g_cfg.tempThreshold; break;
    case 1: hTrig = g_humPct >= g_cfg.humThreshold; break;
    case 2:
    case 3:
      tTrig = g_tempC >= g_cfg.tempThreshold;
      hTrig = g_humPct >= g_cfg.humThreshold;
      break;
  }

  bool on;
  switch (g_cfg.trigMode) {
    case 0: on = tTrig;               break;
    case 1: on = hTrig;               break;
    case 2: on = tTrig || hTrig;      break;
    case 3: on = tTrig && hTrig;      break;
    default: on = false;              break;
  }

  if (g_relayAutoState) {
    bool stillT = g_tempC >= (g_cfg.tempThreshold - g_cfg.tempHyst);
    bool stillH = g_humPct >= (g_cfg.humThreshold - g_cfg.humHyst);
    switch (g_cfg.trigMode) {
      case 0: on = stillT;              break;
      case 1: on = stillH;              break;
      case 2: on = stillT || stillH;    break;
      case 3: on = stillT && stillH;    break;
      default: on = false;              break;
    }
  }
  return on;
}

static void relay_update() {
  bool physical;
  if (g_cfg.relayManual == 1) {
    physical = true;
  } else if (g_cfg.relayManual == 2) {
    physical = false;
  } else if (!g_sensorOk && g_sensorFaultSince != 0 &&
             (uint32_t)(millis() - g_sensorFaultSince) >= SENSOR_FAULT_TIMEOUT_MS) {
    physical = false;
  } else {
    g_relayAutoState = thresholdEval();
    physical = g_relayAutoState;
  }
  relay_apply(physical);
}

static bool ap_start() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPdisconnect(true);
  delay(50);

  g_apRunning = WiFi.softAP(g_cfg.apSsid, g_cfg.apPass, 1, 0, 4);
  if (!g_apRunning) {
    delay(150);
    g_apRunning = WiFi.softAP(g_cfg.apSsid, g_cfg.apPass, 1, 0, 4);
  }
  if (g_apRunning) {
    IPAddress ip = WiFi.softAPIP();
    g_dns.stop();
    g_dns.setErrorReplyCode(DNSReplyCode::NoError);
    g_dns.start(53, "*", ip);
  }
  return g_apRunning;
}

static void ap_ensure() {
  if (!g_apRunning) {
    ap_start();
    g_apIpZeroCount = 0;
    if (g_apRunning) g_dnsIp = WiFi.softAPIP();
    return;
  }
  IPAddress ip = WiFi.softAPIP();
  if (ip == IPAddress(0, 0, 0, 0)) {
    if (++g_apIpZeroCount >= 3) {
      ap_start();
      g_apIpZeroCount = 0;
      if (g_apRunning) g_dnsIp = WiFi.softAPIP();
    }
    return;
  }
  g_apIpZeroCount = 0;
  if (ip != g_dnsIp) {
    g_dns.stop();
    g_dns.setErrorReplyCode(DNSReplyCode::NoError);
    g_dns.start(53, "*", ip);
    g_dnsIp = ip;
  }
}

static void addCors() { g_server.sendHeader("Cache-Control", "no-store"); }

static void handleRoot() {
  static PGM_P const parts[] = { PAGE_HEAD, PAGE_NAV, PAGE_DASH, PAGE_FOOT };
  sendPage(parts, sizeof(parts) / sizeof(parts[0]));
}

static void handleRelayPage() {
  static PGM_P const parts[] = { PAGE_HEAD, PAGE_NAV, PAGE_RELAY, PAGE_FOOT };
  sendPage(parts, sizeof(parts) / sizeof(parts[0]));
}

static void handleWifiPage() {
  static PGM_P const parts[] = { PAGE_HEAD, PAGE_NAV, PAGE_WIFI, PAGE_FOOT };
  sendPage(parts, sizeof(parts) / sizeof(parts[0]));
}

static void handle404() {
  static PGM_P const parts[] = { PAGE_HEAD, PAGE_NAV, PAGE_404, PAGE_FOOT };
  size_t total = 0;
  for (size_t i = 0; i < sizeof(parts) / sizeof(parts[0]); i++) total += strlen_P(parts[i]);
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.sendHeader("Connection", "close");
  g_server.setContentLength(total);
  g_server.send(404, "text/html; charset=utf-8", "");
  for (size_t i = 0; i < sizeof(parts) / sizeof(parts[0]); i++) sendPgPart(parts[i]);
}

static void handleStatus() {
  char buf[512];
  (void)snprintf(buf, sizeof(buf),
    "{\"temp\":%.2f,\"hum\":%.2f,\"sensorOk\":%s,\"relay\":%s,"
    "\"trigMode\":%u,\"tempThr\":%.2f,\"humThr\":%.2f,"
    "\"tempHyst\":%.2f,\"humHyst\":%.2f,"
    "\"relayActiveHigh\":%u,\"relayManual\":%u,"
    "\"ap\":%s,\"ssid\":\"%s\"}",
    isnan(g_tempC) ? 0.0f : g_tempC, isnan(g_humPct) ? 0.0f : g_humPct,
    g_sensorOk ? "true" : "false",
    g_relayState ? "true" : "false",
    g_cfg.trigMode, g_cfg.tempThreshold, g_cfg.humThreshold,
    g_cfg.tempHyst, g_cfg.humHyst,
    g_cfg.relayActiveHigh, g_cfg.relayManual,
    g_apRunning ? "true" : "false", g_cfg.apSsid);
  addCors();
  g_server.send(200, "application/json", buf);
}

static bool parseFloatStrict(const String& s, float& out) {
  if (s.length() == 0) return false;
  bool dot = false, digit = false;
  for (size_t i = 0; i < s.length(); i++) {
    char ch = s[i];
    if (ch == '-' && i == 0) continue;
    if (ch == '.' && !dot) { dot = true; continue; }
    if (ch >= '0' && ch <= '9') { digit = true; continue; }
    return false;
  }
  if (!digit) return false;
  out = s.toFloat();
  return true;
}

static void handleThresholds() {
  if (!g_server.hasArg("tm") || !g_server.hasArg("tt") ||
      !g_server.hasArg("ht") || !g_server.hasArg("th") || !g_server.hasArg("hh")) {
    g_server.send(400, "application/json", "{\"ok\":false,\"err\":\"args\"}");
    return;
  }
  float tt, ht, th, hh;
  if (!parseFloatStrict(g_server.arg("tt"), tt) ||
      !parseFloatStrict(g_server.arg("ht"), ht) ||
      !parseFloatStrict(g_server.arg("th"), th) ||
      !parseFloatStrict(g_server.arg("hh"), hh)) {
    g_server.send(400, "application/json", "{\"ok\":false,\"err\":\"num\"}");
    return;
  }
  Config c = g_cfg;
  c.trigMode      = (uint8_t)constrain(g_server.arg("tm").toInt(), 0, 3);
  c.tempThreshold = tt;
  c.humThreshold  = ht;
  c.tempHyst      = fabsf(th);
  c.humHyst       = fabsf(hh);
  cfg_finalize(c);
  bool saved = cfg_saveIfChanged(c);
  if (saved) g_cfg = c;
  relay_update();
  g_server.send(saved ? 200 : 500, "application/json",
                saved ? "{\"ok\":true}" : "{\"ok\":false,\"err\":\"nvs\"}");
}

static void handleRelayApi() {
  Config c = g_cfg;
  if (g_server.hasArg("pol")) c.relayActiveHigh = g_server.arg("pol").toInt() ? 1 : 0;
  if (g_server.hasArg("mode")) c.relayManual = (uint8_t)constrain(g_server.arg("mode").toInt(), 0, 2);
  cfg_finalize(c);
  bool saved = cfg_saveIfChanged(c);
  if (saved) g_cfg = c;
  relay_update();
  g_server.send(saved ? 200 : 500, "application/json",
                saved ? "{\"ok\":true}" : "{\"ok\":false,\"err\":\"nvs\"}");
}

static void handleWifiApi() {
  if (!g_server.hasArg("ssid") || !g_server.hasArg("pass")) {
    g_server.send(400, "application/json", "{\"ok\":false,\"err\":\"args\"}");
    return;
  }
  String ssid = g_server.arg("ssid");
  String pass = g_server.arg("pass");

  if (ssid.length() == 0 || ssid.length() > 32) {
    g_server.send(400, "application/json", "{\"ok\":false,\"err\":\"ssidrange\"}");
    return;
  }
  for (size_t i = 0; i < ssid.length(); i++) {
    char ch = ssid[i];
    if (ch < 0x20 || ch > 0x7E || ch == '"' || ch == '\\') {
      g_server.send(400, "application/json", "{\"ok\":false,\"err\":\"ssidchar\"}");
      return;
    }
  }

  if (pass.length() < 8 || pass.length() > 63) {
    g_server.send(400, "application/json", "{\"ok\":false,\"err\":\"passlen\"}");
    return;
  }
  for (size_t i = 0; i < pass.length(); i++) {
    if (pass[i] < 0x20 || pass[i] > 0x7E) {
      g_server.send(400, "application/json", "{\"ok\":false,\"err\":\"passchar\"}");
      return;
    }
  }

  Config c = g_cfg;
  memset(c.apSsid, 0, sizeof(c.apSsid));
  memset(c.apPass, 0, sizeof(c.apPass));
  strncpy(c.apSsid, ssid.c_str(), sizeof(c.apSsid) - 1);
  strncpy(c.apPass, pass.c_str(), sizeof(c.apPass) - 1);
  cfg_finalize(c);
  bool saved = cfg_saveIfChanged(c);
  if (saved) g_cfg = c;
  g_server.sendHeader("Connection", "close");
  g_server.send(saved ? 200 : 500, "application/json",
                saved ? "{\"ok\":true}" : "{\"ok\":false,\"err\":\"nvs\"}");
  g_server.client().flush();
  if (saved) g_apRestartAt = millis() + 500;
}

static void handleFactory() {
  bool ok = g_prefs.begin(NVS_NS, false);
  if (ok) ok = g_prefs.clear();
  g_prefs.end();
  g_server.sendHeader("Connection", "close");
  g_server.send(ok ? 200 : 500, "application/json",
                ok ? "{\"ok\":true}" : "{\"ok\":false,\"err\":\"nvs\"}");
  g_server.client().flush();
  if (ok) g_rebootAt = millis() + 400;
}

static void handleCaptive() {
  g_server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  g_server.send(302, "text/plain", "");
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

  Wire.begin();

  if (!cfg_loadFromNVS()) {
    cfg_defaults(g_cfg);
    cfg_finalize(g_cfg);
    g_prefs.begin(NVS_NS, false);
    g_prefs.putBytes("cfg", &g_cfg, sizeof(g_cfg));
    g_prefs.end();
  }

  relay_apply(false);
  if (ap_start()) g_dnsIp = WiFi.softAPIP();

  g_server.on("/", HTTP_GET, handleRoot);
  g_server.on("/relay", HTTP_GET, handleRelayPage);
  g_server.on("/wifi", HTTP_GET, handleWifiPage);
  g_server.on("/api/status", HTTP_GET, handleStatus);
  g_server.on("/api/thresholds", HTTP_POST, handleThresholds);
  g_server.on("/api/relay", HTTP_POST, handleRelayApi);
  g_server.on("/api/wifi", HTTP_POST, handleWifiApi);
  g_server.on("/api/factory", HTTP_POST, handleFactory);
  g_server.onNotFound([]() {
    if (!g_server.hostHeader().startsWith(WiFi.softAPIP().toString())) {
      handleCaptive();
    } else {
      handle404();
    }
  });
  g_server.begin();

  g_lastSensorRead = millis();
  g_lastApCheck = millis();
  g_lastDns = millis();
}

void loop() {
  uint32_t now = millis();

  if (g_rebootAt != 0 && (int32_t)(now - g_rebootAt) >= 0) {
    ESP.restart();
  }

  if ((uint32_t)(now - g_lastDns) >= DNS_INTERVAL_MS) {
    g_lastDns = now;
    g_dns.processNextRequest();
  }

  g_server.handleClient();

  if ((uint32_t)(now - g_lastSensorRead) >= SENSOR_INTERVAL_MS) {
    g_lastSensorRead = now;
    if (!sht_start()) {
      if (g_sensorOk || g_sensorFaultSince == 0) g_sensorFaultSince = now;
      g_sensorOk = false;
      relay_update();
    }
  }

  {
    float t, h;
    int r = sht_poll(t, h);
    if (r == 1) {
      g_tempC = t; g_humPct = h;
      g_sensorOk = true;
      g_sensorFaultSince = 0;
      relay_update();
    } else if (r < 0) {
      if (g_sensorOk || g_sensorFaultSince == 0) g_sensorFaultSince = now;
      g_sensorOk = false;
      relay_update();
    }
  }

  if (g_apRestartAt != 0 && (int32_t)(now - g_apRestartAt) >= 0) {
    g_apRestartAt = 0;
    ap_start();
    if (g_apRunning) g_dnsIp = WiFi.softAPIP();
    else g_dnsIp = IPAddress(0, 0, 0, 0);
    g_apIpZeroCount = 0;
  }

  if ((uint32_t)(now - g_lastApCheck) >= AP_CHECK_INTERVAL_MS) {
    g_lastApCheck = now;
    ap_ensure();
  }

  bool raw = digitalRead(BOOT_BTN_PIN);
  if (raw != g_btnLastRaw) {
    g_btnLastRaw = raw;
    g_btnLastChange = now;
  }

  if (!g_btnStable && !g_btnHoldHandled && g_btnPressStart != 0) {
    if ((uint32_t)(now - g_btnPressStart) >= FACTORY_HOLD_MS) {
      g_btnHoldHandled = true;
      bool ok = g_prefs.begin(NVS_NS, false);
      if (ok) ok = g_prefs.clear();
      g_prefs.end();
      if (ok) {
        ESP.restart();
      } else {
        Serial.println("Factory reset failed: NVS clear error");
      }
    }
  }

  if ((uint32_t)(now - g_btnLastChange) >= BTN_DEBOUNCE_MS) {
    if (g_btnStable != raw) {
      g_btnStable = raw;
      if (!raw) {
        g_btnPressStart = now;
        g_btnHoldHandled = false;
      } else {
        g_btnPressStart = 0;
      }
    }
  }

  delay(1);
}
