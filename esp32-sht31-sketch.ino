
/*
* ESP32 SHT31 Industrial Climate Controller & Relay Automation Engine
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Adafruit_SHT31.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>

/* Pin Assignments */
static const uint8_t PIN_I2C_SDA_GPIO                 = 21;
static const uint8_t PIN_I2C_SCL_GPIO                 = 22;
static const uint8_t PIN_RELAY_UNIFIED_OR_TEMP_GPIO   = 25;
static const uint8_t PIN_RELAY_HUM_GPIO               = 26;
static const uint8_t PIN_BOOT_BUTTON_GPIO             = 0;

/* Configuration Constants */
static const uint64_t CONFIG_MAGIC_U64                = 0x534854333150524FULL; /* 'SHT31PRO' */
static const uint64_t CONFIG_VERSION_U64              = 1ULL;

/* Relay Modes */
static const uint64_t RELAY_MODE_UNIFIED              = 0ULL;
static const uint64_t RELAY_MODE_INDIVIDUAL           = 1ULL;

/* Unified Relay Logic Rules */
static const uint64_t LOGIC_OR                        = 0ULL;
static const uint64_t LOGIC_AND                       = 1ULL;
static const uint64_t LOGIC_TEMP_ONLY                 = 2ULL;
static const uint64_t LOGIC_HUM_ONLY                  = 3ULL;

/* Trigger Directions */
static const uint64_t TRIGGER_ABOVE_HIGH              = 0ULL; /* Cooling or Dehumidify */
static const uint64_t TRIGGER_BELOW_LOW               = 1ULL; /* Heating or Humidify */

/* Electrical Relay Polarity */
static const uint64_t POLARITY_ACTIVE_HIGH            = 0ULL;
static const uint64_t POLARITY_ACTIVE_LOW             = 1ULL;

/* Fail-Safe Modes upon Sensor Failure */
static const uint64_t FAILSAFE_OFF                    = 0ULL;
static const uint64_t FAILSAFE_ON                     = 1ULL;
static const uint64_t FAILSAFE_HOLD                   = 2ULL;

/* SHT31 Sensor Command IDs */
static const uint16_t SHT31_CMD_MEAS_HIGHREP_NOSTRETCH = 0x2400;
static const uint16_t SHT31_CMD_READ_STATUS            = 0xF32D;
static const uint16_t SHT31_CMD_CLEAR_STATUS           = 0x3041;
static const uint16_t SHT31_CMD_HEATER_ENABLE          = 0x306D;
static const uint16_t SHT31_CMD_HEATER_DISABLE         = 0x3066;

/* Configuration Structure with 64-bit alignment and packed layout */
struct DeviceConfig {
  uint64_t magic;
  uint64_t version;
  char     ap_ssid[32];
  char     ap_password[64];
  uint64_t ap_channel;
  uint64_t ap_hidden;
  uint64_t ap_max_conn;
  uint64_t relay_mode;
  uint64_t unified_logic;
  uint64_t relay_polarity;
  uint64_t failsafe_mode;
  double   temp_threshold_high;
  double   temp_threshold_low;
  uint64_t temp_trigger_mode;
  double   hum_threshold_high;
  double   hum_threshold_low;
  uint64_t hum_trigger_mode;
  uint64_t poll_interval_ms;
  uint64_t min_switch_interval_ms;
  double   temp_offset;
  double   hum_offset;
  uint64_t heater_enabled;
  uint64_t crc64;
};

/* Sensor Live Operational State */
struct SensorRuntimeState {
  double   current_temp_c;
  double   current_hum_rh;
  double   current_dew_point_c;
  double   current_heat_index_c;
  double   min_temp_c;
  double   max_temp_c;
  double   min_hum_rh;
  double   max_hum_rh;
  bool     sensor_is_online;
  bool     heater_active;
  bool     temp_trigger_active;
  bool     hum_trigger_active;
  bool     relay25_energized;
  bool     relay26_energized;
};

/* Diagnostics & Multi-Year Operational Telemetry */
struct DiagnosticsCounters {
  uint64_t total_sensor_polls_u64;
  uint64_t total_sensor_errors_u64;
  uint64_t total_i2c_bus_recoveries_u64;
  uint64_t total_nvs_writes_u64;
  uint64_t total_relay25_switch_cycles_u64;
  uint64_t total_relay26_switch_cycles_u64;
  uint64_t total_http_requests_served_u64;
  uint64_t total_dns_queries_served_u64;
  uint64_t total_factory_resets_u64;
};

/* Sensor Measurement State Machine */
enum SensorReadState {
  SENSOR_STATE_IDLE,
  SENSOR_STATE_CONVERTING
};

/* Global Objects & State */
static DeviceConfig         device_config;
static SensorRuntimeState   runtime_state;
static DiagnosticsCounters  diag_counters;

static Adafruit_SHT31       sht31_driver(&Wire);
static WebServer            web_server(80);
static DNSServer            dns_server;
static Preferences          nv_storage;

static const IPAddress      AP_STATIC_IP(192, 168, 4, 1);
static const IPAddress      AP_SUBNET_MASK(255, 255, 255, 0);

static uint8_t              sht31_i2c_address             = 0x44;
static SensorReadState      sensor_engine_state           = SENSOR_STATE_IDLE;
static int64_t              sensor_request_start_us_i64   = 0;
static uint64_t             last_sensor_poll_ms_u64       = 0;
static uint64_t             consecutive_sensor_errors_u64 = 0;

static uint64_t             last_button_sample_ms_u64     = 0;
static uint64_t             button_press_start_ms_u64     = 0;

static uint64_t             last_relay25_switch_ms_u64    = 0;
static uint64_t             last_relay26_switch_ms_u64    = 0;

static bool                 pending_system_restart        = false;
static uint64_t             restart_scheduled_ms_u64      = 0;

/* Embedded Dashboard Web Application */
static const char INDEX_PAGE_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>ESP32 SHT31 Industrial Climate Controller</title>
  <style>
    :root {
      --bg: #090d16;
      --card-bg: #131b2e;
      --card-border: rgba(255, 255, 255, 0.08);
      --card-hover: rgba(255, 255, 255, 0.12);
      --text: #f1f5f9;
      --text-muted: #94a3b8;
      --primary: #06b6d4;
      --primary-glow: rgba(6, 182, 212, 0.25);
      --temp: #f97316;
      --hum: #10b981;
      --relay-on: #10b981;
      --relay-off: #475569;
      --danger: #ef4444;
      --warning: #f59e0b;
      --radius: 12px;
      --font: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    }

    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: var(--bg);
      color: var(--text);
      font-family: var(--font);
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      -webkit-font-smoothing: antialiased;
    }

    header {
      background: rgba(19, 27, 46, 0.85);
      backdrop-filter: blur(12px);
      -webkit-backdrop-filter: blur(12px);
      border-bottom: 1px solid var(--card-border);
      position: sticky;
      top: 0;
      z-index: 100;
      padding: 12px 20px;
    }

    .header-content {
      max-width: 1200px;
      margin: 0 auto;
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 16px;
    }

    .logo-group {
      display: flex;
      align-items: center;
      gap: 12px;
    }

    .logo-icon {
      width: 38px;
      height: 38px;
      background: linear-gradient(135deg, #06b6d4, #3b82f6);
      border-radius: 10px;
      display: flex;
      align-items: center;
      justify-content: center;
      box-shadow: 0 0 15px var(--primary-glow);
    }

    .logo-title h1 {
      font-size: 1.15rem;
      font-weight: 700;
      letter-spacing: -0.02em;
      color: #ffffff;
    }

    .logo-title p {
      font-size: 0.72rem;
      color: var(--text-muted);
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }

    .status-badges {
      display: flex;
      align-items: center;
      gap: 8px;
      flex-wrap: wrap;
    }

    .badge {
      font-size: 0.75rem;
      font-weight: 600;
      padding: 4px 10px;
      border-radius: 20px;
      display: inline-flex;
      align-items: center;
      gap: 6px;
      border: 1px solid var(--card-border);
    }

    .badge-dot {
      width: 7px;
      height: 7px;
      border-radius: 50%;
      background: currentColor;
    }

    .badge-green { background: rgba(16, 185, 129, 0.15); color: #34d399; border-color: rgba(16, 185, 129, 0.3); }
    .badge-cyan { background: rgba(6, 182, 212, 0.15); color: #38bdf8; border-color: rgba(6, 182, 212, 0.3); }
    .badge-red { background: rgba(239, 68, 68, 0.15); color: #f87171; border-color: rgba(239, 68, 68, 0.3); }

    .nav-container {
      background: #0f1523;
      border-bottom: 1px solid var(--card-border);
    }

    .nav-inner {
      max-width: 1200px;
      margin: 0 auto;
      display: flex;
      overflow-x: auto;
      scrollbar-width: none;
    }
    .nav-inner::-webkit-scrollbar { display: none; }

    .nav-tab {
      background: none;
      border: none;
      color: var(--text-muted);
      padding: 14px 20px;
      font-size: 0.88rem;
      font-weight: 600;
      cursor: pointer;
      display: flex;
      align-items: center;
      gap: 8px;
      white-space: nowrap;
      position: relative;
      transition: color 0.2s;
    }

    .nav-tab:hover { color: var(--text); }
    .nav-tab.active {
      color: var(--primary);
    }

    .nav-tab.active::after {
      content: '';
      position: absolute;
      bottom: 0;
      left: 0;
      right: 0;
      height: 2px;
      background: var(--primary);
      box-shadow: 0 0 10px var(--primary);
    }

    main {
      max-width: 1200px;
      margin: 0 auto;
      padding: 24px 20px;
      width: 100%;
      flex: 1;
    }

    .tab-pane { display: none; }
    .tab-pane.active { display: block; animation: fadeIn 0.25s ease; }

    @keyframes fadeIn {
      from { opacity: 0; transform: translateY(4px); }
      to { opacity: 1; transform: translateY(0); }
    }

    .grid-4 {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
      gap: 18px;
      margin-bottom: 24px;
    }

    .grid-2 {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(360px, 1fr));
      gap: 20px;
      margin-bottom: 24px;
    }

    .card {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: var(--radius);
      padding: 20px;
      position: relative;
      overflow: hidden;
      box-shadow: 0 4px 20px rgba(0, 0, 0, 0.25);
    }

    .card-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 12px;
    }

    .card-title {
      font-size: 0.82rem;
      text-transform: uppercase;
      letter-spacing: 0.06em;
      color: var(--text-muted);
      font-weight: 700;
      display: flex;
      align-items: center;
      gap: 8px;
    }

    .card-value {
      font-size: 2.8rem;
      font-weight: 800;
      line-height: 1.1;
      letter-spacing: -0.03em;
      margin-bottom: 8px;
    }

    .card-footer {
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 0.78rem;
      color: var(--text-muted);
      border-top: 1px solid var(--card-border);
      padding-top: 10px;
      margin-top: 10px;
    }

    .val-temp { color: var(--temp); }
    .val-hum { color: var(--hum); }
    .val-dew { color: var(--primary); }
    .val-heat { color: #a855f7; }

    .relay-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      gap: 18px;
      margin-bottom: 24px;
    }

    .relay-card {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: var(--radius);
      padding: 20px;
      display: flex;
      flex-direction: column;
      justify-content: space-between;
      position: relative;
      transition: border-color 0.2s;
    }

    .relay-card.active-state {
      border-color: rgba(16, 185, 129, 0.5);
      background: linear-gradient(180deg, rgba(16, 185, 129, 0.08), var(--card-bg));
    }

    .relay-status-pill {
      font-size: 0.82rem;
      font-weight: 700;
      padding: 6px 14px;
      border-radius: 20px;
      display: inline-flex;
      align-items: center;
      gap: 6px;
    }

    .relay-on {
      background: rgba(16, 185, 129, 0.2);
      color: #34d399;
      border: 1px solid rgba(16, 185, 129, 0.4);
      box-shadow: 0 0 12px rgba(16, 185, 129, 0.25);
    }

    .relay-off {
      background: rgba(100, 116, 139, 0.2);
      color: #94a3b8;
      border: 1px solid rgba(100, 116, 139, 0.3);
    }

    .form-section {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: var(--radius);
      padding: 24px;
      margin-bottom: 24px;
    }

    .section-title {
      font-size: 1.1rem;
      font-weight: 700;
      margin-bottom: 6px;
      display: flex;
      align-items: center;
      gap: 10px;
      color: #ffffff;
    }

    .section-desc {
      font-size: 0.85rem;
      color: var(--text-muted);
      margin-bottom: 20px;
      line-height: 1.4;
    }

    .form-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 18px;
    }

    .form-group {
      display: flex;
      flex-direction: column;
      gap: 6px;
    }

    label {
      font-size: 0.82rem;
      font-weight: 600;
      color: var(--text-muted);
    }

    input[type="text"],
    input[type="password"],
    input[type="number"],
    select {
      background: #0b111e;
      border: 1px solid var(--card-border);
      color: #ffffff;
      padding: 10px 14px;
      border-radius: 8px;
      font-size: 0.92rem;
      font-family: inherit;
      outline: none;
      transition: border-color 0.2s, box-shadow 0.2s;
    }

    input:focus, select:focus {
      border-color: var(--primary);
      box-shadow: 0 0 0 2px var(--primary-glow);
    }

    .segmented-control {
      display: flex;
      background: #0b111e;
      border: 1px solid var(--card-border);
      border-radius: 8px;
      padding: 4px;
      gap: 4px;
    }

    .segmented-btn {
      flex: 1;
      padding: 8px 12px;
      background: none;
      border: none;
      color: var(--text-muted);
      font-size: 0.82rem;
      font-weight: 600;
      border-radius: 6px;
      cursor: pointer;
      transition: all 0.2s;
      text-align: center;
    }

    .segmented-btn.active {
      background: var(--primary);
      color: #0b111e;
      font-weight: 700;
    }

    .btn {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      padding: 10px 20px;
      border-radius: 8px;
      font-size: 0.9rem;
      font-weight: 600;
      cursor: pointer;
      border: none;
      transition: all 0.2s;
    }

    .btn-primary {
      background: linear-gradient(135deg, #06b6d4, #0284c7);
      color: #ffffff;
      box-shadow: 0 2px 10px var(--primary-glow);
    }
    .btn-primary:hover { opacity: 0.92; }

    .btn-danger {
      background: rgba(239, 68, 68, 0.2);
      color: #f87171;
      border: 1px solid rgba(239, 68, 68, 0.4);
    }
    .btn-danger:hover {
      background: rgba(239, 68, 68, 0.3);
    }

    .btn-secondary {
      background: #1e293b;
      color: var(--text);
      border: 1px solid var(--card-border);
    }
    .btn-secondary:hover { background: #334155; }

    .stats-table {
      width: 100%;
      border-collapse: collapse;
      font-size: 0.88rem;
    }

    .stats-table tr {
      border-bottom: 1px solid var(--card-border);
    }

    .stats-table td {
      padding: 12px 6px;
    }

    .stats-table td:last-child {
      text-align: right;
      font-weight: 600;
      font-family: monospace;
      color: #38bdf8;
    }

    .toast {
      position: fixed;
      bottom: 24px;
      right: 24px;
      background: #1e293b;
      color: #ffffff;
      padding: 12px 20px;
      border-radius: 8px;
      border-left: 4px solid var(--primary);
      box-shadow: 0 10px 25px rgba(0, 0, 0, 0.5);
      z-index: 1000;
      display: none;
      font-size: 0.88rem;
      align-items: center;
      gap: 10px;
      animation: slideUp 0.3s ease;
    }

    @keyframes slideUp {
      from { transform: translateY(20px); opacity: 0; }
      to { transform: translateY(0); opacity: 1; }
    }

    @media (max-width: 640px) {
      .header-content { flex-direction: column; align-items: flex-start; }
      .status-badges { width: 100%; justify-content: flex-start; }
      .card-value { font-size: 2.2rem; }
      .nav-tab { padding: 12px 14px; font-size: 0.82rem; }
      main { padding: 16px 12px; }
      .form-section { padding: 18px 14px; }
    }
  </style>
</head>
<body>

  <header>
    <div class="header-content">
      <div class="logo-group">
        <div class="logo-icon">
          <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
            <path d="M14 14.76V3.5a2.5 2.5 0 0 0-5 0v11.26a4.5 4.5 0 1 0 5 0z"></path>
          </svg>
        </div>
        <div class="logo-title">
          <h1>ESP32 SHT31 Controller</h1>
          <p>Industrial Climate & Relay Automation</p>
        </div>
      </div>
      <div class="status-badges">
        <span id="badge-sensor" class="badge badge-green">
          <span class="badge-dot"></span><span id="badge-sensor-text">SHT31 ONLINE</span>
        </span>
        <span id="badge-mode" class="badge badge-cyan">
          <span id="badge-mode-text">UNIFIED RELAY</span>
        </span>
        <span id="badge-poll" class="badge" style="background: rgba(255,255,255,0.06); color: var(--text-muted);">
          <span class="badge-dot" style="background:#06b6d4;"></span><span id="badge-poll-text">SYNC</span>
        </span>
      </div>
    </div>
  </header>

  <nav class="nav-container">
    <div class="nav-inner">
      <button class="nav-tab active" onclick="switchTab('dashboard')">
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="7" height="9"></rect><rect x="14" y="3" width="7" height="5"></rect><rect x="14" y="12" width="7" height="9"></rect><rect x="3" y="16" width="7" height="5"></rect></svg>
        Dashboard
      </button>
      <button class="nav-tab" onclick="switchTab('sensor_settings')">
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="3"></circle><path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z"></path></svg>
        Sensor & Relays
      </button>
      <button class="nav-tab" onclick="switchTab('wifi_settings')">
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M5 12.55a11 11 0 0 1 14.08 0"></path><path d="M1.42 9a16 16 0 0 1 21.16 0"></path><path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path><line x1="12" y1="20" x2="12.01" y2="20"></line></svg>
        WiFi AP Settings
      </button>
      <button class="nav-tab" onclick="switchTab('system_diagnostics')">
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="22 12 18 12 15 21 9 3 6 12 2 12"></polyline></svg>
        Diagnostics
      </button>
    </div>
  </nav>

  <main>
    
    <div id="tab-dashboard" class="tab-pane active">
      <div class="grid-4">
        
        <div class="card">
          <div class="card-header">
            <span class="card-title">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="var(--temp)" stroke-width="2.5"><path d="M14 14.76V3.5a2.5 2.5 0 0 0-5 0v11.26a4.5 4.5 0 1 0 5 0z"></path></svg>
              Temperature
            </span>
            <button class="btn btn-secondary" style="padding: 2px 8px; font-size: 0.72rem;" onclick="toggleUnit()">
              <span id="unit-label">°C / °F</span>
            </button>
          </div>
          <div class="card-value val-temp"><span id="val-temp">--.-</span><span style="font-size: 1.5rem; font-weight: 500;" id="val-temp-unit">°C</span></div>
          <div class="card-footer">
            <span>Min: <b id="val-temp-min">--</b> | Max: <b id="val-temp-max">--</b></span>
            <span id="val-temp-status" class="badge badge-green" style="font-size: 0.7rem; padding: 2px 8px;">NOMINAL</span>
          </div>
        </div>

        
        <div class="card">
          <div class="card-header">
            <span class="card-title">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="var(--hum)" stroke-width="2.5"><path d="M12 2.69l5.66 5.66a8 8 0 1 1-11.31 0z"></path></svg>
              Relative Humidity
            </span>
            <span id="val-hum-comfort" class="badge badge-green" style="font-size: 0.7rem; padding: 2px 8px;">OPTIMAL</span>
          </div>
          <div class="card-value val-hum"><span id="val-hum">--.-</span><span style="font-size: 1.5rem; font-weight: 500;">%</span></div>
          <div class="card-footer">
            <span>Min: <b id="val-hum-min">--</b> | Max: <b id="val-hum-max">--</b></span>
            <span>RH Range: <b id="val-hum-range">60 - 70%</b></span>
          </div>
        </div>

        
        <div class="card">
          <div class="card-header">
            <span class="card-title">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="var(--primary)" stroke-width="2.5"><circle cx="12" cy="12" r="10"></circle><path d="M8 12h8"></path></svg>
              Dew Point
            </span>
            <span id="val-dew-risk" class="badge badge-cyan" style="font-size: 0.7rem; padding: 2px 8px;">DRY</span>
          </div>
          <div class="card-value val-dew"><span id="val-dew">--.-</span><span style="font-size: 1.5rem; font-weight: 500;" id="val-dew-unit">°C</span></div>
          <div class="card-footer">
            <span>Condensation Risk</span>
            <span id="val-dew-status">SAFE</span>
          </div>
        </div>

        
        <div class="card">
          <div class="card-header">
            <span class="card-title">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="#a855f7" stroke-width="2.5"><path d="M8.5 14.5A2.5 2.5 0 0 0 11 12c0-1.38-.5-2-1-3-1.072-2.143-.224-4.054 2-6 .5 2.5 2 4.9 4 6.5 2 1.6 3 3.5 3 5.5a7 7 0 1 1-14 0c0-1.153.433-2.294 1-3a2.5 2.5 0 0 0 2.5 2.5z"></path></svg>
              Apparent Temp
            </span>
            <span class="badge" style="background: rgba(168,85,247,0.15); color: #c084fc;">HEAT INDEX</span>
          </div>
          <div class="card-value val-heat"><span id="val-heat">--.-</span><span style="font-size: 1.5rem; font-weight: 500;" id="val-heat-unit">°C</span></div>
          <div class="card-footer">
            <span>Thermal Comfort</span>
            <span id="val-heat-status">COMFORTABLE</span>
          </div>
        </div>
      </div>

      
      <h2 style="font-size: 1.05rem; font-weight: 700; margin-bottom: 14px; color: #fff; display: flex; align-items: center; gap: 8px;">
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M13 2L3 14h9l-1 8 10-12h-9l1-8z"></path></svg>
        Relay Output Channels
      </h2>
      <div class="relay-grid">
        
        <div id="card-relay25" class="relay-card">
          <div>
            <div style="display: flex; justify-content: space-between; align-items: flex-start; margin-bottom: 14px;">
              <div>
                <h3 style="font-size: 1.1rem; font-weight: 700; color: #fff;" id="relay25-title">GPIO 25 (Unified Relay)</h3>
                <p style="font-size: 0.8rem; color: var(--text-muted); margin-top: 2px;" id="relay25-sub">Triggers on Temperature OR Humidity</p>
              </div>
              <span id="pill-relay25" class="relay-status-pill relay-off">OFF</span>
            </div>
            <div style="font-size: 0.85rem; color: var(--text-muted); line-height: 1.6;">
              <div>Trigger Status: <b id="relay25-reason" style="color: #fff;">Idle</b></div>
              <div>Operating Polarity: <b id="relay25-polarity" style="color: #38bdf8;">Active Low</b></div>
            </div>
          </div>
          <div class="card-footer" style="margin-top: 16px;">
            <span>Cycles: <b id="relay25-cycles">0</b></span>
            <span>Pin: <b>ESP32 GPIO 25</b></span>
          </div>
        </div>

        
        <div id="card-relay26" class="relay-card">
          <div>
            <div style="display: flex; justify-content: space-between; align-items: flex-start; margin-bottom: 14px;">
              <div>
                <h3 style="font-size: 1.1rem; font-weight: 700; color: #fff;" id="relay26-title">GPIO 26 (Humidity Relay)</h3>
                <p style="font-size: 0.8rem; color: var(--text-muted); margin-top: 2px;" id="relay26-sub">Dedicated Humidity Channel</p>
              </div>
              <span id="pill-relay26" class="relay-status-pill relay-off">OFF</span>
            </div>
            <div style="font-size: 0.85rem; color: var(--text-muted); line-height: 1.6;">
              <div>Trigger Status: <b id="relay26-reason" style="color: #fff;">Idle</b></div>
              <div>Mode State: <b id="relay26-mode-state" style="color: #94a3b8;">Standby in Unified Mode</b></div>
            </div>
          </div>
          <div class="card-footer" style="margin-top: 16px;">
            <span>Cycles: <b id="relay26-cycles">0</b></span>
            <span>Pin: <b>ESP32 GPIO 26</b></span>
          </div>
        </div>
      </div>

      
      <div style="display: flex; gap: 12px; flex-wrap: wrap; margin-top: 10px;">
        <button id="btn-heater" class="btn btn-secondary" onclick="toggleHeater()">
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 2v20M17 5H9.5a3.5 3.5 0 0 0 0 7h5a3.5 3.5 0 0 1 0 7H6"></path></svg>
          <span id="heater-label">SHT31 Heater: OFF</span>
        </button>
        <button class="btn btn-secondary" onclick="resetStats()">
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="1 4 1 10 7 10"></polyline><path d="M3.51 15a9 9 0 1 0 2.13-9.36L1 10"></path></svg>
          Reset Min / Max Records
        </button>
      </div>
    </div>

    
    <div id="tab-sensor_settings" class="tab-pane">
      <form id="form-sensor" onsubmit="saveSensorSettings(event)">
        
        <div class="form-section">
          <div class="section-title">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="var(--primary)" stroke-width="2"><rect x="2" y="7" width="20" height="14" rx="2" ry="2"></rect><path d="M16 21V5a2 2 0 0 0-2-2h-4a2 2 0 0 0-2 2v16"></path></svg>
            Relay Operating Architecture
          </div>
          <p class="section-desc">Select whether GPIO 25 acts as a unified relay for both sensors or if channels are split individually (GPIO 25 for Temperature and GPIO 26 for Humidity).</p>
          
          <div class="form-grid">
            <div class="form-group">
              <label>Operating Mode</label>
              <div class="segmented-control">
                <button type="button" id="mode-unified-btn" class="segmented-btn active" onclick="setRelayModeUI(0)">Unified Relay (GPIO 25)</button>
                <button type="button" id="mode-individual-btn" class="segmented-btn" onclick="setRelayModeUI(1)">Individual Relays (GPIO 25 & 26)</button>
              </div>
              <input type="hidden" id="cfg-relay-mode" value="0">
            </div>

            <div class="form-group" id="grp-unified-logic">
              <label>Unified Trigger Combination Logic</label>
              <select id="cfg-unified-logic">
                <option value="0">OR Logic (Trigger if EITHER Temp OR Hum triggers)</option>
                <option value="1">AND Logic (Trigger ONLY if BOTH Temp AND Hum trigger)</option>
                <option value="2">Temperature Only (Ignore humidity for unified relay)</option>
                <option value="3">Humidity Only (Ignore temperature for unified relay)</option>
              </select>
            </div>
          </div>
        </div>

        
        <div class="form-section">
          <div class="section-title">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="var(--temp)" stroke-width="2"><path d="M14 14.76V3.5a2.5 2.5 0 0 0-5 0v11.26a4.5 4.5 0 1 0 5 0z"></path></svg>
            Temperature Trigger Thresholds & Hysteresis
          </div>
          <p class="section-desc">Configure upper and lower temperature limits. Dual-threshold hysteresis prevents rapid relay fluttering.</p>

          <div class="form-grid">
            <div class="form-group">
              <label>High Threshold (°C)</label>
              <input type="number" step="0.1" id="cfg-temp-high" required>
            </div>
            <div class="form-group">
              <label>Low Threshold (°C)</label>
              <input type="number" step="0.1" id="cfg-temp-low" required>
            </div>
            <div class="form-group">
              <label>Temperature Action Mode</label>
              <select id="cfg-temp-mode">
                <option value="0">Cooling Mode (ON when Temp &gt;= High, OFF when Temp &lt;= Low)</option>
                <option value="1">Heating Mode (ON when Temp &lt;= Low, OFF when Temp &gt;= High)</option>
              </select>
            </div>
          </div>
        </div>

        
        <div class="form-section">
          <div class="section-title">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="var(--hum)" stroke-width="2"><path d="M12 2.69l5.66 5.66a8 8 0 1 1-11.31 0z"></path></svg>
            Humidity Trigger Thresholds & Hysteresis
          </div>
          <p class="section-desc">Configure upper and lower relative humidity limits for dehumidification or humidification control.</p>

          <div class="form-grid">
            <div class="form-group">
              <label>High Threshold (% RH)</label>
              <input type="number" step="0.1" min="0" max="100" id="cfg-hum-high" required>
            </div>
            <div class="form-group">
              <label>Low Threshold (% RH)</label>
              <input type="number" step="0.1" min="0" max="100" id="cfg-hum-low" required>
            </div>
            <div class="form-group">
              <label>Humidity Action Mode</label>
              <select id="cfg-hum-mode">
                <option value="0">Dehumidify Mode (ON when Hum &gt;= High, OFF when Hum &lt;= Low)</option>
                <option value="1">Humidify Mode (ON when Hum &lt;= Low, OFF when Hum &gt;= High)</option>
              </select>
            </div>
          </div>
        </div>

        
        <div class="form-section">
          <div class="section-title">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#f59e0b" stroke-width="2"><path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"></path></svg>
            Electrical, Anti-Chatter & Fail-Safe Protection
          </div>
          <p class="section-desc">Hardware polarity configuration and mechanical relay cycle guards for multi-year longevity.</p>

          <div class="form-grid">
            <div class="form-group">
              <label>Relay Output Polarity</label>
              <select id="cfg-polarity">
                <option value="1">Active Low (Standard 5V Relay Modules - LOW = Energized)</option>
                <option value="0">Active High (Direct NPN / SSR - HIGH = Energized)</option>
              </select>
            </div>
            <div class="form-group">
              <label>Anti-Chatter Minimum Switch Interval (seconds)</label>
              <input type="number" id="cfg-anti-chatter" min="1" max="300" required>
            </div>
            <div class="form-group">
              <label>Sensor Disconnect Fail-Safe State</label>
              <select id="cfg-failsafe">
                <option value="0">Force Relays OFF (Safe Shutdown)</option>
                <option value="1">Force Relays ON (Emergency Heating/Cooling)</option>
                <option value="2">Hold Last Known State</option>
              </select>
            </div>
          </div>
        </div>

        
        <div class="form-section">
          <div class="section-title">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#a855f7" stroke-width="2"><circle cx="12" cy="12" r="10"></circle><line x1="12" y1="8" x2="12" y2="12"></line><line x1="12" y1="16" x2="12.01" y2="16"></line></svg>
            Sensor Calibration & Polling Rate
          </div>
          <div class="form-grid">
            <div class="form-group">
              <label>Temperature Offset (°C)</label>
              <input type="number" step="0.05" id="cfg-temp-offset" required>
            </div>
            <div class="form-group">
              <label>Humidity Offset (% RH)</label>
              <input type="number" step="0.1" id="cfg-hum-offset" required>
            </div>
            <div class="form-group">
              <label>Sampling Polling Rate (ms)</label>
              <input type="number" id="cfg-poll-ms" min="500" max="60000" step="100" required>
            </div>
          </div>
        </div>

        <button type="submit" class="btn btn-primary" style="padding: 14px 28px; font-size: 1rem;">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"></path><polyline points="17 21 17 13 7 13 7 21"></polyline><polyline points="7 3 7 8 15 8"></polyline></svg>
          Save Sensor & Relay Configuration
        </button>
      </form>
    </div>

    
    <div id="tab-wifi_settings" class="tab-pane">
      <form id="form-wifi" onsubmit="saveWifiSettings(event)">
        <div class="form-section">
          <div class="section-title">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="var(--primary)" stroke-width="2"><path d="M5 12.55a11 11 0 0 1 14.08 0"></path><path d="M1.42 9a16 16 0 0 1 21.16 0"></path><path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path><line x1="12" y1="20" x2="12.01" y2="20"></line></svg>
            WiFi SoftAP Configuration
          </div>
          <p class="section-desc">Broadcasts a local wireless access point. Any device connecting to this AP will be redirected by the captive portal to <b>192.168.4.1</b> automatically.</p>

          <div class="form-grid">
            <div class="form-group">
              <label>Access Point SSID (Network Name)</label>
              <input type="text" id="cfg-ap-ssid" maxlength="31" required>
            </div>
            <div class="form-group">
              <label>WPA2 Password (min 8 characters)</label>
              <input type="text" id="cfg-ap-pass" minlength="8" maxlength="63" required>
            </div>
            <div class="form-group">
              <label>WiFi Radio Channel (1 - 13)</label>
              <select id="cfg-ap-chan">
                <option value="1">Channel 1 (2.412 GHz)</option>
                <option value="6">Channel 6 (2.437 GHz)</option>
                <option value="11">Channel 11 (2.462 GHz)</option>
                <option value="2">Channel 2</option>
                <option value="3">Channel 3</option>
                <option value="4">Channel 4</option>
                <option value="5">Channel 5</option>
                <option value="7">Channel 7</option>
                <option value="8">Channel 8</option>
                <option value="9">Channel 9</option>
                <option value="10">Channel 10</option>
                <option value="12">Channel 12</option>
                <option value="13">Channel 13</option>
              </select>
            </div>
            <div class="form-group">
              <label>SSID Visibility</label>
              <select id="cfg-ap-hidden">
                <option value="0">Broadcast SSID (Visible to all clients)</option>
                <option value="1">Hidden SSID (Stealth Mode)</option>
              </select>
            </div>
            <div class="form-group">
              <label>Max Client Connections</label>
              <select id="cfg-ap-max-conn">
                <option value="4">4 Simultaneous Clients (Recommended)</option>
                <option value="2">2 Simultaneous Clients</option>
                <option value="1">1 Dedicated Management Client</option>
              </select>
            </div>
          </div>
        </div>

        <div class="form-section">
          <div class="section-title">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#10b981" stroke-width="2"><circle cx="12" cy="12" r="10"></circle><line x1="2" y1="12" x2="22" y2="12"></line><path d="M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z"></path></svg>
            Captive Portal & IP Architecture
          </div>
          <table class="stats-table">
            <tr><td>Static Webserver IP</td><td>192.168.4.1</td></tr>
            <tr><td>Subnet Mask</td><td>255.255.255.0</td></tr>
            <tr><td>DNS Catch-All Service</td><td>Port 53 (Active Redirect to 192.168.4.1)</td></tr>
            <tr><td>Connected WiFi Clients</td><td id="ap-client-count">1</td></tr>
          </table>
        </div>

        <button type="submit" class="btn btn-primary" style="padding: 14px 28px; font-size: 1rem;">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"></path><polyline points="17 21 17 13 7 13 7 21"></polyline><polyline points="7 3 7 8 15 8"></polyline></svg>
          Save & Apply WiFi AP Settings
        </button>
      </form>
    </div>

    
    <div id="tab-system_diagnostics" class="tab-pane">
      <div class="form-section">
        <div class="section-title">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="var(--primary)" stroke-width="2"><polyline points="22 12 18 12 15 21 9 3 6 12 2 12"></polyline></svg>
          Multi-Year Health Metrics & Diagnostics
        </div>
        <p class="section-desc">64-bit non-blocking operational statistics, memory health, and flash persistence telemetry.</p>

        <table class="stats-table">
          <tr><td>System Uptime (64-bit Non-Rollover)</td><td id="diag-uptime">--</td></tr>
          <tr><td>Free System Heap RAM</td><td id="diag-free-heap">--</td></tr>
          <tr><td>Minimum Free Heap Recorded</td><td id="diag-min-heap">--</td></tr>
          <tr><td>Largest Allocatable Heap Block</td><td id="diag-alloc-heap">--</td></tr>
          <tr><td>Estimated Heap Fragmentation</td><td id="diag-heap-frag">--</td></tr>
          <tr><td>Total SHT31 Sensor Polls</td><td id="diag-sensor-polls">--</td></tr>
          <tr><td>Sensor CRC / I2C Communication Errors</td><td id="diag-sensor-errors">--</td></tr>
          <tr><td>I2C Bus Hang Automatic Recoveries</td><td id="diag-i2c-recovers">--</td></tr>
          <tr><td>NVS Flash Commit Operations (Wear Guard)</td><td id="diag-nvs-writes">--</td></tr>
          <tr><td>Relay Channel 1 (GPIO 25) Cycles</td><td id="diag-relay25-cycles">--</td></tr>
          <tr><td>Relay Channel 2 (GPIO 26) Cycles</td><td id="diag-relay26-cycles">--</td></tr>
          <tr><td>SDA / SCL Hardware Pins</td><td>ESP32 GPIO 21 / 22</td></tr>
        </table>
      </div>

      
      <div class="form-section">
        <div class="section-title" style="color: #f87171;">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"></circle><line x1="15" y1="9" x2="9" y2="15"></line><line x1="9" y1="9" x2="15" y2="15"></line></svg>
          System Maintenance Operations
        </div>
        <p class="section-desc">Hardware control, clean software reboot, and factory parameter restoration.</p>

        <div style="display: flex; gap: 14px; flex-wrap: wrap;">
          <button class="btn btn-secondary" onclick="rebootSystem()">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M21.5 2v6h-6M21.34 15.57a10 10 0 1 1-.57-8.38l5.67-5.67"></path></svg>
            Reboot ESP32
          </button>
          <button class="btn btn-danger" onclick="confirmFactoryReset()">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="3 6 5 6 21 6"></polyline><path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"></path></svg>
            Restore Factory Defaults
          </button>
        </div>

        <div style="margin-top: 18px; padding: 12px; background: rgba(255,255,255,0.03); border-radius: 8px; font-size: 0.8rem; color: var(--text-muted);">
          <b>Hardware Reset Alternative:</b> Hold down the onboard <b>GPIO 0 (BOOT)</b> button continuously for <b>5 seconds</b> at any time to trigger an instant factory reset and wipe configuration from NVS.
        </div>
      </div>
    </div>
  </main>

  <div id="toast" class="toast">
    <span id="toast-msg">Configuration saved</span>
  </div>

  <script>
    let isFahrenheit = false;
    let currentData = null;
    let formLoaded = false;

    function switchTab(tabId) {
      document.querySelectorAll('.tab-pane').forEach(el => el.classList.remove('active'));
      document.querySelectorAll('.nav-tab').forEach(el => el.classList.remove('active'));
      document.getElementById('tab-' + tabId).classList.add('active');
      const idx = ['dashboard', 'sensor_settings', 'wifi_settings', 'system_diagnostics'].indexOf(tabId);
      if (idx !== -1) {
        document.querySelectorAll('.nav-tab')[idx].classList.add('active');
      }
    }

    function showToast(msg) {
      const toast = document.getElementById('toast');
      document.getElementById('toast-msg').textContent = msg;
      toast.style.display = 'flex';
      setTimeout(() => { toast.style.display = 'none'; }, 3500);
    }

    function toggleUnit() {
      isFahrenheit = !isFahrenheit;
      document.getElementById('val-temp-unit').textContent = isFahrenheit ? '°F' : '°C';
      document.getElementById('val-dew-unit').textContent = isFahrenheit ? '°F' : '°C';
      document.getElementById('val-heat-unit').textContent = isFahrenheit ? '°F' : '°C';
      if (currentData) updateDashboardUI(currentData);
    }

    function formatTemp(degC) {
      if (degC === undefined || isNaN(degC)) return '--.-';
      if (isFahrenheit) {
        return (degC * 9 / 5 + 32).toFixed(1);
      }
      return degC.toFixed(1);
    }

    function formatUptime(sec) {
      if (!sec) return '0s';
      const d = Math.floor(sec / 86400);
      const h = Math.floor((sec % 86400) / 3600);
      const m = Math.floor((sec % 3600) / 60);
      const s = sec % 60;
      let res = '';
      if (d > 0) res += d + 'd ';
      if (h > 0 || d > 0) res += h + 'h ';
      if (m > 0 || h > 0 || d > 0) res += m + 'm ';
      res += s + 's';
      return res;
    }

    function setRelayModeUI(mode) {
      document.getElementById('cfg-relay-mode').value = mode;
      if (mode === 0) {
        document.getElementById('mode-unified-btn').classList.add('active');
        document.getElementById('mode-individual-btn').classList.remove('active');
        document.getElementById('grp-unified-logic').style.display = 'flex';
      } else {
        document.getElementById('mode-unified-btn').classList.remove('active');
        document.getElementById('mode-individual-btn').classList.add('active');
        document.getElementById('grp-unified-logic').style.display = 'none';
      }
    }

    function updateDashboardUI(d) {
      currentData = d;
      // Sensor badges
      const badgeSensor = document.getElementById('badge-sensor');
      const badgeSensorText = document.getElementById('badge-sensor-text');
      if (d.sensor_ok) {
        badgeSensor.className = 'badge badge-green';
        badgeSensorText.textContent = 'SHT31 ONLINE';
      } else {
        badgeSensor.className = 'badge badge-red';
        badgeSensorText.textContent = 'SENSOR FAULT';
      }

      // Mode badge
      const badgeModeText = document.getElementById('badge-mode-text');
      badgeModeText.textContent = d.relay_mode === 0 ? 'UNIFIED RELAY' : 'INDIVIDUAL RELAYS';

      // Values
      document.getElementById('val-temp').textContent = formatTemp(d.temp);
      document.getElementById('val-temp-min').textContent = formatTemp(d.temp_min);
      document.getElementById('val-temp-max').textContent = formatTemp(d.temp_max);

      document.getElementById('val-hum').textContent = d.hum !== undefined ? d.hum.toFixed(1) : '--.-';
      document.getElementById('val-hum-min').textContent = d.hum_min !== undefined ? d.hum_min.toFixed(1) + '%' : '--';
      document.getElementById('val-hum-max').textContent = d.hum_max !== undefined ? d.hum_max.toFixed(1) + '%' : '--';
      document.getElementById('val-hum-range').textContent = (d.hum_low || 60) + ' - ' + (d.hum_high || 70) + '%';

      document.getElementById('val-dew').textContent = formatTemp(d.dew);
      document.getElementById('val-heat').textContent = formatTemp(d.heat_idx);

      // Comfort
      const humComfort = document.getElementById('val-hum-comfort');
      if (d.hum < 30) {
        humComfort.textContent = 'DRY';
        humComfort.className = 'badge badge-cyan';
      } else if (d.hum > 70) {
        humComfort.textContent = 'HUMID';
        humComfort.className = 'badge badge-red';
      } else {
        humComfort.textContent = 'OPTIMAL';
        humComfort.className = 'badge badge-green';
      }

      // Dew point condensation status
      const dewStatus = document.getElementById('val-dew-status');
      const dewRisk = document.getElementById('val-dew-risk');
      if (d.temp - d.dew <= 2.5) {
        dewRisk.textContent = 'HIGH RISK';
        dewRisk.className = 'badge badge-red';
        dewStatus.textContent = 'CONDENSATION IMMINENT';
      } else if (d.temp - d.dew <= 5.0) {
        dewRisk.textContent = 'ELEVATED';
        dewRisk.className = 'badge badge-cyan';
        dewStatus.textContent = 'MOISTURE PRESENT';
      } else {
        dewRisk.textContent = 'LOW RISK';
        dewRisk.className = 'badge badge-green';
        dewStatus.textContent = 'SAFE & DRY';
      }

      // Relays
      const card25 = document.getElementById('card-relay25');
      const pill25 = document.getElementById('pill-relay25');
      const r25Title = document.getElementById('relay25-title');
      const r25Sub = document.getElementById('relay25-sub');
      const r25Reason = document.getElementById('relay25-reason');
      const r25Cycles = document.getElementById('relay25-cycles');

      if (d.relay_mode === 0) {
        r25Title.textContent = 'GPIO 25 (Unified Relay)';
        r25Sub.textContent = 'Automates Climate Load (Temp + Hum)';
        r25Reason.textContent = d.relay25 ? 'Threshold Triggered Active' : 'Conditions Satisfied / Idle';
      } else {
        r25Title.textContent = 'GPIO 25 (Temperature Relay)';
        r25Sub.textContent = 'Dedicated Temperature Output';
        r25Reason.textContent = d.temp_trig ? 'Temp Threshold Active' : 'Temp Within Range';
      }

      if (d.relay25) {
        card25.classList.add('active-state');
        pill25.className = 'relay-status-pill relay-on';
        pill25.textContent = 'ENERGIZED';
      } else {
        card25.classList.remove('active-state');
        pill25.className = 'relay-status-pill relay-off';
        pill25.textContent = 'IDLE';
      }
      r25Cycles.textContent = d.relay25_cycles || 0;
      document.getElementById('relay25-polarity').textContent = d.polarity === 1 ? 'Active Low' : 'Active High';

      const card26 = document.getElementById('card-relay26');
      const pill26 = document.getElementById('pill-relay26');
      const r26Reason = document.getElementById('relay26-reason');
      const r26ModeState = document.getElementById('relay26-mode-state');
      const r26Cycles = document.getElementById('relay26-cycles');

      if (d.relay_mode === 0) {
        r26ModeState.textContent = 'Standby (Unified Mode Active)';
        r26Reason.textContent = 'Channel Disabled in Unified Mode';
      } else {
        r26ModeState.textContent = 'Active (Individual Mode)';
        r26Reason.textContent = d.hum_trig ? 'Hum Threshold Active' : 'Hum Within Range';
      }

      if (d.relay26) {
        card26.classList.add('active-state');
        pill26.className = 'relay-status-pill relay-on';
        pill26.textContent = 'ENERGIZED';
      } else {
        card26.classList.remove('active-state');
        pill26.className = 'relay-status-pill relay-off';
        pill26.textContent = 'IDLE';
      }
      r26Cycles.textContent = d.relay26_cycles || 0;

      // Heater button
      document.getElementById('heater-label').textContent = 'SHT31 Heater: ' + (d.heater_on ? 'ON (ACTIVE)' : 'OFF');

      // Populate Settings Forms once
      if (!formLoaded && d.temp_high !== undefined) {
        document.getElementById('cfg-temp-high').value = d.temp_high;
        document.getElementById('cfg-temp-low').value = d.temp_low;
        document.getElementById('cfg-temp-mode').value = d.temp_mode;
        document.getElementById('cfg-hum-high').value = d.hum_high;
        document.getElementById('cfg-hum-low').value = d.hum_low;
        document.getElementById('cfg-hum-mode').value = d.hum_mode;
        document.getElementById('cfg-polarity').value = d.polarity;
        document.getElementById('cfg-anti-chatter').value = d.anti_chatter_sec || 5;
        document.getElementById('cfg-failsafe').value = d.failsafe || 0;
        document.getElementById('cfg-temp-offset').value = d.temp_offset || 0;
        document.getElementById('cfg-hum-offset').value = d.hum_offset || 0;
        document.getElementById('cfg-poll-ms').value = d.poll_ms || 1000;
        document.getElementById('cfg-unified-logic').value = d.unified_logic || 0;
        setRelayModeUI(d.relay_mode || 0);

        // WiFi AP Form
        if (d.ap_ssid) document.getElementById('cfg-ap-ssid').value = d.ap_ssid;
        if (d.ap_channel) document.getElementById('cfg-ap-chan').value = d.ap_channel;
        if (d.ap_hidden !== undefined) document.getElementById('cfg-ap-hidden').value = d.ap_hidden;
        if (d.ap_max_conn) document.getElementById('cfg-ap-max-conn').value = d.ap_max_conn;

        formLoaded = true;
      }

      // Diagnostics Table
      document.getElementById('diag-uptime').textContent = formatUptime(d.uptime_sec);
      document.getElementById('diag-free-heap').textContent = (d.free_heap / 1024).toFixed(1) + ' KB';
      document.getElementById('diag-min-heap').textContent = (d.min_free_heap / 1024).toFixed(1) + ' KB';
      document.getElementById('diag-alloc-heap').textContent = (d.alloc_heap / 1024).toFixed(1) + ' KB';
      document.getElementById('diag-heap-frag').textContent = (d.heap_frag || 0) + '%';
      document.getElementById('diag-sensor-polls').textContent = d.sensor_polls || 0;
      document.getElementById('diag-sensor-errors').textContent = d.sensor_errors || 0;
      document.getElementById('diag-i2c-recovers').textContent = d.i2c_recovers || 0;
      document.getElementById('diag-nvs-writes').textContent = d.nvs_writes || 0;
      document.getElementById('diag-relay25-cycles').textContent = d.relay25_cycles || 0;
      document.getElementById('diag-relay26-cycles').textContent = d.relay26_cycles || 0;
      document.getElementById('ap-client-count').textContent = d.ap_clients || 0;
    }

    async function pollStatus() {
      try {
        const res = await fetch('/api/status');
        if (res.ok) {
          const data = await res.json();
          updateDashboardUI(data);
          const pollBadge = document.getElementById('badge-poll-text');
          pollBadge.textContent = 'SYNC OK';
        }
      } catch (e) {
        document.getElementById('badge-poll-text').textContent = 'OFFLINE';
      }
    }

    async function saveSensorSettings(e) {
      e.preventDefault();
      const payload = {
        relay_mode: parseInt(document.getElementById('cfg-relay-mode').value),
        unified_logic: parseInt(document.getElementById('cfg-unified-logic').value),
        temp_high: parseFloat(document.getElementById('cfg-temp-high').value),
        temp_low: parseFloat(document.getElementById('cfg-temp-low').value),
        temp_mode: parseInt(document.getElementById('cfg-temp-mode').value),
        hum_high: parseFloat(document.getElementById('cfg-hum-high').value),
        hum_low: parseFloat(document.getElementById('cfg-hum-low').value),
        hum_mode: parseInt(document.getElementById('cfg-hum-mode').value),
        polarity: parseInt(document.getElementById('cfg-polarity').value),
        anti_chatter_sec: parseInt(document.getElementById('cfg-anti-chatter').value),
        failsafe: parseInt(document.getElementById('cfg-failsafe').value),
        temp_offset: parseFloat(document.getElementById('cfg-temp-offset').value),
        hum_offset: parseFloat(document.getElementById('cfg-hum-offset').value),
        poll_ms: parseInt(document.getElementById('cfg-poll-ms').value)
      };

      try {
        const res = await fetch('/api/settings/sensor', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload)
        });
        if (res.ok) {
          showToast('Sensor & Relay settings applied successfully');
          pollStatus();
        } else {
          showToast('Failed to save settings');
        }
      } catch (err) {
        showToast('Error communicating with ESP32');
      }
    }

    async function saveWifiSettings(e) {
      e.preventDefault();
      const payload = {
        ap_ssid: document.getElementById('cfg-ap-ssid').value,
        ap_pass: document.getElementById('cfg-ap-pass').value,
        ap_channel: parseInt(document.getElementById('cfg-ap-chan').value),
        ap_hidden: parseInt(document.getElementById('cfg-ap-hidden').value),
        ap_max_conn: parseInt(document.getElementById('cfg-ap-max-conn').value)
      };

      if (payload.ap_pass.length < 8) {
        alert('Password must be at least 8 characters.');
        return;
      }

      if (confirm('WiFi AP configuration will update and restart the wireless subsystem. Continue?')) {
        try {
          const res = await fetch('/api/settings/wifi', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
          });
          if (res.ok) {
            showToast('WiFi settings saved. Reconnecting AP...');
          } else {
            showToast('Failed to save WiFi settings');
          }
        } catch (err) {
          showToast('WiFi AP restarted');
        }
      }
    }

    async function toggleHeater() {
      try {
        const res = await fetch('/api/action/heater', { method: 'POST' });
        if (res.ok) {
          showToast('Toggled SHT31 Onboard Heater');
          pollStatus();
        }
      } catch (e) {
        showToast('Action failed');
      }
    }

    async function resetStats() {
      if (confirm('Reset recorded minimum and maximum values?')) {
        try {
          const res = await fetch('/api/action/reset_stats', { method: 'POST' });
          if (res.ok) {
            showToast('Min/Max statistics reset');
            pollStatus();
          }
        } catch (e) {
          showToast('Action failed');
        }
      }
    }

    async function rebootSystem() {
      if (confirm('Reboot the ESP32 controller now?')) {
        try {
          await fetch('/api/action/reboot', { method: 'POST' });
          showToast('Rebooting ESP32...');
        } catch (e) {
          showToast('Reboot command sent');
        }
      }
    }

    async function confirmFactoryReset() {
      if (confirm('WARNING: This will erase all configured thresholds, AP settings, and restore factory defaults. Are you sure?')) {
        try {
          await fetch('/api/action/factory_reset', { method: 'POST' });
          showToast('Factory reset initiated. Device restarting...');
        } catch (e) {
          showToast('Factory reset commanded');
        }
      }
    }

    // Start live polling loop
    pollStatus();
    setInterval(pollStatus, 1000);
  </script>
</body>
</html>
)rawhtml";

/* 64-bit Non-Wrapping Microsecond and Millisecond Time Providers */
static inline int64_t get_system_time_micros_i64(void) {
  return esp_timer_get_time();
}

static inline uint64_t get_system_time_millis_u64(void) {
  return (uint64_t)(esp_timer_get_time() / 1000LL);
}

static inline uint64_t get_system_uptime_seconds_u64(void) {
  return (uint64_t)(esp_timer_get_time() / 1000000LL);
}

/* CRC-64-ECMA Implementation for Data Integrity Verification */
static uint64_t compute_crc64(const uint8_t *data, size_t length) {
  uint64_t crc = 0xFFFFFFFFFFFFFFFFULL;
  for (size_t i = 0; i < length; ++i) {
    crc ^= ((uint64_t)data[i]) << 56;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000000000000000ULL) {
        crc = (crc << 1) ^ 0x42F0E1EBA9EA3693ULL;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc ^ 0xFFFFFFFFFFFFFFFFULL;
}

/* SHT31 Hardware CRC-8 Verification */
static uint8_t compute_sht31_crc8(const uint8_t *data, size_t length) {
  uint8_t crc = 0xFF;
  const uint8_t polynomial = 0x31;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ polynomial);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

/* Psychrometric Calculations: Dew Point and Apparent Temperature */
static double calculate_dew_point(double temp_c, double hum_rh) {
  if (hum_rh <= 0.0) return -40.0;
  const double a = 17.27;
  const double b = 237.7;
  double safe_hum = hum_rh > 100.0 ? 100.0 : hum_rh;
  double alpha = ((a * temp_c) / (b + temp_c)) + log(safe_hum / 100.0);
  return (b * alpha) / (a - alpha);
}

static double calculate_heat_index(double temp_c, double hum_rh) {
  double temp_f = (temp_c * 1.8) + 32.0;
  double safe_rh = hum_rh < 0.0 ? 0.0 : (hum_rh > 100.0 ? 100.0 : hum_rh);
  double hi_f;
  if (temp_f < 80.0) {
    hi_f = 0.5 * (temp_f + 61.0 + ((temp_f - 68.0) * 1.2) + (safe_rh * 0.094));
  } else {
    hi_f = -42.379 + (2.04901523 * temp_f) + (10.14333127 * safe_rh)
           - (0.22475541 * temp_f * safe_rh) - (0.00683783 * temp_f * temp_f)
           - (0.05481717 * safe_rh * safe_rh) + (0.00122874 * temp_f * temp_f * safe_rh)
           + (0.00085282 * temp_f * safe_rh * safe_rh) - (0.00000199 * temp_f * temp_f * safe_rh * safe_rh);
  }
  return (hi_f - 32.0) / 1.8;
}

/* Production Hardcoded Factory Defaults */
static void populate_default_configuration(DeviceConfig *cfg) {
  memset(cfg, 0, sizeof(DeviceConfig));
  cfg->magic                  = CONFIG_MAGIC_U64;
  cfg->version                = CONFIG_VERSION_U64;
  strncpy(cfg->ap_ssid, "ESP32_SHT31", sizeof(cfg->ap_ssid) - 1);
  strncpy(cfg->ap_password, "12345678", sizeof(cfg->ap_password) - 1);
  cfg->ap_channel             = 1ULL;
  cfg->ap_hidden              = 0ULL;
  cfg->ap_max_conn            = 4ULL;
  cfg->relay_mode             = RELAY_MODE_UNIFIED;
  cfg->unified_logic          = LOGIC_OR;
  cfg->relay_polarity         = POLARITY_ACTIVE_LOW;
  cfg->failsafe_mode          = FAILSAFE_OFF;
  cfg->temp_threshold_high    = 28.0;
  cfg->temp_threshold_low     = 25.0;
  cfg->temp_trigger_mode      = TRIGGER_ABOVE_HIGH;
  cfg->hum_threshold_high     = 70.0;
  cfg->hum_threshold_low      = 60.0;
  cfg->hum_trigger_mode       = TRIGGER_ABOVE_HIGH;
  cfg->poll_interval_ms       = 1000ULL;
  cfg->min_switch_interval_ms = 5000ULL;
  cfg->temp_offset            = 0.0;
  cfg->hum_offset             = 0.0;
  cfg->heater_enabled         = 0ULL;
  cfg->crc64                  = compute_crc64(reinterpret_cast<const uint8_t*>(cfg), offsetof(DeviceConfig, crc64));
}

/* NVS Flash Wear Mitigation and Persistence Engine */
static bool commit_configuration_if_modified(const DeviceConfig &new_cfg) {
  /* Compare in-RAM active config against proposed config up to checksum boundary */
  if (memcmp(&device_config, &new_cfg, offsetof(DeviceConfig, crc64)) == 0) {
    /* Exact match: no physical flash write required, preserving endurance */
    return true;
  }

  DeviceConfig target_cfg = new_cfg;
  target_cfg.magic   = CONFIG_MAGIC_U64;
  target_cfg.version = CONFIG_VERSION_U64;
  target_cfg.crc64   = compute_crc64(reinterpret_cast<const uint8_t*>(&target_cfg), offsetof(DeviceConfig, crc64));

  bool write_succeeded = false;
  if (nv_storage.begin("sht31_cfg", false)) {
    size_t written = nv_storage.putBytes("device_cfg", &target_cfg, sizeof(DeviceConfig));
    nv_storage.end();
    if (written == sizeof(DeviceConfig)) {
      device_config = target_cfg;
      diag_counters.total_nvs_writes_u64++;
      write_succeeded = true;
    }
  }
  return write_succeeded;
}

static void load_configuration_from_storage(void) {
  bool valid_data_loaded = false;
  if (nv_storage.begin("sht31_cfg", true)) {
    if (nv_storage.isKey("device_cfg")) {
      DeviceConfig loaded_cfg;
      size_t bytes_read = nv_storage.getBytes("device_cfg", &loaded_cfg, sizeof(DeviceConfig));
      if (bytes_read == sizeof(DeviceConfig)) {
        if (loaded_cfg.magic == CONFIG_MAGIC_U64 && loaded_cfg.version == CONFIG_VERSION_U64) {
          uint64_t expected_crc = compute_crc64(reinterpret_cast<const uint8_t*>(&loaded_cfg), offsetof(DeviceConfig, crc64));
          if (expected_crc == loaded_cfg.crc64) {
            device_config = loaded_cfg;
            valid_data_loaded = true;
          }
        }
      }
    }
    nv_storage.end();
  }

  if (!valid_data_loaded) {
    populate_default_configuration(&device_config);
    if (nv_storage.begin("sht31_cfg", false)) {
      nv_storage.putBytes("device_cfg", &device_config, sizeof(DeviceConfig));
      nv_storage.end();
      diag_counters.total_nvs_writes_u64++;
    }
  }
}

/* Hardware Output Driver with Polarity Inversion */
static void write_relay_output_pin(uint8_t pin, bool energized, uint64_t polarity) {
  int level;
  if (polarity == POLARITY_ACTIVE_LOW) {
    level = energized ? LOW : HIGH;
  } else {
    level = energized ? HIGH : LOW;
  }
  digitalWrite(pin, level);
}

/* I2C Bus Hang Clearing and Recovery Sequence */
static void execute_i2c_bus_recovery(void) {
  Wire.end();
  pinMode(PIN_I2C_SDA_GPIO, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL_GPIO, INPUT_PULLUP);

  /* Clock SCL 9 times to encourage slave to release SDA */
  pinMode(PIN_I2C_SCL_GPIO, OUTPUT);
  for (uint8_t i = 0; i < 9; ++i) {
    digitalWrite(PIN_I2C_SCL_GPIO, LOW);
    esp_rom_delay_us(6);
    digitalWrite(PIN_I2C_SCL_GPIO, HIGH);
    esp_rom_delay_us(6);
  }

  /* Generate I2C STOP condition */
  pinMode(PIN_I2C_SDA_GPIO, OUTPUT);
  digitalWrite(PIN_I2C_SDA_GPIO, LOW);
  esp_rom_delay_us(6);
  digitalWrite(PIN_I2C_SCL_GPIO, HIGH);
  esp_rom_delay_us(6);
  digitalWrite(PIN_I2C_SDA_GPIO, HIGH);
  esp_rom_delay_us(6);

  /* Re-initialize Wire and SHT31 driver */
  Wire.begin(PIN_I2C_SDA_GPIO, PIN_I2C_SCL_GPIO, 100000ULL);
  Wire.setTimeOut(25);
  if (sht31_driver.begin(0x44)) {
    sht31_i2c_address = 0x44;
    runtime_state.sensor_is_online = true;
  } else if (sht31_driver.begin(0x45)) {
    sht31_i2c_address = 0x45;
    runtime_state.sensor_is_online = true;
  }
  diag_counters.total_i2c_bus_recoveries_u64++;
}

/* Relay Threshold Evaluation Engine with Hysteresis and Anti-Chatter Protection */
static void evaluate_and_update_relays(void) {
  uint64_t current_time_ms_u64 = get_system_time_millis_u64();

  /* Case 1: Sensor offline or hardware fault */
  if (!runtime_state.sensor_is_online) {
    bool desired25 = runtime_state.relay25_energized;
    bool desired26 = runtime_state.relay26_energized;

    if (device_config.failsafe_mode == FAILSAFE_OFF) {
      desired25 = false;
      desired26 = false;
    } else if (device_config.failsafe_mode == FAILSAFE_ON) {
      desired25 = true;
      desired26 = true;
    }

    if (desired25 != runtime_state.relay25_energized) {
      runtime_state.relay25_energized = desired25;
      write_relay_output_pin(PIN_RELAY_UNIFIED_OR_TEMP_GPIO, desired25, device_config.relay_polarity);
      diag_counters.total_relay25_switch_cycles_u64++;
      last_relay25_switch_ms_u64 = current_time_ms_u64;
    }

    if (desired26 != runtime_state.relay26_energized) {
      runtime_state.relay26_energized = desired26;
      write_relay_output_pin(PIN_RELAY_HUM_GPIO, desired26, device_config.relay_polarity);
      diag_counters.total_relay26_switch_cycles_u64++;
      last_relay26_switch_ms_u64 = current_time_ms_u64;
    }
    return;
  }

  /* Case 2: Sensor online - calculate thresholds with hysteresis */
  double t_high = device_config.temp_threshold_high;
  double t_low  = device_config.temp_threshold_low;
  if (t_high < t_low) {
    double tmp = t_high; t_high = t_low; t_low = tmp;
  }

  if (device_config.temp_trigger_mode == TRIGGER_ABOVE_HIGH) {
    /* Cooling mode */
    if (runtime_state.current_temp_c >= t_high) {
      runtime_state.temp_trigger_active = true;
    } else if (runtime_state.current_temp_c <= t_low) {
      runtime_state.temp_trigger_active = false;
    }
  } else {
    /* Heating mode */
    if (runtime_state.current_temp_c <= t_low) {
      runtime_state.temp_trigger_active = true;
    } else if (runtime_state.current_temp_c >= t_high) {
      runtime_state.temp_trigger_active = false;
    }
  }

  double h_high = device_config.hum_threshold_high;
  double h_low  = device_config.hum_threshold_low;
  if (h_high < h_low) {
    double tmp = h_high; h_high = h_low; h_low = tmp;
  }

  if (device_config.hum_trigger_mode == TRIGGER_ABOVE_HIGH) {
    /* Dehumidify mode */
    if (runtime_state.current_hum_rh >= h_high) {
      runtime_state.hum_trigger_active = true;
    } else if (runtime_state.current_hum_rh <= h_low) {
      runtime_state.hum_trigger_active = false;
    }
  } else {
    /* Humidify mode */
    if (runtime_state.current_hum_rh <= h_low) {
      runtime_state.hum_trigger_active = true;
    } else if (runtime_state.current_hum_rh >= h_high) {
      runtime_state.hum_trigger_active = false;
    }
  }

  /* Determine desired relay physical states */
  bool target_relay25 = false;
  bool target_relay26 = false;

  if (device_config.relay_mode == RELAY_MODE_UNIFIED) {
    /* Unified Relay Mode on GPIO 25 */
    if (device_config.unified_logic == LOGIC_OR) {
      target_relay25 = (runtime_state.temp_trigger_active || runtime_state.hum_trigger_active);
    } else if (device_config.unified_logic == LOGIC_AND) {
      target_relay25 = (runtime_state.temp_trigger_active && runtime_state.hum_trigger_active);
    } else if (device_config.unified_logic == LOGIC_TEMP_ONLY) {
      target_relay25 = runtime_state.temp_trigger_active;
    } else if (device_config.unified_logic == LOGIC_HUM_ONLY) {
      target_relay25 = runtime_state.hum_trigger_active;
    }
    target_relay26 = false; /* GPIO 26 remains inactive in unified mode */
  } else {
    /* Individual Mode: GPIO 25 is Temperature, GPIO 26 is Humidity */
    target_relay25 = runtime_state.temp_trigger_active;
    target_relay26 = runtime_state.hum_trigger_active;
  }

  /* Apply anti-chatter timing guards before switching */
  if (target_relay25 != runtime_state.relay25_energized) {
    if (last_relay25_switch_ms_u64 == 0ULL || (current_time_ms_u64 - last_relay25_switch_ms_u64) >= device_config.min_switch_interval_ms) {
      runtime_state.relay25_energized = target_relay25;
      write_relay_output_pin(PIN_RELAY_UNIFIED_OR_TEMP_GPIO, target_relay25, device_config.relay_polarity);
      diag_counters.total_relay25_switch_cycles_u64++;
      last_relay25_switch_ms_u64 = current_time_ms_u64;
    }
  }

  if (target_relay26 != runtime_state.relay26_energized) {
    if (last_relay26_switch_ms_u64 == 0ULL || (current_time_ms_u64 - last_relay26_switch_ms_u64) >= device_config.min_switch_interval_ms) {
      runtime_state.relay26_energized = target_relay26;
      write_relay_output_pin(PIN_RELAY_HUM_GPIO, target_relay26, device_config.relay_polarity);
      diag_counters.total_relay26_switch_cycles_u64++;
      last_relay26_switch_ms_u64 = current_time_ms_u64;
    }
  }
}

/* Non-Blocking SHT31 Polling State Machine */
static void service_sensor_engine(void) {
  uint64_t current_time_ms_u64 = get_system_time_millis_u64();
  int64_t current_time_us_i64 = get_system_time_micros_i64();

  if (sensor_engine_state == SENSOR_STATE_IDLE) {
    if ((current_time_ms_u64 - last_sensor_poll_ms_u64) >= device_config.poll_interval_ms) {
      Wire.beginTransmission(sht31_i2c_address);
      Wire.write((uint8_t)(SHT31_CMD_MEAS_HIGHREP_NOSTRETCH >> 8));
      Wire.write((uint8_t)(SHT31_CMD_MEAS_HIGHREP_NOSTRETCH & 0xFF));
      uint8_t transmission_status = Wire.endTransmission();

      if (transmission_status == 0) {
        sensor_request_start_us_i64 = current_time_us_i64;
        sensor_engine_state = SENSOR_STATE_CONVERTING;
      } else {
        diag_counters.total_sensor_errors_u64++;
        consecutive_sensor_errors_u64++;
        if (consecutive_sensor_errors_u64 >= 5ULL) {
          execute_i2c_bus_recovery();
          consecutive_sensor_errors_u64 = 0;
        }
        last_sensor_poll_ms_u64 = current_time_ms_u64;
      }
    }
  } else if (sensor_engine_state == SENSOR_STATE_CONVERTING) {
    /* 20,000 microseconds (20ms) conversion window for high repeatability */
    if ((current_time_us_i64 - sensor_request_start_us_i64) >= 20000LL) {
      uint8_t raw_buffer[6];
      size_t bytes_read = Wire.requestFrom((uint8_t)sht31_i2c_address, (uint8_t)6);

      if (bytes_read == 6) {
        for (uint8_t i = 0; i < 6; ++i) {
          raw_buffer[i] = (uint8_t)Wire.read();
        }

        uint8_t temp_crc = compute_sht31_crc8(raw_buffer, 2);
        uint8_t hum_crc  = compute_sht31_crc8(raw_buffer + 3, 2);

        if (temp_crc == raw_buffer[2] && hum_crc == raw_buffer[5]) {
          uint32_t raw_temp_u32 = ((uint32_t)raw_buffer[0] << 8) | raw_buffer[1];
          uint32_t raw_hum_u32  = ((uint32_t)raw_buffer[3] << 8) | raw_buffer[4];

          double raw_temp_c = -45.0 + (175.0 * (double)raw_temp_u32 / 65535.0);
          double raw_hum_rh = 100.0 * (double)raw_hum_u32 / 65535.0;

          runtime_state.current_temp_c = raw_temp_c + device_config.temp_offset;
          runtime_state.current_hum_rh = raw_hum_rh + device_config.hum_offset;

          if (runtime_state.current_hum_rh < 0.0) runtime_state.current_hum_rh = 0.0;
          if (runtime_state.current_hum_rh > 100.0) runtime_state.current_hum_rh = 100.0;

          runtime_state.current_dew_point_c = calculate_dew_point(runtime_state.current_temp_c, runtime_state.current_hum_rh);
          runtime_state.current_heat_index_c = calculate_heat_index(runtime_state.current_temp_c, runtime_state.current_hum_rh);

          if (runtime_state.current_temp_c < runtime_state.min_temp_c) runtime_state.min_temp_c = runtime_state.current_temp_c;
          if (runtime_state.current_temp_c > runtime_state.max_temp_c) runtime_state.max_temp_c = runtime_state.current_temp_c;
          if (runtime_state.current_hum_rh < runtime_state.min_hum_rh) runtime_state.min_hum_rh = runtime_state.current_hum_rh;
          if (runtime_state.current_hum_rh > runtime_state.max_hum_rh) runtime_state.max_hum_rh = runtime_state.current_hum_rh;

          runtime_state.sensor_is_online = true;
          consecutive_sensor_errors_u64 = 0;
          diag_counters.total_sensor_polls_u64++;

          evaluate_and_update_relays();
        } else {
          diag_counters.total_sensor_errors_u64++;
          consecutive_sensor_errors_u64++;
        }
      } else {
        diag_counters.total_sensor_errors_u64++;
        consecutive_sensor_errors_u64++;
      }

      if (consecutive_sensor_errors_u64 >= 5ULL) {
        runtime_state.sensor_is_online = false;
        execute_i2c_bus_recovery();
        consecutive_sensor_errors_u64 = 0;
        evaluate_and_update_relays();
      }

      sensor_engine_state = SENSOR_STATE_IDLE;
      last_sensor_poll_ms_u64 = current_time_ms_u64;
    }
  }
}

/* Debounced Non-Blocking 5-Second Factory Reset Service */
static void service_factory_reset_button(void) {
  uint64_t current_time_ms_u64 = get_system_time_millis_u64();
  if ((current_time_ms_u64 - last_button_sample_ms_u64) < 20ULL) {
    return;
  }
  last_button_sample_ms_u64 = current_time_ms_u64;

  int pin_reading = digitalRead(PIN_BOOT_BUTTON_GPIO);
  if (pin_reading == LOW) {
    if (button_press_start_ms_u64 == 0ULL) {
      button_press_start_ms_u64 = current_time_ms_u64;
    } else {
      uint64_t held_duration_ms_u64 = current_time_ms_u64 - button_press_start_ms_u64;
      if (held_duration_ms_u64 >= 5000ULL) {
        /* 5 seconds continuous hold confirmed */
        diag_counters.total_factory_resets_u64++;
        if (nv_storage.begin("sht31_cfg", false)) {
          nv_storage.clear();
          nv_storage.end();
        }
        write_relay_output_pin(PIN_RELAY_UNIFIED_OR_TEMP_GPIO, false, device_config.relay_polarity);
        write_relay_output_pin(PIN_RELAY_HUM_GPIO, false, device_config.relay_polarity);
        esp_restart();
      }
    }
  } else {
    button_press_start_ms_u64 = 0ULL;
  }
}

/* Captive Portal Probe and Redirection Handlers */
static void handle_captive_portal_redirect(void) {
  diag_counters.total_http_requests_served_u64++;
  web_server.sendHeader("Location", "http://192.168.4.1/", true);
  web_server.send(302, "text/plain", "");
}

static void handle_root_request(void) {
  diag_counters.total_http_requests_served_u64++;
  web_server.send_P(200, "text/html", INDEX_PAGE_HTML);
}

/* Heap and Telemetry Status JSON Serializer without Dynamic Allocations */
static void handle_api_status(void) {
  diag_counters.total_http_requests_served_u64++;

  size_t free_heap_bytes  = ESP.getFreeHeap();
  size_t min_heap_bytes   = ESP.getMinFreeHeap();
  size_t max_alloc_bytes  = ESP.getMaxAllocHeap();
  uint64_t uptime_sec_u64 = get_system_uptime_seconds_u64();
  uint8_t connected_clients = WiFi.softAPgetStationNum();

  /* Estimate heap fragmentation percentage */
  double frag_ratio = 0.0;
  if (free_heap_bytes > 0 && max_alloc_bytes > 0) {
    frag_ratio = 100.0 - ((double)max_alloc_bytes * 100.0 / (double)free_heap_bytes);
    if (frag_ratio < 0.0) frag_ratio = 0.0;
  }

  char json_buffer[1536];
  snprintf(json_buffer, sizeof(json_buffer),
    "{"
    "\"temp\":%.2f,\"hum\":%.2f,\"dew\":%.2f,\"heat_idx\":%.2f,"
    "\"temp_min\":%.2f,\"temp_max\":%.2f,\"hum_min\":%.2f,\"hum_max\":%.2f,"
    "\"sensor_ok\":%s,\"heater_on\":%s,"
    "\"relay25\":%s,\"relay26\":%s,"
    "\"relay_mode\":%" PRIu64 ",\"unified_logic\":%" PRIu64 ","
    "\"temp_trig\":%s,\"hum_trig\":%s,"
    "\"uptime_sec\":%" PRIu64 ","
    "\"free_heap\":%u,\"min_free_heap\":%u,\"alloc_heap\":%u,\"heap_frag\":%.1f,"
    "\"sensor_polls\":%" PRIu64 ",\"sensor_errors\":%" PRIu64 ",\"i2c_recovers\":%" PRIu64 ","
    "\"nvs_writes\":%" PRIu64 ",\"relay25_cycles\":%" PRIu64 ",\"relay26_cycles\":%" PRIu64 ","
    "\"ap_ssid\":\"%s\",\"ap_channel\":%" PRIu64 ",\"ap_hidden\":%" PRIu64 ",\"ap_max_conn\":%" PRIu64 ",\"ap_clients\":%u,"
    "\"temp_high\":%.2f,\"temp_low\":%.2f,\"temp_mode\":%" PRIu64 ","
    "\"hum_high\":%.2f,\"hum_low\":%.2f,\"hum_mode\":%" PRIu64 ","
    "\"polarity\":%" PRIu64 ",\"failsafe\":%" PRIu64 ","
    "\"temp_offset\":%.2f,\"hum_offset\":%.2f,\"poll_ms\":%" PRIu64 ",\"anti_chatter_sec\":%" PRIu64
    "}",
    runtime_state.current_temp_c, runtime_state.current_hum_rh,
    runtime_state.current_dew_point_c, runtime_state.current_heat_index_c,
    runtime_state.min_temp_c, runtime_state.max_temp_c,
    runtime_state.min_hum_rh, runtime_state.max_hum_rh,
    runtime_state.sensor_is_online ? "true" : "false",
    runtime_state.heater_active ? "true" : "false",
    runtime_state.relay25_energized ? "true" : "false",
    runtime_state.relay26_energized ? "true" : "false",
    device_config.relay_mode, device_config.unified_logic,
    runtime_state.temp_trigger_active ? "true" : "false",
    runtime_state.hum_trigger_active ? "true" : "false",
    uptime_sec_u64,
    (unsigned int)free_heap_bytes, (unsigned int)min_heap_bytes, (unsigned int)max_alloc_bytes, frag_ratio,
    diag_counters.total_sensor_polls_u64, diag_counters.total_sensor_errors_u64, diag_counters.total_i2c_bus_recoveries_u64,
    diag_counters.total_nvs_writes_u64, diag_counters.total_relay25_switch_cycles_u64, diag_counters.total_relay26_switch_cycles_u64,
    device_config.ap_ssid, device_config.ap_channel, device_config.ap_hidden, device_config.ap_max_conn, (unsigned int)connected_clients,
    device_config.temp_threshold_high, device_config.temp_threshold_low, device_config.temp_trigger_mode,
    device_config.hum_threshold_high, device_config.hum_threshold_low, device_config.hum_trigger_mode,
    device_config.relay_polarity, device_config.failsafe_mode,
    device_config.temp_offset, device_config.hum_offset, device_config.poll_interval_ms,
    (uint64_t)(device_config.min_switch_interval_ms / 1000ULL)
  );

  web_server.send(200, "application/json", json_buffer);
}

/* HTTP Parameter Parsing Helpers for Uniform Key-Value and JSON Handling */
static bool extract_http_double(const char *name, double &destination) {
  if (web_server.hasArg(name)) {
    destination = web_server.arg(name).toDouble();
    return true;
  }
  if (web_server.hasArg("plain")) {
    const String &body = web_server.arg("plain");
    char key_pattern[64];
    snprintf(key_pattern, sizeof(key_pattern), "\"%s\"", name);
    int index = body.indexOf(key_pattern);
    if (index >= 0) {
      int colon = body.indexOf(':', index + strlen(key_pattern));
      if (colon >= 0) {
        int val_start = colon + 1;
        while (val_start < (int)body.length() && (body[val_start] == ' ' || body[val_start] == '\t' || body[val_start] == '\r' || body[val_start] == '\n')) {
          val_start++;
        }
        if (val_start < (int)body.length()) {
          destination = body.substring(val_start).toDouble();
          return true;
        }
      }
    }
  }
  return false;
}

static bool extract_http_uint64(const char *name, uint64_t &destination) {
  if (web_server.hasArg(name)) {
    destination = (uint64_t)strtoull(web_server.arg(name).c_str(), NULL, 10);
    return true;
  }
  if (web_server.hasArg("plain")) {
    const String &body = web_server.arg("plain");
    char key_pattern[64];
    snprintf(key_pattern, sizeof(key_pattern), "\"%s\"", name);
    int index = body.indexOf(key_pattern);
    if (index >= 0) {
      int colon = body.indexOf(':', index + strlen(key_pattern));
      if (colon >= 0) {
        int val_start = colon + 1;
        while (val_start < (int)body.length() && (body[val_start] == ' ' || body[val_start] == '\t' || body[val_start] == '\r' || body[val_start] == '\n')) {
          val_start++;
        }
        if (val_start < (int)body.length()) {
          destination = (uint64_t)strtoull(body.substring(val_start).c_str(), NULL, 10);
          return true;
        }
      }
    }
  }
  return false;
}

static bool extract_http_string(const char *name, char *dest, size_t max_len) {
  if (web_server.hasArg(name)) {
    strncpy(dest, web_server.arg(name).c_str(), max_len - 1);
    dest[max_len - 1] = '\0';
    return true;
  }
  if (web_server.hasArg("plain")) {
    const String &body = web_server.arg("plain");
    char key_pattern[64];
    snprintf(key_pattern, sizeof(key_pattern), "\"%s\"", name);
    int index = body.indexOf(key_pattern);
    if (index >= 0) {
      int colon = body.indexOf(':', index + strlen(key_pattern));
      if (colon >= 0) {
        int quote_start = body.indexOf('"', colon + 1);
        if (quote_start >= 0) {
          int quote_end = body.indexOf('"', quote_start + 1);
          if (quote_end > quote_start) {
            String extracted = body.substring(quote_start + 1, quote_end);
            strncpy(dest, extracted.c_str(), max_len - 1);
            dest[max_len - 1] = '\0';
            return true;
          }
        }
      }
    }
  }
  return false;
}

/* Sensor and Relay Settings Management Endpoint */
static void handle_api_settings_sensor(void) {
  diag_counters.total_http_requests_served_u64++;
  DeviceConfig updated_cfg = device_config;

  uint64_t u64_val = 0ULL;
  double   f64_val = 0.0;

  if (extract_http_uint64("relay_mode", u64_val)) updated_cfg.relay_mode = u64_val;
  if (extract_http_uint64("unified_logic", u64_val)) updated_cfg.unified_logic = u64_val;
  if (extract_http_double("temp_high", f64_val)) updated_cfg.temp_threshold_high = f64_val;
  if (extract_http_double("temp_low", f64_val)) updated_cfg.temp_threshold_low = f64_val;
  if (extract_http_uint64("temp_mode", u64_val)) updated_cfg.temp_trigger_mode = u64_val;
  if (extract_http_double("hum_high", f64_val)) updated_cfg.hum_threshold_high = f64_val;
  if (extract_http_double("hum_low", f64_val)) updated_cfg.hum_threshold_low = f64_val;
  if (extract_http_uint64("hum_mode", u64_val)) updated_cfg.hum_trigger_mode = u64_val;
  if (extract_http_uint64("polarity", u64_val)) updated_cfg.relay_polarity = u64_val;

  uint64_t anti_chatter_sec_u64 = 5ULL;
  if (extract_http_uint64("anti_chatter_sec", anti_chatter_sec_u64)) {
    if (anti_chatter_sec_u64 < 1ULL) anti_chatter_sec_u64 = 1ULL;
    if (anti_chatter_sec_u64 > 300ULL) anti_chatter_sec_u64 = 300ULL;
    updated_cfg.min_switch_interval_ms = anti_chatter_sec_u64 * 1000ULL;
  }

  if (extract_http_uint64("failsafe", u64_val)) updated_cfg.failsafe_mode = u64_val;
  if (extract_http_double("temp_offset", f64_val)) updated_cfg.temp_offset = f64_val;
  if (extract_http_double("hum_offset", f64_val)) updated_cfg.hum_offset = f64_val;

  uint64_t new_poll_ms_u64 = 1000ULL;
  if (extract_http_uint64("poll_ms", new_poll_ms_u64)) {
    if (new_poll_ms_u64 < 500ULL) new_poll_ms_u64 = 500ULL;
    if (new_poll_ms_u64 > 60000ULL) new_poll_ms_u64 = 60000ULL;
    updated_cfg.poll_interval_ms = new_poll_ms_u64;
  }

  commit_configuration_if_modified(updated_cfg);
  evaluate_and_update_relays();

  web_server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Sensor and relay parameters applied\"}");
}

/* WiFi AP Settings Management Endpoint */
static void handle_api_settings_wifi(void) {
  diag_counters.total_http_requests_served_u64++;
  DeviceConfig updated_cfg = device_config;

  char new_ssid[32] = {0};
  char new_pass[64] = {0};

  if (extract_http_string("ap_ssid", new_ssid, sizeof(new_ssid)) && strlen(new_ssid) > 0) {
    strncpy(updated_cfg.ap_ssid, new_ssid, sizeof(updated_cfg.ap_ssid) - 1);
  }

  if (extract_http_string("ap_pass", new_pass, sizeof(new_pass)) && strlen(new_pass) >= 8) {
    strncpy(updated_cfg.ap_password, new_pass, sizeof(updated_cfg.ap_password) - 1);
  }

  uint64_t new_chan_u64 = 1ULL;
  if (extract_http_uint64("ap_channel", new_chan_u64)) {
    if (new_chan_u64 >= 1ULL && new_chan_u64 <= 13ULL) {
      updated_cfg.ap_channel = new_chan_u64;
    }
  }

  uint64_t u64_wifi_val = 0ULL;
  if (extract_http_uint64("ap_hidden", u64_wifi_val)) updated_cfg.ap_hidden = u64_wifi_val;

  uint64_t new_max_conn_u64 = 4ULL;
  if (extract_http_uint64("ap_max_conn", new_max_conn_u64)) {
    if (new_max_conn_u64 >= 1ULL && new_max_conn_u64 <= 4ULL) {
      updated_cfg.ap_max_conn = new_max_conn_u64;
    }
  }

  commit_configuration_if_modified(updated_cfg);

  web_server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"WiFi AP configuration saved. Refreshing network...\"}");

  /* Schedule safe non-blocking SoftAP restart */
  pending_system_restart = true;
  restart_scheduled_ms_u64 = get_system_time_millis_u64() + 1000ULL;
}

/* SHT31 Onboard Heater Control */
static void handle_api_action_heater(void) {
  diag_counters.total_http_requests_served_u64++;
  runtime_state.heater_active = !runtime_state.heater_active;
  sht31_driver.heater(runtime_state.heater_active);

  DeviceConfig updated_cfg = device_config;
  updated_cfg.heater_enabled = runtime_state.heater_active ? 1ULL : 0ULL;
  commit_configuration_if_modified(updated_cfg);

  web_server.send(200, "application/json", runtime_state.heater_active ? "{\"heater\":true}" : "{\"heater\":false}");
}

/* Reset Recorded Extremes */
static void handle_api_action_reset_stats(void) {
  diag_counters.total_http_requests_served_u64++;
  runtime_state.min_temp_c = runtime_state.current_temp_c;
  runtime_state.max_temp_c = runtime_state.current_temp_c;
  runtime_state.min_hum_rh = runtime_state.current_hum_rh;
  runtime_state.max_hum_rh = runtime_state.current_hum_rh;
  web_server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Statistical records cleared\"}");
}

/* Clean System Reboot */
static void handle_api_action_reboot(void) {
  diag_counters.total_http_requests_served_u64++;
  web_server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"System reboot initiated\"}");
  pending_system_restart = true;
  restart_scheduled_ms_u64 = get_system_time_millis_u64() + 600ULL;
}

/* Factory Defaults Restoration */
static void handle_api_action_factory_reset(void) {
  diag_counters.total_http_requests_served_u64++;
  if (nv_storage.begin("sht31_cfg", false)) {
    nv_storage.clear();
    nv_storage.end();
  }
  populate_default_configuration(&device_config);
  web_server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Factory configuration restored\"}");
  pending_system_restart = true;
  restart_scheduled_ms_u64 = get_system_time_millis_u64() + 600ULL;
}

/* Catch-All 404 & Captive Portal Redirection */
static void handle_unknown_request(void) {
  handle_captive_portal_redirect();
}

/* SoftAP Wireless Subsystem Initialization */
static void initialize_soft_access_point(void) {
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_STATIC_IP, AP_STATIC_IP, AP_SUBNET_MASK);
  WiFi.softAP(
    device_config.ap_ssid,
    device_config.ap_password,
    (int)device_config.ap_channel,
    (int)device_config.ap_hidden,
    (int)device_config.ap_max_conn
  );
}

/* System Setup */
void setup(void) {
  /* Initialize GPIO outputs in safe de-energized states */
  pinMode(PIN_RELAY_UNIFIED_OR_TEMP_GPIO, OUTPUT);
  pinMode(PIN_RELAY_HUM_GPIO, OUTPUT);
  pinMode(PIN_BOOT_BUTTON_GPIO, INPUT_PULLUP);

  /* Configure Task Watchdog Timer for 8 seconds */
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 8000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&twdt_config);
  esp_task_wdt_add(NULL);

  /* Load verified configuration from NVS */
  load_configuration_from_storage();

  /* Initialize relay hardware pins to default polarity */
  write_relay_output_pin(PIN_RELAY_UNIFIED_OR_TEMP_GPIO, false, device_config.relay_polarity);
  write_relay_output_pin(PIN_RELAY_HUM_GPIO, false, device_config.relay_polarity);

  /* Initialize I2C Bus on GPIO 21 and 22 */
  Wire.begin(PIN_I2C_SDA_GPIO, PIN_I2C_SCL_GPIO, 100000ULL);
  Wire.setTimeOut(25);

  /* Detect and initialize SHT31 sensor */
  if (sht31_driver.begin(0x44)) {
    sht31_i2c_address = 0x44;
    runtime_state.sensor_is_online = true;
  } else if (sht31_driver.begin(0x45)) {
    sht31_i2c_address = 0x45;
    runtime_state.sensor_is_online = true;
  } else {
    runtime_state.sensor_is_online = false;
  }

  if (runtime_state.sensor_is_online && device_config.heater_enabled == 1ULL) {
    runtime_state.heater_active = true;
    sht31_driver.heater(true);
  }

  /* Initialize statistical baseline */
  runtime_state.min_temp_c = 999.0;
  runtime_state.max_temp_c = -999.0;
  runtime_state.min_hum_rh = 999.0;
  runtime_state.max_hum_rh = -999.0;

  /* Start SoftAP */
  initialize_soft_access_point();

  /* Start DNS Catch-All Server on Port 53 */
  dns_server.setErrorReplyCode(DNSReplyCode::NoError);
  dns_server.start(53, "*", AP_STATIC_IP);

  /* Configure Web Server Endpoints */
  web_server.on("/", HTTP_GET, handle_root_request);
  web_server.on("/api/status", HTTP_GET, handle_api_status);
  web_server.on("/api/settings/sensor", HTTP_POST, handle_api_settings_sensor);
  web_server.on("/api/settings/wifi", HTTP_POST, handle_api_settings_wifi);
  web_server.on("/api/action/heater", HTTP_POST, handle_api_action_heater);
  web_server.on("/api/action/reset_stats", HTTP_POST, handle_api_action_reset_stats);
  web_server.on("/api/action/reboot", HTTP_POST, handle_api_action_reboot);
  web_server.on("/api/action/factory_reset", HTTP_POST, handle_api_action_factory_reset);

  /* Captive Portal OS Identification URLs */
  web_server.on("/generate_204", HTTP_GET, handle_captive_portal_redirect);
  web_server.on("/gen_204", HTTP_GET, handle_captive_portal_redirect);
  web_server.on("/ncsi.txt", HTTP_GET, handle_captive_portal_redirect);
  web_server.on("/hotspot-detect.html", HTTP_GET, handle_captive_portal_redirect);
  web_server.on("/canonical.html", HTTP_GET, handle_captive_portal_redirect);
  web_server.on("/connecttest.txt", HTTP_GET, handle_captive_portal_redirect);
  web_server.on("/redirect", HTTP_GET, handle_captive_portal_redirect);
  web_server.onNotFound(handle_unknown_request);

  web_server.begin();
}

/* Non-Blocking Continuous Execution Loop */
void loop(void) {
  /* Service Hardware Watchdog Timer */
  esp_task_wdt_reset();

  /* Service DNS Requests for Captive Portal */
  dns_server.processNextRequest();

  /* Service Asynchronous Web Requests */
  web_server.handleClient();

  /* Service Non-Blocking SHT31 Sensor Polling */
  service_sensor_engine();

  /* Service Non-Blocking 5-Second Factory Reset Button */
  service_factory_reset_button();

  /* Check Scheduled Restarts */
  if (pending_system_restart) {
    if (get_system_time_millis_u64() >= restart_scheduled_ms_u64) {
      write_relay_output_pin(PIN_RELAY_UNIFIED_OR_TEMP_GPIO, false, device_config.relay_polarity);
      write_relay_output_pin(PIN_RELAY_HUM_GPIO, false, device_config.relay_polarity);
      esp_restart();
    }
  }
}
