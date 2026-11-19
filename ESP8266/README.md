## 1. Hardware Wiring (NodeMCU / ESP8266)

| Signal | ESP8266 Pin | NodeMCU Label |
|---|---|---|
| SHT31 SDA | GPIO 5 | D1 |
| SHT31 SCL | GPIO 4 | D2 |
| Relay 1 (Temp/Unified) | GPIO 14 | D5 |
| Relay 2 (Humidity) | GPIO 12 | D6 |
| Factory reset button | GPIO 0 | D3 (FLASH key) |
| SHT31 VCC | 3.3 V | 3V3 |
| SHT31 GND | GND | GND |

> SHT31 address is auto-detected at **0x44** or **0x45**. Use 3.3 V logic only — do not feed 5 V into GPIOs.

## 2. Flash the Firmware

1. Install **Arduino IDE** + **ESP8266 board package** (Boards Manager URL: `https://arduino.esp8266.com/stable/package_esp8266com_index.json`).
2. Install libraries via Library Manager:
   - `Adafruit SHT31 Library`
   - `Adafruit BusIO` (dependency)
3. Open `esp8266_sht31_controller.ino`.
4. Select **NodeMCU 1.0 (ESP-12E Module)** and the correct COM port.
5. Upload.

## 3. First Connection

1. Power the board. It boots as a Wi-Fi access point.
2. On your phone/PC, connect to SSID **`ESP8266_SHT31`**, password **`12345678`**.
3. A captive portal should open automatically; if not, browse to **http://192.168.4.1**.

## 4. Configure

- **Sensor & Relays tab** — set:
  - Relay mode: *Unified* (one relay for both) or *Individual* (temp on D5, hum on D6)
  - Temperature / humidity high & low thresholds and action mode (cool/heat, dehumidify/humidify)
  - Relay polarity (Active Low for most 5 V relay modules)
  - Anti-chatter interval (default 5 s)
  - Fail-safe state on sensor loss
  - Calibration offsets and polling rate
- **WiFi AP Settings tab** — change SSID, password (min 8 chars), channel, visibility, max clients. Saving restarts the Wi-Fi subsystem.
- **Diagnostics tab** — uptime, heap, error counts, relay cycles, reboot, and factory reset.

## 5. Everyday Operation

- The dashboard auto-refreshes every second.
- Tap **°C / °F** to switch units.
- **SHT31 Heater** button toggles the sensor's onboard heater (for condensation recovery).
- **Reset Min/Max** clears recorded extremes.

## 6. Factory Reset

- **Web:** Diagnostics → *Restore Factory Defaults*.
- **Hardware:** Hold the **FLASH (D3/GPIO0)** button for **5 seconds** — relays go off and the device reboots with defaults.

## 7. Default Thresholds (out of the box)

| Parameter | Default |
|---|---|
| Temp high / low | 28 °C / 25 °C (cooling) |
| Humidity high / low | 70 % / 60 % (dehumidify) |
| Relay mode | Unified, OR logic |
| Polarity | Active Low |
| Poll interval | 1000 ms |
| Anti-chatter | 5 s |
| Fail-safe | Relays OFF |

## Troubleshooting

- **Sensor fault badge** — check SDA/SCL wiring and 3.3 V supply; the firmware auto-recovers a hung I2C bus after 5 consecutive errors.
- **Relay never switches** — confirm polarity matches your module (Active Low is typical), and that the anti-chatter interval hasn't blocked a recent toggle.
- **Can't reach 192.168.4.1** — disable mobile data on the phone, or manually set a static IP in `192.168.4.0/24`.
