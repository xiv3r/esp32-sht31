## Hardware Wiring

| Component | ESP8266 Pin | Notes |
|---|---|---|
| SHT31 SDA | GPIO5 (D1) | 4.7 kΩ pull-up to 3.3 V |
| SHT31 SCL | GPIO4 (D2) | 4.7 kΩ pull-up to 3.3 V |
| SHT31 VCC | 3.3 V | — |
| SHT31 GND | GND | — |
| Relay IN | GPIO12 (D6) | **Add 10 kΩ pulldown to GND** (boot strap) |
| Relay VCC | 3.3 V or 5 V | Per module |
| Relay GND | GND | — |
| Boot button | GPIO0 ↔ GND | Internal pull-up enabled; **do not hold at power-on** |

Relay module powered separately if coil > 3.3 V; use opto-isolated module for mains loads.

---

## Flash & First Boot

1. Open the code in Arduino IDE.
2. Install boards/libs:
   - **ESP8266 core** (Boards Manager: `esp8266` by ESP8266 Community)
   - **Adafruit SHT31 Library** (Library Manager)
3. Board: **Generic ESP8266 Module** (or your exact board). Flash size ≥ 1 MB.
4. Compile & upload. **Do not hold GPIO0 during upload power-on reset** unless your board requires it for flash mode.
5. On first boot the device creates Wi-Fi AP:
   - **SSID:** `ESP8266_SHT31`
   - **Password:** `12345678`

---

## Connect & Use

1. On phone/PC, join `ESP8266_SHT31`.
2. Captive portal should auto-open. If not, browse to **http://192.168.4.1**.
3. Dashboard shows live:
   - Temperature (°C)
   - Humidity (%)
   - Relay state (ON/OFF)
   - Sensor status (OK/ERROR)
   - Updates every 2 s
4. Set **Temp Threshold**, **Hum Threshold**, **Hysteresis** → **Save**.
   - Relay turns **ON** when temp ≥ temp threshold **OR** humidity ≥ hum threshold.
   - Relay turns **OFF** when temp ≤ (threshold − hysteresis) **AND** humidity ≤ (threshold − hysteresis).
5. **Restart** → soft reboot.
6. **Factory Reset** → erases saved settings, reboots with defaults.

---

## Factory Reset via Button

- **Hold GPIO0 button for 5 seconds** while device is running.
- Device reboots with default thresholds (Temp 30 °C, Hum 70 %, Hysteresis 1.0).

---

## Default Settings

| Setting | Default |
|---|---|
| Temp threshold | 30.0 °C |
| Hum threshold | 70.0 % |
| Hysteresis | 1.0 |
| Valid temp range | -50.0 … 150.0 °C |
| Valid hum range | 0.0 … 100.0 % |
| Valid hysteresis | 0.0 … 50.0 |

Settings persist across reboot (flash, CRC-protected, wear-minimized).

---

## LED / Behavior Reference

| Behavior | Meaning |
|---|---|
| Page values show `--` | First read pending (up to 2 s) |
| Sensor status **ERROR** | SHT31 not detected — check wiring/pull-ups |
| Relay ON, values dropping | Normal — waits for hysteresis band before turning OFF |
| No AP visible | Check 3.3 V supply (needs ≥ 300 mA peak) |

---

## Troubleshooting

| Problem | Fix |
|---|---|
| AP not visible | Power cycle; verify 3.3 V regulator can supply ~500 mA peak |
| Dashboard won't open | Manually go to `http://192.168.4.1` |
| Sensor ERROR | Verify SDA=GPIO5, SCL=GPIO4, pull-ups present, sensor at 0x44 or 0x45 |
| Relay stuck ON | Check threshold values; factory reset |
| Can't flash | Disconnect relay, hold GPIO0 to GND during reset |
| Captive portal doesn't pop up | Open browser manually to `192.168.4.1` |

---

## Notes

- AP is **always on** — device does not connect to your home Wi-Fi.
- Only one client action at a time is needed; multiple devices can view simultaneously.
- Settings survive power loss; unsaved changes within 2 s of a power cut may be lost (by design, to protect flash).
