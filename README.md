## **Components required**
- ESP32
- SHT31
- DUPONT WIRE

## 1. Wire it up
| SHT31 | ESP32 |
|---|---|
| VIN | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

| Relay module | ESP32 |
|---|---|
| IN | GPIO25 |
| VCC | 3V3 (or 5V per module) |
| GND | GND |

No extra wiring for the BOOT button — GPIO0 is already on the board.

## 2. Flash it
1. Install **Arduino IDE** + **ESP32 board support** (Boards Manager → “esp32” by Espressif, v2.0.0 or newer).
2. Open `ESP32_SHT31_Controller.ino`.
3. Tools → Board → **ESP32 Dev Module**.
4. Select the correct **COM port** and click **Upload**
5. or directly download the firmware and flash.
   - https://github.com/xiv3r/esp32-sht31/raw/refs/heads/main/esp32-sht31-0x0.bin

## 3. Connect
1. On your phone or PC, join WiFi:
   - **SSID:** `ESP32_SHT31`
   - **Password:** `12345678`
2. A captive-portal popup should appear automatically.
3. If not, open a browser and go to **http://192.168.4.1**

## 4. Use it
- **Dashboard** — live temperature/humidity, relay state, thresholds and hysteresis.
- **Relay** — choose Active HIGH/LOW, and Auto / Force ON / Force OFF.
- **WiFi** — change SSID and password (AP restarts automatically).
- **Factory Reset** — hold the **BOOT** button on the board for **5 seconds**, or click the button on the WiFi page.

## Notes
- Thresholds and settings survive power loss.
- The AP is always on — the device never joins your home WiFi as a client.
- Defaults are restored by factory reset.
