# Hardware — ESP32 + MCP2515 (PQ35)

## Pinout (firmware 1.2.3)

| MCP2515 | ESP32 | Notes |
|---------|-------|--------|
| VCC | **5 V (VIN)** | Module regulator ~5 V |
| GND | GND | |
| CS | GPIO **5** | |
| SCK | GPIO **18** | |
| MISO | GPIO **19** | |
| MOSI | GPIO **23** | |
| **INT** | GPIO **4** | **Do not use GPIO 2** (boot LED) |

> Firmware **1.1** used INT on GPIO 2. **1.2.x uses GPIO 4** — rewire if upgrading from 1.1.

## Power

- **5 V ≥ 1 A** on VIN for ESP + MCP in the car  
- **100–470 µF** cap on MCP 5 V rail  
- Remove **120 Ω jumper (J1)** on MCP module before connecting to vehicle CAN  
- Connect **CAN H/L** to **Powertrain** bus (not OBD only)

## Wi‑Fi (full variant only)

| Item | Value |
|------|--------|
| SSID | `PQ35-Config` (hidden) |
| AP password | `pq35admin` (changeable in portal) |
| Web login | `admin` / `admin` |
| URL | http://192.168.4.1 |

**Lite** variant has no Wi‑Fi — flash once via USB.

## Serial

115200 baud — expect `MK60 PQ35 ABS 1.2.3-lite` or `1.2.3-full`, then `MCP2515 OK @ 500k`.
