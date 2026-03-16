# PQ35 ABS Module Emulator

**ESP32 + MCP2515** emulator for the ABS module on **Volkswagen PQ35 platform vehicles** (Jetta MK5, Golf MK5, Passat B6, Audi A3 8P, Seat Leon MK2).

Developed and tested on a **VW Jetta 2.5 2009** with a burned-out ABS module (part suffix **AD**).

---

## The Problem

On PQ35 vehicles, the ABS module is responsible for broadcasting the vehicle speed signal on the CAN bus. When the module fails or is removed:

- Speedometer reads zero
- Odometer stops counting
- Fuel gauge becomes erratic
- ABS and ESP warning lights illuminate
- Gateway (J533) logs fault `00625 — Speed signal, No signal/communication`

This project replaces the faulty ABS module with an ESP32 + MCP2515 that reads the transmission speed from the powertrain CAN bus and re-broadcasts it as the original ABS module would.

---

## Important: OBD-II port does NOT work

The PQ35 platform has **three physically separate CAN buses**:

| Bus | Wire codes | What's on it |
|---|---|---|
| **Powertrain CAN** | B383 (High), B390 (Low) | ECU, TCM, ABS, cluster speed signal |
| Comfort CAN | B397 (High), B406 (Low) | Body control, doors, windows |
| Infotainment CAN | A178 (High), A179 (Low) | Radio, navigation |

The OBD-II port only exposes the **diagnostic bus** (gateway J533 pins 9/19), which is a separate channel used by scan tools. **Speed and ABS messages only travel on the Powertrain CAN** — they are never present on the OBD-II connector.

You must tap directly into the powertrain CAN wiring inside the car.

---

## Hardware

| Component | Notes |
|---|---|
| ESP32 DevKit V1 | Any 30 or 38-pin variant |
| MCP2515 module | With TJA1050 transceiver — 8 MHz or 16 MHz crystal |
| USB cable | Power and programming |

---

## Wiring

### ESP32 ↔ MCP2515

```
MCP2515           ESP32 DevKit V1
-------           ---------------
VCC       →       VIN  (5V — do NOT use the 3.3V pin)
GND       →       GND
CS        →       GPIO 5
SCK       →       GPIO 18
MOSI      →       GPIO 23
MISO      →       GPIO 19
INT       →       GPIO 4
```

> **Important:** Power the MCP2515 from the ESP32 **VIN** pin (5V from USB), not from the 3.3V pin. The TJA1050 transceiver is rated for 5V — running it at 3.3V causes unstable CAN communication.

### MCP2515 ↔ Vehicle powertrain CAN bus

```
MCP2515 CANH  →  Powertrain CAN High (orange/black wire — B383)
MCP2515 CANL  →  Powertrain CAN Low  (orange/brown wire — B390)
```

The easiest access point confirmed on the Jetta 2.5 2009 is directly at the **gateway J533 connector** (located in the driver's footwell, behind the dashboard):

| Gateway J533 pin | Signal |
|---|---|
| Pin 16 | Powertrain CAN High |
| Pin 6  | Powertrain CAN Low  |

Alternatively, tap the same twisted pair at the **ABS module connector** or anywhere along the powertrain CAN harness. The wire colors are consistently orange/black (High) and orange/brown (Low) throughout the car.

> **Do not use the OBD-II port.** The powertrain CAN bus is not routed there. Connecting via OBD-II will result in no data being received or transmitted.

---

## CAN Messages

### Transmitted (emulator → bus)

| ID | VAG name | Function | Rate |
|---|---|---|---|
| `0x5A0` | Bremse_2 | Speedometer + distance counter | 10 ms |
| `0x1A0` | Bremse_1 | Wheel speed (read by ECU, TCM) | 10 ms |
| `0x4A0` | Bremse_3 | ABS/ESP module keepalive | 50 ms |

### Received (bus → emulator)

| ID | VAG name | Function |
|---|---|---|
| `0x540` | Getriebe_2 | Transmission output speed (sent by TCM) |

### Encoding

```
0x5A0 speed:     raw = km/h × 148  (16-bit, little-endian)
0x5A0 distance:  50 counts per meter, overflow at 30000
0x1A0 speed:     raw = km/h × 148  (bytes 1–2)
```

> The distance counter in `0x5A0` bytes 5–6 **must** be incremented proportionally to speed. Without it, the speedometer needle rises for ~10 seconds and then drops — a well-documented PQ35 behavior, fixed in this implementation.

---

## Setup

### 1. Install the library

Install **MCP_CAN** by *coryjfowler* via Arduino IDE Library Manager:

```
Tools → Manage Libraries → search "MCP_CAN" → Install
```

### 2. Set the crystal frequency

Open the `.ino` file and find:

```cpp
CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)
```

- **8 MHz** crystal on your board → `MCP_8MHZ` (default)
- **16 MHz** crystal on your board → change to `MCP_16MHZ`

To identify your crystal, look for the small oval metal component near the MCP2515 chip. The number engraved on it indicates the frequency (`8.000` = 8 MHz, `16.000` = 16 MHz).

### 3. Upload

Select **ESP32 Dev Module** as the board in Arduino IDE and upload.

---

## Bench Testing

Without the car connected, the MCP2515 receives no CAN ACK and all transmissions fail with `[TX ERR]`. This is **normal CAN bus behavior** — not a bug.

To test on the bench, enable loopback mode:

```cpp
// In setup(), replace:
CAN.setMode(MCP_NORMAL);
// With:
CAN.setMode(MCP_LOOPBACK);
```

In loopback mode the chip ACKs its own frames and the errors disappear. **Switch back to `MCP_NORMAL` before installing in the car.**

---

## Speed Calibration

The factor `148.0` (defined as `FATOR_VELOCIDADE`) is the documented PQ35 value. If the speedometer reads differently from GPS:

1. Open Serial Monitor at 115200 baud
2. Note the `raw=` value while driving at a known speed
3. Calculate: `FATOR_VELOCIDADE = raw / gps_speed_kmh`
4. Update the constant and re-upload

An alternative factor reported for some PQ35 transmissions: `322.0`

---

## VCDS Preparation

Before installing, it is recommended to:

1. **Address 03 (ABS):** Clear any existing faults
2. **Address 19 (Gateway J533):** In Long Coding Helper, confirm the ABS/ESP module presence bit is **unchecked** if you have already disabled it via VCDS. This prevents the gateway from expecting a UDS diagnostic response from the ABS address.
3. After installing and starting the car, clear all DTCs with VCDS or OBDeleven.

### What this emulator does and does not fix

| Function | Result |
|---|---|
| Speedometer | ✅ Working |
| Odometer | ✅ Working |
| ABS warning light off | ✅ Clears (no more message timeout) |
| Fuel gauge stabilizes | ✅ Improves |
| Appears in VCDS autoscan as ABS module | ❌ No (no UDS diagnostic response) |
| Fault `01316` cleared from gateway | ⚠️ Depends on gateway long coding |

---

## Code Structure

```
pq35_abs_emulator.ino
│
├── setup()             — initializes MCP2515, configures RX filters
├── loop()              — controls transmission timing
│
├── lerCAN()            — reads 0x540 from TCM, extracts speed
├── verificarTimeout()  — zeroes speed if TCM stops transmitting
│
├── enviar5A0()         — transmits speedometer + distance counter
├── enviar1A0()         — transmits wheel speed for ECU/TCM
└── enviar4A0()         — transmits ABS/ESP keepalive
```

---

## Compatibility

Tested on VW Jetta 2.5 2009. Should work on any PQ35 platform vehicle:

| Vehicle | Status |
|---|---|
| VW Jetta MK5 (2005–2010) | Tested |
| VW Golf MK5 (2003–2009) | Expected to work |
| VW Passat B6 (2005–2010) | Expected to work |
| Audi A3 8P (2003–2012) | Expected to work |
| Seat Leon MK2 (2005–2012) | Expected to work |
| Skoda Octavia MK2 (2004–2013) | Expected to work |

If you test on any of these models, please open an issue with your findings.

---

## References

- [VW Instrument Cluster Controller — an-ven](https://github.com/an-ven/VW-Instrument-Cluster-Controller) — `0x5A0` formula tested on real hardware
- [CAN BUS Gaming Simulator — Hackaday](https://hackaday.io/project/6288) — VW PQ35 CAN ID mapping
- [Vehicle Reverse Engineering Wiki — Volkswagen](https://vehicle-reverse-engineering.fandom.com/wiki/Volkswagen) — real CAN log from VW Passat B6 PQ35
- [Autosport Labs MK5 CAN Preset](https://www.autosportlabs.com/2006-2010-vw-mk5-can-bus-preset-now-available/) — confirmed wheel speed channels
- [Gateway J533 pinout — MHH Auto](https://mhhauto.com/Thread-I-need-pinout-gateway-VAG-1K0907530H) — powertrain CAN pins confirmed
- [PQ35 CAN bus wiring — VW Forum UK](https://www.volkswagenforum.co.uk/threads/cable-path-for-canbus.42493/) — wire codes B383/B390 confirmed

---

## License

MIT — use freely, at your own risk. No warranty of any kind for any specific vehicle.

---

## Contributing

Pull requests are welcome. If you tested this on any PQ35 vehicle — successfully or not — please open an issue with your findings, the transmission type (manual/automatic), and the speed calibration factor that worked for you.
