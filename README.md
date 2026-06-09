# PQ35 ABS Emulator (MK60EC1)

Open-source **MK60EC1** CAN emulator for **VW PQ35/PQ46** — **ESP32 + MCP2515**.

Reads speed via OBD **0x7E1 → 0x7E9 PID 0x0D** (09G TCU) and transmits `Bremse_*` frames to restore the **speedometer** and **odometer** without the original ABS module.

**v2.0.0** · CAN only · no Wi‑Fi · silent serial  
**Tested on:** Jetta 2.5 2009 · 09G · PQ35 cluster

> **Does not restore ABS braking.** CAN messages only. Use at your own risk.

---

## Hardware

| MCP2515 | ESP32 |
|---------|-------|
| VCC | 5 V (VIN) |
| GND | GND |
| CS | GPIO 5 |
| SCK | 18 · MISO 19 · MOSI 23 |
| INT | GPIO 4 |

**8 MHz** crystal · **500 kbit/s** CAN · remove **120 Ω jumper (J1)** on the MCP module · connect CAN H/L to the **Powertrain** bus.

Library: [ACAN2515](https://github.com/pierremolinaro/acan2515) · **ESP32 3.x** core

---

## Calibration

Edit the `Config` struct at the top of the `.ino`:

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `speedPanelScaleFactor` | `1.084` | Needle (Bremse_1/3). `1.0` = no bias |
| `odoImpulsesPerKmScale` | `0.2581` | Odometer in ABS frames |
| `wheelCircumferenceM` | `1.985` | Bremse_2 mid wheel speed |

---

## CAN

| ID | Frame | Period |
|----|-------|--------|
| 0x1A0 / 0x4A0 | Bremse_1 / 3 | 10 ms |
| 0x5A0 / 0x3A0 | Bremse_2 / 10 | 20 ms |
| 0x4A8–0x5B7 | ESP companion | 20 ms |

RX: OBD 0x7E9 · Getriebe 0x440 / 0x540

DBC signals: [`docs/vw_pq.dbc`](docs/vw_pq.dbc)

**v2.0 — neutral accelerations:** Bremse_2 raw **127** · Bremse_8 **127 / 361 / 512** (prevents false ABS/ESP warning light)

---

## Changelog v2.0.0

- Single firmware file: `pq35-abs-emulator/pq35-abs-emulator.ino`
- Removed Wi‑Fi / web portal / lite-full variants
- Fixed lateral and longitudinal accelerations (Bremse_2 and Bremse_8)
- Silent serial (no debug output)

Previous Wi‑Fi build: [v1.2.3](https://github.com/pandolfipedro/pq35-abs-emulator/releases/tag/v1.2.3)

---

MIT — [`LICENSE`](LICENSE)
