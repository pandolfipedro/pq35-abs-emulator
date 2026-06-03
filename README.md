# PQ35 ABS Emulator (MK60EC1) — MCP2515

Open-source **MK60EC1** CAN emulator for **VW PQ35/PQ46** using **ESP32 + MCP2515 (TJA1050)**.

Reads vehicle speed via OBD Mode 01 PID `0x0D` (09G TCU) and injects `Bremse_*` frames to restore the **speedometer** and **odometer** without the original ABS module.

**Tested on:** VW Jetta 2.5 2009 · 09G automatic · PQ35 cluster

> **DIY / temporary** open-source solution. Commercial **SpeedCAN** (SN65) is a separate product.

---

## Firmware 1.2.3

| Variant | File | Wi‑Fi | Use |
|---------|------|-------|-----|
| **Lite** | [`releases/MK60EC1_emulator-1.2.3-lite.ino.bin`](releases/MK60EC1_emulator-1.2.3-lite.ino.bin) | No | Most stable, USB flash, lower power |
| **Full** | [`releases/MK60EC1_emulator-1.2.3-full.ino.bin`](releases/MK60EC1_emulator-1.2.3-full.ino.bin) | Hidden AP | Live status on phone + local OTA |

[GitHub Release v1.2.3](https://github.com/pandolfipedro/pq35-abs-emulator/releases/tag/v1.2.3)

### Full — Wi‑Fi portal

1. Add hidden network **`PQ35-Config`** / password **`pq35admin`**
2. Open **http://192.168.4.1** → login **`admin`** / **`admin`**
3. Home page shows speed, gear, OBD, odometer (updates ~1 s)
4. **Update Firmware** → upload the correct `.bin` (lite or full)

### Lite — no Wi‑Fi

Flash the `.bin` over USB and drive. Serial **115200**: `MK60 PQ35 ABS 1.2.3-lite`.

---

## Changes vs 1.1 (previous repo)

| | **1.1** | **1.2.x** |
|---|---------|-----------|
| MCP2515 INT | GPIO **2** | GPIO **4** (avoids boot LED conflict) |
| Speed pipeline | Basic | Panel cache, slew, panel factor ~1.084 |
| Wi‑Fi | No | Optional (**full**) |
| Portal / status | — | **full** only |

If your board still has INT on **GPIO 2** and 1.1 worked, you **do not need to upgrade** — or rewire INT to **GPIO 4** for 1.2.x.

---

## Warnings

- **Does not restore ABS braking** — CAN messages only.
- Wrong CAN wiring can cause bus-off or ECU damage.
- **Use at your own risk.**

---

## Hardware

- ESP32 + MCP2515 + TJA1050, **8 MHz** crystal
- MCP2515 INT → **GPIO 4** (not GPIO 2)
- **5 V ≥ 1 A** on VIN
- Remove **120 Ω jumper (J1)** on the MCP module in the car

Details: [`docs/HARDWARE.md`](docs/HARDWARE.md)

---

## Installation

### Prebuilt binary

[`docs/USB_FLASH.md`](docs/USB_FLASH.md)

### Build from source

```bash
./scripts/setup-dev.sh
./scripts/build.sh all    # builds lite + full into releases/
```

Requirements: Arduino CLI, ESP32 core 3.x, **ACAN2515** library.

---

## Repository layout

| Path | Contents |
|------|----------|
| [`MK60EC1_emulator/`](MK60EC1_emulator/) | Source 1.2.x (lite/full via `PQ35_WIFI_PORTAL`) |
| [`releases/`](releases/) | Prebuilt binaries |
| [`docs/`](docs/) | Hardware, flashing, CAN, DBC |

---

## CAN

IDs and overview: [`docs/CAN_PQ35.md`](docs/CAN_PQ35.md) · DBC: [`docs/vw_pq.dbc`](docs/vw_pq.dbc)

---

## License

MIT — see [`LICENSE`](LICENSE).

---

## History

- **v1.1** — minimal CAN-only firmware, INT GPIO 2
- **v1.2.2** — INT GPIO 4, 1.2 pipeline, lite/full variants
- **v1.2.3** — English docs and portal UI
