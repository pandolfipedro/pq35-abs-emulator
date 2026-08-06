# PQ35 ABS Emulator (MK60EC1)

Open-source **MK60EC1** CAN emulator for **VW PQ35/PQ46** — **ESP32 + MCP2515**.

Reads speed via OBD **0x7E1 → 0x7E9 PID 0x0D** (09G TCU) and transmits `Bremse_*` frames to restore the **speedometer** and **odometer** without the original ABS module.

**v2.0.0** · CAN only · no Wi‑Fi · silent serial
**Tested on:** Jetta 2.5 2009 · 09G · PQ35 cluster

> ⚠️ **Does not restore ABS braking.** CAN messages only. This device emulates instrument cluster signals only—actual ABS/ESP functionality is not restored. Use at your own risk.

---

## What This Does (and Doesn't)

✅ **Restores:**
- Speedometer needle (via Bremse_1/3 CAN frames)
- Odometer km count (via Bremse_2 impulse frames)
- Vehicle motion state (for automatic features, etc.)

❌ **Does NOT restore:**
- ABS braking or stability control
- Real-time acceleration telemetry
- OBD2 diagnostic codes
- Gear display (without Getriebe messages)

**Risk:** Without real ABS frames, braking systems may trigger warning lights or behave unexpectedly.

---

## Quick Start

### 1. Clone & Setup

```bash
git clone https://github.com/pandolfipedro/pq35-abs-emulator.git
cd pq35-abs-emulator
```

### 2. Install Dependencies

- [Arduino IDE](https://www.arduino.cc/en/software) v1.8.19+
- ESP32 Board Core v3.x ([installation guide](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html))
- [ACAN2515 Library](https://github.com/pierremolinaro/acan2515) (via Library Manager or manual)

### 3. Upload

1. Open `pq35-abs-emulator/pq35-abs-emulator.ino` in Arduino IDE
2. Select: **Board → ESP32 Dev Module** (or your board variant)
3. Set: **Upload Speed → 921600**, **Partition Scheme → Default**
4. Click **Upload**

### 4. Verify

- Open **Serial Monitor** (115200 baud) after reset
- Should see: `MK60 PQ35 ABS emu | SO NEUTRO | sem TP20/G85 dinamico`
- Followed by: `MCP2515 OK @ 500k | RX 7E9/440/540`

---

## Hardware

### Pinout

| MCP2515 | ESP32 | Notes |
|---------|-------|-------|
| VCC | 5 V (VIN) | Power from USB or car PSU (5V only) |
| GND | GND | Common ground |
| CS | GPIO 5 | Chip select (SPI) |
| SCK | GPIO 18 | Serial clock |
| MOSI | GPIO 23 | SPI data → MCP2515 SI |
| MISO | GPIO 19 | SPI data ← MCP2515 SO |
| INT | GPIO 4 | Interrupt (active-low) |
| CAN H | Powertrain CAN H | VW Powertrain bus (high) |
| CAN L | Powertrain CAN L | VW Powertrain bus (low) |

### Configuration

**MCP2515 Module:**
- **8 MHz crystal** (not 16 MHz—code is hardcoded for 8 MHz)
- **Remove J1 (120 Ω terminator jumper)**—only enable if MCP2515 is the final node on the bus
- **500 kbit/s** CAN speed (VW Powertrain standard)

**CAN Bus:**
- Connect to **Powertrain CAN** (500 kbit/s), not Comfort CAN
- Use twisted-pair shielded cable (preferably existing OBD2 extension)
- Typical vehicles have 120 Ω resistors at both ends—do not duplicate

**Power:**
- 5V USB or car PSU (stabilized, avoid noisy ground loops)
- Add 100 µF capacitor across VCC/GND near MCP2515 for stability

**Library:** [ACAN2515](https://github.com/pierremolinaro/acan2515) · ESP-IDF 4.x–5.x · Arduino Core ESP32 v3.x

---

## Calibration

Edit the `Config` struct at the top of `pq35-abs-emulator.ino`. **Default values are calibrated for Jetta 2.5 2009 (09G, PQ35).**

| Parameter | Default | Range | Purpose |
|-----------|---------|-------|---------|
| `speedPanelScaleFactor` | `1.084` | 0.9–1.2 | Speedometer needle bias correction. `1.0` = no correction. Adjust if needle reads high/low. |
| `odoImpulsesPerKmScale` | `0.2581` | 0.1–1.0 | Odometer km tracking scale. Adjust based on real distance vs. OBD impulses. |
| `wheelCircumferenceM` | `1.985` | 1.5–2.3 | Wheel perimeter (meters). Used for mid-wheel revs (Bremse_2). Calculate from tire sidewall or 2π×radius. |

### How to Find Your Values

**Speed scale (speedPanelScaleFactor):**
1. Drive at constant GPS speed (e.g., 100 km/h highway)
2. Record what your speedometer needle shows (e.g., 108.4 km/h)
3. Calculate: `speedPanelScaleFactor = GPS speed / needle speed` (e.g., 100 / 108.4 = 0.922)
4. Edit and test small adjustments (±0.01)

**Odometer scale (odoImpulsesPerKmScale):**
1. Record OBD speed at start of a known distance (e.g., highway km marker)
2. Drive 10+ km at steady speed, record ending OBD speed
3. Compare OBD impulses (in serial log) vs. real km traveled
4. Adjust scale proportionally (if OBD shows 2x real distance, scale down to 0.5)

**Wheel circumference (wheelCircumferenceM):**
1. Check tire sidewall (e.g., 205/60R16)
2. Use online calculator: `2π × (rim diameter + (2 × sidewall height))`
3. Or measure: mark wheel, roll forward 1 meter, count rotations
4. Circumference = 1 meter / rotations

---

## CAN Frames

### TX (Transmitted by Emulator)

| ID (hex) | Name | Period | Purpose |
|----------|------|--------|---------|
| 0x1A0 | Bremse_1 | 10 ms | ABS wheel speed (16-bit raw value in bits 17–31). Main speedometer source. |
| 0x4A0 | Bremse_3 | 10 ms | ABS wheel speed (redundant, same as Bremse_1). Some clusters require this. |
| 0x5A0 | Bremse_2 | 20 ms | Mid-wheel revolutions (16-bit) + time counter + odometer impulse count (6-bit). |
| 0x3A0 | Bremse_10 | 20 ms | Individual wheel impulse counts (10-bit each for VL, VR, HL, HR). Used by ABS module. |
| 0x4A8 | Bremse_5 | 10 ms | ESP frame—stillstand bit. Neutral acceleration (prevents false ESP warning). |
| 0x1AC | Bremse_8 | 20 ms | ESP frame—lateral & longitudinal accelerations. Neutral values (127, 361, 512). |
| 0x1A8 | Bremse_6 | 10 ms | ESP frame (3-byte). Neutral accel. |
| 0x5B7 | Bremse_11 | 20 ms | ESP frame. Neutral checksum. |
| 0x2A0 | Bremse_4 | 20 ms | ESP Haldex frame (optional, 3-byte). Only if `emitBremse4HaldexFrame = true`. |

### RX (Expected from Vehicle)

| ID (hex) | Name | Source | Use |
|----------|------|--------|-----|
| 0x7E9 | OBD Response | TCU (09G) | Speed data (PID 0x0D). Primary speed source. |
| 0x440 | Getriebe_1 | TCU | Gear position (wahl) for motion detection. |
| 0x540 | Getriebe_2 | TCU | Engaged gear & ganganzeige for park detection. |

**Signal details:** See [`docs/vw_pq.dbc`](docs/vw_pq.dbc) (CAN database, open in CANdb++, PCAN View, or DBC editor)

**v2.0 Safety:** Bremse_2 uses raw **127** (neutral), Bremse_8 uses **127 / 361 / 512** (neutral accel) to prevent false ABS/ESP warnings.

---

## Testing & Monitoring

### Serial Output

After upload, open **Serial Monitor (115200 baud)** to see real-time diagnostics:

```text
loop dt max(ms)/>12ms: 0.45/0 | km/h filt/panel0/panel/CANtx/panelTx/odo: 0.0/0.0/0.0/0.0/0.0/0.0 | impulsos: 0 | imp/km: 21.98 | calib spd/odoScale: 1.0840/0.2581 | RX buf: 0
Cambio wahl/eg: 255/255 | heap free/min: 245276/240280 | stack: 3376 | TX fail 1A0/4A0/5A0/3A0/esp: 0/0/0/0/0
```

### Key Metrics

- **loop dt max (ms)** — max loop cycle time per second (should be <1 ms; >12 ms = timing issue)
- **km/h filt / panel / CANtx / panelTx** — speed progression through filters (debug oscillation)
- **impulsos** — total odometer impulse count (should increment when moving)
- **TX fail** — CAN transmission errors (should be 0; non-zero = buffer overflow or bus collision)
- **heap free / stack** — memory usage (stack < 512 = stack overflow warning)

### Verification Steps

1. **Start engine** (or turn on ignition; 09G reads speed even at 0 km/h)
2. **Monitor impulsos** — should increase as speed rises
3. **Watch speedometer** — needle should track OBD speed ±5 km/h
4. **Check odometer** — compare with VCDS or cluster after known distance (e.g., 10 km highway)
5. **Look for warnings** — ABS/ESP light should not illuminate (if it does, adjust neutral accel values)

---

## Troubleshooting

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| **Serial shows "FATAL: MCP2515 nao inicializou"** | MCP2515 not communicating | Check SPI wiring (GPIO 18/19/23). Verify 8 MHz crystal (not 16 MHz). Try different USB cable. |
| **Speedometer stuck at 0** | No OBD speed received | Check Powertrain CAN connection. Verify TCU is powered. Monitor 0x7E9 with CAN analyzer. |
| **Speed oscillates wildly** | Calibration error or noisy OBD | Adjust `speedPanelScaleFactor` by ±0.01 increments. Check CAN ground quality. Add ferrite core to CAN cable. |
| **Odometer not incrementing** | `odoImpulsesPerKmScale` incorrect | Verify wheel circumference. Recalibrate scale using real distance vs. OBD speed. |
| **ABS or ESP warning light on** | Bremse_8 neutral values wrong | Verify `kBr8TolNeutral=127`, `kBr8LatNeutral=361`, `kBr8LongNeutral=512` in code. Recompile and upload. |
| **CAN TX failures increasing** | Buffer overrun or bus collision | Reduce frame rate (increase `*Zaehler` periods). Check for other CAN nodes transmitting at same ID. Verify 120 Ω terminator is only at bus ends. |
| **Gear position shows 255 (0xFF)** | Getriebe frames not received | Confirm Getriebe_1 (0x440) and Getriebe_2 (0x540) are on Powertrain CAN. Some TCUs may not broadcast; try disabling motion fusion (`useMotionCanFusion = false`). |
| **Serial shows "Stack baixa!" (low stack)** | Stack overflow risk | Reduce buffer sizes or disable non-critical features. Check for memory leaks. Reboot if persistent. |

---

## Supported Vehicles

| Vehicle | Platform | TCU | Cluster | Tested |
|---------|----------|-----|---------|--------|
| VW Jetta 2.5 (2009) | PQ35 | 09G | MK60EC1 | ✅ Yes |
| VW Rabbit / Golf (2006–2009) | PQ35 | 09G | MK60EC1 | Likely |
| Audi A3 (2006–2009) | PQ35 | 09G | MK60EC1 | Likely |
| VW Passat (2006–2010) | PQ46 | 09G | MK60EC1 | Likely |
| Skoda Octavia (2006–2008) | PQ35 | 09G | MK60EC1 | Likely |

**Other models:** Requires 09G TCU + MK60EC1 ABS module + Powertrain CAN @ 500 kbit/s

**To verify compatibility:**
1. Check your OBD response address (typically 0x7E9 for 09G)
2. Cross-reference DBC file: [`docs/vw_pq.dbc`](docs/vw_pq.dbc)
3. Confirm Bremse_1/2/3/10 frame IDs match
4. Test with monitoring tool (VCDS, CANape, or SocketCAN) before driving

---

## Changelog

### v2.0.0 (Current)
- **Single firmware:** Removed Wi-Fi, web portal, and lite/full variants
- **Neutral accelerations:** Fixed Bremse_8 (127 / 361 / 512) to prevent false ABS/ESP warnings
- **Silent serial:** Reduced debug output; monitoring logs every 1 second only
- **Cleaner codebase:** Removed ~1000 lines of dead code

### v1.2.3 (Previous)
- Wi-Fi variant with web portal for OTA updates
- Available at [v1.2.3 release](https://github.com/pandolfipedro/pq35-abs-emulator/releases/tag/v1.2.3)

---

## Disclaimer & Legal

⚠️ **Use at your own risk.** This firmware emulates an ABS module and instrument cluster signals. The author is **not responsible** for:

- **Loss of ABS/ESP braking** — This device does NOT restore actual ABS functionality, only instrument signals
- **Incorrect speedometer or odometer** — Miscalibration or vehicle incompatibility may cause false readings
- **Electrical damage** — Improper wiring or power supply issues
- **Accidents or injuries** — Resulting from use or malfunction of this device

**Before driving:**
1. Thoroughly test in a parking lot
2. Verify speedometer matches a known GPS source
3. Monitor for warning lights
4. Keep ABS functional (if possible) or accept braking risk

---

## License

MIT — See [`LICENSE`](LICENSE)

**Credits:**
- ABS emulation concept & calibration: community reverse-engineering
- ACAN2515 library: [Pierre Molinaro](https://github.com/pierremolinaro)
- ESP32 support: Espressif Systems

---

## Contributing

Issues, PRs, and vehicle compatibility reports welcome. Please include:
- Vehicle model & year
- TCU/cluster info
- Calibration values used
- Serial logs & errors

---

## Resources

- [ACAN2515 Documentation](https://github.com/pierremolinaro/acan2515)
- [ESP32 Arduino Core](https://docs.espressif.com/projects/arduino-esp32/en/latest/)
- [VW PQ35 DBC (CAN database)](docs/vw_pq.dbc)
- [OBD-II PID Reference](https://en.wikipedia.org/wiki/OBD-II_PIDs)
- [CAN Bus Troubleshooting](https://www.peak-system.com/fileadmin/media/pdf/English/Peak_CANBus_Troubleshooting_Guide.pdf)
