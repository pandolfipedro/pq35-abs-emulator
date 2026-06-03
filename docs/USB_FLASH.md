# USB flashing (ESP32 + Arduino)

## Arduino IDE

1. Board: **ESP32 Dev Module**
2. Flash: **4 MB**
3. Partition: **Minimal SPIFFS (with OTA)** — required for **full** OTA uploads
4. Library: [ACAN2515](https://github.com/pierremolinaro/acan2515)

## Flash prebuilt `.bin`

Use **esptool** or Arduino IDE **Tools → ESP32 Sketch Tool → Upload**:

```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 \
  write_flash 0x10000 MK60EC1_emulator-1.2.3-lite.ino.bin
```

Offset `0x10000` is typical for the `min_spiffs` app partition — confirm with your partition table.

## Which binary?

| File | Use |
|------|-----|
| `MK60EC1_emulator-1.2.3-lite.ino.bin` | No Wi‑Fi, smallest, most stable on weak 5 V |
| `MK60EC1_emulator-1.2.3-full.ino.bin` | Hidden AP + live status + local OTA |

Always flash the variant you intend to run — do not mix lite/full OTA habits.

## Compile from source

```bash
./scripts/setup-dev.sh
./scripts/build.sh lite    # or full / all
```

Set `PQ35_WIFI_PORTAL=1` in Arduino IDE **compiler flags** for **full** if not using `build.sh`.
