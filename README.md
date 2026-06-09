# PQ35 ABS Emulator (MK60EC1)

Emulador CAN do **MK60EC1** para **VW PQ35/PQ46** — **ESP32 + MCP2515**.

Lê velocidade via OBD **0x7E1 → 0x7E9 PID 0x0D** (09G) e transmite frames `Bremse_*` para restaurar **velocímetro** e **odômetro** sem o ABS original.

**v2.0.0** · CAN only · sem Wi‑Fi · serial silencioso  
**Testado:** Jetta 2.5 2009 · 09G · cluster PQ35

> **Não restaura frenagem ABS.** Apenas mensagens CAN. Use por sua conta e risco.

---

## Hardware

| MCP2515 | ESP32 |
|---------|-------|
| VCC | 5 V (VIN) |
| GND | GND |
| CS | GPIO 5 |
| SCK | 18 · MISO 19 · MOSI 23 |
| INT | GPIO 4 |

Cristal **8 MHz** · CAN **500 kbit/s** · remover jumper **120 Ω (J1)** do MCP · CAN H/L no barramento **Powertrain**.

Biblioteca: [ACAN2515](https://github.com/pierremolinaro/acan2515) · Core **ESP32 3.x**

---

## Calibração

Editar struct `Config` no `.ino`:

| Parâmetro | Default | Uso |
|-----------|---------|-----|
| `speedPanelScaleFactor` | `1.084` | Agulha (Bremse_1/3). `1.0` = sem viés |
| `odoImpulsesPerKmScale` | `0.2581` | Odômetro nos frames ABS |
| `wheelCircumferenceM` | `1.985` | Bremse_2 mid-rev |

---

## CAN

| ID | Frame | Período |
|----|-------|---------|
| 0x1A0 / 0x4A0 | Bremse_1 / 3 | 10 ms |
| 0x5A0 / 0x3A0 | Bremse_2 / 10 | 20 ms |
| 0x4A8–0x5B7 | Companion ESP | 20 ms |

RX: OBD 0x7E9 · Getriebe 0x440 / 0x540

Sinais DBC: [`docs/vw_pq.dbc`](docs/vw_pq.dbc)

**v2.0 — acelerações neutras:** Bremse_2 raw **127** · Bremse_8 **127 / 361 / 512** (evita luz ABS/ESP falsa)

---

## Changelog v2.0.0

- Firmware único em `pq35-abs-emulator/pq35-abs-emulator.ino`
- Removido Wi‑Fi / portal / variantes lite-full
- Corrigidas acelerações lateral e longitudinal (Bremse_2 e Bremse_8)
- Serial silencioso

Versão anterior com Wi‑Fi: [v1.2.3](https://github.com/pandolfipedro/pq35-abs-emulator/releases/tag/v1.2.3)

---

MIT — [`LICENSE`](LICENSE)
