# PQ35 ABS Emulator

Emulador de módulo ABS para a plataforma **VW PQ35** (Jetta, Golf, Passat, Tiguan, etc.) usando um **ESP32 + MCP2515 (TJA1050)**. Lê a velocidade da TCU via OBD2 e injeta as mensagens ABS na rede CAN, permitindo que o cluster, o hodômetro e o Cruise Control funcionem normalmente sem o módulo ABS original.

> **Testado em:** VW Jetta 2.5 2009 — câmbio automático 09G — cluster PQ35

***

## Hardware necessário

| Componente | Observação |
|---|---|
| ESP32 (30 pinos) | Qualquer variante com SPI |
| Módulo MCP2515 + TJA1050 | Cristal de **8 MHz** |
| Conector OBD2 fêmea | Acesso ao barramento CAN (pinos 6 e 14) |

### Pinagem

| MCP2515 | ESP32 |
|---|---|
| CS | GPIO 5 |
| INT | GPIO 2 |
| SCK | GPIO 18 |
| MISO | GPIO 19 |
| MOSI | GPIO 23 |
| VCC | **VIN (5V)** |
| GND | GND |

> ⚠️ O TJA1050 exige alimentação entre 4.75–5.25V para gerar níveis diferenciais corretos no barramento CAN. Alimentar pelo **VIN (5V)** é obrigatório — o 3.3V está fora da especificação do transceiver e pode causar perda de frames. O SPI funciona normalmente em 3.3V porque o TJA1050 aceita lógica de entrada a partir de 2.0V.

Conectar `CAN H` e `CAN L` do módulo diretamente ao barramento CAN do carro (OBD2 pinos 6 e 14).

***

## Dependências

| Biblioteca | Versão testada |
|---|---|
| [ACAN2515](https://github.com/pierremolinaro/acan2515) por Pierre Molinaro | **2.1.5** |

Instalar pela Arduino IDE: *Ferramentas → Gerenciar Bibliotecas → ACAN2515*

***

## Como funciona

```
TCU (câmbio 09G)
      │  OBD2 PID 0x0D (velocidade)
      ▼
   ESP32 + MCP2515
      │  publica a cada 10/20ms:
      ├─ 0x1A0  Bremse_1  (velocidade + flag de freio)
      ├─ 0x4A0  Bremse_3  (4 rodas individuais)
      ├─ 0x5A0  Bremse_2  (velocidade + Wegimpulse)
      ├─ 0xDA0  ABS Alive (keepalive do módulo ABS)
      └─ 0x289  Bremse_5  (habilita Cruise Control)

ECU Motor (Motor_2 0x288)
      │  byte2 bit0 = Bremslichtschalter
      ▼
   ESP32 lê o estado do freio e reflete nos frames acima
```

### Mensagens transmitidas

| ID | Nome | Intervalo | Função |
|---|---|---|---|
| `0x1A0` | Bremse_1 | 10 ms | Velocidade principal + flag de freio |
| `0x4A0` | Bremse_3 | 10 ms | Velocidade das 4 rodas individualmente |
| `0x5A0` | Bremse_2 | 20 ms | Velocidade média + contador de pulsos ABS |
| `0xDA0` | ABS Alive | 20 ms | Keepalive do módulo ABS para o cluster |
| `0x289` | Bremse_5 | 20 ms | Sinalização ESP/freio para o Cruise Control |

### Mensagens recebidas

| ID | Nome | Dado lido |
|---|---|---|
| `0x7E9` | OBD2 TCU | Velocidade em km/h (PID 0x0D) |
| `0x288` | Motor_2 | Byte 2 bit 0 = pedal de freio pressionado |

***

## Configuração obrigatória

### Distance Impulse (hodômetro)

Leia o valor no VCDS: *Módulo 17 — Instruments → Codificação*

```cpp
#define VCDS_WEGIMPULS  21960.0f  // altere para o valor do seu carro
```

| Opção VCDS | Pulsos/km | Circunferência |
|---|---|---|
| 1 | 22188 | ~1938 mm |
| 2 | 22076 | ~1948 mm |
| **3 — padrão 205/55R16** | **21960** | **~1958 mm** |
| 4 | 21848 | ~1968 mm |
| 5 | 22304 | ~1928 mm |
| 6 | 22420 | ~1918 mm |
| 7 | 22532 | ~1908 mm |

***

## Detalhes de implementação

### Interpolação de velocidade

A velocidade OBD2 chega em inteiros (km/h) a cada ~80 ms. Um controlador proporcional suaviza a transição:

- Aceleração máxima: **50 km/h/s**
- Desaceleração máxima: **50 km/h/s**

### Zona morta — anti-hodômetro fantasma

Duas camadas de proteção:

1. **`lerCAN()`** — se `rawOBD <= 3.0 km/h`, `velAlvo = 0`
2. **`loop()`** — se `velCache <= 3.0 km/h`, `velEnvio = 0`

### Wegimpulse

O cluster PQ35 exige pulsos de roda acumulados junto com a velocidade no `0x5A0`. Sem eles o cluster detecta "velocidade > 0 mas zero pulsos" e zera o ponteiro.

```
pulsos/s = speedKmh × VCDS_WEGIMPULS / 3600
Contador 11-bit (0–2047), faz wrap automaticamente
```

### Leitura do freio via CAN

Lido do `Motor_2 (0x288)` byte 2 bit 0, sem nenhum fio extra. Refletido em:

- `0x1A0` byte 1 bit 3 — flag de freio para o cluster
- `0x289` byte 1 bit 1 — sinalização de freio para o Cruise Control

### Checksum 0x1A0

```
byte0 = XOR(bytes 1–7) XOR (CAN_ID & 0xFF) XOR (CAN_ID >> 8)
```

***

## Monitoramento serial

Baud rate: **115200**

```
[v9] vel=87.3 freio=0 weg=1423 obd=82ms reads=4821 txOk=98234 txErr=0 busOff=0
```

| Campo | Significado |
|---|---|
| `vel` | Velocidade enviada ao painel (km/h) |
| `freio` | Estado do pedal (`0` = solto, `1` = pressionado) |
| `weg` | Contador Wegimpulse (0–2047) |
| `obd` | Tempo desde a última resposta OBD2 (ms) |
| `reads` | Total de leituras OBD2 |
| `txOk` / `txErr` | Frames enviados com sucesso / com erro |
| `busOff` | Reinicializações por BUS-OFF |

***

## Histórico de versões

 v8 - Correções críticas:
 *   - 0x5A0 fator corrigido: 73.0 → 100.0 (igual 0x1A0/0x4A0, fonte: KMatrix oficial)
 *   - 0x5A0 BR2_Wegimpulse adicionado (bytes 6-7): contador 11-bit de pulsos ABS
 *   - 0xDA0 adicionado: "ABS vivo" exigido pelo cluster PQ35
 *   - Counter 0x1A0 e 0x5A0: sempre incrementa (evita duplicatas que causam rejeição)
 *   - OBD2 timeout: 500ms → 1000ms
 *   - Serial log: 1s → 2s (evita bloqueio coincidindo com timer do 0x1A0)

 v9 - Freio via CAN (sem fio extra):
 *   - Lê Motor_2 (0x288) byte2 bit0 = Bremslichtschalter (fonte: vw_golf_mk4.dbc)
 *   - Reflete estado do freio em 0x1A0 byte1 bit3 e 0x289 byte1 bit1
 *   - Filtros hardware: RXB0→0x7E9 (OBD2), RXB1→0x288 (Motor_2)
 *   - ACAN2515 v2.1.5: begin() com 2 máscaras + 4 filtros (range obrigatório: 3-6)
***

## Referências

- [The07k Wiki — CAN Bus Emulation for Cruise Control](https://the07k.wiki/wiki/Can-bus_emulation_for_cruise_control)
- [PQ35/46 KMatrix V5.20.6F](https://opengarages.org) — `BR2_Wegimpulse`, `BR2_mi_Radgeschw`
- [vw_golf_mk4.dbc — commaai/opendbc](https://github.com/commaai/opendbc) — `Bremslichtschalter`
- [Hackaday — CAN BUS Gaming Simulator](https://hackaday.io/project/6288) — `0xDA0`
- [ACAN2515 v2.1.5 — Pierre Molinaro](https://github.com/pierremolinaro/acan2515)

***

## Licença

MIT
