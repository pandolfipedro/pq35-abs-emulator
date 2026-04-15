# PQ35 ABS Emulator (MK60EC1)

> Emulador de mensagens CAN do **módulo ABS/ESP (Bosch MK60EC1)** para plataforma **VW PQ35 / PQ46**, usando **ESP32 + MCP2515 (TJA1050)**.  
> Lê a velocidade real via **OBD-II Mode 01 PID `0x0D`** (TCU `09G`) no **CAN Powertrain** e injeta frames “Bremse_*” para restaurar **velocímetro** e **odometria** (cluster) sem o ABS original.

**Testado em:** VW Jetta 2.5 2009 · câmbio automático 09G · cluster PQ35  

---

## O problema: MK60EC1

O módulo **Bosch MK60EC1** (Jetta MK5 2009+, Golf MK5+, Beetle TSI) é conhecido por falhas eletrônicas terminais (ex.: erro `01130`, falha interna do G201), muitas vezes sem reparo economicamente viável.

Ao remover/isolar o módulo defeituoso, o barramento CAN deixa de receber as mensagens periódicas do ABS (tipicamente a cada 10–20 ms). Sintomas comuns:

- Velocímetro morto (ponteiro travado em 0)
- Hodômetro e consumo médio paralisados (dependem de impulsos/validação)
- Luzes de ABS/Brake/Traction acesas permanentemente

Este projeto **substitui o nó ABS no CAN**, publicando os frames necessários a partir da velocidade da TCU.

---

## ⚠️ Avisos importantes

- **Não restaura a frenagem ABS.** O sistema hidráulico/controle ABS permanece inoperante. Aqui é **somente emulação de mensagens CAN**.
- **Eletrônica automotiva exige cuidado.** Conexões erradas no CAN podem derrubar o barramento (*bus-off*) e/ou danificar ECUs.
- **Use por sua conta e risco.** Dirija com atenção e dentro das leis.

---

## Hardware

- **ESP32** (qualquer variante com SPI disponível)
- **Módulo MCP2515 + TJA1050** com cristal de **8 MHz** (não 16 MHz)

### Pinagem ESP32 ↔ MCP2515

| Sinal | ESP32 | MCP2515 |
|---|---:|---|
| Chip Select | GPIO 5 | CS |
| Interrupção | GPIO 2 | INT |
| Clock | GPIO 18 | SCK |
| MISO | GPIO 19 | SO |
| MOSI | GPIO 23 | SI |
| Alimentação | **VIN (5V)** | VCC |
| Referência | GND | GND |

### Alimentação (obrigatória em 5V)

O **TJA1050** exige **4.75–5.25V**. Alimente o módulo pelo **VIN (5V)** do ESP32.  
O SPI opera em 3.3V e o TJA1050 aceita o nível lógico de entrada normalmente.

### Resistor de terminação J1 (leia antes de ligar no carro)

Remova o jumper/resistor de **120 Ω** (`J1`) do módulo MCP2515 antes de conectar ao veículo.

O **CAN Powertrain** já tem duas terminações de 120 Ω nas extremidades (impedância total ~60 Ω). Manter o J1 adiciona um terceiro resistor em paralelo (~40 Ω), gerando reflexões e *bus-off* em certas condições.

### Ponto de conexão físico (CAN Powertrain)

Conectar `CAN H` e `CAN L` ao **barramento CAN Powertrain** (não no conector OBD2).  
Um ponto comum de acesso é no gateway abaixo do painel (acima do pedal do acelerador):

- `CAN H` → par trançado **laranja/preto**
- `CAN L` → par trançado **laranja/marrom**

---

## Dependências

- Biblioteca: **ACAN2515** (Pierre Molinaro)  
  - Link: `https://github.com/pierremolinaro/acan2515`

Na Arduino IDE: **Ferramentas → Gerenciar Bibliotecas →** buscar `ACAN2515`.

---

## Arquitetura (visão geral)

```text
TCU (câmbio 09G)
      │
      │  OBD-II (CAN Powertrain)
      │  Request:  0x7E1  (Mode 01 / PID 0x0D)
      │  Response: 0x7E9  (velocidade em km/h inteiro)
      ▼
ESP32 + MCP2515 (500 kbps, 11-bit)
      │
      │  Publica no CAN:
      ├─ 0x1A0  Bremse_1   (10 ms)  — velocidade principal (0.01 km/h) + contador
      ├─ 0x4A0  Bremse_3   (10 ms)  — velocidades das rodas (mesmo valor)
      ├─ 0x5A0  Bremse_2   (20 ms)  — “mid wheel speed” + timestamp + wegimpulse (11-bit) + impulszahl (6-bit)
      └─ 0x3A0  Bremse_10  (20 ms)  — wegimpulse por roda (10-bit) + counter + checksum XOR
```

---

## Mensagens CAN

### Transmitidas

| ID | Nome | Intervalo | Função |
|---|---|---:|---|
| `0x1A0` | Bremse_1 | 10 ms | Velocidade principal para cluster (0.01 km/h) + `Zaehler` (4-bit) |
| `0x4A0` | Bremse_3 | 10 ms | Velocidade das 4 rodas (mesmo valor) |
| `0x5A0` | Bremse_2 | 20 ms | Velocidade média (U/s), timestamp, `Wegimpulse_Vorderachse` (11-bit) e `Impulszahl` (6-bit) |
| `0x3A0` | Bremse_10 | 20 ms | `Wegimpulse` por roda (10-bit), `Zaehler` (4-bit) e checksum XOR em `byte0` |

### Recebidas (filtradas em hardware)

| ID | Nome | Dado extraído |
|---|---|---|
| `0x7E9` | OBD resposta TCU | Velocidade em km/h (PID `0x0D`, byte `data[3]`) |

O MCP2515 é configurado para **aceitar somente `0x7E9`** via filtros/máscaras de hardware (reduz carga de CPU e ruído).

---

## Configuração (no código)

Os parâmetros principais ficam no topo do `.ino`, no `struct Config`:

- **Pinos**: `mcpCs`, `mcpInt` (CS/INT do MCP2515)  
- **Cristal do MCP2515**: `quartzHz` (deve ser `8000000`)  
- **CAN bitrate**: `canBitrate` (padrão `500000`)  
- **Odometria**: `impulsesPerKm` (calibração do veículo)  
- **Raio efetivo do pneu**: `tyreRadiusM` (impacta a conversão de km/h → rev/s no `0x5A0`)  
- **Zona morta**: `standstillDeadBandKmh` (evita hodômetro “fantasma” com ruído de 1–2 km/h)

---

## Implementação (pontos técnicos)

### Velocidade via OBD + filtro

A TCU reporta velocidade como **inteiro (km/h)**. O firmware aplica um filtro de 1ª ordem (`speedTauMs`) e possui lógica de *hold/decay* em caso de timeout OBD (`obdTimeoutMs` / `obdHoldMaxMs`) para evitar degraus ou quedas abruptas.

### Odometria (impulsos)

O projeto integra a velocidade efetiva e acumula impulsos para preencher os campos de plausibilidade que o cluster usa.

Ideia base:

```text
impulsos/s = velocidade_kmh × impulsesPerKm / 3600
acumulador fracionário (double) → contador inteiro
```

### Checksum (Bremse_10 / `0x3A0`)

O checksum é calculado como XOR de `bytes 1..7` e gravado em `byte0`.

---

## Monitor serial

Baud rate: **115200**

Exemplo de log:

```text
km/h (filt/efet): 87.0/87.0 | impulsos: 123456 | RX buf: 0
TX fail 1A0/4A0/5A0/3A0: 0/0/0/0
```

- `filt/efet`: velocidade filtrada vs velocidade após zona morta (`standstillDeadBandKmh`)
- `impulsos`: total integrado (base para campos `Wegimpulse`)
- `TX fail ...`: tentativas de envio que falharam (rede dormindo/desconectada, etc.)

---

## Referências

- `https://github.com/pierremolinaro/acan2515`
- `vw_pq.dbc` (neste repositório) — layout de sinais `Bremse_*`
- KMatrix PQ35/46 (base conceitual de campos `Wegimpulse`)

---

## Licença

MIT

