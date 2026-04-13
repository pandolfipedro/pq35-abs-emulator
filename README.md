# PQ35 ABS Emulator

> Emulador de módulo ABS/ESP para a plataforma **VW PQ35 / PQ46** usando **ESP32 + MCP2515 (TJA1050)**.  
> Lê a velocidade da TCU via CAN Powertrain e injeta as mensagens ABS no barramento,  
> restaurando o cluster, o hodômetro e o Cruise Control sem o módulo ABS original.

**Testado em:** VW Jetta 2.5 2009 · câmbio automático 09G · cluster PQ35

---

## O problema: MK60EC1

O módulo **Bosch MK60EC1** (Jetta MK5 2009+, Golf MK5+, Beetle TSI) é conhecido por falhas eletrônicas terminais — erro `01130`, falha do sensor G201 interno — sem solução de custo viável.

Ao remover ou isolar o módulo defeituoso, o barramento CAN perde as mensagens que ele normalmente produzia a cada 10–20 ms. O resultado imediato:

- Velocímetro morto (ponteiro zerado permanentemente)
- Hodômetro e consumo médio paralisados
- Ponteiro de combustível instável (sem o filtro de suavização do ABS)
- Painel com luzes de ABS, Brake e Traction Control ativas de forma permanente
- **Cruise Control bloqueado** pela ECU do motor, que requer a presença de um nó ABS/ESP saudável no barramento para liberar o sistema

Este projeto substitui o MK60EC1 no barramento CAN. Ele lê a velocidade real da TCU (câmbio 09G) via protocolo CAN e reconstrói todas as mensagens que o ABS original enviaria, tornando o comportamento do veículo indistinguível do estado funcional original.

---

## ⚠️ Avisos importantes

> **Este projeto não restaura a frenagem ABS.** O sistema hidráulico de ABS permanece inoperante.  
> Apenas as mensagens CAN são emuladas. Dirija com atenção e dentro dos limites legais.

> **Conhecimento em eletrônica é necessário.** Conexões erradas no barramento CAN podem danificar ECUs do veículo.

---

## Hardware

| Componente | Detalhe |
|---|---|
| ESP32 (30 pinos) | Qualquer variante com SPI disponível |
| Módulo MCP2515 + TJA1050 | Cristal de **8 MHz** (não 16 MHz) |

### Pinagem ESP32 ↔ MCP2515

| Sinal | ESP32 | MCP2515 |
|---|---|---|
| Chip Select | GPIO 5 | CS |
| Interrupção | GPIO 2 | INT |
| Clock | GPIO 18 | SCK |
| MISO | GPIO 19 | SO |
| MOSI | GPIO 23 | SI |
| Alimentação | **VIN (5V)** | VCC |
| Referência | GND | GND |

### Alimentação

O TJA1050 exige **4.75–5.25V**. Alimentar pelo **VIN (5V)** é obrigatório.  
O barramento SPI opera normalmente em 3.3V — o TJA1050 aceita lógica de entrada a partir de 2.0V.

### Resistor de terminação J1 — leitura obrigatória

Remova o jumper/resistor de **120 Ω** (`J1`) da placa do MCP2515 antes de conectar ao carro.

A rede Powertrain CAN já possui dois resistores de 120 Ω nas extremidades físicas do barramento, resultando em **60 Ω** de impedância total. Manter o J1 insere um terceiro resistor em paralelo, reduzindo a impedância para ~40 Ω e causando reflexões de sinal e *bus-off* em altas rotações.

### Ponto de conexão físico

Conectar `CAN H` e `CAN L` ao **barramento CAN Powertrain** — não ao conector OBD2.  
O acesso mais limpo é pelo conector do gateway, abaixo do painel acima do pedal do acelerador:

- `CAN H` → par trançado **laranja/preto**
- `CAN L` → par trançado **laranja/marrom**

---

## Dependências

| Biblioteca | Versão |
|---|---|
| [ACAN2515](https://github.com/pierremolinaro/acan2515) — Pierre Molinaro | **2.1.5** |

**Arduino IDE:** Ferramentas → Gerenciar Bibliotecas → pesquisar `ACAN2515`

---

## Arquitetura

```text
TCU (câmbio 09G)
      │
      │  CAN — PID 0x0D via CAN Powertrain
      │  Request: 0x7E1  /  Response: 0x7E9
      ▼
   ESP32 + MCP2515
      │
      │  Publica a cada 10 ms:
      ├─ 0x1A0  Bremse_1  — velocidade principal + estado do freio
      ├─ 0x4A0  Bremse_3  — velocidade das 4 rodas individualmente
      ├─ 0x4A8  Bremse_5  — pressão hidráulica de frenagem simulada e Yaw Rate
      │
      │  Publica a cada 20 ms:
      └─ 0x5A0  Bremse_2  — velocidade média + acumulador de pulsos (Wegimpulse) + apagar luzes ABS

ECU Motor
      │
      │  Motor_2 — 0x288, byte2 bits 0-1 = dual brake switch
      ▼
   Leitura via filtro hardware RXB1
   Estado do freio refletido nativamente em 0x4A8 (Bremse_5) para o Piloto Automático
```

---

## Mensagens CAN

### Transmitidas

| ID | Nome | Intervalo | Função |
|---|---|---|---|
| `0x1A0` | Bremse_1 | 10 ms | Velocidade principal com checksum XOR + Flags de validação ASR/ESP |
| `0x4A0` | Bremse_3 | 10 ms | Velocidade das 4 rodas (mesmo valor, evita limp mode no Câmbio Auto) |
| `0x4A8` | Bremse_5 | 10 ms | Pressão hidráulica, Yaw Rate simulados e flag do pedal de freio (Libera Cruise Control) |
| `0x5A0` | Bremse_2 | 20 ms | Velocidade (Painel), apaga luzes Falha ABS/Freio + Acumulador Wegimpulse 11-bit |

### Recebidas

| ID | Nome | Dado extraído |
|---|---|---|
| `0x7E9` | CAN TCU | Velocidade em km/h inteiro — PID `0x0D` |
| `0x288` | Motor_2 | Byte 2, bits 0–1 — dual brake switch |

Os filtros de aceitação são configurados **nos registradores de hardware** do MCP2515:
- `RXB0` com máscara `0x7FF` → aceita somente `0x7E9`
- `RXB1` com máscara `0x7FF` → aceita somente `0x288`

O microcontrolador não processa nenhum frame fora dessas duas regras.

---

## Configuração

### Distance Impulse — calibração do hodômetro

Leia o valor no VCDS: **Módulo 17 → Instruments → Codificação**

```cpp
// pq35_abs_emulator_v9.ino — linha 44
#define VCDS_WEGIMPULS  21960.0f
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

---

## Implementação — pontos técnicos relevantes

### Interpolação de velocidade

A rede CAN retorna valores inteiros (km/h) com resolução de ~80 ms. Um controlador proporcional suaviza continuamente a transição entre leituras, evitando degraus que o cluster interpreta como sinal implausível.

- Aceleração máxima modelada: **50 km/h/s**
- Desaceleração máxima modelada: **50 km/h/s**

### Zona morta — prevenção de hodômetro fantasma

Duas camadas de proteção bloqueiam acúmulo de distância em marcha lenta:

1. **`lerCAN()`** — se `rawOBD ≤ 3.0 km/h`, força `velAlvo = 0`
2. **`loop()`** — se `velCache ≤ 3.0 km/h`, força `velEnvio = 0`

Isso elimina a cauda assintótica da curva de desaceleração antes que ela acumule pulsos indevidos no hodômetro.

### Acumulador Wegimpulse (0x5A0, bytes 6–7)

O cluster PQ35 valida plausibilidade comparando velocidade com pulsos acumulados de roda. Sem o campo `BR2_Wegimpulse` preenchido, o cluster detecta "velocidade > 0 com zero pulsos" e zera o ponteiro.

```text
pulsos/s = velocidade_kmh × VCDS_WEGIMPULS / 3600
Acumulador fracionário float → contador 11-bit com wrap em 2047
```

### Dual brake switch (0x288)

O VW usa dois switches de freio que ativam em momentos distintos do curso do pedal. A sequência de bits ao frear é `00 → 01 → 03 → 02 → 00`. A leitura correta usa `(byte2 & 0x03)` — capturar apenas bit0 perde o estado intermediário `0x02`.

O estado é refletido de volta para a eletrônica em:
- `0x4A8` (Bremse_5) bytes 2–3: injeta a pressão virtual (Ex: `0x0400` bar).
- `0x4A8` (Bremse_5) byte 5: Ativa o bit nativo de Lâmpada de freio (`0x08`).
*(Atenção: A adoção de uso da flag de freio "0x18" na `0x1A0` quebrava a plausibilidade e derrubava a agulha de velocidade, sendo descontinuada na vFINAL)*.

### Checksum — Bremse_1 (0x1A0)

```text
byte0 = XOR(bytes 1–7) XOR (CAN_ID & 0xFF) XOR (CAN_ID >> 8)
```

### Recuperação de BUS-OFF

O loop monitora o contador de erros de transmissão do MCP2515 a cada 2 s. Se ultrapassar 180, reinicializa o chip automaticamente sem reiniciar o ESP32.

---

## Monitor serial

Baud rate: **115200**

```text
[vFINAL] vel=87.3 freio=0 
[vFINAL] vel=0.0 freio=0 | ERRO CAN: txErr=54 busOff=0
```

| Campo | Descrição |
|---|---|
| `vel` | Velocidade sendo enviada ao painel neste momento (km/h) |
| `freio` | Estado do pedal lido do `0x288` (`0` = solto, `1` = pressionado) |
| `ERRO CAN`| Omitido quando saudável. Se aparecer, mostra falhas do MCP2515 (ex: carro em modo Sleep impedindo envio). |
| `txErr` | Acúmulo de frames barrados (rede dormindo ou desconectada) |
| `busOff`| Reinicializações por falha severa (> 0 indica curtos físicos de hardware ou presença incorreta do J1) |

---

## Histórico de versões

### vFINAL (K-Matrix Compliance)
- **Sintaxe K-Matrix Oficial:** Estruturamento de Checksums, counters (Pulsos vivos) e flags para 1A0, 4A0, 4A8, e 5A0, clonando a placa MK60 integralmente.
- **Remoção Anti-Diagnóstico:** Deleção da mensagem fantasma (0xDA0) que causava hodômetro disparar devido ao acionamento do Modo Manutenção / Teste de Bancada do painel. Extinção da ID 0x289 (Restrita a MQB).
- **Ressurreição do Cruise Control:** Injeção de BR5_Bremsdruck (Pressão), BR5_Sta_Gierrate e cálculo de Cheksum implementados do zero no 0x4A8. Lâmpadas do ABS enraizadas como "Off" (0x05) no 0x5A0.
- **Estabilidade do Velocímetro:** Abolido erro experimental `0x18` no Byte 1 (`1A0`) que forçava a agulha ao declínio.

### v9.0
- **Dual brake switch adicionado:** leitura passa a usar `(byte2 & 0x03)` capturando ambos os switches. Emissão primária do frame simulado `0x4A8`.

### v8.1
- Leitura do pedal de freio migrada para CAN (`0x288`) — elimina o fio secundário ao ESP32
- Filtros de hardware dual (`RXB0/RXB1`) configurados no MCP2515
- Função `iniciarCAN()` centralizada — usada tanto no `setup()` quanto na recuperação de BUS-OFF

### v8.0
- Fator de escala do `0x5A0` corrigido: `73.0 → 100.0` (KMatrix PQ35/46 V5.20.6F)
- Acumulador `BR2_Wegimpulse` implementado nos bytes 6–7 do `0x5A0`
- `0xDA0` ABS Alive adicionado: keepalive de 20 ms evita limp mode no cluster

---

## Referências

- [The07k Wiki — CAN Bus Emulation for Cruise Control](https://the07k.wiki/wiki/Can-bus_emulation_for_cruise_control)
- [PQ35/46 KMatrix V5.20.6F](https://opengarages.org) — `BR2_Wegimpulse`, `BR2_mi_Radgeschw`
- [vw_golf_mk4.dbc — commaai/opendbc](https://github.com/commaai/opendbc) — `Bremslichtschalter` (`0x288`)
- [OpenStreetMap Wiki — VW-CAN](https://wiki.openstreetmap.org/wiki/VW-CAN) — dual brake switch, flag `0x18`, `0x4A8`
- [Hackaday — CAN BUS Gaming Simulator v3.0](https://hackaday.io/project/6288) — `0xDA0` ABS Alive
- [ACAN2515 — Pierre Molinaro](https://github.com/pierremolinaro/acan2515)

---

## Licença

MIT
