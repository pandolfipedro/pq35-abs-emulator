# PQ35 ABS Emulator

Emulador de módulo ABS / ESP para a plataforma **VW PQ35 / PQ46** usando um **ESP32 + MCP2515 (TJA1050)**. Lê a velocidade da TCU via OBD2 e injeta as mensagens ABS na rede CAN, permitindo que o cluster, o hodômetro e o Cruise Control funcionem normalmente sem o módulo ABS original.

> **Testado em:** VW Jetta 2.5 2009 — câmbio automático 09G — cluster PQ35

## 🚀 Motivação (Bypass do MK60EC1)

O infame módulo de ABS VAG **MK60EC1** (presente no Jetta MK5 2009+, Golf MK5+, Beetle TSI) frequentemente apresenta falhas terminais crônicas e caríssimas de reparar em sua placa eletrônica (ex: erro 01130 ou falha irreversível do sensor G201 interno). 

Ao ser forçado a desabilitar, desplugar a CAN ou isolar o bloco eletrônico defeituoso do ABS para poder rodar com o carro, toda a rede de comunicação de segurança do veículo desmorona. O resultado punitivo imediato inclui:
- Cluster (Painel de Instrumentos) com árvore de natal de falhas irreversível ("ABS", "Brake", "Traction").
- Ponteiro de velocímetro morto e pneu murcho ativo.
- Hodômetro principal e Trip paralisados.
- **Perda total do Cruise Control** (Piloto Automático), já que a ECU do motor bloqueia seu funcionamento preventivamente se não detectar a presença de um módulo ESP/ABS saudável comunicando via CAN.

Este projeto atua como uma "bomba virtual de ABS" de baixo custo e invisível no barramento CAN. Ele converte secretamente a velocidade verdadeira das engrenagens do câmbio automático (TCU via OBD2) nos datagramas vitais que o ABS original enviaria, restituindo integralmente a funcionalidade do Cluster, do Hodômetro e destravando de volta o Cruise Control para uso normal em estradas.

***

## 🛠️ Hardware necessário

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

> ⚠️ O TJA1050 exige alimentação entre 4.75–5.25V. Alimentar pelo **VIN (5V)** é obrigatório — o 3.3V está fora da especificação do transceiver e pode causar perda de frames. O SPI funciona normalmente em 3.3V pois o TJA1050 aceita lógica de entrada a partir de 2.0V.
>
> **ATENÇÃO CRÍTICA (Pino de Terminação J1):** Remova o jumper/resistor de 120 ohms (`J1`) da placa mecânica azul do MCP2515 se for conectá-la direto à Drivetrain CAN do carro. A rede de Powertrain já possui os resistores de terminação internos nas pontas da malha equivalendo a ~60 ohms; manter este jumper insere um terceiro pino, derrubando sua rede CAN num tripé de 40 Ohms e resultando em *bus-off* fatais em regimes de alta rotação.

Conectar `CAN H` e `CAN L` diretamente ao barramento CAN do carro (OBD2 pinos 6 e 14).

***

## ⚙️ Dependências

| Biblioteca | Versão testada |
|---|---|
| [ACAN2515](https://github.com/pierremolinaro/acan2515) por Pierre Molinaro | **2.1.5** |

Instalar pela Arduino IDE: *Ferramentas → Gerenciar Bibliotecas → ACAN2515*

***

## 🧠 Como funciona

```text
TCU (câmbio 09G)
      │  OBD2 PID 0x0D (velocidade)
      ▼
   ESP32 + MCP2515
      │  publica a cada 10/20ms:
      ├─ 0x1A0  Bremse_1  (velocidade + flag de freio)
      ├─ 0x4A0  Bremse_3  (4 rodas individuais)
      ├─ 0x4A8  Bremse_4  (pressão hidráulica de frenagem simulada)
      ├─ 0x5A0  Bremse_2  (velocidade + Wegimpulse nato)
      ├─ 0xDA0  ABS Alive (keepalive vital do módulo ABS)
      └─ 0x289  Bremse_5  (ESP flag habilita Cruise Control)

ECU Motor (Motor_2 0x288)
      │  byte2 bits 0-1 = dual brake switch
      ▼
   ESP32 lê nativamente o estado elétrico do freio e o retransmite na emulação
```

### Mensagens transmitidas

| ID | Nome | Intervalo | Função |
|---|---|---|---|
| `0x1A0` | Bremse_1 | 10 ms | Velocidade principal + flag de freio unificada |
| `0x4A0` | Bremse_3 | 10 ms | Velocidade das 4 rodas individualmente |
| `0x4A8` | Bremse_4 | 10 ms | Pressão hidráulica simulada em Bar (Plaisibilidade/Cruise) |
| `0x5A0` | Bremse_2 | 20 ms | Velocidade média + contador de pulsos aculumados do ABS |
| `0xDA0` | ABS Alive | 20 ms | Keepalive de varredura livre do módulo ABS para o cluster |
| `0x289` | Bremse_5 | 20 ms | Sinalização redundante ESP OK/Freio para o Engine (Cruise) |

### Mensagens interceptadas 

| ID | Nome | Dado lido |
|---|---|---|
| `0x7E9` | OBD2 TCU | Leitura de velocidade bruta em km/h de engrenagem (PID 0x0D) |
| `0x288` | Motor_2 | Leitura de pedal (Byte 2 bits 0-1 do "dual brake switch") |

Os Acceptance Filters são engatados **diretamente nos registradores de hardware** rxm0 e rxm1 do próprio MCP2515 (RXB0 → `0x7E9`, RXB1 → `0x288`), blindando o buffer de loops desnecessários e protegendo o microcontrolador de inundações.

> ⚠️ O `0x3D0` (Immobilizer) **NÃO deve ser transmitido** — a ECU original do motor já envia e gerencia este ID no barramento. Duplicar ou forjar causaria conflito agudo de arbitragem CAN e imoblização acidental.

***

## 💻 Configuração obrigatória

### Distance Impulse (calibração do hodômetro)

Leia o valor original armazenado no VCDS do seu carro varrendo: *Módulo 17 — Instruments → Codificação*

```cpp
#define VCDS_WEGIMPULS  21960.0f  // Ex: Altere esta const para o número lido no seu VCDS
```

| Opção de Fábrica | Pulsos por Km Perfeito | Circunferência de Raio Equivalente |
|---|---|---|
| Variante 1 | 22188 | ~1938 mm |
| Variante 2 | 22076 | ~1948 mm |
| **Variante 3 (Padrão 205/55R16)** | **21960** | **~1958 mm** |
| Variante 4 | 21848 | ~1968 mm |
| Variante 5 | 22304 | ~1928 mm |
| Variante 6 | 22420 | ~1918 mm |
| Variante 7 | 22532 | ~1908 mm |

***

## 🔬 Detalhes avançados de implementação

### Interpolação passiva contínua

A velocidade pura recebida pela OBD2 (inteiros sem fração em km/h) chega ao chip a cada ~80 ms. Um controlador matemático proporcional desliza a transição da interpolação garantindo que o painel nunca sofra degraus abruptos em milissegundos (o que faria o microcontrolador do cluster disparar flag de "Implausible Signal" e derrubar instantaneamente o ponteiro).

- Taxa de aceleração contínua coberta: **50 km/h por segundo**
- Taxa limite realística de freio: **50 km/h por segundo**

### Defesa de Zona-Morta (Fix para Hodômetro Fantasma)

Nossa arquitetura conta com duas camadas rigorosas montadas em profundidade que destroem oscilações (geradas por tremida em marcha-lenta que flutuam as leituras da transmissão):

1. **Gate `lerCAN()`** — Se o ruído da engrenagem oscilar no patamar de `rawOBD <= 3.0 km/h`, cortamos o cordão umbilical ali setando a matriz primária `velAlvo = 0` instantaneamente.
2. **Selo de Proteção `loop()`** — Uma barreira final com o hard-cut `velEnvio = (velCache <= 3.0 km/h) ? 0.0f : velCache;` garante que a curva exponencial assintótica que se formaria no rodapé atinja inatividade perfeita.

### Wegimpulse (A essência do Acumulador BR2_Wegimpulse)

Muitos tentam emular ABS injetando 73.0 constantes. Isso é matematicamente obsoleto porque o cluster genuíno PQ35 exige receber frações de **pulsos acumulados (e não velocidades flutuantes)** que são transportados de 20ms em 20ms ao redor de um bit-wrapping exato de 11 slots localizados no pacote `0x5A0`.

```text
Fórmula original do processador de pulsos de Roda por segundo na ECU:
pulsos/s = speedKmh × VCDS_WEGIMPULS / 3600
Contador livre de repasse estrito em base 11-bit (0–2047)
```

### Emulação Hidráulica e de Fricção (Motor_2 0x288)

A linha da Volkswagen usa invariavelmente **dois switches de redundância** no pedal do freio (sensores de aproximação que ativam assimetricamente um após o outro, com sequência em tempo-real na malha de `00→01→03→02→00`).

Sem encostar o alicate para pescar nenhum fio extra dos pedais, o código grampeia o Frame original de transmissão de injeção da ECU `0x288` (byte 2, bits 0-1) e reflete digitalmente com latência próxima a zero o acoplamento físico para a nossa bomba irreal:

- O datagrama transaciona o bits de `Bremslichtschalter` duplo no pacote `0x1A0` de freios: `0x18` (bits 3 e 4 ativados)
- Ao mesmo tempo, lança pressão falsa de linha na barra `0x4A8` 
- Apaga a flag Cruise Control de redundância no pacote `0x289` byte 1 espelhadamente.

### Metodologia do Checksum (Bremse_1)

O byte 0 de validação XOR do VAG exige matriz de decifragem que mistura o identificador do CAN (High/Low) invertidos com os pacotes internos:
```text
byte0 = XOR(bytes 1–7) XOR (CAN_ID & 0xFF) XOR (CAN_ID >> 8)
```

***

## 📊 Terminal de depuração serial

Monitore o status do projeto setando a IDE para Baud rate: **115200**

```text
[v9.0] vel=87.3 freio=0 weg=1423 obd=82ms reads=4821 txOk=98234 txErr=0 busOff=0
```

| Métrica | O que ela te diz? |
|---|---|
| `vel` | Variável principal de velocidade calculada sendo enviada na CAN de momento em km/h |
| `freio` | Reflexo instantâneo do duplo-pedal interceptado (`0` = Solto, `1` = Pedal pisado) |
| `weg` | Fila do acumulador Wegimpulse varrendo a contagem infinita de `0 a 2047` |
| `obd` | Latência exata mensurada desde a última colheita viável do OBD2 da TPU (ms) |
| `reads` | Volume massivo de frames inteiros recebidos do protocolo 0x7e9 |
| `txOk` / `txErr` | Contador de buffers despachados integralmente em hardware vs buffers que entraram em colisão CAN |
| `busOff` | Intervenções drásticas e obrigatórias de soft-reset realizadas porque placa sofreu saturação no RX/TX devido ao Jumper J1 soldado |

***

## 📚 Histórico e Log 

### v9.0
- Refatoração do Dual brake switch no pacote (`0x288`): Leitura agora captura `(byte2 & 0x03)` em vez do truncado bit0, assegurando resgate lógico das variavez de varredura `01→03`.
- Conserto de flag `0x1A0`. O freio reage unificado como `0x18` e preenche (bits 3+4 espelhadores).
- Feature adicionada: Emulação virtual no bus de `0x4A8` (Bremse_4) recriando pressão cirúrgica em bar e escoamento virtual de linhas hidráulicas em tempo real, exigidos pelas travas mais modernas de engate do Cruise Control.
- Pacote perigoso `0x3D0` estritamente expurgado pela equipe para sanidade da colisão do Immobilizer base.

### v8.1
- Feature arquitetura CAN Habilite: Abstraído o fio secundário digital. O pedal de breaklight nativo já foi lido e transferido puramente pelas redes em multiplex pelo interceptador `Motor_2`.
- Os Acceptance Filters da base de placa do MCP2515 foram destrinchados utilizando dual masks nativos RX hardware (`RXB0→0x7E9` / `RXB1→0x288`), extinguindo lag CPU.
- Agrupamento nativo da função `iniciarCAN()`.

### v8.0
- Fix monumental de variação e Drift onde o `Fator do Hodômetro` no pacote de milhagem `0x5A0` utilizava multiplicador antigo e truncado. Passamos a injetar nativamente escala FATOR = `100.0`.
- Loop de acúmulo contínuo re-escrito de zero no pino `BR2_Wegimpulse`, integrando buffers sub-decimais flutuantes. Envelopados na ponta dos bytes de telemetria `6–7`.
- Proteção nativamente escrita contra colapso no buffer dos relógios em VAG `Heartbeat (ABS ALive) - 0xDA0` lançado na malha a cada `20ms`. Cluster de painel nunca morrerá em *limp mode*.

***

## 🔗 Referências

O crédito gigantesco das traduções das "roseta stones" VAG pertencem aos esforços incansáveis destas comunidades abertas:
- [The07k Wiki — CAN Bus Emulation for Cruise Control](https://the07k.wiki/wiki/Can-bus_emulation_for_cruise_control)
- [PQ35/46 KMatrix V5.20.6F](https://opengarages.org) — `BR2_Wegimpulse`, `BR2_mi_Radgeschw`
- [vw_golf_mk4.dbc — commaai/opendbc](https://github.com/commaai/opendbc) — `Bremslichtschalter` (`0x288`)
- [OpenStreetMap Wiki — VW-CAN](https://wiki.openstreetmap.org/wiki/VW-CAN) — dual brake switch, `0x1A0` flag `0x18`, `0x4A8`
- [Hackaday — CAN BUS Gaming Simulator v3.0](https://hackaday.io/project/6288) — `0xDA0` ABS Alive
- [ACAN2515 v2.1.5 — Pierre Molinaro](https://github.com/pierremolinaro/acan2515)

***

## Licença

Projeto desenvolvido abertamente em MIT (Open-Source)
