# PQ35 ABS Module Emulator

> 🇺🇸 [Read in English](README.md)

Emulador do módulo ABS para veículos **Volkswagen plataforma PQ35** (Jetta MK5, Golf MK5, Passat B6, Audi A3 8P, Seat Leon MK2) usando **ESP32 DevKit V1** e **MCP2515**.

Desenvolvido e testado em um **VW Jetta 2.5 2009** com módulo ABS queimado (sufixo **AD**).

---

## O problema

No PQ35, o módulo ABS é responsável por transmitir o sinal de velocidade no barramento CAN. Com o módulo queimado ou ausente:

- Velocímetro para em zero
- Odômetro para de contar
- Ponteiro de combustível fica errático
- Luzes de ABS e ESP acendem no painel
- Gateway (J533) registra o fault `00625 — Speed signal, No signal/communication`

Este projeto substitui o módulo ABS defeituoso por um ESP32 + MCP2515 que lê a velocidade da transmissão no barramento CAN e a retransmite como o módulo ABS original faria.

---

## Importante: o conector OBD-II NÃO funciona

A plataforma PQ35 possui **três barramentos CAN fisicamente separados**:

| Barramento | Cores dos fios | O que trafega |
|---|---|---|
| **Powertrain CAN** | Laranja/preto (High), Laranja/marrom (Low) | ECU, TCM, ABS, sinal de velocidade do painel |
| Comfort CAN | — | Módulos de carroceria, portas, vidros |
| Infotainment CAN | — | Rádio, navegação |

O conector OBD-II expõe apenas o **barramento de diagnóstico** (pinos 9/19 do gateway J533), um canal separado utilizado por scanners. **As mensagens de velocidade e ABS trafegam apenas no Powertrain CAN** — elas nunca estão presentes no conector OBD-II.

É necessário conectar diretamente na fiação do Powertrain CAN dentro do carro.

---

## Hardware necessário

| Componente | Observação |
|---|---|
| ESP32 DevKit V1 | Qualquer variante com 30 ou 38 pinos |
| Módulo MCP2515 | Com transceiver TJA1050 — cristal 8 MHz ou 16 MHz |
| Cabo USB | Alimentação e programação |

---

## Ligações

### ESP32 ↔ MCP2515

```
MCP2515           ESP32 DevKit V1
-------           ---------------
VCC       →       VIN  (5V — NÃO usar o pino 3.3V)
GND       →       GND
CS        →       GPIO 5
SCK       →       GPIO 18
MOSI      →       GPIO 23
MISO      →       GPIO 19
INT       →       GPIO 4
```

> **Importante:** Alimente o MCP2515 pelo pino **VIN** do ESP32 (5V via USB), não pelo pino 3.3V. O transceiver TJA1050 é especificado para 5V — usar 3.3V causa instabilidade na comunicação CAN.

### MCP2515 ↔ Barramento Powertrain CAN do veículo

```
MCP2515 CANH  →  CAN High do powertrain (fio laranja/preto — B383)
MCP2515 CANL  →  CAN Low do powertrain  (fio laranja/marrom — B390)
```

O ponto de acesso mais fácil confirmado no Jetta 2.5 2009 é diretamente no **conector do gateway J533** (localizado no painel, no lado do motorista):

| Pino do gateway J533 | Sinal | Cor do fio |
|---|---|---|
| Pino 16 | Powertrain CAN High | Laranja/preto |
| Pino 6  | Powertrain CAN Low  | Laranja/marrom |

Também é possível acessar o mesmo par trançado no **conector do módulo ABS** ou em qualquer ponto ao longo da fiação do Powertrain CAN.

> **Não use o conector OBD-II.** O barramento Powertrain CAN não é roteado até ele. Conectar via OBD-II resultará em nenhum dado sendo recebido ou transmitido.

---

## Mensagens CAN

### Transmitidas (emulador → barramento)

| ID | Nome VAG | Função | Periodicidade |
|---|---|---|---|
| `0x5A0` | Bremse_2 | Velocímetro + contador de distância | 10 ms |
| `0x1A0` | Bremse_1 | Wheel speed (lido pelo ECU e TCM) | 10 ms |
| `0x4A0` | Bremse_3 | Keepalive do módulo ABS/ESP | 50 ms |

### Recebidas (barramento → emulador)

| ID | Nome VAG | Função |
|---|---|---|
| `0x540` | Getriebe_2 | Velocidade de saída da transmissão (enviada pelo TCM) |

### Fórmulas de codificação

```
Velocidade 0x5A0:   raw = km/h × 148  (16 bits, little-endian)
Distância 0x5A0:    50 counts por metro, overflow em 30000
Velocidade 0x1A0:   raw = km/h × 148  (bytes 1–2)
```

> O contador de distância nos bytes 5–6 do `0x5A0` **deve** ser incrementado proporcionalmente à velocidade. Sem isso, o ponteiro do velocímetro sobe por ~10 segundos e cai — comportamento bem documentado no PQ35, corrigido nesta implementação.

---

## Instalação

### 1. Instalar a biblioteca

Instale a biblioteca **MCP_CAN** by *coryjfowler* via Arduino IDE Library Manager:

```
Tools → Manage Libraries → buscar "MCP_CAN" → Install
```

### 2. Configurar o cristal

Abra o arquivo `.ino` e localize:

```cpp
CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)
```

- Cristal de **8 MHz** na placa → `MCP_8MHZ` (padrão)
- Cristal de **16 MHz** na placa → troque para `MCP_16MHZ`

Para identificar o cristal, procure o componente metálico oval soldado próximo ao chip MCP2515. O número gravado nele indica a frequência (`8.000` = 8 MHz, `16.000` = 16 MHz).

### 3. Upload

Selecione **ESP32 Dev Module** como placa no Arduino IDE e faça o upload.

---

## Teste em bancada

Sem o carro conectado, o MCP2515 não recebe ACK de nenhum nó CAN e todos os envios falham com `[TX ERR]`. Isso é **comportamento normal do protocolo CAN** — não é bug.

Para testar em bancada, ative o modo loopback:

```cpp
// Em setup(), troque:
CAN.setMode(MCP_NORMAL);
// Por:
CAN.setMode(MCP_LOOPBACK);
```

No loopback o chip faz ACK dos próprios frames e os erros somem. **Volte para `MCP_NORMAL` antes de instalar no carro.**

---

## Calibração da velocidade

O fator `148.0` (definido como `FATOR_VELOCIDADE`) é o valor documentado para o PQ35. Se o velocímetro mostrar valor diferente do GPS:

1. Abra o Serial Monitor a 115200 baud
2. Anote o campo `raw=` enquanto dirige em velocidade conhecida
3. Calcule: `FATOR_VELOCIDADE = raw / velocidade_gps_kmh`
4. Atualize a constante e reenvie o código

Fator alternativo documentado para algumas transmissões PQ35: `322.0`

---

## Preparação com VCDS

Antes de instalar, é recomendável:

1. **Endereço 03 (ABS):** Limpe os faults existentes
2. **Endereço 19 (Gateway J533):** No Long Coding Helper, confirme que o bit de presença do módulo ABS/ESP está **desmarcado** se você já desabilitou via VCDS. Isso evita que o gateway fique esperando resposta diagnóstica UDS do endereço ABS.
3. Após instalar e ligar o carro, limpe todos os DTCs com VCDS ou OBDeleven.

### O que este emulador resolve e não resolve

| Função | Resultado |
|---|---|
| Velocímetro | ✅ Funciona |
| Odômetro | ✅ Funciona |
| Luz ABS apagar | ✅ Apaga (sem timeout de mensagem) |
| Ponteiro de combustível estabilizar | ✅ Melhora |
| Aparecer no autoscan VCDS como módulo ABS | ❌ Não (sem resposta diagnóstica UDS) |
| Fault `01316` sumir do gateway | ⚠️ Depende do long coding do gateway |

---

## Estrutura do código

```
pq35_abs_emulator.ino
│
├── setup()             — inicializa MCP2515, configura filtros de recepção
├── loop()              — controla temporização dos envios
│
├── lerCAN()            — lê 0x540 do TCM, extrai velocidade
├── verificarTimeout()  — zera velocidade se o TCM parar de transmitir
│
├── enviar5A0()         — transmite velocímetro + contador de distância
├── enviar1A0()         — transmite wheel speed para ECU/TCM
└── enviar4A0()         — transmite keepalive ABS/ESP
```

---

## Compatibilidade

Testado no VW Jetta 2.5 2009. Deve funcionar em qualquer veículo da plataforma PQ35:

| Veículo | Status |
|---|---|
| VW Jetta MK5 (2005–2010) | Testado |
| VW Golf MK5 (2003–2009) | Deve funcionar |
| VW Passat B6 (2005–2010) | Deve funcionar |
| Audi A3 8P (2003–2012) | Deve funcionar |
| Seat Leon MK2 (2005–2012) | Deve funcionar |
| Skoda Octavia MK2 (2004–2013) | Deve funcionar |

Se você testou em algum desses modelos, abra uma issue com os resultados.

---

## Referências

- [VW Instrument Cluster Controller — an-ven](https://github.com/an-ven/VW-Instrument-Cluster-Controller) — fórmula do `0x5A0` testada em hardware real
- [CAN BUS Gaming Simulator — Hackaday](https://hackaday.io/project/6288) — mapeamento de IDs CAN do PQ35
- [Vehicle Reverse Engineering Wiki — Volkswagen](https://vehicle-reverse-engineering.fandom.com/wiki/Volkswagen) — log CAN real do VW Passat B6 PQ35
- [Autosport Labs MK5 CAN Preset](https://www.autosportlabs.com/2006-2010-vw-mk5-can-bus-preset-now-available/) — canais de wheel speed confirmados

---

## Licença

MIT — use à vontade, por sua conta e risco. Nenhuma garantia de funcionamento em qualquer veículo específico.

---

## Contribuições

Pull requests são bem-vindos. Se você testou em algum veículo PQ35 — com sucesso ou não — abra uma issue com os resultados, o tipo de câmbio (manual/automático) e o fator de calibração que funcionou para você.
