/*
 * VW Jetta 2009 - Bypass ABS via CAN Bus
 * ESP32 + MCP2515 (TJA1050)
 *
 * Lê velocidade da TCU via OBD2 (0x7E1 / 0x7E9)
 * e envia mensagens ABS (0x1A0, 0x4A0, 0x5A0)
 *
 * Ref: https://the07k.wiki/wiki/Can-bus_emulation_for_cruise_control
 *      https://wiki.osm.org/wiki/VW-CAN
 *
 * v7.0 - Migração para ACAN2515: ISR-driven, buffers de software,
 *         filtros hardware, tryToSend não-bloqueante.
 *         Mantém: counter condicional, zona morta, filtro passa-baixa.
 */

#include <SPI.h>
#include <ACAN2515.h>

// ===================== CONFIGURAÇÃO DE PINOS =====================

static const uint8_t PIN_CAN_CS   = 5;
static const uint8_t PIN_CAN_INT  = 2;
static const uint8_t PIN_SPI_SCK  = 18;
static const uint8_t PIN_SPI_MISO = 19;
static const uint8_t PIN_SPI_MOSI = 23;

// Frequência do cristal do módulo MCP2515 (8MHz para módulos azuis comuns)
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;

// ===================== CONFIGURAÇÃO DO PROTOCOLO =====================

// OBD2 - TCU (câmbio 09G)
#define OBD2_REQUEST_ID   0x7E1
#define OBD2_RESPONSE_ID  0x7E9
#define OBD2_INTERVAL     80

// Calibração 0x5A0
#define SPEED_FACTOR_5A0    73.0

// Intervalos de envio (ms)
#define INTERVALO_1A0  10
#define INTERVALO_4A0  10
#define INTERVALO_5A0  20

// Cache
#define MAX_DESACEL    40.0

// Timeout para considerar que perdeu a leitura OBD2
#define OBD2_TIMEOUT   500

// Zona morta de velocidade (abaixo disso = parado, evita hodômetro fantasma)
#define SPEED_DEADZONE  3.0

// Filtro passa-baixa (suavização do jitter da TCU)
#define SPEED_ALPHA     0.4

// Intervalo de verificação de saúde do MCP2515 (ms)
#define HEALTH_CHECK_INTERVAL  2000

// ===================== OBJETO CAN (ACAN2515) =====================

ACAN2515 can(PIN_CAN_CS, SPI, PIN_CAN_INT);

// ===================== VARIÁVEIS =====================

bool canOk = false;

float velCache = 0.0;
unsigned long tUltimaLeitura = 0;
unsigned long tUltimoCache = 0;

uint8_t counter1A0 = 0;
uint8_t counter5A0 = 0;

unsigned long tEnvio1A0 = 0, tEnvio4A0 = 0, tEnvio5A0 = 0;
unsigned long tOBD2Req = 0, tLog = 0, tHealthCheck = 0;

// Contadores de erro para diagnóstico
uint32_t errTx = 0;
uint32_t errBusOff = 0;
uint32_t okTx = 0;
uint32_t leituraOBD2 = 0;

// ===================== CHECKSUM =====================

/*
 * Checksum XOR para mensagens ABS do PQ35.
 * O byte 0 é o checksum, calculado com XOR de:
 *   - bytes 1 a 7 do payload
 *   - low byte e high byte do CAN ID
 */
uint8_t calcChecksum(uint8_t data[], uint16_t canId) {
  uint8_t chk = 0;
  for (int i = 1; i < 8; i++) {
    chk ^= data[i];
  }
  chk ^= (uint8_t)(canId & 0xFF);
  chk ^= (uint8_t)((canId >> 8) & 0xFF);
  return chk;
}

// ===================== ENVIO CAN =====================

/*
 * Envia mensagem CAN via ACAN2515 (tryToSend não-bloqueante).
 * A ACAN2515 enfileira internamente e envia via ISR.
 * Retorna true se a mensagem foi aceita no buffer de envio.
 */
bool enviarCAN(uint16_t id, uint8_t data[8]) {
  CANMessage frame;
  frame.id = id;
  frame.ext = false;  // Standard 11-bit ID
  frame.len = 8;
  for (int i = 0; i < 8; i++) {
    frame.data[i] = data[i];
  }

  if (can.tryToSend(frame)) {
    okTx++;
    return true;
  }
  errTx++;
  return false;
}

// ===================== SAÚDE DO MCP2515 =====================

/*
 * Verifica se o MCP2515 está em estado de erro grave (BUS-OFF)
 * e reinicializa se necessário.
 */
void verificarSaudeMCP() {
  unsigned long agora = millis();
  if (agora - tHealthCheck < HEALTH_CHECK_INTERVAL) return;
  tHealthCheck = agora;

  uint8_t txErr = can.transmitErrorCounter();
  uint8_t rxErr = can.receiveErrorCounter();

  // Se TX errors estão altos, o MCP2515 pode estar em BUS-OFF
  if (txErr > 180) {
    errBusOff++;
    Serial.print("[ERRO] TX err=");
    Serial.print(txErr);
    Serial.println(" - Reinicializando MCP2515...");

    // Reinicializar o MCP2515 via ACAN2515
    can.end();
    delay(10);

    ACAN2515Settings settings(QUARTZ_FREQUENCY, 500UL * 1000UL);
    settings.mRequestedMode = ACAN2515Settings::NormalMode;
    settings.mReceiveBufferSize = 32;
    settings.mTransmitBuffer0Size = 16;
    settings.mTransmitBuffer1Size = 0;
    settings.mTransmitBuffer2Size = 0;

    // Filtro para aceitar apenas 0x7E9
    const ACAN2515Mask rxm0 = standard2515Mask(0x7FF, 0, 0);
    const ACAN2515AcceptanceFilter filters[] = {
      {standard2515Filter(OBD2_RESPONSE_ID, 0, 0), NULL},
      {standard2515Filter(OBD2_RESPONSE_ID, 0, 0), NULL}
    };

    uint16_t err = can.begin(settings, [] { can.isr(); }, rxm0, filters, 2);
    if (err == 0) {
      Serial.println("[OK] MCP2515 reinicializado com sucesso");
    } else {
      Serial.print("[ERRO] Falha reinit: 0x");
      Serial.println(err, HEX);
      canOk = false;
    }
  }
}

// ===================== CACHE DE VELOCIDADE =====================

void atualizarCache(float novaVel) {
  unsigned long agora = millis();
  float dt = (agora - tUltimoCache) / 1000.0;
  if (dt <= 0) dt = 0.001;

  // Filtro passa-baixa para suavizar jitter da TCU
  float velSuavizada = velCache * (1.0 - SPEED_ALPHA) + novaVel * SPEED_ALPHA;

  float quedaMax = MAX_DESACEL * dt;

  if (velSuavizada >= velCache) {
    velCache = velSuavizada;
  } else if (velCache - velSuavizada > quedaMax) {
    velCache -= quedaMax;
  } else {
    velCache = velSuavizada;
  }

  if (velCache < 0.5) velCache = 0.0;
  tUltimoCache = agora;
}

void decairCache() {
  unsigned long agora = millis();
  if (agora - tUltimaLeitura > OBD2_TIMEOUT) {
    float dt = (agora - tUltimoCache) / 1000.0;
    float queda = MAX_DESACEL * 0.5 * dt;
    velCache -= queda;
    if (velCache < 0.5) velCache = 0.0;
    tUltimoCache = agora;
  }
}

// ===================== OBD2 =====================

void enviarPedidoOBD2() {
  uint8_t pedido[8] = {0x02, 0x01, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00};
  if (!enviarCAN(OBD2_REQUEST_ID, pedido)) {
    // Falha contabilizada dentro de enviarCAN
  }
}

void lerCAN() {
  CANMessage frame;
  while (can.receive(frame)) {
    if (frame.id == OBD2_RESPONSE_ID &&
        frame.data[1] == 0x41 && frame.data[2] == 0x0D) {
      atualizarCache((float)frame.data[3]);
      tUltimaLeitura = millis();
      leituraOBD2++;
    }
  }
}

// ===================== MENSAGENS ABS =====================

/*
 * 0x1A0 - Bremse_1 (8 bytes, cada 10ms)
 * byte0: checksum XOR (bytes 1-7 XOR CAN ID)
 * byte1: 0x00
 * byte2: bit0 = brake booster, bits 7..1 = speed low (7 bits)
 * byte3: speed high (8 bits)
 * byte4: 0xFE (sem dado)
 * byte5: 0xFE (sem dado)
 * byte6: 0x00
 * byte7: bits 0-3 = counter (4-bit), bit4 = ASR present
 */
void enviarBremse1(float speedKmh) {
  uint16_t speedRaw = (uint16_t)(speedKmh * 100.0);

  uint8_t data[8] = {
    0x00,  // byte0: placeholder para checksum
    0x00,
    (uint8_t)((speedRaw & 0x7F) << 1),
    (uint8_t)((speedRaw >> 7) & 0xFF),
    0xFE,
    0xFE,
    0x00,
    (uint8_t)((counter1A0 & 0x0F) | 0x10)
  };

  // Calcular e inserir checksum XOR no byte 0
  data[0] = calcChecksum(data, 0x1A0);

  // Counter só incrementa no envio bem-sucedido (evita saltos que o cluster rejeita)
  if (enviarCAN(0x1A0, data)) {
    counter1A0 = (counter1A0 + 1) & 0x0F;
  }
}

/*
 * 0x4A0 - Bremse_3 (8 bytes, cada 10ms)
 * 4 rodas: bit0 = direção, bits 15..1 = speed*100
 */
void enviarBremse3(float speedKmh) {
  // Limitar velocidade para evitar overflow
  if (speedKmh > 327.0) speedKmh = 327.0;

  uint16_t speedRaw = (uint16_t)(speedKmh * 100.0);
  uint16_t val = (speedRaw << 1) & 0xFFFE;
  uint8_t lo = val & 0xFF;
  uint8_t hi = (val >> 8) & 0xFF;
  uint8_t data[8] = { lo, hi, lo, hi, lo, hi, lo, hi };
  enviarCAN(0x4A0, data);
}

/*
 * 0x5A0 - Bremse_2 (8 bytes, cada 20ms)
 * byte0: flags (0x00)
 * byte1-2: bits 15..1 = rotação média rodas, bit0 = flag
 * byte3-4: timestamp real (millis/2, unidade ~0.002s)
 * byte5: rolling counter (4-bit, 0-15)
 * byte6-7: 0x00
 */
void enviarBremse2(float speedKmh) {
  // Limitar velocidade para evitar overflow
  if (speedKmh > 327.0) speedKmh = 327.0;

  uint16_t sr = (uint16_t)(speedKmh * SPEED_FACTOR_5A0);
  uint16_t val = (sr << 1) & 0xFFFE;
  uint16_t ts = (uint16_t)(millis() / 2);

  uint8_t data[8] = {
    0x00,
    (uint8_t)(val & 0xFF),
    (uint8_t)((val >> 8) & 0xFF),
    (uint8_t)(ts & 0xFF),
    (uint8_t)((ts >> 8) & 0xFF),
    (uint8_t)(counter5A0 & 0x0F),  // Counter limitado a 4 bits
    0x00,
    0x00
  };
  // Counter só incrementa no envio bem-sucedido (evita saltos que o cluster rejeita)
  if (enviarCAN(0x5A0, data)) {
    counter5A0 = (counter5A0 + 1) & 0x0F;
  }
}

void enviarTodasABS(float speedKmh) {
  unsigned long agora = millis();
  if (agora - tEnvio1A0 >= INTERVALO_1A0) {
    enviarBremse1(speedKmh);
    tEnvio1A0 = agora;
  }
  if (agora - tEnvio4A0 >= INTERVALO_4A0) {
    enviarBremse3(speedKmh);
    tEnvio4A0 = agora;
  }
  if (agora - tEnvio5A0 >= INTERVALO_5A0) {
    enviarBremse2(speedKmh);
    tEnvio5A0 = agora;
  }
}

// ===================== SETUP & LOOP =====================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("VW Jetta 2009 - ABS Bypass v7.0 (ACAN2515)");
  Serial.println("ISR-driven, buffers software, counter condicional, deadzone");

  // Inicializar SPI
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  // Configurar ACAN2515
  Serial.println("Configurando ACAN2515...");

  ACAN2515Settings settings(QUARTZ_FREQUENCY, 500UL * 1000UL);  // 500 kbps
  settings.mRequestedMode = ACAN2515Settings::NormalMode;

  // Buffers de software (muito maiores que os 2 RX / 3 TX do hardware)
  settings.mReceiveBufferSize = 32;    // 32 frames no buffer RX de software
  settings.mTransmitBuffer0Size = 16;  // 16 frames no buffer TX de software
  settings.mTransmitBuffer1Size = 0;
  settings.mTransmitBuffer2Size = 0;

  // Filtro hardware: aceitar apenas 0x7E9 (resposta OBD2 da TCU)
  const ACAN2515Mask rxm0 = standard2515Mask(0x7FF, 0, 0);  // Match exato
  const ACAN2515AcceptanceFilter filters[] = {
    {standard2515Filter(OBD2_RESPONSE_ID, 0, 0), NULL},
    {standard2515Filter(OBD2_RESPONSE_ID, 0, 0), NULL}
  };

  // Inicializar com ISR (interrupt-driven) e filtros
  const uint16_t errorCode = can.begin(settings, [] { can.isr(); }, rxm0, filters, 2);

  if (errorCode == 0) {
    canOk = true;
    Serial.println("CAN OK - 500kbps - ACAN2515 com filtros para 0x7E9");
    Serial.print("Buffer RX: "); Serial.print(settings.mReceiveBufferSize);
    Serial.print(" | Buffer TX: "); Serial.println(settings.mTransmitBuffer0Size);
  } else {
    Serial.print("ERRO: ACAN2515 falhou! Codigo: 0x");
    Serial.println(errorCode, HEX);
  }

  tUltimoCache = millis();
  tUltimaLeitura = millis();

  // Escalonar timers para evitar envios simultâneos
  tEnvio1A0 = millis();
  tEnvio4A0 = millis() + 5;   // offset 5ms
  tEnvio5A0 = millis() + 3;   // offset 3ms
}

void loop() {
  if (!canOk) {
    delay(2000);
    Serial.println("Tentando reconectar ACAN2515...");
    setup();  // Reinicializar tudo
    return;
  }

  unsigned long agora = millis();

  // Ler mensagens recebidas (não precisa mais de digitalRead/INT, a ISR cuida)
  lerCAN();

  if (agora - tOBD2Req >= OBD2_INTERVAL) {
    enviarPedidoOBD2();
    tOBD2Req = agora;
  }

  lerCAN();
  decairCache();

  // Aplicar zona morta antes de enviar (evita hodômetro fantasma com carro parado)
  float velParaEnvio = velCache;
  if (velParaEnvio < SPEED_DEADZONE) velParaEnvio = 0.0;
  enviarTodasABS(velParaEnvio);

  // Verificar saúde do MCP2515 (BUS-OFF recovery)
  verificarSaudeMCP();

  if (agora - tLog >= 1000) {
    Serial.print("[v7.0] vel=");
    Serial.print(velCache, 1);
    Serial.print(" cnt1A0=");
    Serial.print(counter1A0);
    Serial.print(" cnt5A0=");
    Serial.print(counter5A0);
    Serial.print(" obd=");
    Serial.print(agora - tUltimaLeitura);
    Serial.print("ms reads=");
    Serial.print(leituraOBD2);
    Serial.print(" txOk=");
    Serial.print(okTx);
    Serial.print(" txErr=");
    Serial.print(errTx);
    Serial.print(" busOff=");
    Serial.print(errBusOff);
    Serial.print(" rxBuf=");
    Serial.print(can.receiveBufferPeakCount());
    Serial.print("/");
    Serial.println(can.receiveBufferSize());
    tLog = agora;
  }
}
