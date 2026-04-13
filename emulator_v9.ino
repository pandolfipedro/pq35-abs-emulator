/*
 * VW Jetta 2009 - Bypass ABS via CAN Bus
 * ESP32 + MCP2515 (TJA1050)
 *
 * Lê velocidade da TCU via OBD2 (0x7E1 / 0x7E9)
 * e envia mensagens ABS (0x1A0, 0x4A0, 0x5A0, 0xDA0, 0x289)
 *
 * Ref: https://the07k.wiki/wiki/Can-bus_emulation_for_cruise_control
 *      PQ35/46 ACAN KMatrix V5.20.6F
 *      vw_golf_mk4.dbc (commaai/opendbc)
 *
 * v8 - Correções críticas:
 *   - 0x5A0 fator corrigido: 73.0 → 100.0 (igual 0x1A0/0x4A0, fonte: KMatrix oficial)
 *   - 0x5A0 BR2_Wegimpulse adicionado (bytes 6-7): contador 11-bit de pulsos ABS
 *   - 0xDA0 adicionado: "ABS vivo" exigido pelo cluster PQ35
 *   - Counter 0x1A0 e 0x5A0: sempre incrementa (evita duplicatas que causam rejeição)
 *   - OBD2 timeout: 500ms → 1000ms
 *   - Serial log: 1s → 2s (evita bloqueio coincidindo com timer do 0x1A0)
 *
 * v9 - Freio via CAN (sem fio extra):
 *   - Lê Motor_2 (0x288) byte2 bit0 = Bremslichtschalter (fonte: vw_golf_mk4.dbc)
 *   - Reflete estado do freio em 0x1A0 byte1 bit3 e 0x289 byte1 bit1
 *   - Filtros hardware: RXB0→0x7E9 (OBD2), RXB1→0x288 (Motor_2)
 *   - ACAN2515 v2.1.5: begin() com 2 máscaras + 4 filtros (range obrigatório: 3-6)
 */

#include <SPI.h>
#include <ACAN2515.h>

// ===================== CONFIGURAÇÃO DE PINOS =====================

static const uint8_t PIN_CAN_CS   = 5;
static const uint8_t PIN_CAN_INT  = 2;
static const uint8_t PIN_SPI_SCK  = 18;
static const uint8_t PIN_SPI_MISO = 19;
static const uint8_t PIN_SPI_MOSI = 23;

static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;

// ===================== CONFIGURAÇÃO DO PROTOCOLO =====================

#define OBD2_REQUEST_ID   0x7E1
#define OBD2_RESPONSE_ID  0x7E9
#define MOTOR2_ID         0x288   // Motor_2: Bremslichtschalter byte2 bit0
#define OBD2_INTERVAL     80

#define INTERVALO_1A0  10
#define INTERVALO_4A0  10
#define INTERVALO_5A0  20
#define INTERVALO_DA0  20
#define INTERVALO_289  20

#define MAX_ACEL    50.0f
#define MAX_DESACEL 50.0f
#define OBD2_TIMEOUT      1000
#define SPEED_DEADZONE    3.0f
#define HEALTH_CHECK_INTERVAL  2000

// Distance Impulse Number (VCDS coding, módulo 17 - Instruments)
// Fórmula: 43 dentes × (1.000.000 / circunferência_de_rolagem_mm)
// Opção 1 = 22188 (circ ~1938mm)
// Opção 2 = 22076 (circ ~1948mm)
// Opção 3 = 21960 (circ ~1958mm) ← DEFAULT 205/55R16
// Opção 4 = 21848 (circ ~1968mm)
// Opção 5 = 22304 (circ ~1928mm)
// Opção 6 = 22420 (circ ~1918mm)
// Opção 7 = 22532 (circ ~1908mm)
#define VCDS_WEGIMPULS  21960.0f  // <- Altere para o número lido no VCDS do seu carro

// ===================== OBJETO CAN =====================

ACAN2515 can(PIN_CAN_CS, SPI, PIN_CAN_INT);

// ===================== VARIÁVEIS =====================

bool canOk      = false;
bool freioAtivo = false;  // lido do Motor_2 (0x288) via CAN

float velAlvo  = 0.0f;
float velCache = 0.0f;
unsigned long tUltimaLeitura = 0;
unsigned long tUltimoCache   = 0;

uint8_t  counter1A0  = 0;    // rolling 4-bit (0-15)
uint8_t  counter5A0  = 0;    // rolling 4-bit (0-15)
uint16_t counterDA0  = 0;    // free-running 16-bit

float    wegimpulsAcum    = 0.0f;
uint16_t wegimpulsCounter = 0;
unsigned long tUltimoFrame5A0 = 0;

unsigned long tEnvio1A0 = 0, tEnvio4A0 = 0, tEnvio5A0 = 0;
unsigned long tEnvioDA0 = 0, tEnvio289 = 0;
unsigned long tOBD2Req  = 0, tLog = 0, tHealthCheck = 0;

uint32_t errTx = 0, errBusOff = 0, okTx = 0, leituraOBD2 = 0;

// ===================== HELPERS =====================

uint8_t calcChecksum(uint8_t data[], uint16_t canId) {
  uint8_t chk = 0;
  for (int i = 1; i < 8; i++) chk ^= data[i];
  chk ^= (uint8_t)(canId & 0xFF);
  chk ^= (uint8_t)((canId >> 8) & 0xFF);
  return chk;
}

bool enviarCAN(uint16_t id, uint8_t data[8]) {
  CANMessage frame;
  frame.id  = id;
  frame.ext = false;
  frame.len = 8;
  for (int i = 0; i < 8; i++) frame.data[i] = data[i];
  if (can.tryToSend(frame)) { okTx++; return true; }
  errTx++;
  return false;
}

// ===================== FILTROS E BEGIN =====================

static uint16_t iniciarCAN() {
  ACAN2515Settings settings(QUARTZ_FREQUENCY, 500UL * 1000UL);
  settings.mRequestedMode       = ACAN2515Settings::NormalMode;
  settings.mReceiveBufferSize   = 32;
  settings.mTransmitBuffer0Size = 16;
  settings.mTransmitBuffer1Size = 0;
  settings.mTransmitBuffer2Size = 0;

  // RXB0 (RXM0): 0x7E9
  // RXB1 (RXM1): 0x288
  const ACAN2515Mask rxm0 = standard2515Mask(0x7FF, 0, 0);
  const ACAN2515Mask rxm1 = standard2515Mask(0x7FF, 0, 0);

  const ACAN2515AcceptanceFilter filters[] = {
    {standard2515Filter(OBD2_RESPONSE_ID, 0, 0), NULL},
    {standard2515Filter(OBD2_RESPONSE_ID, 0, 0), NULL},
    {standard2515Filter(MOTOR2_ID, 0, 0), NULL},
    {standard2515Filter(MOTOR2_ID, 0, 0), NULL},
  };

  return can.begin(settings, [] { can.isr(); }, rxm0, rxm1, filters, 4);
}

// ===================== SAÚDE DO MCP2515 =====================

void verificarSaudeMCP() {
  unsigned long agora = millis();
  if (agora - tHealthCheck < HEALTH_CHECK_INTERVAL) return;
  tHealthCheck = agora;

  if (can.transmitErrorCounter() > 180) {
    errBusOff++;
    Serial.println("[ERRO] BUS-OFF - Reinicializando MCP2515...");
    can.end();
    delay(10);

    uint16_t err = iniciarCAN();
    if (err == 0) {
      Serial.println("[OK] MCP2515 reinicializado");
    } else {
      Serial.printf("[ERRO] Falha reinit: 0x%X\n", err);
      canOk = false;
    }
  }
}

// ===================== VELOCIDADE =====================

void atualizarVelocidade() {
  unsigned long agora = millis();

  if (agora - tUltimaLeitura > OBD2_TIMEOUT) velAlvo = 0.0f;

  float dt = (agora - tUltimoCache) / 1000.0f;
  if (dt <= 0.0f) return;

  float erro  = velAlvo - velCache;
  float passo = erro * (dt * 10.0f);

  if (passo >  MAX_ACEL    * dt) passo =  MAX_ACEL    * dt;
  if (passo < -MAX_DESACEL * dt) passo = -MAX_DESACEL * dt;

  velCache += passo;
  if (velCache < 0.1f) velCache = 0.0f;

  tUltimoCache = agora;
}

// ===================== OBD2 / CAN RX =====================

void enviarPedidoOBD2() {
  uint8_t p[8] = {0x02, 0x01, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00};
  enviarCAN(OBD2_REQUEST_ID, p);
}

void lerCAN() {
  CANMessage frame;
  while (can.receive(frame)) {

    if (frame.id == OBD2_RESPONSE_ID &&
        frame.data[1] == 0x41 && frame.data[2] == 0x0D) {
      float raw = (float)frame.data[3];
      velAlvo = (raw <= SPEED_DEADZONE) ? 0.0f : raw;
      tUltimaLeitura = millis();
      leituraOBD2++;
    }

    if (frame.id == MOTOR2_ID) {
      freioAtivo = (frame.data[2] & 0x01) != 0;
    }
  }
}

// ===================== MENSAGENS ABS =====================

void enviarBremse1(float speedKmh) {
  uint16_t speedRaw  = (uint16_t)(speedKmh * 100.0f);
  uint8_t  brakeFlag = freioAtivo ? 0x08 : 0x00;

  uint8_t data[8] = {
    0x00,
    brakeFlag,
    (uint8_t)((speedRaw & 0x7F) << 1),
    (uint8_t)((speedRaw >> 7) & 0xFF),
    0xFE, 0xFE, 0x00,
    (uint8_t)((counter1A0 & 0x0F) | 0x10)
  };

  data[0] = calcChecksum(data, 0x1A0);
  enviarCAN(0x1A0, data);
  counter1A0 = (counter1A0 + 1) & 0x0F;
}

void enviarBremse3(float speedKmh) {
  if (speedKmh > 326.0f) speedKmh = 326.0f;

  uint16_t val = ((uint16_t)(speedKmh * 100.0f) << 1) & 0xFFFE;
  uint8_t lo = val & 0xFF;
  uint8_t hi = (val >> 8) & 0xFF;
  uint8_t data[8] = { lo, hi, lo, hi, lo, hi, lo, hi };

  enviarCAN(0x4A0, data);
}

void enviarBremse2(float speedKmh) {
  unsigned long agora = millis();
  float dt = (agora - tUltimoFrame5A0) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.020f;
  tUltimoFrame5A0 = agora;

  if (speedKmh > 326.0f) speedKmh = 326.0f;

  uint16_t vel = ((uint16_t)(speedKmh * 100.0f) << 1) & 0xFFFE;
  uint16_t ts  = (uint16_t)(agora / 2);

  wegimpulsAcum += speedKmh * (VCDS_WEGIMPULS / 3600.0f) * dt;
  uint16_t novos = (uint16_t)wegimpulsAcum;
  wegimpulsAcum -= (float)novos;
  wegimpulsCounter = (wegimpulsCounter + novos) & 0x07FF;

  uint8_t data[8] = {
    0x00,
    (uint8_t)(vel & 0xFF),
    (uint8_t)((vel >> 8) & 0xFF),
    (uint8_t)(ts & 0xFF),
    (uint8_t)((ts >> 8) & 0xFF),
    (uint8_t)(counter5A0 & 0x0F),
    (uint8_t)(wegimpulsCounter & 0xFF),
    (uint8_t)((wegimpulsCounter >> 8) & 0x07)
  };

  enviarCAN(0x5A0, data);
  counter5A0 = (counter5A0 + 1) & 0x0F;
}

void enviarDA0() {
  uint8_t data[8] = {
    0x01, 0x80, 0x00, 0x00, 0x00, 0x00,
    (uint8_t)(counterDA0 & 0xFF),
    (uint8_t)((counterDA0 >> 8) & 0xFF)
  };

  enviarCAN(0xDA0, data);
  counterDA0++;
}

void enviarBremse5() {
  uint8_t b1 = 0x01;
  if (freioAtivo) b1 |= 0x02;

  uint8_t data[8] = { 0x00, b1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  enviarCAN(0x289, data);
}

void enviarTodasABS(float speedKmh) {
  unsigned long agora = millis();

  if (agora - tEnvio1A0 >= INTERVALO_1A0) { enviarBremse1(speedKmh); tEnvio1A0 = agora; }
  if (agora - tEnvio4A0 >= INTERVALO_4A0) { enviarBremse3(speedKmh); tEnvio4A0 = agora; }
  if (agora - tEnvio5A0 >= INTERVALO_5A0) { enviarBremse2(speedKmh); tEnvio5A0 = agora; }
  if (agora - tEnvioDA0 >= INTERVALO_DA0) { enviarDA0();             tEnvioDA0 = agora; }
  if (agora - tEnvio289 >= INTERVALO_289) { enviarBremse5();         tEnvio289 = agora; }
}

// ===================== SETUP & LOOP =====================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("VW Jetta 2009 - ABS Bypass v9");
  Serial.println("Freio via CAN (0x288), filtros duplos, sem fio extra");

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  uint16_t errorCode = iniciarCAN();
  if (errorCode == 0) {
    canOk = true;
    Serial.println("CAN OK - 500kbps | RXB0:0x7E9 RXB1:0x288");
  } else {
    Serial.printf("ERRO ACAN2515: 0x%X\n", errorCode);
  }

  unsigned long agora = millis();
  tUltimoCache    = agora;
  tUltimaLeitura  = agora;
  tUltimoFrame5A0 = agora;
  tEnvio1A0       = agora;
  tEnvio4A0       = agora + 5;
  tEnvio5A0       = agora + 3;
  tEnvioDA0       = agora + 7;
  tEnvio289       = agora + 9;
}

void loop() {
  if (!canOk) {
    delay(2000);
    Serial.println("Tentando reconectar...");
    setup();
    return;
  }

  unsigned long agora = millis();

  lerCAN();

  if (agora - tOBD2Req >= OBD2_INTERVAL) {
    enviarPedidoOBD2();
    tOBD2Req = agora;
  }

  lerCAN();
  atualizarVelocidade();

  float velEnvio = (velCache <= SPEED_DEADZONE) ? 0.0f : velCache;
  enviarTodasABS(velEnvio);

  verificarSaudeMCP();

  if (agora - tLog >= 2000) {
    Serial.printf("[v9] vel=%.1f freio=%d weg=%u obd=%lums reads=%u txOk=%u txErr=%u busOff=%u\n",
      velCache, (int)freioAtivo, wegimpulsCounter,
      agora - tUltimaLeitura,
      leituraOBD2, okTx, errTx, errBusOff);
    tLog = agora;
  }
}
