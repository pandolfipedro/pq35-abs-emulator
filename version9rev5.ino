#include <SPI.h>
#include <ACAN2515.h>

// ===================== CONFIGURACAO DE PINOS =====================

static const uint8_t PIN_CAN_CS   = 5;
static const uint8_t PIN_CAN_INT  = 2;
static const uint8_t PIN_SPI_SCK  = 18;
static const uint8_t PIN_SPI_MISO = 19;
static const uint8_t PIN_SPI_MOSI = 23;

static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;

// ===================== CONFIGURACAO DO PROTOCOLO =====================

#define OBD2_REQUEST_ID   0x7E1
#define OBD2_RESPONSE_ID  0x7E9
#define MOTOR2_ID         0x288   // Motor_2: Bremslichtschalter byte2 bits 0-1
#define OBD2_INTERVAL     80

#define INTERVALO_1A0  10
#define INTERVALO_4A0  10
#define INTERVALO_4A8  10
#define INTERVALO_5A0  20

#define MAX_ACEL       50.0f
#define MAX_DESACEL    50.0f
#define OBD2_TIMEOUT   1000
#define SPEED_DEADZONE  3.0f
#define HEALTH_CHECK_INTERVAL  2000

// Distance Impulse Number (VCDS coding, modulo 17 - Instruments)
// Formula: 43 dentes x (1.000.000 / circunferencia_de_rolagem_mm)
// Opcao 1 = 22188 (circ ~1938mm)
// Opcao 2 = 22076 (circ ~1948mm)
// Opcao 3 = 21960 (circ ~1958mm) <- DEFAULT 205/55R16
// Opcao 4 = 21848 (circ ~1968mm)
// Opcao 5 = 22304 (circ ~1928mm)
// Opcao 6 = 22420 (circ ~1918mm)
// Opcao 7 = 22532 (circ ~1908mm)
#define VCDS_WEGIMPULS  21960.0f  // <- Altere para o numero lido no VCDS do seu carro

// ===================== OBJETO CAN =====================

ACAN2515 can(PIN_CAN_CS, SPI, PIN_CAN_INT);

// ===================== VARIAVEIS =====================

bool canOk      = false;
bool freioAtivo = false;  // lido do Motor_2 (0x288) via CAN

float velAlvo  = 0.0f;
float velCache = 0.0f;
unsigned long tUltimaLeitura = 0;
unsigned long tUltimoCache   = 0;

uint8_t  counter1A0  = 0;    // rolling 4-bit (0-15)
uint8_t  counter5A0  = 0;    // rolling 4-bit (0-15)
uint8_t  counter4A8  = 0;    // rolling 4-bit (0-15)

float    wegimpulsAcum    = 0.0f;
uint16_t wegimpulsCounter = 0;
unsigned long tUltimoFrame5A0 = 0;

unsigned long tEnvio1A0 = 0, tEnvio4A0 = 0, tEnvio4A8 = 0, tEnvio5A0 = 0;
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

  // MCP2515 tem 2 buffers de RX com mascaras independentes:
  // RXB0 (RXM0): 0x7E9 - resposta OBD2 da TCU
  // RXB1 (RXM1): 0x288 - Motor_2 (estado do pedal de freio)
  // ACAN2515 v2.1.5 exige 3-6 filtros ao usar 2 mascaras
  const ACAN2515Mask rxm0 = standard2515Mask(0x7FF, 0, 0);
  const ACAN2515Mask rxm1 = standard2515Mask(0x7FF, 0, 0);
  const ACAN2515AcceptanceFilter filters[] = {
    {standard2515Filter(OBD2_RESPONSE_ID, 0, 0), NULL},
    {standard2515Filter(OBD2_RESPONSE_ID, 0, 0), NULL},
    {standard2515Filter(MOTOR2_ID,        0, 0), NULL},
    {standard2515Filter(MOTOR2_ID,        0, 0), NULL},
  };

  return can.begin(settings, [] { can.isr(); }, rxm0, rxm1, filters, 4);
}

// ===================== SAUDE DO MCP2515 =====================

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
    if (err == 0) Serial.println("[OK] MCP2515 reinicializado");
    else { Serial.printf("[ERRO] Falha reinit: 0x%X\n", err); canOk = false; }
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
      // Ignorar leituras anomalas (0xFF) quando a TCU esta desligada
      if (raw >= 254.0f) {
        velAlvo = 0.0f;
      } else {
        velAlvo = (raw <= SPEED_DEADZONE) ? 0.0f : raw;
      }
      tUltimaLeitura = millis();
      leituraOBD2++;
    }

    if (frame.id == MOTOR2_ID) {
      freioAtivo = (frame.data[2] & 0x03) != 0;
    }
  }
}

// ===================== MENSAGENS ABS =====================

void enviarBremse1(float speedKmh) {
  uint16_t speedRaw  = (uint16_t)(speedKmh * 100.0f);

  uint8_t data[8] = {
    0x00,
    0x00, // IMPORTANTE: NUNCA usar brakeFlag aqui. Qualquer bit invalida Bremse_1 na rede e o velocimetro cai pra 0!
    (uint8_t)((speedRaw & 0x7F) << 1),
    (uint8_t)((speedRaw >> 7) & 0xFF),
    0xFE, 0xFE, 0x00,
    (uint8_t)((counter1A0 & 0x0F) | 0x50) // Bit 4="ASR_ESP verbaut", Bit 6="Sta_ESP gultig"
  };
  data[0] = calcChecksum(data, 0x1A0);
  enviarCAN(0x1A0, data);
  counter1A0 = (counter1A0 + 1) & 0x0F;
}

void enviarBremse3(float speedKmh) {
  if (speedKmh > 326.0f) speedKmh = 326.0f;
  uint16_t val = ((uint16_t)(speedKmh * 100.0f) << 1) & 0xFFFE;
  uint8_t lo = val & 0xFF, hi = (val >> 8) & 0xFF;
  uint8_t data[8] = { lo, hi, lo, hi, lo, hi, lo, hi };
  enviarCAN(0x4A0, data);
}

void enviarBremse4() {
  uint16_t press = freioAtivo ? 0x0400 : 0x0000;
  uint8_t data[8] = {
    0x00, 
    0x40, // BR5_Sta_Gierrate = 1 (valid)
    (uint8_t)(press & 0xFF),
    (uint8_t)((press >> 8) | 0x20), // BR5_Druckgueltig = 1
    0x00,
    (uint8_t)(freioAtivo ? 0x08 : 0x00), // BR5_Bremslicht
    (uint8_t)((counter4A8 & 0x0F) << 4), // BR5_Zaehler
    0x00
  };
  uint8_t chk = 0;
  for (int i = 0; i < 7; i++) chk ^= data[i];
  chk ^= 0xA8; chk ^= 0x04;
  data[7] = chk; // BR5_Checksumme
  enviarCAN(0x4A8, data);
  counter4A8 = (counter4A8 + 1) & 0x0F;
}

void enviarBremse2(float speedKmh) {
  unsigned long agora = millis();
  float dt = (agora - tUltimoFrame5A0) / 1000.0f;
  if (dt < 0.010f || dt > 0.030f) dt = 0.020f;
  tUltimoFrame5A0 = agora;

  if (speedKmh > 326.0f) speedKmh = 326.0f;

  uint16_t vel = ((uint16_t)(speedKmh * 100.0f) << 1) & 0xFFFE;

  wegimpulsAcum += speedKmh * (VCDS_WEGIMPULS / 3600.0f) * dt;
  uint16_t novos = (uint16_t)wegimpulsAcum;
  wegimpulsAcum -= (float)novos;
  wegimpulsCounter = (wegimpulsCounter + novos) & 0x07FF;

  uint8_t data[8] = {
    0x00, // BR2_Querbeschl = 0
    (uint8_t)(vel & 0xFF),
    (uint8_t)((vel >> 8) & 0xFF),
    (uint8_t)(0x05 | (counter5A0 << 4)), // BR2_Lampe_ABS=1, BR2_Lampe_BK=1, BR2_Zaehler
    0x00,
    0x00,
    (uint8_t)(wegimpulsCounter & 0xFF),
    (uint8_t)((wegimpulsCounter >> 8) & 0x07)
  };
  enviarCAN(0x5A0, data);
  counter5A0 = (counter5A0 + 1) & 0x0F;
}

// (Funcoes DA0 e 289 Removidas permanentemente para prevencao de Diagnosticos / Lixo de Rede MQB)
void enviarTodasABS(float speedKmh) {
  unsigned long agora = millis();
  if (agora - tEnvio1A0 >= INTERVALO_1A0) { enviarBremse1(speedKmh); tEnvio1A0 = agora; }
  if (agora - tEnvio4A0 >= INTERVALO_4A0) { enviarBremse3(speedKmh); tEnvio4A0 = agora; }
  if (agora - tEnvio4A8 >= INTERVALO_4A8) { enviarBremse4();         tEnvio4A8 = agora; }
  if (agora - tEnvio5A0 >= INTERVALO_5A0) { enviarBremse2(speedKmh); tEnvio5A0 = agora; }
}

// ===================== SETUP & LOOP =====================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("VW Jetta 2009 - ABS Bypass vFINAL (100% Oficial K-Matrix)");
  Serial.println("Bremse_1(1A0), Bremse_2(5A0), Bremse_3(4A0), Bremse_5(4A8)");

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
  tEnvio4A0       = agora + 3;
  tEnvio4A8       = agora + 5;
  tEnvio5A0       = agora + 7;
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
    if (errTx > 0 || errBusOff > 0) {
      Serial.printf("[vFINAL] vel=%.1f freio=%d | ERRO CAN: txErr=%u busOff=%u\n",
        velCache, (int)freioAtivo, errTx, errBusOff);
    } else {
      Serial.printf("[vFINAL] vel=%.1f freio=%d\n", velCache, (int)freioAtivo);
    }
    tLog = agora;
  }
}
