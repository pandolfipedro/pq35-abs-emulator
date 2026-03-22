/*
 * VW Jetta 2009 - Bypass ABS via CAN Bus
 * ESP32 + MCP2515 (TJA1050)
 *
 * Lê velocidade da TCU via OBD2 (0x7E1 / 0x7E9)
 * e envia mensagens ABS (0x1A0, 0x4A0, 0x5A0)
 *
 * Ref: https://the07k.wiki/wiki/Can-bus_emulation_for_cruise_control
 *      https://wiki.osm.org/wiki/VW-CAN
 */

#include <SPI.h>
#include <mcp_can.h>

// ===================== CONFIGURAÇÃO =====================

#define PIN_CAN_CS    5
#define PIN_CAN_INT   2
#define PIN_SPI_SCK   18
#define PIN_SPI_MISO  19
#define PIN_SPI_MOSI  23
#define MCP_CRYSTAL   MCP_8MHZ

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

// ===================== VARIÁVEIS =====================

MCP_CAN CAN0(PIN_CAN_CS);
bool canOk = false;

long unsigned int rxId;
unsigned char rxLen = 0;
unsigned char rxBuf[8];

float velCache = 0.0;
unsigned long tUltimaLeitura = 0;
unsigned long tUltimoCache = 0;

uint8_t counter1A0 = 0;
uint8_t counter5A0 = 0;

unsigned long tEnvio1A0 = 0, tEnvio4A0 = 0, tEnvio5A0 = 0;
unsigned long tOBD2Req = 0, tLog = 0;

// ===================== CACHE DE VELOCIDADE =====================

void atualizarCache(float novaVel) {
  unsigned long agora = millis();
  float dt = (agora - tUltimoCache) / 1000.0;
  if (dt <= 0) dt = 0.001;

  float quedaMax = MAX_DESACEL * dt;

  if (novaVel >= velCache) {
    velCache = novaVel;
  } else if (velCache - novaVel > quedaMax) {
    velCache -= quedaMax;
  } else {
    velCache = novaVel;
  }

  if (velCache < 0.5) velCache = 0.0;
  tUltimoCache = agora;
}

void decairCache() {
  unsigned long agora = millis();
  if (agora - tUltimaLeitura > 300) {
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
  CAN0.sendMsgBuf(OBD2_REQUEST_ID, 0, 8, pedido);
}

void lerCAN() {
  while (digitalRead(PIN_CAN_INT) == LOW) {
    CAN0.readMsgBuf(&rxId, &rxLen, rxBuf);
    if ((rxId & 0x7FF) == OBD2_RESPONSE_ID &&
        rxBuf[1] == 0x41 && rxBuf[2] == 0x0D) {
      atualizarCache((float)rxBuf[3]);
      tUltimaLeitura = millis();
    }
  }
}

// ===================== MENSAGENS ABS =====================

/*
 * 0x1A0 - Bremse_1 (8 bytes, cada 10ms)
 * byte2: bit0 = brake booster, bits 7..1 = speed low (7 bits)
 * byte3: speed high (8 bits)
 * byte7: bits 0-3 = counter, bit4 = ASR present
 */
void enviarBremse1(float speedKmh) {
  uint16_t speedRaw = (uint16_t)(speedKmh * 100.0);

  uint8_t data[8] = {
    0x00,
    0x00,
    (uint8_t)((speedRaw & 0x7F) << 1),
    (uint8_t)((speedRaw >> 7) & 0xFF),
    0xFE,
    0xFE,
    0x00,
    (uint8_t)((counter1A0 & 0x0F) | 0x10)
  };
  CAN0.sendMsgBuf(0x1A0, 0, 8, data);
  counter1A0 = (counter1A0 + 1) & 0x0F;
}

/*
 * 0x4A0 - Bremse_3 (8 bytes, cada 10ms)
 * 4 rodas: bit0 = direção, bits 15..1 = speed*100
 */
void enviarBremse3(float speedKmh) {
  uint16_t speedRaw = (uint16_t)(speedKmh * 100.0);
  uint16_t val = (speedRaw << 1) & 0xFFFE;
  uint8_t lo = val & 0xFF;
  uint8_t hi = (val >> 8) & 0xFF;
  uint8_t data[8] = { lo, hi, lo, hi, lo, hi, lo, hi };
  CAN0.sendMsgBuf(0x4A0, 0, 8, data);
}

/*
 * 0x5A0 - Bremse_2 (8 bytes, cada 20ms)
 * byte0: flags (0x00)
 * byte1-2: bits 15..1 = rotação média rodas, bit0 = flag
 * byte3-4: timestamp real (millis/2, unidade ~0.002s)
 * byte5: rolling counter
 * byte6-7: 0x00
 */
void enviarBremse2(float speedKmh) {
  uint16_t sr = (uint16_t)(speedKmh * SPEED_FACTOR_5A0);
  uint16_t val = (sr << 1) & 0xFFFE;
  uint16_t ts = (uint16_t)(millis() / 2);

  uint8_t data[8] = {
    0x00,
    (uint8_t)(val & 0xFF),
    (uint8_t)((val >> 8) & 0xFF),
    (uint8_t)(ts & 0xFF),
    (uint8_t)((ts >> 8) & 0xFF),
    counter5A0,
    0x00,
    0x00
  };
  CAN0.sendMsgBuf(0x5A0, 0, 8, data);
  counter5A0++;
}

void enviarTodasABS(float speedKmh) {
  unsigned long agora = millis();
  if (agora - tEnvio1A0 >= INTERVALO_1A0) {
    enviarBremse1(speedKmh);
    tEnvio1A0 = agora;
  }
  lerCAN();
  if (agora - tEnvio4A0 >= INTERVALO_4A0) {
    enviarBremse3(speedKmh);
    tEnvio4A0 = agora;
  }
  lerCAN();
  if (agora - tEnvio5A0 >= INTERVALO_5A0) {
    enviarBremse2(speedKmh);
    tEnvio5A0 = agora;
  }
}

// ===================== SETUP & LOOP =====================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("VW Jetta 2009 - ABS Bypass v4.1");

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_CAN_CS);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_CRYSTAL) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
    pinMode(PIN_CAN_INT, INPUT);
    canOk = true;
    Serial.println("CAN OK - 500kbps");
  } else {
    Serial.println("ERRO: MCP2515 falhou!");
  }

  tUltimoCache = millis();
}

void loop() {
  if (!canOk) { delay(1000); return; }

  unsigned long agora = millis();

  lerCAN();

  if (agora - tOBD2Req >= OBD2_INTERVAL) {
    enviarPedidoOBD2();
    tOBD2Req = agora;
  }

  lerCAN();
  decairCache();
  enviarTodasABS(velCache);

  if (agora - tLog >= 1000) {
    Serial.print("[v4.1] vel=");
    Serial.print(velCache, 1);
    Serial.print(" ts=");
    Serial.print((uint16_t)(millis() / 2));
    Serial.print(" obd=");
    Serial.print(agora - tUltimaLeitura);
    Serial.println("ms");
    tLog = agora;
  }
}
