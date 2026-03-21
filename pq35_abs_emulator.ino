/*
 * VW Jetta 2009 - Bypass ABS via CAN Bus
 * ESP32 + MCP2515 (TJA1050)
 *
 * Lê velocidade da TCU via OBD2 (0x7E1/0x7E9)
 * e envia mensagens ABS (0x1A0, 0x4A0, 0x5A0)
 *
 * MODOS:
 *   0 = BYPASS - Uso normal no veículo
 *   2 = TESTE  - Velocidade manual via Serial
 *   4 = OBD2   - Teste de comunicação OBD2
 */

#include <SPI.h>
#include <mcp_can.h>

// ===================== CONFIGURAÇÃO =====================

#define MODO_OPERACAO  0

// Pinos MCP2515
#define PIN_CAN_CS    5
#define PIN_CAN_INT   2
#define PIN_SPI_SCK   18
#define PIN_SPI_MISO  19
#define PIN_SPI_MOSI  23
#define MCP_CRYSTAL   MCP_8MHZ

// OBD2 - TCU (câmbio)
#define OBD2_REQUEST_ID   0x7E1
#define OBD2_RESPONSE_ID  0x7E9
#define OBD2_INTERVAL     80    // ms entre pedidos

// Calibração do velocímetro (0x5A0)
// Pneu 205/55R16 → circunferência ~1.985m → fator ~70
// Se painel mostra MUITO: diminua. POUCO: aumente.
#define SPEED_FACTOR_5A0    70.0
#define SPEED_FACTOR_WHEEL  100.0

// Intervalos de envio das mensagens ABS (ms)
#define INTERVALO_1A0  10
#define INTERVALO_4A0  10
#define INTERVALO_5A0  20

// Desaceleração máxima permitida no cache (km/h por segundo)
// ~1g de frenagem ≈ 36 km/h/s. Margem extra para segurança.
#define MAX_DESACEL    40.0

// ===================== VARIÁVEIS =====================

MCP_CAN CAN0(PIN_CAN_CS);
bool canOk = false;

long unsigned int rxId;
unsigned char rxLen = 0;
unsigned char rxBuf[8];

// Cache de velocidade
float velCache = 0.0;           // Velocidade "confiável" usada pelo painel
unsigned long tUltimaLeitura = 0; // Quando recebeu última resposta OBD2
unsigned long tUltimoCache = 0;   // Quando atualizou o cache por último

// Contadores para mensagens ABS
uint8_t contadorAlive = 0;
uint16_t timestamp5A0 = 0;

// Timers
unsigned long tEnvio1A0 = 0, tEnvio4A0 = 0, tEnvio5A0 = 0;
unsigned long tOBD2Req = 0, tLog = 0;

// ===================== CACHE DE VELOCIDADE =====================
// Se está a 20 km/h e chega 0, isso é impossível em 80ms.
// O cache só permite queda proporcional à frenagem real.
// Aceleração é livre (resposta rápida ao acelerar).

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

// Decaimento natural: se não chega resposta, freia suavemente
void decairCache() {
  unsigned long agora = millis();
  if (agora - tUltimaLeitura > 300) {
    float dt = (agora - tUltimoCache) / 1000.0;
    float queda = MAX_DESACEL * 0.5 * dt;  // decai mais devagar que frenagem
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

bool lerRespostaOBD2() {
  bool recebeu = false;
  while (digitalRead(PIN_CAN_INT) == LOW) {
    CAN0.readMsgBuf(&rxId, &rxLen, rxBuf);
    unsigned long id = rxId & 0x7FF;

    if (id == OBD2_RESPONSE_ID && rxBuf[1] == 0x41 && rxBuf[2] == 0x0D) {
      float vel = (float)rxBuf[3];
      atualizarCache(vel);
      tUltimaLeitura = millis();
      recebeu = true;
    }
  }
  return recebeu;
}

// ===================== MENSAGENS ABS =====================

void enviarBremse1(float speedKmh) {
  uint16_t sv = (uint16_t)(speedKmh * SPEED_FACTOR_WHEEL);
  uint8_t data[8] = {
    (uint8_t)(speedKmh > 0.5 ? 0x01 : 0x00),
    0x00,
    (uint8_t)(sv & 0xFF),
    (uint8_t)((sv >> 8) & 0xFF),
    0xFE, 0xFE,
    (uint8_t)((contadorAlive & 0x0F) << 4),
    0x00
  };
  CAN0.sendMsgBuf(0x1A0, 0, 8, data);
}

void enviarBremse3(float speedKmh) {
  uint16_t ws = (uint16_t)(speedKmh * SPEED_FACTOR_WHEEL) & 0xFFFE;
  uint8_t lo = ws & 0xFF;
  uint8_t hi = (ws >> 8) & 0xFF;
  uint8_t data[8] = { lo, hi, lo, hi, lo, hi, lo, hi };
  CAN0.sendMsgBuf(0x4A0, 0, 8, data);
}

void enviarBremse2(float speedKmh) {
  uint16_t sr = (uint16_t)(speedKmh * SPEED_FACTOR_5A0);
  uint16_t val = (sr << 1) & 0xFFFE;
  timestamp5A0++;
  uint8_t data[8] = {
    0x80,
    (uint8_t)(val & 0xFF),
    (uint8_t)((val >> 8) & 0xFF),
    (uint8_t)(timestamp5A0 & 0xFF),
    (uint8_t)((timestamp5A0 >> 8) & 0xFF),
    0x00, 0x00, 0xAD
  };
  CAN0.sendMsgBuf(0x5A0, 0, 8, data);
  contadorAlive++;
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

// ===================== MODO BYPASS =====================

void loopBypass() {
  unsigned long agora = millis();

  if (agora - tOBD2Req >= OBD2_INTERVAL) {
    enviarPedidoOBD2();
    tOBD2Req = agora;
  }

  lerRespostaOBD2();
  decairCache();
  enviarTodasABS(velCache);

  if (agora - tLog >= 1000) {
    Serial.print("[BYPASS] ");
    Serial.print(velCache, 0);
    Serial.println(" km/h");
    tLog = agora;
  }
}

// ===================== MODO TESTE =====================

void loopTeste() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      float v = input.toFloat();
      if (v >= 0 && v <= 280) {
        velCache = v;
        Serial.print("[TESTE] ");
        Serial.print(v, 0);
        Serial.println(" km/h");
      }
    }
  }

  enviarTodasABS(velCache);

  unsigned long agora = millis();
  if (agora - tLog >= 2000) {
    Serial.print("[TESTE] ");
    Serial.print(velCache, 0);
    Serial.println(" km/h");
    tLog = agora;
  }
}

// ===================== MODO OBD2 (teste) =====================

void loopOBD2() {
  static unsigned long tReq = 0, tPrint = 0;
  static uint32_t reqCount = 0, rspCount = 0;
  unsigned long agora = millis();

  if (agora - tReq >= 200) {
    enviarPedidoOBD2();
    reqCount++;
    tReq = agora;
  }

  while (digitalRead(PIN_CAN_INT) == LOW) {
    CAN0.readMsgBuf(&rxId, &rxLen, rxBuf);
    if ((rxId & 0x7FF) == OBD2_RESPONSE_ID && rxBuf[1] == 0x41 && rxBuf[2] == 0x0D) {
      rspCount++;
      Serial.print("[OBD2] ");
      Serial.print(rxBuf[3]);
      Serial.println(" km/h");
    }
  }

  enviarTodasABS(0);

  if (agora - tPrint >= 3000) {
    Serial.print("[OBD2] req=");
    Serial.print(reqCount);
    Serial.print(" rsp=");
    Serial.println(rspCount);
    tPrint = agora;
  }
}

// ===================== SETUP & LOOP =====================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("VW Jetta 2009 - ABS Bypass v2.0");

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_CAN_CS);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_CRYSTAL) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
    pinMode(PIN_CAN_INT, INPUT);
    canOk = true;
    Serial.println("CAN OK - 500kbps");
  } else {
    Serial.println("ERRO: MCP2515 falhou!");
  }

  switch (MODO_OPERACAO) {
    case 0: Serial.println("Modo: BYPASS"); break;
    case 2: Serial.println("Modo: TESTE (digite km/h)"); break;
    case 4: Serial.println("Modo: OBD2 (teste)"); break;
  }

  tUltimoCache = millis();
}

void loop() {
  if (!canOk) { delay(1000); return; }

  switch (MODO_OPERACAO) {
    case 0: loopBypass(); break;
    case 2: loopTeste();  break;
    case 4: loopOBD2();   break;
  }
}
