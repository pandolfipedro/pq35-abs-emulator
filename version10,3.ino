/**
 * VW Jetta 2009 (PQ35) — emulação de mensagens ABS (MK60EC1) no ACAN
 *
 * Hardware: ESP32 + MCP2515 (8 MHz) + TJA1050
 * Biblioteca: ACAN2515 @ 500 kbit/s
 *
 * Layout CAN alinhado ao opendbc vw_pq.dbc (Bremse_1/2/3/10).
 * Velocidade: OBD-II Mode 01 PID 0x0D (vários endereços de ECU).
 *
 * Ajuste pinos / constantes em kConfig abaixo.
 */

#include <Arduino.h>
#include <esp_timer.h>
#include <SPI.h>
#include <cmath>
#include <cstring>
#include <ACAN2515.h>

// ---------------------------------------------------------------------------
// Configuração (valores típicos; personalize se necessário)
// ---------------------------------------------------------------------------
struct Config {
  // SPI / MCP2515
  uint8_t mcpCs = 5;
  uint8_t mcpInt = 2;
  uint32_t quartzHz = 8000000UL;
  uint32_t canBitrate = 500000UL;

  // Odometria / impulsos (impulsos por km — calibrar no veículo se preciso)
  float impulsesPerKm = 21960.0f;
  float tyreRadiusM = 0.315f;  // raio dinâmico efetivo (~205/55 R16)

  // Abaixo disto a velocidade efetiva = 0 (CAN + odometria). Evita hodômetro
  // subir aos poucos parado (ruído OBD 1 km/h, filtro residual, abastecimento longo).
  float standstillDeadBandKmh = 2.0f;

  // OBD2 (ISO 15765, mesmo barramento CAN)
  uint32_t obdRequestPeriodMs = 80;
  uint32_t obdTimeoutMs = 450;       // após isto: mantém última velocidade
  uint32_t obdHoldMaxMs = 2500;       // máximo segurando sem resposta válida
  float speedTauMs = 180.0f;          // constante de tempo filtro 1º ordem (ms)
  float speedMaxDecayKmhPerS = 35.0f; // decaimento suave após expirar hold

  // IDs OBD (11-bit)
  uint16_t obdFunctional = 0x7DF;
  uint16_t obdEngineReq = 0x7E0;
  uint16_t obdEngineRsp = 0x7E8;
  uint16_t obdTransReq = 0x7E1;
  uint16_t obdTransRsp = 0x7E9;

  // Janela de envio CAN (micros) — tolerância antes de ressincronizar
  int32_t maxScheduleSlipUs = 3500;

  // Perfil OEM para o Bremse_1 (0x1A0): preenche campos de momentos com 0xFE (254),
  // que no KMATRIX/DBC aparece como valor “padrão” quando não há intervenção.
  bool oemLikeBremse1 = true;
} kConfig;

// IDs mensagens ABS (decimal como no DBC)
static constexpr uint16_t kIdBremse1 = 0x1A0;   // 416
static constexpr uint16_t kIdBremse3 = 0x4A0;   // 1184
static constexpr uint16_t kIdBremse2 = 0x5A0;   // 1440
static constexpr uint16_t kIdBremse10 = 0x3A0;  // 928

static ACAN2515 gCan(kConfig.mcpCs, SPI, kConfig.mcpInt);

// ---------------------------------------------------------------------------
// Utilitários: bits little-endian (como no DBC Vector para sinais LE)
// ---------------------------------------------------------------------------
static inline void writeBitsLE(uint8_t *data, unsigned startBit, unsigned bitLen, uint32_t value) {
  for (unsigned i = 0; i < bitLen; ++i) {
    const unsigned bit = startBit + i;
    const unsigned byte = bit / 8u;
    const unsigned mask = 1u << (bit % 8u);
    if (value & (1u << i))
      data[byte] |= mask;
    else
      data[byte] &= uint8_t(~mask);
  }
}

static inline uint16_t speedToRaw200(float kmh) {
  if (kmh < 0.0f)
    kmh = 0.0f;
  const float x = kmh * 200.0f;
  if (x > 32766.0f)
    return 32766;
  return uint16_t(x + 0.5f);
}

// mittlere_Raddrehzahl (U/s): codificação observada via vw_pq.dbc + cantools
static inline uint16_t midRevsToRaw(float revPerSec) {
  if (revPerSec < 0.0f)
    revPerSec = 0.0f;
  const float r = revPerSec * 1000.0f + 1.0f;
  if (r >= 65535.0f)
    return 65535;
  return uint16_t(r + 0.5f);
}

static inline float kmhToMidRevs(float kmh) {
  const float v = kmh / 3.6f;  // m/s
  const float circ = 2.0f * float(M_PI) * kConfig.tyreRadiusM;
  if (circ <= 0.001f)
    return 0.0f;
  return v / circ;  // rev/s
}

/** Velocidade para CAN + odometria: zero dentro da zona morta (parado real). */
static inline float effectiveSpeedKmh(float filteredKmh) {
  const float a = fabsf(filteredKmh);
  if (a < kConfig.standstillDeadBandKmh)
    return 0.0f;
  return filteredKmh;
}

// ---------------------------------------------------------------------------
// Agendador monotônico (10 ms / 20 ms) com ressincronização anti-drift
// ---------------------------------------------------------------------------
struct PeriodicDeadline {
  int64_t nextUs = 0;
  const int32_t periodUs;

  explicit PeriodicDeadline(int32_t pUs) : periodUs(pUs) {}

  void startNow() { nextUs = esp_timer_get_time() + periodUs; }

  int64_t nextDeadlineUs() const { return nextUs; }

  void syncAfter(const PeriodicDeadline &other, int32_t offsetUs) {
    nextUs = other.nextDeadlineUs() + offsetUs;
  }

  bool poll() {
    const int64_t now = esp_timer_get_time();
    if (now < nextUs)
      return false;
    int64_t slip = now - nextUs;
    if (slip > kConfig.maxScheduleSlipUs) {
      nextUs = now + periodUs;
    } else {
      nextUs += periodUs;
    }
    return true;
  }
};

// ---------------------------------------------------------------------------
// OBD velocidade + filtro + hold em timeout
// ---------------------------------------------------------------------------
struct SpeedModel {
  float filteredKmh = 0.0f;
  float heldKmh = 0.0f;
  uint32_t lastGoodMs = 0;
  uint32_t lastAnyObdMs = 0;
  bool haveEver = false;

  void onObdSpeed(uint8_t rawKmh, uint32_t nowMs) {
    const uint32_t dtMs = nowMs - lastAnyObdMs; // wrap-safe
    heldKmh = float(rawKmh);
    lastGoodMs = nowMs;
    lastAnyObdMs = nowMs;
    haveEver = true;
    if (!isfinite(filteredKmh))
      filteredKmh = 0.0f;
    // ECU reporta 0 km/h → parado; sem isto o filtro pode manter resíduo e o hodômetro integra.
    if (rawKmh == 0) {
      filteredKmh = 0.0f;
      return;
    }
    // Alpha baseado no delta real entre amostras (evita drift se o loop atrasar).
    const float alpha = (kConfig.speedTauMs <= 1.0f)
                            ? 1.0f
                            : (1.0f - expf(-float(dtMs) / kConfig.speedTauMs));
    filteredKmh += (heldKmh - filteredKmh) * alpha;
  }

  void tick(uint32_t nowMs, float dtSec) {
    const uint32_t dtMs = nowMs - lastAnyObdMs; // wrap-safe (uint32_t)
    if (dtMs > kConfig.obdTimeoutMs) {
      if (haveEver && dtMs < kConfig.obdHoldMaxMs) {
        // mantém último valor filtrado (não zera por jitter OBD)
      } else if (haveEver && dtMs >= kConfig.obdHoldMaxMs) {
        const float dec = kConfig.speedMaxDecayKmhPerS * dtSec;
        filteredKmh = fmaxf(0.0f, filteredKmh - dec);
        heldKmh = filteredKmh;
      }
    }
  }

  float displayKmh() const {
    if (!haveEver)
      return 0.0f;
    return filteredKmh;
  }
};

static SpeedModel gSpeed;

// ---------------------------------------------------------------------------
// Odometria (integração velocidade → impulsos)
// ---------------------------------------------------------------------------
struct Odometer {
  double impulseFrac = 0.0;   // acumulador fracionário
  uint32_t totalImpulses = 0;

  void reset() {
    impulseFrac = 0.0;
    totalImpulses = 0;
  }

  void integrate(float kmh, float dtSec) {
    // `kmh` já chega com zona morta aplicada (effectiveSpeedKmh).
    const double impPerM = (double)kConfig.impulsesPerKm / 1000.0;
    const double vMs = (double)kmh / 3.6;
    impulseFrac += vMs * dtSec * impPerM;
    if (impulseFrac >= 1.0) {
      const uint32_t add = (uint32_t)floor(impulseFrac);
      totalImpulses += add;
      impulseFrac -= (double)add;
    }
  }

  uint16_t weg11Front() const { return uint16_t(totalImpulses & 0x7FFu); }
  uint8_t imp6() const { return uint8_t(totalImpulses & 0x3Fu); }
  uint16_t weg10Wheel() const { return uint16_t(totalImpulses & 0x3FFu); }
};

static Odometer gOdo;

// ---------------------------------------------------------------------------
// OBD2 ISO-TP (single frame) — PID 0x0D
// ---------------------------------------------------------------------------
struct ObdClient {
  uint32_t lastTxMs = 0;

  void setup() { lastTxMs = 0; }

  void trySendRequests(uint32_t nowMs) {
    if (nowMs - lastTxMs < kConfig.obdRequestPeriodMs)
      return;
    lastTxMs = nowMs;

    CANMessage m;
    memset(&m, 0, sizeof(m));
    m.len = 8;
    m.ext = false;
    m.rtr = false;

    // Velocidade vem apenas da TCU (0x7E9)
    m.id = kConfig.obdTransReq;
    m.data[0] = 0x02;
    m.data[1] = 0x01;
    m.data[2] = 0x0D;
    (void)gCan.tryToSend(m);

    // ocasionalmente broadcast funcional (ajuda se endereço ECU variar)
    static uint8_t s = 0;
    s++;
    if ((s % 6u) == 0u) {
      m.id = kConfig.obdFunctional;
      (void)gCan.tryToSend(m);
    }
  }

  void pollReceive(uint32_t nowMs) {
    CANMessage m;
    while (gCan.receive(m)) {
      if (m.len < 4 || m.rtr)
        continue;
      if (m.data[1] != 0x41 || m.data[2] != 0x0D)
        continue;
      const uint8_t nBytes = m.data[0];
      if (nBytes < 3)
        continue;
      const uint8_t v = m.data[3];
      gSpeed.onObdSpeed(v, nowMs);
    }
  }
};

static ObdClient gObd;

// ---------------------------------------------------------------------------
// Montagem frames ABS
// ---------------------------------------------------------------------------
static uint8_t gBr1Zaehler = 0;
static uint8_t gB10Zaehler = 0;

static void buildBremse1(uint8_t *d, float kmh) {
  memset(d, 0, 8);
  // Layout conforme vw_pq.dbc (Bremse_1 / 0x1A0).
  // Mantemos todos os status em 0 (sem intervenção / sem lâmpadas) e setamos apenas:
  // - BR1_Rad_kmh @ bit 17 len 15, fator 0.01 (km/h)
  // - BR1_Zaehler @ bit 56 len 4
  // - BR1_Ersatz_Kmh @ bit 63 (0 = OK)
  // Opcionalmente (modo OEM), também:
  // - BR1_ASRMo_sl @ bit 32 len 8  (0xFE)
  // - BR1_ASRMo_fa @ bit 40 len 8  (0xFE)
  // - BR1_MSR_Mo   @ bit 48 len 8  (0xFE)
  if (kmh < 0.0f)
    kmh = 0.0f;
  const float rawF = kmh * 100.0f;           // 0.01 km/h por bit
  const uint32_t raw = (rawF > 32767.0f) ? 32767u : uint32_t(rawF + 0.5f);
  writeBitsLE(d, 17, 15, raw);
  if (kConfig.oemLikeBremse1) {
    writeBitsLE(d, 32, 8, 0xFE);
    writeBitsLE(d, 40, 8, 0xFE);
    writeBitsLE(d, 48, 8, 0xFE);
  }
  writeBitsLE(d, 56, 4, gBr1Zaehler & 0x0Fu);
  writeBitsLE(d, 63, 1, 0); // Ersatz_Kmh = OK
}

static void buildBremse3(uint8_t *d, float kmh) {
  memset(d, 0, 8);
  const uint16_t w = speedToRaw200(kmh);
  d[0] = uint8_t(w & 0xFFu);
  d[1] = uint8_t((w >> 8) & 0xFFu);
  d[2] = uint8_t(w & 0xFFu);
  d[3] = uint8_t((w >> 8) & 0xFFu);
  d[4] = uint8_t(w & 0xFFu);
  d[5] = uint8_t((w >> 8) & 0xFFu);
  d[6] = uint8_t(w & 0xFFu);
  d[7] = uint8_t((w >> 8) & 0xFFu);
}

static void buildBremse2(uint8_t *d, float kmh, uint16_t zeitTicks, uint16_t weg11, uint8_t imp6) {
  memset(d, 0, 8);
  // Estrutura conforme vw_pq.dbc (Bremse_2 / 0x5A0).
  // - Querbeschl__TimerTic @ bit 8 (mux): 1 => ramo Timer
  // - Timer @ bit 0 len 8, fator 0.04: valor 250 => 10.0
  // - mittlere_Raddrehzahl__Bremse_2 @ bit 9 len 15, fator 0.002 (U/s)
  // - Zeitstempel @ bit 24 len 16 (tics)
  // - Wegimpulse_Vorderachse @ bit 40 len 11
  // - Impulszahl @ bit 56 len 6
  writeBitsLE(d, 8, 1, 1); // Querbeschl__TimerTic = 1 (Timer branch)
  d[0] = 250;              // Timer = 10.0 (250 * 0.04)

  const float mid = kmhToMidRevs(kmh);
  const uint16_t rawMid = uint16_t(midRevsToRaw(mid) & 0x7FFFu); // campo é 15-bit
  writeBitsLE(d, 9, 15, rawMid);

  writeBitsLE(d, 24, 16, zeitTicks);
  writeBitsLE(d, 40, 11, uint32_t(weg11 & 0x7FFu));
  writeBitsLE(d, 56, 6, uint32_t(imp6 & 0x3Fu));
}

static void buildBremse10(uint8_t *d, uint16_t weg10each) {
  memset(d, 0, 8);
  // B10_Zaehler @ bit 8 len 4
  writeBitsLE(d, 8, 4, gB10Zaehler & 0x0Fu);
  // Qualidade bits = 1 (válido)
  writeBitsLE(d, 12, 1, 1);
  writeBitsLE(d, 13, 1, 1);
  writeBitsLE(d, 14, 1, 1);
  writeBitsLE(d, 15, 1, 1);
  const uint32_t w = uint32_t(weg10each & 0x3FFu);
  writeBitsLE(d, 16, 10, w);
  writeBitsLE(d, 26, 10, w);
  writeBitsLE(d, 36, 10, w);
  writeBitsLE(d, 46, 10, w);
  // QB Fahrtr_* @ 56..59 = 1 (válido); Fahrtr_* @ 60..63 = 0 — alinha encode padrão vw_pq.dbc
  writeBitsLE(d, 56, 1, 1);
  writeBitsLE(d, 57, 1, 1);
  writeBitsLE(d, 58, 1, 1);
  writeBitsLE(d, 59, 1, 1);
  writeBitsLE(d, 60, 1, 0);
  writeBitsLE(d, 61, 1, 0);
  writeBitsLE(d, 62, 1, 0);
  writeBitsLE(d, 63, 1, 0);
  // Checksum: XOR bytes 1..7 (comum em VAG para este layout)
  uint8_t x = 0;
  for (int i = 1; i < 8; ++i)
    x ^= d[i];
  d[0] = x;
}

static inline bool sendFrame(uint16_t id, const uint8_t *data8) {
  CANMessage m;
  memset(&m, 0, sizeof(m));
  m.id = id;
  m.len = 8;
  m.ext = false;
  m.rtr = false;
  memcpy(m.data, data8, 8);
  return gCan.tryToSend(m);
}

static void IRAM_ATTR canISR() { gCan.isr(); }

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("MK60 PQ35 ABS emulator — OBD PID 0x0D + CAN 0x1A0/0x4A0/0x5A0/0x3A0"));

  SPI.begin(18, 19, 23);

  ACAN2515Settings settings(kConfig.quartzHz, kConfig.canBitrate);
  settings.mRequestedMode = ACAN2515Settings::NormalMode;
  // Filtros RX (hardware MCP2515):
  // - RXB0 (RXM0 + filtros 0..1): aceita somente 0x7E9
  // - RXB1 (RXM1 + filtros 2..5): bloqueia tudo (ID=0x000 com máscara 0x7FF)
  const ACAN2515Mask rxm = standard2515Mask(0x7FF, 0x00, 0x00); // compara todos os 11 bits do ID padrão
  const ACAN2515AcceptanceFilter filters[] = {
      {standard2515Filter(kConfig.obdTransRsp, 0x00, 0x00), nullptr},
      {standard2515Filter(kConfig.obdTransRsp, 0x00, 0x00), nullptr},
      {standard2515Filter(0x000, 0x00, 0x00), nullptr},
      {standard2515Filter(0x000, 0x00, 0x00), nullptr},
      {standard2515Filter(0x000, 0x00, 0x00), nullptr},
      {standard2515Filter(0x000, 0x00, 0x00), nullptr},
  };
  const uint16_t err =
      gCan.begin(settings, canISR, rxm, rxm, filters, uint8_t(sizeof(filters) / sizeof(filters[0])));
  if (err != 0) {
    Serial.print(F("ACAN2515 begin erro: 0x"));
    Serial.println(err, HEX);
    while (true)
      delay(500);
  }
  Serial.println(F("MCP2515 OK @ 500k"));

  gObd.setup();
}

void loop() {
  const uint32_t ms = millis();
  static uint32_t lastLoopMs = ms;
  const float dtSec = fmaxf((ms - lastLoopMs) / 1000.0f, 0.0005f);
  lastLoopMs = ms;

  gObd.pollReceive(ms);
  gObd.trySendRequests(ms);
  gSpeed.tick(ms, dtSec);

  const float kmhFilt = gSpeed.displayKmh();
  const float kmh = effectiveSpeedKmh(kmhFilt);
  gOdo.integrate(kmh, dtSec);

  static PeriodicDeadline t10(10000);
  static PeriodicDeadline t20(20000);
  static bool tInit = false;
  if (!tInit) {
    t10.startNow();
    t20.syncAfter(t10, 10000);  // 0x5A0/0x3A0 defasados 10 ms em relação ao par 10 ms
    tInit = true;
  }

  static uint16_t sZeit = 0;

  uint8_t b1[8], b3[8], b2[8], b10[8];
  static uint32_t txFail1A0 = 0, txFail4A0 = 0, txFail5A0 = 0, txFail3A0 = 0;

  // 10 ms: 0x1A0 e 0x4A0
  if (t10.poll()) {
    sZeit = uint16_t(sZeit + 10);  // incremento coerente com tick 10 ms
    buildBremse1(b1, kmh);
    buildBremse3(b3, kmh);
    if (!sendFrame(kIdBremse1, b1))
      txFail1A0++;
    if (!sendFrame(kIdBremse3, b3))
      txFail4A0++;
    gBr1Zaehler = uint8_t((gBr1Zaehler + 1u) & 0x0Fu);
  }

  // 20 ms: 0x5A0 e 0x3A0
  if (t20.poll()) {
    buildBremse2(b2, kmh, sZeit, gOdo.weg11Front(), gOdo.imp6());
    buildBremse10(b10, gOdo.weg10Wheel());
    if (!sendFrame(kIdBremse2, b2))
      txFail5A0++;
    if (!sendFrame(kIdBremse10, b10))
      txFail3A0++;
    gB10Zaehler = uint8_t((gB10Zaehler + 1u) & 0x0Fu);
  }

  // Diagnóstico leve (1 Hz)
  static uint32_t lastLog = 0;
  if (ms - lastLog > 1000) {
    lastLog = ms;
    Serial.print(F("km/h (filt/efet): "));
    Serial.print(kmhFilt, 1);
    Serial.print('/');
    Serial.print(kmh, 1);
    Serial.print(F(" | impulsos: "));
    Serial.print(gOdo.totalImpulses);
    Serial.print(F(" | RX buf: "));
    Serial.println(gCan.receiveBufferCount());
    Serial.print(F("TX fail 1A0/4A0/5A0/3A0: "));
    Serial.print(txFail1A0);
    Serial.print('/');
    Serial.print(txFail4A0);
    Serial.print('/');
    Serial.print(txFail5A0);
    Serial.print('/');
    Serial.println(txFail3A0);
  }
}
