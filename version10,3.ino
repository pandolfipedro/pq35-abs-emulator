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

  // Odometria / impulsos (impulsos por km — calibrar no veículo se preciso).
  // Em PQ35, "distance impulse number" típico do painel é 21960. Esse número bate
  // com pulsos de roda (subida+descida) e é usado para Wegimpulse no mBremse_2/10.
  float impulsesPerKm = 21960.0f;
  // Calibração de distância: serial a ~20 km/h mostrou ~10× impulsos vs teoria (21960/km).
  // Ajuste fino: km_reais / km_indicados no painel, ou 1.0 se já bater.
  float odoDistanceScale = 0.102f;
  float tyreRadiusM = 0.315f;  // raio dinâmico efetivo (~205/55 R16)

  // Abaixo disto a velocidade efetiva = 0 (CAN + odometria). Evita hodômetro
  // subir aos poucos parado (ruído OBD 1 km/h, filtro residual, abastecimento longo).
  float standstillDeadBandKmh = 2.0f;

  // OBD2 (ISO 15765, mesmo barramento CAN)
  uint32_t obdRequestPeriodMs = 80;
  uint32_t obdTimeoutMs = 600;       // após isto: mantém última velocidade (RX pode atrasar com TX denso)
  uint32_t obdHoldMaxMs = 3500;       // máximo segurando sem resposta válida
  float speedTauMs = 180.0f;          // constante de tempo filtro 1º ordem (ms)
  float speedMaxDecayKmhPerS = 12.0f; // decaimento após falta de OBD (menos agressivo → menos queda de ponteiro)

  // Limite de rampa só no CAN ABS (ponteiro): evita “queda” quando o filtro dá um salto a 0 por 1–2 frames.
  float absCanSlewUpKmhPerS = 220.0f;
  float absCanSlewDownKmhPerS = 65.0f; // subir se o ponteiro ficar “preguiçoso” a descer ao travar

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

static inline uint16_t speedToRaw100(float kmh) {
  if (kmh < 0.0f)
    kmh = 0.0f;
  // Bremse_3 em PQ35 tipicamente usa 0.01 km/h por bit (raw = km/h * 100).
  // O fator 200 fazia o painel marcar ~2x.
  const float x = kmh * 100.0f;
  if (x > 32766.0f)
    return 32766;
  return uint16_t(x + 0.5f);
}

// mittlere_Raddrehzahl (U/s): vw_pq.dbc — fator 0,002 (raw = phys / 0,002 = phys * 500).
// O cantools confirma: o antigo *1000 + 1 dobrava a rotação no 0x5A0.
static inline uint16_t midRevsToRaw(float revPerSec) {
  if (revPerSec < 0.0f)
    revPerSec = 0.0f;
  float r = revPerSec / 0.002f;
  // Com rev=0 o encode OEM usa raw=1 (init), mantendo decode em 0 U/s.
  if (r < 1.0f)
    r = 1.0f;
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

/** Só para hodômetro: zero na zona morta (evita subir parado com ruído OBD). */
static inline float effectiveSpeedKmhForOdo(float filteredKmh) {
  const float a = fabsf(filteredKmh);
  if (a < kConfig.standstillDeadBandKmh)
    return 0.0f;
  return filteredKmh;
}

/** Velocidade alvo nos frames ABS (antes do slew): segue o filtro OBD, sem “corte” a 2 km/h. */
static inline float speedForAbsCan(float filteredKmh) {
  if (!isfinite(filteredKmh) || filteredKmh < 0.0f)
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
  uint32_t lastPositiveKmhMs = 0;
  bool haveEver = false;
  uint8_t zeroRun = 0;

  void onObdSpeed(uint8_t rawKmh, uint32_t nowMs) {
    uint32_t dtMs = nowMs - lastAnyObdMs; // wrap-safe
    if (dtMs == 0) dtMs = 1;
    if (dtMs > 1000) dtMs = 1000; // evita alpha=1 por "buracos" longos
    heldKmh = float(rawKmh);
    lastGoodMs = nowMs;
    lastAnyObdMs = nowMs;
    haveEver = true;
    if (!isfinite(filteredKmh))
      filteredKmh = 0.0f;
    if (rawKmh > 0)
      lastPositiveKmhMs = nowMs;
    // Um único frame 0 com TCU ainda reportando movimento há <120 ms: ignora (glitch).
    // Janela curta para não atrasar parada real após desaceleração.
    if (rawKmh == 0 && filteredKmh > 12.0f && lastPositiveKmhMs != 0 &&
        (nowMs - lastPositiveKmhMs) < 120u) {
      return;
    }
    if (rawKmh == 0)
      zeroRun = (zeroRun < 255) ? uint8_t(zeroRun + 1u) : 255;
    else
      zeroRun = 0;
    // Alpha baseado no delta real entre amostras (evita drift se o loop atrasar).
    const float alpha = (kConfig.speedTauMs <= 1.0f)
                            ? 1.0f
                            : (1.0f - expf(-float(dtMs) / kConfig.speedTauMs));
    // 0 km/h espúrio no OBD acontece; não derruba o ponteiro instantaneamente.
    // Parado real no hodômetro: zona morta em effectiveSpeedKmhForOdo + decaimento do filtro.
    if (rawKmh == 0 && filteredKmh > 3.0f && zeroRun < 10u) {
      // ignora sequências curtas de 0 (~10 amostras @ ~80 ms)
      return;
    }
    if (rawKmh == 0 && filteredKmh > 3.0f) {
      const float alphaZero = fminf(alpha, 0.25f); // limita o degrau pra baixo
      filteredKmh += (0.0f - filteredKmh) * alphaZero;
      return;
    }
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

  bool haveObdEver() const { return haveEver; }
  uint8_t zeroRunCount() const { return zeroRun; }
  float heldRawKmh() const { return heldKmh; }
};

static SpeedModel gSpeed;

/** Rampa no CAN ABS — picos a 0 no filtro não viram 0 instantâneo no ponteiro.
 *  Só usa tipos built-in: o IDE junta .ino por nome e pode compilar antes da struct SpeedModel. */
static float slewLimitAbsCan(float targetKmh, float dtSec, bool haveObdEver, uint8_t zeroRun, float heldRawKmh,
                             float kmhFilt) {
  static float sOut = 0.0f;
  const float dt = fmaxf(dtSec, 0.0005f);
  if (!haveObdEver) {
    sOut = 0.0f;
    return 0.0f;
  }
  const float tgt = fmaxf(0.0f, targetKmh);
  const bool stopOk =
      (kmhFilt < 0.6f && zeroRun >= 12u) || (heldRawKmh < 0.5f && zeroRun >= 12u);
  if (stopOk) {
    sOut = tgt;
    return sOut;
  }
  const float up = kConfig.absCanSlewUpKmhPerS * dt;
  const float dn = kConfig.absCanSlewDownKmhPerS * dt;
  if (tgt > sOut)
    sOut = fminf(tgt, sOut + up);
  else
    sOut = fmaxf(tgt, sOut - dn);
  return sOut;
}

// ---------------------------------------------------------------------------
// Odometria (integração velocidade → impulsos)
// ---------------------------------------------------------------------------
// Um único contador de impulsos de distância (por km); Bremse_2 recebe o baixo
// 11 bits. Bremse_10 reparte o mesmo total em 4 rodas (q+r/4) para que, se o
// Kombi somar VL+VR+HL+HR, a soma ≈ impulsos reais — antes as 4 iguais somavam ~4×.
// Impulszahl fica em 0: muitos clusters já integram só o Wegimpulse cumulativo;
// enviar delta + contador pode dobrar a distância.
static inline void splitVehicleImpToWheels(uint32_t v, uint16_t *vl, uint16_t *vr, uint16_t *hl, uint16_t *hr) {
  const uint32_t q = v / 4u;
  const uint32_t r = v % 4u;
  *vl = uint16_t((q + (r > 0u ? 1u : 0u)) & 0x3FFu);
  *vr = uint16_t((q + (r > 1u ? 1u : 0u)) & 0x3FFu);
  *hl = uint16_t((q + (r > 2u ? 1u : 0u)) & 0x3FFu);
  *hr = uint16_t((q + (r > 3u ? 1u : 0u)) & 0x3FFu);
}

struct Odometer {
  double impulseFrac = 0.0;
  uint32_t vehImpulses = 0;
  uint32_t vehImpulsesAtLastB2 = 0;

  void reset() {
    impulseFrac = 0.0;
    vehImpulses = 0;
    vehImpulsesAtLastB2 = 0;
  }

  void integrate(float kmh, float dtSec) {
    const double impPerM = (double)kConfig.impulsesPerKm / 1000.0 * (double)kConfig.odoDistanceScale;
    const double vMs = (double)kmh / 3.6;
    impulseFrac += vMs * dtSec * impPerM;
    if (impulseFrac >= 1.0) {
      const uint32_t add = (uint32_t)floor(impulseFrac);
      vehImpulses += add;
      impulseFrac -= (double)add;
    }
  }

  uint16_t weg11FrontAxle() const { return uint16_t(vehImpulses & 0x7FFu); }

  /** Impulsos desde o último 0x5A0 (coerente com o incremento de Wegimpulse). */
  uint8_t impulszahlForBremse2() {
    const uint32_t d = vehImpulses - vehImpulsesAtLastB2;
    vehImpulsesAtLastB2 = vehImpulses;
    return (d > 63u) ? 63u : uint8_t(d);
  }

  void weg10Wheels(uint16_t *vl, uint16_t *vr, uint16_t *hl, uint16_t *hr) const {
    splitVehicleImpToWheels(vehImpulses, vl, vr, hl, hr);
  }

  uint32_t totalImpulsesForLogging() const { return vehImpulses; }
};

static Odometer gOdo;

// ---------------------------------------------------------------------------
// OBD2 ISO15765 — PID 0x0D (velocidade)
// Muitos PQ35 respondem no MOTOR 0x7E8 (pedido 0x7E0), não na TCU 0x7E9.
// Filtro MCP2515 tem de aceitar 0x7E8 e 0x7E9. Payload pode ser 03 41 0D XX ou outro PCI.
// ---------------------------------------------------------------------------
static bool extractMode01Pid0d(const CANMessage &m, uint8_t *outKmh) {
  if (m.rtr || m.len < 3)
    return false;
  if (m.len >= 2 && m.data[0] == 0x03u && m.data[1] == 0x7Fu)
    return false; // resposta negativa
  for (unsigned i = 0; i + 2u < (unsigned)m.len && i < 7u; ++i) {
    if (m.data[i] == 0x41u && m.data[i + 1] == 0x0Du) {
      *outKmh = m.data[i + 2];
      return true;
    }
  }
  return false;
}

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
    m.data[0] = 0x02;
    m.data[1] = 0x01;
    m.data[2] = 0x0D;

    m.id = kConfig.obdEngineReq;
    (void)gCan.tryToSend(m);
    m.id = kConfig.obdTransReq;
    (void)gCan.tryToSend(m);

    static uint8_t s = 0;
    s++;
    if ((s % 8u) == 0u) {
      m.id = kConfig.obdFunctional;
      memset(m.data, 0, 8);
      m.data[0] = 0x02;
      m.data[1] = 0x01;
      m.data[2] = 0x0D;
      (void)gCan.tryToSend(m);
    }
  }

  void pollReceive(uint32_t nowMs) {
    CANMessage m;
    while (gCan.receive(m)) {
      const uint16_t id = m.id;
      if (id != kConfig.obdEngineRsp && id != kConfig.obdTransRsp)
        continue;
      uint8_t v = 0;
      if (!extractMode01Pid0d(m, &v))
        continue;
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
  // vw_pq.dbc BO_ 1184 Bremse_3: 4× Radgeschw_* @ 0,01 km/h (15 bit), separados por 1 bit "Frei".
  // Não repetir uint16 em todo o payload — isso quebrava o layout e gerava leitura absurda no Kombi.
  if (kmh < 0.0f)
    kmh = 0.0f;
  const uint32_t raw = speedToRaw100(kmh); // 0,01 km/h por unidade, 15 bit
  writeBitsLE(d, 0, 1, 0);   // Frei_Bremse_3_1
  writeBitsLE(d, 1, 15, raw); // Radgeschw__VL_4_1
  writeBitsLE(d, 16, 1, 0);  // Frei_Bremse_3_2
  writeBitsLE(d, 17, 15, raw); // Radgeschw__VR_4_1
  writeBitsLE(d, 32, 1, 0);  // Frei_Bremse_3_3
  writeBitsLE(d, 33, 15, raw); // Radgeschw__HL_4_1
  writeBitsLE(d, 48, 1, 0);  // Frei_Bremse_3_4
  writeBitsLE(d, 49, 15, raw); // Radgeschw__HR_4_1
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

static void buildBremse10(uint8_t *d, uint16_t weg10VL, uint16_t weg10VR, uint16_t weg10HL, uint16_t weg10HR) {
  memset(d, 0, 8);
  // B10_Zaehler @ bit 8 len 4
  writeBitsLE(d, 8, 4, gB10Zaehler & 0x0Fu);
  // Qualidade bits = 1 (válido)
  writeBitsLE(d, 12, 1, 1);
  writeBitsLE(d, 13, 1, 1);
  writeBitsLE(d, 14, 1, 1);
  writeBitsLE(d, 15, 1, 1);
  writeBitsLE(d, 16, 10, uint32_t(weg10VL & 0x3FFu));
  writeBitsLE(d, 26, 10, uint32_t(weg10VR & 0x3FFu));
  writeBitsLE(d, 36, 10, uint32_t(weg10HL & 0x3FFu));
  writeBitsLE(d, 46, 10, uint32_t(weg10HR & 0x3FFu));
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
  // Filtros RX: OBD responde em 0x7E8 (motor) e/ou 0x7E9 (trans) — aceitar os dois.
  const ACAN2515Mask rxm = standard2515Mask(0x7FF, 0x00, 0x00); // compara todos os 11 bits do ID padrão
  const ACAN2515AcceptanceFilter filters[] = {
      {standard2515Filter(kConfig.obdEngineRsp, 0x00, 0x00), nullptr},
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
  const float kmhCan = speedForAbsCan(kmhFilt);
  const float kmhAbsTx =
      slewLimitAbsCan(kmhCan, dtSec, gSpeed.haveObdEver(), gSpeed.zeroRunCount(), gSpeed.heldRawKmh(), kmhFilt);
  const float kmhOdo = effectiveSpeedKmhForOdo(kmhFilt);
  // Evita um único loop longo (bloqueio) integrar hodômetro como se fossem vários s a ~20 km/h.
  const float dtOdo = fminf(dtSec, 0.08f);
  gOdo.integrate(kmhOdo, dtOdo);
  gObd.pollReceive(ms); // esvazia RX após TX (OBD pode ter chegado durante envio ABS)

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
    buildBremse1(b1, kmhAbsTx);
    buildBremse3(b3, kmhAbsTx);
    if (!sendFrame(kIdBremse1, b1))
      txFail1A0++;
    if (!sendFrame(kIdBremse3, b3))
      txFail4A0++;
    gBr1Zaehler = uint8_t((gBr1Zaehler + 1u) & 0x0Fu);
  }

  // 20 ms: 0x5A0 e 0x3A0
  if (t20.poll()) {
    uint16_t wvl, wvr, whl, whr;
    gOdo.weg10Wheels(&wvl, &wvr, &whl, &whr);
    buildBremse2(b2, kmhAbsTx, sZeit, gOdo.weg11FrontAxle(), gOdo.impulszahlForBremse2());
    buildBremse10(b10, wvl, wvr, whl, whr);
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
    Serial.print(F("km/h filt/CANtx/odo: "));
    Serial.print(kmhFilt, 1);
    Serial.print('/');
    Serial.print(kmhAbsTx, 1);
    Serial.print('/');
    Serial.print(kmhOdo, 1);
    Serial.print(F(" | impulsos: "));
    Serial.print(gOdo.totalImpulsesForLogging());
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
