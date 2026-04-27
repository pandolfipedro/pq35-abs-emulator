
//  +------------------+----------+---------+---------------------------------+
//  | MCP2515          |  GPIO    |  Sinal  |  Observacao                     |
//  +------------------+----------+---------+---------------------------------+
//  |  VCC             |  +5V     |  ---    |  modulo tem regulador 3V3 onbd  |
//  |  GND             |  GND     |  ---    |                                 |
//  |  CS              |  GPIO 5  |  SPI_SS |  kConfig.mcpCs                  |
//  |  SCK             |  GPIO 18 |  SPI_CK |  HSPI/VSPI default do ESP32     |
//  |  MOSI / SI       |  GPIO 23 |  SPI_MO |  HSPI/VSPI default do ESP32     |
//  |  MISO / SO       |  GPIO 19 |  SPI_MI |  HSPI/VSPI default do ESP32     |
//  |  INT             |  GPIO 2  |  IRQ    |  kConfig.mcpInt (RX-IRQ ativo)  |
//  +------------------+----------+---------+---------------------------------+

#include <Arduino.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_idf_version.h>
#include <SPI.h>
#include <cmath>
#include <cstring>
#include <ACAN2515.h>


static constexpr uint32_t kWdtTimeoutMs = 2000;

struct Config {
   uint8_t mcpCs = 5;
   uint8_t mcpInt = 2;
   uint32_t quartzHz = 8000000UL;
   uint32_t canBitrate = 500000UL;
   float wheelCircumferenceM = 1.985f;
   bool odoDeriveImpulsesPerKm = true;
   float absDistanceImpulsesPerWheelRev = 43.59f;
   float impulsesPerKmCoding = 21960.0f;
   float odoImpulsesPerKmScale = 0.2581f;
   float standstillDeadBandKmh = 2.0f;
   uint32_t obdRequestPeriodMs = 80;
   uint32_t obdTimeoutMs = 1500;
   uint32_t obdHoldMaxMs = 25000;
   float speedTauMs = 180.0f;
   float speedMaxDecayKmhPerS = 12.0f;
   float absCanSlewUpKmhPerS = 220.0f;
   float absCanSlewDownKmhPerS = 65.0f;
   uint16_t obdTransReq = 0x7E1;
   uint16_t obdTransRsp = 0x7E9;
   int32_t maxScheduleSlipUs = 3500;
   bool oemLikeBremse1 = true;
   bool panelSpeedCacheEnable = true;
   float panelCacheRejectBelowKmh = 0.5f;
   bool useMotionCanFusion = true;
   bool motionZeroInPark = true;
   uint32_t motionCanTimeoutMs = 500;
   uint8_t gearWahlPark = 8;
   uint8_t gearWahlReverse = 7;
   uint8_t gearWahlNeutral = 6;
   uint8_t gearWahlDrive = 5;
   bool motionAntiDropEnable = true;
   float motionAntiDropMaxKmhPerS = 90.0f;
   bool emitCompanionEspFrames = true;
   bool emitBremse4HaldexFrame = false;
   // Calibracao do painel: a 100 km/h o cluster mostrava ~95, queremos ~103
   // (overshoot OEM). Fator = 103/95 ~= 1.084. Aplicado SO em Bremse_1/3.
   float speedPanelScaleFactor = 1.084f;
} kConfig;

static inline float effectiveImpulsesPerKm() {
  const float scale = (kConfig.odoImpulsesPerKmScale > 0.00001f) ? kConfig.odoImpulsesPerKmScale : 1.0f;
  if (kConfig.odoDeriveImpulsesPerKm && kConfig.wheelCircumferenceM > 0.001f) {
    return (kConfig.absDistanceImpulsesPerWheelRev * 1000.0f / kConfig.wheelCircumferenceM) * scale;
  }
  return kConfig.impulsesPerKmCoding * scale;
}

static constexpr uint16_t kIdBremse1   = 0x1A0;
static constexpr uint16_t kIdBremse3   = 0x4A0;
static constexpr uint16_t kIdBremse2   = 0x5A0;
static constexpr uint16_t kIdBremse10  = 0x3A0;
static constexpr uint16_t kIdBremse5   = 0x4A8;
static constexpr uint16_t kIdBremse6   = 0x1A8;
static constexpr uint16_t kIdBremse4   = 0x2A0;
static constexpr uint16_t kIdBremse8   = 0x1AC;
static constexpr uint16_t kIdBremse11  = 0x5B7;
static constexpr uint16_t kIdGetriebe1 = 0x440;
static constexpr uint16_t kIdGetriebe2 = 0x540;

 static ACAN2515 gCan(kConfig.mcpCs, SPI, kConfig.mcpInt);

 static inline uint32_t nowMillis() {
   return (uint32_t)(esp_timer_get_time() / 1000ULL);
 }

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

 static inline uint32_t readBitsLE(const uint8_t *data, unsigned startBit, unsigned bitLen) {
   uint32_t v = 0;
   for (unsigned i = 0; i < bitLen; ++i) {
     const unsigned bit = startBit + i;
     const unsigned byte = bit / 8u;
     if (data[byte] & (1u << (bit % 8u)))
       v |= (1u << i);
   }
   return v;
 }

 static inline uint16_t speedToRaw100(float kmh) {
   if (kmh < 0.0f)
     kmh = 0.0f;

   const float x = kmh * 100.0f;
   if (x > 32766.0f)
     return 32766;
   return uint16_t(x + 0.5f);
 }

 static inline uint16_t midRevsToRaw(float revPerSec) {
   if (revPerSec < 0.0f)
     revPerSec = 0.0f;
   float r = revPerSec / 0.002f;

   if (r < 1.0f)
     r = 1.0f;
   if (r >= 65535.0f)
     return 65535;
   return uint16_t(r + 0.5f);
 }

 static inline float kmhToMidRevs(float kmh) {
   const float v = kmh / 3.6f;
   const float circ = kConfig.wheelCircumferenceM;
   if (circ <= 0.001f)
     return 0.0f;
   return v / circ;
 }

 static inline float effectiveSpeedKmhForOdo(float filteredKmh) {
   const float a = fabsf(filteredKmh);
   if (a < kConfig.standstillDeadBandKmh)
     return 0.0f;
   return filteredKmh;
 }

 static inline float speedForAbsCan(float filteredKmh) {
   if (!isfinite(filteredKmh) || filteredKmh < 0.0f)
     return 0.0f;
   return filteredKmh;
 }

 // Agendador rate-monotonic em microssegundos.
 //
 // Doutrina: a proxima "deadline" e calculada a partir da deadline anterior
 // (nextUs += periodUs), NAO a partir do "agora". Assim, se a iteracao
 // atrasou X us, a proxima espera fica X us mais curta -> a cadencia media
 // do periodo permanece exata, sem drift. Se o atraso passar de
 // kConfig.maxScheduleSlipUs (sistema travou ou houve um burst), reagenda
 // a partir de "agora" para evitar disparar varios ticks atrasados de uma
 // vez (anti burst-catch).
 //
 // poll(usNow): aceita opcionalmente o "agora" da iteracao para reusar
 // uma unica leitura de esp_timer_get_time() em todo o loop principal,
 // garantindo que tarefas concorrentes ancoradas no mesmo instante
 // disparem coerentemente.
 struct PeriodicDeadline {
   int64_t nextUs = 0;
   const int32_t periodUs;

   explicit PeriodicDeadline(int32_t pUs) : periodUs(pUs) {}

   void startNow() { nextUs = esp_timer_get_time() + periodUs; }

   int64_t nextDeadlineUs() const { return nextUs; }

   void syncAfter(const PeriodicDeadline &other, int32_t offsetUs) {
     nextUs = other.nextDeadlineUs() + offsetUs;
   }

   bool poll(int64_t usNow = -1) {
     const int64_t now = (usNow >= 0) ? usNow : esp_timer_get_time();
     if (now < nextUs)
       return false;
     const int64_t slip = now - nextUs;
     if (slip > kConfig.maxScheduleSlipUs) {
       nextUs = now + periodUs;  // anti burst-catch
     } else {
       nextUs += periodUs;       // rate-monotonic puro
     }
     return true;
   }
 };

 // ============================================================
 // SpeedModel: pipeline de filtro IIR + tolerancia a zeros do OBD.
 // Inalterado em relacao a versao funcional 100%.
 // ============================================================
 struct SpeedModel {
   float filteredKmh = 0.0f;
   float heldKmh = 0.0f;
   uint32_t lastGoodMs = 0;
   uint32_t lastAnyObdMs = 0;
   uint32_t lastPositiveKmhMs = 0;
   bool haveEver = false;
   uint8_t zeroRun = 0;

   void onObdSpeed(uint8_t rawKmh, uint32_t nowMs) {
     uint32_t dtMs = nowMs - lastAnyObdMs;
     if (dtMs == 0) dtMs = 1;
     if (dtMs > 1000) dtMs = 1000;
     heldKmh = float(rawKmh);
     lastGoodMs = nowMs;
     lastAnyObdMs = nowMs;
     haveEver = true;
     if (!isfinite(filteredKmh))
       filteredKmh = 0.0f;
     if (rawKmh > 0)
       lastPositiveKmhMs = nowMs;

     if (rawKmh == 0 && filteredKmh > 12.0f && lastPositiveKmhMs != 0 &&
         (nowMs - lastPositiveKmhMs) < 120u) {
       return;
     }
     if (rawKmh == 0)
       zeroRun = (zeroRun < 255) ? uint8_t(zeroRun + 1u) : 255;
     else
       zeroRun = 0;

     const float alpha = (kConfig.speedTauMs <= 1.0f)
                             ? 1.0f
                             : (1.0f - expf(-float(dtMs) / kConfig.speedTauMs));

     const unsigned maxZeroIgnore =
         (filteredKmh > 45.0f) ? 30u : ((filteredKmh > 40.0f) ? 25u : 10u);
     if (rawKmh == 0 && filteredKmh > 3.0f && zeroRun < maxZeroIgnore) {
       return;
     }
     if (rawKmh == 0 && filteredKmh > 3.0f) {
       const float alphaZero = fminf(alpha, 0.25f);
       filteredKmh += (0.0f - filteredKmh) * alphaZero;
       return;
     }
     filteredKmh += (heldKmh - filteredKmh) * alpha;
   }

   void tick(uint32_t nowMs, float dtSec) {
     const uint32_t dtMs = nowMs - lastAnyObdMs;
     if (dtMs > kConfig.obdTimeoutMs) {
       if (haveEver && dtMs < kConfig.obdHoldMaxMs) {
         // hold: mantem filteredKmh
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

 // ============================================================
 // VehicleMotionFromCan: agora le APENAS Getriebe1/2 (cambio).
 // Motor1 (0x280, RPM) foi removido a pedido. motionDrivingOem
 // baseia-se SOMENTE em wahlPos / engagedGear.
 // ============================================================
struct VehicleMotionFromCan {
  uint8_t wahlPos = 0xFF;
  uint32_t lastGearMs = 0;
  uint8_t ganganzeige = 0xFF;
  uint8_t engagedGear = 0xFF;
  uint32_t lastGear2Ms = 0;

  void onGetriebe1(const CANMessage &m, uint32_t nowMs) {
    if (m.rtr || m.len < 2)
      return;
    wahlPos = uint8_t(readBitsLE(m.data, 12, 4) & 0x0Fu);
    lastGearMs = nowMs;
  }

  void onGetriebe2(const CANMessage &m, uint32_t nowMs) {
    if (m.rtr || m.len < 8)
      return;
    ganganzeige = uint8_t(readBitsLE(m.data, 56, 4) & 0x0Fu);
    engagedGear = uint8_t(readBitsLE(m.data, 60, 4) & 0x0Fu);
    lastGear2Ms = nowMs;
  }

  bool gearFresh(uint32_t nowMs) const {
    return lastGearMs != 0 && (nowMs - lastGearMs) < kConfig.motionCanTimeoutMs;
  }
  bool gear2Fresh(uint32_t nowMs) const {
    return lastGear2Ms != 0 && (nowMs - lastGear2Ms) < kConfig.motionCanTimeoutMs;
  }
  bool isPark() const { return wahlPos == kConfig.gearWahlPark; }
  bool isDriveOrReverse() const {
    return wahlPos == kConfig.gearWahlDrive || wahlPos == kConfig.gearWahlReverse;
  }
  bool hasEngagedForwardGear() const { return engagedGear >= 1u && engagedGear <= 6u; }

  // Sem RPM, "movimento OEM" passa a ser: marcha em D/R ou marcha
  // engatada (1-6) reportada pelo Getriebe2. Em Park retorna false.
  bool motionDrivingOem(uint32_t nowMs) const {
    if (gearFresh(nowMs) && isPark())
      return false;
    if (gearFresh(nowMs) && isDriveOrReverse())
      return true;
    if (gear2Fresh(nowMs) && hasEngagedForwardGear())
      return true;
    return false;
  }
};

static VehicleMotionFromCan gVehicleMotion;

static float oemAntiDropPanelKmh(float panelIn, float dtSec, uint32_t nowMs) {
  static float sLast = -1.0f;
  if (!kConfig.motionAntiDropEnable || !kConfig.useMotionCanFusion)
    return panelIn;
  if (sLast < 0.0f) {
    sLast = panelIn;
    return panelIn;
  }
  if (!gVehicleMotion.motionDrivingOem(nowMs)) {
    sLast = panelIn;
    return panelIn;
  }
  const float maxStepDown = kConfig.motionAntiDropMaxKmhPerS * fmaxf(dtSec, 0.002f);
  float out = panelIn;
  if (panelIn < sLast - maxStepDown)
    out = sLast - maxStepDown;
  if (panelIn > sLast)
    out = panelIn;
  sLast = out;
  return out;
}

 // ============================================================
 // SpeedPanelCache: janela de 5 amostras com rejeicao de outliers.
 // Inalterado em relacao a versao funcional 100%.
 // ============================================================
 struct SpeedPanelCache {
   static constexpr unsigned kN = 5;
   float window[kN] = {};
   unsigned filled = 0;
   float lastAvg = 0.0f;
   bool haveAvg = false;

   void reset() {
     filled = 0;
     lastAvg = 0.0f;
     haveAvg = false;
   }

   static float windowMax(const float *w, unsigned n) {
     float m = 0.0f;
     for (unsigned i = 0; i < n; ++i)
       if (w[i] > m)
         m = w[i];
     return m;
   }

   void finishWindow(uint32_t nowMs) {
     if (kConfig.useMotionCanFusion && kConfig.motionZeroInPark && gVehicleMotion.gearFresh(nowMs) &&
         gVehicleMotion.isPark()) {
       lastAvg = 0.0f;
       haveAvg = true;
       filled = 0;
       return;
     }

     const float mx = windowMax(window, kN);
     float sum = 0.0f;
     unsigned cnt = 0;
     for (unsigned i = 0; i < kN; ++i) {
       const float x = window[i];
       if (x <= 0.0001f)
         continue;
       if (mx > 3.0f && x < kConfig.panelCacheRejectBelowKmh)
         continue;
       sum += x;
       cnt++;
     }

     if (cnt > 0) {
       lastAvg = sum / float(cnt);
     } else {
       if (kConfig.useMotionCanFusion && gVehicleMotion.motionDrivingOem(nowMs) && haveAvg) {
         // mantem lastAvg
       } else {
         lastAvg = 0.0f;
       }
     }
     haveAvg = true;
     filled = 0;
   }

   void addSample(float filteredKmh, uint32_t nowMs) {
     if (!kConfig.panelSpeedCacheEnable)
       return;
     if (filled >= kN)
       return;
     window[filled++] = filteredKmh;
     if (filled == kN)
       finishWindow(nowMs);
   }

   float outputForCan(float filteredKmh, uint32_t nowMs) const {
     if (kConfig.useMotionCanFusion && kConfig.motionZeroInPark && gVehicleMotion.gearFresh(nowMs) &&
         gVehicleMotion.isPark())
       return 0.0f;
     if (!kConfig.panelSpeedCacheEnable)
       return filteredKmh;
     if (!haveAvg)
       return filteredKmh;
     return lastAvg;
   }
 };

 static SpeedPanelCache gPanelCache;

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

 static inline void splitVehicleImpToWheels(uint32_t v, uint16_t *vl, uint16_t *vr, uint16_t *hl, uint16_t *hr) {
   const uint32_t q = v / 4u;
   const uint32_t r = v % 4u;
   *vl = uint16_t((q + (r > 0u ? 1u : 0u)) & 0x3FFu);
   *vr = uint16_t((q + (r > 1u ? 1u : 0u)) & 0x3FFu);
   *hl = uint16_t((q + (r > 2u ? 1u : 0u)) & 0x3FFu);
   *hr = uint16_t((q + (r > 3u ? 1u : 0u)) & 0x3FFu);
 }

 // ============================================================
 // Odometer: contagem de pulsos. Inalterado.
 // ============================================================
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
     const double impPerM = (double)effectiveImpulsesPerKm() / 1000.0;
     const double vMs = (double)kmh / 3.6;
     impulseFrac += vMs * dtSec * impPerM;
     if (impulseFrac >= 1.0) {
       const uint32_t add = (uint32_t)floor(impulseFrac);
       vehImpulses += add;
       impulseFrac -= (double)add;
     }
   }

   uint16_t weg11FrontAxle() const { return uint16_t(vehImpulses & 0x7FFu); }

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

 static bool extractMode01Pid0d(const CANMessage &m, uint8_t *outKmh) {
   if (m.rtr || m.len < 3)
     return false;
   if (m.len >= 2 && m.data[0] == 0x03u && m.data[1] == 0x7Fu)
     return false;
   for (unsigned i = 0; i + 2u < (unsigned)m.len && i < 7u; ++i) {
     if (m.data[i] == 0x41u && m.data[i + 1] == 0x0Du) {
       *outKmh = m.data[i + 2];
       return true;
     }
   }
   return false;
 }

 // ============================================================
 // ObdClient
 //   - Pergunta SO para a transmissao (0x7E1) e aceita SO 0x7E9.
 //   - Auto-mute em Park: nada na rede CAN com o carro parado, libera
 //     o canal de diagnostico para o VCDS conversar com o cambio.
 // ============================================================
 struct ObdClient {
   uint32_t lastTxMs = 0;

   void setup() { lastTxMs = 0; }

   void trySendRequests(uint32_t nowMs) {
     if (nowMs - lastTxMs < kConfig.obdRequestPeriodMs)
       return;

     // Auto-mute em Park: se o cambio reporta P, nao envia OBD.
     if (kConfig.useMotionCanFusion &&
         gVehicleMotion.gearFresh(nowMs) && gVehicleMotion.isPark())
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
     m.id = kConfig.obdTransReq;
     (void)gCan.tryToSend(m);
   }

   void pollReceive(uint32_t nowMs) {
     CANMessage m;
     while (gCan.receive(m)) {
       const uint16_t id = m.id;
       if (id == kConfig.obdTransRsp) {
         uint8_t v = 0;
         if (extractMode01Pid0d(m, &v))
           gSpeed.onObdSpeed(v, nowMs);
       } else if (id == kIdGetriebe1) {
         gVehicleMotion.onGetriebe1(m, nowMs);
       } else if (id == kIdGetriebe2) {
         gVehicleMotion.onGetriebe2(m, nowMs);
       }
     }
   }
 };

 static ObdClient gObd;

 static uint8_t gBr1Zaehler = 0;
 static uint8_t gB10Zaehler = 0;
 static uint8_t gBr5Zaehler = 0;
 static uint8_t gBr8Zaehler = 0;
 static uint8_t gBr11Zaehler = 0;
 static uint8_t gBr6Zaehler = 0;

 static void buildBremse1(uint8_t *d, float kmh) {
   memset(d, 0, 8);

   if (kmh < 0.0f)
     kmh = 0.0f;
   const float rawF = kmh * 100.0f;
   const uint32_t raw = (rawF > 32767.0f) ? 32767u : uint32_t(rawF + 0.5f);
   writeBitsLE(d, 17, 15, raw);
   if (kConfig.oemLikeBremse1) {
     writeBitsLE(d, 32, 8, 0xFE);
     writeBitsLE(d, 40, 8, 0xFE);
     writeBitsLE(d, 48, 8, 0xFE);
   }
   writeBitsLE(d, 56, 4, gBr1Zaehler & 0x0Fu);
   writeBitsLE(d, 63, 1, 0);
 }

 static void buildBremse3(uint8_t *d, float kmh) {
   memset(d, 0, 8);

   if (kmh < 0.0f)
     kmh = 0.0f;
   const uint32_t raw = speedToRaw100(kmh);
   writeBitsLE(d, 0, 1, 0);
   writeBitsLE(d, 1, 15, raw);
   writeBitsLE(d, 16, 1, 0);
   writeBitsLE(d, 17, 15, raw);
   writeBitsLE(d, 32, 1, 0);
   writeBitsLE(d, 33, 15, raw);
   writeBitsLE(d, 48, 1, 0);
   writeBitsLE(d, 49, 15, raw);
 }

 static void buildBremse2(uint8_t *d, float kmh, uint16_t zeitTicks, uint16_t weg11, uint8_t imp6) {
   memset(d, 0, 8);

   writeBitsLE(d, 8, 1, 1);
   d[0] = 250;

   const float mid = kmhToMidRevs(kmh);
   const uint16_t rawMid = uint16_t(midRevsToRaw(mid) & 0x7FFFu);
   writeBitsLE(d, 9, 15, rawMid);

   writeBitsLE(d, 24, 16, zeitTicks);
   writeBitsLE(d, 40, 11, uint32_t(weg11 & 0x7FFu));
   writeBitsLE(d, 56, 6, uint32_t(imp6 & 0x3Fu));
 }

 static void buildBremse10(uint8_t *d, uint16_t weg10VL, uint16_t weg10VR, uint16_t weg10HL, uint16_t weg10HR) {
   memset(d, 0, 8);

   writeBitsLE(d, 8, 4, gB10Zaehler & 0x0Fu);

   writeBitsLE(d, 12, 1, 1);
   writeBitsLE(d, 13, 1, 1);
   writeBitsLE(d, 14, 1, 1);
   writeBitsLE(d, 15, 1, 1);
   writeBitsLE(d, 16, 10, uint32_t(weg10VL & 0x3FFu));
   writeBitsLE(d, 26, 10, uint32_t(weg10VR & 0x3FFu));
   writeBitsLE(d, 36, 10, uint32_t(weg10HL & 0x3FFu));
   writeBitsLE(d, 46, 10, uint32_t(weg10HR & 0x3FFu));

   writeBitsLE(d, 56, 1, 1);
   writeBitsLE(d, 57, 1, 1);
   writeBitsLE(d, 58, 1, 1);
   writeBitsLE(d, 59, 1, 1);
   writeBitsLE(d, 60, 1, 0);
   writeBitsLE(d, 61, 1, 0);
   writeBitsLE(d, 62, 1, 0);
   writeBitsLE(d, 63, 1, 0);

   uint8_t x = 0;
   for (int i = 1; i < 8; ++i)
     x ^= d[i];
   d[0] = x;
 }

 // Builders dos companion frames: APENAS montam o frame. O incremento do
 // Zaehler e feito no caller, somente quando tryToSend() retornar true.
 // Razao: gateway PQ35 detecta "frame perdido" quando ve o counter pular.
 // Se a TX falhar, o frame nao foi pro bus, entao reusar o mesmo counter
 // na proxima tentativa e o comportamento correto.

 // Bremse_5 idle. Se stillstand=true, sinaliza BR5_Stillstand (bit 28) =1
 // -> Autohold/HSA/EPB reconhecem corretamente o estado de parada.
 static void buildBremse5Idle(uint8_t *d, bool stillstand) {
   memset(d, 0, 8);
   if (stillstand) writeBitsLE(d, 28, 1, 1);
   writeBitsLE(d, 52, 4, uint32_t(gBr5Zaehler & 0x0Fu));
   uint8_t x = 0;
   for (int i = 0; i < 7; ++i)
     x ^= d[i];
   d[7] = x;
 }

 // Bremse_8 idle. Aceleracoes neutras (0 m/s^2):
 //   BR8_Istbeschl   (bit 32, 9, scale 0.02,    offset -7.22) -> raw 361
 //   BR8_Laengsbeschl(bit 48, 10, scale 0.03125, offset -16)  -> raw 512
 // Sem isso, raw 0 representa -7.22 / -16 m/s^2 (frenagem violenta) e
 // pode confundir o painel/ESP companion.
 static void buildBremse8Idle(uint8_t *d) {
   memset(d, 0, 8);
   writeBitsLE(d, 8, 4, uint32_t(gBr8Zaehler & 0x0Fu));
   writeBitsLE(d, 32, 9, 361u);
   writeBitsLE(d, 48, 10, 512u);
   uint8_t x = 0;
   for (int i = 1; i < 8; ++i)
     x ^= d[i];
   d[0] = x;
 }

 static void buildBremse6Idle(uint8_t *d) {
   memset(d, 0, 3);
   writeBitsLE(d, 0, 10, 123u);
   writeBitsLE(d, 12, 4, uint32_t(gBr6Zaehler & 0x0Fu));
   d[2] = uint8_t(d[0] ^ d[1]);
 }

 static void buildBremse11Idle(uint8_t *d) {
   memset(d, 0, 8);
   writeBitsLE(d, 8, 4, uint32_t(gBr11Zaehler & 0x0Fu));
   uint8_t x = 0;
   for (int i = 1; i < 8; ++i)
     x ^= d[i];
   d[0] = x;
 }

 // Bremse_4 idle (Haldex). ABS_Vorgabewert_mitte_Kupplungs (bit 0, 8,
 // scale 3, offset -381): raw 127 = 0 Nm/min (torque neutro). Outros
 // bytes ficam 0. DBC nao define counter pra este frame.
 static void buildBremse4Idle(uint8_t *d) {
   memset(d, 0, 3);
   d[0] = 127;
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

 static inline bool sendFrameDlc(uint16_t id, const uint8_t *data, uint8_t len) {
   CANMessage m;
   memset(&m, 0, sizeof(m));
   m.id = id;
   m.len = (len > 8u) ? 8u : len;
   m.ext = false;
   m.rtr = false;
   memcpy(m.data, data, m.len);
   return gCan.tryToSend(m);
 }

 static void IRAM_ATTR canISR() { gCan.isr(); }

 static const __FlashStringHelper* resetReasonStr(esp_reset_reason_t r) {
   switch (r) {
     case ESP_RST_POWERON:  return F("POWERON");
     case ESP_RST_EXT:      return F("EXT");
     case ESP_RST_SW:       return F("SW");
     case ESP_RST_PANIC:    return F("PANIC");
     case ESP_RST_INT_WDT:  return F("INT_WDT");
     case ESP_RST_TASK_WDT: return F("TASK_WDT");
     case ESP_RST_WDT:      return F("WDT");
     case ESP_RST_DEEPSLEEP:return F("DEEPSLEEP");
     case ESP_RST_BROWNOUT: return F("BROWNOUT");
     case ESP_RST_SDIO:     return F("SDIO");
     default:               return F("UNKNOWN");
   }
 }

 void setup() {
   Serial.begin(115200);
   delay(200);
   Serial.println(F("MK60 PQ35 ABS - cambio-only OBD | RX 440/540"));
   Serial.print(F("Reset reason: "));
   Serial.println(resetReasonStr(esp_reset_reason()));

   SPI.begin(18, 19, 23);

   ACAN2515Settings settings(kConfig.quartzHz, kConfig.canBitrate);
   settings.mRequestedMode = ACAN2515Settings::NormalMode;

   const ACAN2515Mask rxm = standard2515Mask(0x7FF, 0x00, 0x00);
   const ACAN2515AcceptanceFilter filters[] = {
       {standard2515Filter(kConfig.obdTransRsp, 0x00, 0x00), nullptr},
       {standard2515Filter(kIdGetriebe1, 0x00, 0x00), nullptr},
       {standard2515Filter(kIdGetriebe2, 0x00, 0x00), nullptr},
   };

   uint16_t err = 0;
   for (int attempt = 1; attempt <= 5; ++attempt) {
     err = gCan.begin(settings, canISR, rxm, rxm,
                      filters, uint8_t(sizeof(filters) / sizeof(filters[0])));
     if (err == 0) break;
     Serial.print(F("ACAN2515 begin tentativa "));
     Serial.print(attempt);
     Serial.print(F(" falhou: 0x"));
     Serial.println(err, HEX);
     delay(500);
   }
   if (err != 0) {
     Serial.println(F("[FATAL] MCP2515 nao inicializou. Reiniciando..."));
     Serial.flush();
     delay(200);
     esp_restart();
   }
   Serial.println(F("MCP2515 OK @ 500k"));

   gObd.setup();

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
   {
     const esp_task_wdt_config_t wdt_cfg = {
       .timeout_ms     = kWdtTimeoutMs,
       .idle_core_mask = 0,
       .trigger_panic  = true,
     };
     esp_task_wdt_reconfigure(&wdt_cfg);
   }
#else
   esp_task_wdt_init(kWdtTimeoutMs / 1000u, true);
#endif
   esp_task_wdt_add(NULL);
 }

 void loop() {
   // Relogio de microssegundos capturado UMA UNICA VEZ por iteracao.
   // Todos os subsistemas que precisam de "agora" derivam deste valor:
   //   - ms   = usNow / 1000    (modulos legados que esperam millis)
   //   - dtUs = usNow - last    (precisao sub-ms para o IIR)
   //   - PeriodicDeadline::poll(usNow) reusa este relogio.
   // Assim, todas as decisoes da iteracao sao coerentes com o mesmo
   // instante e nao ha drift entre tarefas.
   const int64_t usNow = esp_timer_get_time();
   const uint32_t ms   = (uint32_t)(usNow / 1000LL);

   // Telemetria de jitter: dt entre inicios de loop (em us, 64 bits para
   // nao haver wrap). Se passar de 12 ms, conta um "slow loop".
   static int64_t lastLoopUs = 0;
   static uint32_t loopDtMaxUsThisSec = 0;
   static uint32_t slowLoopOver12ms = 0;
   if (lastLoopUs != 0) {
     const int64_t dtUs64 = usNow - lastLoopUs;
     const uint32_t dtUs = (dtUs64 > 0 && dtUs64 < 0x7FFFFFFFLL) ? (uint32_t)dtUs64 : 0u;
     if (dtUs > loopDtMaxUsThisSec)
       loopDtMaxUsThisSec = dtUs;
     if (dtUs > 12000u)
       slowLoopOver12ms++;
   }

   // dtSec calculado em US para precisao sub-ms no filtro IIR de
   // velocidade. Antes era em ms inteiros, perdendo ate 999us de
   // resolucao por iteracao -> agora cada loop cycle conta exato.
   const int64_t dtUsForFilter = (lastLoopUs == 0) ? 1000LL : (usNow - lastLoopUs);
   lastLoopUs = usNow;
   const float dtSec = fmaxf((float)dtUsForFilter / 1.0e6f, 0.0005f);

   gObd.pollReceive(ms);
   gObd.trySendRequests(ms);
   gSpeed.tick(ms, dtSec);

   const float kmhFilt = gSpeed.displayKmh();

   static PeriodicDeadline t10(10000);
   static PeriodicDeadline t20(20000);
   static bool tInit = false;
   if (!tInit) {
     t10.startNow();
     t20.syncAfter(t10, 10000);
     tInit = true;
   }

   const bool tick10ms = t10.poll(usNow);
   if (tick10ms)
     gPanelCache.addSample(kmhFilt, ms);

   const float kmhPanelRaw = gPanelCache.outputForCan(kmhFilt, ms);
   const float kmhPanel    = oemAntiDropPanelKmh(kmhPanelRaw, dtSec, ms);
   const float kmhCan      = speedForAbsCan(kmhPanel);
   const float kmhAbsTx    = slewLimitAbsCan(kmhCan, dtSec, gSpeed.haveObdEver(),
                                             gSpeed.zeroRunCount(),
                                             gSpeed.heldRawKmh(), kmhPanel);

   // Velocidade ESCALADA (somente para frames lidos pelo painel/cluster).
   // Nao afeta o odometro: gOdo.integrate usa kmhAbsTx puro -> 100 km no
   // GPS = 100 km no hodometro, independente do fator do painel.
   const float kmhPanelTx = fmaxf(0.0f, kmhAbsTx * kConfig.speedPanelScaleFactor);

   const float kmhOdo = effectiveSpeedKmhForOdo(kmhAbsTx);

   const float dtOdo = fminf(dtSec, 0.08f);
   gOdo.integrate(kmhOdo, dtOdo);
   gObd.pollReceive(ms);

   static uint16_t sZeit = 0;

   uint8_t b1[8], b3[8], b2[8], b10[8];
   static uint32_t txFail1A0 = 0, txFail4A0 = 0, txFail5A0 = 0, txFail3A0 = 0;
   static uint32_t txFailEsp = 0;

   if (tick10ms) {
     sZeit = uint16_t(sZeit + 10);
     // Bremse_1 e Bremse_3 vao ESCALADOS: o painel mostrara ~103 quando
     // o GPS estiver em 100 (overshoot OEM). Bremse_2 (mid-rev, ABS
     // interno) usa kmhAbsTx puro.
     buildBremse1(b1, kmhPanelTx);
     buildBremse3(b3, kmhPanelTx);
     // Zaehler so incrementa se o frame foi efetivamente enfileirado
     // para TX. Se falhar, mesmo counter sera reusado no proximo ciclo.
     if (sendFrame(kIdBremse1, b1))
       gBr1Zaehler = uint8_t((gBr1Zaehler + 1u) & 0x0Fu);
     else
       txFail1A0++;
     if (!sendFrame(kIdBremse3, b3))  // Bremse_3 nao tem counter no DBC
       txFail4A0++;
   }

   if (t20.poll(usNow)) {
     uint16_t wvl, wvr, whl, whr;
     gOdo.weg10Wheels(&wvl, &wvr, &whl, &whr);
     buildBremse2(b2, kmhAbsTx, sZeit, gOdo.weg11FrontAxle(), gOdo.impulszahlForBremse2());
     buildBremse10(b10, wvl, wvr, whl, whr);
     if (!sendFrame(kIdBremse2, b2))  // Bremse_2 nao tem counter
       txFail5A0++;
     if (sendFrame(kIdBremse10, b10))
       gB10Zaehler = uint8_t((gB10Zaehler + 1u) & 0x0Fu);
     else
       txFail3A0++;

     if (kConfig.emitCompanionEspFrames) {
       uint8_t b5[8], b8[8], b6[3], b11[8];
       const bool stillstand = (kmhAbsTx < 1.0f);
       buildBremse5Idle(b5, stillstand);
       buildBremse8Idle(b8);
       buildBremse6Idle(b6);
       buildBremse11Idle(b11);
       if (sendFrame(kIdBremse5, b5))
         gBr5Zaehler = uint8_t((gBr5Zaehler + 1u) & 0x0Fu);
       else
         txFailEsp++;
       if (sendFrame(kIdBremse8, b8))
         gBr8Zaehler = uint8_t((gBr8Zaehler + 1u) & 0x0Fu);
       else
         txFailEsp++;
       if (sendFrameDlc(kIdBremse6, b6, 3))
         gBr6Zaehler = uint8_t((gBr6Zaehler + 1u) & 0x0Fu);
       else
         txFailEsp++;
       if (sendFrame(kIdBremse11, b11))
         gBr11Zaehler = uint8_t((gBr11Zaehler + 1u) & 0x0Fu);
       else
         txFailEsp++;
       if (kConfig.emitBremse4HaldexFrame) {
         uint8_t b4[3];
         buildBremse4Idle(b4);
         if (!sendFrameDlc(kIdBremse4, b4, 3))  // Bremse_4 nao tem counter
           txFailEsp++;
       }
     }
   }

   esp_task_wdt_reset();

   // Log 1Hz baseado em us (int64_t -> sem wrap). Robusto e coerente com
   // o relogio capturado no inicio do loop.
   static int64_t lastLogUs = 0;
   if (usNow - lastLogUs > 1000000LL) {
     lastLogUs = usNow;

     const uint32_t heapFree = esp_get_free_heap_size();
     const uint32_t heapMin  = esp_get_minimum_free_heap_size();
     const uint32_t stackWM  = uxTaskGetStackHighWaterMark(NULL);

     Serial.print(F("loop dt max(ms)/>12ms: "));
     Serial.print(loopDtMaxUsThisSec / 1000.0f, 2);
     Serial.print('/');
     Serial.print(slowLoopOver12ms);
     Serial.print(F(" | "));
     loopDtMaxUsThisSec = 0;
     slowLoopOver12ms = 0;
     Serial.print(F("km/h filt/panel0/panel/CANtx/panelTx/odo: "));
     Serial.print(kmhFilt, 1);
     Serial.print('/');
     Serial.print(kmhPanelRaw, 1);
     Serial.print('/');
     Serial.print(kmhPanel, 1);
     Serial.print('/');
     Serial.print(kmhAbsTx, 1);
     Serial.print('/');
     Serial.print(kmhPanelTx, 1);
     Serial.print('/');
     Serial.print(kmhOdo, 1);
     Serial.print(F(" | impulsos: "));
     Serial.print(gOdo.totalImpulsesForLogging());
     Serial.print(F(" | imp/km: "));
     Serial.print(effectiveImpulsesPerKm(), 1);
     Serial.print(F(" | RX buf: "));
     Serial.println(gCan.receiveBufferCount());
     Serial.print(F("Cambio wahl/eg: "));
     Serial.print(gVehicleMotion.wahlPos, DEC);
     Serial.print('/');
     Serial.print(gVehicleMotion.engagedGear, DEC);
     Serial.print(F(" | heap free/min: "));
     Serial.print(heapFree);
     Serial.print('/');
     Serial.print(heapMin);
     Serial.print(F(" | stack: "));
     Serial.print(stackWM);
     Serial.print(F(" | TX fail 1A0/4A0/5A0/3A0/esp: "));
     Serial.print(txFail1A0);
     Serial.print('/');
     Serial.print(txFail4A0);
     Serial.print('/');
     Serial.print(txFail5A0);
     Serial.print('/');
     Serial.print(txFail3A0);
     Serial.print('/');
     Serial.println(txFailEsp);

     if (stackWM < 512u)
       Serial.println(F("[WARN] Stack baixa!"));
   }
 }
