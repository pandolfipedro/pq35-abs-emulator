#include <Arduino.h>
#include <esp_timer.h>
#include <SPI.h>
#include <cmath>
#include <cstring>
#include <ACAN2515.h>

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
   uint16_t obdFunctional = 0x7DF;
   uint16_t obdEngineReq = 0x7E0;
   uint16_t obdEngineRsp = 0x7E8;
   uint16_t obdTransReq = 0x7E1;
   uint16_t obdTransRsp = 0x7E9;
   int32_t maxScheduleSlipUs = 3500;
   bool oemLikeBremse1 = true;
   bool panelSpeedCacheEnable = true;
   float panelCacheRejectBelowKmh = 0.5f;
   bool useMotionCanFusion = true;
   bool motionZeroInPark = true;
   uint32_t motionCanTimeoutMs = 500;
   float motionRpmMinDriving = 450.0f;
   uint8_t gearWahlPark = 8;
   uint8_t gearWahlReverse = 7;
   uint8_t gearWahlNeutral = 6;
   uint8_t gearWahlDrive = 5;
   bool motionAntiDropEnable = true;
   float motionAntiDropMaxKmhPerS = 90.0f;
   bool emitCompanionEspFrames = true;
   bool emitBremse4HaldexFrame = false;
} kConfig;

static inline float effectiveImpulsesPerKm() {
  const float scale = (kConfig.odoImpulsesPerKmScale > 0.00001f) ? kConfig.odoImpulsesPerKmScale : 1.0f;
  if (kConfig.odoDeriveImpulsesPerKm && kConfig.wheelCircumferenceM > 0.001f) {
    return (kConfig.absDistanceImpulsesPerWheelRev * 1000.0f / kConfig.wheelCircumferenceM) * scale;
  }
  return kConfig.impulsesPerKmCoding * scale;
}

static constexpr uint16_t kIdBremse1 = 0x1A0;
static constexpr uint16_t kIdBremse3 = 0x4A0;
static constexpr uint16_t kIdBremse2 = 0x5A0;
static constexpr uint16_t kIdBremse10 = 0x3A0;
static constexpr uint16_t kIdBremse5 = 0x4A8;
static constexpr uint16_t kIdBremse6 = 0x1A8;
static constexpr uint16_t kIdBremse4 = 0x2A0;
static constexpr uint16_t kIdBremse8 = 0x1AC;
static constexpr uint16_t kIdBremse11 = 0x5B7;
static constexpr uint16_t kIdMotorBremse = 0x284;
static constexpr uint16_t kIdMotor1 = 0x280;
static constexpr uint16_t kIdGetriebe1 = 0x440;
static constexpr uint16_t kIdGetriebe2 = 0x540;

 static ACAN2515 gCan(kConfig.mcpCs, SPI, kConfig.mcpInt);

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

struct VehicleMotionFromCan {
  float motorRpm = 0.0f;
  uint32_t lastMotor1Ms = 0;
  uint8_t wahlPos = 0xFF;
  uint32_t lastGearMs = 0;
  uint8_t ganganzeige = 0xFF;
  uint8_t engagedGear = 0xFF;
  uint32_t lastGear2Ms = 0;

  void onMotor1(const CANMessage &m, uint32_t nowMs) {
    if (m.rtr || m.len < 4)
      return;
    const uint32_t raw = readBitsLE(m.data, 16, 16);
    motorRpm = float(raw) * 0.25f;
    lastMotor1Ms = nowMs;
  }

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

  bool motorFresh(uint32_t nowMs) const {
    return lastMotor1Ms != 0 && (nowMs - lastMotor1Ms) < kConfig.motionCanTimeoutMs;
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
  bool drivingLoadLikely() const { return motorRpm >= kConfig.motionRpmMinDriving; }
  bool hasEngagedForwardGear() const { return engagedGear >= 1u && engagedGear <= 6u; }

  bool motionDrivingOem(uint32_t nowMs) const {
    if (!motorFresh(nowMs) || !drivingLoadLikely())
      return false;
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

     if ((s % 32u) == 0u) {
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
       if (id == kConfig.obdEngineRsp || id == kConfig.obdTransRsp) {
         uint8_t v = 0;
         if (extractMode01Pid0d(m, &v))
           gSpeed.onObdSpeed(v, nowMs);
       } else if (id == kIdMotor1) {
         gVehicleMotion.onMotor1(m, nowMs);
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
 static uint8_t gMobZaehler = 0;
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

 static void buildBremse5Idle(uint8_t *d) {
   memset(d, 0, 8);
   writeBitsLE(d, 52, 4, uint32_t(gBr5Zaehler & 0x0Fu));
   uint8_t x = 0;
   for (int i = 0; i < 7; ++i)
     x ^= d[i];
   d[7] = x;
   gBr5Zaehler = uint8_t((gBr5Zaehler + 1u) & 0x0Fu);
 }

 static void buildBremse8Idle(uint8_t *d) {
   memset(d, 0, 8);
   writeBitsLE(d, 8, 4, uint32_t(gBr8Zaehler & 0x0Fu));
   uint8_t x = 0;
   for (int i = 1; i < 8; ++i)
     x ^= d[i];
   d[0] = x;
   gBr8Zaehler = uint8_t((gBr8Zaehler + 1u) & 0x0Fu);
 }

 static void buildMotorBremseIdle(uint8_t *d) {
   memset(d, 0, 6);
   writeBitsLE(d, 8, 4, uint32_t(gMobZaehler & 0x0Fu));
   uint8_t x = 0;
   for (int i = 1; i < 6; ++i)
     x ^= d[i];
   d[0] = x;
   gMobZaehler = uint8_t((gMobZaehler + 1u) & 0x0Fu);
 }

 static void buildBremse6Idle(uint8_t *d) {
   memset(d, 0, 3);
   writeBitsLE(d, 0, 10, 123u);
   writeBitsLE(d, 12, 4, uint32_t(gBr6Zaehler & 0x0Fu));
   d[2] = uint8_t(d[0] ^ d[1]);
   gBr6Zaehler = uint8_t((gBr6Zaehler + 1u) & 0x0Fu);
 }

 static void buildBremse11Idle(uint8_t *d) {
   memset(d, 0, 8);
   writeBitsLE(d, 8, 4, uint32_t(gBr11Zaehler & 0x0Fu));
   uint8_t x = 0;
   for (int i = 1; i < 8; ++i)
     x ^= d[i];
   d[0] = x;
   gBr11Zaehler = uint8_t((gBr11Zaehler + 1u) & 0x0Fu);
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

 void setup() {
   Serial.begin(115200);
   delay(200);
   Serial.println(F("MK60 PQ35 ABS — OBD 0x0D + Bremse 1/2/3/10 + opt. ESP sat. | RX 280/440/540"));

   SPI.begin(18, 19, 23);

   ACAN2515Settings settings(kConfig.quartzHz, kConfig.canBitrate);
   settings.mRequestedMode = ACAN2515Settings::NormalMode;

   const ACAN2515Mask rxm = standard2515Mask(0x7FF, 0x00, 0x00);
   const ACAN2515AcceptanceFilter filters[] = {
       {standard2515Filter(kConfig.obdEngineRsp, 0x00, 0x00), nullptr},
       {standard2515Filter(kConfig.obdTransRsp, 0x00, 0x00), nullptr},
       {standard2515Filter(kIdMotor1, 0x00, 0x00), nullptr},
      {standard2515Filter(kIdGetriebe1, 0x00, 0x00), nullptr},
      {standard2515Filter(kIdGetriebe2, 0x00, 0x00), nullptr},
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

   static uint32_t lastLoopStartUs = 0;
   static uint32_t loopDtMaxUsThisSec = 0;
   static uint32_t slowLoopOver12ms = 0;
   const uint32_t loopStartUs = micros();
   if (lastLoopStartUs != 0) {
     const uint32_t dtUs = loopStartUs - lastLoopStartUs;
     if (dtUs > loopDtMaxUsThisSec)
       loopDtMaxUsThisSec = dtUs;
     if (dtUs > 12000u)
       slowLoopOver12ms++;
   }
   lastLoopStartUs = loopStartUs;

   static uint32_t lastLoopMs = ms;
   const float dtSec = fmaxf((ms - lastLoopMs) / 1000.0f, 0.0005f);
   lastLoopMs = ms;

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

   const bool tick10ms = t10.poll();
   if (tick10ms)
     gPanelCache.addSample(kmhFilt, ms);

  const float kmhPanelRaw = gPanelCache.outputForCan(kmhFilt, ms);
  const float kmhPanel = oemAntiDropPanelKmh(kmhPanelRaw, dtSec, ms);
  const float kmhCan = speedForAbsCan(kmhPanel);
   const float kmhAbsTx = slewLimitAbsCan(kmhCan, dtSec, gSpeed.haveObdEver(), gSpeed.zeroRunCount(),
                                          gSpeed.heldRawKmh(), kmhPanel);

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
     buildBremse1(b1, kmhAbsTx);
     buildBremse3(b3, kmhAbsTx);
     if (!sendFrame(kIdBremse1, b1))
       txFail1A0++;
     if (!sendFrame(kIdBremse3, b3))
       txFail4A0++;
     gBr1Zaehler = uint8_t((gBr1Zaehler + 1u) & 0x0Fu);
   }

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

    if (kConfig.emitCompanionEspFrames) {
      uint8_t b5[8], b8[8], mob[6], b6[3], b11[8];
      buildBremse5Idle(b5);
      buildBremse8Idle(b8);
      buildMotorBremseIdle(mob);
      buildBremse6Idle(b6);
      buildBremse11Idle(b11);
      if (!sendFrame(kIdBremse5, b5))
        txFailEsp++;
      if (!sendFrame(kIdBremse8, b8))
        txFailEsp++;
      if (!sendFrameDlc(kIdMotorBremse, mob, 6))
        txFailEsp++;
      if (!sendFrameDlc(kIdBremse6, b6, 3))
        txFailEsp++;
      if (!sendFrame(kIdBremse11, b11))
        txFailEsp++;
      if (kConfig.emitBremse4HaldexFrame) {
        const uint8_t b4[3] = {0, 0, 0};
        if (!sendFrameDlc(kIdBremse4, b4, 3))
          txFailEsp++;
      }
    }
  }

   static uint32_t lastLog = 0;
   if (ms - lastLog > 1000) {
     lastLog = ms;
     Serial.print(F("loop dt max(ms)/>12ms: "));
     Serial.print(loopDtMaxUsThisSec / 1000.0f, 2);
     Serial.print('/');
     Serial.print(slowLoopOver12ms);
     Serial.print(F(" | "));
     loopDtMaxUsThisSec = 0;
     slowLoopOver12ms = 0;
    Serial.print(F("km/h filt/panel0/panel/CANtx/odo: "));
    Serial.print(kmhFilt, 1);
    Serial.print('/');
    Serial.print(kmhPanelRaw, 1);
    Serial.print('/');
    Serial.print(kmhPanel, 1);
     Serial.print('/');
     Serial.print(kmhAbsTx, 1);
     Serial.print('/');
     Serial.print(kmhOdo, 1);
     Serial.print(F(" | impulsos: "));
     Serial.print(gOdo.totalImpulsesForLogging());
     Serial.print(F(" | imp/km: "));
     Serial.print(effectiveImpulsesPerKm(), 1);
     Serial.print(F(" | RX buf: "));
     Serial.println(gCan.receiveBufferCount());
    Serial.print(F("Motion RPM/wahl/eg: "));
    Serial.print(gVehicleMotion.motorRpm, 0);
    Serial.print('/');
    Serial.print(gVehicleMotion.wahlPos, DEC);
    Serial.print('/');
    Serial.print(gVehicleMotion.engagedGear, DEC);
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
   }
 }
