// ================================================================
//  PQ35 ABS Module Emulator
//  ESP32 DevKit V1 + MCP2515
//
//  Emulates the ABS module on Volkswagen PQ35 platform vehicles
//  (Jetta MK5, Golf MK5, Passat B6, Audi A3 8P, Seat Leon MK2)
//
//  How it works:
//    1. Listens to the powertrain CAN bus
//    2. Reads vehicle speed from frame 0x540 (Getriebe_2), sent by TCM
//    3. Re-broadcasts speed as the original ABS module would:
//       - 0x5A0  Bremse_2  (10ms) — speedometer + distance counter
//       - 0x1A0  Bremse_1  (10ms) — wheel speed for ECU and TCM
//       - 0x4A0  Bremse_3  (50ms) — ABS/ESP module keepalive
//
//  Wiring — MCP2515 to ESP32:
//    VCC   →  VIN   (5V from USB — do NOT use 3.3V pin)
//    GND   →  GND
//    CS    →  GPIO 5
//    SCK   →  GPIO 18
//    MOSI  →  GPIO 23
//    MISO  →  GPIO 19
//    INT   →  GPIO 4
//
//  Wiring — MCP2515 to vehicle:
//    CANH  →  Powertrain CAN High (orange/black wire — B383)
//    CANL  →  Powertrain CAN Low  (orange/brown wire  — B390)
//
//  Access point confirmed on Jetta 2.5 2009:
//    Gateway J533 connector, pin 16 (CANH) and pin 6 (CANL)
//
//  WARNING: Do NOT connect via OBD-II port.
//    The powertrain CAN bus is NOT routed to the OBD-II connector.
//    You must tap directly into the powertrain CAN wiring.
//
//  MCP2515 crystal:
//    8 MHz  board → MCP_8MHZ  (default below)
//    16 MHz board → change to MCP_16MHZ
//
//  Library required: MCP_CAN by coryjfowler
//    Arduino IDE → Tools → Manage Libraries → search "MCP_CAN"
// ================================================================

#include <SPI.h>
#include <mcp_can.h>

// ── Pin definitions ────────────────────────────────────────────
#define PIN_CAN_CS   5
#define PIN_CAN_INT  4

MCP_CAN CAN(PIN_CAN_CS);

// ── CAN IDs — received (TCM → emulator) ───────────────────────
// 0x540 = Getriebe_2: transmission output speed frame sent by TCM
#define ID_GETRIEBE_2   0x540

// ── CAN IDs — transmitted (emulator → bus) ────────────────────
#define ID_BREMSE_2     0x5A0   // Speedometer (cluster)
#define ID_BREMSE_1     0x1A0   // Wheel speed (ECU, TCM)
#define ID_BREMSE_3     0x4A0   // ABS/ESP keepalive

// ── Speed encoding factor ──────────────────────────────────────
// Confirmed formula for PQ35 (tested on real hardware):
//   raw = km/h × 148  →  km/h = raw / 148
// If speedometer reads differently from GPS, adjust this value.
// Alternative factor reported for some PQ35 transmissions: 322.0
#define SPEED_FACTOR    148.0f

// ── Distance counter (0x5A0 bytes 5–6) ────────────────────────
// 50 counts per meter traveled, overflow at 30000.
// CRITICAL: this counter MUST be incremented proportionally to
// speed. Without it, the speedometer needle rises for ~10 seconds
// and then drops — a well-documented PQ35 behavior.
#define COUNTS_PER_METER  50.0f
#define DIST_OVERFLOW     30000

// ── Timing ─────────────────────────────────────────────────────
#define INTERVAL_FAST_MS   10   // 100 Hz — 0x5A0 and 0x1A0
#define INTERVAL_SLOW_MS   50   // 20 Hz  — 0x4A0 keepalive

// ── State ──────────────────────────────────────────────────────
float    speedKmh      = 0.0f;  // Current speed in km/h
uint32_t lastRxTime    = 0;     // Timestamp of last 0x540 received
uint32_t lastFastTick  = 0;     // Fast loop timer
uint32_t lastSlowTick  = 0;     // Slow loop timer

// Distance counter
float    distAccum     = 0.0f;  // Sub-meter accumulator
uint16_t distCounter   = 0;     // Integer counter (0..29999)

// Frame life counter (incremented on every transmission)
uint8_t  frameCount    = 0;

// Moving average filter (smooths TCM speed readings)
#define  AVG_SAMPLES  5
float    avgBuf[AVG_SAMPLES] = {0};
uint8_t  avgIdx               = 0;

// ================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== PQ35 ABS Emulator ==="));

  // Initialize MCP2515
  // CAN_500KBPS = VAG powertrain CAN bus speed
  // MCP_8MHZ    = crystal on your MCP2515 board (change to MCP_16MHZ if needed)
  uint8_t attempt = 0;
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.printf("[INIT] MCP2515 failed (attempt %d). Check:\n", ++attempt);
    Serial.println(F("  - MCP2515 VCC connected to VIN (5V), not 3.3V"));
    Serial.println(F("  - SPI pins: CS=5, SCK=18, MOSI=23, MISO=19"));
    Serial.println(F("  - Crystal: MCP_8MHZ or MCP_16MHZ?"));
    delay(1000);
    if (attempt >= 15) {
      Serial.println(F("[INIT] CRITICAL FAILURE. Halting."));
      while (true) delay(1000);
    }
  }

  // Configure hardware RX filters — accept only 0x540 (Getriebe_2)
  // This prevents the ESP32 from processing every frame on the bus
  CAN.init_Mask(0, 0, 0x7FF);
  CAN.init_Mask(1, 0, 0x7FF);
  CAN.init_Filt(0, 0, ID_GETRIEBE_2);
  CAN.init_Filt(1, 0, ID_GETRIEBE_2);
  CAN.init_Filt(2, 0, ID_GETRIEBE_2);
  CAN.init_Filt(3, 0, ID_GETRIEBE_2);
  CAN.init_Filt(4, 0, ID_GETRIEBE_2);
  CAN.init_Filt(5, 0, ID_GETRIEBE_2);

  CAN.setMode(MCP_NORMAL);
  // For bench testing without the car, use MCP_LOOPBACK instead:
  // CAN.setMode(MCP_LOOPBACK);

  Serial.println(F("[INIT] CAN bus OK — 500 kbps"));
  Serial.println(F("[INIT] Listening : 0x540 (Getriebe_2 from TCM)"));
  Serial.println(F("[INIT] Sending   : 0x5A0 (speedometer), 0x1A0 (wheel speed), 0x4A0 (ABS status)"));
  Serial.println(F("[INIT] Ready.\n"));
}

// ================================================================
void loop() {
  uint32_t now = millis();

  readCAN(now);
  checkTimeout(now);

  // Fast loop — 10ms (100 Hz)
  if (now - lastFastTick >= INTERVAL_FAST_MS) {
    uint32_t dt = now - lastFastTick;
    lastFastTick = now;
    frameCount++;

    // Update distance counter
    // distance(m) = speed(m/s) × time(s) = (km/h / 3.6) × (dt / 1000)
    float distM = (speedKmh / 3.6f) * (dt / 1000.0f);
    distAccum += distM;
    uint32_t newCounts = (uint32_t)(distAccum * COUNTS_PER_METER);
    if (newCounts > 0) {
      distAccum -= newCounts / COUNTS_PER_METER;
      distCounter = (distCounter + newCounts) % DIST_OVERFLOW;
    }

    sendBremse2();
    sendBremse1();
  }

  // Slow loop — 50ms (20 Hz)
  if (now - lastSlowTick >= INTERVAL_SLOW_MS) {
    lastSlowTick = now;
    sendBremse3();
  }
}

// ================================================================
// Read 0x540 (Getriebe_2) from TCM and extract vehicle speed
//
// Frame format (VAG PQ35 KMatrix confirmed):
//   Byte 0: TCM sequence counter
//   Byte 1: Speed HIGH byte
//   Byte 2: Speed LOW byte
//   Byte 3–7: Transmission status, gear position, flags
//
// Decoding: raw = (byte1 << 8) | byte2
//           km/h = raw / SPEED_FACTOR
//
// Calibration: if speedometer differs from GPS, adjust SPEED_FACTOR.
//   Open Serial Monitor (115200 baud), note the raw= value at a
//   known GPS speed, then: SPEED_FACTOR = raw / gps_speed_kmh
// ================================================================
void readCAN(uint32_t now) {
  if (digitalRead(PIN_CAN_INT) == LOW) {
    uint32_t rxId;
    uint8_t  rxLen;
    uint8_t  buf[8];

    if (CAN.readMsgBuf(&rxId, &rxLen, buf) == CAN_OK
        && rxId == ID_GETRIEBE_2
        && rxLen >= 3) {

      uint16_t rawSpeed = ((uint16_t)buf[1] << 8) | buf[2];
      float newSpeed = rawSpeed / SPEED_FACTOR;

      // Clamp to reasonable range
      newSpeed = constrain(newSpeed, 0.0f, 280.0f);

      // Moving average filter
      avgBuf[avgIdx] = newSpeed;
      avgIdx = (avgIdx + 1) % AVG_SAMPLES;
      float sum = 0;
      for (uint8_t i = 0; i < AVG_SAMPLES; i++) sum += avgBuf[i];
      speedKmh = sum / AVG_SAMPLES;

      lastRxTime = now;

      // Log once per second (~100 fast cycles)
      static uint8_t logTick = 0;
      if (++logTick >= 100) {
        logTick = 0;
        Serial.printf("[RX 0x540] raw=%5u | speed=%6.1f km/h | distCnt=%5u\n",
                      rawSpeed, speedKmh, distCounter);
      }
    }
  }
}

// ================================================================
// Timeout: if no 0x540 received for 500ms, assume vehicle stopped
// ================================================================
void checkTimeout(uint32_t now) {
  if (speedKmh > 0.1f && (now - lastRxTime) > 500) {
    speedKmh = 0.0f;
    for (uint8_t i = 0; i < AVG_SAMPLES; i++) avgBuf[i] = 0;
    Serial.println(F("[TIMEOUT] 0x540 lost — speed set to zero"));
  }
}

// ================================================================
// 0x5A0 — Bremse_2 — Cluster speedometer
//
// Source: github.com/an-ven/VW-Instrument-Cluster-Controller
//         Tested on VW UP (PQ35), confirmed working.
//
// Byte 0: 0xFF (fixed)
// Byte 1: LSB(raw_speed)    raw = km/h × 148
// Byte 2: MSB(raw_speed)
// Byte 3: flags (bit 3 = tire pressure warning)
// Byte 4: 0x00
// Byte 5: LSB(distCounter)  50 counts/meter, overflow at 30000
// Byte 6: MSB(distCounter)
// Byte 7: 0xAD (fixed PQ35 frame identifier)
// ================================================================
void sendBremse2() {
  uint16_t rawSpeed = (uint16_t)(speedKmh * SPEED_FACTOR);

  uint8_t data[8] = {
    0xFF,
    (uint8_t)(rawSpeed & 0xFF),           // Byte 1: speed LSB
    (uint8_t)((rawSpeed >> 8) & 0xFF),    // Byte 2: speed MSB
    0x00,                                 // Byte 3: flags (normal)
    0x00,
    (uint8_t)(distCounter & 0xFF),        // Byte 5: distance LSB
    (uint8_t)((distCounter >> 8) & 0xFF), // Byte 6: distance MSB
    0xAD                                  // Byte 7: fixed
  };

  if (CAN.sendMsgBuf(ID_BREMSE_2, 0, 8, data) != CAN_OK)
    Serial.println(F("[TX ERR] 0x5A0"));
}

// ================================================================
// 0x1A0 — Bremse_1 — Wheel speed for ECU and TCM
//
// Source: vehicle-reverse-engineering.fandom.com/wiki/Volkswagen
//         Real CAN log from VW Passat B6 PQ35.
//
// Byte 0: 0x18 (normal status — no ABS intervention)
// Byte 1: speed LSB   raw = km/h × 148
// Byte 2: speed MSB
// Byte 3: ABS/ESP flags
//           0x00 = normal
//           0x04 = ABS active
//           0x81 = ABS warning light
//           0x84 = handbrake applied
// Byte 4: 0xFE (fixed)
// Byte 5: 0xFE (fixed)
// Byte 6: 0x00
// Byte 7: frame life counter
// ================================================================
void sendBremse1() {
  uint16_t rawSpeed = (uint16_t)(speedKmh * SPEED_FACTOR);

  uint8_t data[8] = {
    0x18,                                 // Byte 0: normal status
    (uint8_t)(rawSpeed & 0xFF),           // Byte 1: speed LSB
    (uint8_t)((rawSpeed >> 8) & 0xFF),    // Byte 2: speed MSB
    0x00,                                 // Byte 3: no ABS/ESP intervention
    0xFE,                                 // Byte 4: fixed
    0xFE,                                 // Byte 5: fixed
    0x00,
    frameCount                            // Byte 7: life counter
  };

  if (CAN.sendMsgBuf(ID_BREMSE_1, 0, 8, data) != CAN_OK)
    Serial.println(F("[TX ERR] 0x1A0"));
}

// ================================================================
// 0x4A0 — Bremse_3 — ABS/ESP module keepalive
//
// Informs the bus that the ABS module is present and healthy.
// Without this frame, the cluster and ECU will flag a module
// communication fault.
// ================================================================
void sendBremse3() {
  uint8_t data[8] = {
    0x00,   // No faults
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    frameCount  // Life counter
  };

  if (CAN.sendMsgBuf(ID_BREMSE_3, 0, 8, data) != CAN_OK)
    Serial.println(F("[TX ERR] 0x4A0"));
}
