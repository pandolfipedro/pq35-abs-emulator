#pragma once

#include <stdint.h>

struct VehicleStatusLite {
  uint32_t seq = 0;
  uint32_t updatedMs = 0;

  float speedFiltKmh = 0.0f;
  float speedPanelKmh = 0.0f;
  float speedCanTxKmh = 0.0f;
  float speedOdoKmh = 0.0f;
  bool obdValid = false;

  uint8_t gearWahl = 0xFF;
  uint8_t gearEngaged = 0xFF;
  bool gearValid = false;
  bool inPark = false;
  bool motionDriving = false;

  uint32_t odoImpulses = 0;
  float impPerKm = 0.0f;
  uint8_t rxBuf = 0;

  uint32_t txFail1A0 = 0;
  uint32_t txFail4A0 = 0;
  uint32_t txFail5A0 = 0;
  uint32_t txFail3A0 = 0;
  uint32_t txFailEsp = 0;

  bool mcpOk = false;
};

void vehicleStatusBegin();
void vehicleStatusPublish(const VehicleStatusLite &s);
void vehicleStatusCopy(VehicleStatusLite &out);
const char *vehicleStatusGearWahlLabel(uint8_t wahl);
