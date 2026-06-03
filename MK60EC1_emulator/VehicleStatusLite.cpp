#include "VehicleStatusLite.h"

#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

namespace {

portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
VehicleStatusLite gSnap;

}  // namespace

void vehicleStatusBegin() {
  portENTER_CRITICAL(&gMux);
  memset(&gSnap, 0, sizeof(gSnap));
  portEXIT_CRITICAL(&gMux);
}

void vehicleStatusPublish(const VehicleStatusLite &s) {
  portENTER_CRITICAL(&gMux);
  gSnap = s;
  gSnap.seq++;
  portEXIT_CRITICAL(&gMux);
}

void vehicleStatusCopy(VehicleStatusLite &out) {
  portENTER_CRITICAL(&gMux);
  out = gSnap;
  portEXIT_CRITICAL(&gMux);
}

const char *vehicleStatusGearWahlLabel(uint8_t wahl) {
  switch (wahl) {
    case 8:
      return "P";
    case 7:
      return "R";
    case 6:
      return "N";
    case 5:
      return "D";
    case 12:
      return "S";
    default:
      return "?";
  }
}
