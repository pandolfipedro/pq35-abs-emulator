#pragma once

#include <Arduino.h>

#ifndef PQ35_WIFI_PORTAL
#define PQ35_WIFI_PORTAL 0
#endif

#if PQ35_WIFI_PORTAL
void webPortalBegin(const char *firmwareVersion, const char *variant);
#else
inline void webPortalBegin(const char *, const char *) {}
#endif
