#include "WebPortal.h"

#if PQ35_WIFI_PORTAL

#include <WebServer.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Update.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "VehicleStatusLite.h"
#include "WebPortalPages.h"

namespace {

static constexpr const char *kApSsid = "PQ35-Config";
static constexpr const char *kNvsNamespace = "pq35_portal";
static constexpr const char *kDefaultWifiPass = "pq35admin";
static constexpr const char *kDefaultWebUser = "admin";
static constexpr const char *kDefaultWebPass = "admin";
static constexpr uint8_t kCfgVersion = 1;
static constexpr size_t kMinPassLen = 6;

static WebServer gServer(80);
static Preferences gPrefs;
static String gFirmwareVersion;
static String gFirmwareVariant;
static String gWifiPass;
static String gWebUser;
static String gWebPass;
static TaskHandle_t gWebTaskHandle = nullptr;

static const char *resetReasonToStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

static String formatUptime(uint32_t sec) {
  const uint32_t d = sec / 86400u;
  sec %= 86400u;
  const uint32_t h = sec / 3600u;
  sec %= 3600u;
  const uint32_t m = sec / 60u;
  const uint32_t s = sec % 60u;
  char buf[32];
  if (d > 0)
    snprintf(buf, sizeof(buf), "%lud %02luh %02lum %02lus", (unsigned long)d, (unsigned long)h,
             (unsigned long)m, (unsigned long)s);
  else
    snprintf(buf, sizeof(buf), "%02luh %02lum %02lus", (unsigned long)h, (unsigned long)m,
             (unsigned long)s);
  return String(buf);
}

static void loadCredentialsFromNvs() {
  gPrefs.begin(kNvsNamespace, false);

  const uint8_t cfgVer = gPrefs.getUChar("cfg_ver", 0);
  if (cfgVer != kCfgVersion) {
    gPrefs.putUChar("cfg_ver", kCfgVersion);
    gPrefs.putString("wifi_pass", kDefaultWifiPass);
    gPrefs.putString("web_user", kDefaultWebUser);
    gPrefs.putString("web_pass", kDefaultWebPass);
  }

  gWifiPass = gPrefs.getString("wifi_pass", kDefaultWifiPass);
  gWebUser = gPrefs.getString("web_user", kDefaultWebUser);
  gWebPass = gPrefs.getString("web_pass", kDefaultWebPass);
  gPrefs.end();

  if (gWifiPass.length() < kMinPassLen)
    gWifiPass = kDefaultWifiPass;
  if (gWebUser.length() == 0)
    gWebUser = kDefaultWebUser;
  if (gWebPass.length() < kMinPassLen)
    gWebPass = kDefaultWebPass;
}

static void saveCredentialsToNvs() {
  gPrefs.begin(kNvsNamespace, false);
  gPrefs.putUChar("cfg_ver", kCfgVersion);
  gPrefs.putString("wifi_pass", gWifiPass);
  gPrefs.putString("web_user", gWebUser);
  gPrefs.putString("web_pass", gWebPass);
  gPrefs.end();
}

static bool requireAuth() {
  if (!gServer.authenticate(gWebUser.c_str(), gWebPass.c_str())) {
    gServer.requestAuthentication();
    return false;
  }
  return true;
}

static String readProgmemCss() { return String(FPSTR(kPortalCss)); }

static String replaceAll(String src, const String &from, const String &to) {
  if (from.length() == 0)
    return src;
  int idx = 0;
  while ((idx = src.indexOf(from, idx)) >= 0) {
    src.replace(from, to);
    idx += to.length();
  }
  return src;
}

static String htmlEscape(const String &in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); ++i) {
    const char c = in.charAt(i);
    if (c == '&')
      out += "&amp;";
    else if (c == '<')
      out += "&lt;";
    else if (c == '>')
      out += "&gt;";
    else if (c == '"')
      out += "&quot;";
    else
      out += c;
  }
  return out;
}

static void sendHomePage() {
  const String css = readProgmemCss();
  String html = String(FPSTR(kPageHomeHead)) + css + String(FPSTR(kPageHomeTail));

  const uint32_t uptimeSec = millis() / 1000u;
  const uint32_t heapFree = esp_get_free_heap_size();
  const uint32_t heapMin = esp_get_minimum_free_heap_size();

  html = replaceAll(html, "%VERSION%", htmlEscape(gFirmwareVersion));
  html = replaceAll(html, "%VARIANT%", htmlEscape(gFirmwareVariant));
  html = replaceAll(html, "%UPTIME%", formatUptime(uptimeSec));
  html = replaceAll(html, "%HEAP%", String(heapFree) + " / " + String(heapMin));
  html = replaceAll(html, "%RESET%", resetReasonToStr(esp_reset_reason()));

  gServer.send(200, F("text/html; charset=utf-8"), html);
}

static void sendSettingsPage(const String &msgHtml = String()) {
  const String css = readProgmemCss();
  String html = String(FPSTR(kPageSettings)) + css + String(FPSTR(kPageSettingsBody));
  html = replaceAll(html, "%MSG%", msgHtml);
  html = replaceAll(html, "%WEB_USER%", htmlEscape(gWebUser));
  gServer.send(200, F("text/html; charset=utf-8"), html);
}

static void sendUpdatePage() {
  const String css = readProgmemCss();
  const String html = String(FPSTR(kPageUpdate)) + css + String(FPSTR(kPageUpdateBody));
  gServer.send(200, F("text/html; charset=utf-8"), html);
}

static void handleRoot() {
  if (!requireAuth())
    return;
  sendHomePage();
}

static void handleSettingsGet() {
  if (!requireAuth())
    return;
  sendSettingsPage();
}

static void handleSettingsPost() {
  if (!requireAuth())
    return;

  const String currentPass = gServer.arg("current_web_pass");
  const String newWebUser = gServer.arg("web_user");
  const String newWebPass = gServer.arg("web_pass_new");
  const String newWebPassConfirm = gServer.arg("web_pass_confirm");
  const String newWifiPass = gServer.arg("wifi_pass_new");
  const String newWifiPassConfirm = gServer.arg("wifi_pass_confirm");

  auto fail = [&](const char *msg) {
    sendSettingsPage(String("<div class=\"msg msg-err\">") + msg + "</div>");
  };

  if (currentPass != gWebPass) {
    fail("Current web password is incorrect.");
    return;
  }
  if (newWebUser.length() == 0) {
    fail("Invalid web username.");
    return;
  }

  bool wifiChanged = false;
  bool credsChanged = false;

  if (newWebPass.length() > 0) {
    if (newWebPass.length() < kMinPassLen) {
      fail("New web password must be at least 6 characters.");
      return;
    }
    if (newWebPass != newWebPassConfirm) {
      fail("Web password confirmation does not match.");
      return;
    }
    gWebPass = newWebPass;
    credsChanged = true;
  }

  if (newWebUser != gWebUser) {
    gWebUser = newWebUser;
    credsChanged = true;
  }

  if (newWifiPass.length() > 0) {
    if (newWifiPass.length() < kMinPassLen) {
      fail("New Wi-Fi password must be at least 6 characters.");
      return;
    }
    if (newWifiPass != newWifiPassConfirm) {
      fail("Wi-Fi password confirmation does not match.");
      return;
    }
    gWifiPass = newWifiPass;
    wifiChanged = true;
    credsChanged = true;
  }

  if (!credsChanged) {
    fail("No changes submitted.");
    return;
  }

  saveCredentialsToNvs();

  if (wifiChanged) {
    gServer.send(200, F("text/html; charset=utf-8"),
                 F("<html><body><p>Settings saved. Restarting...</p></body></html>"));
    delay(500);
    esp_restart();
    return;
  }

  sendSettingsPage("<div class=\"msg msg-ok\">Settings saved successfully.</div>");
}

static void handleStatusJson() {
  if (!requireAuth())
    return;

  VehicleStatusLite vs;
  vehicleStatusCopy(vs);

  const uint32_t uptimeSec = millis() / 1000u;
  const char *gearW = vs.gearValid ? vehicleStatusGearWahlLabel(vs.gearWahl) : "?";

  char json[768];
  snprintf(json, sizeof(json),
           "{\"version\":\"%s\",\"variant\":\"%s\",\"uptime_s\":%lu,\"heap_free\":%lu,"
           "\"heap_min\":%lu,\"reset\":\"%s\",\"mcp_ok\":%s,"
           "\"speed_filt\":%.1f,\"speed_panel\":%.1f,\"speed_can_tx\":%.1f,\"speed_odo\":%.1f,"
           "\"obd_valid\":%s,\"gear_wahl\":\"%s\",\"gear_wahl_n\":%u,\"gear_engaged\":%u,"
           "\"gear_valid\":%s,\"in_park\":%s,\"motion_driving\":%s,"
           "\"odo_impulses\":%lu,\"imp_per_km\":%.0f,\"rx_buf\":%u,"
           "\"tx_fail_1a0\":%lu,\"tx_fail_4a0\":%lu,\"tx_fail_5a0\":%lu,\"tx_fail_3a0\":%lu,"
           "\"tx_fail_esp\":%lu,\"status_age_ms\":%lu}",
           gFirmwareVersion.c_str(), gFirmwareVariant.c_str(), (unsigned long)uptimeSec,
           (unsigned long)esp_get_free_heap_size(), (unsigned long)esp_get_minimum_free_heap_size(),
           resetReasonToStr(esp_reset_reason()), vs.mcpOk ? "true" : "false", vs.speedFiltKmh,
           vs.speedPanelKmh, vs.speedCanTxKmh, vs.speedOdoKmh, vs.obdValid ? "true" : "false", gearW,
           (unsigned)vs.gearWahl, (unsigned)vs.gearEngaged, vs.gearValid ? "true" : "false",
           vs.inPark ? "true" : "false", vs.motionDriving ? "true" : "false",
           (unsigned long)vs.odoImpulses, vs.impPerKm, (unsigned)vs.rxBuf,
           (unsigned long)vs.txFail1A0, (unsigned long)vs.txFail4A0, (unsigned long)vs.txFail5A0,
           (unsigned long)vs.txFail3A0, (unsigned long)vs.txFailEsp,
           (unsigned long)(millis() - vs.updatedMs));
  gServer.send(200, F("application/json"), json);
}

static void handleUpdatePostDone() {
  if (!requireAuth())
    return;

  gServer.sendHeader(F("Connection"), F("close"));
  if (Update.hasError()) {
    gServer.send(500, F("text/plain; charset=utf-8"),
                 String("OTA failed: ") + String(Update.errorString()));
    return;
  }

  gServer.send(200, F("text/plain; charset=utf-8"), F("OK"));
  delay(500);
  esp_restart();
}

static void handleUpdateUpload() {
  HTTPUpload &upload = gServer.upload();

  if (upload.status == UPLOAD_FILE_START) {
    if (!requireAuth()) {
      Update.abort();
      return;
    }
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
  }
}

static void setupRoutes() {
  gServer.on(F("/"), HTTP_GET, handleRoot);
  gServer.on(F("/settings"), HTTP_GET, handleSettingsGet);
  gServer.on(F("/settings"), HTTP_POST, handleSettingsPost);
  gServer.on(F("/update"), HTTP_GET, []() {
    if (!requireAuth())
      return;
    sendUpdatePage();
  });
  gServer.on(F("/update"), HTTP_POST, handleUpdatePostDone, handleUpdateUpload);
  gServer.on(F("/api/status"), HTTP_GET, handleStatusJson);
  gServer.onNotFound([]() {
    gServer.send(404, F("text/plain; charset=utf-8"), F("Not found"));
  });
}

static void startHiddenAp() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(WIFI_PS_MIN_MODEM);
  const bool ok = WiFi.softAP(kApSsid, gWifiPass.c_str(), 1, 1, 4);
  WiFi.softAPsetHostname("pq35-config");

  Serial.print(F("[WebPortal] Hidden AP "));
  Serial.print(kApSsid);
  Serial.print(ok ? F(" OK @ ") : F(" FAILED @ "));
  Serial.println(WiFi.softAPIP());
}

static void webTask(void *param) {
  (void)param;
  setupRoutes();
  gServer.begin();
  Serial.println(F("[WebPortal] HTTP at http://192.168.4.1"));

  for (;;) {
    gServer.handleClient();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

}  // namespace

void webPortalBegin(const char *firmwareVersion, const char *variant) {
  gFirmwareVersion = firmwareVersion ? firmwareVersion : "unknown";
  gFirmwareVariant = variant ? variant : "full";
  loadCredentialsFromNvs();
  startHiddenAp();

  if (gWebTaskHandle != nullptr)
    return;

  const BaseType_t created =
      xTaskCreatePinnedToCore(webTask, "web_portal", 8192, nullptr, 1, &gWebTaskHandle, 0);
  if (created != pdPASS) {
    Serial.println(F("[WebPortal] ERROR: failed to create task on Core 0"));
    gWebTaskHandle = nullptr;
  } else {
    Serial.println(F("[WebPortal] Task started on Core 0"));
  }
}

#endif  // PQ35_WIFI_PORTAL
