#pragma once
// Web tuning app: static page + small JSON API. The page polls /api/live.
#include <WebServer.h>
#include <DNSServer.h>
#include <Update.h>
#include <ArduinoJson.h>
#include "web_ui.h"

WebServer server(80);
DNSServer dns;

extern volatile uint32_t g_map_raw_mv;       // defined in the .ino
extern volatile uint8_t g_lpg_override, g_lpg_in;
extern volatile int32_t g_lpg_mv;
extern volatile uint32_t g_map_sensor_mv;    // filtered sensor-output mV (before the divider)
extern float g_baro_hint_kpa;
static bool g_otaOpen = false;

// Params <-> JSON in human units (degrees, not tenths).
#define P_DEG(name, field)   do { if (rd) { if (!j[name].isNull()) p.field = (int16_t)lroundf(j[name].as<float>() * 10.0f); } else j[name] = p.field / 10.0f; } while (0)
#define P_INT(name, field)   do { if (rd) { if (!j[name].isNull()) p.field = j[name].as<long>(); } else j[name] = (long)p.field; } while (0)

static void paramsJson(JsonObject j, bool rd, IgnParams& p) {
  P_DEG("trigger_adv", trigger_adv_x10);
  P_INT("latency_us", latency_us);
  P_INT("trigger_edge", trigger_edge);
  P_INT("trigger_ref", trigger_ref);
  P_DEG("open_adv", open_adv_x10);
  P_INT("map_sel", map_sel);
  P_DEG("factory_static", factory_static_x10);
  P_INT("cylinders", cylinders);
  P_INT("idle_enable", idle_enable);
  P_INT("accel_comp", accel_comp_pct);
  P_INT("crank_rpm", crank_rpm);
  P_DEG("crank_adv", crank_adv_x10);
  P_DEG("min_adv", min_adv_x10);
  P_DEG("max_adv", max_adv_x10);
  P_INT("rev_soft", rev_soft_rpm);
  P_INT("rev_hard", rev_hard_rpm);
  P_DEG("rev_retard", rev_retard_x10);
  P_DEG("adv_slew", adv_slew_x10);
  P_INT("min_dwell", min_dwell_us);
  P_INT("max_dwell", max_dwell_us);
  P_INT("idle_target", idle_target_rpm);
  P_INT("idle_map_max", idle_map_max_kpa);
  P_DEG("idle_kp", idle_kp);
  P_DEG("idle_ki", idle_ki);
  P_DEG("idle_trim_max", idle_trim_max_x10);
  P_INT("map_gain_x1000", map_gain_x1000);
  P_DEG("map_offset", map_offset_x10);
  P_INT("lpg_mode", lpg_mode);
  P_DEG("lpg_offset", lpg_offset_x10);
  if (rd) {
    JsonArrayConst cm = j["map_cal_mv"], ck = j["map_cal_kpa"];
    if (cm.size() == 2 && ck.size() == 2)
      for (int i = 0; i < 2; i++) { p.map_cal_mv[i] = cm[i].as<int>(); p.map_cal_kpa_x10[i] = (int16_t)lroundf(ck[i].as<float>() * 10.0f); }
  } else {
    JsonArray cm = j["map_cal_mv"].to<JsonArray>(), ck = j["map_cal_kpa"].to<JsonArray>();
    for (int i = 0; i < 2; i++) { cm.add(p.map_cal_mv[i]); ck.add(p.map_cal_kpa_x10[i] / 10.0f); }
  }
  if (rd) {
    JsonArrayConst mv = j["dwell_mv"], us = j["dwell_us"];
    for (int i = 0; i < IGN_NDWELL; i++) {
      if (mv.size() == IGN_NDWELL) p.dwell_mv[i] = mv[i].as<int>();
      if (us.size() == IGN_NDWELL) p.dwell_us[i] = us[i].as<int>();
    }
  } else {
    JsonArray mv = j["dwell_mv"].to<JsonArray>(), us = j["dwell_us"].to<JsonArray>();
    for (int i = 0; i < IGN_NDWELL; i++) { mv.add(p.dwell_mv[i]); us.add(p.dwell_us[i]); }
  }
}

static void tablesJson(JsonObject j, bool rd, IgnTables& t) {
  if (rd) {
    JsonArrayConst ra = j["rpm_axis"], ma = j["map_axis"], adv = j["adv"], advl = j["adv_lpg"];
    if (ra.size() == IGN_NRPM) for (int i = 0; i < IGN_NRPM; i++) t.rpm_axis[i] = ra[i].as<int>();
    if (ma.size() == IGN_NMAP) for (int i = 0; i < IGN_NMAP; i++) t.map_axis[i] = ma[i].as<int>();
    if (adv.size() == IGN_NRPM * IGN_NMAP)
      for (int i = 0; i < IGN_NRPM; i++) for (int k = 0; k < IGN_NMAP; k++)
        t.adv_x10[i][k] = (int16_t)lroundf(adv[i * IGN_NMAP + k].as<float>() * 10.0f);
    if (advl.size() == IGN_NRPM * IGN_NMAP)
      for (int i = 0; i < IGN_NRPM; i++) for (int k = 0; k < IGN_NMAP; k++)
        t.adv_lpg_x10[i][k] = (int16_t)lroundf(advl[i * IGN_NMAP + k].as<float>() * 10.0f);
    JsonArrayConst advp = j["adv_pre"];
    if (advp.size() == 3)
      for (int s = 0; s < 3; s++) {
        JsonArrayConst a = advp[s];
        if (a.size() == IGN_NRPM * IGN_NMAP)
          for (int i = 0; i < IGN_NRPM; i++) for (int k = 0; k < IGN_NMAP; k++)
            t.adv_pre_x10[s][i][k] = (int16_t)lroundf(a[i * IGN_NMAP + k].as<float>() * 10.0f);
      }
  } else {
    JsonArray ra = j["rpm_axis"].to<JsonArray>(), ma = j["map_axis"].to<JsonArray>(), adv = j["adv"].to<JsonArray>(), advl = j["adv_lpg"].to<JsonArray>();
    for (int i = 0; i < IGN_NRPM; i++) ra.add(t.rpm_axis[i]);
    for (int i = 0; i < IGN_NMAP; i++) ma.add(t.map_axis[i]);
    for (int i = 0; i < IGN_NRPM; i++) for (int k = 0; k < IGN_NMAP; k++) { adv.add(t.adv_x10[i][k] / 10.0f); advl.add(t.adv_lpg_x10[i][k] / 10.0f); }
    JsonArray advp = j["adv_pre"].to<JsonArray>();
    for (int s = 0; s < 3; s++) {
      JsonArray a = advp.add<JsonArray>();
      for (int i = 0; i < IGN_NRPM; i++) for (int k = 0; k < IGN_NMAP; k++) a.add(t.adv_pre_x10[s][i][k] / 10.0f);
    }
  }
}

static void sendJson(JsonDocument& d, int code = 200) {
  String s; serializeJson(d, s);
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", s);
}
static void sendOk(const char* msg = "ok") { server.send(200, "application/json", String("{\"ok\":true,\"msg\":\"") + msg + "\"}"); }
static void sendErr(int code, const char* msg) { server.send(code, "application/json", String("{\"ok\":false,\"msg\":\"") + msg + "\"}"); }

static bool readBody(JsonDocument& d) {
  if (!server.hasArg("plain")) return false;
  return !deserializeJson(d, server.arg("plain"));
}

static int stateCode() {
  if (g_sim_on) return 5;
  if (!g_state.have_edge) return 0;
  if (g_lock_en) return 4;
  if (g_state.cut) return 3;
  return g_state.cranking ? 1 : 2;
}

static void hLive() {
  JsonDocument d;
  d["rpm"] = g_sim_on ? g_sim_rpm : (g_state.have_period ? g_state.rpm : 0);
  d["map"] = g_map_kpa_x10 / 10.0f;
  d["vbat"] = g_vbat_mv / 1000.0f;
  d["adv"] = g_state.adv_x10 / 10.0f;
  d["trim"] = g_idle_trim_x10 / 10.0f;
  d["dwell"] = g_state.dwell_us;
  d["state"] = stateCode();
  d["coil"] = g_state.coil_on;
  d["dirty"] = (bool)g_dirty;
  d["can_save"] = !engineRunning();
  d["lock"] = (bool)g_lock_en;
  d["lock_adv"] = g_lock_adv_x10 / 10.0f;
  d["map_mv"] = g_map_sensor_mv;
  d["sparks"] = g_spark_count;
  d["map_sel"] = g_params.map_sel;
  d["lpg"] = g_lpg_active; d["lpg_in"] = g_lpg_in; d["lpg_ov"] = g_lpg_override; d["lpg_mv"] = g_lpg_mv;
  JsonObject c = d["diag"].to<JsonObject>();
  c["open"] = g_state.open_count; c["bounce"] = g_state.bounce_count; c["late"] = g_state.late_count;
  c["cut"] = g_state.cut_count; c["dwell_limited"] = g_state.dwell_limited_count; c["failsafe"] = g_failsafe_count;
  c["heap"] = ESP.getFreeHeap(); c["up"] = millis() / 1000;
  sendJson(d);
}

static void hConfigGet() {
  JsonDocument d;
  d["fw"] = FIRMWARE_VERSION;
  d["model"] = DEVICE_MODEL;
  paramsJson(d["params"].to<JsonObject>(), false, g_params);
  tablesJson(d["tables"].to<JsonObject>(), false, g_tables);
  sendJson(d);
}

static void hConfigPost() {
  JsonDocument d;
  if (!readBody(d)) return sendErr(400, "bad json");
  // Edit a working copy so the ISRs never see a half-applied/unsanitised set.
  IgnParams p = g_params; IgnTables t = g_tables;
  if (d["params"].is<JsonObject>()) paramsJson(d["params"].as<JsonObject>(), true, p);
  if (d["tables"].is<JsonObject>()) tablesJson(d["tables"].as<JsonObject>(), true, t);
  sanitize(p, t);
  portENTER_CRITICAL(&s_mux);
  g_params = p; g_tables = t;
  portEXIT_CRITICAL(&s_mux);
  ecuApplyEdge();
  g_dirty = true;
  sendOk();
}

// Fast path for the map editor: [[row,col,deg],...]
static void hCells() {
  JsonDocument d;
  if (!readBody(d) || !d.is<JsonArray>()) return sendErr(400, "bad json");
  for (JsonArray c : d.as<JsonArray>()) {
    int r = c[0], k = c[1];
    if (r < 0 || r >= IGN_NRPM || k < 0 || k >= IGN_NMAP) continue;
    int m = c.size() > 3 ? c[3].as<int>() : 0;                // 0 custom, 1 LPG, 2..4 factory presets
    int16_t (*dst)[IGN_NMAP] = m == 1 ? g_tables.adv_lpg_x10 : (m >= 2 && m <= 4) ? g_tables.adv_pre_x10[m - 2] : g_tables.adv_x10;
    dst[r][k] = ign_clamp((int32_t)lroundf(c[2].as<float>() * 10.0f), g_params.min_adv_x10, 900);
  }
  g_dirty = true;
  sendOk();
}

static void hSave() {
  if (engineRunning()) return sendErr(409, "engine running - stop it to save");
  if (!storeSave()) return sendErr(500, "flash write failed");
  sendOk("saved");
}

static void hDefaults() {
  JsonDocument d;
  readBody(d);
  const char* what = d["what"] | "all";
  IgnParams pd; IgnTables td; ign_default_params(&pd); ign_default_tables(&td);
  if (!strcmp(what, "all") || !strcmp(what, "params")) g_params = pd;
  if (!strcmp(what, "all") || !strcmp(what, "tables")) g_tables = td;
  sanitize(g_params, g_tables); ecuApplyEdge(); g_dirty = true;
  sendOk();
}

static void hLock() {
  JsonDocument d;
  if (!readBody(d)) return sendErr(400, "bad json");
  float adv = d["adv"] | 10.0f;
  g_lock_adv_x10 = ign_clamp((int32_t)lroundf(adv * 10), g_params.min_adv_x10, g_params.trigger_adv_x10);
  g_lock_en = d["en"] ? 1 : 0;
  sendOk();
}

// Regenerate a factory preset slot (1..3) from the distributor data, optionally with a new static timing.
static void hPreset() {
  JsonDocument d;
  if (!readBody(d)) return sendErr(400, "bad json");
  int slot = d["slot"] | 0;
  if (slot < 1 || slot > 3) return sendErr(400, "slot out of range");
  if (!d["static"].isNull()) g_params.factory_static_x10 = ign_clamp((int32_t)lroundf(d["static"].as<float>() * 10.0f), -50, 200);
  int16_t tmp[IGN_NRPM][IGN_NMAP];
  ign_factory_fill(&g_tables, tmp, slot - 1, g_params.factory_static_x10);
  portENTER_CRITICAL(&s_mux);
  memcpy(g_tables.adv_pre_x10[slot - 1], tmp, sizeof tmp);
  portEXIT_CRITICAL(&s_mux);
  g_dirty = true;
  sendOk();
}

static void hLpg() {
  JsonDocument d;
  if (!readBody(d)) return sendErr(400, "bad json");
  int o = d["override"] | 0;
  g_lpg_override = (o >= 0 && o <= 2) ? o : 0;
  sendOk();
}

// Copy petrol map -> LPG map, adding an optional offset (deg).
static void hCopyMap() {
  JsonDocument d;
  readBody(d);
  int off = (int)lroundf((d["offset"] | 0.0f) * 10.0f);
  for (int i = 0; i < IGN_NRPM; i++) for (int k = 0; k < IGN_NMAP; k++)
    g_tables.adv_lpg_x10[i][k] = ign_clamp(((g_params.map_sel >= 1 && g_params.map_sel <= 3) ? g_tables.adv_pre_x10[g_params.map_sel - 1][i][k] : g_tables.adv_x10[i][k]) + off,
                                           g_params.min_adv_x10, 900);
  g_dirty = true;
  sendOk();
}

static void hSim() {
  JsonDocument d;
  if (!readBody(d)) return sendErr(400, "bad json");
  simSet(d["rpm"] | 0);
  sendOk();
}

// Set the MAP offset so the reading equals the given pressure (engine off =
// atmospheric). Based on the *uncalibrated* reading + current gain.
static void hBaro() {
  JsonDocument d;
  if (!readBody(d)) return sendErr(400, "bad json");
  float kpa = d["kpa"] | 101.3f;
  if (kpa < 60 || kpa > 110) return sendErr(400, "kpa out of range");
  float shown = g_map_kpa_x10 / 10.0f;
  g_params.map_offset_x10 = ign_clamp((int32_t)lroundf(g_params.map_offset_x10 + (kpa - shown) * 10), -300, 300);
  g_dirty = true;
  sendOk();
}

static void hUpdateDone() {
  bool ok = !Update.hasError();
  server.sendHeader("Connection", "close");
  server.send(ok ? 200 : 500, "text/plain; charset=utf-8", ok ? "OK - rebooting / рестартиране" : "FAILED / грешка");
  if (ok) { delay(300); ESP.restart(); }
}

static void hUpdateUpload() {
  HTTPUpload& u = server.upload();
  if (u.status == UPLOAD_FILE_START) {
    ecuOutputOff();
    Update.begin(UPDATE_SIZE_UNKNOWN);
  } else if (u.status == UPLOAD_FILE_WRITE) {
    Update.write(u.buf, u.currentSize);
  } else if (u.status == UPLOAD_FILE_END) {
    Update.end(true);
  }
}

static bool authOk() {
  if (server.authenticate("admin", OTA_PASSWORD)) return true;
  server.requestAuthentication();
  return false;
}

static void webBegin() {
  server.on("/", HTTP_GET, []() { server.sendHeader("Cache-Control", "no-cache"); server.sendHeader("Content-Encoding", "gzip"); server.send_P(200, "text/html; charset=utf-8", (PGM_P)WEB_UI_GZ, WEB_UI_GZ_LEN); });
  server.on("/api/live", HTTP_GET, hLive);
  server.on("/api/config", HTTP_GET, hConfigGet);
  server.on("/api/config", HTTP_POST, hConfigPost);
  server.on("/api/cells", HTTP_POST, hCells);
  server.on("/api/save", HTTP_POST, hSave);
  server.on("/api/defaults", HTTP_POST, hDefaults);
  server.on("/api/lock", HTTP_POST, hLock);
  server.on("/api/sim", HTTP_POST, hSim);
  server.on("/api/lpg", HTTP_POST, hLpg);
  server.on("/api/preset", HTTP_POST, hPreset);
  server.on("/api/copymap", HTTP_POST, hCopyMap);
  server.on("/api/baro", HTTP_POST, hBaro);
  server.on("/update", HTTP_GET, []() {
    if (!authOk()) return;
    if (engineRunning()) return server.send(409, "text/plain; charset=utf-8", "Stop the engine first / Първо спрете двигателя");
    server.send(200, "text/html",
      "<meta charset=utf-8><meta name=viewport content='width=device-width'><body style='font-family:sans-serif;padding:16px'>"
      "<h3>Firmware update / Обновяване на фърмуера</h3><form method=POST enctype=multipart/form-data>"
      "<input type=file name=fw accept='.bin'><br><br><button>Upload / Качване</button></form>");
  });
  server.on("/update", HTTP_POST, []() { if (authOk()) hUpdateDone(); },
            []() { if (server.authenticate("admin", OTA_PASSWORD) && !engineRunning()) hUpdateUpload(); });
  // captive portal: anything else goes to the app
  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain; charset=utf-8", "");
  });
  server.begin();
  dns.start(53, "*", WiFi.softAPIP());
}
