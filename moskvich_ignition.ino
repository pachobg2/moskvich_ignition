// Moskvich 412/2140 ignition ECU -- ESP32-C3
//
// Points -> crank trigger only. ECU fires the coil through an IGBT with a
// tunable timing map (rpm x MAP), dwell control, idle trim and a rev limit.
// Tuned from a phone via the ECU's own WiFi network (http://192.168.4.1).
// See README.md for hardware, wiring and the (important) install procedure.

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "ecu_engine.h"
#include "ecu_store.h"
#include "ecu_web.h"

volatile uint32_t g_map_raw_mv = 0;
volatile uint32_t g_map_sensor_mv = 0;
static float s_mapMvF = 0;
float g_baro_hint_kpa = 101.3f;

static float s_mapKpa = 100.0f;      // filtered, calibrated
static float s_vbatMv = 12000.0f;
static float s_idleInt = 0;
static uint32_t s_lastAdc = 0, s_lastCtl = 0;
static bool s_wasRunning = false;
static float s_lpgMv = 0;
static int s_lpgCnt = 0;
static uint8_t s_lpgIn = 0;
volatile uint8_t g_lpg_override = 0;      // 0 auto (follow valve), 1 force petrol, 2 force LPG
volatile uint8_t g_lpg_in = 0;
volatile int32_t g_lpg_mv = 0;

static void sampleAnalog() {
  // Two reads per tick, EMA across ticks (~1 kHz): smooths intake pulsation
  // without adding the lag a heavy filter would.
  uint32_t mv = (analogReadMilliVolts(PIN_MAP) + analogReadMilliVolts(PIN_MAP)) / 2;
  g_map_raw_mv = mv;
  float vs = (float)mv / MAP_DIV_RATIO;                          // sensor output, mV
  s_mapMvF += (vs - s_mapMvF) * (1.0f / 12.0f);
  g_map_sensor_mv = (uint32_t)s_mapMvF;
  const IgnParams& q = g_params;                                  // two-point linear calibration
  float k0 = q.map_cal_kpa_x10[0] / 10.0f, k1 = q.map_cal_kpa_x10[1] / 10.0f;
  float kpa = k0 + (vs - q.map_cal_mv[0]) * (k1 - k0) / (float)(q.map_cal_mv[1] - q.map_cal_mv[0]);
  kpa = kpa * (g_params.map_gain_x1000 / 1000.0f) + g_params.map_offset_x10 / 10.0f;
  kpa = constrain(kpa, 0.0f, 250.0f);
  s_mapKpa += (kpa - s_mapKpa) * (1.0f / 12.0f);
  g_map_kpa_x10 = (int32_t)(s_mapKpa * 10.0f);

  float vb = (float)analogReadMilliVolts(PIN_VBAT) / VBAT_DIV_RATIO + VBAT_CAL_MV;
  s_vbatMv += (vb - s_vbatMv) * (1.0f / 32.0f);
  g_vbat_mv = (int32_t)s_vbatMv;

  float lv = (float)analogReadMilliVolts(PIN_LPG) / LPG_DIV_RATIO;
  s_lpgMv += (lv - s_lpgMv) * (1.0f / 16.0f);
  g_lpg_mv = (int32_t)s_lpgMv;
}

// LPG valve input: hysteresis + ~300 ms debounce, then pick the active map.
static void lpgUpdate() {
  if (s_lpgMv > LPG_ON_MV) { if (s_lpgCnt < 6) s_lpgCnt++; }
  else if (s_lpgMv < LPG_OFF_MV) { if (s_lpgCnt > 0) s_lpgCnt--; }
  if (s_lpgCnt >= 6) s_lpgIn = 1; else if (s_lpgCnt <= 0) s_lpgIn = 0;
  g_lpg_in = s_lpgIn;
  g_lpg_active = g_lpg_override == 0 ? s_lpgIn : (g_lpg_override == 2);
}

// Idle: nudge ignition timing to hold the target rpm. PI on rpm error, only
// while the throttle is closed (low MAP) and the engine is at idle speed.
static void idleControl() {
  const IgnParams& p = g_params;
  uint32_t rpm = g_state.have_period ? g_state.rpm : 0;
  bool active = p.idle_enable && !g_lock_en && !g_sim_on && g_state.have_period && !g_state.cranking &&
                rpm > p.crank_rpm + p.crank_rpm / 5 && rpm < (uint32_t)p.idle_target_rpm + 500 &&
                s_mapKpa < p.idle_map_max_kpa;
  const float dt = 0.05f;
  if (active) {
    float err = (float)p.idle_target_rpm - (float)rpm;           // + = too slow -> more advance
    if (fabsf(err) < 15) err = 0;                                 // deadband
    s_idleInt += p.idle_ki * (err / 100.0f) * dt;
    s_idleInt = constrain(s_idleInt, -(float)p.idle_trim_max_x10, (float)p.idle_trim_max_x10);
    float trim = p.idle_kp * (err / 100.0f) + s_idleInt;
    g_idle_trim_x10 = (int32_t)constrain(trim, -(float)p.idle_trim_max_x10, (float)p.idle_trim_max_x10);
  } else {
    s_idleInt *= 0.93f;                                           // bleed off when leaving idle
    g_idle_trim_x10 = (int32_t)s_idleInt;
  }
}

static void ledUpdate() {
  bool on;
  uint32_t ms = millis();
  if (!g_state.have_edge && !g_sim_on) on = (ms % 2000) < 80;              // stopped: short blip
  else if (g_state.cranking && !g_sim_on) on = (ms % 200) < 100;           // cranking: fast blink
  else on = true;                                                          // running: solid
  if (g_output_enabled == 0) on = (ms % 100) < 50;                         // OTA
  digitalWrite(PIN_LED, (on ^ LED_ACTIVE_LOW) ? HIGH : LOW);
}

void setup() {
  // Coil off before anything else. (The external pulldown holds the gate low
  // while the chip is in reset/boot; this keeps it low from here on.)
  pinMode(PIN_COIL, OUTPUT);
  digitalWrite(PIN_COIL, LOW);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_POINTS, INPUT);
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MAP, ADC_11db);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
  analogSetPinAttenuation(PIN_LPG, ADC_11db);

  storeDefaults();
  bool loaded = storeLoad();
  sanitize(g_params, g_tables);
  ign_reset(&g_state); ign_reset(&g_stateO);
  g_state.last_adv_x10 = g_stateO.last_adv_x10 = g_params.crank_adv_x10;
  Serial.printf("\nMoskvich ECU fw %s, settings %s\n", FIRMWARE_VERSION, loaded ? "loaded" : "DEFAULT");

  for (int i = 0; i < 40; i++) { sampleAnalog(); delay(2); }   // let the filters settle before first spark
  ecuEngineBegin();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, 3);
  WiFi.setTxPower(WIFI_TX_POWER);
  MDNS.begin("moskvich-ecu");
  webBegin();

  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.setHostname("moskvich-ecu");
  ArduinoOTA.onStart([]() { ecuOutputOff(); });
  ArduinoOTA.begin();
  Serial.printf("AP \"%s\" up, http://%s/\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

void loop() {
  uint32_t now = micros();
  if (now - s_lastAdc >= 1000) { s_lastAdc = now; sampleAnalog(); }

  uint32_t ms = millis();
  if (ms - s_lastCtl >= 50) {
    s_lastCtl = ms;
    ign_stall_check(&g_state, (uint32_t)esp_timer_get_time());
    ign_stall_check(&g_stateO, (uint32_t)esp_timer_get_time());
    idleControl();
    lpgUpdate();
    bool running = engineRunning();
    if (s_wasRunning && !running && g_dirty) {                         // auto-save tuning edits when the engine stops
      if (storeSave()) Serial.println("auto-saved");
    }
    s_wasRunning = running;
    ledUpdate();
  }

  server.handleClient();
  dns.processNextRequest();
  ArduinoOTA.handle();
}
