#pragma once
// ESP32-C3 hardware layer for ignition_core.h: points-edge ISR, the two
// one-shot timers that switch the coil, and a bench simulator.
//
// Timing chain:  points open --GPIO ISR--> ign_on_open() --> arm timers
//                spark timer  -> coil OFF (spark)
//                coil  timer  -> coil ON  (dwell start)
// Both timers and the GPIO interrupt run at priority level 3 so WiFi (level 1)
// cannot delay them. Flash writes (NVS/OTA) disable the cache and DO delay
// these ISRs -- the firmware refuses to write flash while the engine runs.

#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "soc/gpio_struct.h"

#define IGN_ATTR IRAM_ATTR
#include "ignition_core.h"

// ---- shared state (written by web/loop, read by ISRs) -------------------
IgnParams g_params;
IgnTables g_tables;
IgnState  g_state;     // reference edge stream (open edge, or close edge in trigger_ref=1 mode)
IgnState  g_stateO;    // open-edge stream: drives the spark while cranking in close-edge mode
volatile int32_t g_map_kpa_x10 = 1000;     // filtered MAP, kPa x10
volatile int32_t g_vbat_mv     = 12000;
volatile int32_t g_idle_trim_x10 = 0;
volatile uint8_t g_lock_en  = 0;           // timing-light mode
volatile int16_t g_lock_adv_x10 = 100;
volatile uint8_t g_lpg_active = 0;         // map selection (valve input or override)
volatile uint8_t g_sim_on   = 0;           // bench simulator running (real GPIO ignored)
volatile uint32_t g_sim_rpm = 0;
volatile uint32_t g_spark_count = 0;
volatile uint32_t g_failsafe_count = 0;
volatile uint8_t  g_output_enabled = 1;    // cleared during OTA

static gptimer_handle_t s_tSpark = nullptr, s_tCoil = nullptr;
static volatile uint8_t s_sparkArmed = 0;
static esp_timer_handle_t s_simTimer = nullptr, s_simTimer2 = nullptr;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static inline void IRAM_ATTR coilHigh() { GPIO.out_w1ts.val = (1u << PIN_COIL); }
static inline void IRAM_ATTR coilLow()  { GPIO.out_w1tc.val = (1u << PIN_COIL); }

static void IRAM_ATTR fireSpark() {
  coilLow();
  if (g_state.coil_on && !s_sparkArmed) g_failsafe_count++;
  g_state.coil_on = 0;
  s_sparkArmed = 0;
  g_spark_count++;
}

static void IRAM_ATTR fireCoil() {
  if (g_state.coil_on || !g_output_enabled) return;
  coilHigh();
  g_state.coil_on = 1;
  if (!s_sparkArmed) {                 // nothing scheduled to turn it off: arm the watchdog
    gptimer_alarm_config_t c = {};
    c.alarm_count = IGN_FAILSAFE_US;
    gptimer_set_raw_count(s_tSpark, 0);
    gptimer_set_alarm_action(s_tSpark, &c);
  }
}

static bool IRAM_ATTR onSparkTimer(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*) {
  fireSpark(); return false;
}
static bool IRAM_ATTR onCoilTimer(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*) {
  fireCoil(); return false;
}

// Arm a one-shot `delay_us` from the edge, compensating for ISR time already spent.
static inline void IRAM_ATTR armOneShot(gptimer_handle_t t, uint32_t delay_us, uint32_t spent_us, bool spark) {
  uint32_t d = delay_us > spent_us ? delay_us - spent_us : 0;
  if (d < 8) { if (spark) fireSpark(); else fireCoil(); return; }
  gptimer_alarm_config_t c = {};
  c.alarm_count = d;
  gptimer_set_raw_count(t, 0);
  gptimer_set_alarm_action(t, &c);
}

// Shared by the real GPIO ISR and the bench simulator.
//   trigger_ref = 0: only the OPEN edge arrives (interrupt armed on that edge).
//   trigger_ref = 1: both edges arrive. The CLOSE edge (~120 deg) drives the spark while
//   running; the OPEN edge (factory static timing, ~10 deg) drives it while cranking,
//   because a ~120 deg prediction is far too coarse at cranking-speed variation.
static void IRAM_ATTR handleEdge(uint32_t now, bool isOpen) {
  const IgnParams* p = &g_params;
  IgnInputs in;
  in.map_kpa = (g_map_kpa_x10 + 5) / 10;
  in.vbat_mv = g_vbat_mv;
  in.idle_trim_x10 = g_idle_trim_x10;
  in.lpg = g_lpg_active;
  in.lock_en = g_lock_en;
  in.lock_adv_x10 = g_lock_adv_x10;
  IgnActions a;
  uint8_t wasCranking = g_state.cranking;
  if (!p->trigger_ref) {
    ign_on_open(&g_state, p, &g_tables, &in, now, &a, p->trigger_adv_x10, 0);
  } else if (isOpen) {
    uint8_t active = g_state.cranking;
    g_stateO.coil_on = g_state.coil_on;
    ign_on_open(&g_stateO, p, &g_tables, &in, now, &a, p->open_adv_x10, !active);
    if (active) { g_state.adv_x10 = g_stateO.adv_x10; g_state.dwell_us = g_stateO.dwell_us; g_state.cut = 0; }
  } else {
    ign_on_open(&g_state, p, &g_tables, &in, now, &a, p->trigger_adv_x10, g_state.cranking);
  }
  if (g_state.cranking != wasCranking)          // stream handover: drop the other stream's pre-scheduled coil-on
    gptimer_set_alarm_action(s_tCoil, nullptr);
  if (a.rejected || !g_output_enabled) return;
  uint32_t spent = (uint32_t)esp_timer_get_time() - now;
  if (a.coil_now) { coilHigh(); g_state.coil_on = 1; }
  if (a.spark_valid) { s_sparkArmed = 1; armOneShot(s_tSpark, a.spark_in_us, spent, true); }
  if (a.coil_valid)  armOneShot(s_tCoil, a.coil_in_us, spent, false);
}

static void IRAM_ATTR onPointsEdge(void*) {
  uint32_t now = (uint32_t)esp_timer_get_time();
  if (g_sim_on) return;
  // Level after the edge tells which edge it was. In open-edge mode require it to be the
  // "opened" level so a sub-microsecond glitch cannot fire a spark.
  uint32_t lvl = (GPIO.in.val >> PIN_POINTS) & 1u;
  bool isOpen = (lvl == (g_params.trigger_edge ? 1u : 0u));
  if (!g_params.trigger_ref && !isOpen) return;
  handleEdge(now, isOpen);
}

// ---- bench simulator ----------------------------------------------------
// Emits trigger events internally at a chosen rpm so spark timing can be
// checked on the bench (scope / timing light on the coil output) without an
// engine. ~tens of microseconds of jitter -- fine for a sanity check only.
static void simTick(void*) {
  uint32_t rpm = g_sim_rpm;
  if (!g_sim_on || rpm < 50) return;
  uint32_t period = (uint32_t)(120000000ULL / ((uint64_t)rpm * g_params.cylinders));
  esp_timer_start_once(s_simTimer, period);
  if (g_params.trigger_ref) esp_timer_start_once(s_simTimer2, period * 39 / 100);   // close edge ~70 deg after open
  portENTER_CRITICAL(&s_mux);
  handleEdge((uint32_t)esp_timer_get_time(), true);
  portEXIT_CRITICAL(&s_mux);
}
static void simClose(void*) {
  if (!g_sim_on) return;
  portENTER_CRITICAL(&s_mux);
  handleEdge((uint32_t)esp_timer_get_time(), false);
  portEXIT_CRITICAL(&s_mux);
}

static void simSet(uint32_t rpm) {
  if (rpm >= 50) {
    g_sim_rpm = rpm;
    if (!g_sim_on) {
      g_sim_on = 1;
      esp_timer_start_once(s_simTimer, 1000);
    }
  } else {
    g_sim_on = 0; g_sim_rpm = 0;
    esp_timer_stop(s_simTimer);
    esp_timer_stop(s_simTimer2);
  }
}

// ---- setup / control ------------------------------------------------------
static void ecuOutputOff() {
  g_output_enabled = 0;
  coilLow();
  g_state.coil_on = 0;
  if (s_tSpark) gptimer_set_alarm_action(s_tSpark, nullptr);
  if (s_tCoil)  gptimer_set_alarm_action(s_tCoil, nullptr);
  s_sparkArmed = 0;
}

static gpio_int_type_t ecuIntrType() {
  if (g_params.trigger_ref) return GPIO_INTR_ANYEDGE;
  return g_params.trigger_edge ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
}

static void ecuApplyEdge() {
  gpio_set_intr_type((gpio_num_t)PIN_POINTS, ecuIntrType());
}

static void ecuEngineBegin() {
  gpio_config_t out = {};
  out.pin_bit_mask = 1ULL << PIN_COIL;
  out.mode = GPIO_MODE_OUTPUT;
  gpio_config(&out);
  coilLow();

  gptimer_config_t tc = {};
  tc.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  tc.direction = GPTIMER_COUNT_UP;
  tc.resolution_hz = 1000000;
  tc.intr_priority = 3;
  gptimer_event_callbacks_t cb = {};
  gptimer_new_timer(&tc, &s_tSpark);
  cb.on_alarm = onSparkTimer;
  gptimer_register_event_callbacks(s_tSpark, &cb, nullptr);
  gptimer_enable(s_tSpark); gptimer_start(s_tSpark);
  gptimer_new_timer(&tc, &s_tCoil);
  cb.on_alarm = onCoilTimer;
  gptimer_register_event_callbacks(s_tCoil, &cb, nullptr);
  gptimer_enable(s_tCoil); gptimer_start(s_tCoil);

  esp_timer_create_args_t sa = {};
  sa.callback = simTick;
  sa.name = "sim";
  esp_timer_create(&sa, &s_simTimer);
  sa.callback = simClose;
  sa.name = "simc";
  esp_timer_create(&sa, &s_simTimer2);

  // Points input: GPIO interrupt service at level 3 (must be installed before
  // Arduino's attachInterrupt would install it at the default level).
  gpio_config_t in = {};
  in.pin_bit_mask = 1ULL << PIN_POINTS;
  in.mode = GPIO_MODE_INPUT;
  in.pull_up_en = GPIO_PULLUP_DISABLE;     // external pull-up/inverter stage drives this pin
  in.pull_down_en = GPIO_PULLDOWN_DISABLE;
  in.intr_type = ecuIntrType();
  gpio_config(&in);
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
  gpio_isr_handler_add((gpio_num_t)PIN_POINTS, onPointsEdge, nullptr);
}
