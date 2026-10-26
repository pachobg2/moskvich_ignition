#pragma once
// Pure ignition logic -- NO Arduino/ESP includes, so the exact code that runs
// in the ISRs is also compiled and tested on a PC (see test/test_core.cpp).
//
// Concept
// -------
// The distributor's points no longer switch the coil. They are only a crank
// position sensor: the point where they OPEN is a fixed crank angle,
// `trigger_adv` degrees BTDC (the distributor's mechanical advance is locked
// at that angle). The ECU can only DELAY the spark relative to that event, so
// every target advance <= trigger_adv:
//
//     spark_delay = (trigger_adv - target_adv) / 180deg * last_period
//
// (180deg = crank angle between sparks on a 4-cylinder; `period` is the time
// between the last two open events.)
//
// The coil is switched by an IGBT. Dwell is controlled by the ECU. Only the
// OPEN edge is used (the close edge carries no information we need):
//   * delay >= dwell (idle, cranking): coil-on is timed from this same edge,
//     `delay - dwell` later, so the coil charges for exactly `dwell`.
//   * delay <  dwell (higher rpm / small retard): the coil must already be
//     charging when the edge arrives, so it is scheduled one cycle ahead from
//     the measured period.
//
// Units: time in microseconds (uint32, wrap-safe via unsigned subtraction),
// angles in tenths of a degree (_x10), pressure in kPa, voltage in mV.

#include <stdint.h>

#ifndef IGN_ATTR
#define IGN_ATTR            // ESP build defines this as IRAM_ATTR before include
#endif

#define IGN_NRPM    16
#define IGN_NMAP    16
#define IGN_NDWELL  5

// Fixed timing constants (not user tunable -- protect the hardware)
#define IGN_MIN_OFF_US        1200UL     // min coil-off time between sparks (spark burn)
#define IGN_MIN_LOCKOUT_US    300UL      // open-edge bounce lockout floor
#define IGN_MAX_LOCKOUT_US    20000UL    // ... and ceiling
#define IGN_MAX_PERIOD_US     1000000UL  // slower than this = no valid period (~30 rpm)
#define IGN_STALL_US          1500000UL  // no open edge for this long = engine stopped
#define IGN_FAILSAFE_US       9000UL     // coil-on hard limit if no spark was ever scheduled

struct IgnParams {
  // Trigger geometry
  int16_t  trigger_adv_x10;       // crank deg BTDC at which the points open (locked distributor)
  int16_t  latency_us;            // fixed output latency to subtract from every delay
  uint8_t  trigger_edge;          // edge on the conditioned GPIO that means "points opened": 0=falling 1=rising
  uint8_t  cylinders;             // 4 (rpm and degrees-per-event derive from this)
  uint8_t  idle_enable;
  uint8_t  accel_comp_pct;        // period extrapolation, 0 = off
  // Cranking
  uint16_t crank_rpm;             // below this = cranking mode
  int16_t  crank_adv_x10;
  // Limits
  int16_t  min_adv_x10;
  int16_t  max_adv_x10;
  uint16_t rev_soft_rpm;          // retard starts
  uint16_t rev_hard_rpm;          // spark cut
  int16_t  rev_retard_x10;        // retard reached at hard limit
  int16_t  adv_slew_x10;          // max advance change per spark
  // Dwell vs battery voltage (mV ascending)
  uint16_t dwell_mv[IGN_NDWELL];
  uint16_t dwell_us[IGN_NDWELL];
  uint16_t min_dwell_us;
  uint16_t max_dwell_us;
  // Idle control (ignition-trim PI)
  uint16_t idle_target_rpm;
  uint8_t  idle_map_max_kpa;      // idle control only below this MAP (closed throttle)
  uint8_t  _pad0;
  int16_t  idle_kp;               // trim x10 deg per 100 rpm error
  int16_t  idle_ki;               // integral, same units per second
  int16_t  idle_trim_max_x10;
  // MAP calibration (applied to the sensor-formula result)
  int16_t  map_gain_x1000;        // 1000 = 1.000
  int16_t  map_offset_x10;        // kPa x10
  // MAP sensor two-point calibration (sensor output mV -> kPa x10), any linear 1-bar sensor
  uint16_t map_cal_mv[2];
  int16_t  map_cal_kpa_x10[2];
};

struct IgnTables {
  uint16_t rpm_axis[IGN_NRPM];
  uint16_t map_axis[IGN_NMAP];            // kPa
  int16_t  adv_x10[IGN_NRPM][IGN_NMAP];   // [rpm row][map col]
};

struct IgnInputs {                // sampled by the caller at the moment of the open edge
  int32_t map_kpa;
  int32_t vbat_mv;
  int32_t idle_trim_x10;
  uint8_t lock_en;                // timing-light mode: fixed advance
  int16_t lock_adv_x10;
};

struct IgnState {
  uint32_t last_open_us;
  uint32_t period_us;
  uint32_t prev_period_us;
  uint8_t  have_edge;
  uint8_t  have_period;
  uint8_t  have_prev_period;
  uint8_t  cranking;
  uint8_t  coil_on;               // maintained by whoever executes the actions
  uint8_t  cut;
  int16_t  last_adv_x10;
  int16_t  adv_x10;               // advance used for the most recent spark (reporting)
  uint32_t rpm;
  uint32_t dwell_us;              // dwell planned for the next spark (reporting)
  uint32_t open_count, bounce_count, late_count, cut_count, dwell_limited_count;
};

struct IgnActions {
  uint8_t  rejected;              // bounce, nothing to do
  uint8_t  coil_now;              // turn the coil on right now (late fallback)
  uint8_t  spark_valid;
  uint32_t spark_in_us;           // spark (coil off) this long after the edge
  uint8_t  coil_valid;
  uint32_t coil_in_us;            // (next) coil-on this long after the edge
};

// ---------------------------------------------------------------- helpers

IGN_ATTR static inline int32_t ign_clamp(int32_t v, int32_t lo, int32_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Index i (0..n-2) and 0..256 fraction such that axis[i] <= x <= axis[i+1].
// Tolerates unsorted/degenerate axes (they can be mid-edit from the web UI).
IGN_ATTR static inline void ign_axis_find(const uint16_t* ax, int n, int32_t x, int* idx, int32_t* frac) {
  if (x <= (int32_t)ax[0]) { *idx = 0; *frac = 0; return; }
  if (x >= (int32_t)ax[n - 1]) { *idx = n - 2; *frac = 256; return; }
  int i = 0;
  while (i < n - 2 && x >= (int32_t)ax[i + 1]) i++;
  int32_t span = (int32_t)ax[i + 1] - (int32_t)ax[i];
  *idx = i;
  *frac = span > 0 ? ign_clamp(((x - (int32_t)ax[i]) * 256) / span, 0, 256) : 0;
}

IGN_ATTR static inline int32_t ign_table_lookup(const IgnTables* t, int32_t rpm, int32_t map_kpa) {
  int r, c; int32_t fr, fc;
  ign_axis_find(t->rpm_axis, IGN_NRPM, rpm, &r, &fr);
  ign_axis_find(t->map_axis, IGN_NMAP, map_kpa, &c, &fc);
  int32_t v00 = t->adv_x10[r][c],     v01 = t->adv_x10[r][c + 1];
  int32_t v10 = t->adv_x10[r + 1][c], v11 = t->adv_x10[r + 1][c + 1];
  int32_t top = v00 * (256 - fc) + v01 * fc;
  int32_t bot = v10 * (256 - fc) + v11 * fc;
  int32_t v = top * (256 - fr) + bot * fr;               // x65536
  return v >= 0 ? (v + 32768) >> 16 : -((-v + 32768) >> 16);
}

IGN_ATTR static inline uint32_t ign_dwell_for_voltage(const IgnParams* p, int32_t mv) {
  int n = IGN_NDWELL;
  uint32_t d;
  if (mv <= (int32_t)p->dwell_mv[0]) d = p->dwell_us[0];
  else if (mv >= (int32_t)p->dwell_mv[n - 1]) d = p->dwell_us[n - 1];
  else {
    int i = 0;
    while (i < n - 2 && mv >= (int32_t)p->dwell_mv[i + 1]) i++;
    int32_t span = (int32_t)p->dwell_mv[i + 1] - (int32_t)p->dwell_mv[i];
    int32_t f = span > 0 ? ((mv - (int32_t)p->dwell_mv[i]) * 256) / span : 0;
    d = ((int32_t)p->dwell_us[i] * (256 - f) + (int32_t)p->dwell_us[i + 1] * f) >> 8;
  }
  return (uint32_t)ign_clamp((int32_t)d, p->min_dwell_us, p->max_dwell_us);
}

IGN_ATTR static inline uint32_t ign_rpm_from_period(const IgnParams* p, uint32_t period_us) {
  if (period_us == 0) return 0;
  return (uint32_t)(120000000ULL / ((uint64_t)period_us * p->cylinders));
}

// Final target advance for this spark (before slew limiting).
IGN_ATTR static inline int32_t ign_target_advance(const IgnParams* p, const IgnTables* t,
                                                  const IgnInputs* in, uint32_t rpm, uint8_t cranking) {
  int32_t adv;
  int32_t hi = p->max_adv_x10 < p->trigger_adv_x10 ? p->max_adv_x10 : p->trigger_adv_x10;
  if (in->lock_en) {
    adv = in->lock_adv_x10;
  } else if (cranking) {
    adv = p->crank_adv_x10;
  } else {
    adv = ign_table_lookup(t, (int32_t)rpm, in->map_kpa) + in->idle_trim_x10;
    if (rpm > p->rev_soft_rpm && p->rev_hard_rpm > p->rev_soft_rpm) {
      int32_t span = (int32_t)p->rev_hard_rpm - (int32_t)p->rev_soft_rpm;
      int32_t over = ign_clamp((int32_t)rpm - (int32_t)p->rev_soft_rpm, 0, span);
      adv -= (p->rev_retard_x10 * over) / span;
    }
  }
  return ign_clamp(adv, p->min_adv_x10, hi);
}

// Open edge -> spark delay (relative to the edge) for a target advance.
IGN_ATTR static inline uint32_t ign_spark_delay_us(const IgnParams* p, int32_t adv_x10, uint32_t period_us) {
  int32_t retard_x10 = (int32_t)p->trigger_adv_x10 - adv_x10;     // deg x10 AFTER the trigger
  if (retard_x10 < 0) retard_x10 = 0;
  int32_t deg_per_event_x10 = 7200 / p->cylinders;                // 1800 for a 4-cyl
  uint64_t d = ((uint64_t)retard_x10 * period_us) / (uint32_t)deg_per_event_x10;
  uint32_t lat = (uint32_t)(p->latency_us > 0 ? p->latency_us : 0);
  return d > lat ? (uint32_t)(d - lat) : 0;
}

// ------------------------------------------------------------ state machine

static inline void ign_reset(IgnState* s) {
  s->have_edge = s->have_period = s->have_prev_period = 0;
  s->cranking = 1; s->cut = 0; s->rpm = 0; s->period_us = s->prev_period_us = 0;
}

// Call from the main loop; clears speed history when the engine has stopped.
static inline void ign_stall_check(IgnState* s, uint32_t now_us) {
  if (s->have_edge && (uint32_t)(now_us - s->last_open_us) > IGN_STALL_US) ign_reset(s);
}

// The points just OPENED. Decide what to do. The caller executes the actions
// (arming the spark timer etc.) and keeps s->coil_on current.
IGN_ATTR static inline void ign_on_open(IgnState* s, const IgnParams* p, const IgnTables* t,
                                        const IgnInputs* in, uint32_t now, IgnActions* a) {
  a->rejected = a->coil_now = a->spark_valid = a->coil_valid = 0;
  a->spark_in_us = a->coil_in_us = 0;

  if (s->have_edge) {
    uint32_t dt = now - s->last_open_us;
    uint32_t lock = ign_clamp((int32_t)(s->period_us / 4), IGN_MIN_LOCKOUT_US, IGN_MAX_LOCKOUT_US);
    if (s->have_period && dt < lock) { s->bounce_count++; a->rejected = 1; return; }
    if (dt <= IGN_MAX_PERIOD_US) {
      s->have_prev_period = s->have_period;
      s->prev_period_us = s->period_us;
      s->period_us = dt;
      s->have_period = 1;
    } else {
      s->have_period = s->have_prev_period = 0;
    }
  }
  s->last_open_us = now;
  s->have_edge = 1;
  s->open_count++;
  if (!s->have_period) return;      // no speed info yet: cannot place a spark safely

  s->rpm = ign_rpm_from_period(p, s->period_us);

  // cranking <-> running with hysteresis
  if (s->cranking) { if (s->rpm > p->crank_rpm + p->crank_rpm / 10) s->cranking = 0; }
  else             { if (s->rpm < p->crank_rpm - p->crank_rpm / 5)  s->cranking = 1; }

  // period used for placing the spark (optionally extrapolated while accelerating)
  int32_t T = (int32_t)s->period_us;
  if (!s->cranking && s->have_prev_period && p->accel_comp_pct) {
    int32_t d = ((int32_t)s->period_us - (int32_t)s->prev_period_us) * p->accel_comp_pct / 100;
    T = ign_clamp(T + d, T - T / 4, T + T / 4);
  }

  int32_t target = ign_target_advance(p, t, in, s->rpm, s->cranking);
  int32_t adv = target;
  if (!s->cranking && !in->lock_en)
    adv = ign_clamp(target, s->last_adv_x10 - p->adv_slew_x10, s->last_adv_x10 + p->adv_slew_x10);
  s->last_adv_x10 = (int16_t)adv;
  s->adv_x10 = (int16_t)adv;

  s->cut = (!s->cranking && p->rev_hard_rpm && s->rpm > p->rev_hard_rpm);
  uint32_t dwell = ign_dwell_for_voltage(p, in->vbat_mv);
  s->dwell_us = dwell;
  uint32_t delay = ign_spark_delay_us(p, adv, (uint32_t)T);

  if (s->cut) {
    s->cut_count++;
    if (s->coil_on) { a->spark_valid = 1; a->spark_in_us = delay; }  // already charged: let it go
    return;
  }

  a->spark_valid = 1;
  a->spark_in_us = delay;
  if (!s->coil_on) {
    if (delay >= dwell) {                  // normal at idle/cranking: charge from this edge
      a->coil_valid = 1;
      a->coil_in_us = delay - dwell;
    } else {                               // charge didn't happen in time: late fallback
      s->late_count++;
      a->coil_now = 1;
      a->spark_in_us = delay > p->min_dwell_us ? delay : p->min_dwell_us;
    }
  }
  if (delay < dwell) {                     // pre-schedule the NEXT coil-on (predicted)
    uint32_t next_spark = (uint32_t)T + delay;                 // rel. to this edge, same advance assumed
    uint32_t earliest = a->spark_in_us + IGN_MIN_OFF_US;
    uint32_t on_at = next_spark > dwell ? next_spark - dwell : 0;
    if (on_at < earliest) { on_at = earliest; s->dwell_limited_count++; }
    a->coil_valid = 1;
    a->coil_in_us = on_at;
  }
}

// -------------------------------------------------------------- defaults

static inline void ign_default_params(IgnParams* p) {
  p->trigger_adv_x10 = 400;
  p->latency_us = 0;
  p->trigger_edge = 0;
  p->cylinders = 4;
  p->idle_enable = 1;
  p->accel_comp_pct = 0;
  p->crank_rpm = 350;
  p->crank_adv_x10 = 50;
  p->min_adv_x10 = -50;
  p->max_adv_x10 = 380;
  p->rev_soft_rpm = 5600;
  p->rev_hard_rpm = 6000;
  p->rev_retard_x10 = 100;
  p->adv_slew_x10 = 40;
  static const uint16_t mv[IGN_NDWELL] = { 8000, 10000, 12000, 14000, 16000 };
  static const uint16_t us[IGN_NDWELL] = { 5500, 4800, 4000, 3500, 3200 };
  for (int i = 0; i < IGN_NDWELL; i++) { p->dwell_mv[i] = mv[i]; p->dwell_us[i] = us[i]; }
  p->min_dwell_us = 1500;
  p->max_dwell_us = 7000;
  p->idle_target_rpm = 850;
  p->idle_map_max_kpa = 55;
  p->_pad0 = 0;
  p->idle_kp = 8;
  p->idle_ki = 6;
  p->idle_trim_max_x10 = 80;
  p->map_gain_x1000 = 1000;
  p->map_offset_x10 = 0;
  p->map_cal_mv[0] = 425;   p->map_cal_kpa_x10[0] = 200;    // MPX4115 @ 5 V: 20 kPa
  p->map_cal_mv[1] = 4025;  p->map_cal_kpa_x10[1] = 1000;   //                100 kPa
}

// Conservative starting map for a stock-ish carburetted 4-cylinder. NOT a
// tune -- a safe place to begin. Verify with a timing light and listen for
// knock before loading the engine.
static inline void ign_default_tables(IgnTables* t) {
  static const uint16_t rpm[IGN_NRPM] = { 500, 700, 900, 1100, 1400, 1800, 2200, 2600,
                                          3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500 };
  static const uint16_t map[IGN_NMAP] = { 20, 25, 30, 35, 40, 45, 50, 55,
                                          60, 65, 70, 80, 85, 90, 95, 100 };
  static const int base[IGN_NRPM]  = { 8, 10, 13, 16, 20, 24, 26, 28, 29, 30, 30, 30, 30, 30, 30, 30 };
  static const int wot[IGN_NRPM]   = { 10, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 6, 6, 6, 6 };   // reduction at 100 kPa
  for (int i = 0; i < IGN_NRPM; i++) t->rpm_axis[i] = rpm[i];
  for (int j = 0; j < IGN_NMAP; j++) t->map_axis[j] = map[j];
  for (int i = 0; i < IGN_NRPM; i++)
    for (int j = 0; j < IGN_NMAP; j++) {
      int load = map[j] > 35 ? map[j] - 35 : 0;                  // 0..65
      int v10 = base[i] * 10 - (wot[i] * 10 * load) / 65;
      if (v10 < 60) v10 = 60;
      t->adv_x10[i][j] = (int16_t)v10;
    }
}
