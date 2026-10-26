// Host-side tests for ignition_core.h: a crude engine model feeds the real
// ISR logic with points-open edges (plus injected bounce) and checks where the sparks land.
//
//   build: zig c++ -O1 -std=c++17 -I.. test_core.cpp -o test_core.exe   (or g++/clang++)
//   run:   ./test_core.exe
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <vector>
#include "ignition_core.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

// ------------------------------------------------------------------ lookup
static void test_lookup() {
  printf("lookup/dwell\n");
  IgnTables t; ign_default_tables(&t);
  for (int i = 0; i < IGN_NRPM; i++)
    for (int j = 0; j < IGN_NMAP; j++) CHECK(ign_table_lookup(&t, t.rpm_axis[i], t.map_axis[j]) == t.adv_x10[i][j],
                                              "grid point %d,%d", i, j);
  // midpoint of four equal corners
  t.adv_x10[3][4] = 100; t.adv_x10[3][5] = 200; t.adv_x10[4][4] = 300; t.adv_x10[4][5] = 400;
  int32_t rpm = (t.rpm_axis[3] + t.rpm_axis[4]) / 2, mp = (t.map_axis[4] + t.map_axis[5]) / 2;   // 1250, 42 (frac .4)
  int32_t v = ign_table_lookup(&t, rpm, mp);
  CHECK(v >= 238 && v <= 242, "bilinear centre %d (want 240)", v);
  CHECK(ign_table_lookup(&t, 0, 0) == t.adv_x10[0][0], "clamp low");
  CHECK(ign_table_lookup(&t, 99999, 999) == t.adv_x10[IGN_NRPM - 1][IGN_NMAP - 1], "clamp high");
  // degenerate (equal) axis values must not divide by zero
  t.rpm_axis[5] = t.rpm_axis[6]; ign_table_lookup(&t, t.rpm_axis[5], 50);

  IgnParams p; ign_default_params(&p);
  CHECK(ign_dwell_for_voltage(&p, 12000) == 4000, "dwell @12V");
  CHECK(ign_dwell_for_voltage(&p, 13000) == 3750, "dwell @13V interp (%u)", ign_dwell_for_voltage(&p, 13000));
  CHECK(ign_dwell_for_voltage(&p, 5000) == 5500, "dwell low clamp");
  CHECK(ign_dwell_for_voltage(&p, 20000) == 3200, "dwell high clamp");
  CHECK(ign_rpm_from_period(&p, 37500) == 800, "rpm from period (%u)", ign_rpm_from_period(&p, 37500));
  // 800 rpm: 37.5 ms per 180deg. 30deg retard = 1/6 of that = 6250 us
  CHECK(ign_spark_delay_us(&p, 100, 37500) == 6250, "delay (%u)", ign_spark_delay_us(&p, 100, 37500));
}

// -------------------------------------------------------------------- sim
struct Result { int sparks = 0, expected = 0; double max_err = 0, max_over = 0, mean_abs = 0; int bad_dwell = 0, no_charge = 0, failsafe = 0;
                uint32_t max_dwell = 0, min_dwell = 0xFFFFFFFF; };

struct SimOpts {
  std::function<double(double t_us)> rpm;        // crank speed profile
  double end_us = 3e6;
  double bounce_us = 0;                          // inject an open-edge bounce this long after each open
  double isr_latency_us = 0;                     // detection latency of edges
  double stop_from = -1, stop_to = -1;           // no events in this window (engine stalled)
  uint8_t accel_comp = 0;
  int32_t map_kpa = 40;
  int32_t vbat = 12500;
  bool lock = false; int16_t lock_adv = 100;
};

static Result run(const SimOpts& o, IgnParams p, IgnTables t, int skip_first = 3, IgnState* out_state = nullptr) {
  Result R;
  p.accel_comp_pct = o.accel_comp;
  IgnState s{}; ign_reset(&s);
  s.last_adv_x10 = p.crank_adv_x10;
  IgnInputs in{}; in.map_kpa = o.map_kpa; in.vbat_mv = o.vbat; in.lock_en = o.lock; in.lock_adv_x10 = o.lock_adv;

  const double trig = p.trigger_adv_x10 / 10.0;
  double theta = 0;                              // crank angle, deg
  long k = 0;                                    // next event index
  auto open_theta = [&](long kk) { return 180.0 * kk + 180.0 - trig; };
  auto tdc_theta  = [&](long kk) { return 180.0 * kk + 180.0; };
  // timers (absolute us, <0 = idle)
  double spark_at = -1, coil_at = -1; long spark_k = -1; double coil_on_since = 0;
  struct Pend { double at; int kind; long k; };   // delayed (latency) edge deliveries
  std::vector<Pend> pend;
  double sum_abs = 0;
  std::vector<double> bounce_at;

  auto fire_open = [&](double now_us, long kk) {
    IgnActions a;
    ign_on_open(&s, &p, &t, &in, (uint32_t)now_us, &a);
    if (a.rejected) return;
    if (a.coil_now) { s.coil_on = 1; coil_on_since = now_us; }
    if (a.spark_valid) { spark_at = now_us + a.spark_in_us; spark_k = kk; }
    if (a.coil_valid) coil_at = now_us + a.coil_in_us;
  };

  for (double now = 0; now < o.end_us; now += 1.0) {
    bool stalled = o.stop_from >= 0 && now >= o.stop_from && now < o.stop_to;
    double rpm = stalled ? 0 : o.rpm(now);
    theta += rpm * 360.0 / 60e6;

    if (!stalled) {
      if (theta >= open_theta(k)) {                 // points open
        long kk = k++;
        pend.push_back({now + o.isr_latency_us, 0, kk});
        if (o.bounce_us > 0) { pend.push_back({now + o.bounce_us + o.isr_latency_us, 2, kk}); pend.push_back({now + o.bounce_us * 1.5 + o.isr_latency_us, 0, kk}); }
      }
    } else {
      if (theta >= open_theta(k)) { k++; }          // engine "stopped" but keep angle bookkeeping sane
    }
    for (size_t i = 0; i < pend.size();) {
      if (pend[i].at <= now) {
        Pend e = pend[i]; pend.erase(pend.begin() + i);
        if (e.kind == 0) fire_open(now, e.k);   // kind 2 = bounce close edge: carries no information
      } else i++;
    }
    if (coil_at >= 0 && now >= coil_at) {
      coil_at = -1;
      if (!s.coil_on) { s.coil_on = 1; coil_on_since = now; if (spark_at < 0) { spark_at = now + IGN_FAILSAFE_US; spark_k = -1; } }
    }
    if (spark_at >= 0 && now >= spark_at) {
      spark_at = -1;
      if (s.coil_on) {
        s.coil_on = 0;
        uint32_t dw = (uint32_t)(now - coil_on_since);
        if (spark_k < 0) R.failsafe++;
        else {
          double adv = tdc_theta(spark_k) - theta;                 // deg BTDC actually achieved
          double want = s.adv_x10 / 10.0;
          double err = adv - want;
          if (k > skip_first && !stalled) {
            R.sparks++; sum_abs += fabs(err);
            if (err > R.max_over) R.max_over = err;               // positive = MORE advanced than intended
            if (fabs(err) > R.max_err) R.max_err = fabs(err);
            if (dw > R.max_dwell) R.max_dwell = dw;
            if (dw < R.min_dwell) R.min_dwell = dw;
          }
        }
      }
    }
    if (stalled && s.have_edge == 0) {}
    if ((long)now % 1000 == 0) ign_stall_check(&s, (uint32_t)now);
  }
  R.expected = (int)(k - skip_first);
  R.mean_abs = R.sparks ? sum_abs / R.sparks : 0;
  R.no_charge = (int)s.late_count;
  if (out_state) *out_state = s;
  return R;
}

static void report(const char* name, const Result& r, const IgnState* s = nullptr) {
  printf("  %-30s sparks %4d/%-4d  err mean %.2f max %.2f (adv-side %.2f) deg  dwell %u..%u us  late %d failsafe %d%s\n", name, r.sparks, r.expected,
         r.mean_abs, r.max_err, r.max_over, r.sparks ? r.min_dwell : 0, r.max_dwell, r.no_charge, r.failsafe, s ? "" : "");
}

int main() {
  test_lookup();
  IgnParams p; ign_default_params(&p);
  IgnTables t; ign_default_tables(&t);

  printf("steady state\n");
  for (int rpm : { 900, 1500, 3000, 5000, 5800 }) {
    SimOpts o; o.rpm = [=](double) { return (double)rpm; }; o.end_us = 2.5e6; o.map_kpa = 45;
    char nm[64]; snprintf(nm, sizeof nm, "%d rpm", rpm);
    IgnState s; Result r = run(o, p, t, 5, &s);
    report(nm, r);
    CHECK(r.sparks >= r.expected - 3, "%s: missed sparks %d/%d", nm, r.sparks, r.expected);
    CHECK(r.max_err < 0.2, "%s: spark angle error %.2f", nm, r.max_err);
    CHECK(r.max_dwell <= p.max_dwell_us + 100, "%s: dwell too long %u", nm, r.max_dwell);
    CHECK(r.min_dwell >= 1400, "%s: dwell too short %u", nm, r.min_dwell);
    CHECK(r.failsafe == 0, "%s: failsafe fired", nm);
  }

  printf("accel / decel\n");
  {
    SimOpts o; o.rpm = [](double tt) { return 1000 + (5500 - 1000) * (tt / 4e6); }; o.end_us = 4e6; o.map_kpa = 85;
    for (int comp : { 0, 100 }) {
      o.accel_comp = (uint8_t)comp;
      char nm[64]; snprintf(nm, sizeof nm, "1000->5500 in 4s (comp %d%%)", comp);
      Result r = run(o, p, t); report(nm, r);
      CHECK(r.max_err < 2.0, "%s: error %.2f", nm, r.max_err);
      CHECK(r.failsafe == 0, "%s: failsafe", nm);
    }
    SimOpts q; q.rpm = [](double tt) { return 3000 - 2200 * (tt / 1.0e6); }; q.end_us = 1.0e6;   // violent decel
    Result r = run(q, p, t); report("3000->800 in 1s", r);
    CHECK(r.max_err < 3.0, "decel error %.2f", r.max_err);
  }

  printf("cranking\n");
  {
    SimOpts o; o.end_us = 3e6;
    o.rpm = [](double tt) { static double th = 0; th += 0.0; return 150.0 * (1.0 + 0.15 * sin(tt / 1e6 * 2 * 3.14159 * 2.5)); };
    Result r = run(o, p, t); report("150 rpm +-15% wobble", r);
    CHECK(r.sparks >= r.expected - 3, "cranking missed sparks %d/%d", r.sparks, r.expected);
    CHECK(r.max_err < 5.0, "cranking error %.2f (crank adv %.1f)", r.max_err, p.crank_adv_x10 / 10.0);
    CHECK(r.failsafe == 0, "cranking failsafe");
  }

  printf("crank -> run handover\n");
  {
    SimOpts o; o.end_us = 4e6;
    o.rpm = [](double tt) { return tt < 1.5e6 ? 180.0 : (tt < 2.5e6 ? 180.0 + (900.0 - 180.0) * (tt - 1.5e6) / 1e6 : 900.0); };
    Result r = run(o, p, t); report("180 -> 900 rpm catch", r);
    CHECK(r.failsafe == 0, "handover failsafe fired");
    CHECK(r.max_over < 2.0, "handover over-advance %.2f", r.max_over);   // accelerating = late (safe) side
    CHECK(r.sparks >= r.expected - 4, "handover missed sparks %d/%d", r.sparks, r.expected);
  }

  printf("bounce rejection\n");
  {
    SimOpts o; o.rpm = [](double) { return 1200.0; }; o.end_us = 2e6; o.bounce_us = 150;
    Result r = run(o, p, t); report("150us open bounce @1200", r);
    CHECK(r.sparks >= r.expected - 3 && r.sparks <= r.expected, "bounce changed spark count %d/%d", r.sparks, r.expected);
    CHECK(r.max_err < 0.2, "bounce error %.2f", r.max_err);
  }

  printf("isr latency (15us)\n");
  {
    SimOpts o; o.rpm = [](double) { return 3000.0; }; o.end_us = 2e6; o.isr_latency_us = 15;
    Result r = run(o, p, t); report("3000 rpm +15us edge latency", r);
    CHECK(r.max_err < 0.7, "latency error %.2f", r.max_err);
  }

  printf("rev limiter\n");
  {
    SimOpts o; o.rpm = [](double tt) { return 5000 + 3000 * (tt / 3e6); }; o.end_us = 3e6;
    IgnState s; Result r = run(o, p, t, 3, &s); report("5000->8000 rpm", r);
    CHECK(s.cut_count > 20, "rev cut never engaged (%u)", s.cut_count);
    CHECK(r.sparks < r.expected, "spark cut did not reduce sparks");
  }

  printf("stall / restart\n");
  {
    SimOpts o; o.rpm = [](double) { return 1000.0; }; o.end_us = 5e6; o.stop_from = 1.5e6; o.stop_to = 3.0e6;
    IgnState s; Result r = run(o, p, t, 3, &s); report("1.5s stall then restart", r);
    CHECK(r.failsafe <= 1, "restart failsafe x%d", r.failsafe);   // one stray spark at the instant of stall is expected
    CHECK(s.open_count > 20, "did not recover after stall");
  }

  printf("lock (timing light) mode\n");
  {
    SimOpts o; o.rpm = [](double) { return 1000.0; }; o.end_us = 2e6; o.lock = true; o.lock_adv = 150;
    Result r = run(o, p, t); report("locked at 15.0 deg", r);
    CHECK(r.max_err < 0.2, "lock error %.2f", r.max_err);
  }

  printf("low battery (8V) dwell\n");
  {
    SimOpts o; o.rpm = [](double) { return 4000.0; }; o.end_us = 2e6; o.vbat = 8000;
    Result r = run(o, p, t); report("4000 rpm @8V", r);
    CHECK(r.failsafe == 0 && r.max_err < 0.2, "8V behaviour");
    CHECK(r.max_dwell <= p.max_dwell_us + 100, "8V dwell cap");
  }

  printf(g_fail ? "\n%d FAILURE(S)\n" : "\nall tests passed\n", g_fail);
  return g_fail ? 1 : 0;
}
