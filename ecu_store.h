#pragma once
// Persistence of params + tables in NVS, with size/magic validation so a
// firmware that changes the structs falls back to defaults instead of
// loading garbage into the ignition path.
#include <Preferences.h>

struct StoreHdr { uint32_t magic; uint16_t ver; uint16_t sz_params; uint16_t sz_tables; uint16_t crc; };
static const uint32_t STORE_MAGIC = 0x4D534B34;   // "MSK4"
static const uint16_t STORE_VER = 4;

volatile bool g_dirty = false;

static uint16_t crc16(const uint8_t* d, size_t n) {
  uint16_t c = 0xFFFF;
  while (n--) { c ^= *d++; for (int i = 0; i < 8; i++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1; }
  return c;
}

static uint16_t storeCrc() {
  uint16_t c = crc16((const uint8_t*)&g_params, sizeof g_params);
  return crc16((const uint8_t*)&g_tables, sizeof g_tables) ^ c;
}

static bool engineRunning() { return g_state.have_edge || g_sim_on; }

// Clamp everything the web app (or a corrupt flash page) could set to values
// that cannot hurt the engine or the coil.
static void sanitize(IgnParams& p, IgnTables& t) {
  p.cylinders = (p.cylinders == 6 || p.cylinders == 8) ? p.cylinders : 4;
  p.trigger_edge = p.trigger_edge ? 1 : 0;
  p.trigger_ref = p.trigger_ref ? 1 : 0;
  p.latency_us = ign_clamp(p.latency_us, 0, 500);
  p.accel_comp_pct = p.accel_comp_pct > 100 ? 100 : p.accel_comp_pct;
  p.crank_rpm = ign_clamp(p.crank_rpm, 150, 800);
  p.min_adv_x10 = ign_clamp(p.min_adv_x10, -200, 200);
  // The spark must land before the NEXT reference edge (delay < 180 deg), so
  // trigger - min_adv must stay under ~170 deg.
  p.trigger_adv_x10 = ign_clamp(p.trigger_adv_x10, 100, 1700 + p.min_adv_x10 < 1600 ? 1700 + p.min_adv_x10 : 1600);
  p.max_adv_x10 = ign_clamp(p.max_adv_x10, p.min_adv_x10, p.trigger_adv_x10);
  p.crank_adv_x10 = ign_clamp(p.crank_adv_x10, p.min_adv_x10, ign_clamp(150, p.min_adv_x10, p.max_adv_x10));
  p.rev_hard_rpm = ign_clamp(p.rev_hard_rpm, 1500, 9000);
  p.rev_soft_rpm = ign_clamp(p.rev_soft_rpm, 1000, p.rev_hard_rpm);
  p.rev_retard_x10 = ign_clamp(p.rev_retard_x10, 0, 300);
  p.adv_slew_x10 = ign_clamp(p.adv_slew_x10, 5, 200);
  for (int i = 0; i < IGN_NDWELL; i++) {
    p.dwell_us[i] = ign_clamp(p.dwell_us[i], 800, 8000);
    if (i && p.dwell_mv[i] <= p.dwell_mv[i - 1]) p.dwell_mv[i] = p.dwell_mv[i - 1] + 500;
  }
  p.min_dwell_us = ign_clamp(p.min_dwell_us, 500, 4000);
  p.max_dwell_us = ign_clamp(p.max_dwell_us, p.min_dwell_us, 8000);
  p.idle_target_rpm = ign_clamp(p.idle_target_rpm, 500, 2000);
  p.idle_map_max_kpa = ign_clamp(p.idle_map_max_kpa, 20, 100);
  p.idle_kp = ign_clamp(p.idle_kp, 0, 100);
  p.idle_ki = ign_clamp(p.idle_ki, 0, 100);
  p.idle_trim_max_x10 = ign_clamp(p.idle_trim_max_x10, 0, 150);
  p.map_gain_x1000 = ign_clamp(p.map_gain_x1000, 800, 1200);
  p.map_offset_x10 = ign_clamp(p.map_offset_x10, -300, 300);
  p.open_adv_x10 = ign_clamp(p.open_adv_x10, 0, 450);
  p.lpg_mode = p.lpg_mode ? 1 : 0;
  p.lpg_offset_x10 = ign_clamp(p.lpg_offset_x10, -100, 150);
  if (p.map_cal_mv[1] < p.map_cal_mv[0] + 200 || p.map_cal_kpa_x10[1] < p.map_cal_kpa_x10[0] + 100 ||
      p.map_cal_mv[1] > 5000 || p.map_cal_kpa_x10[1] > 3000) {            // unusable pair: fall back to MPX4115
    p.map_cal_mv[0] = 425;  p.map_cal_kpa_x10[0] = 200;
    p.map_cal_mv[1] = 4025; p.map_cal_kpa_x10[1] = 1000;
  }

  for (int i = 0; i < IGN_NRPM; i++) {
    if (i && t.rpm_axis[i] <= t.rpm_axis[i - 1]) t.rpm_axis[i] = t.rpm_axis[i - 1] + 50;
    for (int j = 0; j < IGN_NMAP; j++) {
      t.adv_x10[i][j] = ign_clamp(t.adv_x10[i][j], p.min_adv_x10, p.trigger_adv_x10);
      t.adv_lpg_x10[i][j] = ign_clamp(t.adv_lpg_x10[i][j], p.min_adv_x10, p.trigger_adv_x10);
    }
  }
  for (int j = 0; j < IGN_NMAP; j++) {
    t.map_axis[j] = ign_clamp(t.map_axis[j], 10, 250);
    if (j && t.map_axis[j] <= t.map_axis[j - 1]) t.map_axis[j] = t.map_axis[j - 1] + 1;
  }
}

static void storeDefaults() {
  ign_default_params(&g_params);
  ign_default_tables(&g_tables);
}

static bool storeLoad() {
  Preferences pr;
  if (!pr.begin("ign", true)) return false;
  StoreHdr h = {};
  bool ok = pr.getBytes("hdr", &h, sizeof h) == sizeof h && h.magic == STORE_MAGIC && h.ver == STORE_VER &&
            h.sz_params == sizeof(IgnParams) && h.sz_tables == sizeof(IgnTables);
  IgnParams p; IgnTables t;
  ok = ok && pr.getBytes("params", &p, sizeof p) == sizeof p && pr.getBytes("tables", &t, sizeof t) == sizeof t;
  pr.end();
  if (!ok) return false;
  uint16_t c = crc16((const uint8_t*)&p, sizeof p);
  c = crc16((const uint8_t*)&t, sizeof t) ^ c;
  if (c != h.crc) return false;
  g_params = p; g_tables = t;
  return true;
}

// Writes flash -> disables the cache for a few ms -> stalls ignition ISRs.
// Refused while the engine is turning.
static bool storeSave() {
  if (engineRunning()) return false;
  sanitize(g_params, g_tables);
  StoreHdr h = { STORE_MAGIC, STORE_VER, (uint16_t)sizeof(IgnParams), (uint16_t)sizeof(IgnTables), storeCrc() };
  Preferences pr;
  if (!pr.begin("ign", false)) return false;
  bool ok = pr.putBytes("params", &g_params, sizeof g_params) == sizeof g_params &&
            pr.putBytes("tables", &g_tables, sizeof g_tables) == sizeof g_tables &&
            pr.putBytes("hdr", &h, sizeof h) == sizeof h;
  pr.end();
  if (ok) g_dirty = false;
  return ok;
}
