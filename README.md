# moskvich_ignition — programmable ignition ECU for the Moskvich 412 / 2140

ESP32-C3 replaces the points-and-coil ignition's *switching* job while keeping
the original distributor. The points become a crank-position sensor, the ECU
fires the coil through an IGBT at a timing it looks up from an **rpm × MAP
map**, controls dwell from battery voltage, trims idle speed with spark
timing, and limits revs. Everything is tuned from a phone over the ECU's own
WiFi network — no app to install, no router needed.

> **Experimental, safety-relevant.** Wrong spark timing can wreck an engine,
> and a dead ECU means a dead engine (see *Limitations*). Bench-test with the
> simulator, verify timing with a timing light before every first start, and
> start from the conservative default map.

## How it works

```
 12 V switched ──┬── protection + buck ──► ESP32-C3 ──► gate ──► IGBT ──► coil (−)
                 │                           ▲  ▲
 distributor ────┼── pull-up + NPN ──────────┘  │ ADC
   (points)      │                              │
 MAP hose ───────┴── MPX4115-class sensor ──────┘
```

* **Points = sensor only.** They no longer carry coil current (no arcing, no
  condenser needed). The instant they *open* marks a fixed crank angle,
  `trigger angle` (≈ 40° BTDC).
* **The distributor's mechanical advance is locked at that angle** (weights
  blocked, vacuum capsule disconnected). The ECU can only *delay* the spark
  relative to the trigger, so any advance up to the trigger angle is
  available:
  `delay = (trigger − target advance) / 180° × time since last trigger`.
* **Dwell** is ECU-controlled and voltage-compensated: at idle/cranking the
  coil is switched on `delay − dwell` after the trigger (exact dwell); at
  higher rpm it is scheduled one cycle ahead from the measured period.
* **Timing map**: 16 rpm × 16 MAP bins, bilinear interpolation, ± idle trim,
  rev-limit retard, slew limit per spark, min/max clamps.
* **LPG**: a 12 V signal from the LPG valve (GPIO3 via divider) switches the
  timing. Two modes (Setup → LPG): **static offset** (petrol map + N degrees,
  default +3°) or a **separate LPG map** (Map tab → LPG map, with *Copy petrol → LPG*
  and an optional offset). The change is slewed in at the normal per-spark limit,
  never stepped. A *Force petrol / Force LPG* override in Setup helps testing without
  the valve connected.
* **Cranking** uses a fixed retarded advance below `crank rpm`.
* **Rev limiter**: retard ramp from the soft limit, spark cut above the hard limit.
* **Idle control**: PI loop that nudges timing to hold the target idle rpm
  while the throttle is closed (MAP below a threshold).
* **Ignition ISRs run at interrupt priority 3**, above WiFi, so tuning on a
  phone does not move the spark. Flash writes (saving, OTA) *do* stall them,
  so the ECU **refuses to write flash while the engine turns** — edits apply
  live and are saved when you press Save with the engine stopped, or
  automatically when it stops.
* Hardware safeguards in firmware: coil-on watchdog (9 ms) if no spark was
  scheduled, no spark without a valid period measurement, bounce rejection,
  all inputs from the web app clamped by `sanitize()`.

## Files

| File | Purpose |
|---|---|
| `moskvich_ignition.ino` | setup/loop: ADC sampling, idle PI, LED, WiFi AP, OTA |
| `ignition_core.h` | **all ignition math & scheduling** — pure C++, no Arduino; tested on a PC |
| `ecu_engine.h` | ESP32-C3 layer: GPIO ISR, two one-shot hardware timers, bench simulator |
| `ecu_store.h` | NVS persistence (checksummed) and parameter `sanitize()` |
| `ecu_web.h` | web server + JSON API |
| `web/index.html` | the tuning app (single file, no dependencies; add `?mock` to try it on a PC) |
| `web_ui.h` | generated from `web/index.html` by `python tools/embed_ui.py` |
| `test/test_core.cpp` | engine-model simulation of the real ISR logic |
| `config.h.example` | copy to `config.h`: AP/OTA passwords, pins, sensor constants (gitignored) |
| `docs/HARDWARE.md` | circuit, parts list, wiring, install & tuning procedure |

## Build

Arduino IDE / arduino-cli, **esp32 core 3.x**, board *ESP32C3 Dev Module*
(*USB CDC On Boot: Enabled* for the SuperMini), library **ArduinoJson 7**.

```bash
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc .
```

Copy `config.h.example` → `config.h` and set the passwords. After editing
`web/index.html` run `python tools/embed_ui.py`.

Host tests (any C++17 compiler):

```bash
c++ -O2 -std=c++17 -I. test/test_core.cpp -o test/test_core && ./test/test_core
```

## Using it

1. Join WiFi **Moskvich-ECU** (password from `config.h`); the app opens as a
   captive portal, or browse to `http://192.168.4.1`.
2. **Live** — rpm, advance, MAP, battery, dwell, 60 s chart.
3. **Map** — petrol/LPG map toggle, tap cells, ±0.5/±1, Set, Smooth, Range select, Undo, editable axes.
   The cells around the current operating point glow while the engine runs.
4. **Setup** — trigger angle, cranking, idle, limits, dwell table, MAP sensor two-point calibration.
5. **Tools** — timing-light lock, bench simulator, JSON backup/restore, reset, firmware update.

Install, wiring and the timing-light calibration procedure are in
[docs/HARDWARE.md](docs/HARDWARE.md).

## Limitations / ideas

* **No limp-home.** With advance locked at 40°, running the stock points
  circuit would kick back on starting, so there is no relay bypass. Carry a
  spare ECU or the original distributor internals.
* No cam/phase signal: cylinder-to-cylinder cam-lobe and bushing wear shows up as
  a few degrees of scatter that the ECU cannot correct. A Hall-effect or
  optical distributor conversion would use the same input (set the edge).
* No knock sensor, no coolant/air temperature correction (spare ADC pins exist).
* ISR jitter vs. WiFi has been reasoned about, not scope-measured: verify with a
  scope/timing light at 6000 rpm on the bench simulator and adjust `latency_us`.
* Idea: log a short ring buffer of rpm/MAP/advance for download after a drive.

## Version History

| Version | Date | Changes |
|---|---|---|
| 1.0.0 | 2026-10-26 | Initial release: points-trigger delay-only timing, 16×16 rpm×MAP map, voltage-compensated dwell, cranking/idle/rev-limit logic, WiFi AP web tuner (live data, map editor, setup, timing-light lock, bench simulator, backup, OTA), host-side engine-model tests. |
| 1.1.0 | 2026-10-27 | MAP sensor two-point calibration in Setup (any linear 1-bar analog sensor, e.g. GM/Lada/Bosch; capture-from-live-reading buttons); `MAP_SLOPE`/`MAP_OFFSET`/`MAP_SUPPLY_MV` removed from `config.h`. Stored settings from 1.0.0 are discarded on first boot (defaults load). |
| 1.2.0 | 2026-10-27 | LPG timing: 12 V valve-signal input (GPIO3, hysteresis + debounce), static-offset mode or separate LPG map, copy petrol → LPG (+offset), manual override, LPG badge in the UI. Settings from 1.1.0 are discarded on first boot (defaults load). |
