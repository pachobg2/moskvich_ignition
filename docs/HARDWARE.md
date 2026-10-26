# Hardware, install and tuning

Values below are a **starting design**, not a tested board — check them against
your coil, ballast resistor and sensor datasheets before building. (The ESP32-C3
tolerates 3.3 V only on every pin.)

## Connections to the car (4 wires + 1 hose)

| ECU terminal | Goes to | Notes |
|---|---|---|
| **+12 V** | Ignition-switched 12 V | Tap the feed to the coil **before** the ballast resistor (the voltage after it sags every time the coil charges). Fuse 5 A. |
| **GND** | Chassis / battery negative | Short, thick, to the same point as the coil's return. |
| **COIL −** | Coil negative terminal | Replaces the wire that used to run to the distributor. |
| **POINTS** | The wire that ran from coil (−) to the distributor | Now a quiet signal line. |
| **MAP hose** | Intake manifold vacuum (below the throttle) | Nipple on the ECU; keep the hose short, rising toward the manifold so fuel/condensation drains out. |

Leave the coil's (+) wiring and the ballast resistor as they are.

## Circuit blocks

1. **Power input** — reverse-polarity protection (P-MOSFET or Schottky), TVS for
   load-dump (SMBJ24A-class), LC filter, then a wide-input buck to 5 V (≥ 1 A,
   e.g. a 40 V automotive buck). 5 V feeds the MAP sensor and the C3 board's
   5 V pin (on-board LDO makes 3.3 V). Bulk 100 µF + 100 nF at the sensor.
2. **Points input** — pull-up from the protected 12 V through **1 kΩ** to the
   POINTS terminal (≈ 12 mA wetting current keeps the contacts clean). From
   the terminal: 10 kΩ → base of a BC847 NPN (base–emitter 22 kΩ), collector
   pulled up to 3.3 V by 4.7 kΩ and wired to GPIO4 through a 1 kΩ plus
   10 nF to ground. Points **closed** → pin high; points **open** → pin low
   (the *falling* edge is the trigger — matches the default setting).
   **Remove the distributor condenser**: with no coil current through the points
   it is unnecessary and would slow the edge (RC with the pull-up shifts the
   timing by 100–200 µs ≈ several degrees at high rpm).
3. **Coil driver** — an ignition IGBT with internal clamp, e.g.
   **ISL9V3040 / ISL9V5036** (≈ 400 V clamp, logic-level gate) or a
   Darlington ignition driver with proper drive. GPIO5 → 100 Ω → gate, **10 kΩ
   gate-to-emitter pulldown (mandatory: keeps the coil off while the chip
   boots)**. Collector → COIL −, emitter → GND. Heat-sink it (mount to the
   case), place a 100 nF + TVS near the collector. If your IGBT's V<sub>CE(sat)</sub>
   is poor at 3.3 V gate drive, add a 5 V buffer (e.g. MCP1407 / a small NPN+PNP stage).
4. **Stuck-on guard (recommended for the first PCB)** — the firmware limits coil
   on-time to 9 ms, but a hung CPU cannot. Add a retriggerable one-shot (74HC123
   or a 555 monostable, ~12 ms) in the gate path, or AC-couple the gate through a
   capacitor with a bleed resistor, so the coil can never stay charged. Without
   it, a firmware crash with GPIO5 held high will cook the coil.
5. **MAP sensor** — NXP MPXH6115A6U / MPX4115 family on the 5 V rail:
   `Vout = Vs·(0.009·kPa − 0.095)` → 4.0 V at 100 kPa, up to 4.8 V. Divider
   **10 kΩ / 15 kΩ** (ratio 0.60 → ≤ 2.9 V) to GPIO0 with 100 nF. The ratio and
   formula constants live in `config.h`. A tiny pneumatic restriction (a pin-hole
   in the hose barb) calms intake pulsation; the firmware also filters.
6. **Battery sense** — 100 kΩ / 22 kΩ divider + 100 nF from the protected 12 V
   to GPIO1 (ratio 0.1803 → 16 V reads 2.9 V). Used for dwell compensation.
7. **Status LED** — on-board (GPIO8): blip = stopped, fast blink = cranking,
   solid = running.

Pins in `config.h`: MAP 0, VBAT 1, POINTS 4, COIL 5, LED 8 (ADC1 pins only, avoiding
GPIO2/8/9 strapping for anything driven at boot — GPIO8 is just the LED).

## Preparing the distributor (one-time)

1. Remove the cap, rotor, points. Note the **centrifugal advance**: block the
   weights in the **fully advanced** position (e.g. tack/solder the weights
   to the plate, or fit a stop). Everything the ECU will use is *retard* from
   this position.
2. **Disconnect the vacuum capsule**; plug the carburettor/manifold vacuum port.
3. Remove the condenser. Keep the points (or fit a cleaned set), gap as per
   the workshop manual. They now switch only ~12 mA.
4. Refit, rotate the distributor so that, on the timing mark, the points
   **open ≈ 40° BTDC with the advance locked** (trigger angle). Anything
   between ~35° and 45° works; the exact value is calibrated next.

The default trigger angle (40°) is deliberately above the highest default map
value (30°) so there is always room to retard.

## Calibrating the trigger angle (every install)

1. Connect a timing light to cylinder 1; engine warm, idling.
2. **Tools → Timing light**: lock at **10°** → *Lock*. (Spark = `trigger − 10°` after the points open.)
3. Read the true advance on the crank marks. If it shows *X°* instead of 10°:
   `new trigger angle = old trigger angle + (X − 10)`. Enter it in **Setup →
   Trigger angle**, Apply, re-check. Repeat at a second lock value (e.g. 25°) and
   at ~3000 rpm — a **drift with rpm** means output latency or a slow input edge
   (see `latency_us`, and the condenser/pull-up above).
4. *Unlock* before driving. The lock also disappears on reset.

## First start checklist

1. **Bench first**: coil disconnected from the engine, ECU powered from a bench
   supply, scope/test lamp on COIL −. *Tools → Bench simulator* at 800 / 3000 / 6000 rpm:
   confirm dwell, one spark per trigger, no output without trigger.
2. Check coil-driver temperature after 5 minutes of simulated 6000 rpm.
3. Confirm default map is conservative for your fuel; reduce WOT values if
   the engine pinks. Defaults assume a stock, carburetted engine — they are a
   starting point, not a tune.
4. Set **Setup → MAP calibration** with the engine off (atmospheric).
5. Start; verify idle with the timing light (lock mode off): the real advance
   should follow the Live tab's "Spark advance".

## Tuning notes

* Work from idle/cruise first (listen for knock under load before leaning on it).
* Idle: adjust the carb idle screw a little *below* the target and let the
  ignition trim bring it up — it is the quick, silent half of idle control.
* Dwell: with a current probe, adjust the dwell/voltage rows so the coil reaches
  its rated current at 12 V and the driver stays cool. At very high rpm the
  firmware shortens dwell automatically to keep a 1.2 ms coil-off gap for the
  spark to burn (see *Dwell limited* in Tools → Diagnostics).
* Never exceed the trigger angle: the editor and firmware clamp it, but an
  accurate trigger angle matters more than any map value.
