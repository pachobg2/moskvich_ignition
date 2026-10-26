# Hardware, install and tuning

Values below are a **starting design**, not a tested board — check them against
your coil, ballast resistor and sensor datasheets before building. (The ESP32-C3
tolerates 3.3 V only on every pin.) The board design is in `../hardware/`.

## Connections to the car (4 wires + 1 hose)

| ECU terminal | Goes to | Notes |
|---|---|---|
| **+12 V** | Ignition-switched 12 V | Tap the feed to the coil **before** the ballast resistor (the voltage after it sags every time the coil charges). Fuse 5 A. |
| **GND** | Chassis / battery negative | Short, thick, to the same point as the coil's return. |
| **COIL −** | Coil negative terminal | Replaces the wire that used to run to the distributor. |
| **POINTS** | The wire that ran from coil (−) to the distributor | Now a quiet signal line. |
| **LPG signal** (optional) | The 12 V wire that is live while the LPG valve/ECU is running on gas | High impedance sense only (see circuit block 8); no fuse needed beyond the wire's own. Share ground with the ECU. |
| **MAP hose** | Intake manifold vacuum (below the throttle) | Nipple on the ECU; keep the hose short, rising toward the manifold so fuel/condensation drains out. |

Leave the coil's (+) wiring and the ballast resistor as they are.

## Circuit blocks

The complete design — schematic, PCB (SMD, two-sided, 48 × 41.8 mm, ESP32-C3-Zero on sockets),
BOM and mounting details for the IGBT cut-out — is in **[`hardware/`](../hardware/README.md)**.
In short:

1. **Power** — 0.5 A SMD fuse, SS34 Schottky (reverse polarity), SMBJ24A TVS, 10 µF, then an
   **AP63205** SMD buck (3.8–32 V in, fixed 5 V / 2 A, 4.7 µH + 2 × 22 µF as per its
   datasheet). Coil current does **not** flow through any of this: only the ECU's ~0.2 A.
2. **Points input** — 1 kΩ pull-up from the protected 12 V gives ≈ 12 mA wetting current;
   10 k / 22 k into a BC847 inverter (points open ⇒ GPIO4 low), 1 k + 10 nF filter.
   **Remove the distributor condenser**: with no coil current through the points it is
   unnecessary and would slow the edge.
3. **Coil driver** — GPIO6 → **CD74HCT221** one-shot (≈ 16–23 ms, 15 k + 2.2 µF) → 100 Ω →
   **ISL9V3040P3** ignition IGBT. The one-shot is the **stuck-on guard** (a hung MCU can
   never leave the coil charged) and level-shifts to a 5 V gate drive. 10 k pull-downs
   keep the coil off while the MCU boots. Q1 lies flat on the case floor in a PCB cut-out
   (see `hardware/README.md`); the tab is the coil negative, so **insulate it from the case**.
4. **MAP sensor** — any linear 5 V 1-bar analog sensor on J2 (GM-style 3-wire, Lada/Bosch,
   MPX4115 …). 10 k / 15 k divider + 100 nF + 3.6 V zener into GPIO1. Slope/offset are
   set in the web app (Setup → MAP sensor, two-point calibration).
5. **Battery and LPG sense** — 100 k / 22 k dividers (+ 100 nF, 3.6 V zener) on GPIO3 and
   GPIO0. LPG: > 7 V = on, < 4 V = off, ~300 ms debounce; unconnected = petrol.
6. **Status LED** — the module's on-board RGB LED: blue blip = stopped, yellow = cranking,
   green = petrol, cyan = LPG, violet = simulator / OTA.

Pins in `config.h` (match the PCB): LPG 0, MAP 1, VBAT 3, POINTS 4, COIL 6, LED 10 —
ADC1 pins only, because ADC2 is unusable while WiFi is on.

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

### Alternative: keep the factory distributor (close-edge reference)

Setup → **Trigger reference → Points CLOSE edge**. Instead of locking the advance at 40°:

1. Block the centrifugal weights **at rest** (no advance), disconnect the vacuum capsule, remove the condenser.
2. Set the distributor to its factory static timing (≈10° BTDC at the points-open moment) — this is the cranking reference (*Points-open angle*, default 10°).
3. The points *closing* edge then falls roughly 120° before the next cylinder's TDC (depends on the points gap/dwell angle). Start with the default 120° and calibrate with the timing light exactly as below — the formula is the same.
4. Keep *Acceleration compensation* at 100% and the cranking advance ≤3°. Both are filled in when you pick this mode.

Trade-off: the spark is placed from a signal ~3× further ahead, so error grows with rpm change, and lobe-to-lobe distributor wear is no longer cancelled. The closing edge also bounces; look at the signal on a scope before trusting it at high rpm.

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
