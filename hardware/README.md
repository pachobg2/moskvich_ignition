# ECU hardware — schematic, PCB, BOM

Through-hole only (no SMD). An **ESP32-C3-Zero** plugs into sockets on the
board, so nothing needs reflow and everything can be built with a plain
soldering iron.

| File | What it is |
|---|---|
| `schematic.svg` | One-sheet schematic (open in any browser) |
| `pcb_top.svg` | Board preview: top view, both copper layers, silkscreen, IGBT cut-out |
| `moskvich_ecu.kicad_pcb` | The PCB, KiCad 7 format (footprints embedded, tracks, vias, GND pours, outline + cut-out) |
| `bom.csv`, `netlist.csv` | Parts list and net list |
| `gen/pcbgen.py` | Generator: parts, placement, auto-router, design-rule check, KiCad/SVG/BOM export |
| `gen/gen_schematic.py` | Draws `schematic.svg` |

Regenerate everything (needs `numpy scipy shapely`):

```bash
python hardware/gen/pcbgen.py && python hardware/gen/gen_schematic.py
```

> **Status — read this.** The board was generated and checked by my own
> script (copper clearance ≥ 0.3 mm, every net connected, nothing within
> 0.6 mm of the edge or inside the cut-out, no hole collisions) and the file
> parses in a KiCad-format library. **It has not been opened in KiCad, run
> through KiCad's DRC, or built.** Before ordering: open it in KiCad ≥ 7,
> press **B** to fill the zones, run **Inspect → Design Rules Checker**, and
> compare the footprints with the real parts in your hand.

## Circuit in one paragraph

12 V (ignition switched) → PTC fuse → reverse diode → TVS → 470 µF → **TSR
1-2450** buck module → 5 V. Points signal (J1.1) is pulled up through 1 kΩ
(wetting current), divided, and inverted by a BC547 into GPIO4. LPG valve
signal, battery voltage and the MAP sensor each go through a divider with a
100 nF cap and a 3.6 V zener into ADC1 pins. The MCU's coil command (GPIO6)
feeds a **74HCT123** that (a) turns the 3.3 V logic into a 5 V gate drive and
(b) limits coil-on time to ≈ 12 ms in hardware, so a hung MCU can never leave
the coil charged. The 74HCT123 drives the **ISL9V3040P3** ignition IGBT; its
collector (the tab) is the coil negative (J1.4). Coil current never touches
the 5 V rail or the sensor grounds' thin traces: it returns through the GND
pour and the 3 mm J1.3 ground path.

## Board facts

* 101.6 × 63.5 mm (4″ × 2.5″), 2 layers, 1.6 mm, M3 holes at the four corners (3.81 mm in from each edge).
* Grid 0.254 mm (0.01″); tracks 0.5 mm (signal), 0.8 mm (power), **2.0 mm** collector track on the bottom layer, GND pour on both layers (solid pad connection).
* Origin is the top-left corner of the board, X right, Y down (as in KiCad).
* USB-C of the module points at the **top** edge (2.2 mm from it) — flash through a hole in the box, or use OTA.

## Mounting the IGBT through the cut-out

The IGBT is cooled by the **case floor**, which acts as the heat sink.

```
   box floor (aluminium) ─────────────────────────────  z = 0
        │     ┌ M3 screw into a tapped hole in the floor
   tab ─┴─ insulating pad (silicone, 0.25 mm) + shoulder washer
   TO-220 lies flat in the cut-out  ───►  leads bent UP through the 3 pads
   PCB on 6 mm standoffs  ─────────────────────────────  z = 6
```

* **Cut-out:** 18.0 × 13.2 mm window at x 74.17 … 92.20, y 37.85 … 51.05.
* **Lead pads** (Q1): gate (71.12, 41.91), collector (71.12, 44.45), emitter (71.12, 46.99) — 3.0 mm left of the window, 2.54 mm pitch, 1.2 mm drill.
* **Tab hole** ends up at about (87.6, 44.45) — drill/tap the M3 hole in the floor from the PCB itself (use the board as a template through the mounting holes) and check against the real part before drilling.
* **Lead forming:** leave ~4 mm flat from the package, bend up 90°, then trim to just clear the board top. The leads are 0.5 mm thick, 0.8 mm wide, so they sit in the 1.2 mm holes with a bend radius ≥ 1 mm.
* Pin order seen from the top with the part lying face-up and leads pointing left: **gate (top), collector (middle), emitter (bottom)** — the silkscreen shows the outline. Check the datasheet of the exact part (pin 1 = gate, 2 = collector/tab, 3 = emitter for the ISL9V3040P3).
* **The tab is the coil negative** (battery voltage plus spikes to ~400 V). It must be insulated from the case: silicone pad + plastic shoulder washer under the screw head. Check with a multimeter that tab ↔ case is open before powering the board.
* Dissipation is ≈ 2–3 W at 6000 rpm (≈ 0.3 W at idle); the floor of a metal box is plenty. On a plastic box use a real heat sink bolted to the tab instead.

## Parts that need a decision before you buy

| Item | Why |
|---|---|
| ESP32-C3-Zero pad order | Pad numbers/GPIOs follow the Waveshare list (1 = 5V, 2 = GND, 3 = 3V3, 4–9 = IO0…IO5 down the left side, 10–18 = IO6, IO7, IO8, IO9, IO10, IO18, IO19, IO20, IO21 up the right side, USB at the top). **Read the silkscreen on your module** and measure the header row distance (15.24 mm assumed). If it differs, only `config.h` pins (and, if analog pins move off GPIO0–4, the PCB) change. |
| TSR 1-2450 pinout | Assumed 1 = Vin, 2 = GND, 3 = Vout (like a 7805). Confirm with the datasheet of the module you buy. |
| Module antenna | The area under the module body is kept copper-free (no pour); check where the antenna is on your revision and keep metal and tall parts away from it. |
| MAP sensor connector | J2.2 +5 V, J2.3 signal, J2.4 GND. GM-style sensors are usually pin A = ground, B = signal, C = 5 V — check your pinout. |

## Bring-up

1. **Without** the module, the 74HCT123 and the IGBT: power from a bench supply (current limit 0.3 A). Check +5 V at C3, check reverse polarity is blocked, check the TVS does not conduct at 14.4 V.
2. Insert U3 (in its socket) and the module. Connect USB, flash the firmware, join the `Moskvich-ECU` WiFi. The on-board LED: blue blip = stopped, yellow = cranking, green = running on petrol, cyan = running on LPG, violet = simulator/OTA.
3. In the web app (Tools → Bench simulator) run 800 / 3000 / 6000 rpm with a **100 Ω resistor and LED, or a scope, instead of the coil** on J1.4. Check: one pulse per trigger, on-time = the dwell in the Live tab (≈ 3.9 ms at 12.5 V), nothing when the simulator is off.
4. Hold the command high on purpose (a jumper from 3.3 V to GPIO6 with the firmware stopped): the 74HCT123 output must drop after ≈ 12 ms. That is the stuck-on guard working.
5. Only then fit Q1 on the floor, connect the coil and the distributor, and follow `docs/HARDWARE.md` (distributor preparation, timing-light calibration, first start).

## Not done / ideas

* No gerbers: export them from KiCad (File → Plot) after the DRC. Add your own fab text and a silk logo.
* A coil-current sense resistor and an over-current IGBT limit were left out to keep the build simple.
* The two spare MCU pins on the left header (IO2, IO5) are unconnected — a coolant-temperature divider would use one of them (ADC1: IO2).
