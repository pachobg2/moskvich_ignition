# ECU hardware — schematic, PCB, BOM (rev B, SMD)

A small two-sided board: **SMD** passives and ICs (0603 / 0805 / 1206 / 1210, SOT-23, SOIC-16, SMA/SMB), an
**SMD buck regulator** for the 5 V rail, an **ESP32-C3-Zero** on pin sockets, 3.5 mm screw terminals, and the
ignition **IGBT lying in a cut-out** so the case floor cools it. **48 × 41.8 mm** (about a third of the
through-hole rev A).

| File | What it is |
|---|---|
| `schematic.svg` | One-sheet schematic (open in any browser) |
| `pcb_top.svg`, `pcb_bottom.svg` | Board previews (both copper layers, courtyards, cut-out) |
| `moskvich_ecu.kicad_sch` | **Editable schematic**, KiCad 7 format (every pin has a net label; no hand-routed wires). Its netlist is checked identical to the PCB's. |
| `moskvich_ecu.kicad_pro` | Minimal KiCad project so the schematic and PCB open together |
| `moskvich_ecu.kicad_pcb` | The PCB, KiCad 7 format (footprints embedded, tracks, vias, GND pours, outline + cut-out) |
| `bom.csv`, `netlist.csv` | Parts list and net list |
| `gen/pcbgen.py` | Generator: parts, placement, auto-router, design-rule check, KiCad/SVG/BOM export |
| `gen/gen_schematic.py` | Draws `schematic.svg` (the readable picture) |
| `gen/gen_kicad_sch.py` | Writes `moskvich_ecu.kicad_sch` and verifies it against the PCB netlist |

Regenerate everything (needs `numpy scipy shapely`):

```bash
python hardware/gen/pcbgen.py && python hardware/gen/gen_schematic.py && python hardware/gen/gen_kicad_sch.py
```

> **Status — read this.** The board was generated and checked by my own script: copper clearance ≥ 0.2 mm
> (≥ 0.7 mm around the coil net), every net connected, nothing within 0.5 mm of the edge or inside the
> cut-out, no footprint overlaps, no hole collisions. The file parses in a KiCad-format library.
> **It has not been opened in KiCad, run through KiCad's DRC, or built.** The footprints are generic
> IPC-style land patterns I defined, not manufacturer drawings. Before ordering: open it in KiCad ≥ 7, press
> **B** to fill the zones, run **Inspect → Design Rules Checker**, and compare every footprint with the
> datasheet of the exact part you buy (especially the inductor, the fuse, the terminals and the module).

## Opening it in KiCad or EasyEDA

* **KiCad 7+:** open `moskvich_ecu.kicad_pro`. The schematic uses simple embedded symbols (boxes with named pins) and
  the footprint names of the PCB; reassign footprints/symbols to library parts as you like, then *Update PCB from schematic*.
* **EasyEDA:** import the files through EasyEDA's KiCad import (File → Import; menu names differ between the
  Standard and Pro editions — I have not tried it here). Import the schematic and the PCB; if the editor
  complains, open them in KiCad first and re-export. Replace the generic footprints with EasyEDA/LCSC library parts
  (this also gives JLCPCB assembly part numbers).
* The schematic is a **grouped parts-and-nets drawing**: connections are carried by net labels, not long wires.
  `schematic.svg` is the readable hand-arranged picture of the same circuit.
* Neither file was opened in KiCad/EasyEDA by me; the script re-reads the schematic and checks every pin's net
  against the PCB netlist (42 symbols, 24 nets, no mismatches).

## Circuit in one paragraph

Ignition-switched 12 V → 0.5 A fuse → SS34 Schottky (reverse polarity) → SMBJ24A TVS → 10 µF → **AP63205**
buck (3.8–32 V in, fixed 5 V, 2 A; L = 4.7 µH, 2 × 22 µF out, 100 nF boot — the datasheet's values) → 5 V. The
points signal (J1.1) is pulled up through 1 kΩ (wetting current), divided and inverted by a BC847 into GPIO4.
The LPG valve signal, the battery voltage and the MAP sensor each go through a divider with a 100 nF cap and
a 3.6 V zener into ADC1 pins. The MCU's coil command (GPIO6) feeds a **74HCT123** that (a) turns the 3.3 V
logic into a 5 V gate drive and (b) limits coil-on time in hardware (≈ 15–20 ms with the 22 kΩ / 2.2 µF
network; the firmware limits it to 9 ms first), so a hung MCU can never leave the coil charged. It drives the
**ISL9V3040P3** ignition IGBT; its collector (the tab) is the coil negative (J1.4). Coil current never flows
through the buck or the sensor grounds: it uses the 1.5 mm top-layer collector track and the GND pours / J1.3.

## Board facts

* **48 × 41.8 mm**, 2 layers, 1.6 mm, three M2.5 mounting holes. Origin = top-left corner, X right, Y down (as in KiCad).
* Rules used: tracks 0.2 mm (signal) / 0.4 mm (power) / **1.5 mm** (collector), clearance 0.2 mm (0.7 mm around the collector net), vias 0.8 / 0.4 mm, GND pours on both layers. These are within normal 2-layer fab capabilities.
* **Top side:** module sockets, J1/J2, the IGBT pads, and the buck stage (U2, L1, C1–C4, C11, D1, D2, F1).
* **Bottom side:** everything else (dividers, points input, the 74HCT123 and its resistors). The bottom parts sit in the 6 mm gap between the board and the case floor, so keep them ≤ 3 mm tall.
* **Terminals:** J2 (top right) = LPG signal, +5 V, MAP signal, GND. J1 (right) = points lead, +12 V, GND, coil −. The coil track and the GND pour handle the 4 A pulses; use ≥ 1 mm² wire on J1.3 / J1.4.
* The module's USB-C faces the **top** edge (2.2 mm from it).

## Mounting the IGBT through the cut-out

The IGBT is cooled by the **case floor**, which acts as the heat sink.

```
   box floor (aluminium) ─────────────────────────────  z = 0
        │     ┌ M3 screw into a tapped hole in the floor
   tab ─┴─ insulating pad (silicone, 0.25 mm) + shoulder washer
   TO-220 lies flat in the cut-out  ───►  leads bent UP through the 3 pads
   PCB on 6 mm standoffs  ─────────────────────────────  z = 6
```

* **Cut-out:** 18.0 × 13.0 mm window at x 3.64 … 21.64, y 27.79 … 40.79 (bottom-left of the board). The strip below it is only 1.0 mm wide — handle the board by the edges.
* **Lead pads** (Q1), 3.0 mm to the right of the window, 2.54 mm pitch, 1.0 mm drill: collector (24.64, 34.29), emitter (24.64, 31.75), gate (24.64, 36.83) — the part is rotated 180° compared with a part lying leads-left, so **the gate is the lowest pad**. The silkscreen outline shows the package.
* **Tab hole** ends up near (8.1, 34.3) (the board is the drilling template through the cut-out). Check against your real part before drilling the floor.
* **Lead forming:** leave ~4 mm flat from the package, bend up 90°, trim to just clear the board top.
* **The tab is the coil negative** (battery voltage plus spikes to ~400 V). Insulate it from the case: silicone pad + plastic shoulder washer under the screw head. Check with a multimeter that tab ↔ case is open before powering the board.
* Dissipation is ≈ 2–3 W at 6000 rpm (≈ 0.3 W at idle); the floor of a metal box is plenty. On a plastic box use a real heat sink bolted to the tab.
* Conformal-coat the board: the collector net sees ~400 V flyback spikes with only 0.7–0.8 mm clearance.

## Parts and where the numbers come from

| Item | Notes |
|---|---|
| **AP63205WU-7** (U2) | Pinout, 4.7 µH / 10 µF / 2 × 22 µF / 100 nF and "EN to VIN" are from the Diodes datasheet (DS41326). Abs. max VIN 35 V DC / 40 V for 400 ms — hence the SMBJ24A TVS (clamps ≈ 39 V). Fit enough ceramic capacitance after derating (X7R/X5R lose capacity with DC bias). |
| **L1** | Any shielded 4.7 µH inductor, I_sat ≥ 2 A, DCR < 100 mΩ, about 4 × 4 mm. The land pattern is generic: adjust it to your part. |
| **F1** | An SMD fuse (0.5 A, ≥ 32 V rating) or a PTC of the same 1206 size. Ordinary 1206 PTCs are often rated only 6–15 V: do not use those. |
| **C10 / R14** | 2.2 µF X7R + 22 kΩ ⇒ guard time ≈ 0.45·R·C, with the capacitor derated by its DC bias: 15–20 ms. If you change them keep it above the firmware's 9 ms and below ~25 ms. |
| ESP32-C3-Zero pad order | Pad numbers/GPIOs follow the Waveshare list (1 = 5V, 2 = GND, 3 = 3V3, 4–9 = IO0…IO5 down the left side, 10–18 = IO6, IO7, IO8, IO9, IO10, IO18, IO19, IO20, IO21 up the right side, USB at the top). **Read the silkscreen on your module** and measure the header row distance (15.24 mm assumed). |
| Module antenna | I did not leave a copper-free zone under the module (SMD parts and pours sit under its body); check where the antenna is on your revision and, if needed, move parts and add a keep-out there. |
| MAP sensor connector | J2.2 +5 V, J2.3 signal, J2.4 GND. GM-style sensors are usually A = ground, B = signal, C = 5 V — check yours. |

## Bring-up

1. **Without** the module, the 74HCT123 and the IGBT: power from a bench supply (current limit 0.3 A). Check +5 V, check reverse polarity is blocked, check the TVS does not conduct at 14.4 V.
2. Fit U3 and the module, flash, join `Moskvich-ECU`. LED: blue blip = stopped, yellow = cranking, green = petrol, cyan = LPG, violet = simulator / OTA.
3. Tools → Bench simulator at 800 / 3000 / 6000 rpm with a **100 Ω resistor and LED, or a scope, instead of the coil** on J1.4. Check one pulse per trigger and on-time = the dwell shown in the Live tab.
4. Hold the command high on purpose (firmware stopped, jumper 3.3 V → GPIO6): the 74HCT123 output must drop after ≈ 15–20 ms.
5. Only then fit Q1, connect the coil and the distributor, and follow `docs/HARDWARE.md`.

## Not done / ideas

* No gerbers: export them from KiCad (File → Plot) after the DRC. Add a fab note and silk text.
* Hand-soldering: everything is 0603 or larger; the SOIC-16 and TSOT-23-6 need a fine tip and flux. Solder the buck IC and its caps first, then check the 5 V rail.
* A coil-current sense resistor and an IGBT over-current limit were left out to keep the build simple.
* Unused module pins on the left header (IO2, IO5) are unconnected — a coolant-temperature divider would use IO2 (ADC1).
