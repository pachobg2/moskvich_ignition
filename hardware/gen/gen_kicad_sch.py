#!/usr/bin/env python3
"""Writes hardware/moskvich_ecu.kicad_sch (KiCad 7) from the same part list as the PCB (gen/pcbgen.py).

Every pin gets a short wire and a net label, so connectivity is carried by the labels (no long wires): the
sheet is a grouped parts-and-nets drawing, not a hand-routed picture (see schematic.svg for that).
After writing, the script re-reads its own output and checks that every pin lands on the net the PCB has.
"""
import os, sys, math, uuid
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pcbgen as P

OUT = os.path.join(P.OUT, "moskvich_ecu.kicad_sch")
ROOT = str(uuid.uuid4())
U = lambda: str(uuid.uuid4())

FPREF = {
    "R_0603": "Resistor_SMD:R_0603_1608Metric", "C_0603": "Capacitor_SMD:C_0603_1608Metric", "C_0805": "Capacitor_SMD:C_0805_2012Metric",
    "R_1206": "Resistor_SMD:R_1206_3216Metric", "Fuse_1206": "Fuse:Fuse_1206_3216Metric", "C_1210": "Capacitor_SMD:C_1210_3225Metric",
    "D_SOD-123": "Diode_SMD:D_SOD-123", "D_SMA": "Diode_SMD:D_SMA", "D_SMB": "Diode_SMD:D_SMB", "L_4x4": "Inductor_SMD:L_4x4 (generic: pick your inductor)",
    "SOT-23": "Package_TO_SOT_SMD:SOT-23", "SOT-23-6": "Package_TO_SOT_SMD:SOT-23-6", "SOIC-16_3.9x9.9mm_P1.27mm": "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm",
}
PINNAMES = {
    "U1": {1: "5V", 2: "GND", 3: "3V3", 4: "IO0", 5: "IO1", 6: "IO2", 7: "IO3", 8: "IO4", 9: "IO5", 10: "IO6", 11: "IO7", 12: "IO8", 13: "IO9 BOOT", 14: "IO10 RGB",
           15: "IO18", 16: "IO19", 17: "IO20", 18: "IO21"},
    "U2": {1: "FB", 2: "EN", 3: "VIN", 4: "GND", 5: "SW", 6: "BST"},
    "U3": {1: "1A", 2: "1B", 3: "/1R", 4: "/1Q", 5: "2Q", 6: "2Cx", 7: "2RxCx", 8: "GND", 9: "2A", 10: "2B", 11: "/2R", 12: "/2Q", 13: "1Q", 14: "1Cx", 15: "1RxCx", 16: "VCC"},
    "Q1": {1: "G", 2: "C", 3: "E"}, "Q2": {1: "B", 2: "E", 3: "C"},
    "J1": {1: "POINTS", 2: "+12V", 3: "GND", 4: "COIL-"}, "J2": {1: "LPG", 2: "+5V", 3: "MAP", 4: "GND"},
}
TWO_PIN_NAMES = {"D": {1: "K", 2: "A"}, "L": {1: "1", 2: "2"}, "F": {1: "1", 2: "2"}}

GROUPS = [
    ("CONNECTORS", ["J1", "J2"]),
    ("POWER INPUT -> 5 V", ["F1", "D1", "D2", "C1", "C2", "U2", "L1", "C11", "C3", "C4"]),
    ("POINTS INPUT", ["R1", "R2", "R3", "D3", "Q2", "R4", "R5", "C5"]),
    ("ANALOG INPUTS (LPG / BATTERY / MAP)", ["R6", "R7", "C6", "D4", "R8", "R9", "C7", "D5", "R11", "R12", "C8", "D6"]),
    ("COIL DRIVER", ["R10", "R13", "U3", "C9", "R14", "C10", "R15", "R16", "Q1"]),
    ("MCU MODULE", ["U1"]),
]

pcb = {p.ref: p for p in P.PARTS}


def pins_of(part):
    ref = part.ref
    nums = [int(p["name"]) for p in part.pads]
    names = PINNAMES.get(ref)
    if names is None:
        names = TWO_PIN_NAMES.get(ref[0], {n: "~" for n in nums})
        if ref[0] in "RC": names = {1: "~", 2: "~"}
    return [(n, names.get(n, str(n))) for n in sorted(nums)]


def layout_pins(part):
    """returns (left, right) lists of (number, name) top->bottom"""
    pins = pins_of(part)
    n = len(pins)
    if part.ref == "U1": return [p for p in pins if p[0] <= 9], [p for p in sorted(pins, key=lambda t: -t[0]) if p[0] >= 10]
    if part.ref == "U3": return [p for p in pins if p[0] <= 8], [p for p in sorted(pins, key=lambda t: -t[0]) if p[0] >= 9]
    if part.ref == "U2": return [p for p in pins if p[0] in (3, 2, 1, 4)], [p for p in pins if p[0] in (5, 6)]
    if n == 2: return [pins[0]], [pins[1]]
    if part.ref in ("Q1", "Q2"): return [pins[0]], pins[1:]
    return pins, []


def symdef(part, left, right):
    name = f"ECU:{part.ref}"
    rows = max(len(left), len(right), 1)
    maxl = max([len(t[1]) for t in left] or [1]); maxr = max([len(t[1]) for t in right] or [1])
    w = max(7.62, math.ceil((maxl + maxr + 3) * 1.1 / 2.54) * 2.54)
    h = rows * 2.54
    top = (rows - 1) * 1.27
    o = []
    a = o.append
    a(f'    (symbol "{name}" (pin_names (offset 0.762)) (in_bom yes) (on_board yes)')
    a(f'      (property "Reference" "{part.ref[0]}" (at 0 {h / 2 + 1.5:.3f} 0) (effects (font (size 1.27 1.27))))')
    a(f'      (property "Value" "{part.value}" (at 0 {-h / 2 - 1.5:.3f} 0) (effects (font (size 1.27 1.27))))')
    a(f'      (property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))')
    a(f'      (property "Datasheet" "~" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))')
    a(f'      (symbol "{part.ref}_0_1" (rectangle (start {-w / 2:.3f} {h / 2:.3f}) (end {w / 2:.3f} {-h / 2:.3f}) (stroke (width 0.254) (type default)) (fill (type background))))')
    a(f'      (symbol "{part.ref}_1_1"')
    for i, (num, nm) in enumerate(left):
        y = top - i * 2.54
        a(f'        (pin passive line (at {-w / 2 - 2.54:.3f} {y:.3f} 0) (length 2.54) (name "{nm}" (effects (font (size 1.016 1.016)))) (number "{num}" (effects (font (size 1.016 1.016)))))')
    for i, (num, nm) in enumerate(right):
        y = top - i * 2.54
        a(f'        (pin passive line (at {w / 2 + 2.54:.3f} {y:.3f} 180) (length 2.54) (name "{nm}" (effects (font (size 1.016 1.016)))) (number "{num}" (effects (font (size 1.016 1.016)))))')
    a('      )')
    a('    )')
    return "\n".join(o), w, h, top


def main():
    lib, body = [], []
    placed = {}                                   # (ref, pin) -> (x, y) schematic coordinates of the connection point
    labels = {}                                   # (x, y) -> net name
    x0, y0 = 20.0, 28.0
    cx, cy = x0, y0
    rowh = 0
    PAGE_W = 570
    text = []
    for title, refs in GROUPS:
        cx = x0
        cy += (rowh + 14) if rowh else 0
        rowh = 0
        text.append((cx, cy - 6, title))
        cy += 4
        for ref in refs:
            part = pcb[ref]
            left, right = layout_pins(part)
            sd, w, h, top = symdef(part, left, right)
            lib.append(sd)
            cellw = w + 2 * (2.54 + 5.08) + 62
            if cx + cellw > PAGE_W:
                cx = x0; cy += rowh + 10; rowh = 0
            X = cx + 31 + 7.62 + w / 2
            Y = cy + max(h / 2 + 8, 14)
            inst = []
            inst.append(f'  (symbol (lib_id "ECU:{ref}") (at {X:.3f} {Y:.3f} 0) (unit 1) (in_bom yes) (on_board yes) (dnp no) (uuid {U()})')
            inst.append(f'    (property "Reference" "{ref}" (at {X:.3f} {Y - h / 2 - 3:.3f} 0) (effects (font (size 1.27 1.27))))')
            inst.append(f'    (property "Value" "{part.value}" (at {X:.3f} {Y + h / 2 + 3:.3f} 0) (effects (font (size 1.27 1.27))))')
            inst.append(f'    (property "Footprint" "{FPREF.get(part.fpname, part.fpname)}" (at {X:.3f} {Y:.3f} 0) (effects (font (size 1.27 1.27)) hide))')
            inst.append(f'    (property "Datasheet" "~" (at {X:.3f} {Y:.3f} 0) (effects (font (size 1.27 1.27)) hide))')
            for num, _ in left + right: inst.append(f'    (pin "{num}" (uuid {U()}))')
            inst.append(f'    (instances (project "moskvich_ecu" (path "/{ROOT}" (reference "{ref}") (unit 1))))')
            inst.append('  )')
            body.append("\n".join(inst))
            for side, pl in (("L", left), ("R", right)):
                for i, (num, _) in enumerate(pl):
                    py = Y - (top - i * 2.54)                      # lib y is up, schematic y is down
                    px = X - w / 2 - 2.54 if side == "L" else X + w / 2 + 2.54
                    sx = px - 5.08 if side == "L" else px + 5.08
                    net = next((p["net"] for p in part.pads if p["name"] == str(num)), None)
                    placed[(ref, str(num))] = (round(px, 3), round(py, 3))
                    if net:
                        body.append(f'  (wire (pts (xy {px:.3f} {py:.3f}) (xy {sx:.3f} {py:.3f})) (stroke (width 0) (type default)) (uuid {U()}))')
                        just = "right" if side == "L" else "left"
                        body.append(f'  (label "{net}" (at {sx:.3f} {py:.3f} {180 if side == "L" else 0}) (effects (font (size 1.27 1.27)) (justify {just} bottom)) (uuid {U()}))')
                        labels[(round(sx, 3), round(py, 3))] = (net, (ref, str(num)))
                    else:
                        body.append(f'  (no_connect (at {px:.3f} {py:.3f}) (uuid {U()}))')
            cx += cellw
            rowh = max(rowh, h + 16)
    out = ['(kicad_sch (version 20230121) (generator eeschema)', f'  (uuid {ROOT})', '  (paper "A2")', '  (lib_symbols']
    out += lib
    out.append('  )')
    for x, y, t in text:
        out.append(f'  (text "{t}" (at {x:.2f} {y:.2f} 0) (effects (font (size 3 3) (bold yes)) (justify left bottom)) (uuid {U()}))')
    out.append(f'  (text "Moskvich ECU rev B - generated by hardware/gen/gen_kicad_sch.py from the PCB part list. Connectivity is by net labels." (at 20 14 0) (effects (font (size 2.5 2.5)) (justify left bottom)) (uuid {U()}))')
    out += body
    out.append('  (sheet_instances (path "/" (page "1")))')
    out.append(')')
    open(OUT, "w", encoding="utf-8", newline="\n").write("\n".join(out))
    # project file so KiCad opens the schematic and PCB together
    pro = os.path.join(P.OUT, "moskvich_ecu.kicad_pro")
    if not os.path.exists(pro):
        open(pro, "w", encoding="utf-8").write('{\n  "meta": { "filename": "moskvich_ecu.kicad_pro", "version": 1 }\n}\n')
    return placed, labels


def verify():
    """independent check: re-read the written file and compare every pin's net with the PCB netlist"""
    from kiutils.schematic import Schematic
    sch = Schematic.from_file(OUT)
    lab = {(round(l.position.X, 3), round(l.position.Y, 3)): l.text for l in sch.labels}
    wires = [(round(w.points[0].X, 3), round(w.points[0].Y, 3), round(w.points[1].X, 3), round(w.points[1].Y, 3)) for w in sch.graphicalItems if w.__class__.__name__ == "Connection"]
    ends = {}
    for (x1, y1, x2, y2) in wires: ends[(x1, y1)] = (x2, y2); ends[(x2, y2)] = (x1, y1)
    bad = 0
    sch_nets = {}
    for sym in sch.schematicSymbols:
        ref = next(p.value for p in sym.properties if p.key == "Reference")
        part = pcb[ref]
        libsym = next(s for s in sch.libSymbols if s.entryName == f"ECU:{ref}" or s.libId == f"ECU:{ref}")
        for unit in libsym.units:
            for pin in unit.pins:
                px = round(sym.position.X + pin.position.X, 3); py = round(sym.position.Y - pin.position.Y, 3)
                far = ends.get((px, py))
                net = lab.get(far) if far else None
                want = next((p["net"] for p in part.pads if p["name"] == pin.number), None)
                if net != want:
                    bad += 1; print(f"MISMATCH {ref}.{pin.number}: schematic {net} vs PCB {want}")
                if net: sch_nets.setdefault(net, set()).add(f"{ref}.{pin.number}")
    pcb_nets = {}
    for p in P.PARTS:
        for pd in p.pads:
            if pd["net"]: pcb_nets.setdefault(pd["net"], set()).add(f'{pd["ref"]}.{pd["name"]}')
    same = sch_nets == pcb_nets
    print(f"symbols={len(sch.schematicSymbols)} labels={len(sch.labels)} nets={len(sch_nets)} pin mismatches={bad} netlists identical={same}")
    return bad == 0 and same


if __name__ == "__main__":
    main()
    ok = verify()
    print("wrote", OUT)
    sys.exit(0 if ok else 1)
