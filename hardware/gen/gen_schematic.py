#!/usr/bin/env python3
"""Draws hardware/schematic.svg (one sheet). Refs/values match gen/pcbgen.py (the PCB netlist)."""
import os, html

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "schematic.svg")
W, H = 2560, 1200
o = []
INK, WIRE, TXT, NETC = "#1a1a1a", "#0a6b2c", "#111", "#7a1fa2"


def add(s): o.append(s)


def line(x1, y1, x2, y2, c=INK, w=1.8):
    add(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{c}" stroke-width="{w}" stroke-linecap="round"/>')


def wire(*a):
    pts = list(a) if isinstance(a[0], tuple) else list(zip(a[0::2], a[1::2]))
    add(f'<polyline points="{" ".join(f"{x},{y}" for x, y in pts)}" fill="none" stroke="{WIRE}" stroke-width="1.8" stroke-linejoin="round"/>')


def dot(x, y): add(f'<circle cx="{x}" cy="{y}" r="3.6" fill="{WIRE}"/>')


def text(x, y, s_, size=12, anchor="start", c=TXT, bold=False, italic=False):
    st = ' font-weight="700"' if bold else ""
    st += ' font-style="italic"' if italic else ""
    add(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" fill="{c}"{st}>{html.escape(str(s_), quote=False)}</text>')


def box(x, y, w, h, c=INK, fill="none", sw=1.8, dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}" stroke="{c}" stroke-width="{sw}"{d} rx="3"/>')


def netlabel(x, y, name, side="r"):
    """flag whose attachment point is (x, y); side r = body extends to the right of x, l = to the left"""
    w = 7.4 * len(name) + 16
    if side == "r":
        add(f'<path d="M{x},{y} l9,-9 h{w - 9} v18 h-{w - 9} z" fill="#f4e6fb" stroke="{NETC}" stroke-width="1.3"/>')
        text(x + 12, y + 4.5, name, 12, c=NETC, bold=True)
    else:
        add(f'<path d="M{x},{y} l-9,-9 h-{w - 9} v18 h{w - 9} z" fill="#f4e6fb" stroke="{NETC}" stroke-width="1.3"/>')
        text(x - 12, y + 4.5, name, 12, "end", c=NETC, bold=True)


def gnd(x, y):
    line(x, y, x, y + 7); line(x - 11, y + 7, x + 11, y + 7); line(x - 7, y + 12, x + 7, y + 12); line(x - 3, y + 17, x + 3, y + 17)


def vcc(x, y, name="+5V"):
    line(x, y, x, y - 12); line(x - 10, y - 12, x + 10, y - 12); text(x, y - 18, name, 12, "middle", bold=True)


def sym(kind, x, y, orient, ref, val, L=60, pol=False, rev=False, tside="r"):
    """2-pin symbol. 'h': pins (x,y)-(x+L,y). 'v': pins (x,y)-(x,y+L). rev flips diode direction (cathode toward pin 1)."""
    m = L / 2
    g = '<g stroke="%s" stroke-width="1.8" fill="none" stroke-linecap="round" transform="' % INK
    if orient == "h":
        g += f'translate({x},{y})' + (f' translate({L},0) scale(-1,1)' if rev else '')
    else:
        g += f'translate({x},{y}) rotate(90)' + (f' translate({L},0) scale(-1,1)' if rev else '')
    g += '">'
    if kind == "R":
        g += f'<path d="M0,0 H{m - 18} M{m + 18},0 H{L}"/><rect x="{m - 18}" y="-6.5" width="36" height="13"/>'
    elif kind == "C":
        g += f'<path d="M0,0 H{m - 4} M{m + 4},0 H{L} M{m - 4},-11 V11 M{m + 4},-11 V11"/>'
    elif kind == "F":
        g += f'<path d="M0,0 H{m - 20} M{m + 20},0 H{L} M{m - 20},0 H{m + 20}"/><rect x="{m - 20}" y="-6.5" width="40" height="13"/>'
    elif kind == "L":
        g += f'<path d="M0,0 H{m - 16} a4,4 0 0 1 8,0 a4,4 0 0 1 8,0 a4,4 0 0 1 8,0 a4,4 0 0 1 8,0 H{L}"/>'
    elif kind in ("D", "Z", "T"):
        g += f'<path d="M0,0 H{m - 8} M{m + 8},0 H{L}"/><path d="M{m - 8},-10 V10 L{m + 8},0 Z" fill="#fff"/>'
        g += f'<path d="M{m + 8},-10 V10"/>' if kind == "D" else f'<path d="M{m + 8},-10 V10 M{m + 8},-10 L{m + 13},-14 M{m + 8},10 L{m + 3},14"/>'
    g += '</g>'
    add(g)
    if orient == "h":
        text(x + m, y - 15, ref, 12.5, "middle", bold=True); text(x + m, y + 27, val, 11.5, "middle")
        if pol: text(x + 8, y - 8, "+", 14, bold=True)
        return (x, y), (x + L, y)
    tx = x + 15 if tside == "r" else x - 15
    an = "start" if tside == "r" else "end"
    text(tx, y + m - 3, ref, 12.5, an, bold=True); text(tx, y + m + 12, val, 11.5, an)
    if pol: text(x - 8, y + 14, "+", 14, "end", bold=True)
    return (x, y), (x, y + L)


def npn(x, y, ref, val):
    """base pin (x,y); collector pin (x+34,y-40); emitter pin (x+34,y+40)"""
    add(f'<g stroke="{INK}" stroke-width="1.8" fill="none" stroke-linecap="round"><circle cx="{x + 26}" cy="{y}" r="24"/>'
        f'<path d="M{x},{y} H{x + 14} M{x + 14},{y - 14} V{y + 14}"/>'
        f'<path d="M{x + 14},{y - 6} L{x + 34},{y - 22} V{y - 40}"/><path d="M{x + 14},{y + 6} L{x + 34},{y + 22} V{y + 40}"/>'
        f'<path d="M{x + 25},{y + 16} L{x + 34},{y + 22} L{x + 26},{y + 25} Z" fill="{INK}"/></g>')
    text(x + 62, y - 4, ref, 12.5, bold=True); text(x + 62, y + 11, val, 11.5)
    return (x, y), (x + 34, y - 40), (x + 34, y + 40)


def igbt(x, y, ref, val):
    """gate pin (x,y); collector pin (x+42,y-40); emitter pin (x+42,y+40)"""
    add(f'<g stroke="{INK}" stroke-width="1.8" fill="none" stroke-linecap="round"><path d="M{x},{y} H{x + 12} M{x + 12},{y - 16} V{y + 16}"/>'
        f'<path d="M{x + 20},{y - 18} V{y + 18} M{x + 20},{y - 11} H{x + 42} V{y - 40} M{x + 20},{y + 11} H{x + 42} V{y + 40}"/>'
        f'<path d="M{x + 33},{y + 6} L{x + 42},{y + 11} L{x + 33},{y + 16} Z" fill="{INK}"/></g>')
    text(x + 54, y - 4, ref, 12.5, bold=True); text(x + 54, y + 11, val, 11.5)
    return (x, y), (x + 42, y - 40), (x + 42, y + 40)


def ic(x, y, w, h, title, sub, left=(), right=(), top=(), bottom=()):
    box(x, y, w, h, fill="#fffbea")
    text(x + w / 2, y + 18, title, 14, "middle", bold=True); text(x + w / 2, y + 33, sub, 11, "middle", italic=True)
    P = {}
    for no, nm, yy in left:
        line(x - 22, y + yy, x, y + yy); text(x + 6, y + yy + 4, nm, 11.5); text(x - 6, y + yy - 4, str(no), 10, "end", c="#666"); P[no] = (x - 22, y + yy)
    for no, nm, yy in right:
        line(x + w, y + yy, x + w + 22, y + yy); text(x + w - 6, y + yy + 4, nm, 11.5, "end"); text(x + w + 6, y + yy - 4, str(no), 10, c="#666"); P[no] = (x + w + 22, y + yy)
    for no, nm, xx in top:
        line(x + xx, y - 22, x + xx, y); text(x + xx, y + 16, nm, 11.5, "middle"); text(x + xx + 5, y - 8, str(no), 10, c="#666"); P[no] = (x + xx, y - 22)
    for no, nm, xx in bottom:
        line(x + xx, y + h, x + xx, y + h + 22); text(x + xx, y + h - 8, nm, 11.5, "middle"); text(x + xx + 5, y + h + 15, str(no), 10, c="#666"); P[no] = (x + xx, y + h + 22)
    return P


def block(x, y, w, h, title):
    box(x, y, w, h, c="#9aa5b1", sw=1.2, dash="7 5")
    text(x + 10, y + 20, title, 14, bold=True, c="#334")


add(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" style="width:100%;height:auto;background:#fff" font-family="Arial, Helvetica, sans-serif">')
add(f'<rect width="{W}" height="{H}" fill="#fff"/>')
text(30, 34, "Moskvich 412 / 2140 ignition ECU — schematic", 22, bold=True)
text(30, 56, "Rev B · SMD (two-sided) + through-hole connectors, module sockets and IGBT · refs/values match hardware/moskvich_ecu.kicad_pcb", 12.5, c="#555")

# =================================================================== connectors
block(20, 80, 250, 560, "CONNECTORS")
for (y0, ref, sub, pins) in ((120, "J1", "Power / coil / points", ["POINTS_RAW", "+12V_IN", "GND", "COIL_N"]),
                             (390, "J2", "Sensors", ["LPG_IN", "+5V", "MAP_SIG", "GND"])):
    box(36, y0, 80, 168, fill="#eef4ff")
    text(76, y0 + 20, ref, 14, "middle", bold=True); text(76, y0 + 36, sub[:14], 10, "middle", italic=True)
    for i, nm in enumerate(pins):
        yy = y0 + 62 + i * 32
        line(116, yy, 138, yy); text(110, yy + 4, str(i + 1), 11, "end")
        if nm == "GND": gnd(138, yy)
        else: netlabel(138, yy, nm)
text(36, 316, "J1.1 distributor lead (was coil−)", 10.5, c="#555"); text(36, 331, "J1.2 ign-switched +12 V (pre-ballast)", 10.5, c="#555")
text(36, 346, "J1.3 chassis, thick wire (4 A pulses)", 10.5, c="#555"); text(36, 361, "J1.4 coil negative (−)", 10.5, c="#555")
text(36, 585, "J2.1 LPG valve +12 V signal", 10.5, c="#555"); text(36, 600, "J2.2-4 GM-type 3-wire MAP sensor", 10.5, c="#555")

# =================================================================== power
bx = 290
block(bx, 80, 1120, 505, "POWER INPUT → 5 V  (AP63205 buck)")
ry = 220
netlabel(bx + 14, ry, "+12V_IN"); wire(bx + 104, ry, bx + 120, ry)
sym("F", bx + 120, ry, "h", "F1", "0.5A 32V fuse", 76)
nA = bx + 230
wire(bx + 196, ry, nA, ry); dot(nA, ry)
wire(nA, ry, nA, ry - 44); netlabel(nA, ry - 44, "VBAT_F (to R8)")
sym("D", nA + 20, ry, "h", "D1", "SS34", 64)
wire(nA, ry, nA + 20, ry)
nB = bx + 380
wire(nA + 84, ry, nB, ry); dot(nB, ry)
text(nB + 10, ry + 20, "VIN_P", 11.5, c=NETC, bold=True)
sym("R", nB, ry - 100, "v", "R1", "1k 0.25W", 56)
wire(nB, ry - 44, nB, ry); netlabel(nB, ry - 100, "POINTS_RAW") if False else None
wire(nB, ry - 100, nB, ry - 112); netlabel(nB, ry - 112, "POINTS_RAW")
xs = [nB + 110, nB + 220, nB + 310]
wire(nB, ry, xs[2], ry)
sym("T", xs[0], ry, "v", "D2", "SMBJ24A TVS", 64, rev=True); dot(xs[0], ry); gnd(xs[0], ry + 64)
sym("C", xs[1], ry, "v", "C1", "10µF 50V", 64, pol=True); dot(xs[1], ry); gnd(xs[1], ry + 64)
sym("C", xs[2], ry, "v", "C2", "100nF", 64); dot(xs[2], ry); gnd(xs[2], ry + 64)
yb = ry + 150
wire(nB, ry, nB, yb)
ux = nB + 60
wire(nB, yb, ux - 22, yb)
P = ic(ux, yb - 30, 160, 130, "U2", "AP63205WU-7", [(3, "VIN", 30), (2, "EN", 60)], [(5, "SW", 30), (6, "BST", 60), (1, "FB", 90)], bottom=[(4, "GND", 80)])
wire(ux - 22, yb + 30, ux - 40, yb + 30, ux - 40, yb); dot(ux - 40, yb)
gnd(ux + 80, yb + 122)
swx = ux + 182
wire(swx, yb, swx + 20, yb); dot(swx + 20, yb)
wire(swx + 20, yb, swx + 100, yb)
sym("L", swx + 100, yb, "h", "L1", "4.7µH", 80)
n5 = swx + 230
wire(swx + 180, yb, n5, yb); dot(n5, yb)
wire(swx, yb + 30, swx + 40, yb + 30); sym("C", swx + 40, yb, "v", "C11", "100nF", 30)
wire(swx, yb + 90, n5, yb + 90, n5, yb)
sym("C", n5 + 50, yb, "v", "C3", "22µF 10V", 64); gnd(n5 + 50, yb + 64)
sym("C", n5 + 140, yb, "v", "C4", "22µF 10V", 64); gnd(n5 + 140, yb + 64)
wire(n5, yb, n5 + 140, yb); dot(n5 + 50, yb)
wire(n5 + 140, yb, n5 + 190, yb); netlabel(n5 + 190, yb, "+5V")
text(bx + 14, 530, "D1 blocks reverse polarity; D2 clamps load dump to < 40 V (the buck's limit); the buck runs from 3.8 V, so cranking sag is no problem.", 11, c="#555")
text(bx + 14, 547, "U2 (Diodes AP63205): 3.8–32 V in, fixed 5 V / 2 A; datasheet values L = 4.7 µH, Cin = 10 µF, Cout = 2 x 22 µF, boot = 100 nF. Coil current does NOT flow here.", 11, c="#555")
text(bx + 14, 564, "Fuse the +12V_IN feed (5 A) at the source; F1 is the board-level 0.5 A fuse for the electronics only.", 11, c="#555")

# =================================================================== points
py = 610
block(bx, py, 900, 340, "POINTS INPUT   (points open ⇒ PT_G / GPIO4 goes LOW)")
y = py + 190
netlabel(bx + 14, y, "POINTS_RAW"); wire(bx + 110, y, bx + 130, y)
sym("R", bx + 130, y, "h", "R2", "10k", 64)
n1 = bx + 250
wire(bx + 194, y, n1, y); dot(n1, y); text(n1 - 6, y - 18, "PT_B", 11.5, "end", c=NETC, bold=True)
sym("R", n1, y, "v", "R3", "22k", 64); gnd(n1, y + 64)
n2 = n1 + 130
wire(n1, y, n2 + 60, y); dot(n2, y)
sym("D", n2, y, "v", "D3", "1N4148W", 64, rev=True); gnd(n2, y + 64)
Q = npn(n2 + 60, y, "Q2", "BC847B")
b_, c_, e_ = Q
gnd(e_[0], e_[1])
yc = y - 60
wire(c_[0], c_[1], c_[0], yc); dot(c_[0], yc)
text(c_[0] - 8, yc + 16, "PT_C", 11.5, "end", c=NETC, bold=True)
sym("R", c_[0], yc - 84, "v", "R4", "4.7k", 56); wire(c_[0], yc - 28, c_[0], yc); netlabel(c_[0], yc - 84, "+3V3") if False else None
wire(c_[0], yc - 84, c_[0], yc - 96); netlabel(c_[0], yc - 96, "+3V3")
r5 = c_[0] + 60
wire(c_[0], yc, r5, yc)
sym("R", r5, yc, "h", "R5", "1k", 64)
n3 = r5 + 120
wire(r5 + 64, yc, n3 + 60, yc); dot(n3, yc)
sym("C", n3, yc, "v", "C5", "10nF", 64); gnd(n3, yc + 64)
netlabel(n3 + 60, yc, "PT_G → U1 pad 8")
text(bx + 14, py + 300, "R1 (above) gives ≈12 mA wetting current through the points. The BC847 inverts, so points open ⇒ PT_C low.", 11, c="#555")
text(bx + 14, py + 317, "R5 + C5 form a 10 µs filter; the firmware also rejects contact bounce. D3 clamps negative spikes at the base.", 11, c="#555")

# =================================================================== analog inputs
cx = 1440
block(cx, 80, 740, 560, "ANALOG INPUTS   (ADC1 pins GPIO0 / 1 / 3 — ADC2 is unusable with WiFi on)")


def divider(y, inn, rs, rsv, rh, rhv, cref, dref, node, outl):
    netlabel(cx + 14, y, inn)
    wire(cx + 14 + 7.4 * len(inn) + 16, y, cx + 150, y)
    sym("R", cx + 150, y, "h", rs, rsv, 64)
    n = cx + 270
    wire(cx + 214, y, n, y); dot(n, y)
    text(n - 6, y - 20, node, 11.5, "end", c=NETC, bold=True)
    sym("R", n, y, "v", rh, rhv, 64); gnd(n, y + 64)
    n2 = n + 110; wire(n, y, n2, y); dot(n2, y)
    sym("C", n2, y, "v", cref, "100nF", 64); gnd(n2, y + 64)
    n3 = n2 + 110; wire(n2, y, n3, y); dot(n3, y)
    sym("Z", n3, y, "v", dref, "BZT52C3V6", 64, rev=True); gnd(n3, y + 64)
    wire(n3, y, n3 + 60, y); netlabel(n3 + 60, y, outl)


divider(170, "LPG_IN", "R6", "100k", "R7", "22k", "C6", "D4", "LPG_S", "→ U1 pad 4 (GPIO0)")
divider(320, "VBAT_F", "R8", "100k", "R9", "22k", "C7", "D5", "VB_S", "→ U1 pad 7 (GPIO3)")
divider(470, "MAP_SIG", "R11", "10k", "R12", "15k", "C8", "D6", "MAP_S", "→ U1 pad 5 (GPIO1)")
text(cx + 14, 580, "3.6 V zeners protect the pins from wiring faults.  VBAT 12 V → 2.2 V,  LPG ≥ 7 V = on / ≤ 4 V = off,  MAP 4.8 V → 2.9 V.", 11, c="#555")
text(cx + 14, 597, "MAP sensor: any linear 5 V 1-bar analog type (e.g. GM-style), calibrated with the two-point tool in the web app.", 11, c="#555")

# =================================================================== coil driver
dy = 670
block(cx, dy, 1090, 500, "COIL DRIVER   — CD74HCT221 = on-time guard (≈ 20 ms) + 5 V gate drive")
netlabel(cx + 14, dy + 190, "CMD_G")
wire(cx + 96, dy + 190, cx + 120, dy + 190)
sym("R", cx + 120, dy + 190, "h", "R10", "100R", 64)
nc_ = cx + 240
wire(cx + 184, dy + 190, nc_, dy + 190); dot(nc_, dy + 190)
text(nc_ - 6, dy + 172, "CMD", 11.5, "end", c=NETC, bold=True)
sym("R", nc_, dy + 190, "v", "R13", "10k", 60); gnd(nc_, dy + 250)
ix, iy = cx + 330, dy + 130
P = ic(ix, iy, 150, 210, "U3", "CD74HCT221M96 (SOIC-16)", [(1, "1A", 40), (2, "1B", 80), (3, "/1R", 120)],
       [(13, "1Q", 40), (14, "1Cx", 90), (15, "1RxCx", 130)], top=[(16, "VCC", 40), (9, "2A", 110)], bottom=[(8, "GND", 40), (10, "2B", 75), (11, "/2R", 110)])
bus = cx + 292
wire(nc_, dy + 190, bus, dy + 190); wire(bus, P[2][1], bus, P[3][1]); dot(bus, dy + 190)
wire(bus, P[2][1], P[2][0], P[2][1]); wire(bus, P[3][1], P[3][0], P[3][1])
wire(P[1][0], P[1][1], cx + 310, P[1][1]); gnd(cx + 310, P[1][1])
vtop = iy - 52
wire(P[16][0], P[16][1], P[16][0], vtop); wire(P[9][0], P[9][1], P[9][0], vtop); wire(P[16][0], vtop, P[9][0], vtop); vcc((P[16][0] + P[9][0]) / 2, vtop, "+5V")
vb = iy + 210 + 52
for k in (8, 10, 11): wire(P[k][0], P[k][1], P[k][0], vb)
wire(P[8][0], vb, P[11][0], vb); gnd(P[10][0], vb)
text(ix - 6, vb + 40, "NC pins: 4, 5, 6, 7, 12.  Section 2 parked (A high, B/R low).", 11, c="#666")
c9x = ix + 230
sym("C", c9x, vtop - 2, "v", "C9", "100nF (U3 bypass)", 50); vcc(c9x, vtop - 2, "+5V"); gnd(c9x, vtop + 48)
rx = P[15][0]
wire(rx, P[15][1], rx + 40, P[15][1]); dot(rx + 40, P[15][1])
text(rx + 52, P[15][1] + 16, "REXT", 11.5, c=NETC, bold=True)
wire(P[14][0], P[14][1], rx + 40, P[14][1])
sym("C", rx + 40, P[14][1], "v", "C10", "2.2µF X7R", 40)
text(rx + 52, P[14][1] - 10, "CEXT", 11.5, c=NETC, bold=True)
wire(rx + 40, P[15][1], rx + 40, P[15][1] + 30)
sym("R", rx + 40, P[15][1] + 30, "v", "R14", "15k", 56); wire(rx + 40, P[15][1] + 86, rx + 40, P[15][1] + 98); netlabel(rx + 40, P[15][1] + 98, "+5V")
gy = P[13][1]
wire(P[13][0], gy, rx + 150, gy)
sym("R", rx + 150, gy, "h", "R15", "100R", 64)
ng = rx + 270
wire(rx + 214, gy, ng, gy); dot(ng, gy)
text(ng - 6, gy - 20, "GATE", 11.5, "end", c=NETC, bold=True)
sym("R", ng, gy, "v", "R16", "10k", 60); gnd(ng, gy + 60)
Qi = igbt(ng + 50, gy, "Q1", "ISL9V3040P3")
wire(ng, gy, ng + 50, gy)
wire(Qi[1][0], Qi[1][1], Qi[1][0], Qi[1][1] - 14); netlabel(Qi[1][0], Qi[1][1] - 14, "COIL_N")
gnd(Qi[2][0], Qi[2][1])
nx_ = cx + 780
text(nx_, dy + 400, "Q1: TO-220 lies flat on the case floor, tab insulated", 11.5, c="#333", bold=True)
text(nx_, dy + 416, "(pad + shoulder washer), inside the PCB cut-out;", 11.5, c="#333")
text(nx_, dy + 432, "leads bent up through the pads beside the cut-out.", 11.5, c="#333")
text(nx_, dy + 448, "Tab = collector = coil −: up to ~400 V!", 11.5, c="#a00", bold=True)
text(cx + 14, dy + 470, "Q (13) is high only while CMD is high AND for at most 0.7·15k·C(2.2 µF, derated) ≈ 16–23 ms, then drops until CMD toggles: a hung MCU cannot cook the coil.", 11, c="#555")
text(cx + 14, dy + 487, "R13 / R16 (10k pull-downs) keep the coil off while the MCU resets or the 5 V rail is missing.", 11, c="#555")

# =================================================================== MCU
mx = 2210
block(mx, 80, 330, 560, "MCU MODULE")
bx_, by_ = mx + 95, 130
box(bx_, by_, 150, 440, fill="#eaffee")
text(bx_ + 75, by_ + 24, "U1", 14, "middle", bold=True); text(bx_ + 75, by_ + 41, "ESP32-C3-Zero", 12, "middle")
text(bx_ + 75, by_ + 56, "2×9 sockets, USB-C up", 10.5, "middle", italic=True)
Lp = [(1, "5V", "+5V"), (2, "GND", None), (3, "3V3", "+3V3"), (4, "IO0", "LPG_S"), (5, "IO1", "MAP_S"), (6, "IO2", None), (7, "IO3", "VB_S"), (8, "IO4", "PT_G"), (9, "IO5", None)]
Rp = [(18, "IO21", None), (17, "IO20", None), (16, "IO19", None), (15, "IO18", None), (14, "IO10 RGB LED", None), (13, "IO9 BOOT", None), (12, "IO8", None), (11, "IO7", None), (10, "IO6", "CMD_G")]
for i, (n, nm, net) in enumerate(Lp):
    yy = by_ + 85 + i * 40
    line(bx_ - 22, yy, bx_, yy); text(bx_ + 8, yy + 4, nm, 12); text(bx_ - 4, yy - 5, str(n), 10, "end", c="#666")
    if nm == "GND": gnd(bx_ - 22, yy)
    elif net: netlabel(bx_ - 22, yy, net, "l")
    else: text(bx_ - 28, yy + 4, "nc", 10.5, "end", c="#999")
for i, (n, nm, net) in enumerate(Rp):
    yy = by_ + 85 + i * 40
    line(bx_ + 150, yy, bx_ + 172, yy); text(bx_ + 142, yy + 4, nm, 11.5, "end"); text(bx_ + 154, yy - 5, str(n), 10, c="#666")
    if net: netlabel(bx_ + 172, yy, net, "r")
    else: text(bx_ + 178, yy + 4, "nc", 10.5, c="#999")
text(mx + 10, 600, "Pad order per the Waveshare list (anticlockwise from the USB connector).", 10.5, c="#555")
text(mx + 10, 615, "VERIFY against the silkscreen on your module before soldering.", 10.5, c="#a00", bold=True)

# =================================================================== firmware pin box
block(20, 660, 250, 300, "FIRMWARE PINS (config.h)")
for i, t in enumerate(["PIN_LPG    = 0   (U1 pad 4)", "PIN_MAP    = 1   (pad 5)", "PIN_VBAT   = 3   (pad 7)", "PIN_POINTS = 4   (pad 8)", "PIN_COIL   = 6   (pad 10)",
                       "PIN_LED    = 10  (on-board RGB)", "", "ADC1 = GPIO0-4 only"]):
    text(34, 704 + i * 21, t, 12, c="#222")
add('</svg>')
open(OUT, "w", encoding="utf-8").write("\n".join(o))
print("wrote", OUT)
