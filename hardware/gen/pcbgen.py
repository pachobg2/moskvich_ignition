#!/usr/bin/env python3
"""Moskvich ECU board generator (SMD, two-sided): parts, placement, auto-router, DRC, KiCad + SVG + BOM export.

    python hardware/gen/pcbgen.py            (needs numpy, scipy, shapely)

Passives/ICs are SMD (0603 / SOT-23 / SOIC-16 / SMA / SMB), on both sides of the board; the connectors, the
ESP32-C3-Zero sockets and the ignition IGBT (TO-220, lies in a board cut-out) are through-hole.
"""
import heapq, math, os, sys, csv
import numpy as np
from scipy import ndimage
from shapely.geometry import Point, LineString, box
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)

G = 0.127
BW, BH = 48.0, 41.8
NX, NY = int(round(BW / G)) + 1, int(round(BH / G)) + 1
CLR = 0.20                  # copper clearance
HV_CLR = 0.70               # clearance around the coil net (collector, up to ~400 V flyback)
EDGE = 0.50
W_SIG, W_PWR, W_HEAVY = 0.20, 0.40, 1.50
VIA_D, VIA_DRILL = 0.8, 0.4
LN = ("F.Cu", "B.Cu")


def rcells(w, extra=0.0):
    """dilation radius (cells) for a track of width w to keep `CLR + extra` from foreign copper (raster adds 0.5 cell)"""
    return int(math.ceil((CLR + extra + w / 2) / G + 0.5))


R_VIA = int(math.ceil((CLR + VIA_D / 2) / G + 0.5))
HV_EXTRA = int(math.ceil((HV_CLR - CLR) / G))


# ----------------------------------------------------------------- footprints
def two(pitch, w, h, name):
    return dict(name=name, smd=True, pads=[("1", -pitch / 2, 0, "rect", w, h, 0), ("2", pitch / 2, 0, "rect", w, h, 0)])


FP = {
    "R0603": two(1.524, 0.9, 1.0, "R_0603"), "C0603": two(1.524, 0.9, 1.0, "C_0603"),
    "C0805": two(2.032, 1.1, 1.4, "C_0805"),
    "R1206": two(3.048, 1.3, 1.8, "R_1206"), "F1206": two(3.048, 1.3, 1.8, "Fuse_1206"),
    "C1210": two(3.81, 1.5, 2.8, "C_1210"),
    "D123": two(3.302, 0.9, 1.2, "D_SOD-123"), "DSMA": two(4.064, 2.5, 1.8, "D_SMA"), "DSMB": two(4.318, 2.5, 2.4, "D_SMB"),
    "L4040": two(3.302, 1.5, 3.4, "L_4x4"),
    "SOT23": dict(name="SOT-23", smd=True, pads=[("1", -0.95, 1.0, "rect", 0.8, 1.0, 0), ("2", 0.95, 1.0, "rect", 0.8, 1.0, 0), ("3", 0.0, -1.0, "rect", 0.8, 1.0, 0)]),
    "SOT23_6": dict(name="SOT-23-6", smd=True, pads=[("1", -0.95, 1.25, "rect", 0.6, 1.0, 0), ("2", 0.0, 1.25, "rect", 0.6, 1.0, 0), ("3", 0.95, 1.25, "rect", 0.6, 1.0, 0),
                                                       ("4", 0.95, -1.25, "rect", 0.6, 1.0, 0), ("5", 0.0, -1.25, "rect", 0.6, 1.0, 0), ("6", -0.95, -1.25, "rect", 0.6, 1.0, 0)]),
    "SOIC16": dict(name="SOIC-16_3.9x9.9mm_P1.27mm", smd=True,
                   pads=[(str(n), -2.7, -4.445 + 1.27 * (n - 1), "rect", 1.5, 0.6, 0) for n in range(1, 9)] +
                        [(str(n), 2.7, 4.445 - 1.27 * (n - 9), "rect", 1.5, 0.6, 0) for n in range(9, 17)]),
    "TERM35": dict(name="TerminalBlock_P3.50_1x04", smd=False, court=(-3.8, -2.0, 3.8, 12.5),
                   pads=[(str(i + 1), 0, 3.5 * i, "rect" if i == 0 else "circle", 2.0, 2.0, 1.2) for i in range(4)]),
    "MODULE": dict(name="ESP32-C3-Zero_2x9_Socket", smd=False,
                   pads=[(str(i + 1), 0, 2.54 * i, "rect" if i == 0 else "circle", 1.7, 1.7, 1.0) for i in range(9)] +
                        [(str(18 - i), 15.24, 2.54 * i, "circle", 1.7, 1.7, 1.0) for i in range(9)], court=(-1.38, -1.6, 16.62, 21.9)),
    "TO220": dict(name="TO-220_LayDown_Cutout", smd=False, court=(3.0, -6.5, 21.0, 6.5),
                  pads=[("1", 0, -2.54, "circle", 1.7, 1.7, 1.0), ("2", 0, 0, "circle", 1.7, 1.7, 1.0), ("3", 0, 2.54, "circle", 1.7, 1.7, 1.0)]),
    "MH": dict(name="MountingHole_2.7_M2.5", smd=False, pads=[("", 0, 0, "npth", 2.7, 2.7, 2.7)]),
}


def rot(x, y, a):
    a = a % 360
    return {0: (x, y), 90: (-y, x), 180: (-x, -y), 270: (y, -x)}[a]


def pad_geom(p):
    if p["shape"] == "rect":
        return box(p["x"] - p["w"] / 2, p["y"] - p["h"] / 2, p["x"] + p["w"] / 2, p["y"] + p["h"] / 2)
    return Point(p["x"], p["y"]).buffer(p["w"] / 2, 24)


class Part:
    def __init__(self, ref, value, fp, x, y, r=0, nets=None, side="F", desc=""):
        self.ref, self.value, self.fp, self.r, self.side, self.desc = ref, value, fp, r, side, desc
        self.x, self.y = round(x, 3), round(y, 3)
        self.nets = nets or {}
        d = FP[fp]
        self.fpname, self.smd = d["name"], d["smd"]
        self.pads = []
        for (n, px, py, shape, w, h, drill) in d["pads"]:
            if side == "B": px = -px
            rx, ry = rot(px, py, r)
            if shape == "rect" and r % 180 == 90: w, h = h, w
            layer = (LN[0] if side == "F" else LN[1]) if d["smd"] else "TH"
            self.pads.append(dict(ref=ref, name=n, x=round(self.x + rx, 4), y=round(self.y + ry, 4), shape=shape, w=w, h=h, drill=drill,
                                  net=self.nets.get(n), lx=rx, ly=ry, layer=layer))
        xs0 = [p["x"] - p["w"] / 2 for p in self.pads] + [p["x"] + p["w"] / 2 for p in self.pads]
        ys0 = [p["y"] - p["h"] / 2 for p in self.pads] + [p["y"] + p["h"] / 2 for p in self.pads]
        bb = box(min(xs0), min(ys0), max(xs0), max(ys0))
        if "court" in d:
            c = d["court"]
            cx = [rot(c[0], c[1], r), rot(c[2], c[3], r)]
            bb = unary_union([bb, box(self.x + min(cx[0][0], cx[1][0]), self.y + min(cx[0][1], cx[1][1]), self.x + max(cx[0][0], cx[1][0]), self.y + max(cx[0][1], cx[1][1]))])
        self.court = bb.buffer(0.1)                                   # pads + body
        self.court_pads = unary_union([pad_geom(q) for q in self.pads if q["shape"] != "npth"] or [bb]).buffer(0.1)                              # copper only (what a part on the other side can collide with)


# ------------------------------------------------------------------ the design
PARTS = []
A = PARTS.append


def smdpart(ref, val, fp, x, y, r, nets, side="B", desc=""):
    A(Part(ref, val, fp, x, y, r, nets, side, desc))


# --- through-hole parts / top side
A(Part("U1", "ESP32-C3-Zero", "MODULE", 5.08, 3.81, 0,
       {"1": "+5V", "2": "GND", "3": "+3V3", "4": "LPG_S", "5": "MAP_S", "7": "VB_S", "8": "PT_G", "10": "CMD_G"}, "F", "Waveshare ESP32-C3-Zero on 2x 1x9 female headers"))
A(Part("J2", "Sensors", "TERM35", 43.5, 3.81, 0, {"1": "LPG_IN", "2": "+5V", "3": "MAP_SIG", "4": "GND"}, "F", "LPG / MAP sensor terminal"))
A(Part("J1", "Power", "TERM35", 43.5, 22.86, 0, {"1": "POINTS_RAW", "2": "+12V_IN", "3": "GND", "4": "COIL_N"}, "F", "Points / +12V / GND / Coil-"))
A(Part("Q1", "ISL9V3040P3", "TO220", 24.64, 34.29, 180, {"1": "GATE", "2": "COIL_N", "3": "GND"}, "F", "ignition IGBT, lies flat on the case floor in the cut-out"))
for i, (mx, my) in enumerate([(1.9, 15.0), (45.8, 39.8), (45.8, 18.5)]):
    A(Part(f"H{i + 1}", "M2.5", "MH", mx, my, 0, {}, "F"))
# --- buck regulator (top)
A(Part("U2", "AP63205WU-7", "SOT23_6", 31.5, 12.0, 0, {"1": "+5V", "2": "VIN_P", "3": "VIN_P", "4": "GND", "5": "SW", "6": "BST"}, "F", "5 V 2 A buck, 3.8-32 V in"))
A(Part("L1", "4.7uH", "L4040", 31.5, 6.2, 0, {"1": "SW", "2": "+5V"}, "F", "shielded 4x4 mm, >= 2 A sat, DCR < 100 mOhm"))
A(Part("C11", "100nF", "C0603", 28.0, 10.6, 90, {"1": "BST", "2": "SW"}, "F"))
A(Part("C3", "22uF 10V", "C0805", 36.2, 6.8, 90, {"1": "+5V", "2": "GND"}, "F"))
A(Part("C4", "22uF 10V", "C0805", 36.2, 10.6, 90, {"1": "+5V", "2": "GND"}, "F"))
A(Part("C1", "10uF 50V", "C1210", 31.5, 16.4, 0, {"1": "VIN_P", "2": "GND"}, "F"))
A(Part("C2", "100nF", "C0603", 35.6, 16.4, 0, {"1": "VIN_P", "2": "GND"}, "F"))
A(Part("D2", "SMBJ24A", "DSMB", 31.0, 20.0, 0, {"1": "VIN_P", "2": "GND"}, "F", "TVS, clamps load dump below the buck's 40 V limit"))
A(Part("D1", "SS34", "DSMA", 31.0, 23.4, 0, {"1": "VIN_P", "2": "VBAT_F"}, "F", "reverse-polarity Schottky"))
A(Part("F1", "0.5A 32V fuse", "F1206", 35.5, 27.2, 0, {"1": "VBAT_F", "2": "+12V_IN"}, "F", "SMD fuse or PTC, >= 32 V rating"))


def cellb(x0, y0, vals, nin, node, refs):
    rs, rsh = vals
    r1, r2, c1, d1 = refs
    smdpart(r1, rs, "R0603", x0 + 0.762, y0, 0, {"1": nin, "2": node})
    smdpart(r2, rsh, "R0603", x0 + 2.4, y0 + 2.6, 90, {"1": node, "2": "GND"})
    smdpart(c1, "100nF", "C0603", x0 + 4.6, y0 + 2.6, 90, {"1": node, "2": "GND"})
    smdpart(d1, "BZT52C3V6", "D123", x0 + 6.8, y0 + 3.2, 90, {"1": node, "2": "GND"})


cellb(8.0, 4.0, ("100k", "22k"), "LPG_IN", "LPG_S", ("R6", "R7", "C6", "D4"))
cellb(8.0, 11.4, ("100k", "22k"), "VBAT_F", "VB_S", ("R8", "R9", "C7", "D5"))
cellb(8.0, 18.8, ("10k", "15k"), "MAP_SIG", "MAP_S", ("R11", "R12", "C8", "D6"))
smdpart("R1", "1k 0.25W", "R1206", 26.0, 6.0, 0, {"1": "VIN_P", "2": "POINTS_RAW"})
smdpart("R2", "10k", "R0603", 24.0, 10.0, 0, {"1": "POINTS_RAW", "2": "PT_B"})
smdpart("R3", "22k", "R0603", 24.0, 12.6, 0, {"1": "PT_B", "2": "GND"})
smdpart("D3", "1N4148W", "D123", 24.0, 15.6, 0, {"1": "PT_B", "2": "GND"})
smdpart("Q2", "BC847B", "SOT23", 28.0, 9.0, 0, {"1": "PT_B", "2": "GND", "3": "PT_C"})
smdpart("R4", "4.7k", "R0603", 32.0, 8.4, 90, {"1": "PT_C", "2": "+3V3"})
smdpart("R5", "1k", "R0603", 34.0, 8.4, 90, {"1": "PT_C", "2": "PT_G"})
smdpart("C5", "10nF", "C0603", 37.6, 8.4, 90, {"1": "GND", "2": "PT_G"})
smdpart("U3", "SN74HCT123D", "SOIC16", 32.0, 25.5, 180,
        {"1": "GND", "2": "CMD", "3": "CMD", "8": "GND", "9": "+5V", "10": "GND", "11": "GND", "13": "GATE_DRV", "14": "CEXT", "15": "REXT", "16": "+5V"},
        desc="retriggerable one-shot = coil on-time guard + 5 V gate driver")
smdpart("R10", "100R", "R0603", 27.0, 26.5, 90, {"1": "CMD_G", "2": "CMD"})
smdpart("R13", "10k", "R0603", 27.0, 29.8, 90, {"1": "CMD", "2": "GND"})
smdpart("C9", "100nF", "C0603", 36.8, 20.5, 90, {"1": "+5V", "2": "GND"})
smdpart("R14", "22k", "R0603", 39.6, 31.0, 90, {"1": "REXT", "2": "+5V"})
smdpart("C10", "2.2uF 25V X7R", "C0805", 39.6, 27.6, 90, {"1": "CEXT", "2": "REXT"})
smdpart("R15", "100R", "R0603", 37.2, 26.135, 0, {"1": "GATE", "2": "GATE_DRV"})
smdpart("R16", "10k", "R0603", 33.5, 33.5, 0, {"1": "GATE", "2": "GND"})

WIN = box(24.64 - 21.0, 34.29 - 6.5, 24.64 - 3.0, 34.29 + 6.5)          # IGBT cut-out (leads on the right)
MODBODY = box(5.08 - 1.38, 3.81 - 1.6, 5.08 + 16.62, 3.81 + 21.9)
OUTLINE = box(0, 0, BW, BH)
WIDTH = {"+5V": W_PWR, "VIN_P": W_PWR, "VBAT_F": W_PWR, "+12V_IN": W_PWR, "SW": W_PWR}


# ------------------------------------------------------------------ geometry db
def pad_geom(p):
    if p["shape"] == "rect":
        return box(p["x"] - p["w"] / 2, p["y"] - p["h"] / 2, p["x"] + p["w"] / 2, p["y"] + p["h"] / 2)
    return Point(p["x"], p["y"]).buffer(p["w"] / 2, 24)


def pad_layers(p):
    return LN if p["layer"] == "TH" else (p["layer"],)


def bbox(cx0, cy0, cx1, cy1):
    return max(0, int(cx0 / G) - 1), min(NX - 1, int(cx1 / G) + 2), max(0, int(cy0 / G) - 1), min(NY - 1, int(cy1 / G) + 2)


def mark_circle(a, cx, cy, r):
    r2 = r + G * 0.5
    x0, x1, y0, y1 = bbox(cx - r2, cy - r2, cx + r2, cy + r2)
    xs = np.arange(x0, x1 + 1) * G; ys = np.arange(y0, y1 + 1) * G
    a[x0:x1 + 1, y0:y1 + 1] |= ((xs[:, None] - cx) ** 2 + (ys[None, :] - cy) ** 2) <= r2 * r2


def mark_rect(a, x1_, y1_, x2_, y2_):
    m = G * 0.5
    x0, x1, y0, y1 = bbox(x1_ - m, y1_ - m, x2_ + m, y2_ + m)
    xs = np.arange(x0, x1 + 1) * G; ys = np.arange(y0, y1 + 1) * G
    a[x0:x1 + 1, y0:y1 + 1] |= ((xs >= x1_ - m) & (xs <= x2_ + m))[:, None] & ((ys >= y1_ - m) & (ys <= y2_ + m))[None, :]


def mark_seg(a, x1_, y1_, x2_, y2_, hw):
    r = hw + G * 0.5
    x0, x1, y0, y1 = bbox(min(x1_, x2_) - r, min(y1_, y2_) - r, max(x1_, x2_) + r, max(y1_, y2_) + r)
    xs = np.arange(x0, x1 + 1) * G; ys = np.arange(y0, y1 + 1) * G
    px, py = xs[:, None], ys[None, :]
    dx, dy = x2_ - x1_, y2_ - y1_
    L2 = dx * dx + dy * dy
    t = np.clip(((px - x1_) * dx + (py - y1_) * dy) / L2, 0, 1) if L2 > 0 else np.zeros((len(xs), len(ys)))
    a[x0:x1 + 1, y0:y1 + 1] |= ((px - (x1_ + t * dx)) ** 2 + (py - (y1_ + t * dy)) ** 2) <= r * r


class Board:
    def __init__(self):
        self.tracks, self.vias = [], []
        self.pads = [p for part in PARTS for p in part.pads if p["shape"] != "npth"]
        self.holes = [p for part in PARTS for p in part.pads if p["shape"] == "npth"]
        self.cu, self._dil, self._ver = {}, {}, {}
        self.static = {L: np.zeros((NX, NY), bool) for L in LN}
        for p in self.pads:
            p["rnet"] = p["net"] or f"NC_{p['ref']}_{p['name']}"
            for L in pad_layers(p):
                a = self.arr(p["rnet"], L)
                if p["shape"] == "rect": mark_rect(a, p["x"] - p["w"] / 2, p["y"] - p["h"] / 2, p["x"] + p["w"] / 2, p["y"] + p["h"] / 2)
                else: mark_circle(a, p["x"], p["y"], p["w"] / 2)
            self._bump(p["rnet"])
        self._static()

    def arr(self, net, L):
        d = self.cu.setdefault(net, {})
        if L not in d: d[L] = np.zeros((NX, NY), bool)
        return d[L]

    def _bump(self, n): self._ver[n] = self._ver.get(n, 0) + 1

    def _static(self):
        for L in LN:
            a = self.static[L]
            m = int(math.ceil((EDGE + 0.1) / G)) + 1
            a[:m, :] = True; a[-m:, :] = True; a[:, :m] = True; a[:, -m:] = True
            for h in self.holes: mark_circle(a, h["x"], h["y"], 1.35 + CLR + 0.15)
            minx, miny, maxx, maxy = WIN.bounds
            mark_rect(a, minx - 0.8, miny - 0.8, maxx + 0.8, maxy + 0.8)

    def add_track(self, net, L, w, pts):
        self.tracks.append(dict(net=net, layer=L, w=w, pts=pts))
        a = self.arr(net, L)
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]): mark_seg(a, x1, y1, x2, y2, w / 2)
        self._bump(net)

    def add_via(self, net, x, y):
        self.vias.append(dict(net=net, x=x, y=y))
        for L in LN: mark_circle(self.arr(net, L), x, y, VIA_D / 2)
        self._bump(net)

    def dil(self, net, L, r):
        key = (net, L, r)
        c = self._dil.get(key)
        if c and c[0] == self._ver.get(net, 0): return c[1]
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
        st = (xx * xx + yy * yy) <= r * r + 0.5
        out = ndimage.binary_dilation(self.arr(net, L), structure=st)
        self._dil[key] = (self._ver.get(net, 0), out)
        return out

    def blocked(self, net, L, r):
        b = self.static[L].copy()
        for m, d in self.cu.items():
            if m == net or L not in d: continue
            b |= self.dil(m, L, r + (HV_EXTRA if m == "COIL_N" else 0))
        return b


# ------------------------------------------------------------------ router
def pad_cell(p): return (int(round(p["x"] / G)), int(round(p["y"] / G)))


def astar(B, net, sources, goals, width, layers=(0, 1), r=None, via_ok=True):
    r = r or rcells(width)
    blk = [B.blocked(net, L, r) for L in LN]
    blkv = [B.blocked(net, L, R_VIA) for L in LN]
    gl = set(goals)
    gx = [g[1] for g in gl]; gy = [g[2] for g in gl]
    gmin = (min(gx), min(gy), max(gx), max(gy))

    def h(ix, iy):
        return max(gmin[0] - ix, 0, ix - gmin[2]) + max(gmin[1] - iy, 0, iy - gmin[3])
    openh, g, came = [], {}, {}
    for s in sources:
        if s[0] in layers:
            g[s] = 0.0; heapq.heappush(openh, (h(s[1], s[2]), 0.0, s))
    while openh:
        f, gc, cur = heapq.heappop(openh)
        if gc > g.get(cur, 1e18): continue
        if cur in gl:
            path = [cur]
            while cur in came: cur = came[cur]; path.append(cur)
            return path[::-1]
        L, ix, iy = cur
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = ix + dx, iy + dy
            if not (0 <= nx < NX and 0 <= ny < NY): continue
            n = (L, nx, ny)
            if blk[L][nx, ny] and n not in gl: continue
            c = gc + (1.0 if ((dx != 0) == (L == 0)) else 1.7)
            if c < g.get(n, 1e18):
                g[n] = c; came[n] = cur; heapq.heappush(openh, (c + h(nx, ny), c, n))
        if via_ok and len(layers) == 2:
            o = 1 - L
            n = (o, ix, iy)
            if not blkv[0][ix, iy] and not blkv[1][ix, iy]:
                c = gc + 8.0
                if c < g.get(n, 1e18):
                    g[n] = c; came[n] = cur; heapq.heappush(openh, (c + h(ix, iy), c, n))
    return None


def emit_path(B, net, path, width, end_pad=None):
    runs = []
    for a, b in zip(path, path[1:]):
        if a[0] != b[0]:
            B.add_via(net, a[1] * G, a[2] * G); continue
        d = (b[1] - a[1], b[2] - a[2])
        if runs and runs[-1]["layer"] == a[0] and runs[-1]["dir"] == d and runs[-1]["end"] == (a[1], a[2]): runs[-1]["end"] = (b[1], b[2])
        else: runs.append(dict(layer=a[0], dir=d, start=(a[1], a[2]), end=(b[1], b[2])))
    polys = []
    for r in runs:
        if polys and polys[-1]["layer"] == r["layer"] and polys[-1]["pts"][-1] == r["start"]: polys[-1]["pts"].append(r["end"])
        else: polys.append(dict(layer=r["layer"], pts=[r["start"], r["end"]]))
    for p in polys: B.add_track(net, LN[p["layer"]], width, [(x * G, y * G) for x, y in p["pts"]])
    if end_pad is not None:                                  # exact connection to an off-grid pad centre
        L = path[-1][0]
        cx, cy = path[-1][1] * G, path[-1][2] * G
        if math.hypot(cx - end_pad["x"], cy - end_pad["y"]) > 0.004:
            B.add_track(net, LN[L], min(width, 0.2), [(cx, cy), (end_pad["x"], end_pad["y"])])


def route_net(B, net, pads, width, log, layers=(0, 1)):
    cells = [pad_cell(p) for p in pads]
    nodes = {(L, *cells[0]) for L in range(2) if LN[L] in pad_layers(pads[0])}
    connected, unrouted = {0}, 0
    while len(connected) < len(pads):
        best = None
        for j in range(len(pads)):
            if j in connected: continue
            d = min(abs(cells[i][0] - cells[j][0]) + abs(cells[i][1] - cells[j][1]) for i in connected)
            if best is None or d < best[0]: best = (d, j)
        j = best[1]
        goals = {(L, *cells[j]) for L in range(2) if LN[L] in pad_layers(pads[j])}
        path = astar(B, net, nodes, goals, width, layers)
        if path is None:
            log.append(f"  !! could not route {net}: pad {pads[j]['ref']}.{pads[j]['name']}")
            unrouted += 1
        else:
            emit_path(B, net, path, width, end_pad=pads[j])
            nodes |= set(path)
        nodes |= goals
        connected.add(j)
    return unrouted


def autoroute():
    B = Board()
    log = []
    by = {}
    for p in B.pads: by.setdefault(p["rnet"], []).append(p)
    prio = ["COIL_N", "+5V", "SW", "VIN_P", "+12V_IN", "VBAT_F", "BST"]
    nets = [n for n in by if len(by[n]) > 1 and n != "GND"]
    nets.sort(key=lambda n: (prio.index(n) if n in prio else 99, sum(abs(p["x"] - by[n][0]["x"]) + abs(p["y"] - by[n][0]["y"]) for p in by[n])))
    for n in nets:
        if n == "COIL_N":
            q = list(by[n])
            miss = 0
            cells = [pad_cell(p) for p in q]
            goals = {(0, *cells[1])}
            path = astar(B, n, {(0, *cells[0])}, goals, W_HEAVY, layers=(0,), r=rcells(W_HEAVY, HV_CLR - CLR), via_ok=False)
            if path is None:
                path = astar(B, n, {(0, *cells[0])}, goals, W_HEAVY, layers=(0,), r=rcells(W_HEAVY, 0.3), via_ok=False)
                log.append("  (coil track routed with reduced HV clearance)")
            if path is None: miss = 1; log.append("  !! coil track failed")
            else: emit_path(B, n, path, W_HEAVY, end_pad=q[1])
            log.append(f"routed {n:12s} pads={len(q)} unrouted={miss}")
            continue
        miss = route_net(B, n, by[n], WIDTH.get(n, W_SIG), log)
        log.append(f"routed {n:12s} pads={len(by[n]):2d} unrouted={miss}")
    return B, by, log


# ------------------------------------------------------------------ geometry, pours, DRC
def geoms(B):
    G_ = {}
    def add(net, L, g): G_.setdefault((net, L), []).append(g)
    for p in B.pads:
        for L in pad_layers(p): add(p["rnet"], L, pad_geom(p))
    for t in B.tracks: add(t["net"], t["layer"], LineString(t["pts"]).buffer(t["w"] / 2, 8))
    for v in B.vias:
        for L in LN: add(v["net"], L, Point(v["x"], v["y"]).buffer(VIA_D / 2, 16))
    return {k: unary_union(v) for k, v in G_.items()}


def pour(B, geo):
    res = {}
    base = OUTLINE.buffer(-EDGE)
    for L in LN:
        r = base.difference(WIN.buffer(EDGE + 0.3))
        for (n, l), g in geo.items():
            if l == L and n != "GND": r = r.difference(g.buffer((HV_CLR if n == "COIL_N" else CLR) + 0.01, 16))
        for h in B.holes: r = r.difference(Point(h["x"], h["y"]).buffer(1.35 + CLR, 16))
        res[L] = r
    return res


def polys_of(g):
    return list(g.geoms) if hasattr(g, "geoms") else [g]


def drc(B, geo, pours):
    errs = []
    nets = sorted({n for n, _ in geo})
    for L in LN:
        items = {n: geo[(n, L)] for n in nets if (n, L) in geo}
        names = list(items)
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                need = HV_CLR if "COIL_N" in (names[i], names[j]) else CLR
                d = items[names[i]].distance(items[names[j]])
                if d < need - 0.004: errs.append(f"{L}: {names[i]} <-> {names[j]} clearance {d:.3f} < {need}")
        for n, g in items.items():
            if not OUTLINE.buffer(-(EDGE - 0.05)).contains(g): errs.append(f"{L}: {n} too close to the board edge")
            if g.intersects(WIN.buffer(0.2)): errs.append(f"{L}: {n} inside the IGBT cut-out")
    th = [p for p in B.pads if p["layer"] == "TH"]
    for i in range(len(th)):
        for j in range(i + 1, len(th)):
            a, b = th[i], th[j]
            if math.hypot(a["x"] - b["x"], a["y"] - b["y"]) < (a["drill"] + b["drill"]) / 2 + 0.3: errs.append(f"holes too close {a['ref']}.{a['name']} / {b['ref']}.{b['name']}")
    for i in range(len(PARTS)):
        for j in range(i + 1, len(PARTS)):
            a, b = PARTS[i], PARTS[j]
            if a.smd and b.smd and a.side != b.side: continue
            ca, cb = a.court, b.court
            if a.smd and not b.smd and a.side == "B": cb = b.court_pads    # THT body is on top; only its pins reach the bottom
            if b.smd and not a.smd and b.side == "B": ca = a.court_pads
            if ca.intersects(cb): errs.append(f"footprints overlap: {a.ref} / {b.ref}")
        if not OUTLINE.buffer(-0.2).contains(PARTS[i].court) and not PARTS[i].ref.startswith("H"):
            errs.append(f"{PARTS[i].ref} courtyard leaves the board")
        if PARTS[i].court.intersects(WIN) and PARTS[i].ref != "Q1": errs.append(f"{PARTS[i].ref} inside the cut-out")
    for n in nets:
        if n.startswith("NC_"): continue
        pn = [p for p in B.pads if p["rnet"] == n]
        if len(pn) < 2: continue
        elems = []
        for p in pn: elems.append(({L: pad_geom(p) for L in pad_layers(p)}, ("pad", p)))
        for t in B.tracks:
            if t["net"] == n: elems.append(({t["layer"]: LineString(t["pts"]).buffer(t["w"] / 2, 8)}, ("trk", t)))
        for v in B.vias:
            if v["net"] == n:
                gg = Point(v["x"], v["y"]).buffer(VIA_D / 2, 16); elems.append(({L: gg for L in LN}, ("via", v)))
        if n == "GND":
            for L, r in pours.items():
                for pg in polys_of(r):
                    if pg.area > 0.5: elems.append(({L: pg}, ("pour", L)))
        par = list(range(len(elems)))
        def f(x):
            while par[x] != x: par[x] = par[par[x]]; x = par[x]
            return x
        for i in range(len(elems)):
            for j in range(i + 1, len(elems)):
                if any(L in elems[j][0] and elems[i][0][L].intersects(elems[j][0][L]) for L in elems[i][0]): par[f(i)] = f(j)
        roots = {f(i) for i, e in enumerate(elems) if e[1][0] == "pad"}
        if len(roots) > 1:
            groups = {}
            for i, e in enumerate(elems):
                if e[1][0] == "pad": groups.setdefault(f(i), []).append(f"{e[1][1]['ref']}.{e[1][1]['name']}")
            errs.append(f"OPEN net {n}: {len(roots)} islands: " + " | ".join(",".join(v) for v in groups.values()))
    return errs


def gnd_components(B, pours):
    elems = []
    for p in B.pads:
        if p["rnet"] == "GND": elems.append(({L: pad_geom(p) for L in pad_layers(p)}, ("pad", p)))
    for t in B.tracks:
        if t["net"] == "GND": elems.append(({t["layer"]: LineString(t["pts"]).buffer(t["w"] / 2, 8)}, ("trk", t)))
    for v in B.vias:
        if v["net"] == "GND":
            gg = Point(v["x"], v["y"]).buffer(VIA_D / 2, 16); elems.append(({L: gg for L in LN}, ("via", v)))
    for L, r in pours.items():
        for pg in polys_of(r):
            if pg.area > 0.5: elems.append(({L: pg}, ("pour", (L, pg))))
    par = list(range(len(elems)))
    def f(x):
        while par[x] != x: par[x] = par[par[x]]; x = par[x]
        return x
    for i in range(len(elems)):
        for j in range(i + 1, len(elems)):
            if any(L in elems[j][0] and elems[i][0][L].intersects(elems[j][0][L]) for L in elems[i][0]): par[f(i)] = f(j)
    comps = {}
    for i, e in enumerate(elems): comps.setdefault(f(i), []).append(e)
    return list(comps.values())


def fix_gnd(B, log, rounds=14):
    """connect GND pads that the pours do not reach, by routing them to the main GND component"""
    for _ in range(rounds):
        geo = geoms(B); pours = pour(B, geo)
        comps = gnd_components(B, pours)
        npads = [sum(1 for e in c if e[1][0] == "pad") for c in comps]
        main = comps[int(np.argmax(npads))]
        stray = [(c, n) for c, n in zip(comps, npads) if c is not main and n > 0]
        if not stray: return geo, pours
        sources = set()
        for e in main:
            if e[1][0] == "pour":
                L, pg = e[1][1]
                x0, y0, x1, y1 = pg.bounds
                for ix in range(int(x0 / G), int(x1 / G) + 1, 3):
                    for iy in range(int(y0 / G), int(y1 / G) + 1, 3):
                        if pg.contains(Point(ix * G, iy * G)): sources.add((LN.index(L), ix, iy))
            elif e[1][0] == "pad":
                q = e[1][1]
                for L in pad_layers(q): sources.add((LN.index(L), *pad_cell(q)))
        c, _n = stray[0]
        bad = next(e[1][1] for e in c if e[1][0] == "pad")
        starts = {(LN.index(L), *pad_cell(bad)) for L in pad_layers(bad)}
        path = astar(B, "GND", starts, sources, W_SIG)
        if path is None:
            log.append(f"  !! GND fix failed for {bad['ref']}.{bad['name']}"); break
        emit_path(B, "GND", path, W_SIG, end_pad=None)
        sx, sy = path[0][1] * G, path[0][2] * G
        if math.hypot(sx - bad["x"], sy - bad["y"]) > 0.004:
            B.add_track("GND", LN[path[0][0]], 0.2, [(bad["x"], bad["y"]), (sx, sy)])
        log.append(f"  GND stitch for {bad['ref']}.{bad['name']}")
    geo = geoms(B); return geo, pour(B, geo)


# ------------------------------------------------------------------ KiCad export
NETS = ["GND", "+5V", "+3V3", "+12V_IN", "VBAT_F", "VIN_P", "SW", "BST", "COIL_N", "POINTS_RAW", "PT_B", "PT_C", "PT_G", "LPG_IN", "LPG_S", "MAP_SIG", "MAP_S",
        "VB_S", "CMD_G", "CMD", "GATE_DRV", "GATE", "CEXT", "REXT"]


def kicad(B, pours):
    nid = {n: i + 1 for i, n in enumerate(NETS)}
    o = []
    w = o.append
    w('(kicad_pcb (version 20221018) (generator pcbgen)')
    w('  (general (thickness 1.6))')
    w('  (paper "A4")')
    w('  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (32 "B.Adhes" user "B.Adhesive") (33 "F.Adhes" user "F.Adhesive") (34 "B.Paste" user) (35 "F.Paste" user)'
      ' (36 "B.SilkS" user "B.Silkscreen") (37 "F.SilkS" user "F.Silkscreen") (38 "B.Mask" user) (39 "F.Mask" user) (40 "Dwgs.User" user "User.Drawings")'
      ' (41 "Cmts.User" user "User.Comments") (44 "Edge.Cuts" user) (45 "Margin" user) (46 "B.CrtYd" user "B.Courtyard") (47 "F.CrtYd" user "F.Courtyard") (48 "B.Fab" user) (49 "F.Fab" user))')
    w('  (setup (pad_to_mask_clearance 0.05) (pcbplotparams (layerselection 0x00010fc_ffffffff) (outputdirectory "gerber/")))')
    w('  (net 0 "")')
    for n in NETS: w(f'  (net {nid[n]} "{n}")')
    for part in PARTS:
        back = part.side == "B"
        sl = "B.SilkS" if back else "F.SilkS"
        w(f'  (footprint "{part.fpname}" (layer "{"B.Cu" if back else "F.Cu"}") (at {part.x:.3f} {part.y:.3f})')
        w(f'    (descr "{part.desc or part.value}")')
        mir = " (justify mirror)" if back else ""
        w(f'    (fp_text reference "{part.ref}" (at 0 -2.2) (layer "{sl}") (effects (font (size 0.8 0.8) (thickness 0.12)){mir}))')
        w(f'    (fp_text value "{part.value}" (at 0 2.2) (layer "{"B.Fab" if back else "F.Fab"}") (effects (font (size 0.8 0.8) (thickness 0.12)){mir}))')
        for p in part.pads:
            if p["shape"] == "npth":
                w(f'    (pad "" np_thru_hole circle (at {p["lx"]:.3f} {p["ly"]:.3f}) (size {p["w"]} {p["h"]}) (drill {p["drill"]}) (layers "*.Cu" "*.Mask"))'); continue
            nt = f' (net {nid[p["net"]]} "{p["net"]}")' if p["net"] else ""
            if p["layer"] == "TH":
                shp = "rect" if p["shape"] == "rect" else "circle"
                w(f'    (pad "{p["name"]}" thru_hole {shp} (at {p["lx"]:.3f} {p["ly"]:.3f}) (size {p["w"]} {p["h"]}) (drill {p["drill"]}) (layers "*.Cu" "*.Mask"){nt})')
            else:
                sd = "B" if back else "F"
                w(f'    (pad "{p["name"]}" smd rect (at {p["lx"]:.3f} {p["ly"]:.3f}) (size {p["w"]} {p["h"]}) (layers "{sd}.Cu" "{sd}.Paste" "{sd}.Mask"){nt})')
        w('  )')
    for t in B.tracks:
        for (x1, y1), (x2, y2) in zip(t["pts"], t["pts"][1:]):
            w(f'  (segment (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) (width {t["w"]}) (layer "{t["layer"]}") (net {nid[t["net"]]}))')
    for v in B.vias: w(f'  (via (at {v["x"]:.3f} {v["y"]:.3f}) (size {VIA_D}) (drill {VIA_DRILL}) (layers "F.Cu" "B.Cu") (net {nid[v["net"]]}))')
    def rect(x1, y1, x2, y2):
        for a, b in (((x1, y1), (x2, y1)), ((x2, y1), (x2, y2)), ((x2, y2), (x1, y2)), ((x1, y2), (x1, y1))):
            w(f'  (gr_line (start {a[0]:.3f} {a[1]:.3f}) (end {b[0]:.3f} {b[1]:.3f}) (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))')
    rect(0, 0, BW, BH); rect(*WIN.bounds)
    w('  (gr_text "MOSKVICH ECU rev B" (at 30 40.2) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))')
    for L in LN:
        w(f'  (zone (net {nid["GND"]}) (net_name "GND") (layer "{L}") (hatch edge 0.5) (connect_pads yes (clearance {CLR})) (min_thickness 0.2) (filled_areas_thickness no)'
          f' (fill yes (thermal_gap 0.3) (thermal_bridge_width 0.5))')
        w(f'    (polygon (pts (xy {EDGE} {EDGE}) (xy {BW - EDGE} {EDGE}) (xy {BW - EDGE} {BH - EDGE}) (xy {EDGE} {BH - EDGE}))))')
    w(')')
    return "\n".join(o)


# ------------------------------------------------------------------ SVG
def svg(B, pours, which="top"):
    S = 16.0
    flip = which == "bottom"
    W_, H_ = BW * S, BH * S
    X = lambda x: (BW - x) * S if flip else x * S
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="-12 -12 {W_ + 24} {H_ + 24}" style="width:100%;height:auto;background:#2b2b2b" font-family="Arial">']
    o.append(f'<rect x="0" y="0" width="{W_}" height="{H_}" fill="#0b4f2a" stroke="#e8d44d" stroke-width="2"/>')
    def poly(g, fill, op):
        d = ""
        for p in polys_of(g):
            if p.is_empty or p.area < 0.3: continue
            for ring in [p.exterior] + list(p.interiors):
                d += "M" + " L".join(f"{X(x):.1f},{y * S:.1f}" for x, y in ring.coords) + " Z "
        o.append(f'<path d="{d}" fill="{fill}" fill-opacity="{op}" fill-rule="evenodd"/>')
    order = ["B.Cu", "F.Cu"] if not flip else ["F.Cu", "B.Cu"]
    col = {"F.Cu": "#d9453d", "B.Cu": "#3d7ad9"}
    for L in order:
        top = L == order[1]
        poly(pours[L], col[L], 0.30 if top else 0.14)
        for t in B.tracks:
            if t["layer"] != L: continue
            pts = " ".join(f"{X(x):.1f},{y * S:.1f}" for x, y in t["pts"])
            o.append(f'<polyline points="{pts}" fill="none" stroke="{col[L]}" stroke-opacity="{0.95 if top else 0.35}" stroke-width="{t["w"] * S:.1f}" stroke-linecap="round" stroke-linejoin="round"/>')
        for p in B.pads:
            if L not in pad_layers(p): continue
            fillc = "#d4b24a" if top else "#7d6a2c"
            if p["shape"] == "rect":
                xa, xb = X(p["x"] - p["w"] / 2), X(p["x"] + p["w"] / 2)
                o.append(f'<rect x="{min(xa, xb):.1f}" y="{(p["y"] - p["h"] / 2) * S:.1f}" width="{p["w"] * S:.1f}" height="{p["h"] * S:.1f}" fill="{fillc}"/>')
            else: o.append(f'<circle cx="{X(p["x"]):.1f}" cy="{p["y"] * S:.1f}" r="{p["w"] / 2 * S:.1f}" fill="{fillc}"/>')
            if p["layer"] == "TH" and top: o.append(f'<circle cx="{X(p["x"]):.1f}" cy="{p["y"] * S:.1f}" r="{p["drill"] / 2 * S:.1f}" fill="#101010"/>')
    for v in B.vias:
        o.append(f'<circle cx="{X(v["x"]):.1f}" cy="{v["y"] * S:.1f}" r="{VIA_D / 2 * S:.1f}" fill="#c9c9c9"/><circle cx="{X(v["x"]):.1f}" cy="{v["y"] * S:.1f}" r="{VIA_DRILL / 2 * S:.1f}" fill="#111"/>')
    for h in B.holes: o.append(f'<circle cx="{X(h["x"]):.1f}" cy="{h["y"] * S:.1f}" r="{h["drill"] / 2 * S:.1f}" fill="#101010" stroke="#999" stroke-width="1"/>')
    x0, y0, x1, y1 = WIN.bounds
    o.append(f'<rect x="{min(X(x0), X(x1)):.1f}" y="{y0 * S:.1f}" width="{(x1 - x0) * S:.1f}" height="{(y1 - y0) * S:.1f}" fill="#2b2b2b" stroke="#e8d44d" stroke-width="2"/>')
    side = "B" if flip else "F"
    for part in PARTS:
        if part.ref.startswith("H"): continue
        if part.smd and part.side != side: continue
        b = part.court.bounds
        o.append(f'<rect x="{min(X(b[0]), X(b[2])):.1f}" y="{b[1] * S:.1f}" width="{(b[2] - b[0]) * S:.1f}" height="{(b[3] - b[1]) * S:.1f}" fill="none" stroke="#f2f2f2" stroke-width="0.9" stroke-opacity="0.8"/>')
        cx = X((b[0] + b[2]) / 2); cy = (b[1] + b[3]) / 2 * S
        small = (b[2] - b[0]) < 5 and (b[3] - b[1]) < 5
        o.append(f'<text x="{cx:.1f}" y="{cy + 3:.1f}" fill="#fff" font-size="{6.5 if small else 9}" text-anchor="middle">{part.ref}</text>')
    o.append(f'<text x="{(X(x0) + X(x1)) / 2:.1f}" y="{(y0 - 0.6) * S:.1f}" fill="#e8d44d" font-size="9" text-anchor="middle">IGBT CUT-OUT {x1 - x0:.0f} x {y1 - y0:.0f} mm</text>')
    o.append(f'<text x="6" y="-3" fill="#ddd" font-size="9">{"BOTTOM (mirrored)" if flip else "TOP"} view — board {BW:.0f} x {BH:.0f} mm</text>')
    o.append('</svg>')
    return "\n".join(o)


BOM_HINT = {
    "R0603": "Resistor 0603 1 % (thick film)", "C0603": "Capacitor 0603 X7R 50 V", "C0805": "Capacitor 0805 X5R/X7R (22 uF >= 10 V, 2.2 uF >= 25 V)",
    "R1206": "Resistor 1206, 0.25 W", "F1206": "SMD fuse 0.5 A, rated >= 32 V (1206 / 2410), or a resettable PTC of the same footprint",
    "C1210": "Capacitor 1210 X7R 10 uF 50 V", "D123": "SOD-123 diode (pin 1 = cathode)", "DSMA": "Schottky SMA (pin 1 = cathode)", "DSMB": "TVS SMB, unidirectional (pin 1 = cathode)",
    "L4040": "Shielded power inductor 4.7 uH, 4x4 mm, I_sat >= 2 A, DCR < 100 mOhm (check the land pattern of the part you buy)",
    "SOT23": "NPN BC847B (1 = B, 2 = E, 3 = C)", "SOT23_6": "AP63205WU-7 (TSOT-26): 1 FB, 2 EN, 3 VIN, 4 GND, 5 SW, 6 BST",
    "SOIC16": "SN74HCT123D (SOIC-16 narrow)", "TERM35": "Screw terminal 3.5 mm pitch, 4 positions, 8 A", "MODULE": "Waveshare ESP32-C3-Zero on 2x 1x9 female headers (2.54 mm, 15.24 mm apart)",
    "TO220": "Ignition IGBT ISL9V3040P3 (TO-220), laid flat on the case floor in the cut-out", "MH": "M2.5 screw + 6 mm standoff",
}


def exports(B):
    groups = {}
    for part in PARTS:
        key = ("M2.5 mounting hole (6 mm standoff)", "MH") if part.ref.startswith("H") else (part.value, part.fp)
        groups.setdefault(key, []).append(part.ref)
    with open(os.path.join(OUT, "bom.csv"), "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f); wr.writerow(["Qty", "Refs", "Value", "Footprint", "Description"])
        for (val, fp), refs in sorted(groups.items(), key=lambda kv: kv[1][0]):
            wr.writerow([len(refs), " ".join(sorted(refs)), val, FP[fp]["name"], BOM_HINT.get(fp, "")])
    nets = {}
    for p in B.pads:
        if p["net"]: nets.setdefault(p["net"], []).append(f'{p["ref"]}.{p["name"]}')
    with open(os.path.join(OUT, "netlist.csv"), "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f); wr.writerow(["Net", "Pins"])
        for n in sorted(nets): wr.writerow([n, " ".join(nets[n])])


def main():
    B, by, log = autoroute()
    geo, pours = fix_gnd(B, log)
    print("\n".join(log))
    errs = drc(B, geo, pours)
    print(f"tracks={len(B.tracks)} vias={len(B.vias)}  DRC problems: {len(errs)}")
    for e in errs[:80]: print("  -", e)
    open(os.path.join(OUT, "moskvich_ecu.kicad_pcb"), "w", encoding="utf-8").write(kicad(B, pours))
    open(os.path.join(OUT, "pcb_top.svg"), "w", encoding="utf-8").write(svg(B, pours, "top"))
    open(os.path.join(OUT, "pcb_bottom.svg"), "w", encoding="utf-8").write(svg(B, pours, "bottom"))
    exports(B)
    x0, y0, x1, y1 = WIN.bounds
    print(f"IGBT cut-out: x {x0:.2f}..{x1:.2f}  y {y0:.2f}..{y1:.2f}  ({x1 - x0:.1f} x {y1 - y0:.1f} mm); board {BW} x {BH} mm")
    return len(errs)


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
