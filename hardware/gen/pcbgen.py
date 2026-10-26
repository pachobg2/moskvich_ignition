#!/usr/bin/env python3
"""Moskvich ECU board generator: parts, placement, auto-router, DRC, KiCad + SVG export.

    python hardware/gen/pcbgen.py            (needs numpy, scipy, shapely)

Everything is through-hole (no SMD). Grid is 0.254 mm (0.01") so all 2.54 mm
pitched pads land exactly on routing nodes. See hardware/README.md.
"""
import heapq, math, os, sys
import numpy as np
from scipy import ndimage
from shapely.geometry import Point, LineString, box, Polygon, MultiPolygon
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)

G = 0.254
BW, BH = 101.6, 63.5                       # 4.0" x 2.5"
NX, NY = int(round(BW / G)) + 1, int(round(BH / G)) + 1
CLR = 0.30                                 # copper clearance
EDGE = 0.60                                # copper to board edge
W_SIG, W_PWR = 0.50, 0.80
VIA_D, VIA_DRILL = 1.0, 0.5
D_TRK, D_VIA = 3, 4                        # dilation radius in cells (0.762 / 1.016 mm)


def s(v):
    return round(v / G) * G


# ----------------------------------------------------------------- footprints
# pad: (name, x, y, shape, w, h, drill)   silk: ('l',x1,y1,x2,y2) ('c',cx,cy,r) ('r',x1,y1,x2,y2)
def fp_two(pitch, pad, drill, pol, silk, name):
    return dict(name=name, pads=[("1", 0, 0, "rect" if pol else "circle", pad, pad, drill),
                                 ("2", pitch, 0, "circle", pad, pad, drill)], silk=silk)


FP = {
    "R254": fp_two(2.54, 1.8, 0.9, False, [('r', -1.2, -1.4, 3.74, 1.4)], "R_Axial_Vertical_P2.54"),
    "D254": fp_two(2.54, 1.8, 0.9, True, [('r', -1.2, -1.4, 3.74, 1.4)], "D_Axial_Vertical_P2.54"),
    "C254": fp_two(2.54, 1.8, 0.9, False, [('r', -1.2, -1.4, 3.74, 1.4)], "C_Disc_P2.54"),
    "CRAD508": fp_two(5.08, 2.2, 1.1, True, [('c', 2.54, 0, 5.4)], "CP_Radial_D10_P5.08"),
    "CRAD254": fp_two(2.54, 1.8, 0.9, True, [('c', 1.27, 0, 3.4)], "CP_Radial_D6.3_P2.54"),
    "CMKT762": fp_two(7.62, 2.0, 1.0, False, [('r', -1.6, -3.2, 9.22, 3.2)], "C_Rect_P7.62"),
    "FRAD508": fp_two(5.08, 2.0, 1.0, False, [('r', -1.8, -3.0, 6.88, 3.0)], "Fuse_Radial_P5.08"),
    "TVS508": fp_two(5.08, 2.0, 1.0, True, [('r', -1.6, -2.0, 6.68, 2.0)], "D_DO-15_Vertical_P5.08"),
    "SIP3": dict(name="SIP3_TSR1", pads=[("1", 0, 0, "rect", 1.8, 1.8, 1.0), ("2", 2.54, 0, "circle", 1.8, 1.8, 1.0),
                                          ("3", 5.08, 0, "circle", 1.8, 1.8, 1.0)],
                 silk=[('r', -2.0, -2.0, 7.08, 2.0)]),
    "TO92": dict(name="TO-92_Inline", pads=[("1", 0, 0, "rect", 1.6, 1.6, 0.9), ("2", 2.54, 0, "circle", 1.6, 1.6, 0.9),
                                             ("3", 5.08, 0, "circle", 1.6, 1.6, 0.9)],
                 silk=[('r', -1.5, -2.6, 6.58, 1.4)]),
    "DIP16": dict(name="DIP-16_W7.62", pads=[(str(i + 1), 0, 2.54 * i, "rect" if i == 0 else "circle", 1.7, 1.7, 0.9) for i in range(8)] +
                  [(str(16 - i), 7.62, 2.54 * i, "circle", 1.7, 1.7, 0.9) for i in range(8)],
                  silk=[('r', -1.4, -1.6, 9.02, 19.4), ('l', 2.0, -1.6, 2.0, -0.8), ('l', 5.6, -1.6, 5.6, -0.8), ('l', 2.0, -0.8, 5.6, -0.8)]),
    "TERM4": dict(name="TerminalBlock_P5.08_1x04", pads=[(str(i + 1), 0, 5.08 * i, "rect" if i == 0 else "circle", 3.0, 3.0, 1.4) for i in range(4)],
                  silk=[('r', -4.1, -2.9, 4.1, 18.14)]),
    "MODULE": dict(name="ESP32-C3-Zero_2x9_Socket",
                   pads=[(str(i + 1), 0, 2.54 * i, "rect" if i == 0 else "circle", 1.8, 1.8, 1.0) for i in range(9)] +
                        [(str(18 - i), 15.24, 2.54 * i, "circle", 1.8, 1.8, 1.0) for i in range(9)],
                   silk=[('r', -1.38, -1.6, 16.62, 21.9), ('l', -1.38, 1.2, 16.62, 1.2)]),
    "TO220": dict(name="TO-220_LayDown_Cutout", pads=[("1", 0, -2.54, "circle", 2.2, 2.2, 1.2), ("2", 0, 0, "circle", 2.2, 2.2, 1.2),
                                                       ("3", 0, 2.54, "circle", 2.2, 2.2, 1.2)],
                  silk=[('r', 4.0, -5.08, 20.0, 5.08), ('c', 16.5, 0, 1.9), ('r', 3.0, -6.5, 21.0, 6.5)]),
    "MH": dict(name="MountingHole_3.2", pads=[("", 0, 0, "npth", 3.2, 3.2, 3.2)], silk=[('c', 0, 0, 3.0)]),
}


def rot(x, y, a):
    a = a % 360
    if a == 0: return x, y
    if a == 90: return -y, x
    if a == 180: return -x, -y
    if a == 270: return y, -x
    raise ValueError(a)


class Part:
    def __init__(self, ref, value, fp, x, y, r=0, nets=None, desc=""):
        self.ref, self.value, self.fp, self.r, self.desc = ref, value, fp, r, desc
        self.x, self.y = s(x), s(y)
        self.nets = nets or {}
        d = FP[fp]
        self.fpname = d["name"]
        self.pads = []
        for (n, px, py, shape, w, h, drill) in d["pads"]:
            rx, ry = rot(px, py, r)
            if shape == "rect" and r % 180 == 90: w, h = h, w
            self.pads.append(dict(ref=ref, name=n, x=round(self.x + rx, 4), y=round(self.y + ry, 4), shape=shape, w=w, h=h, drill=drill,
                                  net=self.nets.get(n), lx=rx, ly=ry))
        self.silk = []
        for it in d["silk"]:
            if it[0] == 'c':
                cx, cy = rot(it[1], it[2], r); self.silk.append(('c', cx, cy, it[3]))
            else:
                a, b = rot(it[1], it[2], r), rot(it[3], it[4], r)
                self.silk.append((it[0], a[0], a[1], b[0], b[1]))


# ------------------------------------------------------------------ the design
def cell(prefix, xn, y0, vals, nin, nnode, extra):
    """sensor divider cell: series R, shunt R, 100nF, 3V6 zener (all vertical/2.54)"""
    r1, r2 = vals
    P = []
    P.append(Part(f"R{extra[0]}", r1, "R254", xn - 2.54, y0, 0, {"1": nin, "2": nnode}))
    P.append(Part(f"R{extra[1]}", r2, "R254", xn, y0 + 2.54, 0, {"1": nnode, "2": "GND"}))
    P.append(Part(f"C{extra[2]}", "100nF", "C254", xn, y0 + 5.08, 0, {"1": nnode, "2": "GND"}))
    P.append(Part(f"D{extra[3]}", "BZX55C3V6", "D254", xn, y0 + 7.62, 0, {"1": nnode, "2": "GND"}))
    return P


PARTS = []
A = PARTS.append
# connectors
A(Part("J2", "Sensors", "TERM4", 7.62, 7.62, 0, {"1": "LPG_IN", "2": "+5V", "3": "MAP_SIG", "4": "GND"}, "LPG / MAP sensor terminal"))
A(Part("J1", "Power", "TERM4", 7.62, 34.29, 0, {"1": "POINTS_RAW", "2": "+12V_IN", "3": "GND", "4": "COIL_N"}, "Points / +12V / GND / Coil-"))
# module (pad numbers per Waveshare ESP32-C3-Zero list, anticlockwise from USB)
MOD_NETS = {"1": "+5V", "2": "GND", "3": "+3V3", "4": "LPG_S", "5": "MAP_S", "7": "VB_S", "8": "PT_G", "10": "CMD_G"}
A(Part("U1", "ESP32-C3-Zero", "MODULE", 44.45, 3.81, 0, MOD_NETS, "Waveshare ESP32-C3-Zero on 2x9 sockets"))
# sensor cells
PARTS += cell("LPG", 20.32, 5.08, ("100k", "22k"), "LPG_IN", "LPG_S", (6, 7, 6, 4))
PARTS += cell("MAP", 33.02, 5.08, ("10k", "15k"), "MAP_SIG", "MAP_S", (11, 12, 8, 6))
PARTS += cell("VBAT", 20.32, 17.78, ("100k", "22k"), "VBAT_F", "VB_S", (8, 9, 7, 5))
# points input
A(Part("R1", "1k 0.5W", "R254", 12.7, 27.94, 0, {"1": "VIN_P", "2": "POINTS_RAW"}))
A(Part("R2", "10k", "R254", 17.78, 30.48, 0, {"1": "POINTS_RAW", "2": "PT_B"}))
A(Part("R3", "22k", "R254", 20.32, 33.02, 0, {"1": "PT_B", "2": "GND"}))
A(Part("D3", "1N4148", "D254", 20.32, 35.56, 0, {"1": "PT_B", "2": "GND"}))
A(Part("Q2", "BC547B", "TO92", 27.94, 30.48, 0, {"1": "PT_C", "2": "PT_B", "3": "GND"}))
A(Part("R4", "4.7k", "R254", 27.94, 33.02, 90, {"1": "PT_C", "2": "+3V3"}))
A(Part("R5", "1k", "R254", 30.48, 33.02, 90, {"1": "PT_C", "2": "PT_G"}))
A(Part("C5", "10nF", "C254", 33.02, 33.02, 90, {"1": "PT_G", "2": "GND"}))
# power input
A(Part("F1", "500mA PTC", "FRAD508", 15.24, 40.64, 0, {"1": "+12V_IN", "2": "VBAT_F"}))
A(Part("D1", "1N5819", "D254", 27.94, 40.64, 180, {"1": "VIN_P", "2": "VBAT_F"}))
A(Part("D2", "P6KE27A", "TVS508", 15.24, 48.26, 0, {"1": "VIN_P", "2": "GND"}))
A(Part("C1", "470uF 35V", "CRAD508", 25.4, 55.88, 0, {"1": "VIN_P", "2": "GND"}))
A(Part("C2", "100nF", "C254", 35.56, 55.88, 0, {"1": "VIN_P", "2": "GND"}))
A(Part("U2", "TSR 1-2450", "SIP3", 35.56, 45.72, 0, {"1": "VIN_P", "2": "GND", "3": "+5V"}, "5V 1A buck module (SIP-3)"))
A(Part("C3", "100uF 16V", "CRAD254", 45.72, 53.34, 0, {"1": "+5V", "2": "GND"}))
A(Part("C4", "100nF", "C254", 45.72, 48.26, 0, {"1": "+5V", "2": "GND"}))
# coil driver
A(Part("U3", "74HCT123N", "DIP16", 53.34, 33.02, 0,
       {"1": "GND", "2": "CMD", "3": "CMD", "8": "GND", "9": "+5V", "10": "GND", "11": "GND", "13": "GATE_DRV", "14": "CEXT", "15": "REXT", "16": "+5V"},
       "retriggerable one-shot = coil on-time guard + 5 V gate driver"))
A(Part("R10", "100R", "R254", 46.99, 27.94, 90, {"1": "CMD_G", "2": "CMD"}))
A(Part("R13", "10k", "R254", 46.99, 33.02, 90, {"1": "CMD", "2": "GND"}))
A(Part("C9", "100nF", "C254", 63.5, 33.02, 90, {"1": "+5V", "2": "GND"}))
A(Part("R14", "27k", "R254", 66.04, 33.02, 90, {"1": "+5V", "2": "REXT"}))
A(Part("C10", "1uF film", "CMKT762", 53.34, 55.88, 0, {"1": "CEXT", "2": "REXT"}))
A(Part("R15", "100R", "R254", 63.5, 40.64, 0, {"1": "GATE_DRV", "2": "GATE"}))
A(Part("R16", "10k", "R254", 66.04, 43.18, 90, {"1": "GATE", "2": "GND"}))
A(Part("Q1", "ISL9V3040P3", "TO220", 71.12, 44.45, 0, {"1": "GATE", "2": "COIL_N", "3": "GND"}, "ignition IGBT, lies flat on the case floor in the cut-out"))
for i, (mx, my) in enumerate([(3.81, 3.81), (97.79, 3.81), (3.81, 59.69), (97.79, 59.69)]):
    A(Part(f"H{i + 1}", "M3", "MH", mx, my, 0, {}))

WIN = box(s(71.12 + 3.0), s(44.45 - 6.5), s(71.12 + 21.0), s(44.45 + 6.5))          # IGBT cut-out
MODBODY = box(44.45 - 1.38, 3.81 - 1.6, 44.45 + 16.62, 3.81 + 21.9)
OUTLINE = box(0, 0, BW, BH)

# net classes (width); GND is a pour
WIDTH = {"+5V": W_PWR, "+3V3": W_SIG, "+12V_IN": W_PWR, "VIN_P": W_PWR, "VBAT_F": W_PWR}
MANUAL = [("COIL_N", "B.Cu", 2.0, [(71.12, 44.45), (68.58, 44.45), (68.58, 60.96), (11.43, 60.96), (11.43, 49.53), (7.62, 49.53)])]


# ------------------------------------------------------------------ geometry db
def all_pads():
    return [p for part in PARTS for p in part.pads]


def pad_geom(p):
    if p["shape"] == "rect":
        return box(p["x"] - p["w"] / 2, p["y"] - p["h"] / 2, p["x"] + p["w"] / 2, p["y"] + p["h"] / 2)
    return Point(p["x"], p["y"]).buffer(p["w"] / 2, 24)


class Board:
    def __init__(self):
        self.tracks = []       # dict(net, layer, w, pts)
        self.vias = []         # dict(net, x, y)
        self.pads = [p for p in all_pads() if p["shape"] != "npth"]
        self.holes = [p for p in all_pads() if p["shape"] == "npth"]
        self.cu = {}           # net -> layer -> bool array
        self.static = {"F.Cu": np.zeros((NX, NY), bool), "B.Cu": np.zeros((NX, NY), bool)}
        self._dil = {}
        self._ver = {}
        for p in self.pads:
            n = p["net"] or f"NC_{p['ref']}_{p['name']}"
            p["rnet"] = n
            for L in ("F.Cu", "B.Cu"):
                self._mark_pad(n, L, p)
        self._build_static()

    def arr(self, net, L):
        d = self.cu.setdefault(net, {})
        if L not in d: d[L] = np.zeros((NX, NY), bool)
        return d[L]

    def _bump(self, net):
        self._ver[net] = self._ver.get(net, 0) + 1

    def _mark_pad(self, net, L, p):
        a = self.arr(net, L)
        if p["shape"] == "rect":
            mark_rect(a, p["x"] - p["w"] / 2, p["y"] - p["h"] / 2, p["x"] + p["w"] / 2, p["y"] + p["h"] / 2)
        else:
            mark_circle(a, p["x"], p["y"], p["w"] / 2)
        self._bump(net)

    def _build_static(self):
        for L in self.static:
            a = self.static[L]
            a[:, :] = False
            m = int(math.ceil(EDGE / G)) + 2                     # board edge keep-out (track centre line)
            a[:m, :] = True; a[-m:, :] = True; a[:, :m] = True; a[:, -m:] = True
            for h in self.holes:
                mark_circle(a, h["x"], h["y"], 1.6 + CLR + 0.3)
            minx, miny, maxx, maxy = WIN.bounds
            mark_rect(a, minx - 1.0, miny - 1.0, maxx + 1.0, maxy + 1.0)
            # module body: no routing underneath except close to its own pads
            bx0, by0, bx1, by1 = MODBODY.bounds
            tmp = np.zeros((NX, NY), bool)
            mark_rect(tmp, bx0, by0, bx1, by1)
            allow = np.zeros((NX, NY), bool)
            for p in self.pads:
                if p["ref"] == "U1" and p["net"]:
                    mark_circle(allow, p["x"], p["y"], 1.9)
                    mark_rect(allow, p["x"] - 6.0, p["y"] - 0.45, p["x"], p["y"] + 0.45) if p["lx"] < 5 else None
            a |= tmp & ~allow

    def add_track(self, net, L, w, pts, manual=False):
        self.tracks.append(dict(net=net, layer=L, w=w, pts=pts))
        a = self.arr(net, L)
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            mark_seg(a, x1, y1, x2, y2, w / 2)
        self._bump(net)

    def add_via(self, net, x, y):
        self.vias.append(dict(net=net, x=x, y=y))
        for L in ("F.Cu", "B.Cu"):
            mark_circle(self.arr(net, L), x, y, VIA_D / 2)
        self._bump(net)

    def dil(self, net, L, r):
        key = (net, L, r)
        ver = self._ver.get(net, 0)
        c = self._dil.get(key)
        if c and c[0] == ver: return c[1]
        st = np.zeros((2 * r + 1, 2 * r + 1), bool)
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
        st[(xx * xx + yy * yy) <= r * r + 0.5] = True
        out = ndimage.binary_dilation(self.arr(net, L), structure=st)
        self._dil[key] = (ver, out)
        return out

    def blocked(self, net, L, r):
        b = self.static[L].copy()
        for m in list(self.cu.keys()):
            if m == net: continue
            if L in self.cu[m]: b |= self.dil(m, L, r)
        return b


def _bbox(a, cx0, cy0, cx1, cy1):
    x0 = max(0, int(cx0 / G) - 1); x1 = min(NX - 1, int(cx1 / G) + 2)
    y0 = max(0, int(cy0 / G) - 1); y1 = min(NY - 1, int(cy1 / G) + 2)
    return x0, x1, y0, y1


def mark_circle(a, cx, cy, r):
    r2 = r + G * 0.5
    x0, x1, y0, y1 = _bbox(a, cx - r2, cy - r2, cx + r2, cy + r2)
    xs = np.arange(x0, x1 + 1) * G; ys = np.arange(y0, y1 + 1) * G
    m = ((xs[:, None] - cx) ** 2 + (ys[None, :] - cy) ** 2) <= r2 * r2
    a[x0:x1 + 1, y0:y1 + 1] |= m


def mark_rect(a, x1_, y1_, x2_, y2_):
    m = G * 0.5
    x0, x1, y0, y1 = _bbox(a, x1_ - m, y1_ - m, x2_ + m, y2_ + m)
    xs = np.arange(x0, x1 + 1) * G; ys = np.arange(y0, y1 + 1) * G
    mx = (xs >= x1_ - m) & (xs <= x2_ + m); my = (ys >= y1_ - m) & (ys <= y2_ + m)
    a[x0:x1 + 1, y0:y1 + 1] |= mx[:, None] & my[None, :]


def mark_seg(a, x1_, y1_, x2_, y2_, hw):
    r = hw + G * 0.5
    x0, x1, y0, y1 = _bbox(a, min(x1_, x2_) - r, min(y1_, y2_) - r, max(x1_, x2_) + r, max(y1_, y2_) + r)
    xs = np.arange(x0, x1 + 1) * G; ys = np.arange(y0, y1 + 1) * G
    px, py = xs[:, None], ys[None, :]
    dx, dy = x2_ - x1_, y2_ - y1_
    L2 = dx * dx + dy * dy
    t = np.clip(((px - x1_) * dx + (py - y1_) * dy) / L2, 0, 1) if L2 > 0 else np.zeros_like(px * py)
    d2 = (px - (x1_ + t * dx)) ** 2 + (py - (y1_ + t * dy)) ** 2
    a[x0:x1 + 1, y0:y1 + 1] |= d2 <= r * r


# ------------------------------------------------------------------ router
def route_net(B, net, pads, width, log):
    """connect all pads of `net` (list of pad dicts) with an MST built by A*"""
    pts = [(p["x"], p["y"]) for p in pads]
    cells = [(int(round(x / G)), int(round(y / G))) for x, y in pts]
    connected = {0}
    nodes = {(L, *cells[0]) for L in (0, 1)}
    unrouted = 0
    order = list(range(1, len(pads)))
    while len(connected) < len(pads):
        # nearest unconnected pad to the connected set (Manhattan)
        best = None
        for j in range(len(pads)):
            if j in connected: continue
            d = min(abs(cells[i][0] - cells[j][0]) + abs(cells[i][1] - cells[j][1]) for i in connected)
            if best is None or d < best[0]: best = (d, j)
        j = best[1]
        path = astar(B, net, nodes, cells[j], width)
        if path is None:
            log.append(f"  !! could not route {net}: pad {pads[j]['ref']}.{pads[j]['name']}")
            unrouted += 1
            connected.add(j)
            nodes |= {(L, *cells[j]) for L in (0, 1)}
            continue
        emit_path(B, net, path, width, pts_override={cells[j]: pts[j]})
        for (L, ix, iy) in path: nodes.add((L, ix, iy))
        nodes |= {(L, *cells[j]) for L in (0, 1)}
        connected.add(j)
    return unrouted


def astar(B, net, sources, goal, width):
    LN = ("F.Cu", "B.Cu")
    blk = [B.blocked(net, L, D_TRK) for L in LN]
    blkv = [B.blocked(net, L, D_VIA) for L in LN]
    gx, gy = goal
    openh, g, came = [], {}, {}
    for (L, ix, iy) in sources:
        g[(L, ix, iy)] = 0.0
        heapq.heappush(openh, (abs(ix - gx) + abs(iy - gy), 0.0, (L, ix, iy)))
    while openh:
        f, gc, cur = heapq.heappop(openh)
        if gc > g.get(cur, 1e18): continue
        L, ix, iy = cur
        if (ix, iy) == (gx, gy):
            path = [cur]
            while cur in came:
                cur = came[cur]; path.append(cur)
            return path[::-1]
        # moves
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = ix + dx, iy + dy
            if not (0 <= nx < NX and 0 <= ny < NY): continue
            if blk[L][nx, ny] and (nx, ny) != (gx, gy): continue
            pref = (dx != 0) if L == 0 else (dy != 0)
            c = gc + (1.0 if pref else 2.2)
            n = (L, nx, ny)
            if c < g.get(n, 1e18):
                g[n] = c; came[n] = cur
                heapq.heappush(openh, (c + abs(nx - gx) + abs(ny - gy), c, n))
        # via
        o = 1 - L
        if not blkv[0][ix, iy] and not blkv[1][ix, iy]:
            n = (o, ix, iy)
            c = gc + 12.0
            if c < g.get(n, 1e18):
                g[n] = c; came[n] = cur
                heapq.heappush(openh, (c + abs(ix - gx) + abs(iy - gy), c, n))
    return None


def emit_path(B, net, path, width, pts_override=None):
    LN = ("F.Cu", "B.Cu")
    seg = []
    for a, b in zip(path, path[1:]):
        if a[0] != b[0]:
            B.add_via(net, a[1] * G, a[2] * G)
            continue
        seg.append((a, b))
    # merge collinear runs per layer
    runs = []
    for a, b in seg:
        d = (b[1] - a[1], b[2] - a[2])
        if runs and runs[-1]["layer"] == a[0] and runs[-1]["dir"] == d and runs[-1]["end"] == (a[1], a[2]):
            runs[-1]["end"] = (b[1], b[2])
        else:
            runs.append(dict(layer=a[0], dir=d, start=(a[1], a[2]), end=(b[1], b[2])))
    # join runs of the same layer into polylines
    polys = []
    for r in runs:
        if polys and polys[-1]["layer"] == r["layer"] and polys[-1]["pts"][-1] == r["start"]:
            polys[-1]["pts"].append(r["end"])
        else:
            polys.append(dict(layer=r["layer"], pts=[r["start"], r["end"]]))
    for p in polys:
        pts = [(x * G, y * G) for x, y in p["pts"]]
        B.add_track(net, LN[p["layer"]], width, pts)


def autoroute():
    B = Board()
    log = []
    for net, L, w, pts in MANUAL:
        B.add_track(net, L, w, pts)
    by = {}
    for p in B.pads:
        by.setdefault(p["rnet"], []).append(p)
    nets = [n for n in by if len(by[n]) > 1 and n not in ("GND", "COIL_N")]
    prio = ["+5V", "VIN_P", "+12V_IN", "VBAT_F", "+3V3"]
    nets.sort(key=lambda n: (prio.index(n) if n in prio else 99, sum(abs(p["x"] - by[n][0]["x"]) for p in by[n])))
    for n in nets:
        miss = route_net(B, n, by[n], WIDTH.get(n, W_SIG), log)
        log.append(f"routed {n:12s} pads={len(by[n]):2d} unrouted={miss}")
    # GND is a pour; only the module's own GND pad (pour keep-out under the module) needs a track
    gp = [next(p for p in B.pads if p["ref"] == "U1" and p["name"] == "2"), next(p for p in B.pads if p["ref"] == "D6" and p["name"] == "2")]
    log.append(f"routed GND stub   unrouted={route_net(B, 'GND', gp, W_PWR, log)}")
    return B, by, log


# ------------------------------------------------------------------ copper geometry, DRC, pour
def geoms(B):
    """true shapely geometry per (net, layer) from pads/tracks/vias"""
    G_ = {}
    def add(net, L, g): G_.setdefault((net, L), []).append(g)
    for p in B.pads:
        for L in ("F.Cu", "B.Cu"): add(p["rnet"], L, pad_geom(p))
    for t in B.tracks:
        add(t["net"], t["layer"], LineString(t["pts"]).buffer(t["w"] / 2, 8))
    for v in B.vias:
        for L in ("F.Cu", "B.Cu"): add(v["net"], L, Point(v["x"], v["y"]).buffer(VIA_D / 2, 16))
    return {k: unary_union(v) for k, v in G_.items()}


def keepouts():
    ko = [WIN.buffer(EDGE), MODBODY]
    return ko


def pour(B, geo):
    res = {}
    base = OUTLINE.buffer(-EDGE)
    for L in ("F.Cu", "B.Cu"):
        others = unary_union([g for (n, l), g in geo.items() if l == L and n != "GND"])
        r = base.difference(others.buffer(CLR + 0.01, 16)).difference(unary_union(keepouts()))
        for h in B.holes: r = r.difference(Point(h["x"], h["y"]).buffer(1.6 + CLR, 16))
        res[L] = r
    return res


def drc(B, geo, pours):
    errs = []
    nets = sorted({n for n, _ in geo})
    for L in ("F.Cu", "B.Cu"):
        items = {n: geo[(n, L)] for n in nets if (n, L) in geo}
        names = list(items)
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                d = items[names[i]].distance(items[names[j]])
                if d < CLR - 0.005:
                    errs.append(f"{L}: {names[i]} <-> {names[j]} clearance {d:.3f} mm")
        for n, g in items.items():
            if not OUTLINE.buffer(-(EDGE - 0.1)).contains(g):
                errs.append(f"{L}: {n} too close to board edge")
            if g.intersects(WIN.buffer(0.3)):
                errs.append(f"{L}: {n} inside IGBT cut-out")
    # drill / hole spacing
    pads = B.pads
    for i in range(len(pads)):
        for j in range(i + 1, len(pads)):
            a, b = pads[i], pads[j]
            if math.hypot(a["x"] - b["x"], a["y"] - b["y"]) < (a["drill"] + b["drill"]) / 2 + 0.35:
                errs.append(f"holes too close: {a['ref']}.{a['name']} / {b['ref']}.{b['name']}")
    # connectivity per net
    for n in nets:
        if n.startswith("NC_"): continue
        pn = [p for p in B.pads if p["rnet"] == n]
        if len(pn) < 2: continue
        elems = []                                           # (geom per layer dict, is_tht)
        for p in pn: elems.append(({"F.Cu": pad_geom(p), "B.Cu": pad_geom(p)}, ("pad", p)))
        for t in B.tracks:
            if t["net"] == n: elems.append(({t["layer"]: LineString(t["pts"]).buffer(t["w"] / 2, 8)}, ("trk", t)))
        for v in B.vias:
            if v["net"] == n:
                g = Point(v["x"], v["y"]).buffer(VIA_D / 2, 16); elems.append(({"F.Cu": g, "B.Cu": g}, ("via", v)))
        if n == "GND":
            for L, r in pours.items():
                for pg in (r.geoms if hasattr(r, "geoms") else [r]):
                    if pg.area > 1.0: elems.append(({L: pg}, ("pour", L)))
        par = list(range(len(elems)))
        def f(x):
            while par[x] != x: par[x] = par[par[x]]; x = par[x]
            return x
        for i in range(len(elems)):
            for j in range(i + 1, len(elems)):
                if any(L in elems[j][0] and elems[i][0][L].intersects(elems[j][0][L]) for L in elems[i][0]):
                    par[f(i)] = f(j)
        roots = {f(i) for i, e in enumerate(elems) if e[1][0] == "pad"}
        if len(roots) > 1:
            groups = {}
            for i, e in enumerate(elems):
                if e[1][0] == "pad": groups.setdefault(f(i), []).append(f"{e[1][1]['ref']}.{e[1][1]['name']}")
            errs.append(f"OPEN net {n}: {len(roots)} islands: " + " | ".join(",".join(v) for v in groups.values()))
    return errs


# ------------------------------------------------------------------ KiCad export
def kicad(B, pours, by):
    nets = ["GND", "+5V", "+3V3", "+12V_IN", "VIN_P", "VBAT_F", "COIL_N", "POINTS_RAW", "PT_B", "PT_C", "PT_G", "LPG_IN", "LPG_S", "MAP_SIG", "MAP_S",
            "VB_S", "CMD_G", "CMD", "GATE_DRV", "GATE", "CEXT", "REXT"]
    nid = {n: i + 1 for i, n in enumerate(nets)}
    o = []
    w = o.append
    w('(kicad_pcb (version 20221018) (generator pcbgen)')
    w('  (general (thickness 1.6))')
    w('  (paper "A4")')
    w('  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (34 "B.Paste" user) (35 "F.Paste" user) (36 "B.SilkS" user "B.Silkscreen") (37 "F.SilkS" user "F.Silkscreen")'
      ' (38 "B.Mask" user) (39 "F.Mask" user) (40 "Dwgs.User" user "User.Drawings") (41 "Cmts.User" user "User.Comments") (44 "Edge.Cuts" user) (45 "Margin" user)'
      ' (46 "B.CrtYd" user "B.Courtyard") (47 "F.CrtYd" user "F.Courtyard") (48 "B.Fab" user) (49 "F.Fab" user))')
    w('  (setup (pad_to_mask_clearance 0.05) (pcbplotparams (layerselection 0x00010fc_ffffffff) (outputdirectory "gerber/")))')
    w('  (net 0 "")')
    for n in nets: w(f'  (net {nid[n]} "{n}")')
    # footprints
    for part in PARTS:
        w(f'  (footprint "{part.fpname}" (layer "F.Cu") (at {part.x:.3f} {part.y:.3f})')
        w(f'    (descr "{part.desc or part.value}")')
        w(f'    (fp_text reference "{part.ref}" (at 0 -3.2) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))')
        w(f'    (fp_text value "{part.value}" (at 0 3.5) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))')
        for it in part.silk:
            if it[0] == 'c':
                w(f'    (fp_circle (center {it[1]:.3f} {it[2]:.3f}) (end {it[1] + it[3]:.3f} {it[2]:.3f}) (stroke (width 0.15) (type default)) (fill none) (layer "F.SilkS"))')
            elif it[0] == 'r':
                x1, y1, x2, y2 = it[1:]
                for a, b in (((x1, y1), (x2, y1)), ((x2, y1), (x2, y2)), ((x2, y2), (x1, y2)), ((x1, y2), (x1, y1))):
                    w(f'    (fp_line (start {a[0]:.3f} {a[1]:.3f}) (end {b[0]:.3f} {b[1]:.3f}) (stroke (width 0.15) (type default)) (layer "F.SilkS"))')
            else:
                w(f'    (fp_line (start {it[1]:.3f} {it[2]:.3f}) (end {it[3]:.3f} {it[4]:.3f}) (stroke (width 0.15) (type default)) (layer "F.SilkS"))')
        for p in part.pads:
            if p["shape"] == "npth":
                w(f'    (pad "" np_thru_hole circle (at {p["lx"]:.3f} {p["ly"]:.3f}) (size {p["w"]} {p["h"]}) (drill {p["drill"]}) (layers "*.Cu" "*.Mask"))')
                continue
            shp = "rect" if p["shape"] == "rect" else "circle"
            nt = f' (net {nid[p["net"]]} "{p["net"]}")' if p["net"] else ""
            w(f'    (pad "{p["name"]}" thru_hole {shp} (at {p["lx"]:.3f} {p["ly"]:.3f}) (size {p["w"]} {p["h"]}) (drill {p["drill"]}) (layers "*.Cu" "*.Mask"){nt})')
        w('  )')
    for t in B.tracks:
        for (x1, y1), (x2, y2) in zip(t["pts"], t["pts"][1:]):
            w(f'  (segment (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) (width {t["w"]}) (layer "{t["layer"]}") (net {nid[t["net"]]}))')
    for v in B.vias:
        w(f'  (via (at {v["x"]:.3f} {v["y"]:.3f}) (size {VIA_D}) (drill {VIA_DRILL}) (layers "F.Cu" "B.Cu") (net {nid[v["net"]]}))')
    # board outline + IGBT cut-out
    def rect(x1, y1, x2, y2):
        for a, b in (((x1, y1), (x2, y1)), ((x2, y1), (x2, y2)), ((x2, y2), (x1, y2)), ((x1, y2), (x1, y1))):
            w(f'  (gr_line (start {a[0]:.3f} {a[1]:.3f}) (end {b[0]:.3f} {b[1]:.3f}) (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))')
    rect(0, 0, BW, BH)
    rect(*WIN.bounds)
    w(f'  (gr_text "MOSKVICH IGNITION ECU  rev A" (at 52 61.5) (layer "F.SilkS") (effects (font (size 1.2 1.2) (thickness 0.2))))')
    # GND zones + keep-out rule areas
    for L in ("F.Cu", "B.Cu"):
        w(f'  (zone (net {nid["GND"]}) (net_name "GND") (layer "{L}") (hatch edge 0.5) (connect_pads yes (clearance {CLR})) (min_thickness 0.25) (filled_areas_thickness no)'
          f' (fill yes (thermal_gap 0.4) (thermal_bridge_width 0.8))')
        w(f'    (polygon (pts (xy {EDGE} {EDGE}) (xy {BW - EDGE} {EDGE}) (xy {BW - EDGE} {BH - EDGE}) (xy {EDGE} {BH - EDGE}))))')
    x0, y0, x1, y1 = MODBODY.bounds
    w(f'  (zone (net 0) (net_name "") (layers "F&B.Cu") (hatch edge 0.5) (connect_pads (clearance 0)) (min_thickness 0.25) (keepout (tracks not_allowed) (vias not_allowed) (pads allowed) (copperpour not_allowed) (footprints allowed))'
      f' (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))')
    w(f'    (polygon (pts (xy {x0:.2f} {y0:.2f}) (xy {x1:.2f} {y0:.2f}) (xy {x1:.2f} {y1:.2f}) (xy {x0:.2f} {y1:.2f}))))')
    w(')')
    return "\n".join(o)


# ------------------------------------------------------------------ SVG preview
def svg(B, pours, by, which="top"):
    S = 10.0
    W_, H_ = BW * S, BH * S
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="-10 -10 {W_ + 20} {H_ + 20}" style="width:100%;height:auto;background:#2b2b2b" font-family="Arial">']
    o.append(f'<rect x="-10" y="-10" width="{W_ + 20}" height="{H_ + 20}" fill="#2b2b2b"/>')
    o.append(f'<rect x="0" y="0" width="{W_}" height="{H_}" fill="#0b4f2a" stroke="#e8d44d" stroke-width="2"/>')
    def poly(g, fill, op):
        gs = g.geoms if hasattr(g, "geoms") else [g]
        d = ""
        for p in gs:
            if p.is_empty or p.area < 0.5: continue
            for ring in [p.exterior] + list(p.interiors):
                d += "M" + " L".join(f"{x * S:.1f},{y * S:.1f}" for x, y in ring.coords) + " Z "
        o.append(f'<path d="{d}" fill="{fill}" fill-opacity="{op}" fill-rule="evenodd"/>')
    layers = ["B.Cu", "F.Cu"] if which == "top" else ["F.Cu", "B.Cu"]
    colors = {"F.Cu": "#d9453d", "B.Cu": "#3d6fd9"}
    for L in layers:
        top = (L == layers[1])
        op = 0.95 if top else 0.45
        poly(pours[L], colors[L], 0.28 if top else 0.18)
        for t in B.tracks:
            if t["layer"] != L: continue
            pts = " ".join(f"{x * S:.1f},{y * S:.1f}" for x, y in t["pts"])
            o.append(f'<polyline points="{pts}" fill="none" stroke="{colors[L]}" stroke-opacity="{op}" stroke-width="{t["w"] * S:.1f}" stroke-linecap="round" stroke-linejoin="round"/>')
    for v in B.vias:
        o.append(f'<circle cx="{v["x"] * S:.1f}" cy="{v["y"] * S:.1f}" r="{VIA_D / 2 * S:.1f}" fill="#c9c9c9"/><circle cx="{v["x"] * S:.1f}" cy="{v["y"] * S:.1f}" r="{VIA_DRILL / 2 * S:.1f}" fill="#111"/>')
    for p in B.pads:
        if p["shape"] == "rect":
            o.append(f'<rect x="{(p["x"] - p["w"] / 2) * S:.1f}" y="{(p["y"] - p["h"] / 2) * S:.1f}" width="{p["w"] * S:.1f}" height="{p["h"] * S:.1f}" fill="#d4b24a"/>')
        else:
            o.append(f'<circle cx="{p["x"] * S:.1f}" cy="{p["y"] * S:.1f}" r="{p["w"] / 2 * S:.1f}" fill="#d4b24a"/>')
        o.append(f'<circle cx="{p["x"] * S:.1f}" cy="{p["y"] * S:.1f}" r="{p["drill"] / 2 * S:.1f}" fill="#101010"/>')
    for h in B.holes:
        o.append(f'<circle cx="{h["x"] * S:.1f}" cy="{h["y"] * S:.1f}" r="{h["drill"] / 2 * S:.1f}" fill="#101010" stroke="#999" stroke-width="1"/>')
    x0, y0, x1, y1 = WIN.bounds
    o.append(f'<rect x="{x0 * S:.1f}" y="{y0 * S:.1f}" width="{(x1 - x0) * S:.1f}" height="{(y1 - y0) * S:.1f}" fill="#2b2b2b" stroke="#e8d44d" stroke-width="2"/>')
    # silk
    for part in PARTS:
        for it in part.silk:
            if it[0] == 'c':
                o.append(f'<circle cx="{(part.x + it[1]) * S:.1f}" cy="{(part.y + it[2]) * S:.1f}" r="{it[3] * S:.1f}" fill="none" stroke="#f2f2f2" stroke-width="1.2"/>')
            elif it[0] == 'r':
                o.append(f'<rect x="{(part.x + min(it[1], it[3])) * S:.1f}" y="{(part.y + min(it[2], it[4])) * S:.1f}" width="{abs(it[3] - it[1]) * S:.1f}" height="{abs(it[4] - it[2]) * S:.1f}" fill="none" stroke="#f2f2f2" stroke-width="1.2"/>')
            else:
                o.append(f'<line x1="{(part.x + it[1]) * S:.1f}" y1="{(part.y + it[2]) * S:.1f}" x2="{(part.x + it[3]) * S:.1f}" y2="{(part.y + it[4]) * S:.1f}" stroke="#f2f2f2" stroke-width="1.2"/>')
        if part.ref.startswith("H"): continue
        cx = np.mean([p["x"] for p in part.pads]); cy = np.mean([p["y"] for p in part.pads])
        dy = -3.0 if part.ref not in ("U1", "U3", "J1", "J2", "Q1") else 0
        o.append(f'<text x="{cx * S:.1f}" y="{(cy + dy) * S:.1f}" fill="#fff" font-size="{(9 if part.ref in ("U1","U3","J1","J2","Q1") else 9)}" text-anchor="middle">{part.ref}</text>')
    o.append(f'<text x="{(x0 + (x1 - x0) / 2) * S:.1f}" y="{(y0 - 1.5) * S:.1f}" fill="#e8d44d" font-size="11" text-anchor="middle">IGBT CUT-OUT {x1 - x0:.0f} x {y1 - y0:.0f} mm</text>')
    o.append('</svg>')
    return "\n".join(o)


BOM_HINT = {
    "R254": "Resistor, metal film 1/4 W (R1: 0.5 W), mounted upright (2.54 mm hole pitch)",
    "D254": "Axial diode, mounted upright (2.54 mm pitch), square pad = cathode (bar)",
    "C254": "MLCC radial, 2.54 mm pitch, 50 V X7R",
    "CRAD508": "Electrolytic radial 10 mm, 5.08 mm pitch, 35 V",
    "CRAD254": "Electrolytic radial 6.3 mm, 2.54 mm pitch, 16 V",
    "CMKT762": "Film (MKT/MKS) box, 7.5 mm pitch, 63 V",
    "FRAD508": "Resettable fuse (PTC), radial, 0.5 A hold, e.g. Bourns MF-R050",
    "TVS508": "TVS diode DO-15 mounted upright, square pad = cathode (bar)",
    "SIP3": "Traco TSR 1-2450 (6.5-36 V in, 5 V 1 A out, SIP-3), pin 1 Vin / 2 GND / 3 Vout",
    "TO92": "BC547B (pin 1 = C, 2 = B, 3 = E, flat side facing you)",
    "DIP16": "74HCT123N (DIP-16) in a turned-pin socket",
    "TERM4": "Screw terminal block, 5.08 mm pitch, 4 positions, 10 A",
    "MODULE": "Waveshare ESP32-C3-Zero plugged into 2x 1x9 female headers (2.54 mm, 15.24 mm apart)",
    "TO220": "Ignition IGBT ISL9V3040P3 (TO-220), laid flat on the case floor in the cut-out",
    "MH": "M3 screw + 6 mm standoff",
}


def exports(B, by):
    import csv
    groups = {}
    for part in PARTS:
        if part.ref.startswith("H"): key = ("M3 mounting hole (standoff 6 mm)", "MH")
        else: key = (part.value, part.fp)
        groups.setdefault(key, []).append(part.ref)
    with open(os.path.join(OUT, "bom.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Qty", "Refs", "Value", "Footprint", "Description"])
        for (val, fp), refs in sorted(groups.items(), key=lambda kv: kv[1][0]):
            w.writerow([len(refs), " ".join(refs), val, FP[fp]["name"], BOM_HINT.get(fp, "")])
    nets = {}
    for p in B.pads:
        if p["net"]: nets.setdefault(p["net"], []).append(f'{p["ref"]}.{p["name"]}')
    with open(os.path.join(OUT, "netlist.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Net", "Pins"])
        for n in sorted(nets): w.writerow([n, " ".join(nets[n])])


def main():
    B, by, log = autoroute()
    print("\n".join(log))
    geo = geoms(B)
    pours = pour(B, geo)
    errs = drc(B, geo, pours)
    print(f"tracks={len(B.tracks)} vias={len(B.vias)}  DRC problems: {len(errs)}")
    for e in errs[:60]: print("  -", e)
    os.makedirs(OUT, exist_ok=True)
    open(os.path.join(OUT, "moskvich_ecu.kicad_pcb"), "w", encoding="utf-8").write(kicad(B, pours, by))
    open(os.path.join(OUT, "pcb_top.svg"), "w", encoding="utf-8").write(svg(B, pours, by, "top"))
    exports(B, by)
    wx0, wy0, wx1, wy1 = WIN.bounds
    print(f"IGBT cut-out: x {wx0:.2f}..{wx1:.2f}  y {wy0:.2f}..{wy1:.2f}  ({wx1 - wx0:.1f} x {wy1 - wy0:.1f} mm); board {BW} x {BH} mm")
    return len(errs)


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
