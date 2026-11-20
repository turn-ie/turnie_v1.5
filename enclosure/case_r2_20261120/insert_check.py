# 2D (Y-z section) check: can the r2 board go into case V3 r2 with the SW2 knob through its closed slot?
# Board = rectangles in the final pose (Fusion frame of case V3: +z = front/acrylic, Y as in the case).
# Poses = rotate by th (deg, +Y side up) about the knob centre at the inner wall face, then shift (dY, dz).
# A pose is reachable if a chain of collision-free grid neighbours links it to a pose that can be
# lowered straight down (every board point inside the cavity's Y range).  usage: insert_check.py s0 s1
import math, sys, json, collections, os
HERE = os.path.dirname(os.path.abspath(__file__))
P = json.load(open(HERE + "/params.json"))
F, B = P["zF"], P["zB"]
IY0, IY1, OY0, OY1 = P["IY0"], P["IY1"], P["OY0"], P["OY1"]
RY0, RY1, Z_LEDGE, Z_RIM, Z_BOT = P["RY0"], P["RY1"], P["Z_LEDGE"], P["Z_RIM"], P["Z_BOT"]
s0, s1 = float(sys.argv[1]), float(sys.argv[2])
items = {k: v for k, v in P["board_rects"].items()}          # name: [Y0, Y1, z0, z1]
knob = P["knob_rect"]
walls = [[OY0, IY0, Z_BOT, Z_LEDGE], [OY0, RY0, Z_LEDGE, Z_RIM],        # y- wall + lip
         [IY1, OY1, Z_BOT, Z_LEDGE], [RY1, OY1, Z_LEDGE, Z_RIM],        # y+ wall + lip
         P["battery_rect"]]
knob_walls = [[OY0, IY0, Z_BOT, s0], [OY0, IY0, s1, Z_LEDGE], [OY0, RY0, Z_LEDGE, Z_RIM], P["battery_rect"]]
K = (IY0, (knob[2] + knob[3]) / 2)

def poly(r, th, dY, dz):
    c, s = math.cos(th), math.sin(th)
    out = []
    for y, z in ((r[0], r[2]), (r[1], r[2]), (r[1], r[3]), (r[0], r[3])):
        y, z = y - K[0], z - K[1]
        out.append((K[0] + c*y - s*z + dY, K[1] + s*y + c*z + dz))
    return out
def sep(pa, pb):                     # separating axis test, convex polygons
    for pp in (pa, pb):
        for i in range(len(pp)):
            x1, y1 = pp[i]; x2, y2 = pp[(i+1) % len(pp)]
            ax, ay = y1 - y2, x2 - x1
            a = [ax*x + ay*y for x, y in pa]; b = [ax*x + ay*y for x, y in pb]
            if max(a) <= min(b) + 1e-9 or max(b) <= min(a) + 1e-9: return True
    return False
def rect(r): return [(r[0], r[2]), (r[1], r[2]), (r[1], r[3]), (r[0], r[3])]
WALLS = [rect(w) for w in walls]; KWALLS = [rect(w) for w in knob_walls]
# The inner corners of V3 are ellipses. Near the right end of the board (USB-C side) only the corner pockets (board outline
# + pocket clearance at the final position) are free, and above the ledge the acrylic recess corner is smaller still.
# So the board items that reach the right corners (slab, header plastic) may not go past Y_CORNER while below the rim.
# limits measured in 3D (board shifted +Y until it touches): recess corner of V3 0.289; below the ledge = the pocket's +Y clearance
M_RECESS = 0.289
M_POCKET = float(sys.argv[3]) if len(sys.argv) > 3 else P["pocket"]["clr_top_right"]
Y_CORNER = P["board_rects"]["slab"][1] + M_POCKET - 0.03
CWALL = [rect([P["board_rects"]["slab"][1] + M_RECESS - 0.03, OY1, Z_LEDGE, Z_RIM]), rect([Y_CORNER, OY1, Z_BOT, Z_LEDGE])]
def free(th, dY, dz):
    for nm, r in items.items():
        p = poly(r, th, dY, dz)
        if any(not sep(p, w) for w in WALLS): return False
        if nm == "slab" and not all(sep(p, w) for w in CWALL): return False
    p = poly(knob, th, dY, dz)
    return all(sep(p, w) for w in KWALLS)
def lowerable(th, dY, dz):
    for r in list(items.values()) + [knob]:
        for y, z in poly(r, th, dY, dz):
            if not (IY0 <= y <= IY1): return False
    return True
TH = [math.radians(a) for a in range(0, 51)]
DY = [round(0.1*i, 2) for i in range(0, 31)]
DZ = [round(0.1*i, 2) for i in range(-10, 81)]
ok = {}
for i, th in enumerate(TH):
    for j, dy in enumerate(DY):
        for k, dz in enumerate(DZ):
            ok[(i, j, k)] = free(th, dy, dz)
start = [n for n, v in ok.items() if v and lowerable(TH[n[0]], DY[n[1]], DZ[n[2]])]
goal = (0, 0, DZ.index(0.0))
seen = set(start); q = collections.deque(start); prev = {n: None for n in start}
while q:
    n = q.popleft()
    if n == goal: break
    for d in ((1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)):
        m = (n[0]+d[0], n[1]+d[1], n[2]+d[2])
        if m in ok and ok[m] and m not in seen:
            seen.add(m); prev[m] = n; q.append(m)
print("slot z %.3f..%.3f  final pose free: %s  start poses: %d  goal reached: %s" % (s0, s1, ok[goal], len(start), goal in seen))
if goal in seen:
    path = []; n = goal
    while n is not None: path.append(n); n = prev[n]
    path.reverse()
    keep = [path[0]] + [p for a, p in zip(path, path[1:]) if p[0] != a[0] or p[1] != a[1]][::1]
    print("Y limit at the right corners %.3f" % Y_CORNER)
    print("max tilt on the path %d deg; path (tilt deg, dY, dz) from start:" % max(p[0] for p in path))
    for p in keep[::max(1, len(keep)//12)] + [goal]:
        print("   %2d  %.1f  %.1f" % (p[0], DY[p[1]], DZ[p[2]]))
    json.dump({"pivot": K, "poses": [[p[0], DY[p[1]], DZ[p[2]]] for p in path]}, open(HERE + "/insert_path.json", "w"))
