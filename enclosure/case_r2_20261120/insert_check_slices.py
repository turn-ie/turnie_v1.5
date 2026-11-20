# Insertion path search on several Y-z sections at once (x = const), with the case taken from the real r2 STEP
# (inner corners, pockets, acrylic recess, knob slot all included) and the board items cut by the same planes.
# A pose (tilt th about the x-axis through the knob centre at the inner wall face, then shift dY, dz) is free if, on
# every section, no sampled point of any board item's section rectangle lies in case material.
# Poses reachable from a pose that can be lowered straight down are found by BFS; the path is re-checked in 3D.
import cadquery as cq, numpy as np, json, os, math, collections, sys
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_OUT
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.gp import gp_Pln, gp_Dir
import board_r2
HERE = os.path.dirname(os.path.abspath(__file__))
P = json.load(open(HERE + "/params.json"))
case = cq.importers.importStep(HERE + "/turnie_v1.5_r2_caseV3.step").val()
B = board_r2.load(P["XOFF"], P["YOFF"], P["ZOFF"])
ITEMS = ["PCB", "HDR_plastic", "HDR_pins", "U1_pcb", "U1_usbc", "SW1", "SW2", "SW2_knob", "D3"]
xb0, xb1 = P["XOFF"] - 166.075, P["XOFF"] - 130.325
SLICES = sorted(set([round(v, 2) for v in (xb0 + 0.05, xb0 + 0.6, 34.0, 35.8, 36.4, 38.4, 50.0, 64.6, 65.6, 66.4, 67.0, 67.5, 67.9, 68.2, xb1 - 0.03)]))
DY0, DZ0, ST = -16.6, -12.0, 0.05
NY, NZ = int(33.2 / ST) + 1, int(20.0 / ST) + 1
Ky, Kz = P["IY0"], (P["knob"][3] + P["knob"][4]) / 2

def occupancy(x):
    cl = BRepClass3d_SolidClassifier(case.wrapped)
    g = np.zeros((NY, NZ), bool)
    for i in range(NY):
        y = DY0 + i * ST
        for k in range(NZ):
            z = DZ0 + k * ST
            if -12.3 < y < 12.3 and -9.6 < z < 3.3: continue          # surely inside the cavity
            cl.Perform(gp_Pnt(x, y, z), 1e-6)
            g[i, k] = cl.State() != TopAbs_OUT
    return g

def section_rect(shape, x):
    sec = BRepAlgoAPI_Section(shape.wrapped, gp_Pln(gp_Pnt(x, 0, 0), gp_Dir(1, 0, 0))); sec.Build()
    s = cq.Shape.cast(sec.Shape())
    if not s.Edges(): return None
    b = s.BoundingBox(); return (b.ymin, b.ymax, b.zmin, b.zmax)

def perim(r, step=0.04):
    y0, y1, z0, z1 = r
    ny = max(2, int((y1 - y0) / step) + 1); nz = max(2, int((z1 - z0) / step) + 1)
    ys = np.linspace(y0, y1, ny); zs = np.linspace(z0, z1, nz)
    pts = [np.c_[ys, np.full(ny, z0)], np.c_[ys, np.full(ny, z1)], np.c_[np.full(nz, y0), zs], np.c_[np.full(nz, y1), zs]]
    if y1 - y0 > 0.3 and z1 - z0 > 0.3:                                 # a coarse interior grid too
        YY, ZZ = np.meshgrid(np.linspace(y0, y1, max(2, ny // 5)), np.linspace(z0, z1, max(2, nz // 5)))
        pts.append(np.c_[YY.ravel(), ZZ.ravel()])
    return np.vstack(pts)

cache = HERE + "/.slices_cache.npz"
key = json.dumps([SLICES, P["XOFF"], P["volume"]["r2"]])
if os.path.exists(cache) and str(np.load(cache, allow_pickle=True)["key"]) == key:
    d = np.load(cache, allow_pickle=True); OCC = list(d["occ"]); PTS = list(d["pts"])
else:
    OCC, PTS = [], []
    for x in SLICES:
        OCC.append(occupancy(x))
        rs = [section_rect(B[n], x) for n in ITEMS]
        PTS.append(np.vstack([perim(r) for r in rs if r is not None]))
        print("slice x %.2f: %d board points" % (x, len(PTS[-1])), flush=True)
    np.savez(cache, key=key, occ=np.array(OCC, dtype=object), pts=np.array(PTS, dtype=object))

def free(th, dy, dz):
    c, s = math.cos(th), math.sin(th)
    for g, p in zip(OCC, PTS):
        y = p[:, 0] - Ky; z = p[:, 1] - Kz
        Y = Ky + c * y - s * z + dy; Z = Kz + s * y + c * z + dz
        i = np.round((Y - DY0) / ST).astype(int); k = np.round((Z - DZ0) / ST).astype(int)
        inside = (i >= 0) & (i < NY) & (k >= 0) & (k < NZ)
        if g[i[inside], k[inside]].any(): return False
    return True
def lowerable(th, dy, dz):            # every point inside the cavity's straight y-range, so it can come straight down
    c, s = math.cos(th), math.sin(th)
    for p in PTS:
        y = p[:, 0] - Ky; z = p[:, 1] - Kz
        Y = Ky + c * y - s * z + dy
        if (Y < P["IY0"] + 0.02).any() or (Y > P["IY1"] - 0.6).any(): return False
    return True

TH = [math.radians(a) for a in range(0, 46)]
DY = [round(0.1 * i, 2) for i in range(0, 31)]
DZ = [round(0.1 * i, 2) for i in range(-5, 81)]
ok = {}
def isok(n):
    if n not in ok: ok[n] = free(TH[n[0]], DY[n[1]], DZ[n[2]])
    return ok[n]
goal = (0, 0, DZ.index(0.0))
print("final pose free:", isok(goal))
# search backwards from the goal (lazy), until a lowerable pose is met
seen = {goal: None}; q = collections.deque([goal]); found = None
while q:
    n = q.popleft()
    if lowerable(TH[n[0]], DY[n[1]], DZ[n[2]]) and DY[n[1]] >= 2.3: found = n; break
    for d in ((1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)):
        m = (n[0]+d[0], n[1]+d[1], n[2]+d[2])
        if 0 <= m[0] < len(TH) and 0 <= m[1] < len(DY) and 0 <= m[2] < len(DZ) and m not in seen and isok(m):
            seen[m] = n; q.append(m)
print("poses evaluated %d, path found: %s" % (len(ok), found is not None))
if found:
    path = []; n = found
    while n is not None: path.append(n); n = seen[n]
    poses = [[TH[p[0]] * 180 / math.pi, DY[p[1]], DZ[p[2]]] for p in path]
    json.dump({"pivot": [Ky, Kz], "poses": [[round(a, 1), b, c] for a, b, c in poses]}, open(HERE + "/insert_path.json", "w"))
    print("path (tilt deg, dY, dz), start -> final:")
    for a, b, c in poses[::max(1, len(poses) // 14)] + [poses[-1]]: print("   %4.0f  %.1f  %.1f" % (a, b, c))
