# turnie v1.5 r2 case = case V3 (Fusion "turnie" document, the case in use) with the openings changed for the r2 board.
# Outside, cavity, walls, front recess and its claws, and the USB-C opening are V3 as exported (ref_caseV3_from_fusion.step).
# Changes: the old slide-switch slot (left wall) and the two phi 3 holes are filled; a slot for the r2 SW2 knob is cut in the
# bottom wall; two ribs on the left inner wall hold the board against the USB-C side.
# run: ~/.venvs/cadquery/bin/python make_case_r2.py   (then check_r2.py, insert_check.py, views_r2.py)
import cadquery as cq, json, math, os
from OCP.BRepAlgoAPI import BRepAlgoAPI_Defeaturing
from OCP.TopTools import TopTools_ListOfShape
import board_r2
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = lambda n: os.path.join(HERE, n)

# ---------------- V3 as exported ----------------
v3 = cq.importers.importStep(OUT("ref_caseV3_from_fusion.step")).val()
def plane_at(axis, sign, lo, hi, minarea):
    """coordinate of the planar faces whose normal is sign*axis, inside lo..hi along that axis"""
    vals = []
    for f in v3.Faces():
        if f.geomType() != "PLANE" or f.Area() < minarea: continue
        n = f.normalAt(); c = f.Center()
        if abs(getattr(n, axis) - sign) < 1e-6 and lo < getattr(c, axis) < hi: vals.append(round(getattr(c, axis), 4))
    assert len(set(vals)) == 1, (axis, sign, vals)
    return vals[0]
bb = v3.BoundingBox()
OX0, OX1, OY0, OY1, Z_BOT, Z_RIM = bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax
IX0 = plane_at("x", 1, 28, 29, 200);  IX1 = plane_at("x", -1, 68, 69, 200)      # cavity walls
IY0 = plane_at("y", 1, -14, -13, 300); IY1 = plane_at("y", -1, 13.5, 14.3, 300)
RY0 = plane_at("y", 1, -14.6, -14.2, 50); RY1 = plane_at("y", -1, 14.3, 14.7, 50)  # front recess (acrylic) walls
Z_FLOOR = plane_at("z", 1, -10.5, -9.5, 900); Z_LEDGE = plane_at("z", 1, 3.5, 3.9, 50)
RX0 = plane_at("x", 1, 28.0, 28.5, 30); RX1 = plane_at("x", -1, 69.0, 69.3, 30)
print("V3 outer %.3f x %.3f x %.3f, cavity x %.3f..%.3f y %.3f..%.3f, floor %.3f, ledge %.3f, recess y %.3f..%.3f"
      % (OX1 - OX0, OY1 - OY0, Z_RIM - Z_BOT, IX0, IX1, IY0, IY1, Z_FLOOR, Z_LEDGE, RY0, RY1))

# ---------------- fill the old openings (keep the USB-C opening: it already matches the r2 USB-C, see below) ----------------
sel = []
for f in v3.Faces():
    b = f.BoundingBox(); g = f.geomType()
    slot = b.xmax < IX0 + 0.02 and b.ymin >= -2.89 and b.ymax <= 2.33 and b.zmin >= -1.76 and b.zmax <= 0.26 and not (g == "PLANE" and b.xlen < 0.01)
    hole = g == "CYLINDER" and abs(f._geomAdaptor().Cylinder().Radius() - 1.5) < 1e-3
    if slot or hole: sel.append(f)
lst = TopTools_ListOfShape()
for f in sel: lst.Append(f.wrapped)
dfe = BRepAlgoAPI_Defeaturing(); dfe.SetShape(v3.wrapped); dfe.AddFacesToRemove(lst); dfe.Build(); assert dfe.IsDone()
case = cq.Shape.cast(dfe.Shape())
# the two small bumps on the acrylic recess wall (x 68.44-69.14, z 5.18-5.98, y -9.92..-7.92 / 8.08..10.08) hold the acrylic
# by snapping; removed at the user's request (11/20): cut back to the recess wall plane x = RX1
before = case.Volume()
for y0, y1 in ((-10.2, -7.6), (7.8, 10.4)):
    case = case.cut(cq.Workplane("XY").box(RX1 - 68.0, y1 - y0, 1.5, centered=False).translate((68.0, y0, 5.0)).val()).clean()
bump_vol = before - case.Volume()
assert not [f for f in case.Faces() if f.BoundingBox().xmin >= 68.0 and f.BoundingBox().xmax < RX1 - 0.01 and f.BoundingBox().zmin >= 5.0], "bumps left"
print("acrylic bumps removed: %.3f mm3" % bump_vol)
# the left-wall phi 3 hole runs into the corner surface and is left by the defeaturing: plug it with the wall itself
back = min([f for f in v3.Faces() if f.geomType() == "PLANE"], key=lambda f: f.Center().z)
floor = [f for f in v3.Faces() if f.geomType() == "PLANE" and abs(f.Center().z - Z_FLOOR) < 1e-3 and f.Area() > 900][0]
env = cq.Solid.extrudeLinear(cq.Face.makeFromWires(back.outerWire()), cq.Vector(0, 0, Z_RIM - Z_BOT))
cav = cq.Solid.extrudeLinear(cq.Face.makeFromWires(floor.outerWire()), cq.Vector(0, 0, Z_LEDGE - Z_FLOOR))
for f in case.Faces():
    if f.geomType() == "CYLINDER" and abs(f._geomAdaptor().Cylinder().Radius() - 1.5) < 1e-3:
        c = f._geomAdaptor().Cylinder().Location(); a = f._geomAdaptor().Cylinder().Axis().Direction()
        assert abs(a.X()) > 0.99, "only the left-wall hole is expected here"
        plug = cq.Workplane("YZ").circle(1.65).extrude(4).translate((OX0 - 1, c.Y(), c.Z())).val().intersect(env).cut(cav)
        case = case.fuse(plug).clean()
assert not [f for f in case.Faces() if f.geomType() == "CYLINDER" and abs(f._geomAdaptor().Cylinder().Radius() - 1.5) < 1e-3]
filled = case.Volume() - v3.Volume()

# ---------------- board placement ----------------
CENTRED = True        # 11/20 user: board in the middle of the cavity like the old board (about 2.0 per side); False = pushed to the USB-C wall
GAP_USB = (IX1 - IX0 - (166.075 - 130.325)) / 2 if CENTRED else 0.30   # board edge (USB-C side) to the right inner wall
LED_GAP = 0.67        # LED tops to the acrylic underside (= ledge). Old module: LED tops z 3.01 in the V3 document (body "LED (1) (1)"),
                      # which also puts the r2 USB-C on the centre of the V3 USB-C opening (checked below)
XOFF = IX1 - GAP_USB + 130.325
YOFF = (IY0 + IY1) / 2 - (80.95 + 107.35) / 2                 # board centred in y (0.765 per side)
ZOFF = (Z_LEDGE - LED_GAP) - board_r2.S_LED_TOP
zF, zB = board_r2.S_F + ZOFF, board_r2.S_B + ZOFF
X = lambda xk: XOFF - xk
Y = lambda yk: yk + YOFF
board = board_r2.load(XOFF, YOFF, ZOFF)

# USB-C: V3 opening is 11.0 x 4.0 (full round ends) centred y 0.00, z -1.13; the r2 receptacle centre after placement:
ub = board["U1_usbc"].BoundingBox()
USB_Y, USB_Z = Y(94.153), board_r2.S_F + 0.822 + ZOFF        # receptacle shell centre (measured on the STEP section, 11/19)
print("USB-C centre y %.3f z %.3f (V3 opening centre 0.000, -1.130); mouth x %.3f, %.3f inside the outer face" % (USB_Y, USB_Z, ub.xmax, OX1 - ub.xmax))

# ---------------- SW2 knob slot (bottom wall, y-) ----------------
kb = board["SW2_knob"].BoundingBox(); ko = board["SW2_knob_other_pos"].BoundingBox()
SLOT_CLR_X = 0.40     # along the travel (0.4 so the R0.8 corners stay 0.19 from the knob corners)
SLOT_CLR = 0.30       # above / below the knob
SLOT_DOWN = 0.30      # extra depth below the knob so the board can be tilted in with the knob first (insert_check.py)
SLOT_R = 0.8          # corner radius as the V3 slide-switch slot
SX0, SX1 = min(kb.xmin, ko.xmin) - SLOT_CLR_X, max(kb.xmax, ko.xmax) + SLOT_CLR_X
SZ0, SZ1 = kb.zmin - SLOT_CLR - SLOT_DOWN, kb.zmax + SLOT_CLR
slot = (cq.Workplane("XZ").center((SX0 + SX1) / 2, (SZ0 + SZ1) / 2).rect(SX1 - SX0, SZ1 - SZ0).extrude(-(IY0 - OY0 + 2))
        .edges("|Y").fillet(SLOT_R).translate((0, OY0 - 1, 0)))
case = case.cut(slot.val())

# ---------------- corner pockets: the V3 inner corners are ellipses (about 4.2 x 4.0); with the board 0.30 from the USB-C wall
# its R1.5 corners and the header plastic would cut into the two right corners. Clear them from the inside
# (board outline + POCKET_CLR, R1.5 + POCKET_CLR), from below the board up to the acrylic ledge, because the corners pass there
# on the way in. The top-right corner gets more room in +y (POCKET_TR_EXTRA): the board goes in tilted with the SW2 knob first,
# and on the way that corner sits up to ~0.3 further +y (insert_check.py). The outside is not touched.
POCKET_CLR = 0.25
POCKET_TR_EXTRA = 0.35
PZ0, PZ1 = zB - 0.3, Z_LEDGE
def pocket_at(dy):
    return (cq.Workplane("XY").rect(166.075 - 130.325 + 2 * POCKET_CLR, 107.35 - 80.95 + 2 * POCKET_CLR).extrude(PZ1 - PZ0)
            .edges("|Z").fillet(1.5 + POCKET_CLR).translate((X((130.325 + 166.075) / 2), Y((80.95 + 107.35) / 2) + dy, PZ0)).val())
right_top = cq.Workplane("XY").box(10, 20, 30, centered=False).translate((X(130.325) - 8, 0, PZ0 - 1)).val()
pocket = pocket_at(0).fuse(pocket_at(POCKET_TR_EXTRA).intersect(right_top))
before = case.Volume()
if not CENTRED: case = case.cut(pocket).clean()   # centred: the corners clear the V3 inner corners, no pockets
pocket_vol = before - case.Volume()

# ---------------- board locating ribs (left inner wall) ----------------
RIB_CLR, RIB_W = 0.15, 1.2
RIB_Y = (-5.5, 4.0)
bx0 = X(166.075)                                              # board edge on the left
RZ0, RZ1 = zB - 0.3, zF + 0.3
bx1 = X(130.325)                                              # board edge on the USB-C side
RIBS = [(IX0 - 0.05, bx0 - RIB_CLR, yc) for yc in RIB_Y]
if CENTRED: RIBS += [(bx1 + RIB_CLR, IX1 + 0.05, yc) for yc in (-9.5, 9.5)]   # USB-C side too, clear of the notch
for xa, xb, yc in RIBS:
    rib = cq.Workplane("XY").box(xb - xa, RIB_W, RZ1 - RZ0, centered=False).translate((xa, yc - RIB_W / 2, RZ0))
    case = case.fuse(rib.edges(">Z and |Y").chamfer(0.3).val()).clean()      # 45 deg lead-in on the top edge

# ---------------- reference battery ----------------
BATT = (36.0, 24.5, 5.6)
BATT_X0 = 29.90      # the left inner corners (ellipses) reach x 29.51 at the cell's corner height; 0.39 more. PCM end on the right (USB-C side)
battery = cq.Workplane("XY").box(*BATT, centered=False).translate((BATT_X0, (IY0 + IY1) / 2 - BATT[1] / 2, Z_FLOOR)).val()

# ---------------- strap holes + key holder (11/20): V3 had two phi 3 holes around one corner for a strap. The side-wall hole is
# kept where V3 had it; the bottom-wall one moves from x 34.89 to x 30.4 because the SW2 slot now takes that place.
STRAP_Z, STRAP_R = 0.95, 2.1     # phi 4.2 (V3 had 3.0): the ring wire crosses the wall at an angle
STRAP_SIDE_Y, STRAP_BOTTOM_X = -8.6, 31.1      # 11/20: moved apart a little for the phi 30 ring
# 11/20 user: the ring goes on the top-left corner seen from the top (= the USB-C side corner); V3's holes were on the other corner,
# so they are mirrored across the cavity centre: side hole in the USB-C wall, bottom hole at x = 2*cx - 30.4
STRAP_BOTTOM_X = (IX0 + IX1) - STRAP_BOTTOM_X
case = case.cut(cq.Workplane("YZ").circle(STRAP_R).extrude(6).translate((OX1 - 4, STRAP_SIDE_Y, STRAP_Z)).val())
case = case.cut(cq.Workplane("XZ").circle(STRAP_R).extrude(-6).translate((STRAP_BOTTOM_X, OY0 - 2, STRAP_Z)).val())
# key holder = one silver split ring through both holes (11/20 user): ring in the hole plane, through the hole centres
import math
RING_R, RING_WIRE = float(os.environ.get("RING_R", 9.0)), 0.7                     # ring phi 18 centre line, wire 1.4, outside 19.4 (the size the user liked on the first version), wire phi 1.2
HA, HB = (OX1 - (OX1 - IX1) / 2, STRAP_SIDE_Y), (STRAP_BOTTOM_X, -13.95)
HM = ((HA[0] + HB[0]) / 2, (HA[1] + HB[1]) / 2); hv = (HB[0] - HA[0], HB[1] - HA[1]); hl = math.hypot(*hv); hn = (-hv[1] / hl, hv[0] / hl)
hd = math.sqrt(RING_R ** 2 - (hl / 2) ** 2)
if hn[0] < 0: hn = (-hn[0], -hn[1])           # centre on the outer side of the corner (+x, -y)
ring = cq.Solid.makeTorus(RING_R, RING_WIRE, pnt=cq.Vector(HM[0] + hn[0] * hd, HM[1] + hn[1] * hd, STRAP_Z), dir=cq.Vector(0, 0, 1))
print("ring R %.1f vs case %.4f, vs board %.4f" % (RING_R, ring.intersect(case).Volume(), ring.intersect(cq.Compound.makeCompound(list(board.values()))).Volume()))

# ---------------- top plate (acrylic panel): fills the V3 recess outline, 2.0 thick on the ledge, hole for the SW1 plunger ----------------
T_TOP = 2.0
SW1_HOLE = 4.6        # 11/20: phi 4.4 shaft of the 3DP cap (was 4.1 for the bare 9.5 mm plunger)
slab = cq.Workplane("XY").box(100, 100, T_TOP, centered=False).translate((0, -50, Z_LEDGE)).val()
top = slab.intersect(env).cut(case)
top = top.cut(cq.Workplane("XY").circle(SW1_HOLE / 2).extrude(5).translate((X(162.1), Y(102.6), Z_LEDGE - 1)).val())
tb = top.BoundingBox()
print("top plate %.3f x %.3f x %.3f, SW1 hole phi %.1f at (%.3f, %.3f)" % (tb.xlen, tb.ylen, tb.zlen, SW1_HOLE, X(162.1), Y(102.6)))

# ---------------- SW1 cap (11/20 user): generic 6x6 tact, 5 mm (KiCad SW_PUSH_6mm_H5mm) + 3DP cap from Fusion "turnie_ボタン修正版"
# (body スイッチカバーv2_3.7 (1) (1) (2)): flange phi 5.9 x 0.5, shaft phi 4.4, 6.0 overall, blind hole phi 3.7 x 4.5 from the bottom.
# Placed with its bottom on the plunger top; the flange stays under the acrylic.
CAP_FL_D, CAP_FL_T, CAP_D, CAP_H, CAP_HOLE_D, CAP_HOLE_H = 5.9, 0.5, 4.4, 6.0, 3.7, 4.5
sw1_top = board["SW1"].BoundingBox().zmax
cap = (cq.Workplane("XY").circle(CAP_FL_D / 2).extrude(CAP_FL_T).faces(">Z").workplane().circle(CAP_D / 2).extrude(CAP_H - CAP_FL_T)
       .faces("<Z").workplane().hole(CAP_HOLE_D, CAP_HOLE_H)).translate((X(162.1), Y(102.6), sw1_top)).val()
print("SW1 cap: bottom z %.3f (plunger top), flange top %.3f = %.3f under the acrylic, top z %.3f = %.3f above the acrylic, %.3f above the rim; cap x case %.4f, cap x top %.4f"
      % (sw1_top, sw1_top + CAP_FL_T, Z_LEDGE - sw1_top - CAP_FL_T, sw1_top + CAP_H, sw1_top + CAP_H - Z_LEDGE - T_TOP, sw1_top + CAP_H - Z_RIM,
         cap.intersect(case).Volume(), cap.intersect(top).Volume()))

# ---------------- outputs ----------------
# Output frame = the product orientation (11/20 user): acrylic on top (+z), seen from the top the button SW1 at the bottom right.
# V3 frame -> output: turn 180 deg about z and centre the case outline on x = y = 0 (z unchanged: rim z 5.983, back -11.617).
CX, CY = (OX0 + OX1) / 2, (OY0 + OY1) / 2
def T(sh): return sh.translate((-CX, -CY, 0)).rotate((0, 0, 0), (0, 0, 1), 180)
bcomp = cq.Compound.makeCompound([v for k, v in board.items() if k != "SW2_knob_other_pos"])
for sh, fn in ((case, "turnie_v1.5_r2_caseV3.step"), (top, "turnie_v1.5_r2_top_plate.step"), (battery, "ref_battery_36x24.5x5.6.step"),
               (bcomp, "turnie_v1.5r2_board_positioned.step"), (ring, "turnie_v1.5_r2_keyholder.step"), (cap, "turnie_v1.5_r2_sw1_cap.step")):
    cq.exporters.export(cq.Workplane(obj=T(sh)), OUT(fn))
asm = cq.Assembly(name="turnie_v1.5_r2_caseV3")
asm.add(T(case), name="case", color=cq.Color(0.85, 0.85, 0.82))
asm.add(T(ring), name="key_ring", color=cq.Color(0.75, 0.75, 0.78))
asm.add(T(top), name="top_plate", color=cq.Color(0.85, 0.9, 0.95, 0.5))
asm.add(T(cap), name="sw1_cap", color=cq.Color(0.64, 0.17, 0.0))
asm.add(T(battery), name="battery_ref", color=cq.Color(0.6, 0.6, 0.65))
asm.add(T(bcomp), name="board_r2_ref", color=cq.Color(0.2, 0.45, 0.25))
asm.save(OUT("turnie_v1.5_r2_caseV3_assembly.step"))
sw = T(board["SW1"]).BoundingBox()
print("output frame: SW1 centre (%.2f, %.2f) -> should be x > 0, y < 0 (bottom right seen from the top)" % (sw.center.x, sw.center.y))

def r3(v): return round(v, 3)
sw1 = board["SW1"].BoundingBox()
P = dict(
    zF=r3(zF), zB=r3(zB), IX0=IX0, IX1=IX1, IY0=IY0, IY1=IY1, OX0=r3(OX0), OX1=r3(OX1), OY0=r3(OY0), OY1=r3(OY1),
    RX0=RX0, RX1=RX1, RY0=RY0, RY1=RY1, Z_LEDGE=Z_LEDGE, Z_RIM=r3(Z_RIM), Z_BOT=r3(Z_BOT), Z_FLOOR=Z_FLOOR,
    XOFF=r3(XOFF), YOFF=r3(YOFF), ZOFF=r3(ZOFF), GAP_USB=GAP_USB, LED_GAP=LED_GAP,
    usb_centre=[r3(USB_Y), r3(USB_Z)], usb_mouth_depth=r3(OX1 - ub.xmax),
    slot=[r3(SX0), r3(SX1), r3(SZ0), r3(SZ1), SLOT_R], pocket=dict(clr=POCKET_CLR, clr_top_right=round(POCKET_CLR + POCKET_TR_EXTRA, 3), z=[r3(PZ0), r3(PZ1)], vol=r3(pocket_vol)), ribs=dict(y=RIB_Y, x=[IX0, r3(bx0 - RIB_CLR)], z=[r3(RZ0), r3(RZ1)]),
    sw1_centre=[r3(X(162.1)), r3(Y(102.6))], sw1_top=r3(sw1.zmax), sw1_hole=SW1_HOLE, cap_top=r3(sw1_top + CAP_H),
    knob=[r3(kb.xmin), r3(kb.xmax), r3(kb.ymin), r3(kb.zmin), r3(kb.zmax)], knob_other_x=[r3(ko.xmin), r3(ko.xmax)],
    battery=[r3(BATT_X0), r3(BATT_X0 + BATT[0]), r3((IY0 + IY1) / 2 - BATT[1] / 2), r3((IY0 + IY1) / 2 + BATT[1] / 2), Z_FLOOR, r3(Z_FLOOR + BATT[2])],
    volume=dict(V3=r3(v3.Volume()), filled=r3(filled), bumps_removed=r3(bump_vol), r2=r3(case.Volume())),
    # Y-z rectangles of the board in the final pose (for insert_check.py)
    board_rects=dict(
        slab=[r3(Y(80.95)), r3(Y(107.35)), r3(zB), r3(zF)],
        tails=[r3(Y(82.732) - 0.5), r3(Y(105.592) + 0.5), r3(zB - board_r2.TRIM), r3(zB)],
        hdr=[r3(Y(82.732) - 1.25), r3(Y(105.592) + 1.25), r3(zF), r3(board_r2.S_MOD_BOT + ZOFF)],
        module=[r3(Y(81.653)), r3(Y(106.671)), r3(board_r2.S_MOD_BOT + ZOFF), r3(board_r2.S_LED_TOP + ZOFF)],
        sw2=[r3(Y(80.95)), r3(Y(84.45)), r3(zF), r3(zF + 3.5)],
        sw1=[r3(sw1.ymin), r3(sw1.ymax), r3(zF), r3(sw1.zmax)]),
    knob_rect=[r3(kb.ymin), r3(Y(81.25)), r3(kb.zmin), r3(kb.zmax)],
    battery_rect=[r3((IY0 + IY1) / 2 - BATT[1] / 2), r3((IY0 + IY1) / 2 + BATT[1] / 2), Z_FLOOR, r3(Z_FLOOR + BATT[2])],
)
json.dump(P, open(OUT("params.json"), "w"), indent=1, ensure_ascii=False)
print("corner pockets %.3f mm3" % pocket_vol)
print("filled %.3f mm3; r2 volume %.3f (V3 %.3f)" % (filled, case.Volume(), v3.Volume()))
print("slot x %.3f..%.3f z %.3f..%.3f; SW1 centre (%.3f, %.3f) top z %.3f; board z F %.3f B %.3f" % (SX0, SX1, SZ0, SZ1, X(162.1), Y(102.6), sw1.zmax, zF, zB))
