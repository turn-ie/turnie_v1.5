# interference check: enclosure parts vs board components (kicad-cli STEP of turnie_v1.5.kicad_pcb), real 1.6 mm board
# run after make_enclosure.py:  ~/.venvs/cadquery/bin/python check.py | tee check_result.txt
import cadquery as cq, os, json
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
HERE = os.path.dirname(os.path.abspath(__file__))
d = json.load(open(HERE+"/dims.json")); z = d["z"]; Z0 = d["Z0_step"]
zF, zB, zLED = z["board_F"], z["board_B"], z["led_top"]

def bbx(s):
    b = Bnd_Box(); BRepBndLib.AddOptimal_s(s.wrapped, b, False, False); return b.Get()
def place(s):          # STEP (z up = LED side) -> enclosure frame (x, KiCad y, depth from the front face)
    return s.rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, Z0))

doc = TDocStd_Document(TCollection_ExtendedString("x"))
r = STEPCAFControl_Reader(); r.SetNameMode(True); r.ReadFile(HERE+"/turnie_v1.5_board_kicad.step"); r.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()); ls = TDF_LabelSequence(); st.GetFreeShapes(ls)
comps = TDF_LabelSequence(); st.GetComponents_s(ls.Value(1), comps)
def name(l):
    n = TDataStd_Name(); return n.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(), n) else "?"
board = {}
for i in range(1, comps.Length()+1):
    c = comps.Value(i); nm = name(c); s = cq.Shape.cast(st.GetShape_s(c))
    if nm.startswith("=>"): continue                     # board core (1.51 only): replaced by a 1.6 slab below
    if nm == "U1":                                       # split the module into named groups
        g = {"U1_pcb": [], "U1_leds": [], "U1_usbc": [], "U1_antenna": [], "U1_buttons": [], "U1_underside": []}
        for so in s.Solids():
            x0, y0, z0, x1, y1, z1 = bbx(so)
            if so.Volume() > 500: g["U1_pcb"].append(so)
            elif z0 > 5.7: g["U1_leds"].append(so)
            elif x0 < 130.5: g["U1_usbc"].append(so)
            elif x0 > 153.5 and x1 < 156.5 and so.Volume() > 5: g["U1_antenna"].append(so)
            elif x0 > 133.0 and x1 < 136.2 and z0 < 3.0: g["U1_buttons"].append(so)
            else: g["U1_underside"].append(so)
        for k, v in g.items(): board[k] = place(cq.Compound.makeCompound(v))
        continue
    board[nm] = place(s)
# SW2 knob in its other position (datasheet travel 2.0 toward -x): copy of the knob solid
knob = [so for so in board["SW2"].Solids() if bbx(so)[1] < 79.0]          # the only solid past the board edge
assert len(knob) == 1
board["SW2_knob_other_pos"] = cq.Compound.makeCompound([k.translate((-2.0, 0, 0)) for k in knob])
# real 1.6 board slab with the outline (R1.5 corners) and the USB-C notch
pcb = (cq.Workplane("XY").rect(166.075-130.325, 107.35-80.95).extrude(1.6).edges("|Z").fillet(1.5)
       .translate(((130.325+166.075)/2, (80.95+107.35)/2, zF))
       .cut(cq.Workplane("XY").box(138.5-130.325+1, 100.5-87.8, 3, centered=False).translate((130.325-1, 87.8, zF-0.5))))
board["PCB"] = pcb.val()
# assumed 2.54 header (datasheet PH-1x40SG: plastic 2.50, tails 3.0): plastic between the boards,
# tails 1.4 behind the B face, LED-side ends trimmed to the LED top (see README)
hp, ht = [], []
for y in (82.732, 105.592):
    hp.append(cq.Workplane("XY").box(25.4, 2.5, 2.5, centered=False).translate((132.136-1.27, y-1.25, zF-2.5)).val())
    for i in range(10):
        ht.append(cq.Workplane("XY").box(0.64, 0.64, zB+1.4-zLED, centered=False).translate((132.136+2.54*i-0.32, y-0.32, zLED)).val())
board["HDR_plastic(assumed)"] = cq.Compound.makeCompound(hp)
board["HDR_pins(assumed)"] = cq.Compound.makeCompound(ht)

names = ["turnie_v1.5_01_case", "turnie_v1.5_02_back_lid", "turnie_v1.5_03_battery_tray", "ref_battery_36x24.5x5.6"]
parts = {n: cq.importers.importStep(f"{HERE}/{n}.step").val() for n in names}
print("board STEP: turnie_v1.5_board_kicad.step (kicad-cli pcb export step --subst-models), placed with z' =", Z0, "- z_step")
print("board items:", len(board), "->", ", ".join(sorted(board)))
res = []
for pn, p in parts.items():
    for bn, b in board.items():
        try: v = p.intersect(b).Volume()
        except Exception as e: v = -1
        if abs(v) > 1e-4: res.append((pn, bn, round(v, 4)))
for i in range(4):
    for j in range(i+1, 4):
        res.append((names[i], names[j], round(parts[names[i]].intersect(parts[names[j]]).Volume(), 4)))
print("intersections (mm^3), part vs board items with volume > 1e-4 and every part pair:")
for x in res: print("  ", x)
print("sanity (PCB +1 mm in x vs case):", round(parts[names[0]].intersect(pcb.val().translate((1, 0, 0))).Volume(), 3))
if not os.path.exists(HERE+"/board_positioned_ref.step"):
    cq.exporters.export(cq.Workplane(obj=cq.Compound.makeCompound([v for k, v in board.items() if k != "SW2_knob_other_pos"])), HERE+"/board_positioned_ref.step")

def dist(a, b):
    e = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped); return round(e.Value(), 3)
C, L, T, BT = [parts[n] for n in names]
print("min distances (mm):")
for pn, p, refs in (
    ("case", C, ["PCB", "U1_pcb", "U1_leds", "U1_usbc", "SW1", "SW2", "SW2_knob_other_pos", "D3", "Q2", "R5", "HDR_pins(assumed)", "HDR_plastic(assumed)"]),
    ("lid", L, ["SW2", "SW2_knob_other_pos", "SW1", "PCB", "HDR_pins(assumed)"]),
    ("tray", T, ["PCB", "SW1", "SW2", "HDR_pins(assumed)", "U1_usbc", "U1_antenna", "D3"]),
    ("battery", BT, ["U1_antenna", "U1_usbc", "PCB", "SW1", "HDR_pins(assumed)"])):
    print("  ", pn, {r: dist(p, board[r]) for r in refs})
print("   case-tray", dist(C, T), "lid-tray", dist(L, T), "case-lid", dist(C, L))
print("   battery: case", dist(BT, C), "lid", dist(BT, L), "tray", dist(BT, T))
ant = board["U1_antenna"]; ab = bbx(ant)
print("   antenna bbox x %.2f..%.2f y %.2f..%.2f z' %.2f..%.2f" % (ab[0], ab[3], ab[1], ab[4], ab[2], ab[5]))
print("   antenna to battery %.3f, to tray %.3f" % (dist(ant, BT), dist(ant, T)))
ub = bbx(board["U1_usbc"]); print("   USB-C mouth x %.3f (outer wall x %.3f), z' %.3f..%.3f" % (ub[0], d["outer_xy"][0], ub[2], ub[5]))
bb = bbx(board["U1_buttons"]); print("   module BOOT/RESET buttons x %.2f..%.2f y %.2f..%.2f z' %.2f..%.2f (face the shield, gap to PCB %.3f)" % (bb[0], bb[3], bb[1], bb[4], bb[2], bb[5], dist(board["U1_buttons"], board["PCB"])))
sb = bbx(board["SW1"]); print("   SW1 plunger top z' %.3f (front face z' 0)" % sb[2])
kb = bbx(board["SW2"]); print("   SW2 knob tip y %.3f (outer top wall y %.3f)" % (kb[1], d["outer_xy"][2]))
print("sanity lid -0.3 z vs tray:", round(L.translate((0, 0, -0.3)).intersect(T).Volume(), 3), "  lid +0.3 x vs case:", round(L.translate((0.3, 0, 0)).intersect(C).Volume(), 3))
print("sanity tray -0.1 z (toward the board) vs PCB:", round(T.translate((0, 0, -0.1)).intersect(board["PCB"]).Volume(), 3))
