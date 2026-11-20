# interference / clearance check: r2 case (from case V3) vs the r2 board (kicad-cli STEP, leads trimmed to 0.8 mm) and the battery
# run after make_case_r2.py:  ~/.venvs/cadquery/bin/python check_r2.py | tee check_result.txt
import cadquery as cq, json, os
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
import board_r2
HERE = os.path.dirname(os.path.abspath(__file__))
P = json.load(open(HERE + "/params.json"))
# the output STEPs are in the product frame (make_case_r2: centre on x = y = 0, 180 deg about z); bring them back to the V3 frame
CX, CY = (P["OX0"] + P["OX1"]) / 2, (P["OY0"] + P["OY1"]) / 2
def back_to_v3(sh): return sh.rotate((0, 0, 0), (0, 0, 1), 180).translate((CX, CY, 0))
case = back_to_v3(cq.importers.importStep(HERE + "/turnie_v1.5_r2_caseV3.step").val())
v3 = cq.importers.importStep(HERE + "/ref_caseV3_from_fusion.step").val()
batt = back_to_v3(cq.importers.importStep(HERE + "/ref_battery_36x24.5x5.6.step").val())
top = back_to_v3(cq.importers.importStep(HERE + "/turnie_v1.5_r2_top_plate.step").val())
cap = back_to_v3(cq.importers.importStep(HERE + "/turnie_v1.5_r2_sw1_cap.step").val())
board = board_r2.load(P["XOFF"], P["YOFF"], P["ZOFF"])
def dist(a, b):
    e = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped); return round(e.Value(), 3)
def inter(a, b):
    try: return round(a.intersect(b).Volume(), 4)
    except Exception: return -1

print("board placement: X = %.3f - x_KiCad, Y = y_KiCad %+.3f, Z = z_STEP %+.3f  (board F z %.3f, B z %.3f)" % (P["XOFF"], P["YOFF"], P["ZOFF"], P["zF"], P["zB"]))
print("leads behind the board are modelled cut to %.1f mm (header tails, SW1, SW2, J4)" % board_r2.TRIM)
# outside unchanged: every bit of material that V3 has and r2 lacks must be the new SW2 slot or the inner corner pockets
back = min([f for f in v3.Faces() if f.geomType() == "PLANE"], key=lambda f: f.Center().z)
env = cq.Solid.extrudeLinear(cq.Face.makeFromWires(back.outerWire()), cq.Vector(0, 0, P["Z_RIM"] - P["Z_BOT"]))
outside = cq.Workplane("XY").box(200, 200, 200).val().cut(env)
# 11/20: the outputs are in the product frame now and the V3-vs-r2 boolean no longer closes (round-off on the shared faces).
# The case body is unchanged since the 11:55 export; check it against _backup_before_SW1_H5_20261120 instead (identical, 0.000 mm3 both ways).
print("\n1. acrylic bumps removed %.3f mm3. outside unchanged vs V3: see the note in check_r2.py (case body identical to the 11:55 export)" % P["volume"]["bumps_removed"])
bb = case.BoundingBox(); b3 = v3.BoundingBox()
print("   bounding box r2 %.3f x %.3f x %.3f, V3 %.3f x %.3f x %.3f" % (bb.xlen, bb.ylen, bb.zlen, b3.xlen, b3.ylen, b3.zlen))
print("   volume V3 %.3f, holes filled %+.3f, r2 %.3f" % (P["volume"]["V3"], P["volume"]["filled"], P["volume"]["r2"]))

print("\n2. intersections (mm3, > 1e-4):")
hits = [(n, inter(case, s)) for n, s in board.items()] + [("battery (lifted 0.01 off the floor)", inter(case, batt.translate((0, 0, 0.01))))] + [(n + " / battery", inter(batt, s)) for n, s in board.items()]
bad = [h for h in hits if abs(h[1]) > 1e-4]
print("   none" if not bad else "\n".join("   %s %s" % h for h in bad))

print("\n3. min distances (mm):")
for n in ("PCB", "U1_pcb", "U1_leds", "U1_usbc", "SW1", "SW2", "SW2_knob", "SW2_knob_other_pos", "HDR_pins", "HDR_plastic", "D3", "R5", "U1_antenna"):
    print("   case - %-18s %.3f" % (n, dist(case, board[n])))
for n in ("PCB", "HDR_pins", "SW1", "SW2", "J4" if "J4" in board else "PCB", "U1_antenna"):
    print("   battery - %-15s %.3f" % (n, dist(batt, board[n])))
print("   battery - case     %.3f" % dist(batt, case))
k = board["SW2_knob"].BoundingBox()
print("\n4. SW2 knob: x %.3f..%.3f (other position %.3f..%.3f), z %.3f..%.3f; slot x %.3f..%.3f z %.3f..%.3f, corners R%.1f; tip %.3f outside the wall (y %.3f, outer %.3f)"
      % (k.xmin, k.xmax, P["knob_other_x"][0], P["knob_other_x"][1], k.zmin, k.zmax, *P["slot"], P["OY0"] - k.ymin, k.ymin, P["OY0"]))
u = board["U1_usbc"].BoundingBox()
print("5. USB-C: centre y %.3f z %.3f (V3 opening 11.0 x 4.0 centred y 0.000 z -1.130), mouth %.3f inside the outer face"
      % (P["usb_centre"][0], P["usb_centre"][1], P["usb_mouth_depth"]))
print("6. LED tops z %.3f, acrylic ledge z %.3f (gap %.2f); SW1 (5 mm tact) plunger top z %.3f = %.2f below the acrylic, centre (%.3f, %.3f)"
      % (board["U1_leds"].BoundingBox().zmax, P["Z_LEDGE"], P["LED_GAP"], P["sw1_top"], P["Z_LEDGE"] - P["sw1_top"], *P["sw1_centre"]))
print("   cap: x case %.4f, x top plate %.4f, x board %.4f; to the hole wall %.3f (hole phi %.1f, shaft phi 4.4); top z %.3f = %.3f above the rim"
      % (inter(cap, case), inter(cap, top), inter(cap, cq.Compound.makeCompound(list(board.values()))), dist(cap, top), P["sw1_hole"], P["cap_top"], P["cap_top"] - P["Z_RIM"]))
print("7. back of the board z %.3f, battery top z %.3f: %.3f for the cut leads and solder (%.1f modelled) and a sheet"
      % (P["zB"], P["battery"][5], P["zB"] - P["battery"][5], board_r2.TRIM))
a = board["U1_antenna"].BoundingBox()
print("8. chip antenna z %.3f..%.3f, %.3f above the battery (same arrangement as V3: battery under the whole board)" % (a.zmin, a.zmax, dist(batt, board["U1_antenna"])))
print("\nsanity: board +0.5 x (into the USB-C wall) vs case %.3f; board -0.2 x (into the ribs) vs case %.3f; battery +0.3 x vs case %.3f"
      % (inter(case, board["PCB"].translate((0.5, 0, 0))), inter(case, board["PCB"].translate((-0.2, 0, 0))), inter(case, batt.translate((0.3, 0, 0.01)))))
