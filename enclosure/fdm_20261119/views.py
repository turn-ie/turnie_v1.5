# confirmation pictures: cross-sections and outside views (SVG -> PNG). battery = reference body (not printed)
# frame: x = KiCad x, y = KiCad y, z = depth from the outer front face (LED side). run after check.py
import cadquery as cq, os, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__))
imp = lambda n: cq.importers.importStep(f"{HERE}/{n}.step")
case, lid, tray, batt = imp("turnie_v1.5_01_case"), imp("turnie_v1.5_02_back_lid"), imp("turnie_v1.5_03_battery_tray"), imp("ref_battery_36x24.5x5.6")
board = imp("board_positioned_ref")
big = lambda x0, x1, y0, y1: cq.Workplane("XY").box(x1-x0, y1-y0, 80, centered=False).translate((x0, y0, -30))
ONLY = sys.argv[1:]                        # optional: render only these view names
def out(wp, name, d, w=1400, h=1000):
    print("view", name, flush=True)
    cq.exporters.export(wp, f"{HERE}/{name}.svg", opt={"projectionDir": d, "showHidden": False, "width": w, "height": h, "strokeWidth": 0.08})
    subprocess.run(["qlmanage", "-t", "-s", "1600", "-o", HERE, f"{HERE}/{name}.svg"], capture_output=True)   # QuickLook (ImageMagick's MSVG renders these blank)
    subprocess.run(["magick", f"{HERE}/{name}.svg.png", "-background", "white", "-flatten", "-fuzz", "2%", "-trim", "+repage", "-bordercolor", "white", "-border", "40", f"{HERE}/{name}.png"], check=False)
    os.remove(f"{HERE}/{name}.svg.png")
    os.remove(f"{HERE}/{name}.svg")
allp = case.union(lid).union(tray)
bsol = [(s, s.BoundingBox()) for s in board.val().Solids()]
def sec(keep):   # keep = box to remove (the half toward the viewer)
    kb = keep.val().BoundingBox()
    inside = lambda b: kb.xmin <= b.xmin and b.xmax <= kb.xmax and kb.ymin <= b.ymin and b.ymax <= kb.ymax
    apart = lambda b: b.xmax < kb.xmin or b.xmin > kb.xmax or b.ymax < kb.ymin or b.ymin > kb.ymax
    sols = []
    for s, b in bsol:                      # board solids: drop the removed ones, cut only those crossing the plane
        if inside(b): continue
        sols.append(s if apart(b) else s.cut(keep.val()))
    return allp.cut(keep).add(cq.Workplane(obj=cq.Compound.makeCompound(sols))).add(batt.cut(keep))
V = [("view_section_A_SW1", lambda: sec(big(100, 200, 60, 102.6)), (0, -1, 0)),             # plane y = SW1 centre, seen from the top (SW2) side
     ("view_section_B_SW2_knob", lambda: sec(big(162.3, 200, 60, 130)), (1, 0, 0)),         # plane x = SW2 knob, seen from the right
     ("view_section_C_USB_antenna_battery", lambda: sec(big(100, 200, 60, 94.15)), (0, -1, 0)),  # plane y = USB-C / antenna centre
     ("view_section_D_wire_room_J4", lambda: sec(big(100, 200, 60, 86.3)), (0, -1, 0)),     # plane y = 86.3 (next to the J4 row, wire room on the left)
     ("view_outside_iso_front", lambda: case.union(lid), (-1, -1.1, -1.6)),                 # front (LED) face, top (SW2) and left (USB-C)
     ("view_outside_iso_back", lambda: case.union(lid), (1, 1.1, 1.6)),                     # lid side
     ("view_case_inside", lambda: case, (0.4, 0.6, 1.5)),
     ("view_lid_underside", lambda: lid, (0.5, 0.7, -1.5)),
     ("view_tray_battery", lambda: tray.add(batt), (0.5, 0.7, -1.5))]                       # tray legs (board side) + battery
for n, f, dvec in V:
    if not ONLY or n in ONLY: out(f(), n, dvec)
