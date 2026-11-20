# turnie v1.5 (board v1.5 r2 + Waveshare ESP32-S3-Matrix) FDM enclosure - parametric cadquery script
# run: ~/.venvs/cadquery/bin/python make_enclosure.py   (then check.py, views.py)
# frame: x = KiCad x, y = KiCad y (down in the KiCad view), z = 0 at the outer front face (LED side),
#        +z toward the back lid. Seen from the front this is the KiCad F view. units mm.
# The board STEP (kicad-cli, z up = LED side) maps into this frame by a 180 deg turn about x:
#        (x, y_step, z_step) -> (x, -y_step, Z0 - z_step)
import cadquery as cq, math, json, os
HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- measured from turnie_v1.5_board_kicad.step (STEP z, see README) ----------------
S_F       = 1.595     # main (shield) board F seating plane (SMD parts start here; SW2 pins end at 1.595 - 1.6)
S_MOD_BOT = 4.137     # module PCB underside  (2.542 above F: model of a direct 2.54 header)
S_MOD_TOP = 5.737     # module PCB LED face
S_LED_TOP = 6.537     # LED tops (0.80 tall)
T_PCB     = 1.6       # stackup thickness

# ---------------- fit rules (v4 / v5 FDM on the Bambu P2S) ----------------
GAP     = 0.61        # board to inner wall per side (v4 FDM value the user approved)
WALL    = 1.2         # side walls
T_FRONT = 0.6         # front wall over the LEDs (diffuser)
LED_GAP = 0.16        # LED top to inner front face
RIB_GAP = 0.15        # board locating ribs
FIT     = 0.15        # lid plate to inner wall
T_LID   = 1.2

# ---------------- board outline (Edge.Cuts) ----------------
BX0, BX1, BY0, BY1, BR = 130.325, 166.075, 80.95, 107.35, 1.5
NOTCH = (130.325, 138.5, 87.8, 100.5)          # USB-C notch on the left edge

# ---------------- z stack ----------------
Z0  = S_LED_TOP + LED_GAP + T_FRONT            # STEP z of the outer front face
zp  = lambda zs: Z0 - zs                       # STEP z -> enclosure z
zLED, zMT, zMB = zp(S_LED_TOP), zp(S_MOD_TOP), zp(S_MOD_BOT)
zF  = zp(S_F)                                  # shield F face
zB  = zF + T_PCB                               # shield B face
BACK_CLR = 2.0        # B face -> tray front: header tails (3.0 - 1.6 = 1.4), SW1 leads (1.85 in the datasheet), J4 wires
T_TRAY   = 1.0
zTf, zT  = zB + BACK_CLR, zB + BACK_CLR + T_TRAY
BATT     = (36.0, 24.5, 5.6)                   # protected 400 mAh LiPo: x (length incl. PCM), y (width), thickness
BATT_MAX = (36.5, 25.0, 5.9)                   # largest cell accepted (assumption, see README)
FOAM     = 0.5                                 # room for foam / swelling above the thickest cell
zL   = zT + BATT_MAX[2] + FOAM                 # lid plate underside
zTop = zL + T_LID                              # rim = lid top (flush)
WIRE_ROOM = 2.0       # battery PCM end (left) -> left inner wall: the leads bend toward the board here (assumption)
BATT_END_CLR = 0.2    # battery far end -> right inner wall

# ---------------- cavity / outer ----------------
IX0  = BX0 - GAP                               # left inner wall (keeps the USB-C mouth close to the outside)
IXB1 = BX1 + GAP                               # right inner face in the board zone
IX1  = IX0 + WIRE_ROOM + BATT[0] + BATT_END_CLR    # right inner wall in the battery zone (sized from the cell)
IY0, IY1 = BY0 - GAP, BY1 + GAP
OX0, OX1, OY0, OY1 = IX0-WALL, IX1+WALL, IY0-WALL, IY1+WALL
Z_STEP_R = 8.0        # the right wall is thick (board zone) up to here, thin above (battery zone)
PX0, PX1 = IX0 + WIRE_ROOM, IX1                # battery pocket x
PY0, PY1 = IY0, IY0 + BATT_MAX[1] + 0.2        # battery pocket y (top aligned; the lid strip fills below)

# ---------------- parts on the board (KiCad coords, measured) ----------------
USB_Y, USB_ZS, USB_W, USB_H = 94.153, 2.417, 8.94, 3.16   # module USB-C mouth centre (STEP z), shell size
zUSB = zp(USB_ZS)
USB_OPEN = (USB_W + 0.9, USB_H + 0.9, 0.9)     # receptacle + 0.45/side, corner R (as v5)
USB_RELIEF = (12.6, 7.0, 2.0, 0.5)             # outside relief for the plug overmold (w, h, R, depth) - assumption
SW1_X, SW1_Y = 162.1, 102.6                    # tact switch centre (pad centre; the KiCad model sits 0.1 lower in y)
SW1_HOLE = 4.1                                 # plunger: model phi 3.5, datasheet tip phi 3.0
SW1_BODY_TOP = zF - 4.1                        # measured: KiCad model top incl. the cover nubs (datasheet 3.5 body + nubs 0.4 max)
SW1_TUBE_END, SW1_TUBE_WALL = SW1_BODY_TOP - 0.3, 0.8
SW1_LEADS = [(159.85, 99.35), (164.35, 99.35), (159.85, 105.85), (164.35, 105.85)]
SW2_KNOB_X = (159.55, 163.05)                  # knob span over both positions (STEP 161.55..163.05 + 2.0 travel)
zK0, zK1 = zp(4.095), zp(2.595)                # knob z
SW2_SLOT = (SW2_KNOB_X[0]-0.3, SW2_KNOB_X[1]+0.3, zK0-0.3)
SW2_LEADS = [(158.8, 85.6), (161.3, 85.6), (163.8, 85.6)]
J4 = [(137.4, 85.6), (139.94, 85.6)]
PINS = [(132.136 + 2.54*i, y) for i in range(10) for y in (82.732, 105.592)]   # U1 header pins
LED_X = (133.164, 153.968)                     # LED field x on the module
MOD_X = (131.057, 156.075)                     # module PCB x

def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1-x0, y1-y0, z1-z0, centered=False).translate((x0, y0, z0))
def rbox(x0, x1, y0, y1, z0, z1, r):
    return cq.Workplane("XY").rect(x1-x0, y1-y0).extrude(z1-z0).edges("|Z").fillet(r).translate(((x0+x1)/2, (y0+y1)/2, z0))
def cyl(x, y, r, z0, z1):
    return cq.Workplane("XY").circle(r).extrude(z1-z0).translate((x, y, z0))
def prism_x(xw, sgn, y0, y1, zc, h):     # 45-degree ridge on a wall normal to x, pointing sgn*x, spanning y0..y1
    return cq.Workplane("XZ").polyline([(xw, zc-h), (xw+sgn*h, zc), (xw, zc+h)]).close().extrude(-(y1-y0)).translate((0, y0, 0))
def prism_y(yw, sgn, x0, x1, zc, h):     # same on a wall normal to y
    return cq.Workplane("YZ").polyline([(yw, zc-h), (yw+sgn*h, zc), (yw, zc+h)]).close().extrude(x1-x0).translate((x0, 0, 0))

# ---------------- case ----------------
outer = rbox(OX0, OX1, OY0, OY1, 0, zTop, 2.2).faces("<Z").chamfer(0.4).faces(">Z").chamfer(0.4)
case = outer.cut(rbox(IX0, IX1, IY0, IY1, T_FRONT, zTop+1, 1.0))
case = case.union(box(IXB1, IX1, IY0, IY1, T_FRONT, Z_STEP_R))          # board zone: right inner face at board + GAP
# module rests on two ledges on its LED face, in the margins between the LED field and the module edge
LEDGE_W = 1.2
LEDGE_L = ((MOD_X[0]+LED_X[0])/2 - LEDGE_W/2, (MOD_X[0]+LED_X[0])/2 + LEDGE_W/2)
LEDGE_R = ((LED_X[1]+MOD_X[1])/2 - LEDGE_W/2, (LED_X[1]+MOD_X[1])/2 + LEDGE_W/2)
LEDGE_Y = (84.6, 103.7)                         # clear of the corner header pins (y 82.732 / 105.592)
for x0, x1 in (LEDGE_L, LEDGE_R):
    case = case.union(box(x0, x1, LEDGE_Y[0], LEDGE_Y[1], T_FRONT-0.01, zMT))
# SW1: plunger passes through the front face; a short tube stiffens the 0.6 face around the hole
r_h = SW1_HOLE/2
case = case.union(cyl(SW1_X, SW1_Y, r_h+SW1_TUBE_WALL, T_FRONT-0.01, SW1_TUBE_END))
case = case.cut(cyl(SW1_X, SW1_Y, r_h, -1, SW1_TUBE_END+1))
case = case.cut(cq.Workplane("XY").circle(r_h).workplane(offset=0.3).circle(r_h+0.3).loft()
                .translate((SW1_X, SW1_Y, SW1_TUBE_END-0.3)))                       # 45 deg lead-in at the tube end
# USB-C (left wall): opening + shallow outside relief for the plug overmold
uw, uh, ur = USB_OPEN
case = case.cut(cq.Workplane("YZ").center(USB_Y, zUSB).rect(uw, uh).extrude(WALL+2).edges("|X").fillet(ur).translate((OX0-1, 0, 0)))
rw, rh, rr, rd = USB_RELIEF
case = case.cut(cq.Workplane("YZ").center(USB_Y, zUSB).rect(rw, rh).extrude(rd+1).edges("|X").fillet(rr).translate((OX0-1, 0, 0)))
# SW2 knob slot (top wall), open to the rim because the board goes in from the back; the lid tongue closes it
case = case.cut(box(SW2_SLOT[0], SW2_SLOT[1], OY0-1, IY0+0.01, SW2_SLOT[2], zTop+1))
# board locating ribs at the shield edge (45 deg lead-in from the back, 45 deg underside for printing)
RIB_H = GAP - RIB_GAP
def rib_x(xw, sgn, yc):
    e = -sgn*0.05                                          # base sunk 0.05 into the wall so the rib fuses with it
    pts = [(xw+e, zF-0.1-RIB_H), (xw, zF-0.1-RIB_H), (xw+sgn*RIB_H, zF-0.1), (xw+sgn*RIB_H, zB+0.1), (xw, zB+0.1+RIB_H), (xw+e, zB+0.1+RIB_H)]
    return cq.Workplane("XZ").polyline(pts).close().extrude(-1.2).translate((0, yc-0.6, 0))
def rib_y(yw, sgn, xc):
    e = -sgn*0.05
    pts = [(yw+e, zF-0.1-RIB_H), (yw, zF-0.1-RIB_H), (yw+sgn*RIB_H, zF-0.1), (yw+sgn*RIB_H, zB+0.1), (yw, zB+0.1+RIB_H), (yw+e, zB+0.1+RIB_H)]
    return cq.Workplane("YZ").polyline(pts).close().extrude(1.2).translate((xc-0.6, 0, 0))
RIBS_L, RIBS_R, RIBS_T, RIBS_B = (83.3, 104.0), (88.5, 103.5), (136.0, 150.0), (136.0, 150.0)
for yc in RIBS_L: case = case.union(rib_x(IX0, 1, yc))
for yc in RIBS_R: case = case.union(rib_x(IXB1, -1, yc))
for xc in RIBS_T: case = case.union(rib_y(IY0, 1, xc))
for xc in RIBS_B: case = case.union(rib_y(IY1, -1, xc))

# ---------------- lid: plate inside the walls + strip along the bottom wall + tongue in the SW2 slot ----------------
zG = zL - 1.2                    # main snap height (on the strip)
zE = zL + 0.35                   # small plate-edge snaps
STRIP_X0 = PX0 + 0.2
MAIN_B = [(133.0, 144.0), (154.0, 165.0)]            # bottom wall, on the strip (x ranges)
EDGE_T = [(132.5, 142.5), (145.5, 155.5)]            # top wall, clear of the SW2 slot
EDGE_R = [(83.0, 91.0), (97.0, 105.0)]               # right wall (y ranges)
EDGE_L = [(86.0, 102.0)]                             # left wall, above the wire room
PRY = (146.0, 152.0)                                 # pry notch in the bottom wall
for x0, x1 in MAIN_B: case = case.cut(prism_y(IY1-0.01, 1, x0-0.2, x1+0.2, zG, 0.41))
for x0, x1 in EDGE_T: case = case.cut(prism_y(IY0+0.01, -1, x0-0.2, x1+0.2, zE, 0.31))
for y0, y1 in EDGE_R: case = case.cut(prism_x(IX1-0.01, 1, y0-0.2, y1+0.2, zE, 0.31))
for y0, y1 in EDGE_L: case = case.cut(prism_x(IX0+0.01, -1, y0-0.2, y1+0.2, zE, 0.31))
case = case.cut(box(PRY[0], PRY[1], IY1-0.01, IY1+0.5, zTop-0.9, zTop+1))

lx0, lx1, ly0, ly1 = IX0+FIT, IX1-FIT, IY0+FIT, IY1-FIT
lid = rbox(lx0, lx1, ly0, ly1, zL, zTop, 1.0-FIT).faces(">Z").chamfer(0.3)
zone = rbox(lx0, lx1, ly0, ly1, zT+0.2, zL+0.01, 1.0-FIT)        # below the plate, holds the tray 0.2 above
strip = zone.intersect(box(STRIP_X0, lx1, PY1, ly1, zT+0.2, zL+0.01))
lid = lid.union(strip)
for x0, x1 in MAIN_B: lid = lid.union(prism_y(ly1, 1, x0, x1, zG, 0.45))
for x0, x1 in EDGE_T: lid = lid.union(prism_y(ly0, -1, x0, x1, zE, 0.35))
for y0, y1 in EDGE_R: lid = lid.union(prism_x(lx1, 1, y0, y1, zE, 0.35))
for y0, y1 in EDGE_L: lid = lid.union(prism_x(lx0, -1, y0, y1, zE, 0.35))
TG = 0.1                                                            # tongue to slot side gap
lid = lid.union(box(SW2_SLOT[0]+TG, SW2_SLOT[1]-TG, OY0, IY0, zK1+0.3, zL+0.01))
lid = lid.union(box(SW2_SLOT[0]+TG, SW2_SLOT[1]-TG, OY0, ly0+0.5, zL, zTop))

# ---------------- battery tray: plate behind the board, legs on free spots of the B face ----------------
TRAY_G = 0.3
tray = rbox(PX0, IX1-TRAY_G, IY0+TRAY_G, IY1-TRAY_G, zTf, zT, 0.8)
for x, y in SW1_LEADS: tray = tray.cut(cyl(x, y, 1.3, zTf-1, zTf+0.5))         # lead tips (1.9 below B) get 0.5 more room
for x, y in SW2_LEADS: tray = tray.cut(cyl(x, y, 1.0, zTf-1, zTf+0.5))
LEG_R = 0.9
WIRE_BOX = (IX0, 141.5, 83.8, 87.4)              # battery leads from the wire room to J4 run here (behind the board)
def clear(x, y):
    d = min(x-BX0, BX1-x, y-BY0, BY1-y) - 0.5
    nx0, nx1, ny0, ny1 = NOTCH
    if x < nx1 + 0.5 and ny0 - 0.5 < y < ny1 + 0.5: d = min(d, -1.0)
    for (px, py), r in [(p, 1.0) for p in PINS] + [(p, 1.2) for p in J4] + [(p, 1.3) for p in SW1_LEADS] + [(p, 1.0) for p in SW2_LEADS]:
        d = min(d, math.hypot(x-px, y-py) - r)
    x0, x1, y0, y1 = WIRE_BOX
    d = min(d, math.hypot(max(x0-x, 0, x-x1), max(y0-y, 0, y-y1)))
    return d
LEGS = [(SW1_X, SW1_Y), (164.0, 92.5), (157.6, 93.0), (147.0, 94.0), (150.0, 88.6), (141.0, 102.6), (150.5, 102.6), (134.6, 102.8)]
legs = []
for x, y in LEGS:
    c = clear(x, y)
    assert c >= LEG_R + 0.3, (x, y, c)
    legs.append((x, y, round(c, 2)))
    tray = tray.union(cyl(x, y, LEG_R, zB, zTf+0.01))

# ---------------- reference battery (not printed) ----------------
bx1 = IX1 - BATT_END_CLR
batt = rbox(bx1-BATT[0], bx1, PY0+0.2, PY0+0.2+BATT[1], zT, zT+BATT[2], 1.0)

# ---------------- export ----------------
parts = {"turnie_v1.5_01_case": case, "turnie_v1.5_02_back_lid": lid, "turnie_v1.5_03_battery_tray": tray}
for n, p in parts.items():
    cq.exporters.export(p, os.path.join(HERE, n+".stl"), tolerance=0.01, angularTolerance=0.1)
    cq.exporters.export(p, os.path.join(HERE, n+".step"))
cq.exporters.export(batt, os.path.join(HERE, "ref_battery_36x24.5x5.6.step"))
asm = cq.Assembly(name="turnie_v1.5_enclosure")
asm.add(case, name="case", color=cq.Color(0.95, 0.95, 0.95, 0.6)); asm.add(lid, name="back_lid", color=cq.Color(0.9, 0.9, 0.9))
asm.add(tray, name="battery_tray", color=cq.Color(0.3, 0.3, 0.3)); asm.add(batt, name="battery_ref_36x24.5x5.6", color=cq.Color(0.7, 0.7, 0.75))
asm.save(os.path.join(HERE, "turnie_v1.5_enclosure_assembly.step"))
r3 = lambda v: round(v, 3)
dims = dict(outer=(r3(OX1-OX0), r3(OY1-OY0), r3(zTop)), outer_xy=(r3(OX0), r3(OX1), r3(OY0), r3(OY1)),
            cavity_board_zone=(r3(IXB1-IX0), r3(IY1-IY0)), cavity_battery_zone=(r3(IX1-IX0), r3(IY1-IY0)),
            Z0_step=r3(Z0),
            z=dict(front_inner=T_FRONT, led_top=r3(zLED), module_face=r3(zMT), module_under=r3(zMB), board_F=r3(zF), board_B=r3(zB),
                   tray_front=r3(zTf), tray_back=r3(zT), lid_under=r3(zL), top=r3(zTop), knob=(r3(zK0), r3(zK1)), usb_centre=r3(zUSB),
                   sw1_body_top=r3(SW1_BODY_TOP), sw1_plunger_top=r3(zF-9.5), snap_main=r3(zG), snap_edge=r3(zE), right_step=Z_STEP_R),
            battery_pocket=dict(x=(r3(PX0), r3(PX1)), y=(r3(PY0), r3(PY1)), z=(r3(zT), r3(zL))),
            battery_pocket_size=(r3(PX1-PX0), r3(PY1-PY0), r3(zL-zT)), wire_room=(r3(IX0), r3(PX0)),
            battery_ref=dict(x=(r3(bx1-BATT[0]), r3(bx1)), y=(r3(PY0+0.2), r3(PY0+0.2+BATT[1])), z=(r3(zT), r3(zT+BATT[2]))),
            ledges=dict(left=[r3(v) for v in LEDGE_L], right=[r3(v) for v in LEDGE_R], y=LEDGE_Y),
            usb_opening=(r3(uw), r3(uh)), usb_mouth_depth_from_outside=r3(130.31-OX0), usb_mouth_depth_from_relief=r3(130.31-OX0-rd),
            sw2_slot=dict(x=(r3(SW2_SLOT[0]), r3(SW2_SLOT[1])), z_from=r3(SW2_SLOT[2]), knob_out_of_wall=r3(OY0-77.95)),
            sw1_plunger_out_of_front=r3(9.5-zF), legs=legs,
            volumes={n: round(p.val().Volume(), 1) for n, p in parts.items()},
            solids={n: len(p.val().Solids()) for n, p in parts.items()})
json.dump(dims, open(os.path.join(HERE, "dims.json"), "w"), indent=1)
print(json.dumps(dims, indent=1))
