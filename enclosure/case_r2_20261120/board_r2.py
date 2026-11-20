# r2 board (turnie_v1.5 r2 shield + Waveshare ESP32-S3-Matrix) placed in the case V3 frame.
# Source: kicad-cli STEP of turnie_v1.5.kicad_pcb (KiCad x, STEP y = -KiCad y, z up = LED side).
# Case V3 frame (Fusion "turnie" document, mm): +z = front (acrylic), the open side of the tub.
# Placement = 180 deg turn about z, then (XOFF, YOFF, ZOFF):  X = XOFF - x_k,  Y = y_k + YOFF,  Z = z_step + ZOFF
import cadquery as cq
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

SRC = "/Users/kaishiraishi/Desktop/turnie/v1.5/turnie_v1.5_case_r2_20261120/turnie_v1.5r2_board_kicad.step"   # kicad-cli export of turnie_v1.5.kicad_pcb (r2, last edited 11/16)
S_F, S_MOD_BOT, S_MOD_TOP, S_LED_TOP = 1.595, 4.137, 5.737, 6.537     # STEP z (see the 11/19 enclosure README)
S_B = S_F - 1.6
TRIM = 0.8            # leads behind the B face after trimming (header tails 1.4 and SW1 leads 1.9 are cut to this)

def bbx(s):
    b = Bnd_Box(); BRepBndLib.AddOptimal_s(s.wrapped, b, False, False); return b.Get()

def load(xoff, yoff, zoff, trimmed=True):
    def place(s):
        return s.rotate((0, 0, 0), (0, 0, 1), 180).translate((xoff, yoff, zoff))
    doc = TDocStd_Document(TCollection_ExtendedString("x"))
    r = STEPCAFControl_Reader(); r.SetNameMode(True); r.ReadFile(SRC); r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()); ls = TDF_LabelSequence(); st.GetFreeShapes(ls)
    comps = TDF_LabelSequence(); st.GetComponents_s(ls.Value(1), comps)
    def name(l):
        n = TDataStd_Name(); return n.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(), n) else "?"
    keep = cq.Workplane("XY").box(400, 400, 40, centered=(True, True, False)).translate((0, 0, S_B - TRIM)).val()
    out = {}
    for i in range(1, comps.Length() + 1):
        c = comps.Value(i); nm = name(c); s = cq.Shape.cast(st.GetShape_s(c))
        if nm.startswith("=>"): continue                       # STEP core is 1.51 only; a 1.6 slab is built below
        if nm == "U1":
            g = {"U1_pcb": [], "U1_leds": [], "U1_usbc": [], "U1_antenna": [], "U1_other": []}
            for so in s.Solids():
                x0, y0, z0, x1, y1, z1 = bbx(so)
                if so.Volume() > 500: g["U1_pcb"].append(so)
                elif z0 > 5.7: g["U1_leds"].append(so)
                elif x0 < 130.5: g["U1_usbc"].append(so)
                elif x0 > 153.5 and x1 < 156.5 and so.Volume() > 5: g["U1_antenna"].append(so)
                else: g["U1_other"].append(so)
            for k, v in g.items(): out[k] = place(cq.Compound.makeCompound(v))
            continue
        if nm == "SW2":
            body, knob = [], []
            for so in s.Solids():
                (knob if bbx(so)[1] > -81.3 else body).append(so)       # knob = the solid past the board edge
            out["SW2"] = place(cq.Compound.makeCompound(body)); out["SW2_knob"] = place(cq.Compound.makeCompound(knob))
            out["SW2_knob_other_pos"] = place(cq.Compound.makeCompound([k.translate((-2.0, 0, 0)) for k in knob]))
            continue
        if trimmed: s = s.intersect(keep)
        out[nm] = place(s)
    # 1.6 slab with R1.5 corners and the USB-C notch (KiCad x 130.325-138.5, y 87.8-100.5)
    pcb = (cq.Workplane("XY").rect(166.075 - 130.325, 107.35 - 80.95).extrude(1.6).edges("|Z").fillet(1.5)
           .translate(((130.325 + 166.075) / 2, -(80.95 + 107.35) / 2, S_B))
           .cut(cq.Workplane("XY").box(138.5 - 130.325 + 1, 100.5 - 87.8, 3, centered=False).translate((130.325 - 1, -100.5, S_B - 0.5))))
    out["PCB"] = place(pcb.val())
    # 2.54 headers (PH-1x40SG): plastic between the boards, tails cut to TRIM, top ends cut at the LED height
    hp, ht = [], []
    for yk in (82.732, 105.592):
        hp.append(cq.Workplane("XY").box(25.4, 2.5, S_MOD_BOT - S_F, centered=False).translate((132.136 - 1.27, -yk - 1.25, S_F)).val())
        for i in range(10):
            ht.append(cq.Workplane("XY").box(0.64, 0.64, S_LED_TOP - (S_B - TRIM), centered=False)
                      .translate((132.136 + 2.54 * i - 0.32, -yk - 0.32, S_B - TRIM)).val())
    out["HDR_plastic"] = place(cq.Compound.makeCompound(hp))
    out["HDR_pins"] = place(cq.Compound.makeCompound(ht))
    return out
