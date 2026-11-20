# 3D check of the insertion path found by insert_check.py: every pose on the path, board parts vs the r2 case
import cadquery as cq, json, os
import board_r2
HERE = os.path.dirname(os.path.abspath(__file__))
P = json.load(open(HERE + "/params.json")); W = json.load(open(HERE + "/insert_path.json"))
case = cq.importers.importStep(HERE + "/turnie_v1.5_r2_caseV3.step").val()
B = board_r2.load(P["XOFF"], P["YOFF"], P["ZOFF"])
parts = ["PCB", "HDR_plastic", "HDR_pins", "U1_pcb", "U1_usbc", "SW1", "SW2", "SW2_knob", "D3"]
Ky, Kz = W["pivot"]
worst = {}
for th, dy, dz in W["poses"]:
    for n in parts:
        s = B[n].rotate((0, Ky, Kz), (1, Ky, Kz), th).translate((0, dy, dz))
        v = case.intersect(s).Volume()
        if v > 1e-4: worst.setdefault(n, []).append((th, dy, dz, round(v, 4)))
print("poses checked:", len(W["poses"]))
print("collisions:", "none" if not worst else "")
for n, l in worst.items(): print("  ", n, len(l), "poses, e.g.", l[:4])
