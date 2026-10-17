"""Engine meshes from the engines' physical data:  python3 tools/cargen/engine_mesh.py [engine_id ...]

Every row of data/engines/engine_catalog.json gets a generated mesh and an envelope, built from what its engine
file says about itself: architecture, cylinder count, bore, stroke, rod length, bank layout and angle, valves per
cylinder, intake plenum and runners, exhaust primaries, forced induction (turbo count and size). A torque-map engine
carries no geometry; it gets the same generator fed from a class rule (CLASS_RULES below: displacement and cylinder
count of the engine the file describes) and its envelope says so.

Writes data/models/engines/<id>/<id>.glb, <id>.rig.json and <id>.envelope.json (rg.engine_envelope/1), and
data/models/engines/index.json. Deterministic, stdlib only (plus physics_sim's assetgen, read-only).

Engine frame (ISO like the cars): x along the crank toward the timing end, y left, z up along an inline engine's
cylinder axis; origin on the crank axis at the flywheel face of the block (where the bellhousing bolts on). Every
proportion below is a rule of thumb [AS] for production engines of the class, applied to the file's own numbers; the
mesh is a recognisable stand-in sized right, not a CAD model.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cargen  # noqa: E402  (puts assetgen on sys.path)
from assetgen import mesh as M  # noqa: E402
from assetgen.scene import Model  # noqa: E402
from assetgen.vec import compose, translate, rot_axis, xform_point  # noqa: E402
from cargen.common import tube_path  # noqa: E402

REPO = cargen.REPO
OUT = os.path.join(REPO, "data", "models", "engines")
CATALOG = os.path.join(REPO, "data", "engines", "engine_catalog.json")

# Torque-map engines (no geometry in the file): the engine the file describes, from its own source text and the
# catalogue name. Bore and stroke are [AS] for the class (square or published-class values), never fitted.
CLASS_RULES = {
    "i4_2l_na": dict(layout="inline", cylinders=4, bore_mm=86.0, stroke_mm=86.0, rod_mm=145.0, valves=4,
                     note="2.0 L inline-four (the file's source): square 86 x 86 mm [AS], DOHC four-valve [AS]"),
    "n52b30_3l_na": dict(layout="inline", cylinders=6, bore_mm=85.0, stroke_mm=88.0, rod_mm=144.35, valves=4,
                         note="BMW N52B30 3.0 L inline-six (the file's source): 85 x 88 mm, rod 144.35 mm (published "
                              "class values, recalled), DOHC four-valve"),
    "r26b_4rotor": dict(layout="rotary", rotors=4, rotor_width_mm=80.0, generating_radius_mm=105.0,
                        eccentricity_mm=15.0,
                        note="R26B four-rotor Wankel (the file's source): 13B-family rotor housings, generating "
                             "radius 105 mm, eccentricity 15 mm, rotor width 80 mm [AS, recalled 13B values]"),
}

# What a file cannot say about looks: block casting and cam-cover colour [AS].
LOOKS = {
    "pico_i3_1l0_na": dict(block="aluminium", cover=(0.08, 0.08, 0.09)),
    "ventura_v8_5l0_na": dict(block="cast_iron", cover=(0.12, 0.25, 0.55)),
    "runabout_i4_2l9_sv": dict(block="cast_iron", cover=(0.10, 0.10, 0.10)),
    "hyper_v8_one1_audio": dict(block="aluminium", cover=(0.55, 0.06, 0.05)),
    "demo_i4_pipes": dict(block="aluminium", cover=(0.15, 0.15, 0.16)),
    "toyota_a25a_fks_gen": dict(block="aluminium", cover=(0.10, 0.10, 0.11)),
    "i4_2l_na": dict(block="aluminium", cover=(0.12, 0.12, 0.13)),
    "n52b30_3l_na": dict(block="aluminium", cover=(0.18, 0.18, 0.2)),
    "r26b_4rotor": dict(block="aluminium", cover=(0.5, 0.5, 0.52)),
}


def load_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def write_json(path, obj):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2)
        f.write("\n")


# --------------------------------------------------------------------------------------------- description
def describe(row):
    """The geometry the generator needs, from the engine file (or the class rule)."""
    path = os.path.join(REPO, row["file"])
    e = load_json(path)
    eid = row["id"]
    if e.get("kind") == "simulated":
        c = e.get("cycle", {})
        n = int(e["cylinders"])
        banks = c.get("banks")
        layout = "inline"
        if banks and len(banks) == 2:
            layout = "v"
        elif "side-valve" in e.get("source", "") or "side valve" in e.get("source", ""):
            layout = "inline"
        sv = "side-valve" in e.get("source", "") or "flathead" in e.get("source", "")
        intake = c.get("intake", {})
        exhaust = c.get("exhaust", {})
        runners = intake.get("runners", [{}])[0]
        prim = exhaust.get("primaries", [{}])[0]
        turbo_count, turbo_flow = 0, 0.0
        fi = e.get("default_forced_induction")
        if fi:
            fpath = os.path.normpath(os.path.join(os.path.dirname(path), fi))
            cfg = load_json(fpath)
            turbo_count = int(cfg.get("count", 1))
            tpath = os.path.normpath(os.path.join(os.path.dirname(fpath), cfg["turbo"]))
            turbo_flow = float(load_json(tpath).get("max_mass_flow_kg_s", 0.3))
        valves = int(c.get("intake_valves", 1)) + int(c.get("exhaust_valves", 1))
        return dict(id=eid, layout=layout, cylinders=n, bore=e["bore_mm"] / 1000.0, stroke=e["stroke_mm"] / 1000.0,
                    rod=c.get("rod_mm", 1.6 * e["stroke_mm"]) / 1000.0,
                    bank_angle=float(c.get("bank_angle_deg", 90.0)), valves=valves, side_valve=sv,
                    plenum_m3=float(intake.get("plenum_m3", e.get("intake_manifold_volume_l", 2.0) / 1000.0)),
                    runner_len=float(runners.get("length_m", 0.3)), runner_d=float(runners.get("d_in_m", 0.04)),
                    primary_d=float(prim.get("d_m", 0.04)), throttle_d=float(intake.get("throttle_bore_m", 0.06)),
                    turbos=turbo_count, turbo_flow=turbo_flow, class_rule=None,
                    source="the engine file's own geometry (%s)" % row["file"])
    rule = CLASS_RULES.get(eid)
    if rule is None:
        raise SystemExit(f"{eid}: a torque-map engine with no CLASS_RULES entry")
    if rule["layout"] == "rotary":
        return dict(id=eid, layout="rotary", rotors=rule["rotors"], rotor_w=rule["rotor_width_mm"] / 1000.0,
                    gen_r=rule["generating_radius_mm"] / 1000.0, ecc=rule["eccentricity_mm"] / 1000.0, turbos=0,
                    turbo_flow=0.0, class_rule=rule["note"],
                    source="class rule (the file is a torque map with no geometry): " + rule["note"])
    disp = rule["cylinders"] * math.pi / 4 * rule["bore_mm"] ** 2 * rule["stroke_mm"] / 1e9
    return dict(id=eid, layout=rule["layout"], cylinders=rule["cylinders"], bore=rule["bore_mm"] / 1000.0,
                stroke=rule["stroke_mm"] / 1000.0, rod=rule["rod_mm"] / 1000.0, bank_angle=90.0,
                valves=rule["valves"], side_valve=False, plenum_m3=disp * 1.0, runner_len=0.3, runner_d=0.042,
                primary_d=0.040, throttle_d=0.07, turbos=0, turbo_flow=0.0, class_rule=rule["note"],
                source="class rule (the file is a torque map with no geometry): " + rule["note"])


# ------------------------------------------------------------------------------------------------ geometry
def materials(m, looks):
    block = (0.62, 0.63, 0.65) if looks["block"] == "aluminium" else (0.27, 0.27, 0.28)
    m.mat("block", color=block, metallic=0.6 if looks["block"] == "aluminium" else 0.3, roughness=0.6)
    m.mat("head", color=(0.66, 0.67, 0.69), metallic=0.6, roughness=0.5)
    m.mat("cover", color=looks["cover"], metallic=0.3, roughness=0.4)
    m.mat("sump", color=(0.18, 0.18, 0.19), metallic=0.4, roughness=0.6)
    m.mat("intake", color=(0.55, 0.56, 0.58), metallic=0.6, roughness=0.45)
    m.mat("exhaust", color=(0.33, 0.27, 0.22), metallic=0.7, roughness=0.65)
    m.mat("rubber", color=(0.05, 0.05, 0.05), roughness=0.9)
    m.mat("steel", color=(0.5, 0.5, 0.52), metallic=0.9, roughness=0.35)
    m.mat("turbo", color=(0.45, 0.43, 0.40), metallic=0.8, roughness=0.5)
    m.mat("brass", color=(0.78, 0.60, 0.25), metallic=1.0, roughness=0.3)
    m.mat("coil", color=(0.04, 0.04, 0.045), roughness=0.6)
    m.mat("filter", color=(0.10, 0.22, 0.55), metallic=0.3, roughness=0.5)


def dims(d):
    """Derived piston-engine sizes [AS rules of thumb] - every one is reported in the envelope."""
    B, S, L = d["bore"], d["stroke"], d["rod"]
    per_bank = d["cylinders"] if d["layout"] == "inline" else d["cylinders"] // 2
    pitch = 1.12 * B + 0.010                        # bore spacing: bore + water jacket / siamesed wall
    deck = S / 2 + L + 0.33 * B                     # crank axis to deck: half stroke + rod + piston compression height
    below = S / 2 + 0.040                           # counterweight swing below the crank axis
    sump = 0.09 + 0.03 * per_bank / 4               # oil pan depth
    if d["side_valve"]:
        head_h, cover_h, head_w = 0.035, 0.0, B + 0.07
    elif d["valves"] <= 2:
        head_h, cover_h, head_w = 0.10, 0.06, B + 0.08   # pushrod / single cam two-valve head and rocker cover
    else:
        head_h, cover_h, head_w = 0.13, 0.05, B + 0.12   # DOHC four-valve head and cam cover
    front = 0.06 if d["valves"] > 2 else 0.04       # timing case (chain/belt for overhead cams)
    length = per_bank * pitch + 0.03
    if d["layout"] == "v":
        length += 0.022                             # one bank sits one rod width behind the other
    return dict(per_bank=per_bank, pitch=pitch, deck=deck, below=below, sump=sump, head_h=head_h, cover_h=cover_h,
                head_w=head_w, front=front, length=length, block_w=B + 0.06, case_w=S + 0.13)


def bank_xf(angle_rad):
    return rot_axis((1, 0, 0), angle_rad)


def build_piston(m, d):
    mb = m.mb
    k = dims(d)
    L0, Lb = 0.0, k["length"]
    xc = Lb / 2
    # crankcase and sump (below the crank axis), front timing case, flywheel housing face
    M.rounded_box(mb, (xc, 0, -k["below"] / 2 + 0.03), (Lb, k["case_w"], k["below"] + 0.06), 0.015, "block")
    M.rounded_box(mb, (xc, 0, -k["below"] - k["sump"] / 2 + 0.005), (Lb - 0.03, k["case_w"] - 0.02, k["sump"]),
                  0.02, "sump")
    flywheel_r = max(0.14, d["stroke"] / 2 + 0.10)
    M.cylinder(mb, (-0.035, 0, 0), (-0.002, 0, 0), flywheel_r, "steel", count=32)
    M.cylinder(mb, (Lb, 0, 0), (Lb + k["front"], 0, 0), 0.09, "block", count=24)
    M.cylinder(mb, (Lb + k["front"], 0, 0), (Lb + k["front"] + 0.03, 0, 0), 0.075, "rubber", count=24)
    banks = [(0.0, 0.0)] if d["layout"] == "inline" else [(+math.radians(d["bank_angle"]) / 2, 0.0),
                                                          (-math.radians(d["bank_angle"]) / 2, 0.022)]
    ports = []    # (bank index, intake port point, exhaust port point, outward side sign) in engine frame
    for bi, (ang, xoff) in enumerate(banks):
        xf = bank_xf(ang)
        x0 = xoff + 0.015 + k["pitch"] / 2
        bl = k["per_bank"] * k["pitch"]
        bx = xoff + 0.015 + bl / 2
        # cylinder block (crank axis up to the deck), head, cam / rocker cover
        M.rounded_box(mb, (bx, 0, k["deck"] / 2), (bl + 0.02, k["block_w"], k["deck"]), 0.012, "block", xf=xf)
        M.rounded_box(mb, (bx, 0, k["deck"] + k["head_h"] / 2), (bl + 0.03, k["head_w"], k["head_h"]), 0.012,
                      "head", xf=xf)
        if k["cover_h"] > 0:
            cover_parts(mb, d, k, bx, bl, x0, xf)
        else:  # side-valve: the flat head carries the spark plugs; valves and ports live in the block side
            for i in range(k["per_bank"]):
                x = x0 + i * k["pitch"]
                M.cylinder(mb, xform_point(xf, (x, 0, k["deck"] + k["head_h"])),
                           xform_point(xf, (x, 0, k["deck"] + k["head_h"] + 0.03)), 0.008, "brass", count=10)
        block_ribs(mb, k, bx, bl, x0, xf, d)
        for i in range(k["per_bank"]):
            x = x0 + i * k["pitch"]
            if d["layout"] == "v":
                out = -1.0 if ang > 0 else 1.0            # rot about +x by +ang tilts the bank toward -y
                inner = -out
                pz = k["deck"] + k["head_h"] * 0.5
                ip = xform_point(xf, (x, inner * k["head_w"] / 2, pz))
                ep = xform_point(xf, (x, out * k["head_w"] / 2, pz))
                ports.append((bi, ip, ep, out))
            elif d["side_valve"]:
                pz = k["deck"] - 0.05
                ports.append((bi, (x - 0.02, k["block_w"] / 2, pz), (x + 0.02, k["block_w"] / 2, pz - 0.035), 1.0))
            else:
                pz = k["deck"] + k["head_h"] * 0.5
                ports.append((bi, (x, k["head_w"] / 2, pz), (x, -k["head_w"] / 2, pz), -1.0))
    intake(m, d, k, ports, xc)
    exhaust(m, d, k, ports)
    port_flanges(mb, d, k, ports)
    # accessories on the front: water pump and alternator
    M.cylinder(mb, (Lb + 0.005, 0, k["deck"] * 0.55), (Lb + k["front"] + 0.02, 0, k["deck"] * 0.55), 0.05,
               "block", count=16)
    alt_y = -(k["case_w"] / 2 + 0.05) if d["layout"] != "inline" or d["side_valve"] else (k["head_w"] / 2 + 0.03)
    M.cylinder(mb, (Lb - 0.10, alt_y, k["deck"] * 0.35), (Lb + k["front"] - 0.005, alt_y, k["deck"] * 0.35), 0.057,
               "steel", count=16)
    for j in range(3):                                         # alternator cooling slots (dark bands)
        xx = Lb - 0.08 + 0.028 * j
        M.cylinder(mb, (xx, alt_y, k["deck"] * 0.35), (xx + 0.008, alt_y, k["deck"] * 0.35), 0.06, "rubber",
                   count=16)
    if d["valves"] > 2 and d["layout"] == "inline" and not d["side_valve"]:
        # cam drive cover over the front of block and head (chain / belt case), up to below the cam cover
        zt = k["deck"] + k["head_h"] + k["cover_h"] * 0.5
        M.rounded_box(mb, (Lb + k["front"] * 0.4, 0, (zt + k["deck"] * 0.15) / 2),
                      (k["front"] * 0.8, k["head_w"] * 0.9, zt - k["deck"] * 0.15), 0.02, "cover")
    xp = Lb + k["front"]
    belt_drive(mb, xp, [(0.0, 0.0, 0.075), (0.0, k["deck"] * 0.55, 0.052), (alt_y, k["deck"] * 0.35, 0.034)])
    block_extras(mb, d, k)
    return k


def cover_parts(mb, d, k, bx, bl, x0, xf):
    """Cam / rocker cover. Overhead cams: a lower cover with two cam-tunnel ridges and a coil pack per cylinder
    between them (all up to the old cover top). Two-valve: a rocker cover with a flange, hold-down bolts and
    stiffening ribs. Nothing rises above deck + head_h + cover_h (the envelope stays the same)."""
    top = k["deck"] + k["head_h"] + k["cover_h"]
    zb = k["deck"] + k["head_h"]
    w = k["head_w"] - 0.02
    if d["valves"] > 2:
        hb = k["cover_h"] * 0.55
        M.rounded_box(mb, (bx, 0, zb + hb / 2), (bl + 0.01, w, hb), 0.012, "cover", xf=xf)
        for sy in (-1, 1):
            M.rounded_box(mb, (bx, sy * w * 0.27, zb + k["cover_h"] / 2), (bl, w * 0.36, k["cover_h"]), 0.018, "cover",
                          xf=xf)
        for i in range(k["per_bank"]):
            x = x0 + i * k["pitch"]
            M.rounded_box(mb, (x, 0, zb + hb + (top - zb - hb) / 2 - 0.001), (0.05, w * 0.16, top - zb - hb - 0.002),
                          0.006, "coil", xf=xf)
        M.cylinder(mb, xform_point(xf, (bx + bl * 0.3, w * 0.27, top - 0.004)),
                   xform_point(xf, (bx + bl * 0.3, w * 0.27, top - 0.0005)), 0.022, "rubber", count=16)  # oil cap
        return
    # two-valve rocker cover: flange, body with ribs, bolts along the flange
    M.rounded_box(mb, (bx, 0, zb + 0.006), (bl + 0.012, w + 0.008, 0.012), 0.004, "cover", xf=xf)
    M.rounded_box(mb, (bx, 0, zb + k["cover_h"] / 2), (bl + 0.0, w - 0.012, k["cover_h"]), 0.022, "cover", xf=xf)
    for f in (0.3, 0.55, 0.8):
        zz = zb + k["cover_h"] * f
        M.rounded_box(mb, (bx, 0, zz), (bl - 0.04, w - 0.002, 0.008), 0.003, "cover", xf=xf)
    nb = max(2, k["per_bank"])
    for i in range(nb + 1):
        x = bx - bl / 2 + 0.01 + (bl - 0.02) * i / nb
        for sy in (-1, 1):
            M.cylinder(mb, xform_point(xf, (x, sy * (w / 2 + 0.0015), zb + 0.012)),
                       xform_point(xf, (x, sy * (w / 2 + 0.0015), zb + 0.022)), 0.005, "steel", count=8)
    M.cylinder(mb, xform_point(xf, (bx + bl * 0.3, 0, top - 0.004)), xform_point(xf, (bx + bl * 0.3, 0, top)), 0.024,
               "steel", count=16)                                                                     # oil filler


def block_ribs(mb, k, bx, bl, x0, xf, d):
    """Cast stiffening ribs between the cylinders on both block sides, and head bolts for a side-valve head."""
    for i in range(k["per_bank"] + 1):
        x = x0 - k["pitch"] / 2 + i * k["pitch"]
        for sy in (-1, 1):
            M.rounded_box(mb, (x, sy * (k["block_w"] / 2 + 0.004), k["deck"] * 0.55), (0.014, 0.012, k["deck"] * 0.6),
                          0.004, "block", xf=xf)
    if d["side_valve"]:
        for i in range(k["per_bank"] * 2 + 1):
            x = bx - bl / 2 + 0.012 + (bl - 0.024) * i / (k["per_bank"] * 2)
            for sy in (-1, 1):
                p = xform_point(xf, (x, sy * (k["head_w"] / 2 - 0.015), k["deck"] + k["head_h"]))
                q = xform_point(xf, (x, sy * (k["head_w"] / 2 - 0.015), k["deck"] + k["head_h"] + 0.012))
                M.cylinder(mb, p, q, 0.008, "steel", count=6)


def port_flanges(mb, d, k, ports):
    """Exhaust port flanges where each primary leaves the head (or block, side-valve)."""
    for _, _, ep, out in ports:
        if d["layout"] == "v":
            continue          # the V's primaries leave the head at an angle; their flange is the tube root
        M.rounded_box(mb, (ep[0], ep[1] + out * 0.004, ep[2]), (0.06, 0.010, 0.05), 0.004, "exhaust")


def belt_drive(mb, xp, pulleys):
    """Front accessory drive: grooved pulleys at x = xp..xp+0.03 and one closed belt around their outside.
    pulleys: [(y, z, r)] (crank damper first)."""
    xb = xp + 0.015
    for (y, z, r) in pulleys[1:]:
        M.cylinder(mb, (xp - 0.004, y, z), (xp + 0.026, y, z), r, "steel", count=20)
        M.cylinder(mb, (xp + 0.026, y, z), (xp + 0.029, y, z), r * 0.45, "steel", count=12)
    # convex order around the centroid, outer tangent points sampled on each pulley
    cy = sum(p[0] for p in pulleys) / len(pulleys)
    cz = sum(p[1] for p in pulleys) / len(pulleys)
    ps = sorted(pulleys, key=lambda p: math.atan2(p[1] - cz, p[0] - cy))
    n = len(ps)
    path = []
    for i, (y, z, r) in enumerate(ps):
        prv, nxt = ps[i - 1], ps[(i + 1) % n]
        a_in = math.atan2(z - prv[1], y - prv[0]) - math.pi / 2       # outward normal of the incoming span
        a_out = math.atan2(nxt[1] - z, nxt[0] - y) - math.pi / 2      # outward normal of the outgoing span
        while a_out < a_in:
            a_out += 2 * math.pi
        steps = max(2, int((a_out - a_in) / 0.25))
        for j in range(steps + 1):
            a = a_in + (a_out - a_in) * j / steps
            path.append((xb, y + (r + 0.003) * math.cos(a), z + (r + 0.003) * math.sin(a)))
    M.tube(mb, path, 0.004, "rubber", count=6, closed=True)


def block_extras(mb, d, k):
    """Oil filter canister, dipstick and two engine mounts."""
    Lb = k["length"]
    ys = k["case_w"] / 2
    if not d["side_valve"]:   # a 1910s side-valve has no full-flow filter
        M.cylinder(mb, (Lb * 0.45, ys - 0.01, -0.02), (Lb * 0.45, ys + 0.07, -0.05), 0.038, "filter", count=18)
        M.cylinder(mb, (Lb * 0.45, ys + 0.07, -0.05), (Lb * 0.45, ys + 0.074, -0.0515), 0.03, "steel", count=14)
    yd = k["block_w"] / 2 + 0.02 if d["layout"] == "inline" else -(k["case_w"] / 2 + 0.01)
    M.tube(mb, [(Lb * 0.62, yd, -0.02), (Lb * 0.62, yd, k["deck"] * 0.8)], 0.004, "steel", count=6)
    M.tube(mb, [(Lb * 0.62, yd, k["deck"] * 0.8), (Lb * 0.62 + 0.018, yd, k["deck"] * 0.8 + 0.012),
                (Lb * 0.62 + 0.036, yd, k["deck"] * 0.8)], 0.005, "filter", count=6)
    for sy in (-1, 1):
        M.rounded_box(mb, (Lb * 0.55, sy * ys, 0.0), (0.07, 0.03, 0.06), 0.008, "rubber")


def intake(m, d, k, ports, xc):
    mb = m.mb
    rr = max(0.012, d["runner_d"] / 2)
    vol = max(d["plenum_m3"], 0.0004)
    pl_len = max(0.18, k["length"] * 0.8)
    if d["layout"] == "v":
        # plenum in the valley above the banks, runners down to the inner sides of the heads
        top = max(p[1][2] for p in ports) + 0.06
        side = math.sqrt(vol / pl_len)
        pw, ph = min(side * 1.4, 0.32), vol / pl_len / min(side * 1.4, 0.32)
        ph = min(max(ph, 0.06), 0.18)
        M.rounded_box(mb, (xc, 0, top + ph / 2), (pl_len, pw, ph), 0.03, "intake")
        for _, ip, _, _ in ports:
            path = [ip, (ip[0], ip[1] * 0.5, top + 0.01), (ip[0], ip[1] * 0.25, top + ph * 0.4)]
            tube_path(mb, path, rr, "intake", count=10)
        M.cylinder(mb, (k["length"] + 0.0, 0, top + ph / 2), (k["length"] + 0.05, 0, top + ph / 2),
                   d["throttle_d"] / 2 + 0.008, "intake", count=16)
        return
    if d["side_valve"]:
        # one cast manifold along the block side, updraft carburettor below its middle
        ys = k["block_w"] / 2 + 0.03
        pts = [p[1] for p in ports]
        z = pts[0][2] - 0.02
        M.tube(mb, [(pts[0][0] - 0.02, ys, z), (pts[-1][0] + 0.02, ys, z)], 0.022, "intake", count=12)
        mid = (pts[0][0] + pts[-1][0]) / 2
        M.cylinder(mb, (mid, ys + 0.01, z - 0.02), (mid, ys + 0.01, z - 0.13), 0.03, "brass", count=14)
        for p in pts:
            M.cylinder(mb, (p[0], k["block_w"] / 2 - 0.005, p[2]), (p[0], ys, z), 0.015, "intake", count=10)
        return
    # inline: runners curve out from the +y head side and down to a plenum along the block
    reach = min(0.45 * d["runner_len"], 0.20)
    side = math.sqrt(vol / pl_len)
    pd = min(max(side, 0.07), 0.16)
    py = k["head_w"] / 2 + reach + pd / 2
    pz = k["deck"] + k["head_h"] * 0.3 - 0.06
    M.rounded_box(mb, (xc, py, pz), (pl_len, pd, pd), 0.03, "intake")
    for _, ip, _, _ in ports:
        path = [ip, (ip[0], ip[1] + reach * 0.5, ip[2] + 0.03), (ip[0], py - pd * 0.3, pz + 0.01)]
        tube_path(mb, path, rr, "intake", count=10)
    M.cylinder(mb, (xc + pl_len / 2, py, pz), (xc + pl_len / 2 + 0.05, py, pz), d["throttle_d"] / 2 + 0.008,
               "intake", count=16)


def exhaust(m, d, k, ports):
    mb = m.mb
    r = max(0.012, d["primary_d"] / 2)
    by_bank = {}
    for bi, _, ep, out in ports:
        by_bank.setdefault(bi, []).append((ep, out))
    for bi, lst in sorted(by_bank.items()):
        out = lst[0][1]
        zc = min(p[2] for p, _ in lst) - 0.10
        yc = lst[0][0][1] + out * 0.06
        coll = []
        for p, _ in lst:
            path = [p, (p[0], p[1] + out * 0.04, p[2] - 0.01), (p[0], yc, zc + 0.02)]
            tube_path(mb, path, r, "exhaust", count=10)
            coll.append((p[0], yc, zc))
        x0 = min(c[0] for c in coll) - 0.04
        x1 = max(c[0] for c in coll) + 0.02
        M.tube(mb, [(x1, yc, zc), (x0, yc, zc), (x0 - 0.06, yc, zc - 0.05)], r * 1.35, "exhaust", count=12)
        if d["turbos"]:
            per = max(1, d["turbos"] // max(1, len(by_bank)))
            wr = 0.5 * 0.075 * math.sqrt(max(d["turbo_flow"], 0.1) / 0.3)
            for t in range(per):
                tx = (x0 + x1) / 2 - 0.03 + (t - (per - 1) / 2) * 0.22   # beside the block, under the manifold
                ty = yc + out * (wr * 0.6 + r)
                tz = zc - wr * 1.3 - r
                M.cylinder(mb, (tx - 0.05, ty, tz), (tx, ty, tz), wr * 1.15, "turbo", count=20)       # turbine
                M.cylinder(mb, (tx + 0.005, ty, tz), (tx + 0.06, ty, tz), wr * 1.3, "turbo", count=20)  # compressor
                M.cylinder(mb, (tx, ty, tz), (tx + 0.005, ty, tz), wr * 0.5, "steel", count=12)


def build_rotary(m, d):
    mb = m.mb
    n, w = d["rotors"], d["rotor_w"]
    R, e = d["gen_r"], d["ecc"]
    housing_w = w + 0.012
    plate = 0.025
    x = plate
    hx, hy = R + e + 0.06, R + 0.045        # trochoid housing outline (major / minor half axes) plus water jacket
    for i in range(n):
        sec = M.superellipse(hy, hx, n=3.0, count=40)
        loops = []
        for xx in (x, x + housing_w):
            loops.append([(xx, a, b) for (a, b) in sec])
        M.loft(mb, loops, "block")
        x += housing_w
        M.rounded_box(mb, (x + plate / 2, 0, 0), (plate, 2 * hy + 0.02, 2 * hx + 0.02), 0.01, "cover")
        x += plate
    Lb = x
    M.rounded_box(mb, (plate / 2, 0, 0), (plate, 2 * hy + 0.02, 2 * hx + 0.02), 0.01, "cover")
    M.rounded_box(mb, (Lb / 2, 0, -hx - 0.05), (Lb - 0.02, 2 * hy - 0.04, 0.10), 0.02, "sump")
    M.cylinder(mb, (-0.035, 0, 0), (-0.002, 0, 0), 0.15, "steel", count=32)
    M.cylinder(mb, (Lb, 0, 0), (Lb + 0.05, 0, 0), 0.08, "rubber", count=24)
    # peripheral intake runners on +y up to a plenum, exhaust ports on -y down to a collector
    pz = hx + 0.10
    M.rounded_box(mb, (Lb / 2, hy * 0.6, pz), (Lb - 0.02, 0.14, 0.10), 0.03, "intake")
    xs = [plate + housing_w / 2 + i * (housing_w + plate) for i in range(n)]
    for xx in xs:
        tube_path(mb, [(xx, hy * 0.7, hx * 0.5), (xx, hy * 0.75, hx * 0.9), (xx, hy * 0.6, pz - 0.03)], 0.022,
                  "intake", count=10)
        tube_path(mb, [(xx, -hy, -0.02), (xx, -hy - 0.07, -0.05), (xx, -hy - 0.08, -hx * 0.7)], 0.022, "exhaust",
                  count=10)
    M.tube(mb, [(xs[-1] + 0.02, -hy - 0.08, -hx * 0.7), (xs[0] - 0.06, -hy - 0.08, -hx * 0.7)], 0.035, "exhaust",
           count=12)
    return dict(length=Lb, housing_w=housing_w, plate=plate, housing_half_axes=[hx, hy])


# ---------------------------------------------------------------------------------------------------- build
def bounds(mb):
    lo, hi = [1e9] * 3, [-1e9] * 3
    for prim in mb.prims.values():
        for p in prim.pos:
            for i in range(3):
                lo[i] = min(lo[i], p[i])
                hi[i] = max(hi[i], p[i])
    return lo, hi


def build_one(row):
    d = describe(row)
    eid = d["id"]
    m = Model(eid, row["name"], "Generated engine mesh for %s from its physical data." % row["name"],
              "Crank axis at the flywheel face of the block; x toward the timing end, z up.")
    materials(m, LOOKS.get(eid, dict(block="aluminium", cover=(0.15, 0.15, 0.16))))
    k = build_rotary(m, d) if d["layout"] == "rotary" else build_piston(m, d)
    lo, hi = bounds(m.mb)
    m.socket("socket_flywheel", None, (0.0, 0.0, 0.0), "flywheel face of the block on the crank axis (+x toward "
             "the timing end): the gearbox bolts on here")
    m.extra_rig["engine"] = {"layout": d["layout"], "source": d["source"]}
    folder = os.path.join(OUT, eid)
    stats = m.write(folder)
    r6 = lambda v: [round(c, 4) for c in v]  # noqa: E731
    derived = {kk: (round(v, 4) if isinstance(v, float) else v) for kk, v in k.items()}
    env = {
        "format": "rg.engine_envelope/1",
        "engine": eid,
        "file": row["file"],
        "layout": d["layout"],
        "frame": "x along the crank toward the timing end, y left, z up (an inline engine's cylinder axis); origin on "
                 "the crank axis at the flywheel face of the block",
        "bounds_m": {"min": r6(lo), "max": r6(hi)},
        "size_mm": [round((hi[i] - lo[i]) * 1000.0) for i in range(3)],
        "mounts": {"flywheel_face": [0.0, 0.0, 0.0], "crank_axis": [1.0, 0.0, 0.0]},
        "class_rule": d["class_rule"] is not None,
        "derived": derived,
        "source": "tools/cargen/engine_mesh.py from %s. Proportions are [AS] rules of thumb (bore pitch 1.12 x bore + "
                  "10 mm, deck = stroke / 2 + rod + 0.33 x bore, head and cover heights by valve count, manifolds "
                  "drawn compact); the bounding box is the generated mesh's own." % d["source"],
    }
    write_json(os.path.join(folder, eid + ".envelope.json"), env)
    return stats, env


def main(argv):
    rows = load_json(CATALOG)["engines"]
    ids = argv or [r["id"] for r in rows]
    known = {r["id"]: r for r in rows}
    for eid in ids:
        if eid not in known:
            sys.exit(f"unknown engine {eid}; catalogue: {sorted(known)}")
        stats, env = build_one(known[eid])
        print(f"{eid:22s} {stats['triangles']:6d} tris  size {env['size_mm']} mm  {env['layout']}"
              f"{'  (class rule)' if env['class_rule'] else ''}")
    index = [{"id": r["id"], "glb": f"{r['id']}/{r['id']}.glb", "envelope": f"{r['id']}/{r['id']}.envelope.json"}
             for r in rows if os.path.exists(os.path.join(OUT, r["id"], r["id"] + ".glb"))]
    write_json(os.path.join(OUT, "index.json"), index)


if __name__ == "__main__":
    main(sys.argv[1:])
