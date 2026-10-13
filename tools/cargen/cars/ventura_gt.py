"""Ventura GT: 1970s front-engine fastback grand tourer (generic, no real model).

Design language "long hood, short deck": a long flat hood over a bay sized for
a big swapped-in engine, pop-up headlamp pods in a fixed nose header, a
recessed full-width black grille over a wrap-around chrome tube bumper with
rubber over-riders, a fastback roof falling into a Kamm tail with round twin
tail lamps, a paint-coloured shoulder crease with a Coke-bottle kick over the
rear wheel, louvred rear quarter windows, frameless door glass, chrome bullet
mirrors and polished five-spoke wheels. Round shapes (lamps, gauges, tips,
mirrors) everywhere a modern car would use rounded rectangles.

Length 4.45 m, width 1.80 m (body, no mirrors), height 1.22 m, wheelbase
2.55 m, track 1.48 m, tyres 215/60 R15 front (r = 0.3195 m), 225/60 R15 rear
(r = 0.3255 m). Two doors, glass hatch.

Interactive: 4 corners, 2 doors, hatch, hood (with two optional hood variants
as hidden child nodes: bulge and shaker scoop), pop-up lamp pods, steering
wheel (+ horn), gear lever (6-speed H gate), handbrake, 3 pedals, 2 stalks,
six round gauges plus a clock with a minute hand, five chrome toggles, two
heater sliders, a radio with five preset buttons and two knobs, ignition key,
window cranks, wipers.
"""
import math

from cargen import controls as C
from cargen.closed_body import ClosedCarBody
from cargen.flow import FlowRecorder
from cargen.wheels import Wheel, corner
from cargen.common import materials, finish, door_card, seat_closed, cabin_shell, top_z, surface_path
from assetgen.scene import Model
from assetgen.carbody import Panel
from assetgen.parts import mirror  # noqa: F401  (bullet mirrors are drawn here)
from assetgen import mesh as M
from assetgen.vec import norm, compose, translate, rot_axis, xform_point

WB, TRACK = 2.55, 1.48
XF, XR = WB / 2, -WB / 2
X_FRONT, X_REAR = 2.25, -2.20
RF, RR = 0.3195, 0.3255
TYRE_F = Wheel(radius=RF, width=0.215, rim_radius=0.1905, style="polished5", rim_color=(0.78, 0.79, 0.80),
               disc=True, caliper_color=(0.12, 0.12, 0.13))
TYRE_R = Wheel(radius=RR, width=0.225, rim_radius=0.1905, style="polished5", rim_color=(0.78, 0.79, 0.80),
               disc=True, caliper_color=(0.12, 0.12, 0.13))
PAINT = (0.72, 0.30, 0.08)       # burnt orange metallic

DOORS = [("door_L", 1, -0.70, 0.48), ("door_R", -1, -0.70, 0.48)]
HOOD_X0, HOOD_X1 = 0.47, 1.82       # hood panel; the nose header ahead of it carries the lamp pods
HATCH_X = -0.65                      # hatch glass behind this x (hinged at the roof)
POD_X0, POD_X1, POD_Y, POD_W = 1.86, 2.08, 0.60, 0.30
STEER_LOCK_DEG = 34.0
LOCK_TO_LOCK_TURNS = 3.4
EYE_Y = 0.38


def build():
    m = Model("ventura_gt", "Ventura GT (1970s grand tourer)",
              "1970s front-engine fastback GT, 4.45 m, 215/60 R15 + 225/60 R15, front-mid longitudinal V8 bay.",
              "Ground plane below the wheel centres, x = 0 midway between the axles, y = 0 on the centreline.")
    materials(m, PAINT)
    m.mat("paint", color=PAINT, metallic=0.55, roughness=0.28, role="paint")
    m.mat("seat", color=(0.10, 0.07, 0.05), roughness=0.75, tex=("fabric", {"color": (0.10, 0.07, 0.05)}), tile=0.2)
    m.mat("seat_accent", color=(0.62, 0.55, 0.42), roughness=0.85, tex=("fabric", {"color": (0.6, 0.52, 0.4)}),
          tile=0.05)
    m.mat("dash_hard", color=(0.07, 0.065, 0.06), roughness=0.8)
    m.mat("dash_top", color=(0.05, 0.045, 0.04), roughness=0.9)
    m.mat("wood", color=(0.36, 0.18, 0.07), roughness=0.35, metallic=0.0)
    m.mat("rubber_bumper", color=(0.04, 0.04, 0.04), roughness=0.85)
    flow = FlowRecorder(m)
    ck = C.Cockpit(m, LOCK_TO_LOCK_TURNS, STEER_LOCK_DEG)

    wheels = [dict(x=XF, z=RF, r=RF, y_out=TRACK / 2 + TYRE_F.width / 2, y_in=TRACK / 2 - TYRE_F.width / 2),
              dict(x=XR, z=RR, r=RR, y_out=TRACK / 2 + TYRE_R.width / 2, y_in=TRACK / 2 - TYRE_R.width / 2)]
    moving = [d[0] for d in DOORS] + ["hood", "hatch"]
    body = ClosedCarBody(
        m, flow=flow, moving_parts=moving, x_rear=X_REAR, x_front=X_FRONT,
        W=[(X_REAR, 0.80), (-1.95, 0.885), (-1.3, 0.90), (-0.85, 0.885), (-0.3, 0.862), (0.4, 0.868),
           (1.275, 0.89), (1.9, 0.88), (X_FRONT, 0.78)],
        zb=[(X_REAR, 0.36), (-1.9, 0.24), (-1.6, 0.17), (1.6, 0.17), (2.0, 0.24), (X_FRONT, 0.30)],
        zt=[(X_REAR, 0.90), (-1.95, 0.92), (-1.4, 0.915), (-0.9, 0.88), (-0.3, 0.86), (0.45, 0.875),
            (1.45, 0.88), (1.75, 0.86), (2.05, 0.80), (X_FRONT, 0.72)],
        exp=[(X_REAR, 4.0), (-1.8, 4.5), (1.6, 4.5), (X_FRONT, 3.5)],
        end_r=(0.10, 0.18), wheels=wheels, arch_clear=0.045,
        green=dict(x0=-1.90, x1=0.42,
                   zroof=[(-1.90, 0.93), (-1.6, 0.99), (-1.2, 1.08), (-0.8, 1.165), (-0.45, 1.215), (-0.15, 1.22),
                          (0.05, 1.17), (0.25, 1.04), (0.42, 0.90)],
                   wb=[(-1.90, 0.80), (-1.4, 0.83), (-0.5, 0.84), (0.42, 0.83)],
                   wr=[(-1.90, 0.55), (-1.3, 0.62), (-0.5, 0.64), (0.0, 0.63), (0.42, 0.60)],
                   r=0.08, crown=0.03))
    W = body.W

    # ---- opening panels
    def door_inside(side, x0, x1):
        def f(c, n):
            return (x0 - 0.012 < c[0] < x1 + 0.012 and c[1] * side > 0 and n[1] * side > 0.3 and n[2] < 0.75
                    and c[2] > 0.25 and not body.in_arch(c, margin=0.004))
        return f

    def door_border(side, x0, x1):
        def f(c, n):
            return c[0] < x0 + 0.008 or c[0] > x1 - 0.008 or c[2] < 0.27
        return f

    for name, side, x0, x1 in DOORS:
        body.panels.append(Panel(name, door_inside(side, x0, x1), door_border(side, x0, x1)))
    body.panels.append(Panel(
        "hood", lambda c, n: HOOD_X0 - 0.01 < c[0] < HOOD_X1 + 0.01 and body.ls > 0.70 and n[2] > 0.3,
        lambda c, n: c[0] < HOOD_X0 + 0.012 or c[0] > HOOD_X1 - 0.012 or body.ls < 0.72))

    def lower(c, n, b):
        x, y, z = c
        if n[2] < -0.6:
            return "underbody"
        if b.in_arch(c):
            return "arch_liner"
        if z < 0.22 and abs(n[1]) > 0.4:
            return "trim_black"          # thin black sill line
        return Ellipsis

    def green(c, n, b):
        x, y, z = c
        seg, pos = b.gseg, b.gpos
        if seg == "roof":
            if x > 0.03:   # windshield, chrome surround as a trim band on the glass edge
                return "chrome" if pos < 0.035 or x < 0.06 or x > 0.405 else "glass"
            if x < HATCH_X:  # fastback hatch glass with a thin black frame
                if x < -1.86 or x > HATCH_X - 0.03:
                    return ("hatch", "trim_black")
                return ("hatch", "glass_dark") if pos < 0.88 else ("hatch", "trim_black")
            return "paint"
        if seg == "corner":
            if x < HATCH_X:
                return ("hatch", "trim_black")
            return "paint"
        if seg == "side":
            side = 1 if y > 0 else -1
            for name, s, x0, x1 in DOORS:
                if s == side and x0 < x < min(x1, 0.40):
                    if 0.10 < pos < 0.95 and x0 + 0.03 < x:
                        return (name, "glass")          # frameless door glass
                    return (name, "chrome") if pos <= 0.10 else (name, "trim_black")
            if -1.55 < x < -0.72 and 0.12 < pos < 0.88:
                slat = int((x + 1.55) / 0.046) % 2      # louvred quarter window
                return "trim_black" if slat == 0 else "glass_dark"
            return "paint"
        return "paint"

    body.build(lower, green)

    # ---- front: recessed grille, indicators, plate; rear: twin round lamps, plate
    rr, circ = M.rrect, M.circle
    body.patch(rr(1.18, 0.14, 0.03, 4, 0.0, 0.56), "grille", "front", offset=0.003, skirt_mat="chrome")
    for s in (1, -1):
        ind = "light_indicator_left" if s > 0 else "light_indicator_right"
        body.patch(rr(0.10, 0.04, 0.018, 3, s * 0.60, 0.40), ind, "front", offset=0.004, skirt_mat="chrome")
        for yy in (0.42, 0.62):
            body.patch(circ(0.055, 20, s * yy, 0.72), "light_tail", "rear", offset=0.004, skirt_mat="chrome")
            body.patch(circ(0.028, 16, s * yy, 0.72), "light_brake", "rear", offset=0.008, skirt=0.0)
        body.patch(rr(0.07, 0.035, 0.012, 3, s * 0.52, 0.56), "light_reverse", "rear", offset=0.004, skirt=0.0)
        # side markers (amber front, red rear) on the fenders
        body.patch(rr(0.06, 0.025, 0.01, 3, 1.95, 0.62), "light_indicator_left" if s > 0 else
                   "light_indicator_right", "left" if s > 0 else "right", offset=0.003, skirt=0.0)
        body.patch(rr(0.06, 0.025, 0.01, 3, -1.95, 0.66), "light_tail", "left" if s > 0 else "right",
                   offset=0.003, skirt=0.0)
    body.patch(rr(0.52, 0.11, 0.01, 2, 0.0, 0.58), "plate", "rear", skirt_mat="trim_black")
    body.patch(rr(0.46, 0.10, 0.01, 2, 0.0, 0.33), "plate", "front", offset=0.006, skirt_mat="trim_black")
    flow.mark("lamps, grille, plates")

    # ---- bumpers: chrome tubes wrapping round the corners, rubber over-riders
    def bumper(end, z, r, half):
        ys = [half * (-1 + 2 * k / 24) for k in range(25)]
        pts = [(body.surface_x(y, z, end) + end * (r + 0.012), y, z) for y in ys]
        for s in (-1, 1):
            yy = s * half
            xe = body.surface_x(yy, z, end)
            wrap = [(xe - end * d, s * (body.surface_y(xe - end * d, z) + r + 0.01), z) for d in (0.06, 0.16)]
            pts = (list(reversed(wrap)) + pts) if s < 0 else (pts + wrap)
        M.tube(m.mb, pts, r, "chrome", count=12)
        for s in (1, -1):
            yy = s * 0.36
            xo = body.surface_x(yy, z, end) + end * (2 * r + 0.012)
            M.rounded_box(m.mb, (xo, yy, z + 0.01), (0.05, 0.05, 0.11), 0.018, "rubber_bumper")

    bumper(1, 0.42, 0.024, 0.70)
    bumper(-1, 0.50, 0.024, 0.68)
    flow.mark("bumpers")

    # ---- shoulder crease (paint, closed tube) with a kick over the rear wheel
    def crease_z(x):
        kick = 0.025 * math.exp(-((x + 1.15) / 0.30) ** 2)
        return body.zt(x) - 0.075 + kick

    for s in (1, -1):
        segs = [(None, [1.98 - k * (1.98 - 0.50) / 30 for k in range(31)]),
                ([d for d in DOORS if d[1] == s][0][0], [0.46 - k * (0.46 + 0.68) / 24 for k in range(25)]),
                (None, [-0.72 - k * (1.98 - 0.72) / 28 for k in range(29)])]
        for part, xs in segs:
            M.tube(m.mb, surface_path(body, xs, crease_z, offset=0.002, side=s), 0.007, "paint", part or "",
                   count=8)
    flow.mark("shoulder crease")

    # ---- corners
    for key, x, y, steer, spec, r in (("FL", XF, TRACK / 2, True, TYRE_F, RF), ("FR", XF, -TRACK / 2, True, TYRE_F, RF),
                                      ("RL", XR, TRACK / 2, False, TYRE_R, RR),
                                      ("RR", XR, -TRACK / 2, False, TYRE_R, RR)):
        corner(m, key, (x, y, r), spec, steer=steer, steer_limits=(-STEER_LOCK_DEG, STEER_LOCK_DEG),
               travel=(-0.09, 0.09))
    flow.mark("wheels")

    # ---- panel nodes
    for name, side, x0, x1 in DOORS:
        m.node(name, None, (x1, side * (W(x1) - 0.02), 0.55), m.revolute((0, 0, -side), 0, 68, "door opening angle"))
        hx = x0 + 0.16
        zh = crease_z(hx) - 0.035
        M.rounded_box(m.mb, (hx, side * (body.surface_y(hx, zh) + 0.008), zh), (0.13, 0.016, 0.022), 0.008,
                      "chrome", name)                                     # chrome pull handle
    m.node("hood", None, (HOOD_X0, 0.0, body.zt(HOOD_X0)), m.revolute((0, -1, 0), 0, 60, "hood opening angle"))
    m.node("hatch", None, (HATCH_X, 0.0, body.green["zroof"](HATCH_X)),
           m.revolute((0, 1, 0), 0, 70, "hatch opening angle"))
    # bullet mirrors on the doors (chrome)
    for name, side in (("door_L", 1), ("door_R", -1)):
        base = (0.28, side * (W(0.28) - 0.04), 0.88)
        tip = (0.26, side * (W(0.28) + 0.045), 0.95)
        M.tube(m.mb, [base, (0.27, side * (W(0.28) + 0.01), 0.93), tip], 0.008, "chrome", name, count=8)
        M.lathe(m.mb, [(0.0, -0.085), (0.030, -0.07), (0.044, -0.025), (0.046, 0.0), (0.0, 0.0)], "chrome", name,
                axis_xf=compose(translate((tip[0] - 0.01, tip[1], tip[2])), rot_axis((0, 1, 0), -math.pi / 2)),
                count=20, closed_profile=True)
        M.cylinder(m.mb, (tip[0] - 0.009, tip[1], tip[2]), (tip[0] - 0.012, tip[1], tip[2]), 0.036, "mirror", name,
                   count=20)
    # wipers parked at the cowl (parallel pair)
    ws_n = norm((0.75, 0.0, 0.66))
    for name, py, L in (("wiper_L", 0.10, 0.48), ("wiper_R", -0.48, 0.46)):
        p = (0.44, py, 0.905)
        m.node(name, None, p, m.revolute(ws_n, 0, 95, "wiper sweep"))
        M.box(m.mb, (p[0] - 0.01, py + L / 2, p[2] + 0.012), (0.014, L, 0.010), "trim_black", name)
        M.cylinder(m.mb, p, (p[0] + 0.01, p[1], p[2] + 0.022), 0.014, "trim_black", name, count=10)
    # fuel filler cap (chrome, on the left rear quarter)
    xf_, zf_ = -1.55, 0.80
    yf_ = body.surface_y(xf_, zf_)
    M.cylinder(m.mb, (xf_, yf_ - 0.004, zf_), (xf_, yf_ + 0.010, zf_), 0.045, "chrome", count=24)
    # twin exhaust tips under the rear bumper
    for s in (1, -1):
        M.lathe(m.mb, [(0.032, 0.0), (0.032, 0.22), (0.027, 0.22), (0.027, 0.0)], "exhaust_tip",
                axis_xf=compose(translate((-1.93, s * 0.45, 0.30)), rot_axis((0, 1, 0), -math.pi / 2)), count=18,
                closed_profile=True)
    flow.mark("handles, mirrors, wipers, fuel cap, tail pipes")

    build_popups(m, body)
    flow.mark("pop-up headlamp pods")

    build_cabin(m, ck, body)
    build_hood_variants(m, body)
    flow.skip("cabin, cockpit controls, engine bay, hidden hood variants")

    m.socket("socket_driver_eye", None, (-0.62, EYE_Y, 1.02), "driver eye point (camera)")
    m.socket("socket_engine_mount", None, (0.50, 0.0, 0.40),
             "engine origin (crank axis centre at the flywheel face), front-mid longitudinal, crank along -x")
    m.socket("socket_tow_front", None, (2.20, 0.32, 0.30), "front towing eye")
    m.socket("socket_tow_rear", None, (-2.15, -0.30, 0.34), "rear towing eye")
    for s, nm in ((1, "socket_exhaust_L"), (-1, "socket_exhaust_R")):
        m.socket(nm, None, (-2.15, s * 0.45, 0.30), "exhaust outlet, flow along -x")
    m.extra_rig["engine_bay"] = {
        "envelope_mm": [1000, 900, 720],
        "box_m": {"min": [0.45, -0.45, 0.14], "max": [1.45, 0.45, 0.86]},
        "origin": "socket_engine_mount", "layout": "front-mid longitudinal",
        "hood_variants": {"flat": {"clearance_mm": 720}, "hood_bulge": {"clearance_mm": 780},
                          "hood_scoop": {"clearance_mm": 840}},
        "note": "free box between the firewall (x 0.44), radiator (x 1.62), inner fenders (|y| 0.52) and the "
                "crossmember (z 0.14); the hood variants raise the top"}
    m.notes += [
        "Generic 1970s fastback GT; class-typical dimensions, not a specific production car.",
        "Pop-up lamp pods popup_L/popup_R hinge at their rear-bottom edge: 0 = closed, 90 = lamps facing forward.",
        "Hood variants hood_bulge / hood_scoop are children of 'hood' with extras default_hidden = true; the "
        "garage shows the one the installed engine needs (rig engine_bay.hood_variants).",
        "Paint material 'paint'. Cockpit parts are driven by ventura_gt.cockpit.json (rg.cockpit/1).",
    ]
    return finish(m, flow, ck,
                  "Closed visual shells per node; arches, grille mouth and tail pipes open. Hidden hood variants "
                  "and the cabin are not in the selection. Not a boolean union.")


def build_popups(m, body):
    """Pop-up lamp pods: a lid following the nose top, twin round lamps on the pod's underside."""
    mb = m.mb
    for side, name in ((1, "popup_L"), (-1, "popup_R")):
        yc = side * POD_Y
        pivot = (POD_X0, yc, top_z(body, POD_X0, yc) - 0.075)
        m.node(name, None, pivot, m.revolute((0, -1, 0), 0, 90, "pop-up lamp pod, 0 = closed, 90 = raised"))
        secs = []
        n = 10
        for k in range(n + 1):
            x = POD_X0 + (POD_X1 - POD_X0) * k / n
            ring = []
            for j in range(12):
                t = j / 11
                y = yc - POD_W / 2 + POD_W * t
                ring.append((x, y, top_z(body, x, y) + 0.005))
            zb = pivot[2]
            ring += [(x, yc + POD_W / 2, zb), (x, yc - POD_W / 2, zb)]
            secs.append(ring)
        M.loft(mb, secs, "paint", name)
        # black shut line round the lid top
        edge = ([(POD_X0, yc - POD_W / 2 + POD_W * j / 8) for j in range(8)] +
                [(POD_X0 + (POD_X1 - POD_X0) * j / 6, yc + POD_W / 2) for j in range(6)] +
                [(POD_X1, yc + POD_W / 2 - POD_W * j / 8) for j in range(8)] +
                [(POD_X1 - (POD_X1 - POD_X0) * j / 6, yc - POD_W / 2) for j in range(6)])
        M.tube(mb, [(x, y, top_z(body, x, y) + 0.004) for x, y in edge], 0.004, "trim_black", name, count=6,
               closed=True)
        # lamps under the pod: facing down when closed, forward when raised
        for k in (-1, 1):
            cx, cy = (POD_X0 + POD_X1) / 2 + 0.005, yc + k * 0.072
            zb = pivot[2]
            M.cylinder(mb, (cx, cy, zb - 0.002), (cx, cy, zb - 0.020), 0.060, "chrome", name, count=24)
            M.cylinder(mb, (cx, cy, zb - 0.020), (cx, cy, zb - 0.026), 0.054, "light_head", name, count=24)


def build_hood_variants(m, body):
    """Two hidden hood tops for the big-engine options, children of the hood node."""
    mb = m.mb
    # bulge: a smooth +60 mm power bulge along the centre
    m.node("hood_bulge", "hood", (HOOD_X0, 0.0, body.zt(HOOD_X0)), variant="hood", option="hood_bulge",
           default_hidden=True, raise_mm=60)
    secs = []
    for k in range(17):
        x = 0.62 + 0.95 * k / 16
        h = 0.060 * math.sin(math.pi * min(1.0, (x - 0.62) / 0.25) / 2) * (1.0 if x < 1.32 else
                                                                           max(0.0, 1 - (x - 1.32) / 0.25))
        ring = []
        for j in range(17):
            y = -0.30 + 0.60 * j / 16
            fy = (1 - (abs(y) / 0.30) ** 2)
            ring.append((x, y, top_z(body, x, y) - 0.004 + max(0.002, h) * fy + 0.002))
        ring += [(x, 0.30, top_z(body, x, 0.30) - 0.006), (x, -0.30, top_z(body, x, -0.30) - 0.006)]
        secs.append(ring)
    M.loft(mb, secs, "paint", "hood_bulge")
    # shaker scoop: +120 mm intake box standing on the hood, black mouth facing forward
    m.node("hood_scoop", "hood", (HOOD_X0, 0.0, body.zt(HOOD_X0)), variant="hood", option="hood_scoop",
           default_hidden=True, raise_mm=120)
    zb = top_z(body, 1.05, 0.0) - 0.004
    M.rounded_box(mb, (1.05, 0.0, zb + 0.06), (0.40, 0.42, 0.12), 0.035, "paint", "hood_scoop")
    M.rounded_box(mb, (1.255, 0.0, zb + 0.065), (0.02, 0.34, 0.07), 0.01, "grille", "hood_scoop")
    M.rounded_box(mb, (1.05, 0.0, zb + 0.004), (0.46, 0.48, 0.012), 0.004, "trim_black", "hood_scoop")


def build_cabin(m, ck, body):
    """Interior: cliff dash with a wood strip, six cowled gauges + clock, toggles, sliders, radio, console."""
    mb = m.mb
    cabin_shell(m, body, x_front=0.42, x_rear=-1.80, floor_z=0.18, half_w=0.74, roof_drop=0.025)
    # ---- dash: cliff face, wood strip, padded top
    M.rounded_box(mb, (0.38, 0.0, 0.66), (0.22, 1.50, 0.34), 0.04, "dash_hard")
    M.rounded_box(mb, (0.40, 0.0, 0.85), (0.26, 1.48, 0.07), 0.03, "dash_top")
    M.rounded_box(mb, (0.267, 0.0, 0.565), (0.012, 1.30, 0.045), 0.006, "wood")
    M.rounded_box(mb, (0.45, 0.0, 0.40), (0.06, 1.40, 0.44), 0.02, "dash_hard")          # firewall trim
    for yy in (0.66, -0.66):                                                               # round end vents
        f = C.frame((0.268, yy, 0.74), (-1.0, 0.0, 0.0))
        C.disk(mb, f, -0.004, 0.006, 0.032, "chrome", "", count=20)
        C.disk(mb, f, 0.006, 0.009, 0.026, "trim_black", "", count=20)
    # ---- instrument binnacle: speedo + tach, four small gauges in the centre stack
    gn = norm((-1.0, 0.0, 0.18))
    M.rounded_box(mb, (0.30, EYE_Y, 0.765), (0.06, 0.34, 0.15), 0.03, "dash_hard")
    C.round_gauge(m, ck, "speed", (0.265, EYE_Y + 0.08, 0.76), gn, 0.052, "speed_kmh", (0.0, 260.0),
                  sweep_deg=270.0, start_deg=225.0, ticks=14, cowl_mat="dash_top", label="speedometer")
    C.round_gauge(m, ck, "tach", (0.265, EYE_Y - 0.08, 0.76), gn, 0.052, "engine_rpm", (0.0, 8000.0),
                  sweep_deg=270.0, start_deg=225.0, ticks=17, cowl_mat="dash_top", label="tachometer")
    sm = [("fuel", "fuel_fraction", (0.0, 1.0)), ("coolant", "coolant_c", (40.0, 130.0)),
          ("oil", "oil_pressure_bar", (0.0, 6.0)), ("volts", "battery_v", (8.0, 16.0))]
    for k, (nm, src, rg) in enumerate(sm):
        zz = 0.76 if k < 2 else 0.70
        yy = 0.07 if k % 2 == 0 else -0.07
        C.round_gauge(m, ck, nm, (0.268, yy, zz), gn, 0.026, src, rg, sweep_deg=90.0, start_deg=135.0, ticks=5,
                      label=nm + " gauge")
    # clock (hour hand = gauge needle, minute hand = extra needle) on the passenger side
    clk = (0.268, -0.36, 0.73)
    C.round_gauge(m, ck, "clock_h", clk, gn, 0.034, "clock_h", (0.0, 12.0), sweep_deg=360.0, start_deg=90.0,
                  ticks=12, label="clock hour hand")
    C.extra_needle(m, ck, "clock_m", clk, gn, 0.030, "clock_m", {"in": [0.0, 60.0], "out_deg": [0.0, 360.0]},
                   label="clock minute hand", z=0.0085)
    # ---- toggle row on the wood strip
    tn = (-1.0, 0.0, 0.0)
    for k, (nm, lab) in enumerate((("lights", "headlamps (pop-ups)"), ("wipers", "wipers"), ("fan", "heater fan"),
                                   ("hazard", "hazard lights"), ("fog", "fog lamps"))):
        C.toggle_switch(m, ck, nm, (0.262, 0.16 - 0.08 * k, 0.565), tn, nm, label=lab)
    # ---- heater sliders (centre stack, below the gauges)
    sn = norm((-1.0, 0.0, 0.3))
    M.rounded_box(mb, (0.30, 0.0, 0.47), (0.10, 0.28, 0.12), 0.02, "dash_hard")
    for nm, yy, lab in (("heat", 0.05, "heater temperature"), ("air", -0.05, "air distribution")):
        C.slider(m, ck, nm, (0.252, yy, 0.44), sn, (0.0, 0.0, 1.0), 0.05, nm, [0, 1, 2, 3], label=lab)
    # ---- radio: dial window, 5 preset buttons, volume + tuning knobs
    rc = (0.27, 0.0, 0.645)
    C.box_local(mb, C.frame(rc, (-1, 0, 0)), (0, 0, -0.01), (0.18, 0.05, 0.02), "dash_hard", "")
    C.box_local(mb, C.frame(rc, (-1, 0, 0)), (0, 0.008, 0.0005), (0.10, 0.016, 0.001), "gauge_tick", "")
    for k in range(5):
        C.push_button(m, ck, f"radio_{k + 1}", (0.258, -0.04 + 0.02 * k, 0.630), (-1, 0, 0), (0.016, 0.012, 0.008),
                      f"radio_preset_{k + 1}", mat="chrome", momentary=True, label=f"radio preset {k + 1}")
    for nm, yy, lab in (("volume", 0.075, "radio volume"), ("tune", -0.075, "radio tuning")):
        C.rotary_knob(m, ck, nm, (0.26, yy, 0.645), (-1, 0, 0), 0.014, 0.014, f"radio_{nm}", list(range(11)),
                      mat="chrome", label=lab)
    # ignition key (knob turned by the key, right of the column)
    C.rotary_knob(m, ck, "ignition", (0.262, EYE_Y - 0.20, 0.60), (-1, 0, 0), 0.016, 0.02, "ignition",
                  [0, 1, 2, 3], sweep_deg=90.0, mat="chrome", label="ignition key",
                  source="control:ignition")
    # ---- deep-dish wood-rim three-spoke steering wheel with a horn button
    sw_c = (-0.05, EYE_Y, 0.80)
    col = (0.89, 0.0, 0.45)
    node, f = C.steering_wheel(m, ck, sw_c, col, 0.19, 3, "wood", "chrome", "dash_hard", rim_r=0.014,
                               spoke_angles=[-math.pi / 2, math.pi / 6 - math.pi / 6, math.pi], dish=0.05,
                               column_len=0.42)
    axis = norm((-0.89, 0.0, -0.45))
    C.push_button(m, ck, "horn", xform_point(f, (0, 0, -0.05 + 0.006)), axis, (0.05, 0.05, 0.008), "horn",
                  mat="trim_black", part=node, momentary=True, round_r=0.026, label="horn")
    # stalks from the column
    piv = (0.12, EYE_Y, 0.715)
    C.stalk(m, ck, "indicator", (piv[0], piv[1] + 0.035, piv[2]), (-0.2, 1.0, -0.05), 0.14, "indicator",
            ["right", "off", "left"], mat="chrome", tip_mat="trim_black", label="indicator stalk", axis=(1, 0, 0))
    C.stalk(m, ck, "wiper", (piv[0], piv[1] - 0.035, piv[2]), (-0.2, -1.0, -0.05), 0.12, "wipers",
            ["off", "slow", "fast"], mat="chrome", tip_mat="trim_black", label="wiper stalk", axis=(-1, 0, 0))
    # ---- tall tunnel console, 6-speed gate, pull-up handbrake
    M.rounded_box(mb, (-0.02, 0.0, 0.31), (0.86, 0.24, 0.26), 0.06, "dash_hard")
    M.rounded_box(mb, (0.08, 0.0, 0.445), (0.20, 0.16, 0.012), 0.004, "wood")
    gate = {"1": [1, 1], "2": [1, -1], "3": [0, 1], "4": [0, -1], "5": [-1, 1], "6": [-1, -1],
            "R": [1.5, 1], "N": [0, 0]}
    C.gear_lever(m, ck, (0.08, 0.0, 0.45), 0.20, gate, "wood", rod_mat="chrome", boot_mat="boot", lean_deg=8.0,
                 knob_r=0.022, ball=True)
    C.lever(m, ck, "handbrake", (-0.32, 0.0, 0.445), (0.0, 1.0, 0.0), (0.24, 0.0, 0.03), "handbrake", 22.0,
            mat="chrome", grip_mat="dash_hard", label="handbrake", grip_r=0.016)
    # ---- pedals: hanging clutch and brake, floor-hinged organ throttle
    for name, yy, pc, organ in (("clutch", EYE_Y + 0.13, (0.30, EYE_Y + 0.13, 0.30), False),
                                ("brake", EYE_Y + 0.01, (0.29, EYE_Y + 0.01, 0.29), False),
                                ("throttle", EYE_Y - 0.11, (0.33, EYE_Y - 0.11, 0.25), True)):
        piv = (0.40, yy, 0.58) if not organ else (0.36, yy, 0.20)
        C.pedal(m, ck, name, piv, pc, name, organ=organ, label=f"{name} pedal")
    # ---- bucket seats, luggage deck behind them
    for yy in (EYE_Y, -EYE_Y):
        seat_closed(m, (-0.55, yy, 0.36), width=0.50, recline=0.36, mat="seat", accent="seat_accent", depth=0.52,
                    height=0.14)
    M.rounded_box(mb, (-1.30, 0.0, 0.40), (0.70, 1.30, 0.04), 0.01, "carpet")
    for yy in (0.25, -0.25):
        M.rounded_box(mb, (-1.30, yy, 0.425), (0.66, 0.025, 0.006), 0.002, "trim_black")  # luggage straps
    # ---- door cards with window cranks and chrome pulls
    for name, side, x0, x1 in DOORS:
        door_card(m, name, side, x0 + 0.05, x1 - 0.08, 0.30, 0.84, y=0.74, mat="dash_hard", top_mat="dash_top")
        C.rotary_knob(m, ck, f"crank_{name[-1]}", (0.02, side * 0.69, 0.56), (0, -side, 0), 0.03, 0.012,
                      f"window_{name[-1].lower()}", list(range(9)), sweep_deg=1440.0, mat="chrome", part=name,
                      label=f"window crank {name[-1]}", up_hint=(0, 0, 1))
    ck.notes.append("Pedals and the handbrake follow the applied control channels; the gear lever follows the "
                    "selected gear through its gate (R is beyond 1, top left); gauges follow the snapshot; the "
                    "clock has two needles (hour, minute).")
    # ---- engine bay furniture (engine itself: separate generated model at socket_engine_mount)
    M.rounded_box(mb, (1.62, 0.0, 0.52), (0.06, 1.12, 0.42), 0.02, "radiator")
    M.rounded_box(mb, (1.52, 0.0, 0.52), (0.10, 0.80, 0.40), 0.03, "trim_black")        # fan shroud
    for s in (1, -1):
        M.rounded_box(mb, (0.98, s * 0.56, 0.60), (1.00, 0.04, 0.42), 0.015, "underbody")  # inner fenders
        M.rounded_box(mb, (1.15, s * 0.62, 0.78), (0.24, 0.16, 0.14), 0.03, "underbody")   # spring towers
    M.rounded_box(mb, (0.47, 0.0, 0.62), (0.03, 1.10, 0.50), 0.01, "underbody")          # firewall
    M.rounded_box(mb, (1.30, 0.0, 0.12), (0.10, 1.00, 0.06), 0.015, "underbody")         # crossmember
    M.rounded_box(mb, (1.25, -0.42, 0.74), (0.18, 0.14, 0.16), 0.02, "battery")
