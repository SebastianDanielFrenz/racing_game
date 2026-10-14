"""Pico: A-segment five-door city car (class of the Citroen C1 2018 / VW up! 2022; generic, no real model).

Design language "friendly box": short overhangs, wheels at the corners, tall
glass house, one-piece frameless glass tailgate with a black surround, a
smiling lower intake under a slim upper grille, big swept headlamps with an
LED day-running line, black plastic arch and sill cladding, 14 in steel
wheels behind plastic covers. One corner-radius family (rounded rectangles)
for lamps, windows, screen, buttons and vents.

Length 3.55 m, width 1.64 m (body, no mirrors), height 1.48 m, wheelbase
2.42 m, track 1.42 m, tyres 165/65 R14 (r = 0.285 m). Five doors; the rear
door windows pop out (hinged at their front edge).

Interactive: 4 corners, 4 doors, tailgate, rear pop-out windows, hood,
steering wheel (+ audio/phone buttons), gear lever (5-speed H gate),
handbrake, 3 pedals, 2 stalks, 3 climate knobs, hazard button, front window
switches, speedometer needle, mono LCD (rev bar/fuel/trip) and the 7 in
touchscreen as screens, wipers (front pair + rear).
"""
import math

from cargen import controls as C
from cargen.closed_body import ClosedCarBody
from cargen.flow import FlowRecorder
from cargen.wheels import Wheel, corner
from cargen.common import materials, finish, door_card, seat_closed, cabin_shell
from assetgen.scene import Model
from assetgen.carbody import Panel
from assetgen.parts import mirror
from assetgen import mesh as M
from assetgen.vec import norm, compose, translate, rot_axis

WB, TRACK = 2.42, 1.42
XF, XR = WB / 2, -WB / 2
X_FRONT, X_REAR = 1.87, -1.68
R = 0.285
TYRE = Wheel(radius=R, width=0.165, rim_radius=0.1778, style="steel_cover", rim_color=(0.35, 0.36, 0.38),
             disc=True, caliper_color=(0.18, 0.18, 0.19))
TYRE_R = Wheel(radius=R, width=0.165, rim_radius=0.1778, style="steel_cover", rim_color=(0.35, 0.36, 0.38),
               disc=False, drum=True)
PAINT = (0.86, 0.22, 0.16)       # coral red
ROOF = (0.93, 0.93, 0.92)        # contrast roof (off-white); same as paint when the option is off

# doors: (name, side, x0, x1) along the side; front doors hinge at x1 (A pillar), rear at x1 (B pillar)
DOORS = [("door_FL", 1, -0.06, 0.93), ("door_RL", 1, -0.86, -0.08),
         ("door_FR", -1, -0.06, 0.93), ("door_RR", -1, -0.86, -0.08)]
STEER_LOCK_DEG = 37.0
STEERING_WHEEL_LOCK_DEG = 540.0      # degrees from centre to one lock


def build():
    m = Model("pico", "Pico (city car)",
              "A-segment five-door city car, 3.55 m, 165/65 R14, front transverse 1.0 L I3.",
              "Ground plane below the wheel centres, x = 0 midway between the axles, y = 0 on the centreline.")
    materials(m, PAINT, ROOF)
    m.mat("seat", color=(0.16, 0.17, 0.19), roughness=0.9, tex=("fabric", {"color": (0.15, 0.16, 0.18)}), tile=0.2)
    m.mat("seat_accent", color=(0.86, 0.22, 0.16), roughness=0.85)
    m.mat("dash_hard", color=(0.11, 0.115, 0.12), roughness=0.85)
    m.mat("dash_paint", color=PAINT, metallic=0.2, roughness=0.35, role="paint")
    flow = FlowRecorder(m)
    ck = C.Cockpit(m, STEERING_WHEEL_LOCK_DEG, STEER_LOCK_DEG)

    y_out, y_in = TRACK / 2 + TYRE.width / 2, TRACK / 2 - TYRE.width / 2
    wheels = [dict(x=XF, z=R, r=R, y_out=y_out, y_in=y_in), dict(x=XR, z=R, r=R, y_out=y_out, y_in=y_in)]
    moving = [d[0] for d in DOORS] + ["hood", "tailgate", "popout_RL", "popout_RR"]
    body = ClosedCarBody(
        m, flow=flow, moving_parts=moving, x_rear=X_REAR, x_front=X_FRONT,
        W=[(X_REAR, 0.78), (-1.45, 0.81), (-1.1, 0.82), (1.1, 0.82), (1.55, 0.80), (X_FRONT, 0.74)],
        zb=[(X_REAR, 0.34), (-1.55, 0.24), (-1.2, 0.17), (1.2, 0.17), (1.6, 0.21), (X_FRONT, 0.25)],
        zt=[(X_REAR, 1.02), (-1.6, 1.04), (-1.0, 1.03), (0.2, 0.99), (0.95, 0.94), (1.35, 0.88), (1.7, 0.82),
            (X_FRONT, 0.74)],
        exp=[(X_REAR, 4.5), (-1.4, 6.0), (1.4, 6.0), (X_FRONT, 4.0)],
        end_r=(0.10, 0.22), wheels=wheels, arch_clear=0.05,
        green=dict(x0=-1.63, x1=0.97,
                   zroof=[(-1.63, 1.04), (-1.60, 1.25), (-1.55, 1.40), (-1.45, 1.445), (-1.0, 1.465), (0.1, 1.475),
                          (0.3, 1.46), (0.62, 1.24), (0.97, 0.95)],
                   wb=[(-1.63, 0.74), (-1.5, 0.77), (0.6, 0.78), (0.97, 0.76)],
                   wr=[(-1.63, 0.62), (-1.4, 0.66), (0.2, 0.67), (0.97, 0.66)],
                   r=0.11, crown=0.035))
    W = body.W

    # ---- opening panels
    def door_inside(side, x0, x1):
        def f(c, n):
            return (x0 - 0.012 < c[0] < x1 + 0.012 and c[1] * side > 0 and n[1] * side > 0.3 and n[2] < 0.75
                    and c[2] > 0.27 and not body.in_arch(c, margin=0.004))
        return f

    def door_border(side, x0, x1):
        def f(c, n):
            return c[0] < x0 + 0.008 or c[0] > x1 - 0.008 or c[2] < 0.29
        return f

    for name, side, x0, x1 in DOORS:
        body.panels.append(Panel(name, door_inside(side, x0, x1), door_border(side, x0, x1)))
    body.panels.append(Panel(
        "hood", lambda c, n: 0.99 < c[0] < 1.80 and body.ls > 0.68 and n[2] > 0.3,
        lambda c, n: c[0] < 1.012 or c[0] > 1.785 or body.ls < 0.70))
    body.panels.append(Panel(
        "tailgate", lambda c, n: c[0] < -1.52 and c[2] > 0.56 and n[0] < -0.2 and abs(c[1]) < 0.60,
        lambda c, n: c[2] < 0.58 or abs(c[1]) > 0.58))

    def lower(c, n, b):
        x, y, z = c
        if n[2] < -0.6:
            return "underbody"
        if b.in_arch(c):
            return "arch_liner"
        # black cladding: sills and bumper bottoms (straight lines along the grid, no arch flares)
        if z < 0.27 and abs(n[1]) > 0.4:
            return "cladding"
        if (x > 1.56 and z < 0.27) or (x < -1.56 and z < 0.44):
            return "cladding"
        return Ellipsis

    def green(c, n, b):
        x, y, z = c
        seg, pos = b.gseg, b.gpos
        if seg == "roof":
            if x > 0.33:   # windshield with a black ceramic band
                return "trim_black" if pos < 0.10 or x < 0.38 or x > 0.93 else "glass"
            if x < -1.47:  # tailgate glass, frameless, black surround printed on it
                return ("tailgate", "glass_dark") if pos < 0.86 and z > 1.02 else ("tailgate", "trim_black")
            return "paint_roof"
        if seg == "corner":
            if x < -1.47:
                return ("tailgate", "trim_black")
            return "paint_roof" if -1.45 < x < 0.30 else "trim_black"
        if seg == "side":
            side = 1 if y > 0 else -1
            if x < -1.47:
                return ("tailgate", "trim_black")
            for name, s, x0, x1 in DOORS:
                if s == side and x0 < x < x1:
                    in_glass = 0.14 < pos < 0.93 and x0 + 0.035 < x < x1 - 0.035
                    if name in ("door_RL", "door_RR") and in_glass:
                        return ("popout_" + name[-2:], "glass")
                    if in_glass:
                        return (name, "glass")
                    return (name, "trim_black")
            if -1.28 < x < -0.92 and 0.14 < pos < 0.86:
                return "glass"     # rear quarter light
            return "trim_black"
        return "paint"

    body.build(lower, green)

    # ---- lamps, grilles, plate (draped, backed)
    rr = M.rrect
    body.patch(rr(0.44, 0.045, 0.02, 4, 0.0, 0.685), "trim_black", "front", offset=0.003)        # slim upper grille
    smile = [(0.36 * math.cos(math.pi * (1 + k / 20)) * 1.0, 0.40 + 0.06 * math.sin(math.pi * (1 + k / 20)) + 0.0)
             for k in range(21)]
    smile = smile + [(p[0] * 0.92, p[1] + 0.085 + 0.02 * (1 - (p[0] / 0.36) ** 2)) for p in reversed(smile)]
    body.patch(smile, "grille", "front", offset=0.004, skirt_mat="plastic_black")              # smiling lower intake
    for s in (1, -1):
        ind = "light_indicator_left" if s > 0 else "light_indicator_right"
        lamp = [(s * (0.43 + 0.18 * math.cos(math.pi * k / 12)), 0.70 + 0.06 * math.sin(math.pi * k / 12) - 0.02 * k / 12)
                for k in range(13)]
        lamp += [(s * (0.43 - 0.18 + 0.36 * k / 6), 0.655 - 0.02 * math.sin(math.pi * k / 6)) for k in range(7)][1:-1]
        body.patch(lamp, "light_head", "front", offset=0.004, skirt_mat="trim_black")
        body.patch(rr(0.20, 0.016, 0.008, 3, s * 0.43, 0.665), "light_drl", "front", offset=0.007, skirt=0.0)
        body.patch(rr(0.05, 0.03, 0.012, 3, s * 0.575, 0.72), ind, "front", offset=0.007, skirt=0.0)
        body.patch(rr(0.10, 0.04, 0.015, 3, s * 0.54, 0.36), "trim_black", "front", offset=0.004)   # fog blank
        # rear: tall corner clusters in the lower body, below the tailgate glass
        body.patch(rr(0.12, 0.30, 0.045, 4, s * 0.62, 0.84), "light_tail", "rear", skirt_mat="trim_black")
        body.patch(rr(0.085, 0.075, 0.02, 3, s * 0.62, 0.93), "light_brake", "rear", offset=0.007, skirt=0.0)
        body.patch(rr(0.085, 0.05, 0.015, 3, s * 0.62, 0.765), ind, "rear", offset=0.007, skirt=0.0)
        body.patch(rr(0.08, 0.04, 0.012, 3, s * 0.55, 0.42), "light_reverse", "rear", offset=0.004, skirt=0.0)
    body.patch(rr(0.52, 0.11, 0.01, 2, 0.0, 0.70), "plate", "rear", skirt_mat="trim_black")
    body.patch(rr(0.40, 0.09, 0.01, 2, 0.0, 0.41), "plate", "front", offset=0.012, skirt_mat="trim_black")
    flow.mark("lamps, grilles, plates")

    # ---- corners
    for key, x, y, steer in (("FL", XF, TRACK / 2, True), ("FR", XF, -TRACK / 2, True),
                             ("RL", XR, TRACK / 2, False), ("RR", XR, -TRACK / 2, False)):
        corner(m, key, (x, y, R), TYRE if steer else TYRE_R, steer=steer,
               steer_limits=(-STEER_LOCK_DEG, STEER_LOCK_DEG), travel=(-0.08, 0.08))
    flow.mark("wheels")

    # ---- panel nodes
    for name, side, x0, x1 in DOORS:
        m.node(name, None, (x1, side * (W(x1) - 0.02), 0.62),
               m.revolute((0, 0, -side), 0, 70, "door opening angle"))
        hx = x0 + 0.14
        M.rounded_box(m.mb, (hx, side * (W(hx) + 0.006), 0.90), (0.15, 0.022, 0.03), 0.01, "plastic_black", name)
    for name, side in (("popout_RL", 1), ("popout_RR", -1)):
        x1 = -0.12
        m.node(name, "door_" + name[-2:], (x1, side * 0.74, 1.10),
               m.revolute((0, 0, -side), 0, 6, "rear pop-out window, + = opened outward at the rear edge"))
    m.node("hood", None, (1.0, 0.0, body.zt(1.0)), m.revolute((0, -1, 0), 0, 55, "hood opening angle"))
    m.node("tailgate", None, (-1.47, 0.0, 1.47), m.revolute((0, 1, 0), 0, 75, "tailgate opening angle"))
    M.rounded_box(m.mb, (-1.69, 0.0, 0.79), (0.03, 0.18, 0.04), 0.01, "chrome", "tailgate")   # tailgate handle bar
    for name, side in (("door_FL", 1), ("door_FR", -1)):
        mirror(m, name, (0.80, side * (W(0.80) - 0.04), 1.02), side, size=(0.09, 0.17, 0.11),
               body_mat="paint", arm=0.03)
    # wipers: two at the cowl, one on the tailgate glass
    ws_n = norm((0.55, 0.0, 0.83))
    for name, py, L, parent in (("wiper_L", 0.04, 0.55, None), ("wiper_R", -0.56, 0.48, None)):
        p = (0.97, py, 0.955)
        m.node(name, parent, p, m.revolute(ws_n, 0, 100, "wiper sweep"))
        M.box(m.mb, (p[0] - 0.01, py + L / 2, p[2] + 0.012), (0.016, L, 0.011), "trim_black", name)
        M.cylinder(m.mb, p, (p[0] + 0.01, p[1], p[2] + 0.024), 0.016, "trim_black", name, count=10)
    m.node("wiper_rear", "tailgate", (-1.62, 0.0, 1.10), m.revolute((-1, 0, 0), 0, 160, "rear wiper sweep"))
    M.box(m.mb, (-1.625, 0.17, 1.10), (0.012, 0.34, 0.012), "trim_black", "wiper_rear")
    M.cylinder(m.mb, (-1.615, 0.0, 1.10), (-1.64, 0.0, 1.10), 0.018, "trim_black", "wiper_rear", count=10)
    # roof antenna (short stub)
    M.cylinder(m.mb, (-1.25, 0.0, 1.47), (-1.32, 0.0, 1.58), 0.006, "trim_black", count=8)
    M.rounded_box(m.mb, (-1.25, 0.0, 1.485), (0.06, 0.03, 0.02), 0.008, "trim_black")
    # exhaust tail pipe (hidden under the rear bumper, closed tube)
    M.lathe(m.mb, [(0.023, 0.0), (0.023, 0.22), (0.019, 0.22), (0.019, 0.0)], "exhaust_tip",
            axis_xf=compose(translate((-1.48, -0.45, 0.24)), rot_axis((0, 1, 0), -math.pi / 2)), count=16,
            closed_profile=True)
    flow.mark("handles, mirrors, wipers, antenna, tail pipe")

    build_cabin(m, ck, body)
    flow.skip("cabin, cockpit controls, engine bay")

    m.socket("socket_driver_eye", None, (-0.32, 0.34, 1.20), "driver eye point (camera)")
    m.socket("socket_engine_mount", None, (1.40, 0.0, 0.42),
             "engine origin (crank axis centre at the flywheel face), transverse, crank along +y")
    m.socket("socket_tow_front", None, (1.86, 0.30, 0.34), "front towing eye")
    m.socket("socket_tow_rear", None, (-1.68, -0.35, 0.36), "rear towing eye")
    m.socket("socket_exhaust", None, (-1.70, -0.45, 0.24), "exhaust outlet, flow along -x")
    m.extra_rig["engine_bay"] = {"envelope_mm": [620, 900, 560],
                                 "box_m": {"min": [1.08, -0.45, 0.28], "max": [1.70, 0.45, 0.84]},
                                 "origin": "socket_engine_mount",
                                 "layout": "front transverse", "note": "free box under the closed hood"}
    m.notes += [
        "Generic A-segment city car; class-typical dimensions, not a specific production car.",
        "Rear door windows pop out about their front edge (node popout_RL/RR, children of the rear doors).",
        "Paint material 'paint', roof material 'paint_roof' (contrast roof option), dash strip 'dash_paint'.",
        "Cockpit parts are driven by pico.cockpit.json (rg.cockpit/1).",
    ]
    return finish(m, flow, ck,
                  "Closed visual shells per node; arches, intake mouth and tail pipe open. Not a boolean union.")


def build_cabin(m, ck, body):
    """Interior: dash with a body-colour strip, cluster, 7 in screen, controls, seats, door cards."""
    mb = m.mb
    cabin_shell(m, body, x_front=0.90, x_rear=-1.42, floor_z=0.27, half_w=0.70, roof_drop=0.03)
    # ---- dash: hard lower, body-colour strip across, soft top
    M.rounded_box(mb, (0.78, 0.0, 0.70), (0.34, 1.42, 0.26), 0.06, "dash_hard")
    M.rounded_box(mb, (0.66, 0.0, 0.795), (0.08, 1.38, 0.075), 0.02, "dash_paint")
    M.rounded_box(mb, (0.80, 0.0, 0.86), (0.30, 1.36, 0.06), 0.03, "dash_hard")
    M.rounded_box(mb, (0.86, 0.0, 0.52), (0.14, 1.30, 0.36), 0.04, "dash_hard")       # firewall trim
    # round vents: two at the ends, two in the middle (in the body-colour strip)
    for yy in (0.62, 0.12, -0.12, -0.62):
        f = C.frame((0.623, yy, 0.795), (-1.0, 0.0, 0.0))
        C.disk(mb, f, -0.004, 0.006, 0.034, "trim_black", "", count=20)
        for k in range(3):
            C.box_local(mb, f, (0.0, -0.016 + 0.016 * k, 0.007), (0.05, 0.004, 0.003), "dash_hard", "")
    # ---- cluster: hood + speedometer + mono LCD
    eye_y = 0.34
    M.rounded_box(mb, (0.66, eye_y, 0.905), (0.16, 0.30, 0.09), 0.035, "dash_hard")
    cn = norm((-1.0, 0.0, 0.25))
    C.round_gauge(m, ck, "speed", (0.585, eye_y + 0.045, 0.875), cn, 0.055, "speed_kmh", (0.0, 180.0),
                  sweep_deg=240.0, start_deg=210.0, ticks=10, face_mat="gauge_face", label="speedometer")
    C.screen(m, ck, "lcd", (0.588, eye_y - 0.07, 0.872), cn, (0.075, 0.05), (160, 96), "cluster_lcd",
             bezel=0.004, depth=0.008)
    # ---- 7 in touchscreen standing on the dash (shared rounded-rectangle family)
    sn = norm((-1.0, 0.0, 0.12))
    C.screen(m, ck, "center", (0.64, 0.0, 0.985), sn, (0.155, 0.087), (1024, 600), "infotainment", bezel=0.009,
             depth=0.016)
    M.rounded_box(mb, (0.70, 0.0, 0.915), (0.05, 0.08, 0.06), 0.015, "dash_hard")   # screen foot
    # ---- climate knobs + hazard button (centre, below the strip)
    pn = norm((-1.0, 0.0, 0.35))
    M.rounded_box(mb, (0.655, 0.0, 0.70), (0.04, 0.34, 0.11), 0.03, "dash_hard")
    for name, yy, det, lab in (("temp", 0.11, [0, 1, 2, 3, 4, 5, 6], "temperature"),
                               ("fan", 0.0, [0, 1, 2, 3, 4], "fan speed"),
                               ("mode", -0.11, [0, 1, 2, 3], "air distribution")):
        C.rotary_knob(m, ck, name, (0.632, yy, 0.69), pn, 0.026, 0.02, f"climate_{name}", det, label=lab)
    C.push_button(m, ck, "hazard", (0.636, 0.0, 0.745), pn, (0.03, 0.022, 0.008), "hazard",
                  mat="button_red", label="hazard lights", symbol_mat="gauge_tick")
    # ---- steering wheel with audio/phone buttons
    sw_c = (0.36, eye_y, 0.84)
    col = (0.88, 0.0, -0.47)   # forward and down into the dash
    node, f = C.steering_wheel(m, ck, sw_c, col, 0.18, 3, "wheel_rim", "dash_hard", "dash_hard", rim_r=0.016,
                               spoke_angles=[-math.pi / 2, 0.0, math.pi])
    from assetgen.vec import xform_point
    axis = norm((-0.88, 0.0, 0.47))
    for k, (ch, lab) in enumerate((("audio_vol_up", "volume +"), ("audio_vol_down", "volume -"),
                                   ("phone", "phone"), ("audio_source", "source"))):
        side = 1 if k < 2 else -1
        local = (side * 0.105, 0.012 - 0.024 * (k % 2), 0.006)
        c = xform_point(f, local)
        C.push_button(m, ck, f"wheel_{ch}", c, axis, (0.016, 0.014, 0.005), ch, mat="button",
                      part=node, momentary=True, label=lab, up_hint=(0, 0, 1))
    # stalks (indicators left, wipers right) from the column top
    piv = (0.48, eye_y, 0.776)
    C.stalk(m, ck, "indicator", (piv[0], piv[1] + 0.035, piv[2]), (-0.25, 1.0, -0.1), 0.13, "indicator",
            ["right", "off", "left"], label="indicator stalk", axis=(1, 0, 0))
    C.stalk(m, ck, "wiper", (piv[0], piv[1] - 0.035, piv[2]), (-0.25, -1.0, -0.1), 0.13, "wipers",
            ["off", "intermittent", "slow", "fast"], label="wiper stalk", axis=(-1, 0, 0))
    # ---- console: gear lever (5 + R, H gate), handbrake
    M.rounded_box(mb, (0.30, 0.0, 0.40), (0.62, 0.20, 0.22), 0.05, "dash_hard")
    gate = {"1": [1, 1], "2": [1, -1], "3": [0, 1], "4": [0, -1], "5": [-1, 1], "R": [-1, -1], "N": [0, 0]}
    C.gear_lever(m, ck, (0.42, 0.0, 0.51), 0.17, gate, "knob", rod_mat="trim_black", boot_mat="boot",
                 lean_deg=9.0, knob_r=0.024, ball=True)
    C.lever(m, ck, "handbrake", (0.08, 0.0, 0.51), (0.0, 1.0, 0.0), (0.26, 0.0, 0.03), "handbrake", 22.0,
            mat="dash_hard", grip_mat="dash_hard", label="handbrake", grip_r=0.017)
    # ---- pedals (clutch, brake, throttle)
    for name, yy, pc in (("clutch", eye_y + 0.13, (0.66, eye_y + 0.13, 0.36)),
                         ("brake", eye_y + 0.01, (0.65, eye_y + 0.01, 0.36)),
                         ("throttle", eye_y - 0.11, (0.70, eye_y - 0.11, 0.34))):
        C.pedal(m, ck, name, (0.80, yy, 0.62), pc, name, label=f"{name} pedal")
    # ---- seats
    for yy in (0.36, -0.36):
        seat_closed(m, (-0.18, yy, 0.52), width=0.48, recline=0.30, mat="seat", accent="seat_accent")
    seat_closed(m, (-1.05, 0.0, 0.50), width=1.22, recline=0.26, mat="seat", accent=None, headrest=True,
                headrest_split=2)
    # ---- door cards (front cards carry the window switches)
    for name, side, x0, x1 in DOORS:
        door_card(m, name, side, x0 + 0.05, x1 - 0.06, 0.40, 0.92, y=0.68, mat="dash_hard",
                  top_mat="dash_paint" if name in ("door_FL", "door_FR") else "dash_hard",
                  bottle=True)
        if name in ("door_FL", "door_FR"):
            for k, (ch, lab) in enumerate((("window_front", "front window"),)):
                c = (0.25, side * 0.625, 0.70)
                C.push_button(m, ck, f"window_{name[-2:]}", c, (0.0, -side, 0.0) if False else (0.15, -side, 0.9),
                              (0.03, 0.02, 0.008), f"window_{name[-2:].lower()}", mat="button", part=name,
                              momentary=True, label=f"window switch {name[-2:]}")
    # roof grab handles (passenger and rear)
    for yy, xx in ((-0.64, 0.05), (0.64, -0.80), (-0.64, -0.80)):
        M.tube(mb, [(xx + 0.10, yy * 0.98, 1.38), (xx + 0.08, yy * 0.92, 1.33), (xx - 0.08, yy * 0.92, 1.33),
                    (xx - 0.10, yy * 0.98, 1.38)], 0.011, "dash_hard", count=8)
    ck.notes.append("Pedals and the handbrake follow the applied control channels; the gear lever follows the "
                    "selected gear through its gate; the speedometer follows the snapshot speed.")
    # ---- engine bay furniture (the engine itself is a separate generated model at socket_engine_mount)
    M.rounded_box(mb, (1.72, 0.0, 0.58), (0.06, 1.10, 0.36), 0.02, "radiator")          # radiator + fan shroud
    M.rounded_box(mb, (1.35, 0.52, 0.66), (0.16, 0.24, 0.18), 0.02, "battery")          # battery
    for s in (1, -1):
        M.rounded_box(mb, (1.18, s * 0.60, 0.70), (0.20, 0.16, 0.20), 0.03, "underbody")  # strut towers
    M.rounded_box(mb, (1.42, 0.0, 0.27), (0.70, 1.10, 0.03), 0.01, "underbody")          # bay floor / subframe
