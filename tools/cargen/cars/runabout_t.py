"""Runabout T: 1910s-20s open touring car (class of the Ford Model T; generic, no real model).

Design language "coachbuilt": every function is its own visible part. Tall
flat radiator in a brass shell, short bonnet with louvres that folds up from
the centre line, separate cycle wings joined by running boards, an upright
two-pane windscreen (the upper pane tilts), drum headlamps on a bar, carriage
lamps on the scuttle, a tub body with a tulip back, black gloss enamel,
wooden artillery wheels on thin cord tyres, exposed frame, axles and
transverse leaf springs. Verticals and circles; no surface flows into the next.

Length 3.40 m, width 1.70 m, height 1.80 m open / 1.95 m with the top,
wheelbase 2.54 m, track 1.42 m, tyres 30 x 3.5 in on 21 in rims
(r = 0.36 m). Two front doors, two benches.

The fabric top (node 'top', extras option = "roof", default_hidden) is a garage
upgrade option: the open car is the default.

Interactive: 4 corners, 2 doors, bonnet halves, upper windscreen pane, top
(option), steering wheel with the spark and hand-throttle levers under the
rim, 3 pedals, tall gear lever (3-speed gate), tall handbrake, key switch,
choke knob, horn bulb, speedometer and ammeter needles.
"""
import math

from cargen import controls as C
from cargen import quality as Q
from cargen import detail as D
from cargen.flow import FlowRecorder
from cargen.wheels import Wheel, corner
from cargen.common import materials, finish, seat_closed, loft_path, tube_path
from assetgen.scene import Model
from assetgen import mesh as M
from assetgen.vec import norm, compose, translate, rot_axis, xform_point, add, mul

WB, TRACK = 2.54, 1.42
XF, XR = WB / 2, -WB / 2
R = 0.36
TYRE = Wheel(radius=R, width=0.089, rim_radius=0.2667, style="artillery", tread="cord", rim_color=(0.05, 0.05, 0.05),
             disc=False, drum=True, spokes=12)
BLACK = (0.015, 0.015, 0.017)    # gloss black enamel
STEER_LOCK_DEG = 38.0
STEERING_WHEEL_LOCK_DEG = 225.0        # period steering: about a turn and a quarter (planetary reduction)
EYE_Y = 0.30                     # left-hand drive
X_DASH = 0.42                    # dash board / windscreen base
TUB_X0, TUB_X1, TUB_W = -1.28, 0.55, 0.62
FLOOR_Z = 0.60
DOOR_X0, DOOR_X1 = -0.42, 0.10


def slab_path(mb, pts, zb, zt, thick, mat, part=""):
    """Closed wall slab along a plan-view polyline pts [(x, y)], heights zb(s)/zt(s) with s = 0..1."""
    n = len(pts)
    secs = []
    for i, (x, y) in enumerate(pts):
        a = pts[max(0, i - 1)]
        b = pts[min(n - 1, i + 1)]
        tx, ty = b[0] - a[0], b[1] - a[1]
        L = math.hypot(tx, ty) or 1.0
        nx, ny = -ty / L * thick / 2, tx / L * thick / 2
        s = i / (n - 1)
        z0, z1 = zb(s), zt(s)
        secs.append([(x + nx, y + ny, z0), (x + nx, y + ny, z1), (x - nx, y - ny, z1), (x - nx, y - ny, z0)])
    loft_path(mb, secs, mat, part)


def arc_pts(cx, cy, r, a0, a1, n):
    return [(cx + r * math.cos(a0 + (a1 - a0) * k / n), cy + r * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]


def fender(mb, path_xz, y0, y1, crown, thick, mat, part=""):
    """Cycle wing: crowned sheet swept along a path in the x-z plane (path traversed so the
    outward normal is (t_z, -t_x)), closed by its thickness and end caps."""
    n = len(path_xz)
    ym, hw = (y0 + y1) / 2, (y1 - y0) / 2
    ys = [y0 + (y1 - y0) * j / 10 for j in range(11)]
    secs = []
    for i, (x, z) in enumerate(path_xz):
        a = path_xz[max(0, i - 1)]
        b = path_xz[min(n - 1, i + 1)]
        tx, tz = b[0] - a[0], b[1] - a[1]
        L = math.hypot(tx, tz) or 1.0
        nx, nz = tz / L, -tx / L
        outer = [(x + nx * crown * (1 - ((y - ym) / hw) ** 2), y, z + nz * crown * (1 - ((y - ym) / hw) ** 2))
                 for y in ys]
        inner = [(px - nx * thick, py, pz - nz * thick) for (px, py, pz) in reversed(outer)]
        # rolled edges: the inner skin is slightly narrower so the edge reads as a bead
        secs.append(outer + inner)
    M.loft(mb, secs, mat, part)


def bezier(p0, p1, p2, n):
    return [((1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * p1[0] + t * t * p2[0],
             (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * p1[1] + t * t * p2[1]) for t in (k / n for k in range(n + 1))]


def build():
    m = Model("runabout_t", "Runabout T (1910s tourer)",
              "1910s-20s open touring car, 3.40 m, 30 x 3.5 in cord tyres on wooden wheels, front 2.9 L side-valve I4.",
              "Ground plane below the wheel centres, x = 0 midway between the axles, y = 0 on the centreline.")
    materials(m, BLACK, metallic=0.0, roughness=0.12)
    m.mat("paint", color=BLACK, metallic=0.1, roughness=0.12, role="paint")
    m.mat("leather", color=(0.06, 0.04, 0.03), roughness=0.6, tex=("fabric", {"color": (0.06, 0.04, 0.03)}),
          tile=0.08)
    m.mat("floorboard", color=(0.32, 0.20, 0.10), roughness=0.7)
    m.mat("wood", color=(0.40, 0.22, 0.09), roughness=0.4)
    m.mat("canvas", color=(0.08, 0.075, 0.07), roughness=0.95, tex=("fabric", {"color": (0.08, 0.075, 0.07)}),
          tile=0.15)
    m.mat("frame", color=(0.02, 0.02, 0.022), metallic=0.3, roughness=0.5)
    m.mat("dash_board", color=(0.02, 0.02, 0.022), roughness=0.35)
    m.mat("horn_bulb", color=(0.05, 0.04, 0.035), roughness=0.8)
    flow = FlowRecorder(m)
    ck = C.Cockpit(m, STEERING_WHEEL_LOCK_DEG, STEER_LOCK_DEG)
    mb = m.mb

    # ---- chassis: frame rails, axles, transverse leaf springs, splash aprons
    for s in (1, -1):
        M.box(mb, ((XF + 0.10 + XR - 0.15) / 2, s * 0.40, 0.47), (XF - XR + 0.25, 0.05, 0.10), "frame")
    for x, z in ((XF + 0.08, 0.53), (XR - 0.12, 0.53), (0.40, 0.45)):
        M.box(mb, (x, 0.0, z), (0.08, 0.84, 0.06), "frame")                      # cross members
    for xa in (XF, XR):
        M.box(mb, (xa, 0.0, R), (0.05, TRACK - 0.12, 0.05), "frame")              # axle beams
        for k in range(5):                                                       # transverse leaf spring stack
            w = 1.00 - 0.16 * k
            M.box(mb, (xa, 0.0, R + 0.10 + 0.014 * k), (0.05, w, 0.012), "frame")
        M.box(mb, (xa, 0.0, R + 0.06), (0.06, 0.06, 0.08), "frame")
    M.box(mb, (XF - 0.05, 0.0, R + 0.02), (0.30, 0.05, 0.04), "frame")             # radius rod front
    for s in (1, -1):                                                            # radius rods (wishbones) rear
        M.tube(mb, [(XR + 0.02, s * 0.62, R), (XR + 0.80, s * 0.10, 0.42)], 0.014, "frame", count=8)
    M.tube(mb, [(XR, 0.0, R), (0.35, 0.0, 0.47)], 0.045, "frame", count=12)          # torque tube
    M.ellipsoid(mb, (XR, 0.0, R), (0.12, 0.17, 0.14), "frame", count=20, rings=10)  # rear axle pumpkin
    flow.mark("frame, axles, springs")

    # ---- wings (fenders) and running boards
    Rf = R + 0.075
    yf0, yf1 = TRACK / 2 - 0.12, TRACK / 2 + 0.12
    # front: from ahead of the wheel over the top, then sweeping down to the running board
    a_end = math.radians(118)
    pf = arc_pts(XF, R, Rf, math.radians(-12), a_end, 26)
    end_f = (XF - 0.64, 0.53)
    tan = (-math.sin(a_end), math.cos(a_end))
    ctrl = (pf[-1][0] + tan[0] * 0.30, pf[-1][1] + tan[1] * 0.30)
    pf = pf + bezier(pf[-1], ctrl, end_f, 10)[1:]
    # rear: from the running board up over the wheel to behind it
    a_start = math.radians(60)
    pr = arc_pts(XR, R, Rf, a_start, math.radians(195), 26)
    start_r = (XR + 0.62, 0.53)
    tan = (-math.sin(a_start), math.cos(a_start))
    ctrl = (pr[0][0] - tan[0] * 0.28, pr[0][1] - tan[1] * 0.28)
    pr = bezier(start_r, ctrl, pr[0], 10)[:-1] + pr
    for s in (1, -1):
        ylo, yhi = (yf0, yf1) if s > 0 else (-yf1, -yf0)
        fender(mb, pf, ylo, yhi, 0.022, 0.010, "paint")
        fender(mb, pr, ylo, yhi, 0.022, 0.010, "paint")
        # running board (wood with a black edge) and its brackets
        M.rounded_box(mb, ((end_f[0] + start_r[0]) / 2, s * (TRACK / 2 + 0.0), 0.515),
                      (end_f[0] - start_r[0] + 0.02, 0.24, 0.025), 0.006, "floorboard")
        M.rounded_box(mb, ((end_f[0] + start_r[0]) / 2, s * (TRACK / 2 + 0.12), 0.515),
                      (end_f[0] - start_r[0] + 0.02, 0.012, 0.035), 0.004, "frame")
        for xb in (end_f[0] - 0.15, start_r[0] + 0.15):
            M.box(mb, (xb, s * 0.52, 0.49), (0.04, 0.26, 0.03), "frame")
        # splash apron between the board and the frame
        M.box(mb, ((end_f[0] + start_r[0]) / 2, s * 0.50, 0.535), (end_f[0] - start_r[0], 0.18, 0.01), "paint")
    # ---- detail: beaded wing edges, ribbed running boards, tool box and carbide generator on the boards
    for s in (1, -1):
        ylo, yhi = (yf0, yf1) if s > 0 else (-yf1, -yf0)
        for path in (pf, pr):
            for yy in (ylo, yhi):
                tube_path(mb, [(x, yy, z) for (x, z) in path], 0.0075, "paint", count=8)
        for k in range(4):
            M.rounded_box(mb, ((end_f[0] + start_r[0]) / 2, s * (TRACK / 2 - 0.08 + 0.055 * k), 0.531),
                          (end_f[0] - start_r[0] - 0.08, 0.012, 0.008), 0.003, "frame")
    tb = (start_r[0] + 0.30, TRACK / 2 + 0.01, 0.61)                         # tool box, left board
    M.rounded_box(mb, tb, (0.42, 0.18, 0.17), 0.012, "paint")
    M.rounded_box(mb, (tb[0], tb[1], tb[2] + 0.09), (0.44, 0.20, 0.02), 0.006, "paint")
    for dx in (-0.12, 0.12):
        M.rounded_box(mb, (tb[0] + dx, tb[1] + 0.092, tb[2] + 0.04), (0.03, 0.008, 0.05), 0.003, "brass")
    gx = start_r[0] + 0.25                                                   # carbide gas generator, right board
    M.cylinder(mb, (gx, -(TRACK / 2 + 0.02), 0.53), (gx, -(TRACK / 2 + 0.02), 0.84), 0.065, "brass", count=20)
    M.ellipsoid(mb, (gx, -(TRACK / 2 + 0.02), 0.838), (0.0635, 0.0635, 0.03), "brass", count=20, rings=6)
    for zz in (0.58, 0.79):
        M.cylinder(mb, (gx, -(TRACK / 2 + 0.02), zz), (gx, -(TRACK / 2 + 0.02), zz + 0.012), 0.069, "frame", count=20)
    flow.mark("wings and running boards")

    # ---- radiator (brass shell, black core), filler cap, starting crank
    rx = XF + 0.02
    M.rounded_box(mb, (rx, 0.0, 1.05), (0.10, 0.62, 0.66), 0.03, "brass")
    M.rounded_box(mb, (rx + 0.046, 0.0, 1.03), (0.012, 0.50, 0.54), 0.01, "radiator")
    for k in range(23):                                                        # core tubes
        M.box(mb, (rx + 0.054, -0.23 + 0.46 * k / 22, 1.03), (0.006, 0.006, 0.50), "frame")
    for k in range(6):                                                         # cross fins
        M.box(mb, (rx + 0.0565, 0.0, 0.81 + 0.088 * k), (0.004, 0.48, 0.004), "radiator")
    M.cylinder(mb, (rx, 0.0, 1.38), (rx, 0.0, 1.43), 0.03, "brass", count=16)
    M.ellipsoid(mb, (rx, 0.0, 1.445), (0.02, 0.02, 0.025), "brass", count=12, rings=6)
    M.tube(mb, [(rx + 0.08, 0.0, 0.62), (rx + 0.20, 0.0, 0.62), (rx + 0.20, 0.0, 0.52), (rx + 0.24, 0.0, 0.52)],
           0.012, "frame", count=8)                                               # starting crank
    M.box(mb, (rx - 0.02, 0.0, 0.64), (0.06, 0.58, 0.10), "paint")                    # radiator apron
    # headlamp bar and drum headlamps
    hx = XF + 0.18
    M.tube(mb, [(hx - 0.04, -0.62, 0.86), (hx - 0.04, 0.0, 0.84), (hx - 0.04, 0.62, 0.86)], 0.014, "frame", count=8)
    for s in (1, -1):
        c = (hx, s * 0.44, 0.97)
        hxf = compose(translate(c), rot_axis((0, 1, 0), math.pi / 2))
        # open drum (the reflector sits in it) and a brass bezel ring round the lens
        M.lathe(mb, [(0.0, -0.12), (0.075, -0.11), (0.095, -0.06), (0.098, 0.0), (0.091, 0.0), (0.089, -0.05),
                     (0.0, -0.10)], "paint", axis_xf=hxf, count=24, closed_profile=True)
        M.lathe(mb, [(0.089, -0.002), (0.102, -0.002), (0.102, 0.008), (0.096, 0.014), (0.089, 0.014)], "brass",
                axis_xf=hxf, count=24, closed_profile=True)
        # silvered parabolic reflector, the lamp bulb at its focus, a clear domed lens in front
        M.lathe(mb, [(0.004, -0.085), (0.04, -0.075), (0.07, -0.045), (0.088, 0.008), (0.088, 0.011),
                     (0.084, 0.008), (0.066, -0.042), (0.037, -0.071), (0.004, -0.081)], "lamp_reflector",
                axis_xf=hxf, count=24, closed_profile=True)                     # open bowl, 4 mm wall
        if Q.detail() >= 1:
            M.ellipsoid(mb, xform_point(hxf, (0, 0, -0.045)), (0.013, 0.013, 0.02), "light_head")
            M.cylinder(mb, xform_point(hxf, (0, 0, -0.08)), xform_point(hxf, (0, 0, -0.06)), 0.006, "brass")
        M.lathe(mb, [(0.0005, 0.040), (0.030, 0.038), (0.062, 0.032), (0.086, 0.020), (0.088, 0.013),
                     (0.0005, 0.013)], "lens_clear", axis_xf=hxf, count=24, closed_profile=True)    # domed lens
        M.lathe(mb, [(0.0, 0.040), (0.012, 0.040), (0.012, 0.046), (0.0, 0.046)], "brass",
                axis_xf=compose(translate(c), rot_axis((0, 1, 0), math.pi / 2)), count=12, closed_profile=True)
        M.tube(mb, [(c[0] - 0.05, c[1], 0.86), (c[0] - 0.05, c[1], c[2] - 0.08)], 0.010, "frame", count=8)
    flow.mark("radiator, headlamps")

    # ---- bonnet halves, folding up about the centre line; scuttle
    for side, name in ((1, "hood_L"), (-1, "hood_R")):
        zt = 1.30
        m.node(name, None, (0.75, 0.0, zt + 0.01), m.revolute((side, 0, 0), 0, 150,
                                                                "bonnet half, folded up about the centre hinge"))
        secs = []
        for k in range(9):
            x = X_DASH + 0.13 + (rx - 0.05 - X_DASH - 0.13) * k / 8
            prof = []
            for j in range(9):
                a = (math.pi / 2) * j / 8
                prof.append((x, side * 0.30 * math.sin(a), 0.98 + (zt - 0.98) * (1 - (1 - math.cos(a)) ** 1.0)
                             if j < 8 else 0.98))
            prof += [(x, side * 0.30, 0.76)]
            inner = [(px, py - side * 0.01 * (abs(py) > 0.29) , pz - 0.01) for (px, py, pz) in reversed(prof)]
            secs.append(prof + inner)
        M.loft(mb, secs, "paint", name)
        for k in range(10):                                                     # louvres
            xl = X_DASH + 0.22 + 0.055 * k
            M.rounded_box(mb, (xl, side * 0.305, 0.90), (0.022, 0.012, 0.12), 0.004, "paint", name)
        M.cylinder(mb, (X_DASH + 0.40, side * 0.31, 0.80), (X_DASH + 0.40, side * 0.335, 0.80), 0.012, "brass", name,
                   count=10)                                                    # bonnet catch knob
    M.box(mb, (0.75, 0.0, 1.305), (0.62, 0.03, 0.012), "brass")                  # centre hinge strip
    # scuttle: from the bonnet's width out to the tub, carries the carriage lamps
    secs = []
    for k in range(7):
        x = X_DASH - 0.02 + 0.17 * k / 6
        w = TUB_W - (TUB_W - 0.31) * (k / 6) ** 1.5
        top = 1.24 + 0.06 * (k / 6)
        secs.append([(x, w, 0.62), (x, w, top - 0.04), (x, w * 0.6, top), (x, -w * 0.6, top), (x, -w, top - 0.04),
                     (x, -w, 0.62)])
    M.loft(mb, secs, "paint")
    for s in (1, -1):
        c = (X_DASH + 0.02, s * (TUB_W + 0.06), 1.16)
        M.rounded_box(mb, c, (0.10, 0.09, 0.15), 0.02, "paint")
        D.round_lamp(m, (c[0] + 0.052, c[1], c[2]), (1.0, 0.0, 0.0), 0.032, "light_drl", bezel_mat="brass",
                     depth=0.03)                                                # carriage lamp
        M.cylinder(mb, (c[0], c[1], c[2] + 0.075), (c[0], c[1], c[2] + 0.11), 0.016, "paint", count=10)
        M.box(mb, (c[0], s * (TUB_W + 0.01), c[2]), (0.03, 0.04, 0.03), "frame")
    flow.mark("bonnet, scuttle, carriage lamps")

    # ---- tub: rear U wall with a tulip back, front wall pieces, doors
    def tub_top(x):
        return 1.12 + 0.07 * math.exp(-((x - TUB_X0 - 0.30) / 0.35) ** 2) - 0.03 * math.exp(-((x + 0.15) / 0.3) ** 2)

    rc = 0.22
    rear = [(DOOR_X0 - 0.005, TUB_W)] + [(TUB_X0 + rc + (DOOR_X0 - 0.01 - TUB_X0 - rc) * (1 - k / 10), TUB_W)
                                          for k in range(1, 11)]
    rear += arc_pts(TUB_X0 + rc, TUB_W - rc, rc, math.pi / 2, math.pi, 8)[1:]
    rear += [(TUB_X0, TUB_W - rc - (2 * TUB_W - 2 * rc) * k / 10) for k in range(1, 11)]
    rear += arc_pts(TUB_X0 + rc, -TUB_W + rc, rc, math.pi, 1.5 * math.pi, 8)[1:]
    rear += [(TUB_X0 + rc + (DOOR_X0 - 0.005 - TUB_X0 - rc) * k / 10, -TUB_W) for k in range(1, 11)]
    slab_path(mb, rear, lambda s: FLOOR_Z - 0.04, lambda s: tub_top(rear[min(len(rear) - 1, int(s * (len(rear) - 1)))][0]),
              0.035, "paint")
    for s in (1, -1):
        front = [(DOOR_X1 + 0.005 + (X_DASH - DOOR_X1) * k / 6, s * TUB_W) for k in range(7)]
        slab_path(mb, front, lambda u: FLOOR_Z - 0.04, lambda u, f=front: tub_top(f[int(u * 6)][0]), 0.035, "paint")
    # rolled rim on the tub top (black leather bead)
    rim = [(x, y, tub_top(x) + 0.005) for (x, y) in rear]
    tube_path(mb, rim, 0.016, "leather")
    for name, side in (("door_L", 1), ("door_R", -1)):
        m.node(name, None, (DOOR_X1, side * TUB_W, 0.85), m.revolute((0, 0, -side), 0, 75, "door opening angle"))
        dpts = [(DOOR_X0 + 0.005 + (DOOR_X1 - DOOR_X0 - 0.01) * k / 8, side * TUB_W) for k in range(9)]
        slab_path(mb, dpts, lambda u: FLOOR_Z - 0.03, lambda u, d=dpts: tub_top(d[int(u * 8)][0]), 0.035, "paint",
                  name)
        M.tube(mb, [(x, y, tub_top(x) + 0.005) for (x, y) in dpts], 0.016, "leather", name, count=8)
        # outside handle and the inside door strap
        M.rounded_box(mb, (DOOR_X0 + 0.08, side * (TUB_W + 0.025), 1.02), (0.08, 0.012, 0.018), 0.005, "brass",
                      name)
        M.rounded_box(mb, (DOOR_X0 + 0.10, side * (TUB_W - 0.025), 0.95), (0.14, 0.008, 0.025), 0.004, "leather",
                      name)
    # tail: number plate and one tail lamp on the left rear wing
    D.plate_free(m, (TUB_X0 - 0.12, 0.0, 0.66), (-1.0, 0.0, 0.0), 0.30, 0.14, "A 1912", plate_mat="plate",
                 frame_mat="brass")
    M.box(mb, (TUB_X0 - 0.07, 0.0, 0.66), (0.10, 0.04, 0.03), "frame")
    tl = (XR - 0.35, 0.62, 0.86)
    M.rounded_box(mb, tl, (0.08, 0.07, 0.09), 0.015, "paint")
    M.tube(mb, [(TUB_X0 + 0.01, 0.52, 0.80), (tl[0] + 0.035, tl[1], tl[2] - 0.03)], 0.008, "frame", count=8)
    D.round_lamp(m, (tl[0] - 0.042, tl[1], tl[2] + 0.008), (-1.0, 0.0, 0.0), 0.026, "light_tail", lens_mat="lens_red",
                 bezel_mat="brass", depth=0.02)
    D.round_lamp(m, (tl[0] - 0.042, tl[1], tl[2] - 0.033), (-1.0, 0.0, 0.0), 0.013, "light_brake",
                 lens_mat="lens_red", bezel_mat="brass", depth=0.012)
    flow.mark("tub, doors, tail")

    # ---- windscreen: brass frame, fixed lower pane, tilting upper pane
    wz0, wzm, wz1, ww = 1.24, 1.50, 1.78, 0.56
    for s in (1, -1):
        M.box(mb, (X_DASH + 0.02, s * ww, (wz0 + wz1) / 2), (0.03, 0.03, wz1 - wz0 + 0.02), "brass")
        M.tube(mb, [(X_DASH + 0.02, s * ww, 1.30), (X_DASH + 0.25, s * (ww + 0.05), 1.0)], 0.008, "brass", count=6)
    M.box(mb, (X_DASH + 0.02, 0.0, wz0 + 0.01), (0.03, 2 * ww, 0.02), "brass")
    M.box(mb, (X_DASH + 0.02, 0.0, wzm), (0.03, 2 * ww - 0.03, 0.018), "brass")
    M.box(mb, (X_DASH + 0.02, 0.0, (wz0 + wzm) / 2), (0.008, 2 * ww - 0.03, wzm - wz0 - 0.02), "glass")
    m.node("windscreen_upper", None, (X_DASH + 0.02, 0.0, wz1 - 0.02),
           m.revolute((0, -1, 0), 0, 40, "upper windscreen pane, + = bottom edge tilts out forward"))
    M.box(mb, (X_DASH + 0.02, 0.0, wz1), (0.03, 2 * ww - 0.03, 0.02), "brass", "windscreen_upper")
    M.box(mb, (X_DASH + 0.02, 0.0, wzm + 0.02), (0.03, 2 * ww - 0.03, 0.016), "brass", "windscreen_upper")
    M.box(mb, (X_DASH + 0.02, 0.0, (wzm + wz1) / 2 + 0.01), (0.008, 2 * ww - 0.03, wz1 - wzm - 0.04), "glass",
          "windscreen_upper")
    # brass horn on the left wing side with its tube to the bulb
    M.lathe(mb, [(0.0, 0.0), (0.016, 0.0), (0.02, 0.20), (0.055, 0.30), (0.058, 0.31), (0.0, 0.31)], "brass",
            axis_xf=compose(translate((X_DASH + 0.15, TUB_W + 0.10, 1.05)), rot_axis((0, 1, 0), math.pi / 2)),
            count=20, closed_profile=True)
    flow.mark("windscreen, horn")

    # ---- corners
    for key, x, y, steer in (("FL", XF, TRACK / 2, True), ("FR", XF, -TRACK / 2, True),
                             ("RL", XR, TRACK / 2, False), ("RR", XR, -TRACK / 2, False)):
        corner(m, key, (x, y, R), TYRE, steer=steer, steer_limits=(-STEER_LOCK_DEG, STEER_LOCK_DEG),
               travel=(-0.07, 0.07), caliper=False)
    flow.mark("wheels")

    build_top(m)
    with Q.coarse(1.5):     # seen through glass: looser chord tolerance
        build_cabin(m, ck)
    flow.skip("cabin, cockpit controls, engine bay, hidden top")

    m.socket("socket_driver_eye", None, (-0.40, EYE_Y, 1.60), "driver eye point (camera)")
    m.socket("socket_engine_mount", None, (0.62, 0.0, 0.66),
             "engine origin (crank axis centre at the flywheel face), front longitudinal, crank along -x")
    m.socket("socket_tow_front", None, (XF + 0.10, 0.0, 0.47), "front towing point (front cross member)")
    m.socket("socket_tow_rear", None, (XR - 0.16, 0.0, 0.53), "rear towing point (rear cross member)")
    m.socket("socket_exhaust", None, (XR - 0.30, -0.30, 0.42), "exhaust outlet, flow along -x")
    M.tube(mb, [(0.60, -0.25, 0.55), (0.20, -0.30, 0.40), (XR - 0.30, -0.30, 0.42)], 0.022, "exhaust_tip", count=10)
    M.lathe(mb, [(0.021, 0.0), (0.026, 0.05), (0.034, 0.10), (0.036, 0.105), (0.031, 0.10), (0.019, 0.04),
                 (0.017, 0.0)], "exhaust_tip", axis_xf=compose(translate((XR - 0.27, -0.30, 0.418)),
                                                                rot_axis((0, 1, 0), -math.pi / 2)),
            closed_profile=True)                                        # flared tail pipe end
    m.extra_rig["engine_bay"] = {
        "envelope_mm": [700, 600, 700],
        "box_m": {"min": [0.55, -0.30, 0.58], "max": [1.25, 0.30, 1.28]},
        "origin": "socket_engine_mount", "layout": "front longitudinal",
        "note": "under the bonnet between the dash (x 0.55) and the radiator (x 1.25), above the frame rails"}
    m.notes += [
        "Generic 1910s-20s open tourer; class-typical dimensions, not a specific production car.",
        "The fabric top is node 'top' (extras option 'roof', default_hidden): a garage upgrade, open car by default.",
        "Bonnet halves hood_L/hood_R fold up about the centre hinge; the upper windscreen pane tilts.",
        "Paint material 'paint' (black enamel). Cockpit parts are driven by runabout_t.cockpit.json (rg.cockpit/1).",
    ]
    return finish(m, flow, ck,
                  "Closed visual shells per part; separate parts overlap and touch (coachbuilt), not a boolean "
                  "union. The open car is selected; the hidden top and the cabin are not.")


def build_top(m):
    """Fabric top on four bows, open sides, oval rear window; a hidden option node."""
    mb = m.mb
    m.node("top", None, (TUB_X0 + 0.30, 0.0, 1.20), option="roof", default_hidden=True, variant="roof")
    zt = 1.95
    ws = [X_DASH + 0.04 - (X_DASH + 0.04 - TUB_X0 - 0.10) * k / 14 for k in range(15)]
    secs = []
    for x in ws:
        u = (X_DASH + 0.04 - x) / (X_DASH + 0.04 - TUB_X0 - 0.10)
        ztop = zt - 0.03 * u - (0.55 * max(0.0, (u - 0.72) / 0.28) ** 1.6)
        hw = TUB_W + 0.01
        prof = [(x, hw * math.cos(math.pi * j / 10), ztop - 0.09 + 0.09 * math.sin(math.pi * j / 10)) for j in range(11)]
        inner = [(px, py * 0.985, pz - 0.012) for (px, py, pz) in reversed(prof)]
        secs.append(prof + inner)
    M.loft(mb, secs, "canvas", "top")
    for xb in (0.25, -0.25, -0.75):                                             # bows down to the tub sides
        for s in (1, -1):
            M.tube(mb, [(xb, s * (TUB_W + 0.005), zt - 0.12), (xb - 0.15, s * (TUB_W + 0.02), 1.15)], 0.012,
                   "frame", "top", count=8)
    for s in (1, -1):                                                           # straps to the windscreen
        M.tube(mb, [(X_DASH + 0.04, s * 0.55, zt - 0.10), (X_DASH + 0.05, s * 0.56, 1.80)], 0.008, "leather", "top",
               count=6)
    xr = TUB_X0 - 0.03
    M.lathe(mb, [(0.0, -0.004), (0.11, -0.004), (0.11, 0.004), (0.0, 0.004)], "glass", "top",
            axis_xf=compose(translate((xr + 0.06, 0.0, 1.62)), rot_axis((0, 1, 0), -math.pi / 2 - 0.5)), count=24,
            closed_profile=True)


def build_cabin(m, ck):
    """Floorboards, dash board, two buttoned benches, period controls."""
    mb = m.mb
    M.rounded_box(mb, ((TUB_X0 + X_DASH) / 2, 0.0, FLOOR_Z - 0.01), (X_DASH - TUB_X0 - 0.04, 2 * TUB_W - 0.04, 0.025),
                  0.004, "floorboard")
    M.box(mb, (X_DASH - 0.04, 0.0, FLOOR_Z + 0.12), (0.20, 1.0, 0.02), "floorboard")   # sloped toe board
    M.box(mb, (X_DASH + 0.035, 0.0, (FLOOR_Z - 0.03 + 0.98) / 2), (0.02, 2 * TUB_W - 0.06, 0.98 - FLOOR_Z + 0.03),
          "wood")                                                                    # firewall
    # dash board (painted plank) with coil box, speedometer, ammeter, key switch, choke
    M.rounded_box(mb, (X_DASH, 0.0, 1.10), (0.03, 2 * TUB_W - 0.06, 0.26), 0.008, "dash_board")
    M.rounded_box(mb, (X_DASH - 0.07, -0.12, 1.12), (0.12, 0.24, 0.13), 0.01, "wood")      # coil box
    for k in range(4):
        M.box(mb, (X_DASH - 0.132, -0.21 + 0.06 * k, 1.12), (0.004, 0.035, 0.09), "frame")
    gn = (-1.0, 0.0, 0.1)
    C.round_gauge(m, ck, "speed", (X_DASH - 0.018, EYE_Y - 0.02, 1.13), gn, 0.045, "speed_kmh", (0.0, 80.0),
                  sweep_deg=300.0, start_deg=240.0, ticks=9, face_mat="plate", tick_mat="frame", bezel_mat="brass",
                  label="speedometer")
    C.round_gauge(m, ck, "amps", (X_DASH - 0.018, 0.10, 1.13), gn, 0.026, "battery_v", (10.0, 15.0), sweep_deg=90.0,
                  start_deg=135.0, ticks=5, face_mat="plate", tick_mat="frame", bezel_mat="brass", label="ammeter")
    C.rotary_knob(m, ck, "key", (X_DASH - 0.135, -0.12, 1.15), (-1, 0, 0), 0.012, 0.02, "ignition", [0, 1, 2],
                  sweep_deg=60.0, mat="brass", label="key switch (off / battery / magneto)",
                  source="control:ignition")
    C.slider(m, ck, "choke", (X_DASH - 0.018, 0.22, 1.04), (-1, 0, 0), (-1.0, 0.0, 0.0), 0.05, "choke", [0, 1],
             mat="frame", knob_mat="brass", label="choke pull", up_hint=(0, 0, 1))
    # horn bulb on the driver's side of the scuttle
    C.push_button(m, ck, "horn", (X_DASH - 0.05, TUB_W - 0.06, 1.24), (-0.5, -0.4, 0.7), (0.05, 0.05, 0.05), "horn",
                  mat="horn_bulb", momentary=True, round_r=0.032, label="horn bulb", travel=0.015)
    # ---- steering: steep column, four-spoke wooden wheel, spark and hand-throttle levers under it
    col = norm((0.62, 0.0, -0.78))
    sw_c = (-0.02, EYE_Y, 1.38)
    node, f = C.steering_wheel(m, ck, sw_c, col, 0.20, 4, "wood", "wood", "frame", rim_r=0.016,
                               spoke_angles=[math.pi / 4 + k * math.pi / 2 for k in range(4)], dish=0.0,
                               column_mat="frame", column_len=0.62)
    under = add(sw_c, mul(col, 0.07))
    left = (0.0, 1.0, 0.0)
    for nm, sgn, lab, ch, src in (("spark", 1, "spark advance lever", "spark", "cosmetic:spark"),
                                  ("hand_throttle", -1, "hand throttle lever", "hand_throttle",
                                   "control:hand_throttle")):
        C.lever(m, ck, nm, under, mul(col, -1), mul(left, sgn * 0.16), ch, 50.0, mat="brass", grip_mat="frame",
                label=lab, grip_r=0.007, rod_r=0.005, grip_len=0.03, source=src)
    # ---- pedals (clutch, brake, throttle in today's order), tall gear lever, tall handbrake
    for name, yy, pc in (("clutch", EYE_Y + 0.12, (0.20, EYE_Y + 0.12, FLOOR_Z + 0.17)),
                         ("brake", EYE_Y + 0.01, (0.19, EYE_Y + 0.01, FLOOR_Z + 0.17)),
                         ("throttle", EYE_Y - 0.11, (0.22, EYE_Y - 0.11, FLOOR_Z + 0.15))):
        C.pedal(m, ck, name, (0.28, yy, FLOOR_Z + 0.04), pc, name, organ=True, label=f"{name} pedal",
                pad_size=(0.012, 0.06, 0.06))
    gate = {"R": [1, 1], "1": [1, -1], "N": [0, 0], "2": [-1, 1], "3": [-1, -1]}
    C.gear_lever(m, ck, (0.05, 0.0, FLOOR_Z + 0.02), 0.50, gate, "wood", rod_mat="frame", boot_mat=None,
                 lean_deg=7.0, knob_r=0.022, ball=True)
    C.lever(m, ck, "handbrake", (-0.10, TUB_W - 0.08, FLOOR_Z + 0.02), (0.0, 1.0, 0.0), (0.06, 0.0, 0.48),
            "handbrake", 20.0, mat="frame", grip_mat="wood", label="handbrake", grip_r=0.016)
    # ---- buttoned leather benches
    seat_closed(m, (-0.42, 0.0, FLOOR_Z + 0.34), width=1.12, recline=0.22, mat="leather", headrest=False,
                depth=0.46, height=0.16, pleats=6)
    seat_closed(m, (-1.02, 0.0, FLOOR_Z + 0.36), width=1.12, recline=0.24, mat="leather", headrest=False,
                depth=0.44, height=0.16, pleats=6)
    for xs in (-0.42, -1.02):
        for k in range(5):
            for j in range(2):
                M.ellipsoid(mb, (xs - 0.10 + 0.20 * j, -0.44 + 0.22 * k, FLOOR_Z + 0.34 + (0.02 if xs < -0.5 else 0.0)
                                 + 0.003), (0.008, 0.008, 0.006), "frame", count=8, rings=4)
    ck.notes.append("The hand throttle feeds the hand_throttle channel; the spark lever is cosmetic; pedals, gear "
                    "lever and handbrake follow the applied controls; the speedometer reads km/h.")
