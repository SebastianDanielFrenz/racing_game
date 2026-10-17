"""Closed-shell wheels and corners for the cargen cars.

Node names follow physics_sim's convention (data/models/README.md):
susp_XX (prismatic, travel) -> steer_XX (revolute, front only) -> wheel_XX
(continuous spin about +y). Every part of a wheel is a closed lathe or loft,
so the wheel nodes pass the closed-shell audit (assetgen's steel rim and
hubcap are open sheets, hence this module).

Rim styles: "steel_cover" (pressed steel wheel behind a plastic full cover,
city car), "polished5" (five-spoke polished alloy with a chrome lip, 70s GT),
"artillery" (wooden spokes and felloe on a steel hub, 1910s).
"""
import math

from assetgen import mesh as M
from assetgen.vec import frame_z_to, xform_point, compose, translate, rot_axis
from . import quality as Q


class Wheel:
    def __init__(self, radius, width, rim_radius, style, rim_color=(0.75, 0.76, 0.78), tread="road",
                 disc=True, drum=False, caliper_color=(0.2, 0.2, 0.22), spokes=5, tread_grooves=4,
                 whitewall=False):
        self.radius, self.width, self.rim_radius = radius, width, rim_radius
        self.style, self.rim_color, self.tread = style, rim_color, tread
        self.disc, self.drum, self.caliper_color = disc, drum, caliper_color
        self.spokes, self.tread_grooves, self.whitewall = spokes, tread_grooves, whitewall


def tyre_profile(R, W, Rr, kind, grooves=0, block_depth=0.0):
    """Closed tyre cross-section (radius, axial) from the outer bead round to the inner bead.

    Returns (profile, c0, c1, walls): the crown (tread material) is profile[c0:c1]; `walls` holds the indices i
    whose segment i-1 -> i is a groove wall (radial), which the tread UV skips so the texture's grooves sit over
    the geometric ones. Circumferential grooves are cut where the tread texture draws them (tread_car: groove g at
    (g + 1) / (grooves + 1) of the crown arc, 5.6 % of it wide), on the flat part of the crown only.
    block_depth > 0: the flat crown is a groove floor `block_depth` below R (tread_blocks() stands the blocks on
    it), with a radial wall down from each shoulder; the grooves are then the gaps between the block lanes."""
    hw = W / 2
    sh = min(0.35 * (R - Rr), 0.3 * W if kind != "cord" else 0.48 * W)
    bulge = 0.04 * W if kind != "cord" else 0.10 * W
    prof = []
    walls = set()
    ns = 6
    for k in range(ns + 1):
        t = k / ns
        prof.append((Rr + (R - sh - Rr) * t, hw * 0.92 + bulge * math.sin(math.pi * t)))
        if k == 1:   # rim protector rib just above the bead
            prof.append((Rr + (R - sh - Rr) * 0.22, hw * 0.92 + bulge * math.sin(math.pi * 0.22) + 0.004))
    c0 = len(prof)
    for k in range(1, 5):
        t = k / 4 * math.pi / 2
        prof.append((R - sh + sh * math.sin(t), hw - sh + sh * math.cos(t)))
    flat = hw - sh
    crown_w = 2 * flat + math.pi * sh
    gw = 0.056 * crown_w
    depth = min(0.008, 0.25 * (R - Rr))
    cuts = []
    for g in range(grooves):
        s = crown_w * ((g + 1) / (grooves + 1) - 0.5)   # arc offset from the crown centre (outer side first)
        if abs(s) + gw / 2 < flat - 0.004:
            cuts.append(-s)                             # axial position: the crown runs from +flat to -flat
    cuts.sort(reverse=True)
    stops = [flat - 2 * flat * k / 4 for k in range(1, 4)]
    for a in cuts:
        stops = [q for q in stops if abs(q - a) > gw / 2 + 0.003]
    events = sorted([(q, "p") for q in stops] + [(a, "g") for a in cuts], key=lambda e: -e[0])
    if block_depth > 0:
        Rf = R - block_depth
        prof.append((Rf, flat - 0.0005))
        walls.add(len(prof) - 1)
        for q in (flat * 0.5, 0.0, -flat * 0.5):
            prof.append((Rf, q))
        prof.append((Rf, -flat + 0.0005))
        events = []
    for a, kind_ in events:
        if kind_ == "p":
            prof.append((R, a))
            continue
        prof.append((R, a + gw / 2))
        prof.append((R - depth, a + gw / 2 - 0.0005))
        walls.add(len(prof) - 1)
        prof.append((R - depth, a - gw / 2 + 0.0005))
        prof.append((R, a - gw / 2))
        walls.add(len(prof) - 1)
    for k in range(0, 5):
        t = k / 4 * math.pi / 2
        prof.append((R - sh + sh * math.cos(t), -(hw - sh) - sh * math.sin(t)))
        if k == 0 and block_depth > 0:
            walls.add(len(prof) - 1)
    c1 = len(prof) - 1
    for k in range(1, ns + 1):
        t = k / ns
        if k == ns:   # rim protector rib on the inner side too
            prof.append((Rr + (R - sh - Rr) * 0.22, -(hw * 0.92 + bulge * math.sin(math.pi * 0.22) + 0.004)))
        prof.append((R - sh - (R - sh - Rr) * t, -(hw * 0.92 + bulge * math.sin(math.pi * (1 - t)))))
    # inner closure across the bead seat (hidden inside the rim barrel)
    prof.append((Rr - 0.004, -hw * 0.6))
    prof.append((Rr - 0.004, hw * 0.6))
    return prof, c0, c1, walls, cuts, gw, flat


def build_wheel(model, part, center, spec, side, tag):
    mb = model.mb
    R, W, Rr = spec.radius, spec.width, spec.rim_radius
    xf = frame_z_to((0.0, float(side), 0.0), center)
    blocks = spec.tread != "cord" and Q.detail() >= 2
    bd = min(0.007, 0.2 * (R - Rr)) if blocks else 0.0
    prof, c0, c1, walls, cuts, gw, flat = tyre_profile(R, W, Rr, spec.tread, spec.tread_grooves
                                                        if spec.tread != "cord" else 3, block_depth=bd)
    count = Q.seg(R)
    circ = 2 * math.pi * R
    tread_mat = f"tyre_tread_{tag}"
    side_mat = "tyre_sidewall"
    model.mat(side_mat, color=(1, 1, 1), tex=("rubber", {}), tile=0.4)
    if spec.tread == "cord":
        model.mat(tread_mat, color=(1, 1, 1), tex=("tread_car", {"grooves": 3}), tile=(W * 0.9, circ / 90))
    else:
        crown_w = sum(math.dist(prof[i], prof[i + 1]) for i in range(c0, c1) if i + 1 not in walls)
        if blocks:
            model.mat(tread_mat, color=(0.75, 0.75, 0.75), tex=("rubber", {"seed": 7}), tile=0.15)
        else:
            model.mat(tread_mat, color=(1, 1, 1), tex=("tread_car", {"grooves": spec.tread_grooves}),
                      tile=(crown_w, circ / max(1, round(circ / 0.03))))
    arc = [0.0]
    for i in range(1, len(prof)):
        arc.append(arc[-1] + (0.0 if i in walls else math.dist(prof[i - 1], prof[i])))
    arc.append(arc[-1] + math.dist(prof[-1], prof[0]))
    a0 = arc[c0]
    ww = spec.whitewall

    def uv(i, j):
        return (arc[min(i, len(arc) - 1)] - a0, j / count * circ)

    def cls(i, j, c, n):
        if c0 <= i < c1:
            return tread_mat
        if ww and i < c0 - 1 and i >= 2:
            return "tyre_whitewall"
        return side_mat

    if ww:
        model.mat("tyre_whitewall", color=(0.86, 0.85, 0.80), roughness=0.7)
    if blocks:
        model.mat("tyre_groove", color=(0.035, 0.035, 0.037), roughness=0.95)

        def cls_b(i, j, c, n):
            if c0 + 3 <= i <= c0 + 8:
                return "tyre_groove"
            return cls(i, j, c, n)
        M.lathe(mb, prof, tread_mat, part, xf, count=count, classify=cls_b, orient=False, uv=uv,
                closed_profile=True)
        tread_blocks(mb, xf, R, bd, flat, cuts, gw, tread_mat, part, side)
    else:
        M.lathe(mb, prof, tread_mat, part, xf, count=count, classify=cls, orient=False, uv=uv, closed_profile=True)
    if Q.detail() >= 2 and spec.tread != "cord":
        sidewall_lettering(mb, xf, R, W, Rr, side_mat, part)

    rim_mat = f"rim_{tag}"
    model.mat(rim_mat, color=spec.rim_color, metallic=0.9 if spec.style != "artillery" else 0.0,
              roughness=0.25 if spec.style != "artillery" else 0.6)
    hw = W / 2 * 0.9
    if spec.style != "artillery":
        lip = Rr * 1.05
        barrel = [(lip, hw), (Rr, hw - 0.01), (Rr * 0.97, hw * 0.6), (Rr * 0.97, -hw * 0.6), (Rr, -hw + 0.01),
                  (lip, -hw), (Rr * 0.94, -hw + 0.004), (Rr * 0.94, hw - 0.004)]
        bm = "chrome" if spec.style == "polished5" else rim_mat
        M.lathe(mb, barrel, bm, part, xf, count=48, orient=None, closed_profile=True)
    face = hw * 0.35
    if spec.style == "steel_cover":
        _steel_cover(model, part, xf, spec, rim_mat, face)
    elif spec.style == "polished5":
        _polished5(model, part, xf, spec, rim_mat, face)
    elif spec.style == "artillery":
        _artillery(model, part, xf, spec, rim_mat)
    else:
        raise ValueError(spec.style)
    with Q.coarse(2.0):    # brakes sit behind the spokes
        _brakes(model, part, xf, spec)


def _brakes(model, part, xf, spec):
    mb = model.mb
    Rr = spec.rim_radius
    if spec.disc:
        # ventilated disc: two friction rings with the vane gap showing at the edge, a hat to the hub face
        dr = Rr * 0.82
        ri = dr * 0.55
        M.lathe(mb, [(dr, -0.006), (ri, -0.006), (ri, -0.034), (dr, -0.034), (dr, -0.025), (dr - 0.006, -0.0235),
                     (dr - 0.006, -0.0165), (dr, -0.015)], "brake_disc", part, xf, count=40, closed_profile=True)
        model.mat("brake_hat", color=(0.22, 0.22, 0.23), metallic=0.8, roughness=0.5)
        M.lathe(mb, [(ri + 0.004, -0.005), (ri * 0.55, -0.004), (ri * 0.55, 0.004), (ri + 0.004, -0.001)],
                "brake_hat", part, xf, count=28, closed_profile=True)
        model.mat("brake_vane", color=(0.08, 0.08, 0.085), roughness=0.9)
        M.lathe(mb, [(dr - 0.0055, -0.0168), (ri + 0.004, -0.0168), (ri + 0.004, -0.0232), (dr - 0.0055, -0.0232)],
                "brake_vane", part, xf, count=40, closed_profile=True)
        if Q.detail() >= 2:
            # curved cooling slots machined into the outboard friction face
            model.mat("brake_slot", color=(0.06, 0.06, 0.065), metallic=0.5, roughness=0.8)
            rs = (dr + ri) / 2
            for k in range(8):
                a = 2 * math.pi * k / 8
                x = compose(xf, rot_axis((0, 0, 1), a), translate((rs, 0.0, -0.0058)), rot_axis((0, 0, 1), 0.5))
                M.box(mb, (0, 0, 0), ((dr - ri) * 0.62, 0.0035, 0.0016), "brake_slot", part, xf=x)
    if spec.drum:
        dr = Rr * 0.78
        M.lathe(mb, [(dr, -0.002), (dr * 0.3, -0.002), (dr * 0.3, -0.07), (dr, -0.07), (dr + 0.006, -0.064),
                     (dr + 0.006, -0.050), (dr, -0.046), (dr, -0.030), (dr + 0.006, -0.026), (dr + 0.006, -0.012),
                     (dr, -0.008)], "brake_drum", part, xf, count=32, closed_profile=True)   # cooling fins


def tread_blocks(mb, xf, R, depth, flat, cuts, gw, mat, part, side, pitch=0.044):
    """Directional tread: closed block prisms standing on the groove floor, one lane between each pair of
    circumferential grooves; lateral sipes between pitches, lanes offset by half a pitch, blocks skewed into a
    shallow V (mirrored left/right so both sides of the car roll the same way)."""
    edges = [flat] + [x for a in cuts for x in (a + gw / 2, a - gw / 2)] + [-flat]
    lanes = [(edges[k], edges[k + 1]) for k in range(0, len(edges), 2)]
    n = max(12, int(round(2 * math.pi * R / pitch)))
    dth = 2 * math.pi / n
    gap = min(0.45, 0.0055 / (R * dth))            # 5.5 mm lateral groove, as a fraction of the pitch
    r0, r1 = R - depth - 0.0015, R
    for li, (a0, a1) in enumerate(lanes):
        if a0 - a1 < 0.006:
            continue
        stag = 0.5 * (li % 2)

        def skew(a):
            return 0.35 * dth * (abs(a) / max(flat, 1e-3)) * side

        for k in range(n):
            t0 = (k + stag) * dth
            t1 = t0 + dth * (1 - gap)
            quad = []
            for (t, a) in ((t0 + skew(a0), a0), (t1 + skew(a0), a0), (t1 + skew(a1), a1), (t0 + skew(a1), a1)):
                quad.append((math.cos(t), math.sin(t), a))
            bot = [xform_point(xf, (r0 * c, r0 * s_, a)) for (c, s_, a) in quad]
            top = [xform_point(xf, (r1 * c, r1 * s_, a)) for (c, s_, a) in quad]
            M.loft(mb, [bot, top], mat, part)


def sidewall_lettering(mb, xf, R, W, Rr, mat, part):
    """Raised moulded lettering bands on the outer sidewall: two arcs of glyph-sized bumps plus a size code,
    0.6 mm proud. Reads as tyre lettering in a close-up; nothing a texture could not do, but it catches light."""
    hw = W / 2
    sh = min(0.35 * (R - Rr), 0.3 * W)
    rm = Rr + 0.47 * (R - Rr)
    tt = (rm - Rr) / (R - sh - Rr)
    z_side = hw * 0.92 + 0.04 * W * math.sin(math.pi * tt)    # tyre_profile's sidewall at rm
    glyphs = [(0.0, "BRAND"), (math.pi, "BRAND"), (math.pi / 2, "SIZE"), (-math.pi / 2, "SIZE")]
    pat = {"BRAND": [3, 2, 3, 3, 2, 3, 0, 2, 3, 3], "SIZE": [2, 2, 2, 0, 1, 2, 0, 2, 2]}
    for t0, kind in glyphs:
        ws = pat[kind]
        h = 0.012 if kind == "BRAND" else 0.0075
        step = (0.011 if kind == "BRAND" else 0.007) / rm
        t = t0 - step * len(ws) / 2
        for wdt in ws:
            if wdt:
                th = step * (0.35 + 0.18 * wdt)
                z = z_side
                loop = [(rm - h / 2, t), (rm - h / 2, t + th), (rm + h / 2, t + th), (rm + h / 2, t)]
                bot = [xform_point(xf, (r * math.cos(a), r * math.sin(a), z - 0.004)) for r, a in loop]
                top = [xform_point(xf, (r * math.cos(a), r * math.sin(a), z + 0.0006)) for r, a in loop]
                M.loft(mb, [bot, top], mat, part)
            t += step


def _steel_cover(model, part, xf, spec, rim_mat, face):
    """Steel wheel centre (closed disc) and a plastic full cover with vent recesses."""
    mb = model.mb
    Rr = spec.rim_radius
    M.lathe(mb, [(Rr * 0.95, face - 0.022), (Rr * 0.95, face - 0.010), (0.004, face - 0.006), (0.004, face - 0.018)],
            rim_mat, part, xf, count=40, closed_profile=True)
    model.mat("wheel_cover", color=(0.62, 0.64, 0.66), metallic=0.35, roughness=0.4)
    cov = [(Rr * 1.04, face + 0.0), (Rr * 1.00, face + 0.014), (Rr * 0.75, face + 0.024), (Rr * 0.35, face + 0.030),
           (Rr * 0.12, face + 0.034), (0.004, face + 0.035), (0.004, face - 0.004), (Rr * 0.98, face - 0.004)]
    M.lathe(mb, cov, "wheel_cover", part, xf, count=56, closed_profile=True)
    # dark vent recesses: small rounded boxes standing proud of the cover between the spokes of its pattern
    model.mat("wheel_cover_vent", color=(0.05, 0.05, 0.055), roughness=0.8)
    n = 8
    for k in range(n):
        a = 2 * math.pi * k / n + math.pi / n
        r = Rr * 0.62
        c = (r * math.cos(a), r * math.sin(a), face + 0.029)
        x = compose(xf, translate(c), rot_axis((0, 0, 1), a))
        M.rounded_box(mb, (0, 0, 0), (Rr * 0.38, Rr * 0.11, 0.006), 0.008, "wheel_cover_vent", part, xf=x, seg=2)
    model.mat("wheel_badge", color=(0.15, 0.16, 0.18), metallic=0.5, roughness=0.3)
    M.cylinder(mb, xform_point(xf, (0, 0, face + 0.032)), xform_point(xf, (0, 0, face + 0.039)), Rr * 0.13,
               "wheel_badge", part, count=20)


def _polished5(model, part, xf, spec, rim_mat, face):
    mb = model.mb
    Rr = spec.rim_radius
    hub_r = Rr * 0.24
    M.lathe(mb, [(hub_r * 1.45, face - 0.012), (hub_r * 1.35, face + 0.022), (hub_r, face + 0.032),
                 (0.004, face + 0.034), (0.004, face - 0.012)], rim_mat, part, xf, count=32, closed_profile=True)
    n = spec.spokes
    for k in range(n):
        a = 2 * math.pi * k / n + math.pi / 2
        c, s = math.cos(a), math.sin(a)
        t = (-s, c, 0.0)
        secs = []
        sc = Rr / 0.19
        for i in range(9):
            f = i / 8
            r = hub_r * 1.25 + (Rr * 0.965 - hub_r * 1.25) * f
            # taper toward the rim, then flare smoothly into the rim flange (last third)
            flare = max(0.0, (f - 0.66) / 0.34) ** 2
            w = (0.050 - 0.016 * f + 0.030 * flare) * sc
            z = face + 0.018 - 0.026 * f * f
            ctr = (r * c, r * s, z)
            ridge = 0.016 - 0.004 * f   # raised centre ridge: a faceted, cast look
            sec = [xform_point(xf, (ctr[0] + t[0] * u, ctr[1] + t[1] * u, ctr[2] + v))
                   for (u, v) in ((-w / 2, -0.014), (w / 2, -0.014), (w / 2 * 0.85, 0.008), (w * 0.12, ridge),
                                  (-w * 0.12, ridge), (-w / 2 * 0.85, 0.008))]
            secs.append(sec)
        M.loft(mb, secs, rim_mat, part)
    M.lathe(mb, [(Rr * 0.985, face + 0.004), (Rr * 0.90, face - 0.002), (Rr * 0.90, face - 0.014),
                 (Rr * 0.985, face - 0.012)], rim_mat, part, xf, count=48, closed_profile=True)
    model.mat("wheel_nut", color=(0.85, 0.85, 0.86), metallic=1.0, roughness=0.2)
    for k in range(4):
        a = 2 * math.pi * k / 4 + math.pi / 4
        p = xform_point(xf, (hub_r * 0.62 * math.cos(a), hub_r * 0.62 * math.sin(a), face + 0.030))
        q = xform_point(xf, (hub_r * 0.62 * math.cos(a), hub_r * 0.62 * math.sin(a), face + 0.046))
        M.cylinder(mb, p, q, 0.0085, "wheel_nut", part, count=6)
    M.cylinder(mb, xform_point(xf, (0, 0, face + 0.030)), xform_point(xf, (0, 0, face + 0.040)), hub_r * 0.42,
               "chrome", part, count=18)


def _artillery(model, part, xf, spec, rim_mat):
    """Wooden artillery wheel: steel clincher rim, wooden felloe, 12 tapered spokes, steel hub."""
    mb = model.mb
    Rr = spec.rim_radius
    W = spec.width
    model.mat("wheel_wood", color=(0.42, 0.24, 0.11), roughness=0.45)
    # steel clincher band (closed)
    M.lathe(mb, [(Rr + 0.006, W * 0.42), (Rr - 0.004, W * 0.40), (Rr - 0.004, -W * 0.40), (Rr + 0.006, -W * 0.42),
                 (Rr - 0.012, -W * 0.30), (Rr - 0.012, W * 0.30)], rim_mat, part, xf, count=48, closed_profile=True)
    # felloe
    fr0, fr1 = Rr - 0.012, Rr - 0.055
    M.lathe(mb, [(fr0, 0.028), (fr1, 0.026), (fr1, -0.026), (fr0, -0.028)], "wheel_wood", part, xf, count=48,
            closed_profile=True)
    hub_r = 0.075
    n = spec.spokes
    for k in range(n):
        a = 2 * math.pi * k / n
        c, s = math.cos(a), math.sin(a)
        t = (-s, c, 0.0)
        secs = []
        for i in range(5):
            f = i / 4
            r = hub_r * 0.9 + (fr1 + 0.004 - hub_r * 0.9) * f
            w = 0.046 - 0.016 * f
            d = 0.050 - 0.016 * f
            ctr = (r * c, r * s, 0.004 * (1 - f))
            secs.append([xform_point(xf, (ctr[0] + t[0] * u, ctr[1] + t[1] * u, ctr[2] + v))
                         for (u, v) in ((-w / 2, -d / 2), (w / 2, -d / 2), (w / 2, d / 2), (-w / 2, d / 2))])
        M.loft(mb, secs, "wheel_wood", part)
    model.mat("wheel_hub", color=(0.08, 0.08, 0.085), metallic=0.6, roughness=0.4)
    M.lathe(mb, [(hub_r, 0.045), (hub_r, -0.045), (0.004, -0.05), (0.004, 0.05)], "wheel_hub", part, xf, count=28,
            closed_profile=True)
    M.lathe(mb, [(0.045, 0.045), (0.042, 0.085), (0.030, 0.10), (0.004, 0.102), (0.004, 0.044)], "brass", part, xf,
            count=24, closed_profile=True)
    for k in range(6):
        a = 2 * math.pi * k / 6 + 0.26
        p = xform_point(xf, (0.058 * math.cos(a), 0.058 * math.sin(a), 0.044))
        q = xform_point(xf, (0.058 * math.cos(a), 0.058 * math.sin(a), 0.056))
        M.cylinder(mb, p, q, 0.0075, "wheel_hub", part, count=6)


def _caliper(model, part, center, spec, side):
    """Two-piston style caliper: a curved body straddling the disc edge over ~55 deg at the rear-top of the disc,
    with a bridge over the outside and two piston bosses on the outboard face."""
    cm = "caliper"
    model.mat(cm, color=spec.caliper_color, metallic=0.3, roughness=0.4)
    mb = model.mb
    xf = frame_z_to((0.0, float(side), 0.0), center)
    dr = spec.rim_radius * 0.82
    a0, a1 = math.radians(98), math.radians(152)
    # frame_z_to((0, side, 0)): local z = outward, local x = -side * world x, local y = world z
    n = 7
    r_in, r_out = dr * 0.70, dr + 0.012
    z_in, z_out = -0.046, 0.002
    secs = []
    for i in range(n):
        a = a0 + (a1 - a0) * i / (n - 1)
        c, s_ = -side * math.cos(a), math.sin(a)   # a measured in the world: from +x (forward) toward up
        taper = 0.004 * abs(2 * i / (n - 1) - 1)   # rounded ends
        loop = [(r_in + taper, z_in + taper), (r_out - taper, z_in + taper), (r_out - taper, z_out - taper),
                (r_in + taper, z_out - taper)]
        secs.append([xform_point(xf, (r * c, r * s_, z)) for r, z in loop])
    from .common import loft_path
    loft_path(mb, secs, cm, part)
    for k in (0.3, 0.7):
        a = a0 + (a1 - a0) * k
        p = (-side * dr * 0.84 * math.cos(a), dr * 0.84 * math.sin(a))
        M.cylinder(mb, xform_point(xf, (p[0], p[1], z_out - 0.004)), xform_point(xf, (p[0], p[1], z_out + 0.006)),
                   0.016 * dr / 0.15, cm, part, count=14)


def corner(model, key, center, spec, steer=False, steer_limits=(-35, 35), travel=(-0.08, 0.08), caliper=True,
           kingpin_offset=0.0):
    side = 1 if center[1] > 0 else -1
    p = model.node(f"susp_{key}", None, center,
                   model.prismatic((0, 0, 1), travel[0], travel[1], "wheel travel (bump +)"))
    carrier = p
    if steer:
        kp = (center[0], center[1] - side * kingpin_offset, center[2])
        carrier = model.node(f"steer_{key}", p, kp,
                             model.revolute((0, 0, 1), steer_limits[0], steer_limits[1],
                                            "road-wheel steer angle, + = left"))
    w = model.node(f"wheel_{key}", carrier, center, model.continuous((0, 1, 0), "wheel spin, + = rolling forward"),
                   wheel={"radius": spec.radius, "width": spec.width, "side": "left" if side > 0 else "right"})
    build_wheel(model, w, center, spec, side, key[0])
    if caliper and spec.disc:
        _caliper(model, carrier, center, spec, side)
        # dust shield behind the disc (on the carrier: it steers but does not spin)
        model.mat("dust_shield", color=(0.10, 0.10, 0.105), metallic=0.4, roughness=0.7)
        xf = frame_z_to((0.0, float(side), 0.0), center)
        dr = spec.rim_radius * 0.82
        with Q.coarse(3.0):
            M.lathe(model.mb, [(dr * 1.04, -0.040), (dr * 0.45, -0.046), (dr * 0.45, -0.050),
                               (dr * 1.06, -0.044)], "dust_shield", carrier, xf, count=32, closed_profile=True)
    return w
