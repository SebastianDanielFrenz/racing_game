"""Shared car pieces: material set, cabin shell, seats, door cards, finishing checks.

Every mesh here is a closed shell (rounded boxes, lofts with caps), so the
interior passes the same topology audit as the exterior.
"""
from . import ASSETGEN  # noqa: F401  (sys.path setup)
from assetgen import mesh as M
from assetgen.parts import standard_materials
from assetgen.vec import compose, translate, rot_axis
from assetgen.vehicles.hyper_shell import clean_degenerate
from . import quality as Q
from .normals import smooth_model


def materials(m, paint, roof=None, metallic=0.25, roughness=0.2):
    """standard_materials plus the names cargen's parts and cars use. Model.mat never redefines a name, so the
    cargen versions of shared names (lighter glass the cabin shows through) are made first."""
    m.mat("glass", color=(0.62, 0.70, 0.74), alpha=0.28, metallic=0.0, roughness=0.0, blend=True, role="glass")
    standard_materials(m)
    m.mat("paint", color=paint, metallic=metallic, roughness=roughness, role="paint")
    m.mat("paint_roof", color=roof or paint, metallic=metallic, roughness=roughness, role="paint")
    from .detail import detail_materials
    detail_materials(m)
    m.mat("glass_dark", color=(0.05, 0.06, 0.07), alpha=0.75, roughness=0.0, blend=True, role="glass")
    m.mat("cladding", color=(0.06, 0.06, 0.065), roughness=0.85, tex=("rubber", {"seed": 31, "shade": 0.06}),
          tile=0.25)
    m.mat("light_drl", color=(0.9, 0.92, 0.95), roughness=0.1, emissive=(1.0, 1.0, 1.0), light="head",
          light_strength=4.0)
    m.mat("exhaust_tip", color=(0.55, 0.55, 0.56), metallic=1.0, roughness=0.35)
    m.mat("gauge_face", color=(0.02, 0.02, 0.025), roughness=0.5)
    m.mat("gauge_tick", color=(0.92, 0.92, 0.92), roughness=0.4, emissive=(0.9, 0.9, 0.9), light="dash",
          light_strength=1.0)
    m.mat("gauge_needle", color=(0.95, 0.30, 0.05), roughness=0.4, emissive=(1.0, 0.35, 0.05), light="dash",
          light_strength=1.5)
    m.mat("wheel_rim", color=(0.07, 0.07, 0.075), roughness=0.6)
    m.mat("button", color=(0.10, 0.10, 0.11), roughness=0.55)
    m.mat("button_red", color=(0.75, 0.06, 0.05), roughness=0.4)
    m.mat("knob", color=(0.13, 0.13, 0.14), metallic=0.3, roughness=0.45)
    m.mat("boot", color=(0.05, 0.05, 0.05), roughness=0.9)
    m.mat("pedal_rubber", color=(0.06, 0.06, 0.06), roughness=0.95)
    m.mat("radiator", color=(0.12, 0.12, 0.13), metallic=0.5, roughness=0.6)
    m.mat("battery", color=(0.08, 0.08, 0.09), roughness=0.7)
    m.mat("brake_drum", color=(0.30, 0.29, 0.28), metallic=0.7, roughness=0.6)
    m.mat("brass", color=(0.78, 0.60, 0.25), metallic=1.0, roughness=0.3)
    m.mat("headliner", color=(0.55, 0.55, 0.53), roughness=0.95)
    m.mat("carpet", color=(0.07, 0.07, 0.075), roughness=1.0)


def cabin_shell(m, body, x_front, x_rear, floor_z, half_w, roof_drop=0.03, thickness=0.02):
    """Carpeted floor slab and a headliner slab under the roof, both closed."""
    mb = m.mb
    M.rounded_box(mb, ((x_front + x_rear) / 2, 0.0, floor_z - thickness / 2),
                  (x_front - x_rear, 2 * half_w, thickness), 0.008, "carpet")
    g = body.green
    zmax = max(g["zroof"](x) for x in [x_rear + k * (x_front - x_rear) / 40 for k in range(41)])
    xs = []
    n = 24
    for k in range(n + 1):
        x = x_rear + 0.04 + (x_front - x_rear - 0.04) * k / n
        if g["zroof"](x) < zmax - 0.07:
            if xs:
                break
            continue
        xs.append(x)
    secs = []
    for x in xs:
        z = g["zroof"](x) - 0.018 - roof_drop
        w = max(0.1, min(half_w, g["wr"](x) - 0.07))
        loop = [(x, w, z), (x, -w, z), (x, -w, z - thickness), (x, w, z - thickness)]
        secs.append(loop)
    if len(secs) >= 2:
        M.loft(mb, secs, "headliner")


def seat_closed(m, pos, width=0.5, recline=0.3, mat="seat", accent=None, headrest=True, headrest_split=1,
                part="", depth=0.5, height=0.12, bolster=0.0, pleats=0, bolster_mat=None):
    """Seat as closed rounded boxes: cushion, backrest, headrest(s); optional side bolsters (`bolster` = how far
    they rise above the cushion / stand proud of the backrest) and `pleats` (rolled pleats across a backrest).
    pos = cushion centre top."""
    mb = m.mb
    x, y, z = pos
    M.rounded_box(mb, (x, y, z - height / 2), (depth, width, height), 0.04, mat, part)
    if accent:
        M.rounded_box(mb, (x + 0.02, y, z + 0.004), (depth * 0.6, width * 0.3, 0.012), 0.005, accent, part)
    bx = x - depth / 2
    back_h = 0.58
    xf = compose(translate((bx, y, z)), rot_axis((0, 1, 0), -recline), translate((0.0, 0, back_h / 2)))
    M.rounded_box(mb, (0, 0, 0), (0.11, width, back_h), 0.04, mat, part, xf=xf)
    if accent:
        M.rounded_box(mb, (0.062, 0, 0.04), (0.012, width * 0.3, back_h * 0.6), 0.005, accent, part, xf=xf)
    if bolster > 0:
        bw = 0.085
        bm = bolster_mat or mat
        for sy in (-1, 1):
            yb = sy * (width / 2 - bw / 2)
            M.rounded_box(mb, (x + 0.01, y + yb, z - height / 2 + bolster / 2), (depth * 0.94, bw, height + bolster),
                          0.035, bm, part)
            M.rounded_box(mb, (0.02 + bolster / 2, yb, -0.03), (0.11 + bolster, bw, back_h * 0.80), 0.035, bm, part,
                          xf=xf)
        # shoulder roll across the backrest top
        M.rounded_box(mb, (0.015, 0, back_h / 2 - 0.05), (0.12, width - 2 * bw + 0.02, 0.07), 0.03, mat, part, xf=xf)
    if pleats > 0:
        pw = (width - 0.06) / pleats
        for i in range(pleats):
            yy = -width / 2 + 0.03 + pw * (i + 0.5)
            M.rounded_box(mb, (0.06, yy, -0.02), (0.035, pw * 0.92, back_h * 0.78), 0.015, mat, part, xf=xf)
    if headrest:
        k = max(1, headrest_split)
        hw = width * (0.5 if k == 1 else 0.7 / k)
        for i in range(k):
            yy = 0.0 if k == 1 else (i - (k - 1) / 2) * width / k
            xf2 = compose(translate((bx, y + yy, z)), rot_axis((0, 1, 0), -recline),
                          translate((0.0, 0, back_h + 0.12)))
            M.rounded_box(mb, (0, 0, 0), (0.1, hw, 0.16), 0.04, mat, part, xf=xf2)


def door_card(m, part, side, x0, x1, z0, z1, y, mat="interior", top_mat=None, bottle=False):
    """Inner door trim on the door's own node: card, armrest, pull handle, map pocket."""
    mb = m.mb
    yc = side * y
    xc = (x0 + x1) / 2
    M.rounded_box(mb, (xc, yc, (z0 + z1) / 2), (x1 - x0, 0.03, z1 - z0), 0.012, mat, part)
    if top_mat:
        M.rounded_box(mb, (xc, yc - side * 0.008, z1 - 0.03), (x1 - x0 - 0.02, 0.026, 0.05), 0.01, top_mat, part)
    M.rounded_box(mb, (xc - 0.05, yc - side * 0.04, z0 + 0.30), ((x1 - x0) * 0.5, 0.06, 0.035), 0.012, mat, part)
    M.rounded_box(mb, (xc + 0.06, yc - side * 0.035, z0 + 0.36), (0.12, 0.03, 0.025), 0.01, "trim_black", part)
    if bottle:
        M.rounded_box(mb, (xc, yc - side * 0.04, z0 + 0.08), ((x1 - x0) * 0.75, 0.05, 0.12), 0.012, mat, part)


CABIN_MATS = {"seat", "seat_accent", "dash_hard", "dash_top", "dash_paint", "carpet", "headliner", "gauge_face",
              "gauge_tick", "gauge_needle", "button", "button_red", "knob", "boot", "pedal_rubber", "interior"}


def strip_cabin(m, ck):
    """Far LOD: drop the cabin furniture and every cockpit control's geometry (their nodes stay, empty)."""
    ctl = {c["node"] for c in ck.controls} | {s_["node"] for s_ in ck.screens}
    changed = True
    while changed:                                   # descendants of control nodes (horn on the wheel, ...)
        changed = False
        for name, nd in m.nodes.items():
            if nd.parent is not None and nd.parent.name in ctl and name not in ctl:
                ctl.add(name)
                changed = True
    parts = {m.world_part(n) for n in ctl if n in m.nodes}
    for (part, mat), p in m.mb.prims.items():
        if part in parts or mat in CABIN_MATS or mat.startswith("screen_"):
            p.pos, p.nrm, p.uv, p.idx = [], [], [], []


def finish(m, flow, ck, limitations):
    """Final checks and sidecars: nothing degenerate left, flow selection and cockpit attached."""
    removed = clean_degenerate(m.mb)
    if removed and flow._touches_marked(removed):
        raise ValueError("final degenerate cleanup reached the flow selection")
    if Q.detail() == 0:
        strip_cabin(m, ck)
    smooth_model(m.mb)
    m.flow_selection = flow.selection(limitations)
    m.cockpit = ck.as_json()
    return m


def top_z(body, x, y):
    """z of the lower body's upper surface at (x, y) (superellipse section, ignores arches)."""
    W, zb, zt, n = body.lower_params(x)
    zc, H = (zt + zb) / 2, (zt - zb) / 2
    t = min(1.0, abs(y) / W)
    return zc + H * (1 - t ** n) ** (1 / n)


def surface_path(body, xs, z_of_x, offset=0.0, side=1):
    """Points on the body side at heights z_of_x(x), pushed out by `offset` (for creases, trims)."""
    return [(x, side * (body.surface_y(x, z_of_x(x)) + offset), z_of_x(x)) for x in xs]


def loft_path(mb, secs, mat, part=""):
    """M.loft for sections along a bending path (a U-shaped wall or rim): the caps face along the
    path's own end tangents, not along the start-to-end chord, which can be perpendicular to them."""
    def cen(s):
        return tuple(sum(p[k] for p in s) / len(s) for k in range(3))

    def sub(a, b):
        return tuple(a[k] - b[k] for k in range(3))

    t0 = sub(cen(secs[1]), cen(secs[0]))
    t1 = sub(cen(secs[-1]), cen(secs[-2]))
    nw = M.newell(secs[0])
    if sum(nw[k] * t0[k] for k in range(3)) > 0:
        secs = [list(reversed(s)) for s in secs]
    recs = mb.grid(secs, mat, part, closed_v=True)
    recs += mb.polygon(secs[0], mat, part, want_normal=tuple(-c for c in t0))
    recs += mb.polygon(secs[-1], mat, part, want_normal=t1)
    mb.orient_outward(recs)
    return recs


def tube_path(mb, path, radius, mat, part="", count=8):
    """M.tube with loft_path's caps (for rims and beads that bend through more than 90 deg)."""
    import math

    def nrm(v):
        L = math.sqrt(sum(c * c for c in v)) or 1.0
        return tuple(c / L for c in v)

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    count = Q.count_for(radius, count)
    n = len(path)
    tans = []
    for i in range(n):
        a, b = path[max(0, i - 1)], path[min(n - 1, i + 1)]
        tans.append(nrm(tuple(b[k] - a[k] for k in range(3))))
    ref = (0.0, 0.0, 1.0) if abs(tans[0][2]) < 0.9 else (1.0, 0.0, 0.0)
    ex = nrm(cross(ref, tans[0]))
    secs = []
    for p, t in zip(path, tans):
        d = sum(ex[k] * t[k] for k in range(3))
        ex = nrm(tuple(ex[k] - t[k] * d for k in range(3)))
        ey = cross(t, ex)
        secs.append([tuple(p[k] + radius * (ex[k] * math.cos(2 * math.pi * j / count) +
                                            ey[k] * math.sin(2 * math.pi * j / count)) for k in range(3))
                     for j in range(count)])
    return loft_path(mb, secs, mat, part)


def arch_lip(m, body, w, side, mat, width=0.04, proud=0.012, a0=-8.0, a1=188.0, part_of=None, n=None):
    """Wheel-arch lip / flare: a band of `width` around the arch edge standing `proud` off the body side.

    w: one of body.wheels; angles in degrees from +x (forward) toward up. part_of(x) -> node name ("" = body) or
    None (a shut line: no lip there); consecutive samples on the same node make one closed, tapered loft, so a
    flare crossing onto a door is split at the shut line and the door's piece opens with the door."""
    import math
    ra = w["r"] + body.arch_clear
    n = n or max(6, int((a1 - a0) / 360.0 * Q.seg(ra, lo=24)))
    pieces, cur, cur_part = [], [], None
    for k in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * k / n)
        xa, xb = sorted((w["x"] + (ra - 0.004) * math.cos(a), w["x"] + (ra + width) * math.cos(a)))
        part = "" if part_of is None else part_of(xa, xb)
        if part != cur_part or part is None:
            if len(cur) >= 2:
                pieces.append((cur_part, cur))
            cur = []
        cur_part = part
        if part is not None:
            cur.append(a)
    if len(cur) >= 2:
        pieces.append((cur_part, cur))
    for part, angs in pieces:
        secs = []
        for i, a in enumerate(angs):
            c, s = math.cos(a), math.sin(a)
            end = 1.0 if 0 < i < len(angs) - 1 else 0.55          # tapered ends
            loop = []
            for (dr, off) in ((-0.003, -0.006), (width * end, -0.006), (width * end, proud * 0.35 * end),
                              (width * 0.45 * end, proud * end), (0.004, proud * 0.85 * end)):
                r = ra + dr
                x, z = w["x"] + r * c, w["z"] + r * s
                y = body.surface_y(x, max(z, body.zb(x) + 0.01))
                loop.append((x, side * (y + off), z))
            secs.append(loop)
        loft_path(m.mb, secs, mat, part)


def door_part_of(doors, side, margin=0.012):
    """part_of for arch_lip: the door node whose x range holds the band, None on a shut line, else the body."""
    def f(xa, xb):
        for name, s, x0, x1 in doors:
            if s != side:
                continue
            for q in (x0, x1):
                if xa - margin < q < xb + margin:
                    return None
            if x0 < xa and xb < x1:
                return name
        return ""
    return f


def blade_path(mb, path, width, thick, mat, part="", normal=(0.0, 0.0, 1.0)):
    """Flat blade (elliptical section, `width` across, `thick` along `normal`) along a polyline; width/thick may be
    lists per point. For steering-wheel spokes, grille slats."""
    import math

    def nrm(v):
        L = math.sqrt(sum(c * c for c in v)) or 1.0
        return tuple(c / L for c in v)

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    n = len(path)
    ws = width if isinstance(width, (list, tuple)) else [width] * n
    ts = thick if isinstance(thick, (list, tuple)) else [thick] * n
    secs = []
    for i, p in enumerate(path):
        a, b = path[max(0, i - 1)], path[min(n - 1, i + 1)]
        t = nrm(tuple(b[k] - a[k] for k in range(3)))
        ex = nrm(cross(normal, t))
        ey = cross(t, ex)
        nb = (10, 8, 4)[Q.level()]
        secs.append([tuple(p[k] + ws[i] / 2 * math.cos(2 * math.pi * j / nb) * ex[k] +
                           ts[i] / 2 * math.sin(2 * math.pi * j / nb) * ey[k] for k in range(3)) for j in range(nb)])
    return loft_path(mb, secs, mat, part)
