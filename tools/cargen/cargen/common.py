"""Shared car pieces: material set, cabin shell, seats, door cards, finishing checks.

Every mesh here is a closed shell (rounded boxes, lofts with caps), so the
interior passes the same topology audit as the exterior.
"""
from . import ASSETGEN  # noqa: F401  (sys.path setup)
from assetgen import mesh as M
from assetgen.parts import standard_materials
from assetgen.vec import compose, translate, rot_axis
from assetgen.vehicles.hyper_shell import clean_degenerate


def materials(m, paint, roof=None):
    """standard_materials plus the names cargen's parts and cars use."""
    standard_materials(m)
    m.mat("paint", color=paint, metallic=0.25, roughness=0.32, role="paint")
    m.mat("paint_roof", color=roof or paint, metallic=0.25, roughness=0.32, role="paint")
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
                part="", depth=0.5, height=0.12):
    """Seat as closed rounded boxes: cushion with side bolsters, backrest, headrest(s).
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


def finish(m, flow, ck, limitations):
    """Final checks and sidecars: nothing degenerate left, flow selection and cockpit attached."""
    removed = clean_degenerate(m.mb)
    if removed and flow._touches_marked(removed):
        raise ValueError("final degenerate cleanup reached the flow selection")
    m.flow_selection = flow.selection(limitations)
    m.cockpit = ck.as_json()
    return m
