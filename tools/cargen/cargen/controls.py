"""Cockpit parts as rig nodes, plus the rg.cockpit/1 sidecar that drives them.

Every moving cockpit part (steering wheel, pedal, lever, needle, switch,
button, knob, stalk) is its own node with one joint and its own pivot, so a
press, flick or turn is one joint value. Static parts (gauge faces, bezels,
switch plates) go on the node the caller names (usually the body, "").

The sidecar (`Cockpit.as_json`) says for each moving node what drives it
(`source`, `map`) and, for clickable parts, a hit box in the node's own frame
(`hit`: centre relative to the node pivot plus half extents, ISO axes at
rest; the runtime moves it with the node) and what a click/drag does
(`action`). Screens are flat panels with their own material and UV 0..1 on the
display face.

Sources: `wheel_steer_angle` (physics road-wheel angle, rad), `control:<ch>`
(applied control channel), `gear`, `engine_rpm`, `speed_kmh`, `fuel_fraction`,
`coolant_c`, `oil_pressure_bar`, `battery_v`, `clock_h`/`clock_m`, and
`cosmetic:<name>` (local state the cockpit owns: hazard, wipers, lights, ...).

All geometry is closed (boxes, rounded boxes, cylinders, tubes), so every
node passes the closed-shell audit.
"""
import math

from assetgen import mesh as M
from assetgen.vec import add, sub, mul, norm, cross, dot, compose, translate, rot_axis, frame_z_to, xform_point

COCKPIT_FORMAT = "rg.cockpit/1"


def basis(normal, up_hint=(0.0, 0.0, 1.0)):
    """(right, up, normal) for a panel facing `normal` (pointing at the driver)."""
    n = norm(normal)
    r = norm(cross(up_hint, n))
    if r == (0.0, 0.0, 0.0):
        r = norm(cross((1.0, 0.0, 0.0), n))
    u = cross(n, r)
    return r, u, n


def frame(center, normal, up_hint=(0.0, 0.0, 1.0)):
    """Local->model transform: x = right, y = up, z = normal (towards the driver)."""
    r, u, n = basis(normal, up_hint)
    return ((r[0], u[0], n[0], center[0]), (r[1], u[1], n[1], center[1]), (r[2], u[2], n[2], center[2]))


def local(f, p):
    return xform_point(f, p)


def _r(v, k=4):
    return [round(float(x), k) for x in v]


class Cockpit:
    def __init__(self, model, steering_lock_to_lock_turns, road_wheel_lock_deg):
        self.model = model
        self.controls = []
        self.screens = []
        self.notes = []
        self.steering = {"lock_to_lock_turns": steering_lock_to_lock_turns,
                         "road_wheel_lock_deg": road_wheel_lock_deg}

    def add(self, node, kind, source, mapping, hit=None, action=None, label=""):
        nd = self.model.nodes[node]
        entry = {"node": node, "kind": kind, "label": label or node, "source": source, "map": mapping,
                 "joint": nd.joint["type"] if nd.joint else None}
        if hit is not None:
            c, h = hit
            entry["hit"] = {"center": _r(sub(c, nd.pivot)), "half_extents": _r(h)}
        if action is not None:
            entry["action"] = action
        self.controls.append(entry)
        return entry

    def screen(self, node, material, px, role, size_m, hit=None):
        entry = {"node": node, "material": material, "px": list(px), "uv_rect": [0, 0, 1, 1],
                 "size_m": _r(size_m), "role": role, "content": "off (later task)"}
        if hit is not None:
            nd = self.model.nodes[node]
            c, h = hit
            entry["hit"] = {"center": _r(sub(c, nd.pivot)), "half_extents": _r(h)}
            entry["action"] = {"type": "touch", "screen": node}
        self.screens.append(entry)

    def as_json(self):
        return {"format": COCKPIT_FORMAT, "model": self.model.id + ".glb",
                "frame": "ISO 8855 model frame (x forward, y left, z up), rest pose; hit centres relative to the "
                         "node pivot, axes as at rest, moved with the node at runtime",
                "steering": self.steering, "controls": self.controls, "screens": self.screens,
                "notes": self.notes}


# ------------------------------------------------------------------ helpers

def box_local(mb, f, center, size, mat, part):
    """Box in a panel frame (centre and size in local x/y/z)."""
    return M.box(mb, (0.0, 0.0, 0.0), size, mat, part, xf=compose(f, translate(center)))


def rbox_local(mb, f, center, size, r, mat, part, seg=2):
    return M.rounded_box(mb, (0.0, 0.0, 0.0), size, r, mat, part, xf=compose(f, translate(center)), seg=seg)


def disk(mb, f, z0, z1, r, mat, part, count=28, center=(0.0, 0.0)):
    return M.cylinder(mb, local(f, (center[0], center[1], z0)), local(f, (center[0], center[1], z1)), r, mat, part,
                      count=count)


# ------------------------------------------------------------------ gauges

def round_gauge(m, ck, name, center, normal, radius, source, in_range, sweep_deg=270.0, start_deg=225.0,
                ticks=None, face_mat="gauge_face", tick_mat="gauge_tick", needle_mat="gauge_needle",
                bezel_mat="chrome", part="", cowl_mat=None, label="", cowl_depth=0.0, up_hint=(0, 0, 1)):
    """Round analogue gauge. Static face + ticks + bezel on `part`; needle on node needle_<name>.

    start_deg: needle angle at in_range[0], measured counter-clockwise from 'right' as the driver
    sees the face (225 = down-left). Positive joint values sweep clockwise.
    """
    mb = m.mb
    f = frame(center, normal, up_hint)
    disk(mb, f, -0.006, 0.0, radius, face_mat, part, count=36)
    # bezel ring: a thin tube around the face
    ring = [local(f, (radius * math.cos(2 * math.pi * k / 36), radius * math.sin(2 * math.pi * k / 36), 0.002))
            for k in range(36)]
    M.tube(mb, ring, radius * 0.07 + 0.002, bezel_mat, part, count=8, closed=True)
    if cowl_mat:
        depth = cowl_depth or radius * 0.6
        disk(mb, f, -0.02, -0.007, radius * 1.18, cowl_mat, part, count=36)
        hood = [local(f, ((radius * 1.12) * math.cos(math.pi * (0.05 + 0.9 * k / 12)),
                          (radius * 1.12) * math.sin(math.pi * (0.05 + 0.9 * k / 12)), depth * 0.5))
                for k in range(13)]
        M.tube(mb, hood, radius * 0.09, cowl_mat, part, count=8)
    n_ticks = ticks if ticks is not None else 9
    for k in range(n_ticks):
        a = math.radians(start_deg - sweep_deg * k / max(1, n_ticks - 1))
        major = k % 2 == 0
        rl = radius * (0.80 if major else 0.86)
        c = ((rl + radius * 0.92) / 2 * math.cos(a), (rl + radius * 0.92) / 2 * math.sin(a), 0.0008)
        L = radius * 0.92 - rl
        xf = compose(f, translate(c), rot_axis((0, 0, 1), a))
        M.box(mb, (0, 0, 0), (L, 0.0022 if major else 0.0014, 0.0012), tick_mat, part, xf=xf)
    # needle node
    pivot = local(f, (0.0, 0.0, 0.004))
    axis = mul(norm(normal), -1)  # + = clockwise as seen by the driver
    node = m.node(f"needle_{name}", None if not part else part, pivot,
                  m.revolute(axis, 0.0, sweep_deg, f"{label or name} needle, + = clockwise as the driver sees it"))
    a0 = math.radians(start_deg)
    L = radius * 0.86
    xf = compose(f, translate((0.0, 0.0, 0.004)), rot_axis((0, 0, 1), a0))
    M.box(mb, (L * 0.42, 0.0, 0.0), (L * 1.1, 0.003, 0.0016), needle_mat, node, xf=xf)
    M.cylinder(mb, local(f, (0, 0, 0.0025)), local(f, (0, 0, 0.0075)), radius * 0.09, "trim_black", node, count=14)
    ck.add(node, "needle", source, {"in": list(in_range), "out_deg": [0.0, sweep_deg]}, label=label or name)
    return node


# ------------------------------------------------------------------ pedals

def pedal(m, ck, name, pivot, pad_center, channel, travel_deg=18.0, pad_size=(0.012, 0.07, 0.085),
          arm_mat="trim_black", pad_mat="pedal_rubber", organ=False, part=None, label=""):
    """Pedal on node pedal_<name>, revolute about +y (lateral). + = pressed (pad moves forward/down).

    pivot: arm pivot (top, under the dash, or floor hinge for an organ pedal).
    """
    mb = m.mb
    axis = (0.0, -1.0, 0.0) if not organ else (0.0, 1.0, 0.0)
    node = m.node(f"pedal_{name}", part, pivot, m.revolute(axis, 0.0, travel_deg, f"{label or name} pedal travel"))
    if not organ:
        M.tube(mb, [pivot, add(pad_center, (0.0, 0.0, pad_size[2] * 0.3))], 0.008, arm_mat, node, count=8)
    else:
        M.tube(mb, [pivot, add(pivot, (0.01, 0.0, 0.02))], 0.008, arm_mat, node, count=8)
    # pad tilted to face the driver's foot (normal ~ -x, slightly up)
    tilt = math.radians(-30.0 if not organ else -55.0)
    xf = compose(translate(pad_center), rot_axis((0, 1, 0), tilt))
    M.rounded_box(mb, (0, 0, 0), pad_size, 0.005, pad_mat, node, xf=xf, seg=2)
    hit = (pad_center, (0.04, pad_size[1] / 2 + 0.01, pad_size[2] / 2 + 0.01))
    ck.add(node, "pedal", f"control:{channel}", {"in": [0.0, 1.0], "out_deg": [0.0, travel_deg]}, hit=hit,
           action={"type": "drag", "channel": channel, "range": [0.0, 1.0], "drag_m": 0.10, "release_to": 0.0},
           label=label or name)
    return node


# ---------------------------------------------------------- steering wheel

def steering_wheel(m, ck, center, column_dir, radius, spokes, rim_mat, spoke_mat, hub_mat, rim_r=0.016,
                   lock_deg=None, part=None, spoke_angles=None, dish=0.0, hub_r=None, column_mat=None,
                   column_len=0.32):
    """Steering wheel node 'steering_wheel' about the column (+ = steer left, CCW as the driver sees it).

    dish: how far the hub sits forward of the rim plane (deep-dish 70s wheels ~0.05)."""
    mb = m.mb
    axis = norm(mul(column_dir, -1))
    lock = lock_deg if lock_deg is not None else ck.steering["lock_to_lock_turns"] * 180.0
    node = m.node("steering_wheel", part, center, m.revolute(axis, -lock, lock, "steering wheel angle, + = left"))
    f = frame_z_to(axis, center)
    rim = [xform_point(f, (radius * math.cos(2 * math.pi * k / 40), radius * math.sin(2 * math.pi * k / 40), 0))
           for k in range(40)]
    M.tube(mb, rim, rim_r, rim_mat, node, count=10, closed=True)
    hub = xform_point(f, (0, 0, -dish - 0.02))
    angles = spoke_angles or [-math.pi / 2 + 2 * math.pi * k / spokes for k in range(spokes)]
    for a in angles:
        tip = xform_point(f, (radius * 0.97 * math.cos(a), radius * 0.97 * math.sin(a), 0))
        mid = xform_point(f, (radius * 0.55 * math.cos(a), radius * 0.55 * math.sin(a), -dish * 0.45))
        M.tube(mb, [hub, mid, tip], 0.009, spoke_mat, node, count=8)
    hr = hub_r or radius * 0.28
    M.cylinder(mb, xform_point(f, (0, 0, -dish - 0.06)), xform_point(f, (0, 0, -dish + 0.005)), hr, hub_mat, node,
               count=20)
    col_end = add(center, mul(column_dir, column_len))
    M.cylinder(mb, xform_point(f, (0, 0, -dish - 0.06)), col_end, 0.028, column_mat or "trim_black",
               "" if part is None else part, count=12)
    ck.add(node, "steering_wheel", "wheel_steer_angle",
           {"ratio": "lock_to_lock", "lock_to_lock_turns": ck.steering["lock_to_lock_turns"],
            "road_wheel_lock_deg": ck.steering["road_wheel_lock_deg"]},
           hit=(center, (radius * 0.3, radius + rim_r, radius + rim_r)),
           action={"type": "rotate", "channel": "steer", "range": [-1.0, 1.0],
                   "turns_full_range": ck.steering["lock_to_lock_turns"]}, label="steering wheel")
    return node, f


# ------------------------------------------------------- switches & knobs

def toggle_switch(m, ck, name, base, normal, channel, up_hint=(0, 0, 1), throw_deg=30.0, lever_len=0.022,
                  mat="chrome", base_mat="trim_black", part="", label="", source=None):
    """Bat toggle switch: lever node sw_<name> rotates about the panel's right axis (+ = up = on)."""
    mb = m.mb
    f = frame(base, normal, up_hint)
    disk(mb, f, -0.002, 0.004, 0.007, base_mat, part, count=12)
    r, u, n = basis(normal, up_hint)
    pivot = local(f, (0, 0, 0.004))
    node = m.node(f"sw_{name}", part or None, pivot, m.revolute(mul(r, -1), -throw_deg / 2, throw_deg / 2,
                                                                  f"{label or name} toggle, + = up/on"))
    tip = local(f, (0, 0, 0.004 + lever_len))
    M.tube(mb, [pivot, tip], [0.0022, 0.0018], mat, node, count=8)
    M.cylinder(mb, sub(tip, mul(n, 0.002)), add(tip, mul(n, 0.003)), 0.0035, mat, node, count=10)
    ck.add(node, "toggle", source or f"cosmetic:{channel}", {"off_deg": -throw_deg / 2, "on_deg": throw_deg / 2},
           hit=(local(f, (0, 0, 0.004 + lever_len / 2)), (0.012, 0.012, lever_len / 2 + 0.006)),
           action={"type": "toggle", "channel": channel}, label=label or name)
    return node


def push_button(m, ck, name, center, normal, size, channel, mat="button", cap_mat=None, up_hint=(0, 0, 1),
                travel=0.003, part="", label="", momentary=False, round_r=None, source=None, symbol_mat=None):
    """Push button node btn_<name>, prismatic along -normal (+ = pressed)."""
    mb = m.mb
    f = frame(center, normal, up_hint)
    n = norm(normal)
    node = m.node(f"btn_{name}", part or None, center, m.prismatic(mul(n, -1), 0.0, travel,
                                                                    f"{label or name} button, + = pressed"))
    if round_r:
        disk(mb, f, 0.0, size[2], round_r, mat, node, count=18)
    else:
        rbox_local(mb, f, (0, 0, size[2] / 2), size, min(size[0], size[1]) * 0.2, mat, node)
    if symbol_mat:
        box_local(mb, f, (0, 0, size[2] + 0.0004), (size[0] * 0.45, size[1] * 0.18, 0.0008), symbol_mat, node)
    hw = (round_r or max(size[0], size[1]) / 2) + 0.003
    ck.add(node, "button", source or f"cosmetic:{channel}", {"pressed_m": travel},
           hit=(local(f, (0, 0, size[2] / 2)), (hw, hw, size[2] / 2 + 0.004)),
           action={"type": "press" if momentary else "toggle", "channel": channel}, label=label or name)
    return node


def rotary_knob(m, ck, name, center, normal, radius, depth, channel, detents, sweep_deg=270.0, mat="knob",
                mark_mat="gauge_tick", up_hint=(0, 0, 1), part="", label="", source=None):
    """Rotary knob node knob_<name> about the panel normal (+ = clockwise as seen)."""
    mb = m.mb
    f = frame(center, normal, up_hint)
    axis = mul(norm(normal), -1)
    node = m.node(f"knob_{name}", part or None, center, m.revolute(axis, 0.0, sweep_deg,
                                                                    f"{label or name} knob, + = clockwise"))
    disk(mb, f, 0.0, depth, radius, mat, node, count=24)
    # pointer line at the start position (pointing down-left like a dial at 0)
    a0 = math.radians(225.0)
    xf = compose(f, translate((radius * 0.55 * math.cos(a0), radius * 0.55 * math.sin(a0), depth + 0.0004)),
                 rot_axis((0, 0, 1), a0))
    M.box(mb, (0, 0, 0), (radius * 0.7, 0.0018, 0.0008), mark_mat, node, xf=xf)
    ck.add(node, "knob", source or f"cosmetic:{channel}",
           {"detents": detents, "out_deg": [0.0, sweep_deg]},
           hit=(local(f, (0, 0, depth / 2)), (radius + 0.004, radius + 0.004, depth / 2 + 0.004)),
           action={"type": "step", "channel": channel, "values": detents}, label=label or name)
    return node


def stalk(m, ck, name, pivot, direction, length, channel, positions, mat="trim_black", tip_mat=None,
          part=None, label="", axis=(0.0, 1.0, 0.0), throw_deg=12.0):
    """Column stalk node stalk_<name>: a rod from the column, revolute about `axis` (+ = up/push)."""
    mb = m.mb
    node = m.node(f"stalk_{name}", part, pivot, m.revolute(norm(axis), -throw_deg, throw_deg,
                                                           f"{label or name} stalk"))
    tip = add(pivot, mul(norm(direction), length))
    M.tube(mb, [pivot, add(pivot, mul(norm(direction), length * 0.6)), tip], [0.006, 0.006, 0.008], mat, node,
           count=8)
    M.cylinder(mb, add(tip, mul(norm(direction), -0.012)), add(tip, mul(norm(direction), 0.014)), 0.0095,
               tip_mat or mat, node, count=12)
    ck.add(node, "stalk", f"cosmetic:{channel}", {"positions": positions, "throw_deg": throw_deg},
           hit=(tip, (0.025, 0.025, 0.025)), action={"type": "step", "channel": channel, "values": positions},
           label=label or name)
    return node


# ------------------------------------------------------------ gear lever

def gear_lever(m, ck, base, height, gate, knob_mat, rod_mat="chrome", boot_mat=None, lean_deg=8.0,
               knob_r=0.022, ball=True, part=None, label="gear lever", reverse_note=""):
    """Two-axis gear lever: node lever_gear_fa (fore/aft, revolute about +y... see below) carrying
    node lever_gear (side to side). gate: {gear: [side, fore]} with side +1 = left, fore +1 = forward.

    lever_gear_fa: axis (0, 1, 0), + = top moves forward. lever_gear: axis (1, 0, 0) after fa,
    + = top moves left. The sidecar maps the selected gear to the two joint values."""
    mb = m.mb
    fa = m.node("lever_gear_fa", part, base, m.revolute((0.0, 1.0, 0.0), -lean_deg * 1.5, lean_deg * 1.5,
                                                         "gear lever fore/aft, + = top forward"))
    sd = m.node("lever_gear", fa, base, m.revolute((-1.0, 0.0, 0.0), -lean_deg * 1.5, lean_deg * 1.5,
                                                    "gear lever across the gate, + = top left"))
    top = add(base, (0.0, 0.0, height))
    M.tube(mb, [base, add(base, (0.0, 0.0, height * 0.5)), top], [0.009, 0.0075, 0.0065], rod_mat, sd, count=10)
    if ball:
        M.ellipsoid(mb, add(top, (0.0, 0.0, knob_r * 0.6)), (knob_r, knob_r, knob_r * 0.95), knob_mat, sd,
                    count=18, rings=10)
    else:
        M.rounded_box(mb, add(top, (0.0, 0.0, knob_r)), (knob_r * 1.7, knob_r * 1.5, knob_r * 2.2), knob_r * 0.5,
                      knob_mat, sd)
    if boot_mat:
        M.lathe(mb, [(0.0, 0.0), (0.06, 0.0), (0.05, 0.03), (0.03, 0.065), (0.012, 0.09), (0.0, 0.09)], boot_mat,
                part or "", axis_xf=translate(base), count=16, closed_profile=False)
    mapping = {"gate": {str(k): v for k, v in gate.items()}, "deg_per_unit_side": lean_deg,
               "deg_per_unit_fore": lean_deg, "fa_node": "lever_gear_fa"}
    ck.add(fa, "lever_axis", "gear", {"follows": "lever_gear"})
    ck.add(sd, "gear_lever", "gear", mapping, hit=(add(top, (0, 0, knob_r * 0.6)), (0.04, 0.04, 0.04)),
           action={"type": "gear", "gate": mapping["gate"]}, label=label)
    return fa, sd


def lever(m, ck, name, pivot, axis, length_vec, channel, travel_deg, mat="trim_black", grip_mat=None, part=None,
          label="", grip_r=0.016, rod_r=0.008, source=None, grip_len=0.09):
    """Single-axis hand lever (handbrake, hand throttle, spark): node lever_<name>, + = pulled."""
    mb = m.mb
    node = m.node(f"lever_{name}", part, pivot, m.revolute(norm(axis), 0.0, travel_deg,
                                                           f"{label or name} lever, + = pulled/on"))
    tip = add(pivot, length_vec)
    M.tube(mb, [pivot, tip], rod_r, mat, node, count=8)
    d = norm(length_vec)
    M.cylinder(mb, sub(tip, mul(d, grip_len)), add(tip, mul(d, 0.004)), grip_r, grip_mat or mat, node, count=12)
    ck.add(node, "lever", source or f"control:{channel}", {"in": [0.0, 1.0], "out_deg": [0.0, travel_deg]},
           hit=(sub(tip, mul(d, grip_len / 2)), (0.04, 0.04, 0.04)),
           action={"type": "drag", "channel": channel, "range": [0.0, 1.0], "drag_m": 0.12}, label=label or name)
    return node


# ---------------------------------------------------------------- screens

def screen(m, ck, name, center, normal, size, px, role, bezel_mat="trim_black", up_hint=(0, 0, 1), part="",
           bezel=0.008, depth=0.012):
    """Display: bezel slab on `part` plus the display slab on node screen_<name> with material
    screen_<name> whose front face carries UV 0..1 (u right, v down from the top-left corner)."""
    mb = m.mb
    w, h = size
    f = frame(center, normal, up_hint)
    rbox_local(mb, f, (0, 0, -depth / 2), (w + 2 * bezel, h + 2 * bezel, depth), 0.006, bezel_mat, part)
    node = m.node(f"screen_{name}", part or None, local(f, (0, 0, 0)))
    mat = m.mat(f"screen_{name}", color=(0.015, 0.018, 0.022), roughness=0.12, tile=1.0,
                emissive=(0.0, 0.0, 0.0))
    # front face as a 2x2 grid with explicit UVs; the other five faces close the slab
    z1, z0 = 0.0012, -0.0004
    P = [[local(f, (-w / 2 + w * i, h / 2 - h * j, z1)) for j in (0, 1)] for i in (0, 1)]
    mb.grid(P, mat, node, uv=lambda i, j: (float(i), float(j)), flip=True)
    c = [local(f, (sx * w / 2, sy * h / 2, z0)) for sx, sy in ((-1, 1), (1, 1), (1, -1), (-1, -1))]
    t = [local(f, (sx * w / 2, sy * h / 2, z1)) for sx, sy in ((-1, 1), (1, 1), (1, -1), (-1, -1))]
    mb.polygon(c, "trim_black", node, want_normal=mul(norm(normal), -1))
    for k in range(4):
        a, b = k, (k + 1) % 4
        out = sub(add(c[a], c[b]), add(c[(k + 2) % 4], c[(k + 3) % 4]))
        mb.polygon([c[a], c[b], t[b], t[a]], "trim_black", node, want_normal=out)
    ck.screen(node, mat, px, role, size, hit=(local(f, (0, 0, z1)), (w / 2, h / 2, 0.01)))
    return node
