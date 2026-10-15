#!/usr/bin/env python3
"""Import a downloaded CC BY car model as a rigged game model (data/models/imported/<id>/).

    python3 tools/carimport/carimport.py [ids...]     # every car in tools/carimport/cars.json by default
    python3 tools/carimport/carimport.py --check [ids...]   # only re-run the checks on the written files

Per car (tools/carimport/cars.json):
  - source GLB from data/models/downloaded/ -> data/models/imported/<id>/<id>.glb + <id>.rig.json
    (physics_sim.rig/1: ISO x fwd, y left, z up in the rig; the glb in glTF axes gltf(x,y,z) = iso(y,z,x),
    metres, origin on the ground below the wheel centres, x = 0 midway between the axles);
  - the four wheels found by node names, else by geometric clustering (analyze.find_wheels), each moved under
    susp_<C> / steer_<C> (front) / wheel_<C> with every pivot at the wheel's centre; brake parts (calipers)
    under steer_<C> / susp_<C> so they steer but do not spin;
  - baked ground/shadow planes dropped; materials, textures, buffers kept unchanged;
  - attribution + source sha256 in asset.copyright / asset.extras;
  - with a "vehicle" block: the game vehicle file, a copy of the base file with the wheel positions and tyre
    size matched to the model (the wheelbase keeps the base car's centre-of-gravity fraction).
Checks (exit 1 on failure): 4 wheels, every pivot within 1 cm of the wheel's own bbox centre in the written
file, round wheels, wheel bottoms on the ground, attribution present, tyre radius within 1.5 cm of the model's.
The modification record for CC BY goes into data/models/downloaded/attributions.json ("derived" list).
"""
import argparse
import copy
import datetime
import hashlib
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze  # noqa: E402
import glb  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SRC_DIR = os.path.join(REPO, "data", "models", "downloaded")
OUT_DIR = os.path.join(REPO, "data", "models", "imported")
CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cars.json")
ATTRIB = os.path.join(SRC_DIR, "attributions.json")

# ISO -> glTF game axes: gltf(x, y, z) = iso(y, z, x)
P = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], float)
PIVOT_TOL_M = 0.01
TYRE_TOL_M = 0.015
FRONT = ("FL", "FR")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def col_major(m):
    return [float(v) for v in np.asarray(m, float).T.reshape(-1)]


def part_points(part):
    node, sel = part
    if sel is None:
        return node.points()
    pi, tri_idx = sel
    p, t, _ = node.prims[pi]
    return p[np.unique(t[tri_idx].reshape(-1))]


def tyre_text(ts):
    one = lambda s: f"{s['compound']} {s['width_mm']}/{s['aspect']} R{s['rim_in']}"  # noqa: E731
    return f"front {one(ts['front'])}, rear {one(ts['rear'])}" if "front" in ts else one(ts)


def tyre_radius_m(spec):
    return (spec["rim_in"] * 25.4 / 2.0 + spec["width_mm"] * spec["aspect"] / 100.0) / 1000.0


class Builder:
    """Accumulates the output glTF: the source json with a new node tree and extra index accessors."""

    def __init__(self, js, bin_):
        self.js = copy.deepcopy(js)
        self.bin = bytearray(bin_)
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)
        return len(self.nodes) - 1

    def index_accessor(self, idx):
        while len(self.bin) % 4:
            self.bin.append(0)
        data = np.asarray(idx, np.uint32).reshape(-1).tobytes()
        self.js["bufferViews"].append({"buffer": 0, "byteOffset": len(self.bin), "byteLength": len(data),
                                       "target": 34963})
        self.bin += data
        self.js["accessors"].append({"bufferView": len(self.js["bufferViews"]) - 1, "componentType": 5125,
                                     "count": int(len(idx) * 3 if np.ndim(idx) == 2 else len(idx)),
                                     "type": "SCALAR"})
        return len(self.js["accessors"]) - 1

    def sub_mesh(self, mesh_index, prim_tris, name):
        """A new mesh holding only the given triangles of each listed primitive ({prim index: Mx3 vertex
        indices}); the vertex accessors are shared with the source mesh."""
        src = self.js["meshes"][mesh_index]
        prims = []
        for pi, tris in sorted(prim_tris.items()):
            if len(tris) == 0:
                continue
            p = copy.deepcopy(src["primitives"][pi])
            p["indices"] = self.index_accessor(tris)
            prims.append(p)
        if not prims:
            return None
        self.js["meshes"].append({"name": name, "primitives": prims})
        return len(self.js["meshes"]) - 1

    def finish(self, root, asset):
        js = self.js
        js["nodes"] = self.nodes
        js["scenes"] = [{"name": "Scene", "nodes": [root]}]
        js["scene"] = 0
        js.pop("animations", None)
        js.pop("skins", None)
        js.pop("cameras", None)
        js["asset"] = asset
        js["buffers"][0]["byteLength"] = len(self.bin)
        return js, bytes(self.bin)


LIGHT_KINDS = ("head", "tail", "brake", "indicator_left", "indicator_right", "reverse")


def _in_range(v, rng):
    return np.ones(len(v), bool) if rng is None else (v >= rng[0]) & (v <= rng[1])


def split_lamps(b, car, n, mesh_index, iso_of, lamp_mats):
    """Moves the lamp triangles of one body mesh onto their own primitives with a light_<kind> material copy.

    cars.json "lights": rules {kind, node (regex on the node path), material (regex on the source material name),
    x/y/z ([min, max] metres, ISO, output frame, on the triangle centroid)}; kind "indicator" splits into
    indicator_left / indicator_right by the centroid's side. The first matching rule takes a triangle. Returns the
    (possibly new) mesh index."""
    rules = car.get("lights", {}).get("rules", [])
    if not rules or any(p.get("mode", 4) != 4 for p in b.js["meshes"][mesh_index]["primitives"]):
        return mesh_index
    src = b.js["meshes"][mesh_index]
    prims, changed = [], False
    for pi, p in enumerate(src["primitives"]):
        wp, tris, mat = n.prims[pi]
        mat_name = b.js["materials"][mat]["name"] if mat is not None else ""
        cen = iso_of(wp)[tris].mean(1)
        kind = np.full(len(tris), "", object)
        for r in rules:
            if r.get("node") and not re.search(r["node"], n.path, re.I):
                continue
            if r.get("material") and not re.search(r["material"], mat_name, re.I):
                continue
            m = (kind == "") & _in_range(cen[:, 0], r.get("x")) & _in_range(cen[:, 1], r.get("y")) \
                & _in_range(cen[:, 2], r.get("z"))
            if r["kind"] == "indicator":
                kind[m & (cen[:, 1] >= 0)] = "indicator_left"
                kind[m & (cen[:, 1] < 0)] = "indicator_right"
            else:
                kind[m] = r["kind"]
        if not (kind != "").any():
            prims.append(p)
            continue
        changed = True
        for k in [""] + sorted(set(kind) - {""}):
            sel = tris[kind == k]
            if len(sel) == 0:
                continue
            q = copy.deepcopy(p)
            q["indices"] = b.index_accessor(sel)
            if k:
                q["material"] = lamp_material(b, k, mat, lamp_mats)
            prims.append(q)
    if not changed:
        return mesh_index
    b.js["meshes"].append({"name": src.get("name", "") + "_lamps", "primitives": prims})
    return len(b.js["meshes"]) - 1


def lamp_material(b, kind, src_mat, lamp_mats):
    """One light_<kind>[_<n>] copy per (kind, source material); a textured source keeps its colour map and gets it
    as emissive map too, so a lit lamp shows the lamp texture (exterior_lamps.gd sets the colour and strength)."""
    key = (kind, src_mat)
    if key not in lamp_mats:
        m = copy.deepcopy(b.js["materials"][src_mat]) if src_mat is not None else {}
        same = [k for k in lamp_mats if k[0] == kind]
        m["name"] = f"light_{kind}" + (f"_{len(same) + 1}" if same else "")
        tex = m.get("pbrMetallicRoughness", {}).get("baseColorTexture")
        if tex is not None:
            m["emissiveTexture"] = copy.deepcopy(tex)
        m["emissiveFactor"] = [0.0, 0.0, 0.0]
        b.js["materials"].append(m)
        lamp_mats[key] = len(b.js["materials"]) - 1
    return lamp_mats[key]


def rename_materials(b, car):
    """cars.json "materials": paint / rim regexes on the source material names -> materials named "paint" /
    "rim" (the names the garage and vehicle_visual.gd recolour). "paint_tint_texture": regex of a material whose
    colour map is a white-body copy of the paint's map (atlas cars): the paint takes that map and a colour factor
    equal to the original paint's mean colour over it, and is named "paint_tinted" (recolouring keeps the map)."""
    spec = car.get("materials", {})
    renamed = {}
    mats = b.js["materials"]
    for role in ("paint", "rim"):
        rx = spec.get(role)
        if not rx:
            continue
        for i, m in enumerate(mats):
            if i in renamed or not re.search(rx, m.get("name", ""), re.I):
                continue
            renamed[i] = role
            m["name"] = role if role == "rim" or not spec.get("paint_tint_texture") else "paint_tinted"
    tint = spec.get("paint_tint_texture")
    if tint:
        white = [m for m in mats if re.search(tint, m.get("name", ""), re.I)]
        for i, role in renamed.items():
            if role != "paint" or not white:
                continue
            pbr = mats[i].setdefault("pbrMetallicRoughness", {})
            wpbr = white[0].get("pbrMetallicRoughness", {})
            if "baseColorTexture" in wpbr:
                pbr["baseColorTexture"] = copy.deepcopy(wpbr["baseColorTexture"])
                pbr["baseColorFactor"] = spec.get("paint_tint_factor", [1.0, 1.0, 1.0, 1.0])
    return renamed


def driver_eye(car, nodes, iso_of):
    """socket_driver_eye (ISO, output frame): cars.json "driver_eye" {"at": [x, y, z]} or {"steering": regex of the
    steering wheel node, "offset": [dx, dy, dz] from its centre}."""
    spec = car.get("driver_eye")
    if not spec:
        return None
    if "at" in spec:
        return np.asarray(spec["at"], float)
    sw = [n for n in nodes if re.search(spec["steering"], n.path, re.I)]
    if not sw:
        raise SystemExit(f"{car['id']}: steering wheel node /{spec['steering']}/ not found for the driver eye")
    pts = iso_of(np.concatenate([n.points() for n in sw]))
    return 0.5 * (pts.min(0) + pts.max(0)) + np.asarray(spec["offset"], float)


def build(car):
    src = os.path.join(SRC_DIR, car["source"])
    js, bin_, nodes = analyze.load(src)
    nodes, dropped = analyze.drop_ground(nodes, analyze.AXES[car.get("up", "+y")])
    if car.get("forward"):
        frame = analyze.Frame(analyze.AXES[car["forward"]], analyze.AXES[car.get("up", "+y")])
    else:
        frame = analyze.guess_frame(nodes)
    brake_re = re.compile(car.get("brake_parts", "caliper"), re.I)
    brakes = [n for n in nodes if brake_re.search(n.path)]
    rest = [n for n in nodes if n not in brakes]
    wheels, method = analyze.find_wheels(rest, frame)
    if not wheels:
        raise SystemExit(f"{car['id']}: wheels not found ({method})")

    # Wheel centres (ISO, source origin): bbox centre of every part of the wheel.
    centre, radius, width = {}, {}, {}
    for c, w in wheels.items():
        iso = frame.to_iso(np.concatenate([part_points(p) for p in w.parts]))
        lo, hi = iso.min(0), iso.max(0)
        centre[c] = 0.5 * (lo + hi)
        radius[c] = 0.25 * ((hi[0] - lo[0]) + (hi[2] - lo[2]))
        width[c] = hi[1] - lo[1]
    x0 = 0.25 * sum(centre[c][0] for c in analyze.CORNERS)
    y0 = 0.25 * sum(centre[c][1] for c in analyze.CORNERS)
    z0 = 0.25 * sum(centre[c][2] - radius[c] for c in analyze.CORNERS)
    origin = np.array([x0, y0, z0])
    pivot = {c: centre[c] - origin for c in analyze.CORNERS}

    # source world -> output glTF (game axes, metres, new origin)
    R = np.stack([frame.forward, frame.left, frame.up])  # rows: iso axes in source coordinates
    N = np.eye(4)
    N[:3, :3] = P @ R
    N[:3, 3] = -(P @ origin)

    def T_inv(c):
        m = np.eye(4)
        m[:3, 3] = -(P @ pivot[c])
        return m

    b = Builder(js, bin_)
    lamp_mats = {}

    def iso_of(world_points):
        return frame.to_iso(world_points) - origin

    wheel_parts = {}   # source node index -> {corner: {prim: tris}} or "whole" -> corner
    for c, w in wheels.items():
        for node, sel in w.parts:
            if sel is None:
                wheel_parts.setdefault(node.index, {})["whole"] = c
            else:
                pi, tri_idx = sel
                d = wheel_parts.setdefault(node.index, {}).setdefault(c, {})
                d[pi] = np.concatenate([d.get(pi, np.zeros((0, 3), np.int64)), node.prims[pi][1][tri_idx]])

    body_children, corner_children = [], {c: {"steer": [], "wheel": []} for c in analyze.CORNERS}
    by_index = {n.index: n for n in nodes}
    for n in nodes:
        world_out = N @ n.world
        if n in brakes:
            ci = min(analyze.CORNERS, key=lambda c: np.linalg.norm(frame.to_iso(n.points().mean(0)) - centre[c]))
            corner_children[ci]["steer"].append(b.add_node({"name": n.name, "mesh": n.mesh,
                                                            "matrix": col_major(T_inv(ci) @ world_out)}))
            continue
        wp = wheel_parts.get(n.index)
        if wp is None:
            mesh = split_lamps(b, car, n, n.mesh, iso_of, lamp_mats)
            body_children.append(b.add_node({"name": n.name, "mesh": mesh, "matrix": col_major(world_out)}))
            continue
        if "whole" in wp:
            c = wp["whole"]
            corner_children[c]["wheel"].append(b.add_node({"name": n.name, "mesh": n.mesh,
                                                           "matrix": col_major(T_inv(c) @ world_out)}))
            continue
        # split mesh (geometric clustering): wheel triangles per corner, the remainder stays on the body
        taken = {}
        for c, prim_tris in wp.items():
            m = b.sub_mesh(n.mesh, prim_tris, f"{n.name}_wheel_{c}")
            if m is not None:
                corner_children[c]["wheel"].append(b.add_node({"name": f"{n.name}_wheel_{c}", "mesh": m,
                                                               "matrix": col_major(T_inv(c) @ world_out)}))
            for pi, t in prim_tris.items():
                taken.setdefault(pi, set()).update(map(tuple, np.sort(t, axis=1)))
        remain = {}
        for pi, (_, t, _) in enumerate(by_index[n.index].prims):
            if pi in taken:
                keep = np.array([tuple(r) not in taken[pi] for r in np.sort(t, axis=1)], bool)
                remain[pi] = t[keep]
            else:
                remain[pi] = t
        m = b.sub_mesh(n.mesh, remain, f"{n.name}_body")
        if m is not None:
            body_children.append(b.add_node({"name": n.name, "mesh": m, "matrix": col_major(world_out)}))

    root_children = list(body_children)
    for c in analyze.CORNERS:
        wheel = b.add_node({"name": f"wheel_{c}", "children": corner_children[c]["wheel"]})
        if c in FRONT:
            steer = b.add_node({"name": f"steer_{c}", "children": [wheel] + corner_children[c]["steer"]})
            susp_children = [steer]
        else:
            susp_children = [wheel] + corner_children[c]["steer"]
        root_children.append(b.add_node({"name": f"susp_{c}", "translation": [float(v) for v in P @ pivot[c]],
                                         "children": susp_children}))
    eye = driver_eye(car, nodes, iso_of)
    if eye is not None:
        root_children.append(b.add_node({"name": "socket_driver_eye", "translation": [float(v) for v in P @ eye]}))
    renamed = rename_materials(b, car)
    root = b.add_node({"name": car["id"], "children": root_children})

    src_sha = sha256(src)
    ex = js.get("asset", {}).get("extras", {})
    asset = {
        "version": "2.0",
        "generator": "racing_game tools/carimport",
        "copyright": f"\"{ex.get('title', '')}\" by {ex.get('author', '')}, licensed {ex.get('license', '')}; "
                     f"modified (re-rigged, re-oriented) for racing_game",
        "extras": dict(ex, source_file=car["source"], source_sha256=src_sha,
                       modifications="Axes and origin normalised to the physics_sim.rig/1 frame, wheels re-parented "
                                     "under susp_/steer_/wheel_ pivots at the wheel centres, brake calipers under "
                                     "the steer/susp pivots, baked ground/shadow plane removed, lamp triangles moved "
                                     "to light_* material copies, body paint / rim materials renamed, a driver eye "
                                     "socket added. Geometry and textures otherwise unchanged."),
    }
    out_js, out_bin = b.finish(root, asset)
    lamps = {}
    for (kind, _), mi in sorted(lamp_mats.items(), key=lambda kv: kv[1]):
        lamps.setdefault(kind, []).append(out_js["materials"][mi]["name"])
    return dict(js=out_js, bin=out_bin, frame=frame, method=method, pivot=pivot, radius=radius, width=width,
                dropped=[n.name for n in dropped], src_sha=src_sha, extras=ex, lamps=lamps, eye=eye,
                renamed=sorted(set(renamed.values())))


def bounds_iso(js, bin_, under=None):
    """ISO bbox of every mesh in the written file (optionally only below the node named `under`)."""
    world, parent = glb.world_matrices(js)
    names = {i: n.get("name", "") for i, n in enumerate(js["nodes"])}
    pts = []
    for i, m in world.items():
        n = js["nodes"][i]
        if "mesh" not in n:
            continue
        if under is not None:
            j, ok = i, False
            while j is not None:
                if names[j] == under:
                    ok = True
                    break
                j = parent.get(j)
            if not ok:
                continue
        for p in js["meshes"][n["mesh"]]["primitives"]:
            pos = glb.accessor(js, bin_, p["attributes"]["POSITION"]).astype(float)
            if "indices" in p:
                pos = pos[np.unique(glb.accessor(js, bin_, p["indices"]).reshape(-1))]
            g = pos @ m[:3, :3].T + m[:3, 3]
            pts.append(g[:, [2, 0, 1]])  # gltf -> iso
    p = np.concatenate(pts)
    return p.min(0), p.max(0)


def tyre_width(car, corner):
    """{"tyre_width": m} of the stock tyre the vehicle file fits on this wheel (the width the art stands for)."""
    ts = car.get("vehicle", {}).get("tyre_spec")
    if not ts:
        return {}
    spec = ts["front" if corner in FRONT else "rear"] if "front" in ts else ts
    return {"tyre_width": spec["width_mm"] / 1000.0}


def rig_lights(car, r, look):
    """The rig "lights" block (exterior_lamps.gd / headlights.gd) for the light_* materials the import made; tail
    falls back to the brake lamps' materials (one bulb, two levels) when the model has no separate tail lamp."""
    lamps = dict(r["lamps"])
    if "tail" not in lamps and "brake" in lamps:
        lamps["tail"] = list(lamps["brake"])
    out = {}
    for kind in LIGHT_KINDS:
        if kind in lamps:
            out[kind] = dict(look[kind.split("_")[0]], materials=lamps[kind])
    return out


def write_rig(car, r, path, look):
    js, bin_ = r["js"], r["bin"]
    lo, hi = bounds_iso(js, bin_)
    nodes = [{"name": car["id"], "parent": None, "pivot": [0.0, 0.0, 0.0],
              "bounds_local": {"min": [round(float(v), 4) for v in lo], "max": [round(float(v), 4) for v in hi]}}]
    for c in analyze.CORNERS:
        pv = [round(float(v), 4) for v in r["pivot"][c]]
        nodes.append({"name": f"susp_{c}", "parent": car["id"], "pivot": pv,
                      "joint": {"type": "prismatic", "axis": [0, 0, 1], "min": -0.08, "max": 0.08,
                                "note": "wheel travel (bump +)"}})
        parent = f"susp_{c}"
        if c in FRONT:
            nodes.append({"name": f"steer_{c}", "parent": parent, "pivot": pv,
                          "joint": {"type": "revolute", "axis": [0, 0, 1], "min": -0.6, "max": 0.6,
                                    "note": "road-wheel steer angle, + = left"}})
            parent = f"steer_{c}"
        nodes.append({"name": f"wheel_{c}", "parent": parent, "pivot": pv,
                      "joint": {"type": "continuous", "axis": [0, 1, 0], "note": "wheel spin, + = rolling forward"},
                      "wheel": dict({"radius": round(float(r["radius"][c]), 4), "width": round(float(r["width"][c]), 4),
                                     "side": "left" if c[1] == "L" else "right"}, **tyre_width(car, c))})
    ex = r["extras"]
    rig = {
        "format": "physics_sim.rig/1",
        "model": f"{car['id']}.glb",
        "title": car["title"],
        "description": f"Imported from the CC BY model \"{ex.get('title', '')}\" ({car['source']}) by "
                       f"tools/carimport.",
        "frame": "ISO 8855: x forward, y left, z up; metres; radians. The .glb itself uses glTF axes (+Y up, +Z "
                 "forward, +X left): gltf(x,y,z) = iso(y,z,x).",
        "origin": "Ground plane below the wheel centres (mean wheel bottom), x = 0 midway between the axles, y = 0 "
                  "midway between the wheels.",
        "rest_pose": "All node rotations are identity at rest; a node's origin is its joint pivot. Joint values are "
                     "offsets from the rest pose (0 = as modelled).",
        "source": {"file": car["source"], "sha256": r["src_sha"], "title": ex.get("title", ""),
                   "author": ex.get("author", ""), "license": ex.get("license", ""), "url": ex.get("source", "")},
        "wheel_width_scale": "rig",
        "overall_bounds": nodes[0]["bounds_local"],
        "nodes": nodes,
        "sockets": ([{"name": "socket_driver_eye", "parent": car["id"],
                      "position": [round(float(v), 4) for v in r["eye"]], "note": "driver eye point (camera)"}]
                    if r["eye"] is not None else []),
        "lights": rig_lights(car, r, look),
        "materials": {"renamed": r["renamed"], "note": "Source materials renamed for the garage / drive colours "
                      "(paint, paint_tinted, rim) per tools/carimport/cars.json \"materials\"."},
        "notes": [f"Wheels found by {r['method']}.",
                  "wheel_width_scale \"rig\": vehicle_visual.gd / garage_scene.gd scale a wheel by physics tyre "
                  "width / the rig wheel's tyre_width (the stock tyre the art stands for, else its modelled width), "
                  "so the stock tyre keeps the art unscaled.",
                  "Lamps: triangles picked by tools/carimport/cars.json \"lights\" rules (node, material, ISO box) "
                  "moved onto light_* material copies; no cockpit controls or engine bay data."],
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(rig, f, indent=1)
        f.write("\n")
    return rig


def write_vehicle(car, r):
    v = car.get("vehicle")
    if not v:
        return None
    base = json.load(open(os.path.join(REPO, v["base"]), encoding="utf-8"))
    out = copy.deepcopy(base)
    out["name"] = v["name"]
    piv = r["pivot"]
    wb = 0.5 * (piv["FL"][0] + piv["FR"][0] - piv["RL"][0] - piv["RR"][0])
    bf = [w["attachment_local"][0] for w in base["wheels"] if w["is_front"]][0]
    br = [w["attachment_local"][0] for w in base["wheels"] if not w["is_front"]][0]
    a = bf / (bf - br) * wb
    track = {"F": 0.5 * (piv["FL"][1] - piv["FR"][1]), "R": 0.5 * (piv["RL"][1] - piv["RR"][1])}
    for w in out["wheels"]:
        axle = "F" if w["is_front"] else "R"
        side = 1.0 if w["name"].endswith("L") else -1.0
        w["attachment_local"][0] = round(a if axle == "F" else a - wb, 4)
        w["attachment_local"][1] = round(side * abs(track[axle]), 4)
        ts = v["tyre_spec"]
        spec = dict(ts["front" if w["is_front"] else "rear"] if "front" in ts else ts)
        w["tyre_spec"] = spec
        w["wheel_radius"] = round(tyre_radius_m(spec), 4)
        w["wheel_width"] = spec["width_mm"] / 1000.0
    out["source"] = (f"{v['base']} with the wheel positions and tyre size matched to the imported model "
                     f"data/models/imported/{car['id']}/{car['id']}.glb (tools/carimport): wheelbase {wb:.3f} m "
                     f"(front axle {a:.3f} m ahead of the CoG, the base car's CoG fraction kept), track front "
                     f"{2 * abs(track['F']):.3f} m / rear {2 * abs(track['R']):.3f} m, tyre "
                     f"{tyre_text(v['tyre_spec'])} (model wheel radius {np.mean(list(r['radius'].values())):.3f} m). Everything else is the base "
                     f"file's. Base source: " + str(base.get("source", "")))
    path = os.path.join(REPO, v["out"])
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
        f.write("\n")
    return out


def check(car):
    """Checks on the WRITTEN files. Returns a list of failure strings."""
    fails = []
    d = os.path.join(OUT_DIR, car["id"])
    js, bin_ = glb.read(os.path.join(d, car["id"] + ".glb"))
    rig = json.load(open(os.path.join(d, car["id"] + ".rig.json"), encoding="utf-8"))
    names = {n.get("name") for n in js["nodes"]}
    ground = []
    for c in analyze.CORNERS:
        need = [f"susp_{c}", f"wheel_{c}"] + ([f"steer_{c}"] if c in FRONT else [])
        missing = [n for n in need if n not in names]
        if missing:
            fails.append(f"{c}: missing {missing}")
            continue
        lo, hi = bounds_iso(js, bin_, under=f"wheel_{c}")
        ctr = 0.5 * (lo + hi)
        piv = np.array([n["pivot"] for n in rig["nodes"] if n["name"] == f"wheel_{c}"][0])
        susp = [n for n in js["nodes"] if n.get("name") == f"susp_{c}"][0]
        t = np.array(susp.get("translation", [0, 0, 0]))[[2, 0, 1]]
        err = max(np.linalg.norm(ctr - piv), np.linalg.norm(t - piv))
        if err > PIVOT_TOL_M:
            fails.append(f"{c}: pivot {err * 100:.1f} cm from the wheel centre")
        ext = hi - lo
        if abs(ext[0] - ext[2]) > 0.05 * max(ext[0], ext[2]):
            fails.append(f"{c}: wheel not round in side view ({ext[0]:.3f} x {ext[2]:.3f} m)")
        ground.append(lo[2])
        for n in js["nodes"]:
            if n.get("name") in (f"wheel_{c}", f"steer_{c}") and ("translation" in n or "matrix" in n
                                                                   or "rotation" in n):
                fails.append(f"{c}: {n['name']} is not at its parent's pivot")
    if ground and max(abs(g) for g in ground) > PIVOT_TOL_M:
        fails.append(f"wheel bottoms {np.round(ground, 3)} not on the ground plane")
    mat_names = {m.get("name") for m in js.get("materials", [])}
    for kind, spec in rig.get("lights", {}).items():
        missing = [m for m in spec.get("materials", []) if m not in mat_names]
        if missing:
            fails.append(f"lights.{kind}: materials {missing} not in the glb")
    for kind in car.get("lights", {}).get("required", []):
        if kind not in rig.get("lights", {}):
            fails.append(f"lights.{kind}: no lamp triangles matched")
    for role in car.get("materials", {}):
        if role in ("paint", "rim") and not any(n == role or n.startswith(role + "_") for n in mat_names):
            fails.append(f"material role {role}: no material renamed")
    if car.get("driver_eye") and not any(n.get("name") == "socket_driver_eye" for n in js["nodes"]):
        fails.append("socket_driver_eye missing")
    a = js.get("asset", {})
    ex = a.get("extras", {})
    if not (ex.get("author") and "CC-BY" in ex.get("license", "") and ex.get("source_sha256") and a.get("copyright")):
        fails.append("attribution / licence / source hash missing from asset")
    v = car.get("vehicle")
    if v:
        veh = json.load(open(os.path.join(REPO, v["out"]), encoding="utf-8"))
        for w in veh["wheels"]:
            model_r = [n["wheel"]["radius"] for n in rig["nodes"] if n["name"] == f"wheel_{w['name']}"][0]
            if abs(tyre_radius_m(w["tyre_spec"]) - model_r) > TYRE_TOL_M:
                fails.append(f"{w['name']}: tyre radius {tyre_radius_m(w['tyre_spec']):.3f} m vs model {model_r:.3f}")
            pv = [n["pivot"] for n in rig["nodes"] if n["name"] == f"wheel_{w['name']}"][0]
            if abs(abs(w["attachment_local"][1]) - abs(pv[1])) > PIVOT_TOL_M:
                fails.append(f"{w['name']}: physics track half {w['attachment_local'][1]} vs model {pv[1]}")
    return fails


def record_attribution(car, r):
    att = json.load(open(ATTRIB, encoding="utf-8"))
    derived = [e for e in att.get("derived", []) if e.get("id") != car["id"]]
    ex = r["extras"]
    derived.append({
        "id": car["id"],
        "file": f"data/models/imported/{car['id']}/{car['id']}.glb",
        "source_file": car["source"],
        "source_sha256": r["src_sha"],
        "title": ex.get("title", ""),
        "author": ex.get("author", ""),
        "license": "CC-BY-4.0",
        "modifications": "Re-oriented to the game's axes and origin, wheels re-parented under suspension/steering/"
                         "spin pivots at the wheel centres, brake calipers under the steering pivots, baked "
                         "ground/shadow plane removed, lamp triangles moved to light_* material copies, paint/rim "
                         "materials renamed, driver eye socket added (tools/carimport). Geometry and textures "
                         "otherwise unchanged.",
        "modified_on": datetime.date.today().isoformat(),
    })
    att["derived"] = sorted(derived, key=lambda e: e["id"])
    with open(ATTRIB, "w", encoding="utf-8") as f:
        json.dump(att, f, indent=2, ensure_ascii=False)
        f.write("\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ids", nargs="*")
    ap.add_argument("--check", action="store_true", help="only check the written files")
    a = ap.parse_args()
    config = json.load(open(CONFIG, encoding="utf-8"))
    cars = config["cars"]
    look = config["light_look"]
    if a.ids:
        cars = [c for c in cars if c["id"] in a.ids]
    bad = 0
    for car in cars:
        if not a.check:
            r = build(car)
            d = os.path.join(OUT_DIR, car["id"])
            os.makedirs(d, exist_ok=True)
            glb.write(os.path.join(d, car["id"] + ".glb"), r["js"], r["bin"])
            write_rig(car, r, os.path.join(d, car["id"] + ".rig.json"), look)
            write_vehicle(car, r)
            record_attribution(car, r)
            piv = r["pivot"]
            print(f"{car['id']}: wheels by {r['method']}, frame {r['frame'].describe()}, dropped {r['dropped']}, "
                  f"wheelbase {0.5 * (piv['FL'][0] + piv['FR'][0] - piv['RL'][0] - piv['RR'][0]):.3f} m, "
                  f"track {piv['FL'][1] - piv['FR'][1]:.3f} m, wheel r {np.mean(list(r['radius'].values())):.3f} m, "
                  f"lamps {sorted(r['lamps'])}, renamed {r['renamed']}, eye {None if r['eye'] is None else np.round(r['eye'], 3).tolist()}")
        fails = check(car)
        print(f"{car['id']}: check {'PASS' if not fails else 'FAIL'}" + "".join(f"\n  - {f}" for f in fails))
        bad += bool(fails)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
