"""Validate racing_game's generated car models:  python3 tools/cargen/validate.py [model_id ...] [--determinism]

Per model:
- physics_sim's own asset checks (assetgen validate.check_model: glTF structure,
  rig <-> glb node agreement, joints, wheel radius and ground contact, budget),
  pointed at racing_game's data/models;
- closed shells: every node's triangles weld (1 um) into closed, consistently
  wound shells - zero boundary, non-manifold, degenerate, duplicate and
  inconsistent-winding edges (the hypercar's own criterion);
- flow selection: pinned to the GLB sha256, ranges inside their primitives, no
  overlap, no interior material, fewer triangles than the whole model;
- cockpit sidecar: every control/screen node exists in the rig, joint type and
  axis agree, every screen material exists;
- --determinism: a rebuild is byte-identical to the files on disk.
Exit 0 when everything passes.
"""
import hashlib
import importlib
import json
import os
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cargen  # noqa: E402
import importlib.util  # noqa: E402

_spec = importlib.util.spec_from_file_location("ag_validate", os.path.join(cargen.ASSETGEN, "validate.py"))
ag_validate = importlib.util.module_from_spec(_spec)  # physics_sim's assetgen/validate.py, read-only
_spec.loader.exec_module(ag_validate)
from cargen.topology import audit, is_closed  # noqa: E402

OUT = os.path.join(cargen.REPO, "data", "models")
ag_validate.OUT = OUT
BUDGET = {"pico": (170_000, 6.5e6), "ventura_gt": (190_000, 7.0e6), "runabout_t": (170_000, 6.5e6)}
ag_validate.BUDGET.update(BUDGET)
INTERIOR = ("interior", "interior_trim", "engine", "headliner", "carpet", "seat", "seat_accent", "dash_hard")


def load_prims(mid):
    gltf, blob, _ = ag_validate.read_glb(os.path.join(OUT, mid, mid + ".glb"))
    nodes = gltf["nodes"]
    parent = {c: i for i, n in enumerate(nodes) for c in n.get("children", [])}

    def world_t(i):
        t = np.array(nodes[i].get("translation", [0, 0, 0]), dtype=float)
        return t + (world_t(parent[i]) if i in parent else 0.0)

    prims = {}
    for i, n in enumerate(nodes):
        if "mesh" not in n:
            continue
        off = world_t(i)
        for p in gltf["meshes"][n["mesh"]]["primitives"]:
            pos = np.array(ag_validate.accessor(gltf, blob, p["attributes"]["POSITION"]), dtype=float) + off
            idx = np.array(ag_validate.accessor(gltf, blob, p["indices"]), dtype=np.int64).reshape(-1, 3)
            mat = gltf["materials"][p["material"]]["name"]
            prims[(n["name"], mat)] = pos[idx]
    return gltf, prims


def check_topology(mid, prims, errors):
    per_node = defaultdict(list)
    for (node, _), t in prims.items():
        per_node[node].append(t)
    bad = 0
    for node, ts in sorted(per_node.items()):
        rep = audit(np.concatenate(ts))
        if not is_closed(rep):
            bad += 1
            errors.append(f"{mid}: node {node} not closed: " +
                          ", ".join(f"{k}={v}" for k, v in rep.items() if k != "triangles" and v))
    return len(per_node), bad


def check_flow(mid, prims, errors):
    path = os.path.join(OUT, mid, mid + ".flow.json")
    sel = json.load(open(path, encoding="utf-8"))
    sha = hashlib.sha256(open(os.path.join(OUT, mid, mid + ".glb"), "rb").read()).hexdigest()
    if sel.get("source_glb_sha256") != sha:
        errors.append(f"{mid}: flow selection sha256 does not match the glb")
    if sel.get("cfd_ready") is not False or sel.get("status") != "visual_surface_selection_not_fluid_solid":
        errors.append(f"{mid}: flow selection status fields wrong")
    used = defaultdict(list)
    total = sum(len(t) for t in prims.values())
    selected = 0
    for row in sel["ranges"]:
        key = (row["node"], row["material"])
        if key not in prims:
            errors.append(f"{mid}: flow range names missing primitive {key}")
            continue
        a, c = row["first_triangle"], row["triangle_count"]
        if not (0 <= a and c > 0 and a + c <= len(prims[key])):
            errors.append(f"{mid}: flow range {key} {a}+{c} out of bounds ({len(prims[key])})")
        for (s, n) in used[key]:
            if a < s + n and s < a + c:
                errors.append(f"{mid}: flow ranges overlap in {key}")
        used[key].append((a, c))
        if row["material"] in INTERIOR or row["material"].startswith("screen_"):
            errors.append(f"{mid}: flow range on interior material {row['material']}")
        selected += c
    if not 0 < selected < total:
        errors.append(f"{mid}: flow selection covers {selected} of {total} triangles")
    return selected, total


def check_cockpit(mid, gltf, errors):
    ck = json.load(open(os.path.join(OUT, mid, mid + ".cockpit.json"), encoding="utf-8"))
    rig = json.load(open(os.path.join(OUT, mid, mid + ".rig.json"), encoding="utf-8"))
    rn = {n["name"]: n for n in rig["nodes"]}
    mats = {m["name"] for m in gltf["materials"]}
    for c in ck["controls"]:
        n = rn.get(c["node"])
        if n is None:
            errors.append(f"{mid}: cockpit control node {c['node']} not in the rig")
            continue
        j = n.get("joint")
        if c.get("joint") and (not j or j["type"] != c["joint"]):
            errors.append(f"{mid}: cockpit control {c['node']} joint type differs from the rig")
        if c.get("hit") and len(c["hit"].get("half_extents", ())) != 3:
            errors.append(f"{mid}: cockpit control {c['node']} hit box malformed")
    for s in ck["screens"]:
        if s["node"] not in rn:
            errors.append(f"{mid}: screen node {s['node']} not in the rig")
        if s["material"] not in mats:
            errors.append(f"{mid}: screen material {s['material']} not in the glb")
    if ck.get("source_glb_sha256") != hashlib.sha256(
            open(os.path.join(OUT, mid, mid + ".glb"), "rb").read()).hexdigest():
        errors.append(f"{mid}: cockpit sha256 does not match the glb")
    return len(ck["controls"]), len(ck["screens"])


def main(argv):
    det = "--determinism" in argv
    ids = [a for a in argv if not a.startswith("--")]
    if not ids:
        ids = [m["id"] for m in json.load(open(os.path.join(OUT, "index.json"), encoding="utf-8"))]
    errors, warns = [], []
    for mid in ids:
        s = ag_validate.check_model(mid, errors, warns)
        if not s:
            continue
        gltf, prims = load_prims(mid)
        nn, bad = check_topology(mid, prims, errors)
        sel, total = check_flow(mid, prims, errors)
        nc, ns = check_cockpit(mid, gltf, errors)
        print(f"{mid:12s} {s['tris']:7d} tris {s['mb']:5.2f} MB {s['nodes']:3d} nodes {s['joints']:3d} joints "
              f"{s['sockets']:2d} sockets | closed {nn - bad}/{nn} nodes | flow {sel}/{total} tris | "
              f"cockpit {nc} controls {ns} screens")
        if det:
            model = importlib.import_module(f"cars.{mid}").build()
            glb, _ = model._build_glb()
            if glb != open(os.path.join(OUT, mid, mid + ".glb"), "rb").read():
                errors.append(f"{mid}: rebuild is not byte-identical to the file on disk")
            else:
                print(f"{mid:12s} deterministic: rebuild byte-identical")
    for w in warns:
        print("WARN", w)
    for e in errors:
        print("FAIL", e)
    print("OK" if not errors else f"{len(errors)} failure(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
