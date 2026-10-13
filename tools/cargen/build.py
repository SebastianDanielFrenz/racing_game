"""Build racing_game's own car models:  python3 tools/cargen/build.py [model_id ...]

Writes data/models/<id>/<id>.glb, <id>.rig.json (physics_sim.rig/1, same
convention as physics_sim's data/models), <id>.flow.json (exterior CFD surface
selection, pinned to the GLB's sha256) and <id>.cockpit.json (rg.cockpit/1),
plus data/models/index.json. Stdlib only (plus physics_sim's assetgen, read-only).
Deterministic: same code -> byte-identical files.
"""
import hashlib
import importlib
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cargen  # noqa: E402  (puts assetgen on sys.path)

OUT = os.path.join(cargen.REPO, "data", "models")
MODELS = ["pico", "ventura_gt", "runabout_t"]


def write_json(path, obj):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2)
        f.write("\n")


def build_one(mid):
    mod = importlib.import_module(f"cars.{mid}")
    model = mod.build()
    folder = os.path.join(OUT, mid)
    stats = model.write(folder)
    with open(os.path.join(folder, mid + ".glb"), "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    sel = dict(model.flow_selection)
    sel["source_glb_sha256"] = sha
    write_json(os.path.join(folder, mid + ".flow.json"), sel)
    cockpit = dict(model.cockpit)
    cockpit["source_glb_sha256"] = sha
    write_json(os.path.join(folder, mid + ".cockpit.json"), cockpit)
    return stats


def main(argv):
    ids = argv or MODELS
    unknown = [i for i in ids if i not in MODELS]
    if unknown:
        sys.exit(f"unknown model(s): {unknown}; known: {MODELS}")
    for mid in ids:
        t0 = time.time()
        s = build_one(mid)
        print(f"{mid:12s} {s['triangles']:7d} tris  {s['nodes']:3d} nodes  {s['materials']:3d} mats  "
              f"{s['images']:3d} imgs  {s['bytes'] / 1e6:6.2f} MB  {time.time() - t0:5.1f} s")
    index = []
    for mid in MODELS:
        rig = os.path.join(OUT, mid, mid + ".rig.json")
        if os.path.exists(rig):
            with open(rig, encoding="utf-8") as f:
                title = json.load(f)["title"]
            index.append({"id": mid, "title": title, "glb": f"{mid}/{mid}.glb", "rig": f"{mid}/{mid}.rig.json",
                          "flow": f"{mid}/{mid}.flow.json", "cockpit": f"{mid}/{mid}.cockpit.json"})
    write_json(os.path.join(OUT, "index.json"), index)


if __name__ == "__main__":
    main(sys.argv[1:])
