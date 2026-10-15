#!/usr/bin/env python3
"""Survey of the downloaded source car models (data/models/downloaded/*.glb).

Prints one Markdown table row per model (triangles, materials, textures, node structure, axes,
size, wheelbase/track estimate) and, with --json, writes the raw numbers.

    python3 tools/carimport/survey.py [--json out.json] [files...]
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SRC = os.path.join(REPO, "data", "models", "downloaded")


def survey(path):
    js, bin_, nodes = analyze.load(path)
    nodes, ground = analyze.drop_ground(nodes)
    frame = analyze.guess_frame(nodes)
    pts = np.concatenate([n.points() for n in nodes])
    iso = frame.to_iso(pts)
    lo, hi = iso.min(0), iso.max(0)
    imgs = analyze.image_sizes(js, bin_)
    sizes = sorted({f"{s[0]}x{s[1]}" for _, s in imgs if s}, key=lambda t: -int(t.split("x")[0]))
    interior, glass, doors = analyze.interior_glass_doors(js, nodes)
    wheels, method = analyze.find_wheels(nodes, frame)
    r = {
        "file": os.path.basename(path),
        "title": js.get("asset", {}).get("extras", {}).get("title", ""),
        "triangles": int(sum(n.triangles() for n in nodes)),
        "mesh_nodes": len(nodes),
        "materials": len(js.get("materials", [])),
        "textures": len(imgs),
        "texture_sizes": sizes,
        "skins": len(js.get("skins", [])),
        "animations": len(js.get("animations", [])),
        "frame": frame.describe(),
        "size_m": [round(float(v), 2) for v in (hi - lo)],
        "interior": interior,
        "glass": glass,
        "doors": doors,
        "ground_planes": [n.name for n in ground],
        "wheels": method if wheels else f"not found ({method})",
    }
    if wheels:
        c = {k: w.centre for k, w in wheels.items()}
        r["wheelbase_m"] = round(float(0.5 * (c["FL"][0] + c["FR"][0] - c["RL"][0] - c["RR"][0])), 3)
        r["track_m"] = round(float(0.5 * (c["FL"][1] - c["FR"][1] + c["RL"][1] - c["RR"][1])), 3)
        r["wheel_radius_m"] = round(float(np.mean([w.radius for w in wheels.values()])), 3)
        r["wheel_nodes"] = sorted({p[0].name for w in wheels.values() for p in w.parts})[:8]
        named = all(analyze.corner_from_name(" ".join(p[0].path for p in w.parts)) in (k, None)
                    for k, w in wheels.items())
        r["corner_names_agree"] = named
    return r


def row(r):
    wb = f"{r['wheelbase_m']:.2f} / {r['track_m']:.2f} / {r['wheel_radius_m']:.2f}" if "wheelbase_m" in r else "-"
    tex = f"{r['textures']} ({', '.join(r['texture_sizes'][:3])})" if r["textures"] else "0"
    size = " x ".join(f"{v:.2f}" for v in r["size_m"])
    extra = []
    if r["ground_planes"]:
        extra.append("baked ground/shadow plane dropped")
    if r["skins"] or r["animations"]:
        extra.append(f"skins {r['skins']}, anims {r['animations']}")
    return (f"| `{r['file']}` | {r['triangles']:,} | {r['materials']} | {tex} | {r['mesh_nodes']} | {r['wheels']} | "
            f"{'yes' if r['interior'] else 'no'} | {'yes' if r['doors'] else 'no'} | {r['glass']} | {r['frame']} | "
            f"{size} | {wb} | {'; '.join(extra)} |")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="*")
    ap.add_argument("--json")
    a = ap.parse_args()
    files = a.files or sorted(os.path.join(SRC, f) for f in os.listdir(SRC) if f.endswith(".glb"))
    print("| file | triangles | materials | textures (largest) | mesh nodes | wheels | interior | doors | glass | "
          "axes (source) | L x W x H m | wheelbase / track / wheel r m | notes |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    out = []
    for f in files:
        r = survey(f)
        out.append(r)
        print(row(r), flush=True)
    if a.json:
        with open(a.json, "w") as fh:
            json.dump(out, fh, indent=1)


if __name__ == "__main__":
    main()
