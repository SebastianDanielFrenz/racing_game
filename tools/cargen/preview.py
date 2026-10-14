"""Software preview renderer for generated .glb models (numpy + PIL, no GPU, no Godot).

    python3 tools/cargen/preview.py data/models/pico/pico.glb out/cargen_preview/pico [--size 900x560]

Writes front_34.png, rear_34.png, side.png, top.png, front.png and a contact
sheet (sheet.png). Flat Lambert shading with one key light plus ambient; a
textured material is drawn in the mean colour of its base-colour image. Only
the rest pose is drawn (node translations, no joint motion). A review aid,
not a stand-in for the Godot renderer.
"""
import argparse
import io
import json
import math
import os
import struct
import sys

import numpy as np
from PIL import Image

COMP = {5126: ("<f4", 4), 5125: ("<u4", 4), 5123: ("<u2", 2), 5121: ("<u1", 1)}
NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def read_glb(path):
    data = open(path, "rb").read()
    jl, _ = struct.unpack_from("<II", data, 12)
    gltf = json.loads(data[20:20 + jl])
    bl, _ = struct.unpack_from("<II", data, 20 + jl)
    return gltf, data[28 + jl:28 + jl + bl]


def accessor(gltf, blob, i):
    a = gltf["accessors"][i]
    v = gltf["bufferViews"][a["bufferView"]]
    dt, size = COMP[a["componentType"]]
    n = NCOMP[a["type"]]
    off = v.get("byteOffset", 0) + a.get("byteOffset", 0)
    arr = np.frombuffer(blob, dtype=dt, count=a["count"] * n, offset=off)
    return arr.reshape(a["count"], n) if n > 1 else arr


def material_colours(gltf, blob):
    out = []
    for m in gltf.get("materials", []):
        pbr = m.get("pbrMetallicRoughness", {})
        c = np.array(pbr.get("baseColorFactor", [0.8, 0.8, 0.8, 1.0])[:3], dtype=float)
        tex = pbr.get("baseColorTexture")
        if tex is not None:
            img = gltf["images"][gltf["textures"][tex["index"]]["source"]]
            bv = gltf["bufferViews"][img["bufferView"]]
            raw = blob[bv.get("byteOffset", 0):bv.get("byteOffset", 0) + bv["byteLength"]]
            im = np.asarray(Image.open(io.BytesIO(raw)).convert("RGB"), dtype=float) / 255.0
            lin = np.where(im <= 0.04045, im / 12.92, ((im + 0.055) / 1.055) ** 2.4)
            c = c * lin.reshape(-1, 3).mean(axis=0)
        em = np.array(m.get("emissiveFactor", [0, 0, 0]), dtype=float)
        out.append((c, em, m.get("name", "")))
    return out


def load_triangles(path, hide=()):
    """Triangles in ISO frame (x fwd, y left, z up) plus a per-triangle linear colour."""
    gltf, blob = read_glb(path)
    nodes = gltf["nodes"]
    parent = {c: i for i, n in enumerate(nodes) for c in n.get("children", [])}

    def world_t(i):
        t = np.array(nodes[i].get("translation", [0, 0, 0]), dtype=float)
        return t + (world_t(parent[i]) if i in parent else 0.0)

    mats = material_colours(gltf, blob)
    tris, cols = [], []
    for i, n in enumerate(nodes):
        if "mesh" not in n or n.get("name") in hide:
            continue
        off = world_t(i)
        for prim in gltf["meshes"][n["mesh"]]["primitives"]:
            p = accessor(gltf, blob, prim["attributes"]["POSITION"]).astype(float) + off
            idx = accessor(gltf, blob, prim["indices"]).astype(np.int64)
            iso = p[:, [2, 0, 1]]
            t = iso[idx.reshape(-1, 3)]
            c, em, name = mats[prim["material"]]
            tris.append(t)
            cols.append(np.repeat(np.concatenate([c, em])[None, :], len(t), axis=0))
    return np.concatenate(tris), np.concatenate(cols)


def look_at(eye, target, up=(0, 0, 1)):
    f = np.asarray(target, float) - np.asarray(eye, float)
    f /= np.linalg.norm(f)
    r = np.cross(f, up)
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return np.stack([r, u, -f]), np.asarray(eye, float)


def render(tris, cols, eye, target, size=(900, 560), fov_deg=30.0, ortho=None, light=(0.4, 0.6, 0.8),
           bg=(0.82, 0.84, 0.87)):
    W, H = size
    R, e = look_at(eye, target)
    v = (tris - e) @ R.T  # camera frame, -z forward
    if ortho:
        s = H / ortho
        sx = W / 2 + v[..., 0] * s
        sy = H / 2 - v[..., 1] * s
        depth = -v[..., 2]
    else:
        zc = -v[..., 2]
        keep = (zc > 0.05).all(axis=1)
        v, tris, cols, zc = v[keep], tris[keep], cols[keep], zc[keep]
        f = (H / 2) / math.tan(math.radians(fov_deg) / 2)
        sx = W / 2 + f * v[..., 0] / zc
        sy = H / 2 - f * v[..., 1] / zc
        depth = zc
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nl = np.linalg.norm(n, axis=1)
    ok = nl > 1e-14
    n[ok] /= nl[ok, None]
    view_dir = (tris.mean(axis=1) - e) if not ortho else np.repeat((np.asarray(target) - e)[None], len(tris), 0)
    facing = np.einsum("ij,ij->i", n, view_dir) < 0
    nn = np.where(facing[:, None], n, -n)  # two-sided lighting
    L = np.asarray(light, float)
    L /= np.linalg.norm(L)
    lam = np.clip(nn @ L, 0, 1)
    shade = cols[:, :3] * (0.32 + 0.68 * lam[:, None]) + cols[:, 3:6]
    # back faces of closed shells darker so open holes show up in review
    shade = np.where(facing[:, None], shade, shade * 0.55)

    zbuf = np.full((H, W), np.inf)
    img = np.empty((H, W, 3))
    img[:] = np.asarray(bg) ** 2.2
    order = np.argsort(-depth.mean(axis=1))
    X, Y, Z = sx[order], sy[order], depth[order]
    C = shade[order]
    xmin = np.clip(np.floor(X.min(1)).astype(int), 0, W - 1)
    xmax = np.clip(np.ceil(X.max(1)).astype(int), 0, W - 1)
    ymin = np.clip(np.floor(Y.min(1)).astype(int), 0, H - 1)
    ymax = np.clip(np.ceil(Y.max(1)).astype(int), 0, H - 1)
    for k in range(len(X)):
        x0, x1, y0, y1 = xmin[k], xmax[k], ymin[k], ymax[k]
        if x1 < x0 or y1 < y0:
            continue
        ax, bx, cx = X[k]
        ay, by, cy = Y[k]
        den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(den) < 1e-12:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        w0 = ((by - cy) * (gx - cx) + (cx - bx) * (gy - cy)) / den
        w1 = ((cy - ay) * (gx - cx) + (ax - cx) * (gy - cy)) / den
        w2 = 1 - w0 - w1
        m = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
        if not m.any():
            continue
        z = w0 * Z[k, 0] + w1 * Z[k, 1] + w2 * Z[k, 2]
        sub = zbuf[y0:y1 + 1, x0:x1 + 1]
        m &= z < sub
        sub[m] = z[m]
        img[y0:y1 + 1, x0:x1 + 1][m] = C[k]
    out = np.clip(img, 0, 1) ** (1 / 2.2)
    return Image.fromarray((out * 255 + 0.5).astype(np.uint8))


VIEWS = {
    "front_34": dict(eye=(6.0, 4.2, 2.2), target=(0.2, 0, 0.6)),
    "rear_34": dict(eye=(-6.0, -4.4, 2.4), target=(-0.2, 0, 0.6)),
    "side": dict(eye=(0, 30, 0.7), target=(0, 0, 0.7), ortho=3.1),
    "front": dict(eye=(30, 0, 0.75), target=(0, 0, 0.75), ortho=2.0),
    "top": dict(eye=(0.001, 0, 30), target=(0, 0, 0), ortho=3.3),
}


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("glb")
    ap.add_argument("out_dir")
    ap.add_argument("--size", default="900x560")
    ap.add_argument("--hide", default="", help="comma-separated node names not to draw")
    ap.add_argument("--views", default=",".join(VIEWS))
    ap.add_argument("--view", action="append", default=[],
                    help="extra view name:ex,ey,ez:tx,ty,tz[:fov] (perspective)")
    a = ap.parse_args(argv)
    W, H = (int(s) for s in a.size.split("x"))
    tris, cols = load_triangles(a.glb, set(filter(None, a.hide.split(","))))
    os.makedirs(a.out_dir, exist_ok=True)
    views = {k: VIEWS[k] for k in a.views.split(",") if k}
    for spec in a.view:
        parts = spec.split(":")
        v = dict(eye=tuple(map(float, parts[1].split(","))), target=tuple(map(float, parts[2].split(","))))
        if len(parts) > 3:
            v["fov_deg"] = float(parts[3])
        views[parts[0]] = v
    images = []
    for name, v in views.items():
        im = render(tris, cols, size=(W, H), **v)
        im.save(os.path.join(a.out_dir, name + ".png"))
        images.append(im)
    cols_n = 2
    rows = (len(images) + cols_n - 1) // cols_n
    sheet = Image.new("RGB", (W * cols_n, H * rows), (255, 255, 255))
    for i, im in enumerate(images):
        sheet.paste(im, ((i % cols_n) * W, (i // cols_n) * H))
    sheet.save(os.path.join(a.out_dir, "sheet.png"))
    print(f"{len(tris)} triangles, {len(images)} views -> {a.out_dir}")


if __name__ == "__main__":
    main(sys.argv[1:])
