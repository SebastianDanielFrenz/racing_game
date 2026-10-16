"""Geometry analysis of a source car GLB: mesh nodes in world space, axes, wheel detection.

All positions here are in the source file's WORLD frame (every node matrix applied), before
normalisation. `frame()` returns the source axes for ISO forward/left/up.
"""
import io
import re

import numpy as np

import glb

WHEEL_WORD = re.compile(r"wheel|tire|tyre|(?<![a-z])rim|reifen|felge")
NOT_WHEEL = re.compile(r"steer|lenkrad|spare|arch|well|cover")
CORNERS = ("FL", "FR", "RL", "RR")
_CORNER_TOKENS = {
    "fl": "FL", "fr": "FR", "rl": "RL", "rr": "RR", "lf": "FL", "rf": "FR", "lr": "RL", "br": "RR", "bl": "RL",
}


def tokens(name):
    return [t for t in re.split(r"[^a-z0-9]+", (name or "").lower()) if t]


def corner_from_name(name):
    tk = tokens(name)
    for t in tk:
        if t in _CORNER_TOKENS:
            return _CORNER_TOKENS[t]
    s = " ".join(tk)
    fr = "F" if re.search(r"front|vorne", s) else ("R" if re.search(r"rear|back|hinten", s) else None)
    lr = "L" if re.search(r"left|links", s) else ("R" if re.search(r"right|rechts", s) else None)
    if fr and lr:
        return fr + lr
    return None


class MeshNode:
    def __init__(self, index, name, path, mesh, world, prims):
        self.index = index      # source node index
        self.name = name
        self.path = path        # names of the node and its ancestors, joined by '/'
        self.mesh = mesh        # source mesh index
        self.world = world      # 4x4
        self.prims = prims      # list of (positions world Nx3, triangle index array Mx3 or None, material)

    def points(self):
        return np.concatenate([p for p, _, _ in self.prims]) if self.prims else np.zeros((0, 3))

    def bbox(self):
        p = self.points()
        return p.min(0), p.max(0)

    def triangles(self):
        return sum(len(t) for _, t, _ in self.prims)


def load(path):
    js, bin_ = glb.read(path)
    world, parent = glb.world_matrices(js)
    nodes = []
    for i, m in world.items():
        n = js["nodes"][i]
        if "mesh" not in n:
            continue
        chain, j = [], i
        while j is not None:
            chain.append(js["nodes"][j].get("name", ""))
            j = parent.get(j)
        prims = []
        for p in js["meshes"][n["mesh"]]["primitives"]:
            if p.get("mode", 4) != 4:
                continue
            pos = glb.accessor(js, bin_, p["attributes"]["POSITION"]).astype(float)
            wp = pos @ m[:3, :3].T + m[:3, 3]
            if "indices" in p:
                idx = glb.accessor(js, bin_, p["indices"]).reshape(-1).astype(np.int64)
            else:
                idx = np.arange(len(pos))
            prims.append((wp, idx[: len(idx) // 3 * 3].reshape(-1, 3), p.get("material")))
        nodes.append(MeshNode(i, n.get("name", ""), "/".join(reversed(chain)), n["mesh"], m, prims))
    return js, bin_, nodes


def image_sizes(js, bin_):
    from PIL import Image
    out = []
    for im in js.get("images", []):
        if "bufferView" not in im:
            out.append((im.get("uri", "?"), None))
            continue
        bv = js["bufferViews"][im["bufferView"]]
        data = bin_[bv.get("byteOffset", 0): bv.get("byteOffset", 0) + bv["byteLength"]]
        try:
            with Image.open(io.BytesIO(data)) as img:
                out.append((im.get("mimeType", ""), img.size))
        except Exception:  # noqa: BLE001 - report, never fail the survey on one image
            out.append((im.get("mimeType", ""), None))
    return out


# ---------------------------------------------------------------- connected components

def _components(points, tris, weld=1e-3):
    """Connected components of a triangle soup after welding vertices closer than `weld` metres.
    Returns a label per triangle."""
    if len(tris) == 0:
        return np.zeros(0, int)
    key = np.round(points / weld).astype(np.int64)
    _, vid = np.unique(key, axis=0, return_inverse=True)
    vid = vid.reshape(-1)
    t = vid[tris]
    nv = vid.max() + 1
    lab = np.arange(nv)
    a = np.concatenate([t[:, 0], t[:, 1], t[:, 2]])
    b = np.concatenate([t[:, 1], t[:, 2], t[:, 0]])
    while True:
        m = np.minimum(lab[a], lab[b])
        new = lab.copy()
        np.minimum.at(new, a, m)
        np.minimum.at(new, b, m)
        for _ in range(8):
            new = new[new]
        if np.array_equal(new, lab):
            break
        lab = new
    return lab[t[:, 0]]


# ---------------------------------------------------------------- axes

AXES = {"+x": np.array([1.0, 0, 0]), "-x": np.array([-1.0, 0, 0]), "+y": np.array([0, 1.0, 0]),
        "-y": np.array([0, -1.0, 0]), "+z": np.array([0, 0, 1.0]), "-z": np.array([0, 0, -1.0])}


def axis_name(v):
    for k, a in AXES.items():
        if np.allclose(v, a):
            return k
    return str(v)


class Frame:
    """Source-world unit vectors for ISO forward (x), left (y), up (z)."""

    def __init__(self, forward, up):
        self.forward = np.asarray(forward, float)
        self.up = np.asarray(up, float)
        self.left = np.cross(self.up, self.forward)

    def to_iso(self, p):
        p = np.asarray(p, float)
        return np.stack([p @ self.forward, p @ self.left, p @ self.up], axis=-1)

    def describe(self):
        return f"fwd {axis_name(self.forward)}, up {axis_name(self.up)}"


GROUND_WORD = re.compile(r"shadow|ground|floor|plane|backdrop")


def drop_ground(nodes, up=None):
    """Split off baked shadow quads / ground planes: named like one, flat along up, at the very bottom,
    and covering at least the car's footprint. Returns (car nodes, dropped nodes)."""
    up = AXES["+y"] if up is None else up
    k = int(np.argmax(np.abs(up)))
    allp = np.concatenate([n.points() for n in nodes])
    bottom = allp[:, k].min() if up[k] > 0 else allp[:, k].max()
    keep, drop = [], []
    for n in nodes:
        lo, hi = n.bbox()
        ext = hi - lo
        horiz = [a for a in range(3) if a != k]
        low = (lo[k] if up[k] > 0 else hi[k])
        flat = ext[k] < 0.4 and abs(low - bottom) < 0.05 + ext[k]
        big = min(ext[a] for a in horiz) > 1.0
        if GROUND_WORD.search(n.name.lower()) and flat and big:
            drop.append(n)
        else:
            keep.append(n)
    return keep, drop


def guess_frame(nodes, hint_forward=None, hint_up=None):
    """Up: glTF +Y unless hinted. Forward: the longer horizontal bbox axis, sign from names
    (front/head/hood/windshield vs rear/tail/trunk) else from named wheel corners, else +Z."""
    up = AXES[hint_up] if hint_up else AXES["+y"]
    pts = np.concatenate([n.points() for n in nodes])
    lo, hi = pts.min(0), pts.max(0)
    if hint_forward:
        return Frame(AXES[hint_forward], up)
    horiz = [k for k in range(3) if abs(up[k]) < 0.5]
    ext = hi - lo
    k = max(horiz, key=lambda a: ext[a])
    ax = np.zeros(3)
    ax[k] = 1.0
    centre = 0.5 * (lo[k] + hi[k])
    score = 0.0
    for n in nodes:
        name = n.path.lower()
        c = 0.5 * (n.bbox()[0][k] + n.bbox()[1][k]) - centre
        corner = corner_from_name(n.path)
        if WHEEL_WORD.search(name) and corner:
            score += np.sign(c) * (1 if corner[0] == "F" else -1)
        elif re.search(r"headl|windshield|hood|bonnet|front|grill", name):
            score += np.sign(c) * 0.5
        elif re.search(r"taill|brakel|trunk|rear|exhaust|boot", name):
            score -= np.sign(c) * 0.5
    return Frame(ax if score >= 0 else -ax, up)


# ---------------------------------------------------------------- wheels

class Wheel:
    def __init__(self, corner):
        self.corner = corner
        self.parts = []   # (MeshNode, triangle mask per prim or None for the whole node)
        self.lo = None    # ISO bbox
        self.hi = None

    @property
    def centre(self):
        return 0.5 * (self.lo + self.hi)

    @property
    def radius(self):
        return 0.25 * ((self.hi[0] - self.lo[0]) + (self.hi[2] - self.lo[2]))

    @property
    def width(self):
        return self.hi[1] - self.lo[1]

    def add_points(self, iso):
        lo, hi = iso.min(0), iso.max(0)
        self.lo = lo if self.lo is None else np.minimum(self.lo, lo)
        self.hi = hi if self.hi is None else np.maximum(self.hi, hi)


def _is_tyre(lo, hi, ground, size):
    """ISO bbox of a component looks like a tyre or wheel: round in side view, near the ground,
    narrower than its diameter."""
    ext = hi - lo
    d = 0.5 * (ext[0] + ext[2])
    return (0.25 * size < d < 1.6 * size and abs(ext[0] - ext[2]) < 0.12 * d and ext[1] < 0.75 * d
            and lo[2] < ground + 0.15 * d)


def find_wheels(nodes, frame, wheel_size_hint=0.6):
    """Four wheels, each a list of mesh parts. Returns (wheels dict corner->Wheel, method string)
    or (None, reason)."""
    iso_pts = frame.to_iso(np.concatenate([n.points() for n in nodes]))
    ground = iso_pts[:, 2].min()
    length = iso_pts[:, 0].max() - iso_pts[:, 0].min()
    xc = 0.5 * (iso_pts[:, 0].max() + iso_pts[:, 0].min())
    yc = 0.5 * (iso_pts[:, 1].max() + iso_pts[:, 1].min())

    def corner_of(c):
        return ("F" if c[0] > xc else "R") + ("L" if c[1] > yc else "R")

    # 1) named wheel nodes, each small enough to be one wheel.
    named = [n for n in nodes if WHEEL_WORD.search(n.path.lower()) and not NOT_WHEEL.search(n.name.lower())]
    wheels = {}
    ok = bool(named)
    for n in named:
        iso = frame.to_iso(n.points())
        ext = iso.max(0) - iso.min(0)
        if ext[0] > 0.3 * length or ext[1] > 0.6 * (iso_pts[:, 1].max() - iso_pts[:, 1].min()):
            ok = False
            break
        c = 0.5 * (iso.min(0) + iso.max(0))
        corner = corner_of(c)
        w = wheels.setdefault(corner, Wheel(corner))
        w.parts.append((n, None))
        w.add_points(iso)
    if ok and len(wheels) == 4:
        return wheels, "node names"

    # 2) geometric: connected components of every mesh; tyres = round components touching the
    #    ground; every component inside a tyre's bbox (rim, hub, disc) joins that wheel.
    comps = []  # (node, prim index, triangle mask, iso lo, iso hi)
    for n in nodes:
        for pi, (p, t, _) in enumerate(n.prims):
            if len(t) == 0:
                continue
            lab = _components(p, t)
            iso = frame.to_iso(p)
            tri_lo = np.minimum(np.minimum(iso[t[:, 0]], iso[t[:, 1]]), iso[t[:, 2]])
            tri_hi = np.maximum(np.maximum(iso[t[:, 0]], iso[t[:, 1]]), iso[t[:, 2]])
            order = np.argsort(lab, kind="stable")
            ls = lab[order]
            starts = np.flatnonzero(np.r_[True, ls[1:] != ls[:-1]])
            ends = np.r_[starts[1:], len(ls)]
            lo_r = np.minimum.reduceat(tri_lo[order], starts)
            hi_r = np.maximum.reduceat(tri_hi[order], starts)
            for s, e, lo, hi in zip(starts, ends, lo_r, hi_r):
                comps.append((n, pi, order[s:e], lo, hi))
    tyres = [c for c in comps if _is_tyre(c[3], c[4], ground, wheel_size_hint)]
    # one tyre per corner: the largest round thing there (an outer tyre contains its rim)
    best = {}
    for c in tyres:
        corner = corner_of(0.5 * (c[3] + c[4]))
        d = (c[4] - c[3])[0]
        if corner not in best or d > (best[corner][4] - best[corner][3])[0]:
            best[corner] = c
    if len(best) != 4:
        return None, f"geometric: {len(best)} tyre-like parts at the corners"
    wheels = {}
    for corner, t in best.items():
        w = Wheel(corner)
        lo, hi = t[3] - 0.02, t[4] + 0.02
        for c in comps:
            if np.all(c[3] >= lo) and np.all(c[4] <= hi):
                w.parts.append((c[0], (c[1], c[2])))
                w.add_points(np.stack([c[3], c[4]]))
        wheels[corner] = w
    return wheels, "geometric clustering"


def interior_glass_doors(js, nodes):
    names = " ".join(n.path.lower() for n in nodes)
    mats = js.get("materials", [])
    mat_names = " ".join(m.get("name", "").lower() for m in mats)
    blend = sum(1 for m in mats if m.get("alphaMode") == "BLEND")
    interior = bool(re.search(r"interior|seat|dash|steering|cockpit|inside", names + " " + mat_names))
    glass = "named" if re.search(r"glass|window|windshield|windscreen", names) else (
        f"{blend} blend material(s)" if blend else "no")
    doors = bool(re.search(r"door", names))
    return interior, glass, doors
