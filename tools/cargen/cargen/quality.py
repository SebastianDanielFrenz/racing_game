"""Tessellation quality (level of detail) for the cargen cars.

One parameter set per LOD drives every curved surface:

- `tol` (m): chord tolerance. A revolved or round section of radius r gets
  n = pi / acos(1 - tol / r) segments (rounded up to a multiple of 4), so a
  tyre and a 6 mm switch stalk are equally smooth on screen;
- `body_step` / `body_around`: station spacing along x and points around the
  lofted lower body (and `green_step` for the greenhouse);
- `detail`: 2 = everything (garage / close-up), 1 = no small hidden parts
  (interior fine detail, tread blocks), 0 = silhouette only (far).

`install()` routes assetgen's primitives (lathe, tube, cylinder, circle,
rrect, rounded_box, ellipsoid) through these counts once; the active level is
set with `use(level)` before a car is built. Six-sided primitives (hex nuts,
bolt heads) keep their count: that is shape, not tessellation.
"""
import math

from . import ASSETGEN  # noqa: F401  (sys.path setup)
from assetgen import mesh as M
from assetgen.vec import xform_point

LODS = {
    0: dict(name="lod0", min_seg=8, tol=0.00035, max_seg=128, body_step=0.038, body_around=100, green_step=0.036, detail=2,
            use="garage, showroom and close-up camera (< 15 m)"),
    1: dict(name="lod1", min_seg=6, tol=0.0012, max_seg=64, body_step=0.06, body_around=76, green_step=0.06, detail=1,
            use="driving camera and nearby traffic (15 - 60 m)"),
    2: dict(name="lod2", min_seg=4, tol=0.006, max_seg=24, body_step=0.12, body_around=44, green_step=0.09, detail=0,
            use="distant traffic (> 60 m)"),
}

cur = dict(LODS[0])
cur["level"] = 0


def use(level):
    cur.clear()
    cur.update(LODS[level])
    cur["level"] = level


def level():
    return cur["level"]


def detail():
    return cur["detail"]


class coarse:
    """with coarse(k): build parts at k times the chord tolerance (interior seen through glass, hidden parts)."""

    def __init__(self, k):
        self.k = k

    def __enter__(self):
        self.prev = cur.get("scale", 1.0)
        cur["scale"] = self.prev * self.k

    def __exit__(self, *a):
        cur["scale"] = self.prev


def seg(r, lo=8, full=True):
    """Segments for a full circle (full=True) or a quarter (full=False) of radius r."""
    r = max(abs(r), 1e-4)
    tol = min(cur["tol"] * cur.get("scale", 1.0), 0.5 * r)
    n = math.pi / math.acos(1.0 - tol / r)
    q = 4 if cur["level"] == 0 else 2
    lo = min(lo, cur["min_seg"])
    n = max(lo, min(cur["max_seg"], int(math.ceil(n / q)) * q))
    return n if full else max(1, n // 4)


def count_for(r, nominal):
    """Replacement for a hand-picked `count`: radius driven, six-sided shapes kept."""
    if nominal is not None and nominal <= 6:
        return nominal
    return seg(r)


# --------------------------------------------------------------------------- assetgen primitive routing
_orig = {}


def install():
    if _orig:
        return
    for name in ("lathe", "tube", "cylinder", "circle", "rrect", "rounded_box", "ellipsoid"):
        _orig[name] = getattr(M, name)

    def lathe(mb, profile, material, part="", axis_xf=None, count=32, **kw):
        if axis_xf is None:
            from assetgen.vec import IDENTITY
            axis_xf = IDENTITY
        r = max(abs(p[0]) for p in profile)
        return _orig["lathe"](mb, profile, material, part, axis_xf, count=count_for(r, count), **kw)

    def tube(mb, path, radius, material, part="", count=16, **kw):
        r = max(radius) if isinstance(radius, (list, tuple)) else radius
        return _orig["tube"](mb, path, radius, material, part, count=count_for(r, count), **kw)

    def cylinder(mb, p0, p1, r, material, part="", count=24, **kw):
        return _orig["cylinder"](mb, p0, p1, r, material, part, count=count_for(r, count), **kw)

    def circle(r, count=24, ca=0.0, cb=0.0):
        return _orig["circle"](r, count_for(r, count), ca, cb)

    def rrect(w, h, r, seg_=4, ca=0.0, cb=0.0):
        return _orig["rrect"](w, h, r, max(seg_ if seg_ <= 1 else 2, seg(r, full=False)), ca, cb)

    def rounded_box(mb, center, size, r, material, part="", xf=None, seg_=3, classify=None, **kw):
        # assetgen's rounded_box, with the corner segments from the tolerance (the rounded-rectangle sections
        # are made with the ORIGINAL rrect so the count is not applied twice)
        from assetgen.vec import IDENTITY
        if "seg" in kw:
            seg_ = kw.pop("seg")
        xf = IDENTITY if xf is None else xf
        sx, sy, sz = size
        r = min(r, sx / 2 - 1e-4, sy / 2 - 1e-4, sz / 2 - 1e-4)
        q = max(1 if seg_ <= 1 else 2, seg(r, lo=4, full=False))
        secs = []
        stations = []
        for k in range(q + 1):
            t = (math.pi / 2) * (1 - k / q)
            stations.append((-sx / 2 + r - r * math.sin(t), r - r * math.cos(t)))
        for k in range(q + 1):
            t = (math.pi / 2) * (k / q)
            stations.append((sx / 2 - r + r * math.sin(t), r - r * math.cos(t)))
        for x, inset in stations:
            w, h = max(sy - 2 * inset, 1e-3), max(sz - 2 * inset, 1e-3)
            rr = max(r - inset, 1e-4)
            sec = _orig["rrect"](w, h, rr, q)
            secs.append([xform_point(xf, (center[0] + x, center[1] + a, center[2] + b)) for (a, b) in sec])
        return M.loft(mb, secs, material, part, classify=classify)

    def ellipsoid(mb, center, radii, material, part="", xf=None, count=24, rings=12):
        from assetgen.vec import IDENTITY
        r = max(radii)
        n = count_for(r, count)
        return _orig["ellipsoid"](mb, center, radii, material, part, IDENTITY if xf is None else xf, count=n,
                                  rings=max(rings, n // 2))

    M.lathe, M.tube, M.cylinder, M.circle = lathe, tube, cylinder, circle
    M.rrect, M.rounded_box, M.ellipsoid = rrect, rounded_box, ellipsoid


install()
