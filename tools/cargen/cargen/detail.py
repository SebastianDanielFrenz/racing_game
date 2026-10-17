"""Detail parts shared by the cargen cars: multi-element lamps, number plates with recess and characters, grille
lattices in front of a dark mouth, rubber window seals, door-handle recesses.

Everything is a closed shell (lathes with closed profiles, lofts with caps, draped patches with a backing), so
the closed-shell audit still holds. Placement is on the lower body's surface (ClosedCarBody.surface_x/surface_y),
in the same (a, b) outline coordinates as CarBody.patch: (y, z) on the front/rear, (x, z) on the sides.
Fine detail is skipped below quality.detail() 2 (LOD1/LOD2).
"""
import math

from . import ASSETGEN  # noqa: F401  (sys.path setup)
from assetgen import mesh as M
from assetgen.vec import frame_z_to, xform_point, norm, cross, sub, add, mul
from . import quality as Q
from .common import blade_path

FACES = {"front": (1, 0, 0), "rear": (-1, 0, 0), "left": (0, 1, 0), "right": (0, -1, 0)}


def detail_materials(m):
    """Material slots of the detail parts (lens, reflector, housing, seal, plate print)."""
    m.mat("lens_clear", color=(0.85, 0.9, 0.95), alpha=0.18, roughness=0.02, blend=True, role="lens")
    m.mat("lens_red", color=(0.75, 0.02, 0.03), alpha=0.42, roughness=0.03, blend=True, role="lens")
    m.mat("lens_amber", color=(0.95, 0.45, 0.02), alpha=0.45, roughness=0.03, blend=True, role="lens")
    m.mat("lamp_reflector", color=(0.92, 0.92, 0.93), metallic=1.0, roughness=0.06)
    m.mat("lamp_housing", color=(0.07, 0.07, 0.075), metallic=0.3, roughness=0.45)
    m.mat("rubber_seal", color=(0.025, 0.025, 0.027), roughness=0.75)
    m.mat("plate_text", color=(0.02, 0.02, 0.025), roughness=0.5)
    m.mat("plate_band", color=(0.02, 0.15, 0.55), roughness=0.4)
    m.mat("plate_recess", color=(0.04, 0.04, 0.045), roughness=0.8)


# ----------------------------------------------------------------------------- surface placement
def place(body, face, a, b, off):
    if face in ("front", "rear"):
        e = 1 if face == "front" else -1
        return (body.surface_x(a, b, e) + e * off, a, b)
    s = 1 if face == "left" else -1
    return (a, s * (body.surface_y(a, b) + off), b)


def normal_at(body, face, a, b, h=0.01):
    """Outward surface normal at outline point (a, b)."""
    p0 = place(body, face, a, b, 0.0)
    pa = place(body, face, a + h, b, 0.0)
    pb = place(body, face, a, b + h, 0.0)
    n = norm(cross(sub(pa, p0), sub(pb, p0)))
    if sum(n[k] * FACES[face][k] for k in range(3)) < 0:
        n = mul(n, -1)
    return n


def dome(body, outline, mat, face, part="", offset=0.004, height=0.01, rings=4, back=0.004):
    """Domed closed patch (a lamp lens): rows from the centre (f = 0) out to the outline (f = 1) at offset +
    height * (1 - f^2), a flat backing `back` behind the outline and a skirt between them."""
    mb = body.model.mb
    if M.dot(M.newell([place(body, face, a, b, 0) for a, b in outline]), FACES[face]) < 0:
        outline = list(reversed(outline))
    ca = sum(p[0] for p in outline) / len(outline)
    cb = sum(p[1] for p in outline) / len(outline)
    rings = max(2, rings if Q.detail() >= 1 else 2)
    front = []
    for k in range(rings + 1):
        f = k / rings
        front.append([place(body, face, ca + (a - ca) * f, cb + (b - cb) * f, offset + height * (1 - f * f))
                      for a, b in outline])
    backrow = [[place(body, face, ca + (a - ca) * k, cb + (b - cb) * k, offset - back) for a, b in outline]
               for k in (0, 1)]     # hidden backing: one ring is enough
    recs = mb.grid(front, mat, part, closed_v=True)
    mb.grid([backrow[-1], front[-1]], mat, part, closed_v=True, flip=True)
    mb.grid(backrow, mat, part, closed_v=True, flip=True)
    return recs


def inset_outline(outline, k):
    ca = sum(p[0] for p in outline) / len(outline)
    cb = sum(p[1] for p in outline) / len(outline)
    return [(ca + (a - ca) * k, cb + (b - cb) * k) for a, b in outline]


def reflector(m, body, face, a, b, r, emit_mat, part="", offset=0.006, kind="bowl"):
    """One lamp element standing on the lamp's base: a chrome reflector bowl around an emitter.
    kind "bowl": parabolic bowl + bulb; "projector": bowl + glass ball lens + chrome ring; "led": flat emitter
    disc in a shallow bowl."""
    mb = m.mb
    n = normal_at(body, face, a, b)
    c = add(place(body, face, a, b, 0.0), mul(n, offset))
    xf = frame_z_to(n, c)
    d = r * 0.55
    M.lathe(mb, [(0.002, d * 0.15), (r * 0.45, d * 0.32), (r * 0.8, d * 0.68), (r, d), (r * 1.06, d),
                 (r * 1.06, -0.003), (0.002, -0.003)], "lamp_reflector", part, xf, count=24, closed_profile=True)
    if kind == "projector":
        M.ellipsoid(mb, xform_point(xf, (0, 0, d * 0.75)), (r * 0.5, r * 0.5, r * 0.5), "lens_clear", part)
        M.lathe(mb, [(r * 0.52, d * 0.55), (r * 0.62, d * 0.55), (r * 0.62, d * 1.02), (r * 0.52, d * 1.02)],
                "chrome", part, xf, count=24, closed_profile=True)
        M.cylinder(mb, xform_point(xf, (0, 0, d * 0.2)), xform_point(xf, (0, 0, d * 0.35)), r * 0.38, emit_mat, part)
    elif kind == "led":
        M.cylinder(mb, xform_point(xf, (0, 0, d * 0.2)), xform_point(xf, (0, 0, d * 0.45)), r * 0.62, emit_mat, part)
    else:
        M.ellipsoid(mb, xform_point(xf, (0, 0, d * 0.45)), (r * 0.17, r * 0.17, r * 0.26), emit_mat, part)
        M.cylinder(mb, xform_point(xf, (0, 0, d * 0.05)), xform_point(xf, (0, 0, d * 0.3)), r * 0.09, "chrome", part)


def round_lamp(m, center, normal, r, emit_mat, part="", lens_mat="lens_clear", bezel_mat="chrome", depth=None,
               kind="bowl", lens_height=None):
    """Free-standing round lamp (no body surface needed): bezel ring, reflector bowl, emitter, domed lens.
    `center` is the lens base centre, the lamp looks along `normal`."""
    mb = m.mb
    xf = frame_z_to(normal, center)
    d = depth if depth is not None else r * 0.6
    lh = lens_height if lens_height is not None else r * 0.22
    M.lathe(mb, [(r * 0.955, 0.007), (r * 1.10, 0.007), (r * 1.15, 0.002), (r * 1.10, -d - 0.006),
                 (r * 0.955, -d - 0.006)], bezel_mat, part, xf, count=24, closed_profile=True)
    if Q.detail() >= 1:
        M.lathe(mb, [(0.002, -d), (r * 0.45, -d * 0.85), (r * 0.8, -d * 0.45), (r * 0.95, -0.001),
                     (r * 0.95, -d - 0.002), (0.002, -d - 0.002)], "lamp_reflector", part, xf, count=24,
                closed_profile=True)
        if kind == "projector":
            M.ellipsoid(mb, xform_point(xf, (0, 0, -d * 0.45)), (r * 0.42, r * 0.42, r * 0.42), "lens_clear", part)
            M.cylinder(mb, xform_point(xf, (0, 0, -d * 0.95)), xform_point(xf, (0, 0, -d * 0.6)), r * 0.3,
                       emit_mat, part)
        else:
            M.ellipsoid(mb, xform_point(xf, (0, 0, -d * 0.55)), (r * 0.16, r * 0.16, r * 0.24), emit_mat, part)
    else:
        M.cylinder(mb, xform_point(xf, (0, 0, -d * 0.5)), xform_point(xf, (0, 0, -0.002)), r * 0.95, emit_mat, part)
    prof = [(0.0005, lh)]
    for k in range(1, 6):
        f = k / 5
        prof.append((r * 0.97 * f, lh * (1 - f * f) + 0.0005))
    prof += [(r * 0.97, -0.001), (0.0005, -0.001)]
    M.lathe(mb, prof, lens_mat, part, xf, count=24, closed_profile=True)


def lamp(m, body, face, outline, base_mat, lens_mat="lens_clear", elements=(), part="", bezel_mat="trim_black",
         lens_height=0.012, base_offset=0.003):
    """A complete lamp unit: housing (the draped `base_mat` patch on a `bezel_mat` skirt), reflector elements
    [(a, b, r, emit_mat, kind)], and a domed lens over the whole outline."""
    body.patch(outline, base_mat, face, part=part, offset=base_offset, skirt_mat=bezel_mat)
    if Q.detail() >= 1:
        for (a, b, r, emit, kind) in elements:
            reflector(m, body, face, a, b, r, emit, part, offset=base_offset + 0.0015, kind=kind)
    dome(body, inset_outline(outline, 0.985), lens_mat, face, part, offset=base_offset + 0.0025,
         height=lens_height, rings=5)


def optic_ribs(m, body, face, a0, a1, b0, b1, n, mat, part="", offset=0.006, horizontal=True):
    """Fine ribs (a tail-lamp optic pattern) across a rectangle on the surface."""
    if Q.detail() < 2:
        return
    for k in range(n):
        f = (k + 0.5) / n
        if horizontal:
            b = b0 + (b1 - b0) * f
            pts = [place(body, face, a0 + (a1 - a0) * t / 8, b, offset) for t in range(9)]
            w = abs(b1 - b0) / n * 0.45
        else:
            a = a0 + (a1 - a0) * f
            pts = [place(body, face, a, b0 + (b1 - b0) * t / 8, offset) for t in range(9)]
            w = abs(a1 - a0) / n * 0.45
        blade_path(m.mb, pts, w, 0.002, mat, part, normal=FACES[face])


# ----------------------------------------------------------------------------- number plate
SEG7 = {"0": "abcdef", "1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc", "5": "afgcd", "6": "afgedc",
        "7": "abc", "8": "abcdefg", "9": "abcdfg", "R": "efab gc", "G": "afedc", "C": "afed", "P": "abgef",
        "V": "fedcb", "T": "fe g", "A": "efabcg", "E": "afged", "U": "fedcb", "B": "fedcg", " ": ""}


def _seg_rects(ch, x0, y0, w, h, t):
    """7-segment strokes of one character as (a0, b0, a1, b1) rectangles (a = across, b = up)."""
    out = []
    e = 0.0004     # verticals stop short of the bars: overlapping, never sharing a vertex (closed-shell audit)
    for s in SEG7.get(ch, ""):
        if s == "a":
            out.append((x0, y0 + h - t, x0 + w, y0 + h))
        elif s == "g":
            out.append((x0, y0 + h / 2 - t / 2, x0 + w, y0 + h / 2 + t / 2))
        elif s == "d":
            out.append((x0, y0, x0 + w, y0 + t))
        elif s == "f":
            out.append((x0, y0 + h / 2 + e, x0 + t, y0 + h - e))
        elif s == "b":
            out.append((x0 + w - t - e, y0 + h / 2 + e, x0 + w - e, y0 + h - e))
        elif s == "e":
            out.append((x0, y0 + e, x0 + t, y0 + h / 2 - e))
        elif s == "c":
            out.append((x0 + w - t - e, y0 + e, x0 + w - e, y0 + h / 2 - e))
    return out


def plate(m, body, face, ca, cb, w=0.52, h=0.11, text="RG 4520", part="", offset=0.006, recess=True, band=True):
    """EU-format plate in a dark recess: recess panel, plate, blue band, raised characters."""
    rr = M.rrect
    if recess:
        body.patch(rr(w + 0.04, h + 0.035, 0.012, 3, ca, cb), "plate_recess", face, part=part, offset=offset - 0.003,
                   skirt_mat="trim_black")
    body.patch(rr(w, h, 0.008, 2, ca, cb), "plate", face, part=part, offset=offset, skirt_mat="trim_black")
    if Q.detail() < 2:
        return
    # seen from the front the viewer's right is the car's left (+y = +a); from the rear it is -y: text runs
    # along +a on the front and -a on the rear so it reads left to right.
    step = 1 if face == "front" else -1
    if band:
        bw = 0.045
        a_band = ca - step * (w / 2 - bw / 2 - 0.004)
        body.patch(rr(bw, h - 0.012, 0.004, 1, a_band, cb), "plate_band", face, part=part, offset=offset + 0.0015,
                   skirt=0.0)
    ch_h = h * 0.62
    ch_w = ch_h * 0.5
    t = ch_h * 0.14
    pitch = ch_w * 1.45
    n = len(text)
    span = pitch * n - (pitch - ch_w)
    start = ca - step * (span / 2) + (0.0 if not band else step * 0.02)
    for i, chx in enumerate(text):
        x_left = start + step * i * pitch            # viewer-left edge of the glyph, in 'a'
        for (u0, v0, u1, v1) in _seg_rects(chx, 0.0, 0.0, ch_w, ch_h, t):
            a0, a1 = sorted((x_left + step * u0, x_left + step * u1))
            b0, b1 = cb - ch_h / 2 + v0, cb - ch_h / 2 + v1
            body.patch([(a0, b0), (a1, b0), (a1, b1), (a0, b1)], "plate_text", face, part=part,
                       offset=offset + 0.0015, skirt=0.0, rings=1)


# ----------------------------------------------------------------------------- grille lattice
def lattice(m, body, face, a0, a1, b0, b1, na, nb, mat, part="", offset=0.008, bar=0.006, depth=0.012,
            clip=None):
    """Egg-crate grille: `na` bars across a and `nb` bars across b standing on the surface in front of the dark
    mouth patch. clip(a, b) -> bool keeps only the points inside an outline (for non-rectangular mouths)."""
    if Q.detail() < 1:
        return
    nrm = FACES[face]
    sa, sb = (24, 12) if clip else (16, 6)          # finer sampling where an outline clips the bars
    for k in range(nb):
        b = b0 + (b1 - b0) * (k + 0.5) / nb
        pts = [(a0 + (a1 - a0) * t / sa, b) for t in range(sa + 1)]
        _clipped_blade(m, body, face, pts, clip, bar, depth, mat, part, offset, nrm, along_b=False)
    for k in range(na):
        a = a0 + (a1 - a0) * (k + 0.5) / na
        pts = [(a, b0 + (b1 - b0) * t / sb) for t in range(sb + 1)]
        _clipped_blade(m, body, face, pts, clip, bar, depth, mat, part, offset, nrm, along_b=True)


def _clipped_blade(m, body, face, pts, clip, bar, depth, mat, part, offset, nrm, along_b):
    runs, cur = [], []
    for p in pts:
        if clip is None or clip(*p):
            cur.append(p)
        else:
            if len(cur) >= 2:
                runs.append(cur)
            cur = []
    if len(cur) >= 2:
        runs.append(cur)
    for run in runs:
        path = [place(body, face, a, b, offset) for a, b in run]
        # blade: thin across the run (bar), deep along the face normal (depth)
        blade_path(m.mb, path, depth, bar, mat, part, normal=_perp(face, along_b))


def _perp(face, along_b):
    """blade_path's `normal` is the direction of the blade's THIN side."""
    if face in ("front", "rear"):
        return (0.0, 0.0, 1.0) if not along_b else (0.0, 1.0, 0.0)
    return (0.0, 0.0, 1.0) if not along_b else (1.0, 0.0, 0.0)


# ----------------------------------------------------------------------------- greenhouse seals
def green_point(body, x, seg, pos, side):
    """Point on the greenhouse surface at station x, inside segment `seg` ('side' or 'roof') at 0..1 position
    (the same position CarBody.green_segment reports), on side +1/-1 (roof: pos 0 = outer edge)."""
    loop = body._green_section(x)
    (_, nb), (_, ns), (_, nc), (_, nr) = body.GREEN_SEGS
    H = len(loop) // 2 + 1
    half = loop[:H]
    start = {"side": nb, "corner": nb + ns, "roof": nb + ns + nc}[seg]
    cnt = {"side": ns, "corner": nc, "roof": nr}[seg]
    t = start + pos * cnt
    i = min(int(t), len(half) - 2)
    f = t - i
    p = tuple(half[i][k] + (half[i + 1][k] - half[i][k]) * f for k in range(3))
    return (p[0], side * abs(p[1]), p[2])


def seal_loop(m, body, x0, x1, pos0, pos1, side, part="", r=0.0045, mat="rubber_seal", seg="side", off=0.002,
              closed=True, n=None):
    """Rubber seal round a greenhouse window (a rectangle x0..x1, pos0..pos1 in `seg`), standing `off` proud."""
    if Q.detail() < 1:
        return
    n = n or max(6, int((x1 - x0) / 0.04))
    nz = 6
    pts = ([(x0 + (x1 - x0) * k / n, pos0) for k in range(n)] + [(x1, pos0 + (pos1 - pos0) * k / nz) for k in range(nz)]
           + [(x1 - (x1 - x0) * k / n, pos1) for k in range(n)] + [(x0, pos1 - (pos1 - pos0) * k / nz)
                                                                   for k in range(nz)])
    path = []
    for x, p in pts:
        q = green_point(body, x, seg, p, side)
        path.append((q[0], q[1] + side * off * (1 if seg == "side" else 0.5), q[2] + (off if seg != "side" else 0)))
    if not closed:
        path = path[:n + 1]
    M.tube(m.mb, path, r, mat, part, count=8, closed=closed)


def seal_line(m, body, xs_pos, side, part="", r=0.0045, mat="rubber_seal", seg="side", off=0.002):
    """Open seal along a list of (x, pos) points."""
    if Q.detail() < 1:
        return
    path = []
    for x, p in xs_pos:
        q = green_point(body, x, seg, p, side)
        path.append((q[0], q[1] + side * off, q[2]))
    M.tube(m.mb, path, r, mat, part, count=8)


def handle_recess(m, body, face, a, b, w, h, part=""):
    """Dark pocket behind a door pull handle."""
    body.patch(M.rrect(w, h, min(w, h) * 0.45, 3, a, b), "trim_black", face, part=part, offset=0.0012, skirt=0.0)


def plan_offset_path(body, end, z, half, wrap, off, n=40):
    """Smooth plan-view path at height z, `off` outside the body: across the front (end +1) or rear (-1) from
    -half to +half, wrapping `wrap` metres back along both sides (a bumper bar, a valance). Built by offsetting
    the body's own outline along its 2D normal, then resampled evenly by arc length."""
    xe = [body.surface_x(s * half, z, end) for s in (-1, 1)]
    outline = []
    for k in range(9):                       # side, -y
        d = wrap * (1 - k / 8)
        x = xe[0] - end * d
        outline.append((x, -body.surface_y(x, z)))
    for k in range(1, 40):
        y = -half + 2 * half * k / 40
        outline.append((body.surface_x(y, z, end), y))
    for k in range(9):
        d = wrap * k / 8
        x = xe[1] - end * d
        outline.append((x, body.surface_y(x, z)))
    if end < 0:
        outline = outline[::-1]
    pts = []
    for i, (x, y) in enumerate(outline):
        a = outline[max(0, i - 1)]
        b = outline[min(len(outline) - 1, i + 1)]
        tx, ty = b[0] - a[0], b[1] - a[1]
        L = math.hypot(tx, ty) or 1.0
        nx, ny = ty / L, -tx / L                  # right-hand normal: outward for both ends' travel direction
        pts.append((x + nx * off, y + ny * off))
    # smooth twice and resample evenly
    for _ in range(2):
        pts = [pts[0]] + [((pts[i - 1][0] + 2 * pts[i][0] + pts[i + 1][0]) / 4,
                           (pts[i - 1][1] + 2 * pts[i][1] + pts[i + 1][1]) / 4) for i in range(1, len(pts) - 1)] + [pts[-1]]
    seg = [math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    total = sum(seg)
    out, i, acc = [], 0, 0.0
    for k in range(n + 1):
        t = total * k / n
        while i < len(seg) - 1 and acc + seg[i] < t:
            acc += seg[i]
            i += 1
        f = (t - acc) / seg[i] if seg[i] > 0 else 0.0
        out.append((pts[i][0] + (pts[i + 1][0] - pts[i][0]) * f, pts[i][1] + (pts[i + 1][1] - pts[i][1]) * f, z))
    return out


def plate_free(m, center, normal, w, h, text, part="", up=(0.0, 0.0, 1.0), plate_mat="plate", text_mat="plate_text",
               frame_mat="chrome"):
    """Free-standing plate (not draped on a body): a thin rounded slab facing `normal`, a rolled frame, raised
    7-segment characters."""
    mb = m.mb
    z = norm(normal)
    x = norm(cross(up, z))          # viewer's right when looking at the plate
    y = cross(z, x)
    xf = ((x[0], y[0], z[0], center[0]), (x[1], y[1], z[1], center[1]), (x[2], y[2], z[2], center[2]))
    M.rounded_box(mb, (0, 0, 0), (0.004, w, h), 0.0015, plate_mat, part, seg=1,
                  xf=((z[0], x[0], y[0], center[0]), (z[1], x[1], y[1], center[1]), (z[2], x[2], y[2], center[2])))
    path = [(-w / 2 - 0.002, -h / 2 - 0.002), (w / 2 + 0.002, -h / 2 - 0.002), (w / 2 + 0.002, h / 2 + 0.002),
            (-w / 2 - 0.002, h / 2 + 0.002)]
    loop = []
    for i in range(4):
        a, b = path[i], path[(i + 1) % 4]
        for k in range(6):
            t = k / 6
            loop.append(xform_point(xf, (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, 0.0015)))
    M.tube(mb, loop, 0.003, frame_mat, part, count=8, closed=True)
    if Q.detail() < 2:
        return
    ch_h = h * 0.6
    ch_w = ch_h * 0.5
    t = ch_h * 0.14
    pitch = ch_w * 1.45
    span = pitch * len(text) - (pitch - ch_w)
    for i, ch in enumerate(text):
        x0 = -span / 2 + i * pitch
        for (u0, v0, u1, v1) in _seg_rects(ch, x0, -ch_h / 2, ch_w, ch_h, t):
            c = ((u0 + u1) / 2, (v0 + v1) / 2, 0.0025)
            M.box(mb, (0, 0, 0), (u1 - u0, v1 - v0, 0.0015), text_mat, part, xf=compose_t(xf, c))


def compose_t(xf, c):
    from assetgen.vec import compose, translate
    return compose(xf, translate(c))
