"""Vertex normals by angle ("auto smooth"), computed after the whole model is built.

assetgen smooths a grid's normals over every face of one (part, material), so a
lathe profile with a sharp step (rim lip, disc edge, groove wall) or a thin
shell's edge gets one averaged normal and shades as a dark smear. Here each
triangle corner gets the area-weighted sum of the normals of the faces around
the (position-welded) vertex that lie within `angle_deg` of its own face; a
vertex is split where those sums differ. Positions, UVs and the triangle order
are untouched (flow selection ranges stay valid); only vertices are duplicated.
UV-seam duplicates are welded for the computation, so a lathe's wrap seam does
not show.
"""
import numpy as np


def smooth_prim(prim, angle_deg=48.0, weld=1e-6):
    if not prim.idx:
        return
    P = np.asarray(prim.pos, dtype=np.float64)
    UV = np.asarray(prim.uv, dtype=np.float64)
    I = np.asarray(prim.idx, dtype=np.int64).reshape(-1, 3)
    nt = len(I)
    q = np.round(P / weld).astype(np.int64)
    _, wid = np.unique(q, axis=0, return_inverse=True)
    wid = wid.reshape(-1)
    a, b, c = P[I[:, 0]], P[I[:, 1]], P[I[:, 2]]
    fn = np.cross(b - a, c - a)                     # area weighted
    L = np.linalg.norm(fn, axis=1)
    fu = fn / np.where(L > 0, L, 1.0)[:, None]
    # corners
    cf = np.repeat(np.arange(nt), 3)                 # face of each corner
    cw = wid[I.reshape(-1)]                          # welded vertex of each corner
    order = np.lexsort((cf, cw))
    sw, sf = cw[order], cf[order]
    acc = fn[sf].copy()                              # own face
    cos_t = np.cos(np.radians(angle_deg))
    n = len(sw)
    d = 1
    while True:
        same = np.zeros(n, dtype=bool)
        same[:n - d] = sw[:n - d] == sw[d:]
        if not same.any():
            break
        i = np.nonzero(same)[0]
        j = i + d
        ok = (fu[sf[i]] * fu[sf[j]]).sum(1) > cos_t
        i, j = i[ok], j[ok]
        np.add.at(acc, i, fn[sf[j]])
        np.add.at(acc, j, fn[sf[i]])
        d += 1
    cn = np.empty_like(acc)
    cn[order] = acc
    ln = np.linalg.norm(cn, axis=1)
    bad = ln < 1e-20
    cn = cn / np.where(bad, 1.0, ln)[:, None]
    cn[bad] = fu[cf[bad]]
    # new vertex per (original vertex, quantised normal)
    vid = I.reshape(-1)
    key = np.concatenate([vid[:, None], np.round(cn * 1e5).astype(np.int64)], axis=1)
    uk, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    inv = inv.reshape(-1)
    # keep a deterministic, cache-friendly vertex order: by first use
    rank = np.argsort(np.argsort(first, kind="stable"), kind="stable")
    new_index = rank[inv]
    nv = len(uk)
    src = np.empty(nv, dtype=np.int64)
    src[new_index] = vid
    nrm = np.empty((nv, 3))
    nrm[new_index] = cn
    prim.pos = [tuple(map(float, p)) for p in P[src]]
    prim.uv = [tuple(map(float, t)) for t in UV[src]]
    prim.nrm = [tuple(map(float, v)) for v in nrm]
    prim.idx = [int(k) for k in new_index]


def smooth_model(mb, angle_deg=48.0):
    for key in sorted(mb.prims):
        smooth_prim(mb.prims[key], angle_deg)
