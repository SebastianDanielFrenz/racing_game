"""Closed-shell audit of an exported model, per node (numpy port of physics_sim's
hyper_topology_check.audit, same 1 um weld and the same five counts).

boundary_edges / nonmanifold_edges / degenerate_triangles / duplicate_faces /
inconsistent_winding_edges must all be 0 for every node: every node is a set
of closed, consistently wound shells. Material borders are not holes (the
weld ignores materials). This is visual closure, not a fluid-solid proof.
"""
import numpy as np


def audit(tris, tol=1e-6):
    """tris: (n, 3, 3) float array of triangle corners."""
    n = len(tris)
    if n == 0:
        return dict(triangles=0, boundary_edges=0, nonmanifold_edges=0, degenerate_triangles=0,
                    duplicate_faces=0, inconsistent_winding_edges=0)
    q = np.round(tris / tol).astype(np.int64)
    flat = q.reshape(-1, 3)
    _, vid = np.unique(flat, axis=0, return_inverse=True)
    vid = vid.reshape(n, 3)
    u = tris[:, 1] - tris[:, 0]
    v = tris[:, 2] - tris[:, 0]
    area2 = (np.cross(u, v) ** 2).sum(1)
    degenerate = (vid[:, 0] == vid[:, 1]) | (vid[:, 1] == vid[:, 2]) | (vid[:, 0] == vid[:, 2]) | (area2 < 1e-24)
    faces = np.sort(vid, axis=1)
    _, fcount = np.unique(faces, axis=0, return_counts=True)
    a = vid.reshape(-1)
    b = np.roll(vid, -1, axis=1).reshape(-1)
    und = np.stack([np.minimum(a, b), np.maximum(a, b)], 1)
    ukeys, uinv, ucount = np.unique(und, axis=0, return_inverse=True, return_counts=True)
    # winding: for a manifold edge the two uses must go opposite ways
    forward = (a < b).astype(np.int64)
    fsum = np.bincount(uinv.reshape(-1), weights=forward, minlength=len(ukeys))
    inconsistent = (ucount == 2) & (fsum != 1)
    return dict(triangles=int(n), boundary_edges=int((ucount == 1).sum()),
                nonmanifold_edges=int((ucount > 2).sum()), degenerate_triangles=int(degenerate.sum()),
                duplicate_faces=int((fcount - 1).sum()), inconsistent_winding_edges=int(inconsistent.sum()))


def is_closed(rep):
    return all(rep[k] == 0 for k in ("boundary_edges", "nonmanifold_edges", "degenerate_triangles",
                                      "duplicate_faces", "inconsistent_winding_edges"))
