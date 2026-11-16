"""Small exact 2D helpers for the generator's cheap screens (no outside library).

Polygons are (n, 2) arrays, closed implicitly. A shape is a list of polygons
(a section may fall apart into several). Distances are exact for polygons;
round outlines are sampled finely by whoever builds them.
"""
from __future__ import annotations

import math

import numpy as np


def seg_point(p, a, b) -> float:
    p, a, b = (np.asarray(x, float) for x in (p, a, b))
    ab = b - a
    L = float(ab @ ab)
    t = 0.0 if L == 0 else float(np.clip((p - a) @ ab / L, 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def inside(p, poly) -> bool:
    x, y = float(p[0]), float(p[1])
    n = len(poly)
    c = False
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xi = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if xi > x:
                c = not c
    return c


def _segs_cross(a, b, c, d) -> bool:
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return (o1 * o2 < 0) and (o3 * o4 < 0)


def _pt_seg(Pts, A, B):
    """Distances from every point (n,2) to every segment A[j]-B[j] (m,2): (n, m)."""
    AB = B - A
    L = np.einsum("ij,ij->i", AB, AB)
    L = np.where(L == 0, 1e-30, L)
    AP = Pts[:, None, :] - A[None, :, :]
    t = np.clip(np.einsum("nmk,mk->nm", AP, AB) / L[None, :], 0.0, 1.0)
    D = AP - t[..., None] * AB[None, :, :]
    return np.sqrt(np.einsum("nmk,nmk->nm", D, D))


def _cross_any(P, Q) -> bool:
    A, B = P, np.roll(P, -1, axis=0)
    C, D = Q, np.roll(Q, -1, axis=0)

    def orient(p, q, r):  # broadcast: p,q (n,1,2) r (1,m,2)
        return (q[..., 0] - p[..., 0]) * (r[..., 1] - p[..., 1]) - (q[..., 1] - p[..., 1]) * (r[..., 0] - p[..., 0])

    o1 = orient(A[:, None], B[:, None], C[None])
    o2 = orient(A[:, None], B[:, None], D[None])
    o3 = orient(C[None], D[None], A[:, None])
    o4 = orient(C[None], D[None], B[:, None])
    return bool(np.any((o1 * o2 < 0) & (o3 * o4 < 0)))


def poly_dist(P, Q) -> float:
    """Smallest distance between two polygons; 0 if they overlap."""
    P, Q = np.asarray(P, float), np.asarray(Q, float)
    lo = np.maximum(P.min(0), Q.min(0))
    hi = np.minimum(P.max(0), Q.max(0))
    gap = float(np.max(np.maximum(lo - hi, 0.0)))
    if gap > 0 and np.all(lo - hi > 0):
        pass
    if np.all(hi >= lo):  # boxes overlap: they may overlap
        if inside(P[0], Q) or inside(Q[0], P) or _cross_any(P, Q):
            return 0.0
    d1 = _pt_seg(P, Q, np.roll(Q, -1, axis=0)).min()
    d2 = _pt_seg(Q, P, np.roll(P, -1, axis=0)).min()
    return float(min(d1, d2))


def shape_dist(A: list, B: list) -> float:
    return min((poly_dist(p, q) for p in A for q in B), default=math.inf)


def stadium(P, Q, r: float, n: int = 24) -> np.ndarray:
    """The outline of a link: two round ends of radius r at P and Q, joined by tangents."""
    P, Q = np.asarray(P, float), np.asarray(Q, float)
    a = math.atan2(*(Q - P)[::-1])
    out = []
    for k in range(n + 1):
        th = a + math.pi / 2 + math.pi * k / n
        out.append(P + r * np.array([math.cos(th), math.sin(th)]))
    for k in range(n + 1):
        th = a - math.pi / 2 + math.pi * k / n
        out.append(Q + r * np.array([math.cos(th), math.sin(th)]))
    return np.array(out)


def circle(C, r: float, n: int = 32) -> np.ndarray:
    C = np.asarray(C, float)
    return np.array([C + r * np.array([math.cos(2 * math.pi * k / n), math.sin(2 * math.pi * k / n)]) for k in range(n)])


def rect(s0, s1, t0, t1) -> np.ndarray:
    return np.array([(s0, t0), (s1, t0), (s1, t1), (s0, t1)], float)


def move(poly, ang: float, shift) -> np.ndarray:
    """Turn ``poly`` by ``ang`` (rad) about the origin, then shift it."""
    c, s = math.cos(ang), math.sin(ang)
    R = np.array([[c, -s], [s, c]])
    return np.asarray(poly, float) @ R.T + np.asarray(shift, float)


def column(poly, s_: float) -> list:
    """The polygon's material on the vertical line at s: [(t_lo, t_hi)]."""
    A, B = np.asarray(poly, float), np.roll(np.asarray(poly, float), -1, axis=0)
    sel = ((A[:, 0] <= s_) & (s_ < B[:, 0])) | ((B[:, 0] <= s_) & (s_ < A[:, 0]))
    a_, b_ = A[sel], B[sel]
    ts = sorted(a_[:, 1] + (s_ - a_[:, 0]) * (b_[:, 1] - a_[:, 1]) / (b_[:, 0] - a_[:, 0]))
    return [(ts[i], ts[i + 1]) for i in range(0, len(ts) - 1, 2)]


def above(Q: list, P: list, step: float = 1.0) -> bool:
    """True if some part of shape Q lies straight above some part of shape P (on a common vertical line):
    then P cannot come down from above past Q."""
    for p in P:
        for q in Q:
            lo = max(float(np.min(p[:, 0])), float(np.min(q[:, 0])))
            hi = min(float(np.max(p[:, 0])), float(np.max(q[:, 0])))
            if hi < lo:
                continue
            for s_ in np.arange(lo + 1e-3, hi, step):
                cp, cq = column(p, s_), column(q, s_)
                if any(qq[0] >= pp[1] - 1e-6 for pp in cp for qq in cq):
                    return True
    return False
