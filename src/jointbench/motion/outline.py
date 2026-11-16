"""A part's outline at one height, and the clearance between two outlines.

Used for contact search and as a sound shortcut before exact booleans for
parts that are flat plates (every side face straight up and down): their
cross-section is the same at every height between two horizontal faces, so
two such parts overlap only if their outlines at that height do.
"""
from __future__ import annotations

import math

import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_SurfaceOfExtrusion
from OCP.gp import gp_Dir, gp_Pnt

from . import kin

DEFLECTION = 1e-4  # mm: outlines follow the exact curves this closely


class Outline:
    """A part's boundary at one height, as straight segments within
    ``DEFLECTION`` mm of the exact BRep curves (OpenCASCADE's deflection
    sampling of the section edges). Queries are in the part's own frame."""

    def __init__(self, solid, z: float, deflection: float = DEFLECTION):
        self.deflection = deflection
        from OCP.BRepAdaptor import BRepAdaptor_Curve
        from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
        from OCP.GCPnts import GCPnts_QuasiUniformDeflection
        from OCP.gp import gp_Pln
        from OCP.TopAbs import TopAbs_EDGE
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopoDS import TopoDS
        segs = []

        def add(e):
            c = BRepAdaptor_Curve(e)
            q = GCPnts_QuasiUniformDeflection(c, deflection)
            pts = [c.Value(q.Parameter(i)) for i in range(1, q.NbPoints() + 1)]
            for a, b in zip(pts, pts[1:]):
                segs.append(((a.X(), a.Y()), (b.X(), b.Y())))

        if z_levels(solid) is not None:
            # a flat plate: the outline at height z is the bottom edge of each
            # side face that spans z (no sectioning needed)
            for f in solid.faces():
                ad = BRepAdaptor_Surface(f.wrapped)
                if ad.GetType() == GeomAbs_Plane and abs(abs(ad.Plane().Axis().Direction().Z()) - 1) < 1e-9:
                    continue  # horizontal
                bb = f.bounding_box(optimal=True)
                if not (bb.min.Z - 1e-7 <= z <= bb.max.Z + 1e-7):
                    continue
                zs = [[e.position_at(t).Z for t in (0.0, 0.37, 0.71, 1.0)] for e in f.edges()]
                zlo = min(min(z_) for z_ in zs)
                low = [e for e, z_ in zip(f.edges(), zs) if max(abs(v - zlo) for v in z_) < 1e-7]
                for e in low:
                    add(e.wrapped)
        else:
            sec = BRepAlgoAPI_Section(solid.wrapped, gp_Pln(gp_Pnt(0, 0, z), gp_Dir(0, 0, 1)))
            sec.Build()
            ex = TopExp_Explorer(sec.Shape(), TopAbs_EDGE)
            while ex.More():
                add(TopoDS.Edge(ex.Current()))
                ex.Next()
        self.S = np.array(segs, dtype=float).reshape(-1, 2, 2)
        self.A, self.B = self.S[:, 0], self.S[:, 1]
        self.V = self.B - self.A
        self.L2 = np.maximum(np.sum(self.V * self.V, axis=1), 1e-30)
        self.lo = np.minimum(self.A, self.B)
        self.hi = np.maximum(self.A, self.B)

    def closest(self, p, idx=None):
        A, V, L2 = (self.A, self.V, self.L2) if idx is None else (self.A[idx], self.V[idx], self.L2[idx])
        t = np.clip(((p[0] - A[:, 0]) * V[:, 0] + (p[1] - A[:, 1]) * V[:, 1]) / L2, 0, 1)
        Q = A + t[:, None] * V
        d = np.hypot(Q[:, 0] - p[0], Q[:, 1] - p[1])
        k = int(np.argmin(d))
        return float(d[k]), (float(Q[k, 0]), float(Q[k, 1]))

    def dists(self, P):
        """Distance from each point of P (n,2) to the outline."""
        A, V, L2 = self.A[None], self.V[None], self.L2[None]
        D = P[:, None, :] - A
        t = np.clip(np.sum(D * V, axis=2) / L2, 0, 1)
        Q = A + t[..., None] * V
        d = np.hypot(*(Q - P[:, None, :]).transpose(2, 0, 1))
        k = np.argmin(d, axis=1)
        return d[np.arange(len(P)), k], Q[np.arange(len(P)), k]

    def dists_near(self, P, reach):
        """As dists, over the segments within ``reach`` of P's bounding box."""
        lo, hi = P.min(axis=0) - reach, P.max(axis=0) + reach
        k = np.nonzero((self.hi[:, 0] >= lo[0]) & (self.lo[:, 0] <= hi[0]) & (self.hi[:, 1] >= lo[1]) & (self.lo[:, 1] <= hi[1]))[0]
        if len(k) == 0:
            return np.full(len(P), reach), np.zeros((len(P), 2))
        A, V, L2 = self.A[k][None], self.V[k][None], self.L2[k][None]
        D = P[:, None, :] - A
        t = np.clip(np.sum(D * V, axis=2) / L2, 0, 1)
        Q = A + t[..., None] * V
        d = np.hypot(*(Q - P[:, None, :]).transpose(2, 0, 1))
        j = np.argmin(d, axis=1)
        r = np.arange(len(P))
        return np.minimum(d[r, j], reach), Q[r, j]

    def inside(self, P):
        """Even-odd test for points P (n,2) against the closed outline(s)."""
        P = np.atleast_2d(P)
        # only segments spanning the points' heights and reaching right of them can cut the ray
        k = np.nonzero((self.hi[:, 1] >= P[:, 1].min()) & (self.lo[:, 1] <= P[:, 1].max())
                       & (self.hi[:, 0] >= P[:, 0].min()))[0]
        if len(k) == 0:
            return np.zeros(len(P), dtype=bool)
        y0, y1 = self.A[k, 1][None], self.B[k, 1][None]
        x0, x1 = self.A[k, 0][None], self.B[k, 0][None]
        py, px = P[:, 1:2], P[:, 0:1]
        crosses = (y0 > py) != (y1 > py)
        with np.errstate(divide="ignore", invalid="ignore"):
            xi = x0 + (py - y0) * (x1 - x0) / (y1 - y0)
        return (np.sum(crosses & (px < xi), axis=1) % 2) == 1

    def vertices(self):
        return self.A


def _apply_np(pose, P):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return P @ np.array([[c, s], [-s, c]]) + np.array(pose[:2])


def outline_gap(oa: "Outline", pa, ob: "Outline", pb, reach: float = 5.0):
    """Clearance between two outlines placed at plane poses pa and pb:
    (gap, point on a, point on b) in plane coordinates; gap 0 if they overlap.
    Only parts of the outlines within ``reach`` mm of each other are searched;
    a clearance larger than that is reported as ``reach``."""
    if len(oa.S) == 0 or len(ob.S) == 0:
        return reach, None, None
    rel_ab = kin.compose(kin.inverse(pb), pa)  # a's frame -> b's frame
    rel_ba = kin.inverse(rel_ab)
    Va = _apply_np(rel_ab, oa.vertices())  # a's vertices in b's frame
    Vb = _apply_np(rel_ba, ob.vertices())  # b's vertices in a's frame
    blo, bhi = ob.lo.min(axis=0) - reach, ob.hi.max(axis=0) + reach
    alo, ahi = oa.lo.min(axis=0) - reach, oa.hi.max(axis=0) + reach
    sa = np.nonzero(np.all((Va >= blo) & (Va <= bhi), axis=1))[0]
    sb = np.nonzero(np.all((Vb >= alo) & (Vb <= ahi), axis=1))[0]
    if len(sa) == 0 and len(sb) == 0:
        return reach, None, None
    if (len(sa) and ob.inside(Va[sa]).any()) or (len(sb) and oa.inside(Vb[sb]).any()) or _crossing(oa, ob, rel_ab):
        return 0.0, None, None
    best = (reach, None, None)
    if len(sa):
        da, qa = ob.dists_near(Va[sa], reach)
        i = int(np.argmin(da))
        if da[i] < best[0]:
            best = (float(da[i]), kin.apply(pb, tuple(Va[sa][i])), kin.apply(pb, tuple(qa[i])))
    if len(sb):
        db, qb = oa.dists_near(Vb[sb], reach)
        i = int(np.argmin(db))
        if db[i] < best[0]:
            best = (float(db[i]), kin.apply(pa, tuple(qb[i])), kin.apply(pa, tuple(Vb[sb][i])))
    return best


def _crossing(oa, ob, rel_ab) -> bool:
    A1 = _apply_np(rel_ab, oa.A)
    B1 = _apply_np(rel_ab, oa.B)
    lo = np.minimum(A1, B1).min(axis=0)
    hi = np.maximum(A1, B1).max(axis=0)
    near = np.nonzero((ob.hi[:, 0] >= lo[0]) & (ob.lo[:, 0] <= hi[0]) & (ob.hi[:, 1] >= lo[1]) & (ob.lo[:, 1] <= hi[1]))[0]
    if len(near) == 0:
        return False
    C, D = ob.A[near], ob.B[near]

    def orient(P, Q, R):
        return (Q[..., 0] - P[..., 0]) * (R[..., 1] - P[..., 1]) - (Q[..., 1] - P[..., 1]) * (R[..., 0] - P[..., 0])

    a, b = A1[:, None], B1[:, None]
    c, d = C[None], D[None]
    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    return bool(np.any((o1 * o2 < 0) & (o3 * o4 < 0)))


def z_levels(solid):
    """Heights of the horizontal faces if every other face is vertical, else None."""
    from OCP.BRepGProp import BRepGProp_Face
    from OCP.gp import gp_Vec
    zs = set()
    for f in solid.faces():
        ad = BRepAdaptor_Surface(f.wrapped)
        t = ad.GetType()
        if t == GeomAbs_Plane:
            n = ad.Plane().Axis().Direction()
            if abs(abs(n.Z()) - 1) < 1e-9:
                zs.add(round(ad.Plane().Location().Z(), 9))
            elif abs(n.Z()) > 1e-9:
                return None
        elif t == GeomAbs_Cylinder:
            if abs(abs(ad.Cylinder().Axis().Direction().Z()) - 1) > 1e-9:
                return None
        elif t == GeomAbs_SurfaceOfExtrusion:
            if abs(abs(ad.Direction().Z()) - 1) > 1e-9:
                return None
        else:
            return None
    return sorted(zs)
