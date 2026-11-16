"""Measurement on the exact solid, shared by every process.

Two kinds of measurement are used, and every rule says which it uses:

* exact: read straight off the solid's own faces and edges -- a plane's
  normal, a cylinder's radius, the angle between two faces at an edge, the
  number of shells. No sampling, nothing to miss.
* sampled: some properties (wall thickness, gaps, what a mould half can
  see) belong to every point of a surface, not to a face. For those the
  part is covered by a fine triangle mesh, the same mesh a viewer draws:
  the exact surface is triangulated and every triangle is split until no
  side is longer than ``spacing`` mm. Each triangle is judged at its centre,
  taken on the true surface, and stands for its own area. At that point the
  measurement itself is exact (a ray against the true surfaces, a true
  distance), so the only approximation is which points are looked at. The
  mesh depends only on the geometry, so the same solid always gives the same
  samples and the same numbers.

What sampling can miss: a fault smaller than the spacing that lies between
triangle centres -- e.g. a thin spot narrower than ``spacing`` in the middle
of an otherwise thick face. Faults that are a whole face (a thin fin, a
narrow slot) are always seen, because every face gets triangles.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Curve2d, BRepAdaptor_Surface
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.BRepClass3d import BRepClass3d, BRepClass3d_SolidClassifier
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp_Face
from OCP.BRepTools import BRepTools
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.BRepClass import BRepClass_FaceClassifier
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.gp import gp_Dir, gp_Lin, gp_Pnt, gp_Pnt2d, gp_Vec
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.ShapeAnalysis import ShapeAnalysis_Surface
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_IN, TopAbs_REVERSED
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.collections import (
    IndexedDataMap_TopoDS_Shape_List_TopoDS_Shape_TopTools_ShapeMapHasher as _EdgeFaceMap,
    IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as _ShapeMap,
)

RAY_START = 1e-4  # mm: a ray ignores its own starting face for this long
INF = float("inf")


def unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v / n if n > 0 else v


def r(x: float, nd: int = 3):
    """Round for reports, so numbers are stable from run to run."""
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return None
    v = round(float(x), nd)
    return 0.0 if v == 0 else v


def pt(p) -> list[float]:
    return [r(float(c), 3) for c in p]


@dataclass
class Edge:
    index: int
    faces: tuple[int, int]
    mid: np.ndarray
    turn: float  # deg, angle between the two face normals at the edge middle
    concave: bool  # an inner corner (material on more than half the turn)


@dataclass
class Round:
    """Cylindrical faces sharing one axis and radius: a hole, a pin, a fillet."""

    axis_point: np.ndarray
    axis_dir: np.ndarray
    radius: float
    span: float  # deg of the full circle covered by these faces
    concave: bool  # True: material outside the cylinder (hole, inner fillet)
    faces: list[int]
    where: np.ndarray

    @property
    def full(self) -> bool:
        return self.span >= 180.0 - 1e-6


class _FaceIn:
    """Is a (u, v) point inside a face? On faces that do not wrap around,
    first the fast polygon classifier (IntTools_FClass2d, tolerance 1e-7 as
    below); where it says ON (on or near the boundary) the exact
    BRepClass_FaceClassifier decides, as it did for every point before. On
    faces that wrap around, always the exact one. Checked: the sample mesh
    is bit-identical to the exact-only one on all 19 benchmark parts."""

    def __init__(self, face):
        from OCP.IntTools import IntTools_FClass2d

        self.face = face
        ad = BRepAdaptor_Surface(face)
        # on a surface that wraps around (cylinder, cone, sphere...) the fast
        # classifier was seen to misjudge a point near the seam: exact there
        self.fast = None if (ad.IsUPeriodic() or ad.IsVPeriodic()) else IntTools_FClass2d(face, 1e-7)
        self.exact = BRepClass_FaceClassifier()

    def inside(self, u: float, v: float) -> bool:
        from OCP.TopAbs import TopAbs_OUT

        if self.fast is None:
            self.exact.Perform(self.face, gp_Pnt2d(u, v), 1e-7)
            return self.exact.State() == TopAbs_IN
        st = self.fast.Perform(gp_Pnt2d(u, v))
        if st == TopAbs_IN:
            return True
        if st == TopAbs_OUT:
            return False
        self.exact.Perform(self.face, gp_Pnt2d(u, v), 1e-7)
        return self.exact.State() == TopAbs_IN


class Part:
    def __init__(self, solid, spacing: float = 1.0, source: str | None = None):
        self.solid = solid
        self.shape = solid.wrapped
        self.spacing = float(spacing)
        self.source = source  # the STEP file, if any: lets helper processes load the same solid
        self.faces = _faces_of(self.shape)
        fmap = _ShapeMap()
        TopExp.MapShapes_s(self.shape, TopAbs_FACE, fmap)
        self._fmap = fmap
        self._ray = IntCurvesFace_ShapeIntersector()
        self._ray.Load(self.shape, 1e-7)
        self._cls = BRepClass3d_SolidClassifier(self.shape)
        self._cache: dict = {}
        self._sample()

    # ---------------------------------------------------------------- faces

    def face_index(self, face) -> int:
        return self._fmap.FindIndex(face) - 1

    def plane(self, i: int):
        """(outward normal, point) of a planar face, else None. Exact."""
        ad = BRepAdaptor_Surface(self.faces[i])
        if ad.GetType() != GeomAbs_Plane:
            return None
        pl = ad.Plane()
        ax = pl.Axis()
        d, o = ax.Direction(), ax.Location()
        n = np.array([d.X(), d.Y(), d.Z()])
        if not pl.Direct():  # left-handed frame: the plane's normal is flipped
            n = -n
        if self.faces[i].Orientation() == TopAbs_REVERSED:
            n = -n
        return unit(n), np.array([o.X(), o.Y(), o.Z()])

    def area(self, i: int) -> float:
        from OCP.BRepGProp import BRepGProp
        from OCP.GProp import GProp_GProps

        g = GProp_GProps()
        BRepGProp.SurfaceProperties_s(self.faces[i], g)
        return float(g.Mass())

    def normal_at(self, i: int, p) -> np.ndarray:
        f = self.faces[i]
        sas = ShapeAnalysis_Surface(BRep_Tool.Surface_s(f))
        uv = sas.ValueOfUV(gp_Pnt(*map(float, p)), 1e-7)
        pp, nv = gp_Pnt(), gp_Vec()
        BRepGProp_Face(f).Normal(uv.X(), uv.Y(), pp, nv)
        return unit([nv.X(), nv.Y(), nv.Z()])

    # -------------------------------------------------------------- samples

    def _sample(self):
        """Mesh every face to ``spacing`` and take one sample per triangle.

        Per face: points along every boundary edge (at most ``spacing``
        apart, closer on curves; the same points for both faces of an edge,
        so the mesh is closed), plus a regular grid of points inside the face
        at least 0.4 x spacing from the boundary. These are joined by a
        Delaunay triangulation in the face's own surface coordinates; a
        triangle whose centre is outside the face is dropped."""
        from scipy.spatial import Delaunay, cKDTree

        s = self.spacing
        V, T, TF, P, N, W = [], [], [], [], [], []
        vkey: dict = {}
        for i, f in enumerate(self.faces):
            ad = BRepAdaptor_Surface(f)
            prop = BRepGProp_Face(f)
            cls = _FaceIn(f)
            u0, u1, v0, v1 = BRepTools.UVBounds_s(f)
            ku, kv = _metric(ad, u0, u1, v0, v1)
            # boundary points, in surface coordinates
            bnd = []
            ex = TopExp_Explorer(f, TopAbs_EDGE)
            while ex.More():
                e = TopoDS.Edge(ex.Current())
                if not BRep_Tool.Degenerated_s(e):
                    c2 = BRepAdaptor_Curve2d(e, f)
                    for t in _edge_params(e, s):
                        q = c2.Value(t)
                        bnd.append((q.X(), q.Y()))
                ex.Next()
            bnd = _dedupe(bnd, ku, kv)
            pts = list(bnd)
            nu = max(1, math.ceil((u1 - u0) * ku / s))
            nv = max(1, math.ceil((v1 - v0) * kv / s))
            grid = []
            for a in range(nu):
                for b in range(nv):
                    u = u0 + (a + 0.5) * (u1 - u0) / nu
                    v = v0 + (b + 0.5) * (v1 - v0) / nv
                    if cls.inside(u, v):
                        grid.append((u, v))
            if grid and bnd:
                tree = cKDTree(np.array(bnd) * (ku, kv))
                dist, _ = tree.query(np.array(grid) * (ku, kv))
                pts += [g for g, dd in zip(grid, dist) if dd >= 0.4 * s]
            if len(pts) < 3:
                continue
            uv = np.array(pts)
            try:
                tri = Delaunay(uv * (ku, kv), qhull_options="QJ Qbb Pp")
            except Exception:  # noqa: BLE001 - a degenerate face is skipped
                continue
            ids = []
            for u, v in uv:
                q = ad.Value(u, v)
                xyz = (q.X(), q.Y(), q.Z())
                key = tuple(round(c, 6) for c in xyz)
                if key not in vkey:
                    vkey[key] = len(V)
                    V.append(xyz)
                ids.append(vkey[key])
            for a, b, c in tri.simplices:
                cu, cv = uv[[a, b, c]].mean(axis=0)
                if not cls.inside(float(cu), float(cv)):
                    continue
                p, nvec = gp_Pnt(), gp_Vec()
                prop.Normal(float(cu), float(cv), p, nvec)
                m = nvec.Magnitude()
                if m <= 1e-12:
                    continue
                n = np.array([nvec.X(), nvec.Y(), nvec.Z()]) / m
                A, B, C = V[ids[a]], V[ids[b]], V[ids[c]]
                u = (B[0] - A[0], B[1] - A[1], B[2] - A[2])
                w = (C[0] - A[0], C[1] - A[1], C[2] - A[2])
                # the cross product, written out (same arithmetic as np.cross, far less overhead)
                cr = np.array((u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]))
                area = 0.5 * float(np.linalg.norm(cr))
                if area <= 1e-10:
                    continue
                ia, ib, ic = ids[a], ids[b], ids[c]
                if float(cr @ n) < 0:  # wind every triangle outward
                    ib, ic = ic, ib
                T.append((ia, ib, ic))
                TF.append(i)
                P.append((p.X(), p.Y(), p.Z()))
                N.append(tuple(n))
                W.append(area)
        self.V = np.array(V, dtype=float).reshape(-1, 3)
        self.T = np.array(T, dtype=int).reshape(-1, 3)
        self.P = np.array(P, dtype=float).reshape(-1, 3)
        self.N = np.array(N, dtype=float).reshape(-1, 3)
        self.W = np.array(W, dtype=float)
        self.F = np.array(TF, dtype=int)

    def edge_lines(self, deflection: float = 0.02) -> list[np.ndarray]:
        """The part's true edges as polylines, for drawing."""
        out = []
        emap = _EdgeFaceMap()
        TopExp.MapShapesAndAncestors_s(self.shape, TopAbs_EDGE, TopAbs_FACE, emap)
        for k in range(1, emap.Extent() + 1):
            e = TopoDS.Edge(emap.FindKey(k))
            if BRep_Tool.Degenerated_s(e):
                continue
            c = BRepAdaptor_Curve(e)
            q = GCPnts_QuasiUniformDeflection(c, deflection)
            if not q.IsDone() or q.NbPoints() < 2:
                continue
            pts = [q.Value(j) for j in range(1, q.NbPoints() + 1)]
            out.append(np.array([(p.X(), p.Y(), p.Z()) for p in pts]))
        return out

    # ----------------------------------------------------------------- rays

    def ray(self, p, d, start: float = RAY_START) -> float:
        """Distance along d from p to the first surface hit, or inf. Exact."""
        return _ray_exact(self._ray, self.faces, p, d, start, self.inside)

    def inside(self, p, tol: float = 1e-7) -> bool:
        self._cls.Perform(gp_Pnt(*map(float, p)), tol)
        return self._cls.State() == TopAbs_IN

    def thickness(self) -> np.ndarray:
        """Wall thickness at every sample: a ray from the sample straight into
        the material, to where it leaves again. It can only over-estimate a
        wall (a ray along a plate's length), never under-estimate one, so the
        smallest value over all samples is a safe minimum."""
        if "thick" not in self._cache:
            self._both_rays()
        return self._cache["thick"]

    def gap(self) -> np.ndarray:
        """Clear space in front of every sample: a ray straight out of the
        surface to the next surface of the same part (inf if it escapes)."""
        if "gap" not in self._cache:
            self._both_rays()
        return self._cache["gap"]

    def _both_rays(self):
        """Wall and gap rays for every sample, together: on a large part with
        idle cores, split over helper processes (same exact intersector,
        same result to the last bit); otherwise here."""
        if self.source is not None and len(self.P) >= RAY_HELPERS_FROM and _cores_idle():
            got = _run_helpers("rays", self.source, [(self.P[i], self.N[i]) for i in _shares(len(self.P))])
            if got is not None:
                self._cache["thick"] = np.concatenate([g[0] for g in got])
                self._cache["gap"] = np.concatenate([g[1] for g in got])
                return
        self._cache["thick"] = np.array([self.ray(p, -n) for p, n in zip(self.P, self.N)])
        self._cache["gap"] = np.array([self.ray(p, n) for p, n in zip(self.P, self.N)])

    def nominal_wall(self) -> float:
        """Area-weighted median of the sampled thickness: the wall most of the
        part is made of."""
        t = self.thickness()
        ok = np.isfinite(t)
        if not ok.any():
            return INF
        t, w = t[ok], self.W[ok]
        o = np.argsort(t, kind="stable")
        c = np.cumsum(w[o])
        return float(t[o][np.searchsorted(c, c[-1] / 2)])

    def ball(self) -> np.ndarray:
        """Diameter of the largest ball that touches the surface at each sample
        and fits inside the material (shrinking-ball method: start from half
        the ray thickness and shrink until no surface is closer than the
        radius; each distance is exact). This is the true local thickness of
        a section, used to find thick spots. Near a sharp inner corner it
        under-estimates, which cannot make a thick-spot check fail wrongly."""
        if "ball" not in self._cache:
            out = _ball_parallel(self) if _use_workers(self) else _ball_points(self.faces, self.P, self.N, self.thickness())
            self._cache["ball"] = np.array(out)
        return self._cache["ball"]

    def fine(self, deflection: float = None) -> np.ndarray:
        """A fine triangle copy of the surface (for the compiled helpers)."""
        d = FINE_DEFLECTION if deflection is None else deflection
        key = ("fine", d)
        if key not in self._cache:
            self._cache[key] = fine_triangles(self.shape, d)
        return self._cache[key]

    def fine_mesh(self, deflection: float = None):
        """The fine copy loaded into the compiled helper, with its search
        tree, built once per part and accuracy."""
        d = FINE_DEFLECTION if deflection is None else deflection
        key = ("fine_mesh", d)
        if key not in self._cache:
            from . import _fastball

            self._cache[key] = _fastball.Mesh(_le(self.fine(d)))
        return self._cache[key]

    def inside_many(self, pts) -> np.ndarray:
        """``inside`` for many points at once. With the compiled helper, a
        point more than INSIDE_GUARD from the surface, on which three skew
        rays agree, is decided on the fine copy; every other point by the
        exact classifier, as without the helper."""
        pts = np.asarray(pts, float).reshape(-1, 3)
        res = np.zeros(len(pts), np.int8) - 1
        if fast_available() and len(pts):
            res = np.frombuffer(self.fine_mesh(INSIDE_DEFLECTION).inside(_le(pts), INSIDE_GUARD), dtype=np.int8).copy()
        for k in np.flatnonzero(res < 0):
            res[k] = 1 if self.inside(pts[k]) else 0
        return res.astype(bool)

    # The rules ask for the ball through these three; with the exact routine
    # everywhere they are plain look-ups. (A compiled, approximate ball was
    # tried and dropped: see docs/dfm.md, "Speed".)
    def ball_max(self, sel=None) -> tuple[float, int]:
        """The largest ball over samples ``sel`` (all if None), and the
        first sample where it occurs: exactly what the full ball array would
        give, without measuring every sample. A ball never grows past the
        straight-through ray where it starts, so the samples are measured in
        falling order of ray length, and the search stops once the next ray
        is shorter than the best ball found."""
        n = len(self.P)
        idx = np.arange(n) if sel is None else np.asarray(sel, dtype=int)
        if not idx.size:
            return math.nan, -1
        if "ball" in self._cache:  # the full array exists already
            b = self._cache["ball"]
            k = int(idx[np.argmax(b[idx])])
            return float(b[k]), k
        t = self.thickness()
        ub = np.where(np.isfinite(t), t, 100.0)
        known = self._cache.setdefault("ball_known", np.full(n, np.nan))
        order = idx[np.lexsort((idx, -ub[idx]))]  # longest ray first; ties by sample number
        best, besti = -INF, -1
        pos = 0
        while pos < len(order):
            if ub[order[pos]] < best:
                break
            chunk = order[pos:pos + 64]
            todo = chunk[np.isnan(known[chunk])]
            if todo.size:
                if "extrema" not in self._cache:
                    self._cache["extrema"] = _faces_extrema(self.faces)
                known[todo] = _ball_points(self.faces, self.P[todo], self.N[todo], t[todo], self._cache["extrema"])
            for i in chunk:
                if ub[i] < best:
                    pos = len(order)
                    break
                v = known[i]
                if v > best or (v == best and i < besti):
                    best, besti = float(v), int(i)
            pos += 64
        self._cache["ball_measured"] = int(np.isfinite(known).sum())
        return best, besti

    def ball_face_max(self, fi: int) -> float:
        """The largest ball over face ``fi``'s samples (see ball_max)."""
        sel = np.flatnonzero(self.F == fi)
        return self.ball_max(sel)[0] if sel.size else math.nan

    def edges(self) -> list[Edge]:
        """Every edge between two different faces, with the angle the surface
        turns there and whether it is an inner (concave) corner. Exact: the
        normals are taken on the true surfaces at the edge's middle."""
        if "edges" in self._cache:
            return self._cache["edges"]
        emap = _EdgeFaceMap()
        TopExp.MapShapesAndAncestors_s(self.shape, TopAbs_EDGE, TopAbs_FACE, emap)
        out = []
        for k in range(1, emap.Extent() + 1):
            e = TopoDS.Edge(emap.FindKey(k))
            if BRep_Tool.Degenerated_s(e):
                continue
            idx = []
            for f in emap.FindFromIndex(k):
                j = self.face_index(TopoDS.Face(f))
                if j not in idx:
                    idx.append(j)
            if len(idx) != 2:
                continue  # a seam (both sides the same face)
            c = BRepAdaptor_Curve(e)
            q = c.Value((c.FirstParameter() + c.LastParameter()) / 2)
            mid = np.array([q.X(), q.Y(), q.Z()])
            n1, n2 = self.normal_at(idx[0], mid), self.normal_at(idx[1], mid)
            turn = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(n1, n2))))))
            concave = False
            if turn > 0.5:
                probe = mid + 2e-3 * unit(n1 - n2)
                concave = self.inside(probe)
            out.append(Edge(k - 1, (idx[0], idx[1]), mid, turn, concave))
        self._cache["edges"] = out
        return out

    def neighbours(self, i: int) -> list[Edge]:
        return [e for e in self.edges() if i in e.faces]

    # --------------------------------------------------------------- rounds

    def rounds(self) -> list[Round]:
        """Cylindrical faces grouped by axis and radius. Exact."""
        if "rounds" in self._cache:
            return self._cache["rounds"]
        groups: dict = {}
        for i, f in enumerate(self.faces):
            ad = BRepAdaptor_Surface(f)
            if ad.GetType() != GeomAbs_Cylinder:
                continue
            cyl = ad.Cylinder()
            ax = cyl.Axis()
            d = unit([ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()])
            o = np.array([ax.Location().X(), ax.Location().Y(), ax.Location().Z()])
            for c in d:  # one sign for the axis direction
                if abs(c) > 1e-9:
                    d = d if c > 0 else -d
                    break
            o = o - np.dot(o, d) * d  # axis point closest to the origin
            rad = cyl.Radius()
            u0, u1, v0, v1 = BRepTools.UVBounds_s(f)
            span = math.degrees(u1 - u0)
            sel = self.F == i
            if sel.any():
                p, n = self.P[sel][0], self.N[sel][0]
            else:
                q = ad.Value((u0 + u1) / 2, (v0 + v1) / 2)
                p = np.array([q.X(), q.Y(), q.Z()])
                n = self.normal_at(i, p)
            radial = p - o - np.dot(p - o, d) * d
            concave = float(np.dot(n, radial)) < 0  # outward normal points at the axis
            key = (tuple(np.round(d, 6)), tuple(np.round(o, 4)), round(rad, 5), concave)
            if key in groups:
                g = groups[key]
                g.span += span
                g.faces.append(i)
            else:
                groups[key] = Round(o, d, rad, span, concave, [i], p)
        self._cache["rounds"] = list(groups.values())
        return self._cache["rounds"]

    # --------------------------------------------------------------- shells

    def voids(self) -> list[np.ndarray]:
        """A point for each sealed hollow (an inner shell). Exact."""
        return [p for p, _ in self.void_faces()]

    def void_faces(self) -> list[tuple[np.ndarray, list[int]]]:
        """(centre point, face indices) of each sealed hollow."""
        outer = BRepClass3d.OuterShell_s(TopoDS.Solid(self.shape))
        out = []
        for sh in self.solid.shells():
            if sh.wrapped.IsSame(outer):
                continue
            bb = sh.bounding_box()
            c = np.array([(bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2,
                          (bb.min.Z + bb.max.Z) / 2])
            out.append((c, sorted(self.face_index(f.wrapped) for f in sh.faces())))
        return out

    def edge_polyline(self, index: int) -> np.ndarray:
        """Points along edge ``index`` (as numbered in ``edges()``)."""
        emap = _EdgeFaceMap()
        TopExp.MapShapesAndAncestors_s(self.shape, TopAbs_EDGE, TopAbs_FACE, emap)
        e = TopoDS.Edge(emap.FindKey(index + 1))
        c = BRepAdaptor_Curve(e)
        return np.array([(lambda q: (q.X(), q.Y(), q.Z()))(c.Value(t)) for t in _edge_params(e, 1.0)])

    # ----------------------------------------------------------- directions

    def candidate_directions(self, n_faces: int, signed: bool, n_cross: int = 0) -> list[tuple[np.ndarray, str]]:
        """The fixed set of directions tried when the task states none: the
        axes, then the outward normals of the largest flat faces (reversed,
        so that face would lie on the bed or the parting plane), then, for
        each pair of the ``n_cross`` largest flat faces, the direction square
        to both (either way up): both faces then stand upright, and the part
        stands on edges or on a face the designer made for it. Order is
        fixed, so ties always resolve the same way."""
        axes = [((1, 0, 0), "+X"), ((-1, 0, 0), "-X"), ((0, 1, 0), "+Y"),
                ((0, -1, 0), "-Y"), ((0, 0, 1), "+Z"), ((0, 0, -1), "-Z")]
        cands = [(np.array(v, float), name) for v, name in axes]
        flats = []
        for i in range(len(self.faces)):
            pl = self.plane(i)
            if pl is not None:
                flats.append((-round(self.area(i), 6), i, pl[0]))
        flats.sort(key=lambda t: (t[0], t[1]))
        for _, i, nrm in flats[:n_faces]:
            cands.append((unit(-nrm), f"face {i} down"))
        top = flats[:n_cross]
        for a in range(len(top)):
            for b in range(a + 1, len(top)):
                c = np.cross(top[a][2], top[b][2])
                if np.linalg.norm(c) < 0.1:  # (nearly) parallel faces: no single direction square to both
                    continue
                c = unit(c)
                ia, ib = top[a][1], top[b][1]
                cands.append((c, f"faces {ia} and {ib} upright"))
                cands.append((-c, f"faces {ia} and {ib} upright, other way up"))
        out = []
        for v, name in cands:
            v = np.round(v, 9)
            dup = any(np.allclose(v, w, atol=1e-6) or (not signed and np.allclose(v, -w, atol=1e-6))
                      for w, _ in out)
            if not dup:
                out.append((v, name))
        return out


def vertical_extent(part: Part, d: np.ndarray) -> tuple[float, float]:
    """Lowest and highest point of the part along d (samples and vertices)."""
    h = part.P @ d
    vs = np.array([tuple(v) for v in part.solid.vertices()], dtype=float)
    hv = vs @ d if len(vs) else h
    return float(min(h.min(), hv.min())), float(max(h.max(), hv.max()))


FINE_DEFLECTION = 0.0005  # mm: how far the fine copy (for the compiled inside test) may stray from the true surface


def fine_triangles(shape, deflection: float = FINE_DEFLECTION) -> np.ndarray:
    """A fine triangle copy of the solid's surface, (n, 9) floats: within
    ``deflection`` of the true faces. Meshes a copy, so the solid itself
    carries no triangulation afterwards."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy
    from OCP.BRepMesh import BRepMesh_IncrementalMesh

    cp = BRepBuilderAPI_Copy(shape).Shape()
    BRepMesh_IncrementalMesh(cp, deflection, False, 0.1, True)
    # written out as binary STL and read back in one go: the mesher's own
    # triangles in single precision (6e-8 relative, far inside the
    # deflection), and far faster than walking them in Python
    import tempfile

    from OCP.StlAPI import StlAPI_Writer

    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "fine.stl")
        w = StlAPI_Writer()
        w.ASCIIMode = False
        w.Write(cp, path)
        raw = np.fromfile(path, dtype=np.uint8)
    n = int(raw[80:84].view("<u4")[0])
    rec = raw[84:84 + 50 * n].reshape(n, 50)
    return rec[:, 12:48].copy().view("<f4").reshape(n, 9).astype(float)


def fast_available() -> bool:
    """True when the compiled helper is built (python -m
    jointbench.dfm.build_native) and not switched off with
    JOINTBENCH_NATIVE=0. Without it everything runs on the exact routines:
    identical verdicts, only slower."""
    import os

    if os.environ.get("JOINTBENCH_NATIVE", "1") == "0":
        return False
    try:
        from . import _fastball  # noqa: F401
    except ImportError:
        return False
    return True


def _le(a) -> bytes:
    return np.ascontiguousarray(a, dtype="<f8").tobytes()


INSIDE_DEFLECTION = FINE_DEFLECTION
INSIDE_GUARD = 0.005  # mm: closer than this to the fine copy's surface, a point is classified exactly


def _faces_of(shape) -> list:
    fmap = _ShapeMap()
    TopExp.MapShapes_s(shape, TopAbs_FACE, fmap)
    return [TopoDS.Face(fmap.FindKey(i)) for i in range(1, fmap.Extent() + 1)]


def _faces_extrema(faces):
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Compound

    ext = BRepExtrema_DistShapeShape()
    comp = TopoDS_Compound()
    bb = BRep_Builder()
    bb.MakeCompound(comp)
    for f in faces:
        bb.Add(comp, f)
    ext.LoadS2(comp)
    return ext


def _ball_points(faces, P, N, T, ext=None) -> np.ndarray:
    """The shrinking-ball diameters for samples (P, N) with ray thickness T.
    ``ext``: a distance tool already loaded with these faces (reused)."""
    ext = ext if ext is not None else _faces_extrema(faces)
    out = []
    for p, n, t in zip(P, N, T):
        m = -n
        rad = (t / 2) if math.isfinite(t) else 50.0
        for _ in range(40):
            c = p + rad * m
            ext.LoadS1(BRepBuilderAPI_MakeVertex(gp_Pnt(*map(float, c))).Vertex())
            ext.Perform()
            if not ext.IsDone() or ext.NbSolution() == 0:
                break
            dist = ext.Value()
            if dist >= rad - 1e-4:
                break
            q = ext.PointOnShape2(1)
            q = np.array([q.X(), q.Y(), q.Z()])
            w = q - p
            den = 2.0 * float(np.dot(m, w))
            new = float(np.dot(w, w)) / den if den > 1e-12 else 0.0
            if not (0 <= new < rad):
                rad = dist if dist < rad else rad
                break
            rad = new
        out.append(2.0 * rad)
    return np.array(out)


# ---------------------------------------------------------- helper processes
#
# The exact per-sample measurements are independent of each other, so they
# can be split over several processes. Each helper loads the same STEP file
# (so it holds the identical solid, faces in the identical order) and runs
# the identical code on its share of the samples; the shares are joined in
# order. The result is the same to the last bit as one process doing it all.

def workers() -> int:
    import os

    v = os.environ.get("JOINTBENCH_WORKERS")
    if v is not None:
        return max(1, int(v))
    return max(1, min(8, (os.cpu_count() or 1) - 1))


RAY_HELPERS_FROM = 100000  # samples: each helper needs about 3 s just to import the CAD kernel, so below this they cost more than they save


def _cores_idle() -> bool:
    """Helpers pay off only with idle cores."""
    if workers() <= 1:
        return False
    try:
        busy = os.getloadavg()[0]
    except OSError:
        busy = 0.0
    return busy < (os.cpu_count() or 1) / 2


def _use_workers(part) -> bool:
    # the exact ball costs about 0.5 ms a sample; helpers need about 3 s to start
    return part.source is not None and len(part.P) >= 10000 and _cores_idle()


def _shares(n: int) -> list:
    return [i for i in np.array_split(np.arange(n), workers()) if len(i)]


def _run_helpers(op: str, path: str, jobs: list):
    """Run ``op`` on each job in its own helper process (``python -m
    jointbench.dfm._helper``), each loading the same STEP file. Plain
    subprocesses, not multiprocessing, so it works from any calling script;
    inputs and outputs as files, so every helper starts at once. Returns the
    results in order, or None if any helper failed (the caller then does
    the work itself: same result)."""
    import pickle
    import subprocess
    import sys
    import tempfile

    procs = []
    with tempfile.TemporaryDirectory() as tmp:
        try:
            for k, job in enumerate(jobs):
                fin = os.path.join(tmp, f"in{k}.pkl")
                with open(fin, "wb") as fh:
                    pickle.dump((op, path, job), fh)
                procs.append((subprocess.Popen([sys.executable, "-m", "jointbench.dfm._helper", fin, fin + ".out"],
                                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL), fin + ".out"))
            out = []
            for pr, fout in procs:
                if pr.wait() != 0:
                    return None
                with open(fout, "rb") as fh:
                    out.append(pickle.load(fh))
            return out
        except Exception:  # noqa: BLE001
            for pr, _ in procs:
                pr.kill()
            return None


def _ray_exact(ray, faces, p, d, start, classify=None) -> float:
    """First hit along d from p (beyond ``start``), or inf. The whole-shape
    intersector is used. If it fails (OpenCASCADE raises on some valid
    solids: seen where a sphere meets a cylinder with a 0.02 mm gap covered
    by the edge tolerance; the per-face intersector then silently misses the
    hit too), the line is crossed with every face's underlying surface;
    crossings lying on their face (within 0.03 mm) are grouped, and the
    first group across which the solid classifier changes side is the hit. Same exact crossing
    parameter, only slower, and only for that ray. ``classify(point) ->
    inside?``: the part's solid classifier."""
    lin = gp_Lin(gp_Pnt(*map(float, p)), gp_Dir(*map(float, d)))
    try:
        ray.Perform(lin, start, 1e9)
        if not ray.IsDone() or ray.NbPnt() == 0:
            return INF
        return min(ray.WParameter(k) for k in range(1, ray.NbPnt() + 1))
    except Exception:  # noqa: BLE001 -- OCP raises Standard_Failure on some valid solids
        pass
    from OCP.Geom import Geom_Line
    from OCP.GeomAPI import GeomAPI_IntCS

    if classify is None:
        raise RuntimeError("ray could not be measured and no classifier was given")
    gl = Geom_Line(lin)
    hits = []  # (parameter, distance from the crossing to its own face)
    for f in faces:
        try:
            inter = GeomAPI_IntCS(gl, BRep_Tool.Surface_s(f))
        except Exception:  # noqa: BLE001
            continue
        if not inter.IsDone():
            continue
        for k in range(1, inter.NbPoints() + 1):
            w = inter.Parameters(k)[2]
            if not (start < w < 1e6):  # nothing of a part is a kilometre away
                continue
            ext = BRepExtrema_DistShapeShape(BRepBuilderAPI_MakeVertex(inter.Point(k)).Vertex(), f)
            dist = ext.Value() if ext.IsDone() else INF
            if dist <= 0.03:  # on the face, within what a modeller leaves at a seam
                hits.append((w, dist))
    # crossings closer than 0.03 mm along the ray are one boundary; of each
    # group, the one lying closest to its face is taken
    hits.sort()
    groups = []
    for h in hits:
        if groups and h[0] - groups[-1][-1][0] < 0.03:
            groups[-1].append(h)
        else:
            groups.append([h])
    P0, D = np.array(p, float), np.array(d, float)
    for k, g in enumerate(groups):
        lo, hi = g[0][0], g[-1][0]
        gap_prev = lo - (groups[k - 1][-1][0] if k else start)
        gap_next = (groups[k + 1][0][0] - hi) if k + 1 < len(groups) else 1.0
        dl = min(0.05, gap_prev / 2, gap_next / 2)
        if classify(P0 + D * (lo - dl)) != classify(P0 + D * (hi + dl)):
            return float(min(g, key=lambda h: (h[1], h[0]))[0])
    return INF


def _rays_points(shape, P, N):
    """(wall, gap) ray lengths, exactly as Part.ray does them."""
    ray = IntCurvesFace_ShapeIntersector()
    ray.Load(shape, 1e-7)
    faces = _faces_of(shape)
    cls = BRepClass3d_SolidClassifier(shape)

    def inside(q):
        cls.Perform(gp_Pnt(*map(float, q)), 1e-7)
        return cls.State() == TopAbs_IN

    return (np.array([_ray_exact(ray, faces, p, -n, RAY_START, inside) for p, n in zip(P, N)]),
            np.array([_ray_exact(ray, faces, p, n, RAY_START, inside) for p, n in zip(P, N)]))


def _ball_parallel(part) -> np.ndarray:
    T = part.thickness()
    got = _run_helpers("ball", part.source, [(part.P[i], part.N[i], T[i]) for i in _shares(len(part.P))])
    if got is None:
        return _ball_points(part.faces, part.P, part.N, T)
    return np.concatenate(got)


def load(step_path: str):
    """Read a STEP and return its single solid, or raise ValueError."""
    import build123d as bd

    shape = bd.import_step(step_path)
    solids = shape.solids()
    if len(solids) != 1:
        raise ValueError(f"expected exactly 1 solid, found {len(solids)}")
    return solids[0]



def _metric(ad, u0, u1, v0, v1):
    """mm per unit of each surface coordinate (largest over a few lines)."""
    def span(fixed, a, b, along_u):
        pts = []
        for k in range(9):
            t = a + (b - a) * k / 8
            q = ad.Value(t, fixed) if along_u else ad.Value(fixed, t)
            pts.append((q.X(), q.Y(), q.Z()))
        return float(np.linalg.norm(np.diff(np.array(pts), axis=0), axis=1).sum())

    lu = max(span(v0 + (v1 - v0) * k / 2, u0, u1, True) for k in range(3))
    lv = max(span(u0 + (u1 - u0) * k / 2, v0, v1, False) for k in range(3))
    return max(lu / max(u1 - u0, 1e-12), 1e-9), max(lv / max(v1 - v0, 1e-12), 1e-9)


def _edge_params(e, s):
    """Parameters along an edge: fine enough for curves (0.01 mm chord) and
    never more than s apart. Depends only on the edge, so both faces of an
    edge get the same points."""
    c = BRepAdaptor_Curve(e)
    t0, t1 = c.FirstParameter(), c.LastParameter()
    q = GCPnts_QuasiUniformDeflection(c, 0.01)
    ts = [q.Parameter(k) for k in range(1, q.NbPoints() + 1)] if q.IsDone() else [t0, t1]
    if len(ts) < 2:
        ts = [t0, t1]
    out = [ts[0]]
    for a, b in zip(ts[:-1], ts[1:]):
        pa, pb = c.Value(a), c.Value(b)
        n = max(1, math.ceil(pa.Distance(pb) / s))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return out


def _dedupe(pts, ku, kv, tol=1e-6):
    seen, out = set(), []
    for u, v in pts:
        key = (round(u * ku / tol), round(v * kv / tol))
        if key not in seen:
            seen.add(key)
            out.append((u, v))
    return out
