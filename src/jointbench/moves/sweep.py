"""(Copied from level 4's ``jointbench.assemble.sweep`` on branch level4-assembles,
unchanged; when both levels are merged, keep one copy.)

The space a part sweeps through as it is put in place along a straight line.

A part that comes in along direction ``d`` from outside the scene passes
through every point x - t d (t >= 0) of itself. That swept space is exact:
it is the part at its place, plus, for every face of the part, the prism the
face traces as it moves (a point the part passes through is either inside
it at the end, or was crossed by one of its faces on the way). So the check
is a set of exact boolean intersections, measured as volume like
NO_INTERFERENCE: sliding along a face (touching) is zero volume, running
into something is not.

Faces that only slide along themselves (a flat face parallel to ``d``, a
cylinder whose axis is ``d``) trace no volume and are left out. Only the
*leading* faces are needed -- those facing the way the part moves (outward
normal along ``d``): every point the part passes through is reached first by
one of them. A cylinder crossed sideways is cut along the line where it
turns away from ``d`` and only its leading half is traced. Another curved
face (a cone, a fillet, a free form) is traced whole if it faces forward
everywhere (checked on a grid of points); one that turns away is traced as
its fine triangulation (0.01 mm), so its prisms stand within 0.01 mm of the
true swept surface. The overlap reported is the sum over the traced prisms:
never less than the true overlap, and zero exactly when the true one is.
"""
from __future__ import annotations

import math

import build123d as bd
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.gp import gp_Vec

from ..checks.base import volume

TESSELLATION = 0.01  # mm, curved faces other than cylinders
TANGENT = 1e-9


def _unit(v):
    v = np.asarray(v, float)
    n = float(np.linalg.norm(v))
    if n == 0:
        raise ValueError("a direction of length zero")
    return v / n


def _forward(face: bd.Face, d: np.ndarray) -> float:
    """Smallest and largest n . d over a grid of points on the face."""
    vals = []
    for u in np.linspace(0.02, 0.98, 7):
        for v in np.linspace(0.02, 0.98, 7):
            try:
                vals.append(float(np.array(tuple(face.normal_at(u, v))) @ d))
            except Exception:  # noqa: BLE001
                pass
    return (min(vals), max(vals)) if vals else (-1.0, 1.0)


def _pieces(face: bd.Face, d: np.ndarray) -> list[bd.Face]:
    """The leading part of the face (facing along ``d``), split where needed
    so each piece traces a proper prism; [] if none of it leads."""
    ad = BRepAdaptor_Surface(face.wrapped)
    kind = ad.GetType()
    if kind == GeomAbs_Plane:
        n = np.array(tuple(face.normal_at()))
        return [face] if float(n @ d) > TANGENT else []
    if kind == GeomAbs_Cylinder:
        ax = ad.Cylinder().Axis()
        a = np.array([ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()])
        dp = d - a * float(a @ d)
        if float(np.linalg.norm(dp)) < TANGENT:
            return []
        loc = ax.Location()
        cut = bd.Plane(origin=(loc.X(), loc.Y(), loc.Z()), z_dir=tuple(_unit(dp)))
        try:
            got = face.split(cut, keep=bd.Keep.BOTH)
        except Exception:  # noqa: BLE001 - fall back to triangles
            return _triangles(face, d)
        faces = []
        for g in (got if isinstance(got, (list, tuple)) else [got]):
            if g is None:
                continue
            faces += list(g.faces()) if hasattr(g, "faces") else [x for x in g]
        faces = [f for f in faces if f.area > 1e-9]
        if not faces:
            return _triangles(face, d)
        return [f for f in faces if _forward(f, d)[1] > TANGENT]
    lo, hi = _forward(face, d)
    if lo > TANGENT:
        return [face]
    if hi <= TANGENT:
        return []
    return _triangles(face, d)


def _triangles(face: bd.Face, d: np.ndarray) -> list[bd.Face]:
    """The leading triangles of the face's fine triangulation."""
    verts, tris = face.tessellate(TESSELLATION)
    out = []
    for i, j, k in tris:
        p = [tuple(verts[i]), tuple(verts[j]), tuple(verts[k])]
        n = np.cross(np.subtract(p[1], p[0]), np.subtract(p[2], p[0]))
        if np.linalg.norm(n) < 1e-12:
            continue
        nn = _unit(n)
        # orient by the face's own outward normal at the triangle's centre
        try:
            fn = np.array(tuple(face.normal_at(bd.Vector(*np.mean(p, axis=0)))))
        except Exception:  # noqa: BLE001
            fn = nn
        if float(fn @ nn) < 0:
            nn = -nn
        if float(nn @ d) <= TANGENT:
            continue
        out.append(bd.Face(bd.Wire.make_polygon(p, close=True)))
    return out


def travel(part: bd.Shape, d, scene: list[bd.Shape], margin: float = 10.0) -> float:
    """How far the part must come from: far enough that at its start it is
    wholly beyond everything in ``scene`` along ``d``."""
    d = _unit(d)

    def corners(s):
        bb = s.bounding_box()
        return np.array([[x, y, z] for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y)
                         for z in (bb.min.Z, bb.max.Z)])

    top = float((corners(part) @ d).max())
    low = min(float((corners(s) @ d).min()) for s in scene) if scene else top
    return max(top - low, 0.0) + margin


def collisions(part: bd.Shape, d, bodies: dict[str, bd.Shape], length: float | None = None) -> dict[str, float]:
    """Overlap volume (mm3) between the space ``part`` sweeps through, coming
    in along ``d`` from outside the scene (or from ``length`` mm back, for one
    straight move of a path) to where it is, and each body."""
    d = _unit(d)
    L = travel(part, d, list(bodies.values())) if length is None else float(length)
    shift = gp_Vec(*(-L * d))
    boxes = {n: b.bounding_box() for n, b in bodies.items()}
    hit = {n: 0.0 for n in bodies}  # summed over the traced prisms
    for f in part.faces():
        for piece in _pieces(f, d):
            prism = bd.Solid(BRepPrimAPI_MakePrism(piece.wrapped, shift).Shape())
            pb = prism.bounding_box()
            for n, b in bodies.items():
                bb = boxes[n]
                if (pb.max.X < bb.min.X or bb.max.X < pb.min.X or pb.max.Y < bb.min.Y or bb.max.Y < pb.min.Y
                        or pb.max.Z < bb.min.Z or bb.max.Z < pb.min.Z):
                    continue
                hit[n] += volume(prism & b)
    # the part where it ends up (a body it is put into)
    for n, b in bodies.items():
        hit[n] = max(hit[n], volume(part & b))
    return {n: round(v, 4) for n, v in hit.items()}


def describe(d) -> str:
    d = _unit(d)
    return "(" + ", ".join(f"{0.0 if abs(x) < 5e-4 else x:.3f}" for x in d) + ")"


__all__ = ["collisions", "travel", "describe", "math"]
