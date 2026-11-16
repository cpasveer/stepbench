"""Saying where a fault is in terms a modeller can find in their own model.

A bare point is hard to act on: the modeller has to work out which of their
faces bound it. So a failing wall or gap is described by the two faces it
lies between -- each by its type and size, as a modeller would recognise it
("plane face 40.0 x 30.0 mm", "outside of a round member r = 3.0 mm, 60.0 mm
long") -- the direction across it, whether the two faces meet at an edge
(and at what inside angle: a crevice), and how far the narrow region runs.
It says where, never how to fix.
"""
from __future__ import annotations

import math

import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepTools import BRepTools
from OCP.GeomAbs import GeomAbs_Cone, GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_Sphere, GeomAbs_Torus
from OCP.gp import gp_Dir, gp_Lin, gp_Pnt

from .geom import RAY_START, Part, pt, r


def describe_face(part: Part, i: int) -> str:
    """Type and size of face i, as a modeller sees it."""
    import build123d as bd

    f = part.faces[i]
    ad = BRepAdaptor_Surface(f)
    t = ad.GetType()
    u0, u1, v0, v1 = BRepTools.UVBounds_s(f)
    if t == GeomAbs_Plane:
        pl = ad.Plane().Position()
        o, xd, yd = pl.Location(), pl.XDirection(), pl.YDirection()
        o = np.array([o.X(), o.Y(), o.Z()])
        ex, ey = np.array([xd.X(), xd.Y(), xd.Z()]), np.array([yd.X(), yd.Y(), yd.Z()])
        vs = np.array([tuple(v) for v in bd.Face(f).vertices()]) if bd.Face(f).vertices() else np.zeros((1, 3))
        a, b = np.ptp((vs - o) @ ex), np.ptp((vs - o) @ ey)
        a, b = max(a, b), min(a, b)
        return f"plane face {a:.1f} x {b:.1f} mm"
    if t == GeomAbs_Cylinder:
        rad = ad.Cylinder().Radius()
        length = abs(v1 - v0)
        span = math.degrees(u1 - u0)
        concave = next((x.concave for x in part.rounds() if i in x.faces), None)
        if span < 180 - 1e-6:
            return f"{'inside fillet' if concave else 'rounded edge'} r = {rad:.2f} mm, {length:.1f} mm long"
        what = "inside of a hole" if concave else "outside of a round member"
        return f"{what} r = {rad:.2f} mm, {length:.1f} mm long"
    if t == GeomAbs_Cone:
        return f"conical face, {part.area(i):.0f} mm2"
    if t == GeomAbs_Sphere:
        return f"spherical face r = {ad.Sphere().Radius():.2f} mm"
    if t == GeomAbs_Torus:
        return f"toroidal face (a rounded joint) r = {ad.Torus().MinorRadius():.2f} mm"
    return f"curved face, {part.area(i):.0f} mm2"


def hit_face(part: Part, p, d, start: float = RAY_START) -> int | None:
    """The face a straight line from p along d reaches first (None: it
    escapes, or the kernel cannot say)."""
    ray = part._ray
    try:
        ray.Perform(gp_Lin(gp_Pnt(*map(float, p)), gp_Dir(*map(float, d))), start, 1e9)
        if not ray.IsDone() or ray.NbPnt() == 0:
            return None
        k = min(range(1, ray.NbPnt() + 1), key=lambda k: ray.WParameter(k))
        return part.face_index(ray.Face(k))
    except Exception:  # noqa: BLE001
        return None


def open_corner_gaps(part: Part, g: np.ndarray, limit: float, min_angle: float) -> np.ndarray:
    """Samples whose narrow gap (under ``limit``) only lies in an open inside corner: the ray out of
    the surface reaches a face that meets this sample's face along an inner edge at an inside angle
    of at least ``min_angle``. Such a gap shrinks to nothing at the corner line, as every V does,
    and is not a slot. Exact (one ray and the shared edge per narrow sample)."""
    out = np.zeros(len(g), dtype=bool)
    narrow = np.flatnonzero(np.isfinite(g) & (g < limit - 1e-9))
    for k in narrow:
        fi = int(part.F[k])
        fj = hit_face(part, part.P[k], part.N[k])
        if fj is None or fj == fi:
            continue
        e = _shared_edge(part, fi, fj)
        if e is not None and e.concave and 180.0 - e.turn >= min_angle - 1e-9:
            out[k] = True
    return out


def _shared_edge(part: Part, i: int, j: int):
    for e in part.edges():
        if set(e.faces) == {i, j}:
            return e
    return None


def _extent(part: Part, idx: np.ndarray) -> str:
    if len(idx) <= 2:
        return "at one spot"
    pts = part.P[idx]
    c = pts - pts.mean(axis=0)
    _, _, vt = np.linalg.svd(c, full_matrices=False)
    length = float(np.ptp(c @ vt[0])) + part.spacing
    return f"along a strip about {length:.0f} mm long ({len(idx)} sample points)"


def across(part: Part, i: int, sign: int, failing: np.ndarray, word: str) -> tuple[str, dict]:
    """Describe a wall (sign -1: straight into the material) or a gap
    (sign +1: straight out of the surface) at sample i. ``failing``: the
    samples failing the same rule, to measure how far the narrow region runs
    between the same two faces."""
    fi = int(part.F[i])
    d = sign * part.N[i]
    fj = hit_face(part, part.P[i], d)
    if fj is None:
        return "", {"faces": [fi], "direction": pt(d)}
    same = np.array([k for k in np.flatnonzero(failing)
                     if part.F[k] == fi and hit_face(part, part.P[k], sign * part.N[k]) == fj][:400], dtype=int)
    if i not in same:
        same = np.append(same, i)
    a, b = describe_face(part, fi), describe_face(part, fj)
    text = (f"{word} from the {a} (face {fi}) straight {'into the material' if sign < 0 else 'out'} "
            f"to the {b} (face {fj}), direction {[round(float(x), 3) for x in d]}")
    e = _shared_edge(part, fi, fj)
    info = {"faces": [fi, fj], "face_types": [a, b], "direction": [round(float(x), 4) for x in d]}
    if e is not None and sign > 0:
        inside = 180.0 - e.turn if e.concave else 180.0 + e.turn
        text += f"; these two faces meet along an edge at an inside angle of {inside:.0f} deg: a crevice"
        info["meet_at_edge"] = int(e.index)
        info["inside_angle_deg"] = r(inside, 1)
    text += f"; {_extent(part, same)}"
    info["extent"] = _extent(part, same)
    return text, info


def section_pair(part: Part, e, ta: float, tc: float) -> str:
    """Describe a change of section across inside corner e (faces a, c)."""
    a, c = e.faces
    thin, thick = (a, c) if ta < tc else (c, a)
    tt, tk = min(ta, tc), max(ta, tc)
    return (f"where the {describe_face(part, thin)} (face {thin}, section {tt:.2f} mm) meets the "
            f"{describe_face(part, thick)} (face {thick}, section {tk:.2f} mm)")


WALL_RULES = ("FDM_WALL_MIN", "IM_WALL_MIN", "PBF_WALL_MIN")
GAP_RULES = ("FDM_GAP_MIN", "PBF_GAP_MIN")


def _sample_at(part: Part, point) -> int:
    return int(np.argmin(np.linalg.norm(part.P - np.asarray(point, float), axis=1)))


def annotate(results: list, part: Part) -> None:
    """Add 'between what' to every failing wall, gap and transition result:
    appended to its reason, and as ``between`` in its first location and in
    its details. The point stays as it was."""
    for x in results:
        if x.ok or not x.where or x.where[0].get("point") is None:
            continue
        try:
            if x.id in WALL_RULES or x.id in GAP_RULES:
                wall = x.id in WALL_RULES
                vals = part.thickness() if wall else part.gap()
                i = _sample_at(part, x.where[0]["point"])
                failing = np.isfinite(vals) & (vals < x.limit - 1e-9)
                text, info = across(part, i, -1 if wall else +1, failing, "wall" if wall else "gap")
                e = info.get("meet_at_edge")
                if e is not None:
                    line = part.edge_polyline(e)
                    length = float(np.linalg.norm(np.diff(line, axis=0), axis=1).sum())
                    text += f"; their shared edge is {length:.1f} mm long"
                    info["edge_length_mm"] = r(length, 1)
            elif x.id == "PBF_TRANSITION":
                e = min((e for e in part.edges() if e.concave),
                        key=lambda e: float(np.linalg.norm(e.mid - np.asarray(x.where[0]["point"], float))))
                a, c = e.faces
                text = section_pair(part, e, part.ball_face_max(a), part.ball_face_max(c))
                info = {"faces": [int(a), int(c)], "edge": int(e.index)}
            else:
                continue
        except Exception:  # noqa: BLE001 -- a description must never break a check
            continue
        if text:
            x.reason = f"{x.reason}: {text}"
            x.where[0]["between"] = info
            x.details["between"] = info
