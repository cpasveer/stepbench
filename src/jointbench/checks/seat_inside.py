"""SEAT_INSIDE: where the part lies on a mounting face, it stays inside it,
set back from the face's edge.

A part bolted to a fixed part should not be larger than that part's face,
nor turned so its corners stick out past it, and it should not run flush to
the very edge either: a small setback reads as a deliberate edge and absorbs
small errors in position. Measured exactly: every flat face of the submitted
part that lies on a mounting plane (the faces SEATING counts as seats) is
laid onto the fixed part's own face. That face's outer outline (tapped holes
do not count) is offset inward by ``seat_setback``; the area of seat left
outside the offset outline, summed per mounting face, must not exceed
``seat_overhang_max``.

The setback has room: every catalogue fixture's face reaches 3 mm beyond the
edge of each bolt head, and the bolt head's ring must stay covered (SEATING),
so a setback of up to about 2 mm leaves the clamped ring whole.

Only the seat is judged: material that does not touch the mounting plane is
free to go where it must.

Required only where a task lists it (``tolerances["required_checks"]``),
so tasks written before the check existed keep their verdicts.
"""
from __future__ import annotations

import build123d as bd

from .base import CheckResult, Context, area
from .seating import PENETRATION_NOISE, _onto_plane, _planar_faces

NAME = "SEAT_INSIDE"
GROUP = "fits"
DEFAULT_SETBACK = 1.0  # mm the seat stays inside the face's edge (maintainer's decision, 12 November 2026)
DEFAULT_MAX = 0.5      # mm2 of seat allowed outside that line: numerical noise only


def _inset(face: bd.Face, setback: float):
    """The face's outer outline, offset inward by ``setback`` (None if
    nothing is left)."""
    outer = bd.Face(face.outer_wire())
    if setback <= 0:
        return outer
    try:
        got = bd.offset(outer, amount=-setback, kind=bd.Kind.INTERSECTION)
    except Exception:  # noqa: BLE001 - an outline too small to set back: nothing allowed
        return None
    faces = got.faces() if got is not None else []
    return faces[0] if faces else None


def overhang(ctx: Context, pose: int, setback: float) -> list[dict]:
    """Per mounting face: seat area, and how much of it lies outside the
    face's outline set back by ``setback``."""
    t = ctx.task
    gap = t.tolerances["seat_gap_max"]
    fixed = ctx.fixed(pose)
    out = []
    for face in t.faces(ctx.stored(pose)):
        if t.parts[face.part].params.get("joint"):
            continue  # level 4: the face of the answer's other part, at their joint: not a fixed part's edge
        n = face.normal
        neg = tuple(-v for v in n)
        seats, _ = _planar_faces(ctx.solid_at(pose), neg, face.origin, -gap, PENETRATION_NOISE)
        fixed_faces, _ = _planar_faces(fixed[face.part], n, face.origin, -1e-4, 1e-4)
        allowed = [x for x in (_inset(ff, setback) for ff, _ in fixed_faces) if x is not None]
        total = outside = 0.0
        for sf, dist in seats:
            flat = _onto_plane(sf, n, -dist)
            a = area(flat)
            inside = sum(area(flat & o) for o in allowed)
            total += a
            outside += max(0.0, a - inside)
        out.append({"part": face.part, "seat_mm2": round(total, 3), "outside_mm2": round(outside, 3)})
    return out


def run(ctx: Context) -> CheckResult:
    tol = ctx.task.tolerances
    limit = float(tol.get("seat_overhang_max", DEFAULT_MAX))
    setback = float(tol.get("seat_setback", DEFAULT_SETBACK))
    rows = []
    for k in ctx.poses():
        rows += overhang(ctx, k, setback)
    worst = max(rows, key=lambda r: r["outside_mm2"]) if rows else None
    if worst is None:
        return CheckResult(NAME, True, "no mounting face to stay inside", 0.0, "mm2")
    details = {"faces": rows, "setback_mm": setback}
    if worst["outside_mm2"] > limit:
        return CheckResult(NAME, False,
                           f"the part's seat on {worst['part']} has {worst['outside_mm2']:.1f} mm2 within "
                           f"{setback:g} mm of that part's mounting face edge or past it (limit {limit:g} mm2): "
                           f"keep the seat {setback:g} mm inside the face's outline",
                           worst["outside_mm2"], "mm2", details)
    return CheckResult(NAME, True, f"every seat at least {setback:g} mm inside its mounting face's edge "
                                   f"(worst {worst['outside_mm2']:.2f} mm2 beyond, limit {limit:g})",
                       worst["outside_mm2"], "mm2", details)
