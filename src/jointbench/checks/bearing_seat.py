"""BEARING_SEAT: a ball bearing given in space is held as a designer would.

The bearing is a fixed part of kind ``bearing``: its axis is its local +Z,
it enters by the face at z = 0 and is pressed in along +Z until its far face
(z = B) meets a shoulder. Its own solid already keeps the part out of the
bearing (NO_INTERFERENCE). This check measures the rest, exactly:

* **bore**: a cylindrical face of the part on the bearing's axis (within
  ``bearing_axis_max`` mm and 0.1 degree), its diameter from D to D +
  ``bearing_fit_max`` (a press fit band), covering at least 90 % of the
  bearing's width;
* **wall**: material all round the bore, at least ``bearing_wall_min`` mm
  thick over the bearing's width (at least 90 % of that ring is part);
* **shoulder**: past the far face, a ring of part that stops the outer race
  (at least 90 % of the ring between the inner race's outside and the outer
  race's inside, 0.5 mm deep);
* **inner race free**: nothing of the part within the inner race's outside
  radius, along the whole axis, so the shaft passes and the inner race turns;
* **way in**: nothing of the part, and nothing of any other fixed part, in the
  bearing's path from outside to its seat (its outside cylinder, from the
  entry face back along the axis).

Required only where a task lists it.
"""
from __future__ import annotations

import math

import build123d as bd
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder

from ..catalogue import bearing_dims
from .base import CheckResult, Context, volume

NAME = "BEARING_SEAT"
GROUP = "fits"
FIT_MAX = 0.10      # mm on the diameter: the bore may be D to D + this
AXIS_MAX = 0.05     # mm the bore's axis may sit off the bearing's axis
WALL_MIN = 2.0      # mm of material round the bore
FILL = 0.90         # share of a ring that must be part
FAR = 400.0         # mm: "the whole axis" and "from outside"


def inner_race_radius(d: float, D: float) -> float:
    """The inner race's outside radius, about a third of the way out (ISO 15 rings)."""
    return d / 2 + 0.35 * (D - d) / 2


def _ring(r0: float, r1: float, z0: float, z1: float):
    h = z1 - z0
    return bd.Pos(0, 0, z0 + h / 2) * (bd.Cylinder(r1, h) - bd.Cylinder(r0, h + 1)) if r0 > 0 else \
        bd.Pos(0, 0, z0 + h / 2) * bd.Cylinder(r1, h)


def measure(solid, pl, d: float, D: float, B: float, others=(), tol: dict | None = None) -> dict:
    """Every measure of the seat, for a part ``solid`` and a bearing placed by ``pl`` (a Placement)."""
    tol = tol or {}
    fit_max = float(tol.get("bearing_fit_max", FIT_MAX))
    axis_max = float(tol.get("bearing_axis_max", AXIS_MAX))
    wall = float(tol.get("bearing_wall_min", WALL_MIN))
    loc = pl.location()
    origin = bd.Vector(*pl.point((0.0, 0.0, 0.0)))
    axis = bd.Vector(*pl.vec((0.0, 0.0, 1.0))).normalized()
    out = {}
    # bore: cylindrical faces on the axis, radius in the band, covering the width
    covered = []
    for f in solid.faces():
        ad = BRepAdaptor_Surface(f.wrapped)
        if ad.GetType() != GeomAbs_Cylinder:
            continue
        cy = ad.Cylinder()
        r = cy.Radius()
        if not (D / 2 - 1e-4 <= r <= D / 2 + fit_max / 2 + 1e-4):
            continue
        a = cy.Axis()
        u = bd.Vector(a.Direction().X(), a.Direction().Y(), a.Direction().Z())
        if abs(u.dot(axis)) < math.cos(math.radians(0.1)):
            continue
        p = bd.Vector(a.Location().X(), a.Location().Y(), a.Location().Z()) - origin
        off = (p - axis * p.dot(axis)).length
        if off > axis_max:
            continue
        zs = [(bd.Vector(*v) - origin).dot(axis) for v in f.vertices()] or []
        bb = f.bounding_box()
        zs += [(bd.Vector(*c) - origin).dot(axis) for c in (bb.min, bb.max)]
        covered.append((max(0.0, min(zs)), min(B, max(zs)), r, off))
    span = 0.0
    if covered:  # union of the covered stretches of [0, B]
        iv = sorted((a, b) for a, b, _, _ in covered if b > a)
        cur = None
        for a, b in iv:
            if cur and a <= cur[1]:
                cur[1] = max(cur[1], b)
            else:
                if cur:
                    span += cur[1] - cur[0]
                cur = [a, b]
        if cur:
            span += cur[1] - cur[0]
    out["bore"] = {"ok": span >= 0.9 * B - 1e-6, "covered_mm": round(span, 3), "width_mm": B,
                   "diameters": sorted({round(2 * r, 4) for _, _, r, _ in covered})}
    placed = lambda s: s.moved(loc)  # noqa: E731
    ring = placed(_ring(D / 2 + fit_max / 2, D / 2 + wall, 0.0, B))
    fill = volume(solid & ring) / max(volume(ring), 1e-9)
    out["wall"] = {"ok": fill >= FILL, "share": round(fill, 4), "min_mm": wall}
    ri = inner_race_radius(d, D)
    sh = placed(_ring(ri + 0.3, D / 2 - 0.3, B, B + 0.5))
    fill = volume(solid & sh) / max(volume(sh), 1e-9)
    out["shoulder"] = {"ok": fill >= FILL, "share": round(fill, 4)}
    free = placed(bd.Pos(0, 0, 0) * bd.Cylinder(ri, 2 * FAR))
    v = volume(solid & free)
    out["inner_race_free"] = {"ok": v <= 0.01, "overlap_mm3": round(v, 4), "radius_mm": round(ri, 3)}
    way = placed(bd.Pos(0, 0, -FAR / 2 - 0.01) * bd.Cylinder(D / 2 - 0.01, FAR))
    v = volume(solid & way)
    blocked = [n for n, s in others if volume(s & way) > 0.01]
    out["way_in"] = {"ok": v <= 0.01 and not blocked, "overlap_mm3": round(v, 4), "blocked_by": blocked}
    return out


WHY = {"bore": "no bore of {D:g}-{Dmax:g} mm on the bearing's axis covering its width ({covered:.1f} of {B:g} mm)",
       "wall": "the bore's wall is not {wall:g} mm thick all round ({share:.0%} of the ring is part)",
       "shoulder": "nothing stops the bearing at its far face (shoulder {share:.0%} filled)",
       "inner_race_free": "the part reaches inside the inner race's radius ({r:.1f} mm): the shaft cannot pass",
       "way_in": "the bearing cannot be pressed in from its entry side ({what})"}


def run(ctx: Context) -> CheckResult:
    t = ctx.task
    rows, problems = [], []
    for name, itf in t.parts.items():
        if itf.kind != "bearing":
            continue
        d, D, B = bearing_dims(itf)
        for k in ctx.poses():
            loc = t.poses[ctx.stored(k)][name]
            others = [(n, s) for n, s in ctx.fixed(k).items() if n != name]
            m = measure(ctx.solid_at(k), loc, d, D, B, others, t.tolerances)
            rows.append({"bearing": name, **m})
            fit = float(t.tolerances.get("bearing_fit_max", FIT_MAX))
            for key in ("bore", "wall", "shoulder", "inner_race_free", "way_in"):
                if not m[key]["ok"]:
                    x = m[key]
                    what = ("the part is in the way" if x.get("overlap_mm3", 0) > 0.01 else
                            "blocked by " + ", ".join(x.get("blocked_by") or []))
                    problems.append(f"{name}: " + WHY[key].format(
                        D=D, Dmax=D + fit, covered=x.get("covered_mm", 0), B=B, wall=x.get("min_mm", 0),
                        share=x.get("share", 0), r=x.get("radius_mm", 0), what=what))
    if not rows:
        return CheckResult(NAME, None, "no bearing in this task")
    if problems:
        more = f" (+{len(problems) - 1} more)" if len(problems) > 1 else ""
        return CheckResult(NAME, False, problems[0] + more, len(problems), "faults", {"bearings": rows, "problems": problems})
    return CheckResult(NAME, True, "every bearing seated: bore in the fit band, wall, shoulder, inner race free, "
                       "a straight way in", 0, "faults", {"bearings": rows})
