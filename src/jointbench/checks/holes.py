"""HOLES: a clearance hole for every bolt, on its axis, of the right size.

The hole is found by measurement of the exact boundary: a cylindrical face
of the submitted solid whose axis is parallel to the bolt axis and passes
near the bolt. Only a concave cylinder counts -- material outside it, as
round a hole; a rounded outside corner (a convex fillet) is not a hole.
Of these, the one whose axis is nearest the bolt axis is the hole. From
that face we read three numbers:

* axis offset -- distance between the hole's axis and the bolt's axis;
* diameter -- must lie in the ISO 273 fine..coarse range for the bolt;
* grip -- distance from the mounting face to where the hole ends, i.e.
  where the bolt head will bear. It must suit a stock screw.

How the hole was modelled does not matter; any CAD program that writes a
true cylinder passes the same way.
"""
from __future__ import annotations

import math

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder

from ..catalogue import BOLTS
from .base import CheckResult, Context

NAME = "HOLES"


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _cylinders(solid):
    """(axis point, axis direction, radius, face) for every CONCAVE
    cylindrical face: one whose material lies outside it, as round a hole.
    A convex one (a boss, a rounded outside corner) is no hole."""
    out = []
    for f in solid.faces():
        ad = BRepAdaptor_Surface(f.wrapped)
        if ad.GetType() != GeomAbs_Cylinder:
            continue
        cyl = ad.Cylinder()
        ax = cyl.Axis()
        loc, d = ax.Location(), ax.Direction()
        loc, d = (loc.X(), loc.Y(), loc.Z()), (d.X(), d.Y(), d.Z())
        if not _concave(f, loc, d):
            continue
        out.append((loc, d, cyl.Radius(), f))
    return out


def _concave(face, loc, d) -> bool:
    """True when the face's outward normal (out of the material) points
    towards the cylinder's axis: the material is outside the cylinder."""
    q = face.position_at(0.5, 0.5)
    n = face.normal_at(q)
    w = _sub(loc, tuple(q))
    along = _dot(w, d)
    to_axis = [w[i] - along * d[i] for i in range(3)]
    return _dot(tuple(n), to_axis) > 0


def _extent(face, p, a):
    """Range of the face along the bolt axis, measured from the bolt point."""
    s = []
    for e in face.edges():
        for t in (0.0, 0.25, 0.5, 0.75, 1.0):
            s.append(_dot(_sub(tuple(e.position_at(t)), p), a))
    return min(s), max(s)


def measure(solid, bolt, tol_search: float | None = None):
    """Best matching hole for one bolt, or None. Returns a dict of numbers."""
    b = BOLTS[bolt.size]
    p, a = bolt.position, bolt.axis
    search = tol_search if tol_search is not None else b.d
    groups: dict = {}
    for loc, d, r, f in _cylinders(solid):
        if abs(abs(_dot(d, a)) - 1.0) > 1e-6:
            continue
        if not (0.3 * b.d <= r < b.head_d / 2):
            continue
        w = _sub(p, loc)
        along = _dot(w, d)
        perp = [w[i] - along * d[i] for i in range(3)]
        off = math.sqrt(_dot(perp, perp))
        if off > search:
            continue
        lo, hi = _extent(f, p, a)
        key = (round(off, 5), round(r, 5))
        if key in groups:  # split cylinder faces of one hole
            g = groups[key]
            g["start"], g["end"] = min(g["start"], lo), max(g["end"], hi)
        else:
            groups[key] = {"offset": off, "diameter": 2 * r, "start": lo, "end": hi}
    if not groups:
        return None
    # the hole is the one whose axis is nearest the bolt axis; of the
    # coaxial faces there, the clearance hole is the one that starts nearest
    # the mounting face (a counterbore above it is larger and starts higher)
    near = min(g["offset"] for g in groups.values())
    coaxial = [g for g in groups.values() if g["offset"] <= near + 1e-3]
    return min(coaxial, key=lambda g: (round(g["start"], 4), g["diameter"]))


def run(ctx: Context) -> CheckResult:
    tol = ctx.task.tolerances["hole_axis_offset_max"]
    problems, worst_off, rows = [], 0.0, []
    for k in ctx.poses():
        for bolt in ctx.task.bolts(ctx.stored(k)):
            b = BOLTS[bolt.size]
            m = measure(ctx.solid_at(k), bolt)
            if m is None:
                problems.append(f"{bolt.id}: no {bolt.size} hole found within {b.d:g} mm of the bolt axis")
                worst_off = max(worst_off, math.inf)
                ctx.grips[(k, bolt.id)] = b.grip_min
                continue
            grip = m["end"]
            ctx.grips[(k, bolt.id)] = grip
            worst_off = max(worst_off, m["offset"])
            rows.append({"bolt": bolt.id, "offset": round(m["offset"], 4),
                         "diameter": round(m["diameter"], 4), "grip": round(grip, 4)})
            if m["offset"] > tol + 1e-9:
                problems.append(f"{bolt.id}: hole axis {m['offset']:.3f} mm off the bolt axis (tol {tol:g})")
            if not (b.clear_fine - 1e-3 <= m["diameter"] <= b.clear_coarse + 1e-3):
                problems.append(f"{bolt.id}: hole diameter {m['diameter']:.3f} outside "
                                f"{b.clear_fine:g}-{b.clear_coarse:g} for {bolt.size}")
            if not (b.grip_min - 1e-3 <= grip <= b.grip_max + 1e-3):
                problems.append(f"{bolt.id}: grip {grip:.2f} mm outside {b.grip_min:g}-{b.grip_max:g} "
                                f"(material under the head)")
    value = None if math.isinf(worst_off) else round(worst_off, 4)
    n = len(rows) + sum(1 for p in problems if "no " in p and "hole found" in p)
    if problems:
        extra = f" (+{len(problems) - 1} more)" if len(problems) > 1 else ""
        return CheckResult(NAME, False, problems[0] + extra, value, "mm", {"holes": rows, "problems": problems})
    dia = [r["diameter"] for r in rows]
    grips = [r["grip"] for r in rows]
    return CheckResult(
        NAME, True,
        f"{n} holes on axis within {worst_off:.3f} mm (tol {tol:g}), "
        f"dia {min(dia):.2f}-{max(dia):.2f}, grip {min(grips):.1f}-{max(grips):.1f} mm",
        value, "mm", {"holes": rows},
    )
