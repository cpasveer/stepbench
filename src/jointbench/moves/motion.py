"""Moving bodies, and measuring how close they come.

Turning and shifting placements, the exact distance between two shapes, and
the small moves (shifts in 26 directions, tilts) used to probe a pivot for
play. Distances are OpenCASCADE's BRepExtrema on the B-rep; overlaps are
exact boolean volumes, as everywhere else in the grader.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import build123d as bd
import numpy as np
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

from ..checks.base import volume
from ..geometry import Placement, rot_axis_angle

CONTACT = 1e-6  # mm: closer than this is touching
FAR = 5.0  # mm: bodies whose boxes are this far apart are not measured exactly


def about(point, axis, deg: float) -> Placement:
    """Turning by ``deg`` about the line through ``point`` along ``axis`` (right hand)."""
    r = rot_axis_angle(tuple(axis), deg)
    p = np.asarray(point, float)
    rp = np.array([sum(r[i][k] * p[k] for k in range(3)) for i in range(3)])
    return Placement(tuple(p - rp), r)


def shift(v) -> Placement:
    return Placement(tuple(float(x) for x in v), ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))


def then(a: Placement, b: Placement) -> Placement:
    """a, then b."""
    return a.then(b)


_BOX: dict = {}


def _bbox(s):
    """A shape's bounding box, remembered for shapes that are measured again and again (kept while alive)."""
    k = id(s)
    got = _BOX.get(k)
    if got is None or got[0] is not s:
        if len(_BOX) > 20000:
            _BOX.clear()
        got = (s, s.bounding_box())
        _BOX[k] = got
    return got[1]


def distance(a, b) -> float:
    """Exact smallest distance between two shapes; 0 when they touch or overlap
    (a shape inside a solid counts as overlapping)."""
    ba, bb = _bbox(a), _bbox(b)
    gap = max(ba.min.X - bb.max.X, bb.min.X - ba.max.X, ba.min.Y - bb.max.Y, bb.min.Y - ba.max.Y,
              ba.min.Z - bb.max.Z, bb.min.Z - ba.max.Z, 0.0)
    if gap > FAR:  # boxes far apart: the true distance is at least this, and nothing needs it exactly
        return gap
    d = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
    if not d.IsDone():
        raise RuntimeError("the distance between two bodies could not be measured")
    v = float(d.Value())
    if v > CONTACT and d.InnerSolution():
        return 0.0
    return v


@dataclass
class Sides:
    """Two groups of bodies, as (name, shape) pieces at their assembled place: the ``fixed``
    group stays, the ``moving`` group is placed by a Placement. Pairs named in ``skip``
    (moving name, fixed name) are never measured."""

    fixed: list
    moving: list
    point: np.ndarray
    axis: np.ndarray
    skip: frozenset = frozenset()

    def pairs(self, pl: Placement | None):
        for mn, ms in self.moving:
            msp = ms if pl is None else ms.moved(pl.location())
            for fn, fs in self.fixed:
                if (mn, fn) in self.skip:
                    continue
                yield mn, msp, fn, fs

    def closest(self, pl: Placement | None, stop_below: float | None = None):
        """(distance, moving piece, fixed piece) of the closest pair; stops early once a pair
        is closer than ``stop_below``."""
        best = (math.inf, "", "")
        for mn, ms, fn, fs in self.pairs(pl):
            d = distance(ms, fs)
            if d < best[0]:
                best = (d, mn, fn)
                if stop_below is not None and d < stop_below:
                    break
        return best

    def touching(self, pl: Placement) -> bool:
        return self.closest(pl, stop_below=CONTACT)[0] <= CONTACT


def frame(axis) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The turn axis and two directions square to it, for naming moves."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    up = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e2 = up - a * float(up @ a)
    e2 /= np.linalg.norm(e2)
    e3 = np.cross(a, e2)
    return a, e2, e3


def steps(range_deg: float, reach: float, max_step: float, max_move: float) -> list[float]:
    """The turn angles checked: 0 to the end of the range, the step at most
    ``max_step`` degrees and small enough that the moving side's farthest
    point moves at most ``max_move`` mm per step; both ends included."""
    by_move = math.degrees(max_move / reach) if reach > 0 else max_step
    st = min(max_step, by_move)
    n = max(1, math.ceil(range_deg / st - 1e-9))
    return [range_deg * i / n for i in range(n + 1)]


def moves(axis, point, which: str, amount: float):
    """(name, placement) for the small moves of the moving side.

    which="principal": +-along the axis and +-along the two directions square
    to it, and the four tilts (about the two directions square to the axis,
    through the stated axis point). which="all": every one of the 26
    directions of a cube's faces, edges and corners in that frame, plus the
    four tilts. Translations move ``amount`` mm, tilts ``amount`` degrees."""
    a, e2, e3 = frame(axis)
    out = []
    if which == "principal":
        dirs = [(f"{s}axis", sg * a) for s, sg in (("+", 1), ("-", -1))]
        dirs += [(f"{s}{n}", sg * e) for n, e in (("across", e2), ("square", e3)) for s, sg in (("+", 1), ("-", -1))]
    else:
        dirs = []
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                for k in (-1, 0, 1):
                    if i == j == k == 0:
                        continue
                    v = i * a + j * e2 + k * e3
                    dirs.append((f"({i:+d} axis, {j:+d} across, {k:+d} square)", v / np.linalg.norm(v)))
    for n, v in dirs:
        out.append(("shift " + n, "mm", v, lambda t, v=v: shift(t * v)))
    for n, e in (("across", e2), ("square", e3)):
        for s, sg in (("+", 1), ("-", -1)):
            out.append((f"tilt {s}{n}", "deg", sg * e, lambda t, e=e, sg=sg: about(point, sg * e, t)))
    return out
