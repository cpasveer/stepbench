"""A part's way in: one shared sweep for every level whose parts go in through an
opening (4a, the pipe's 4 and 5, level 6).

A way in is a list of steps in world coordinates, applied in order, ending
exactly at the part's place (where its file shows it):

    [{"move": [dx, dy, dz]},
     {"turn": {"point": [x, y, z], "axis": [ax, ay, az], "degrees": d}}, ...]

(``"deg"`` is taken for ``"degrees"``.) The start is the place with every step
undone. Either the part first comes straight in from beyond everything along a
stated direction to that start (``direction``: the levels' ``insert_direction``),
or, with no direction, the start itself must lie wholly outside the box. Straight
stretches are swept exactly (``sweep``); a turn is checked in steps of at most
``sweep.TURN_STEP`` mm of travel of the part's farthest point, so a part may be
tilted on its way in.
"""
from __future__ import annotations

import numpy as np

from .sweep import describe, path_collisions, path_start, _points_of

__all__ = ["sweep_path", "path_start", "outside"]


def outside(part, box: list, tol: float = 1e-6) -> bool:
    """Whether the part lies wholly outside the box: no point of it inside the convex hull of the box's
    walls (the box is convex, its walls round it)."""
    from scipy.spatial import Delaunay

    pts = np.concatenate([_points_of(s) for s in box])
    hull = Delaunay(pts)
    return not bool((hull.find_simplex(_points_of(part), tol=tol) >= 0).any())


def sweep_path(part, segments: list[dict], bodies: dict, *, direction=None, box: list | None = None):
    """Sweep ``part`` along its way in against ``bodies`` (name -> solid).

    Returns (worst overlap per body in mm3, one row per stretch {"stretch", "hits"}, problem): problem is
    None, or why the way in is not a way in (a path with no direction that starts inside the box)."""
    if direction is not None:
        hits, rows = path_collisions(part, direction, segments, bodies)
        return hits, rows, None
    if not segments:
        return {n: 0.0 for n in bodies}, [], "no way in: neither an insert direction nor a path"
    start = path_start(part, segments)
    problem = None
    if box and not outside(start, box):
        problem = "its path starts inside the box (with no insert direction, it must start wholly outside)"
    hits, rows = path_collisions(part, None, segments, bodies)
    return hits, rows, problem


def describe_way(direction, segments) -> str:
    head = f"along {describe(direction)}" if direction is not None else "from outside the box"
    return head + (f" and {len(segments)} more moves and turns" if segments else "")
