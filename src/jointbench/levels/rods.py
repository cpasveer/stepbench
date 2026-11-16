"""Reference members that can be made and are cleanly modelled.

The references are plates joined by round rods. Every rod starts inside a
plate (at half its thickness) and leaves it square to its face; where a
rod path bends, a sphere of the rod's radius makes the bend smooth. So:

* every join is either smooth (rod into sphere) or a right angle (rod out of
  a plate) -- no acute inside corner, which a printer cannot clear and the
  quality checker flags as a crevice;
* no solid section is thicker than the rod's diameter (at most 14 mm,
  inside powder bed fusion's 15 mm limit), and a rod is at most 3 x a 5 mm
  plate, so wall thickness never changes abruptly;
* pieces overlap in volume, never just touch, so the kernel's unions are
  reliable without the thin overlaps that leave stubs and slivers.

A connection that needs more section than one 14 mm rod carries uses
several rods side by side.
"""
from __future__ import annotations

import math

import build123d as bd

R_MAX = 7.0      # rod radius: 14 mm across, inside the 15 mm section limit
R_MIN = 3.0      # thinner rods are fragile and close to the wall rules
GAP = 4.0        # clear space between side-by-side rods
PLATE_T = 5.0    # plate thickness where the bolts allow it


def rod(p: bd.Vector, q: bd.Vector, r: float) -> bd.Part:
    v = q - p
    return bd.Plane(origin=tuple(p), z_dir=tuple(v.normalized())) * bd.Pos(0, 0, v.length / 2) * bd.Cylinder(r, v.length)


def path(points: list, r: float) -> bd.Part:
    """Rods along a polyline, a sphere at every interior point."""
    pts = [points[0]]
    for p in points[1:]:
        if (p - pts[-1]).length > 1e-3:  # drop repeated points: no zero-length rods
            pts.append(p)
    body = None
    for a, b in zip(pts[:-1], pts[1:]):
        piece = rod(a, b, r)
        body = piece if body is None else body + piece
    for p in pts[1:-1]:
        body = body + bd.Pos(*p) * bd.Sphere(r)
    if body is None or not body.solids():
        raise ValueError("rod path produced no solid")
    return body


def sizing(area: float, r_max: float = R_MAX) -> tuple[int, float]:
    """How many rods, and their radius, to carry ``area`` of section."""
    n = max(1, math.ceil(area / (math.pi * r_max ** 2)))
    r = max(R_MIN, math.sqrt(area / n / math.pi))
    return n, r


def offsets(n: int, r: float) -> list[float]:
    pitch = 2 * r + GAP
    return [(i - (n - 1) / 2) * pitch for i in range(n)]


def min_bend_ok(points: list, min_deg: float = 60.0) -> bool:
    """Every bend keeps at least ``min_deg`` between the two rods (a sharper
    bend leaves a narrow crevice beside the sphere)."""
    for a, b, c in zip(points[:-2], points[1:-1], points[2:]):
        u, v = (a - b), (c - b)
        if u.length < 1e-9 or v.length < 1e-9:
            continue
        cosang = max(-1.0, min(1.0, u.normalized().dot(v.normalized())))
        if math.degrees(math.acos(cosang)) < min_deg:
            return False
    return True


def seg_seg(p1, q1, p2, q2) -> float:
    """Shortest distance between two line segments (exact)."""
    import numpy as np

    P1, Q1, P2, Q2 = (np.array(tuple(v), float) for v in (p1, q1, p2, q2))
    d1, d2, r = Q1 - P1, Q2 - P2, P1 - P2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    if a <= 1e-12 and e <= 1e-12:
        return float(np.linalg.norm(r))
    if a <= 1e-12:
        s, t = 0.0, min(1.0, max(0.0, f / e))
    else:
        c = d1 @ r
        if e <= 1e-12:
            t, s = 0.0, min(1.0, max(0.0, -c / a))
        else:
            b = d1 @ d2
            den = a * e - b * b
            s = min(1.0, max(0.0, (b * f - c * e) / den)) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, min(1.0, max(0.0, -c / a))
            elif t > 1.0:
                t, s = 1.0, min(1.0, max(0.0, (b - c) / a))
    return float(np.linalg.norm((P1 + d1 * s) - (P2 + d2 * t)))
