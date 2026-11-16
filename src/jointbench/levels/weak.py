"""Answers that fit but are not stiff: the shapes models built in the
round of November 2026, rebuilt for any two-mount task so the self-test can
prove STIFFNESS rejects them.

* ``bar``    two plates joined by one slender bar beside the bolts;
* ``ring``   the same bar, but plate A is a ring round its bolts;
* ``spokes`` a single rod between the two bolt patterns, each end on thin
             spokes running out to a pad under every bolt.

Each keeps the minimum section where one rod can (1.5 x the requirement,
at most a 14 mm rod) and leaves every rod square to its plate, so what
fails is the stiffness, not the fit.
"""
from __future__ import annotations

import math

import build123d as bd

from ..catalogue import BOLTS, edge_margin
from . import construct as C
from . import rods as R

HC = 12.0  # mm a bar runs square out of its plate before it turns


def _rod_r(task) -> float:
    from ..checks.section_min import required_area

    need = required_area(task, "A", "B")
    return min(R.R_MAX, max(R.R_MIN, math.sqrt(1.5 * need / math.pi)))


def build(task, shape: str, toward: dict, *, lift: float = 0.0) -> bd.Part:
    r = _rod_r(task)
    plates = {}
    for part in ("A", "B"):
        bt = BOLTS[task.parts[part].bolt]
        t = max(R.PLATE_T, bt.grip_min, 2 * r / 3 + 0.1)
        plates[part] = C.plate(task, part, toward[part], t, tab=2 * r + 8.0, z0=lift if part == "A" else 0.0)
    if shape == "spokes":
        body = _spokes(task, plates, r)
    else:
        body = plates["A"]["solid"] + plates["B"]["solid"]
        if shape == "ring":
            P = plates["A"]
            e = edge_margin(BOLTS[task.parts["A"].bolt])
            w, h = P["x1"] - P["x0"] - 4 * e, P["y1"] - P["y0"] - 4 * e
            if w > 2 and h > 2:
                body = body - P["frame"] * bd.Pos((P["x0"] + P["x1"]) / 2, (P["y0"] + P["y1"]) / 2,
                                                  P["z0"] + P["t"] / 2) * bd.Box(w, h, P["t"] + 2)
        ends = []
        for part in ("A", "B"):
            P = plates[part]
            p0 = P["frame"].from_local_coords(bd.Vector(P["x1"] + P["tab"] / 2, (P["y0"] + P["y1"]) / 2,
                                                        P["z0"] + P["t"] / 2))
            ends.append((p0, p0 + P["frame"].z_dir * (P["t"] / 2 + HC)))
        body = body + R.path([ends[0][0], ends[0][1], ends[1][1], ends[1][0]], r)
    body = C.cut_holes(body, task, plates)
    body = C.cut_protrusions(body, task, plates=plates)
    solids = body.solids()
    return solids[0] if len(solids) == 1 else body


def _spokes(task, plates, r):
    """A pad under each bolt, thin spokes in the plate's layer to a hub at
    the bolt pattern's centre, and one rod from hub to hub."""
    body = None
    hubs = {}
    for part, P in plates.items():
        bt = BOLTS[task.parts[part].bolt]
        e = edge_margin(bt)
        fr, t, z = P["frame"], P["t"], P["z0"] + P["t"] / 2
        loc = P["bolts_local"]
        cx, cy = sum(p.X for p in loc) / len(loc), sum(p.Y for p in loc) / len(loc)
        hub = fr.from_local_coords(bd.Vector(cx, cy, z))
        piece = fr * bd.Pos(cx, cy, P["z0"]) * bd.extrude(bd.Circle(r + 3.0), t)
        for p in loc:
            piece = piece + fr * bd.Pos(p.X, p.Y, P["z0"]) * bd.extrude(bd.Circle(e), t)
            piece = piece + R.rod(fr.from_local_coords(bd.Vector(p.X, p.Y, z)), hub, 0.4 * t)
        body = piece if body is None else body + piece
        hubs[part] = (hub, hub + fr.z_dir * (t / 2 + HC))
    return body + R.path([hubs["A"][0], hubs["A"][1], hubs["B"][1], hubs["B"][0]], r)


SELFTEST_ROWS = [
    # (label, shape): each must fail STIFFNESS
    ("one slender bar between two plates (last round)", "bar"),
    ("ring plate with one column (last round)", "ring"),
    ("single rod on thin spokes (last round)", "spokes"),
]
