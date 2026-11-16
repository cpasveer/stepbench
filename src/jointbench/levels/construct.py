"""Construction pieces shared by the level-3 reference and the straight
baselines. Everything here is built from what ``task.json`` says (faces,
normals, bolts), so it works for any placement and any number of parts.
"""
from __future__ import annotations

import math

import build123d as bd

from ..catalogue import BOLTS, edge_margin
from ..geometry import convex_hull
from ..task import Task

TAB = 14.0


def _v(p) -> bd.Vector:
    return bd.Vector(*p)


def hub_point(task: Task, pose: int = 0, out: float = 25.0) -> bd.Vector:
    """A meeting point in connector space: the mean of a point ``out`` mm in
    front of each mounting face."""
    pts = [_v(f.origin) + _v(f.normal) * out for f in task.faces(pose)]
    return sum(pts, bd.Vector()) * (1.0 / len(pts))


def plate_frame(task: Task, part: str, toward: bd.Vector, pose: int = 0) -> bd.Plane:
    f = next(f for f in task.faces(pose) if f.part == part)
    n = _v(f.normal)
    w = toward - _v(f.origin)
    u = w - n * w.dot(n)
    if u.length < 1e-6:
        return bd.Plane(origin=f.origin, z_dir=n)
    return bd.Plane(origin=f.origin, x_dir=u, z_dir=n)


def plate(task: Task, part: str, toward: bd.Vector, t: float, *, tab: float = TAB,
          z0: float = 0.0, pose: int = 0) -> dict:
    """A plate on ``part``'s face covering its bolts, with a tab of length
    ``tab`` on the side facing ``toward``."""
    pl = plate_frame(task, part, toward, pose)
    bt = BOLTS[task.parts[part].bolt]
    loc = [pl.to_local_coords(_v(b.position)) for b in task.bolts(pose) if b.part == part]
    e = edge_margin(bt)
    x0, x1 = min(p.X for p in loc) - e, max(p.X for p in loc) + e
    y0, y1 = min(p.Y for p in loc) - e, max(p.Y for p in loc) + e
    solid = pl * bd.Pos((x0 + x1 + tab) / 2, (y0 + y1) / 2, z0) * bd.extrude(
        bd.Rectangle(x1 + tab - x0, y1 - y0), t)
    return {"part": part, "frame": pl, "solid": solid, "t": t, "z0": z0, "tab": tab,
            "x0": x0, "x1": x1, "y0": y0, "y1": y1, "bolts_local": loc}


def tab_quad(p: dict, width: float | None = None, inset: float = 0.5) -> list[bd.Vector]:
    """Corners of the tab's top, ``inset`` below the surface (pieces that
    only touch face to face are where the kernel's union is unreliable)."""
    y0, y1 = p["y0"], p["y1"]
    if width is not None:
        yc = (y0 + y1) / 2
        y0, y1 = yc - width / 2, yc + width / 2
    z = p["z0"] + p["t"] - inset
    x0, x1 = p["x1"], p["x1"] + p["tab"]
    return [p["frame"].from_local_coords(bd.Vector(x, y, z)) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]


def centre(points) -> bd.Vector:
    return sum(points, bd.Vector()) * (1.0 / len(points))


def cube_points(c: bd.Vector, side: float, axes=None) -> list[bd.Vector]:
    """Corners of a cube of ``side`` centred on ``c``, edges along ``axes``
    (three unit vectors; world axes if None). Aligning the cube with an arm
    keeps the arm exactly ``side`` wide; a world-aligned cube seen along a
    tilted arm is up to 1.7 times wider."""
    h = side / 2
    e1, e2, e3 = axes or (bd.Vector(1, 0, 0), bd.Vector(0, 1, 0), bd.Vector(0, 0, 1))
    return [c + e1 * (sx * h) + e2 * (sy * h) + e3 * (sz * h)
            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]


def hull(points) -> bd.Solid:
    return convex_hull([tuple(p) for p in points])


def cut_holes(body, task: Task, plates: dict, *, pose: int = 0, hole_shift: float = 0.0,
              hole_d: float | None = None, shift_part: str = "A"):
    for b in task.bolts(pose):
        p = plates[b.part]
        bt = BOLTS[b.size]
        dia = hole_d if (hole_d is not None and b.part == shift_part) else bt.clear_medium
        pos = _v(b.position)
        if b.part == shift_part and hole_shift:
            pos = pos + p["frame"].x_dir * hole_shift
        h = p["z0"] + p["t"] + 2.0
        body = body - bd.Plane(origin=pos, z_dir=b.axis) * bd.Pos(0, 0, h / 2 - 1.0) * bd.Cylinder(dia / 2, h)
    return body


def cut_protrusions(body, task: Task, pose: int = 0, plates: dict | None = None):
    """Room for what sticks out of a face. Where a pocket would leave less
    than 2 mm of skin on that face's plate, the clearance goes right through."""
    for part, itf in task.parts.items():
        pp = task.poses[pose][part]
        n = pp.vec((0, 0, 1))
        top = (plates[part]["z0"] + plates[part]["t"]) if plates and part in plates else None
        for pr in itf.protrusions:
            c = pp.point((pr.x, pr.y, 0.0))
            h = pr.height + 1.0
            if top is not None and top - h < 2.0:
                h = top + 2.0
            body = body - bd.Plane(origin=c, z_dir=n) * bd.Pos(0, 0, h / 2 - 0.01) * bd.Cylinder(
                pr.radius + 0.5, h + 0.02)
    return body


def bolt_corridor(task: Task, bolt, tol: dict, grip: float) -> bd.Part:
    """The space a bolt and a straight tool need when the part under the head
    is ``grip`` thick: from the face out to grip + head + tool. Obstacles are
    kept out of it for the reference's grip, so an answer at least as thin
    as the reference can always reach every bolt; a thicker answer that
    pushes its own tool path into an obstacle has made that choice."""
    b = BOLTS[bolt.size]
    r = max(b.head_d / 2 + tol["head_clearance_radial"], (b.head_d + tol.get("tool_diameter_extra", 0.0)) / 2)
    L = grip + b.head_h + max(tol["head_clearance_axial"], tol.get("tool_length", 0.0))
    solid = bd.Plane(origin=bolt.position, z_dir=bolt.axis) * bd.Pos(0, 0, L / 2) * bd.Cylinder(r, L)
    # remembered for a cheap "certainly far apart" test before exact booleans
    solid.corridor = (bd.Vector(*bolt.position), bd.Vector(*bolt.axis), r, L)
    return solid


def far_from_corridor(shape, corridor) -> bool:
    """True when ``shape``'s bounding sphere is farther from the corridor's
    axis segment than the two radii together: then they cannot touch."""
    p, a, r, L = corridor
    bb = shape.bounding_box()
    c = bd.Vector((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2, (bb.min.Z + bb.max.Z) / 2)
    rho = (bb.max - bb.min).length / 2
    t = max(0.0, min(L, (c - p).dot(a)))
    return (c - (p + a * t)).length > r + rho


def perpendiculars(s: bd.Vector, n: int = 12) -> list[bd.Vector]:
    """``n`` unit directions perpendicular to ``s``, evenly round it."""
    a = bd.Vector(1, 0, 0) if abs(s.X) < 0.9 else bd.Vector(0, 1, 0)
    u = s.cross(a).normalized()
    v = s.cross(u).normalized()
    return [u * math.cos(2 * math.pi * k / n) + v * math.sin(2 * math.pi * k / n) for k in range(n)]
