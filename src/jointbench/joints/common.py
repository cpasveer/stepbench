"""What every joint shares: results, frames, exact measurements, sweeps.

Each joint is built and checked in its own *joint frame*: the joint axis is
local z through the origin. The generator places that frame anywhere in the
world (a random rigid placement from the seed), the brief states the frame,
and a check first moves the submitted solid back into the frame by the exact
inverse placement. So every measurement below reads plain numbers on a
z-axis, while the agent still faces an arbitrary orientation.

Cylinders, overlap volumes and STEP files come from the core
(``checks.holes``, ``checks.base``, ``geometry``); nothing here copies them.
"""
from __future__ import annotations

import math
import os
import random
import tempfile
import time
from dataclasses import dataclass, field

import build123d as bd
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cone, GeomAbs_Plane, GeomAbs_Sphere

from ..checks.base import CheckResult, volume
from ..checks.holes import _cylinders, _extent
from ..geometry import Placement, read_step, rot_from_quaternion, write_step

OVERLAP_MAX = 0.01  # mm3, the core's limit (tolerances["overlap_volume_max"])
AXIS_OFFSET_MAX = 0.01  # mm, two cylinders count as coaxial within this
AXIS_ANGLE_MAX = 0.01  # degrees


def res(name, ok, reason, value=None, unit="", where=None, required=True, **details) -> CheckResult:
    """A check result in the core's shape; ``where`` is a point in the joint frame."""
    if where is not None:
        details["where"] = [round(float(c), 4) for c in where]
    return CheckResult(name, ok, reason, None if value is None else round(float(value), 6), unit,
                       details, required=required)


def zone_reason(inst, label, lo, hi, zone, code, words=("too small", "too large"), unit="mm") -> str:
    """The reason for a size checked against a standard's tolerance zone.

    Rules told: the measured size and the zone. Rules not told (level 3): the
    measured size and the direction only, never the limits or the class, so a
    failed try does not hand over what the level asks the agent to know."""
    meas = f"{lo:.4f}" if abs(hi - lo) < 5e-5 else f"{lo:.4f} to {hi:.4f}"
    if inst.rules_told:
        return f"{label} {meas} {unit} (need {zone[0]:.4f} to {zone[1]:.4f}, {code})"
    small, big = lo < zone[0] - 1e-6, hi > zone[1] + 1e-6
    verdict = (f"{words[0]} in places, {words[1]} in others" if small and big else words[1] if big
               else words[0] if small else "within the standard's limits")
    return f"{label} {meas} {unit}: {verdict}"


# ------------------------------------------------------------------ frames


def random_frame(rng: random.Random, aligned: bool = False) -> Placement:
    """The joint frame in the world. ``aligned``: axis = world z, at the origin."""
    if aligned:
        return Placement((0.0, 0.0, 0.0), ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
    q = [rng.gauss(0, 1) for _ in range(4)]
    rot = rot_from_quaternion(*q)
    origin = tuple(round(rng.uniform(-150, 150), 3) for _ in range(3))
    return Placement(origin, rot)


def to_world(shape, frame: Placement):
    return shape.moved(frame.location())


def to_local(shape, frame: Placement):
    return shape.moved(frame.location().inverse())


def frame_text(frame: Placement) -> str:
    o = frame.origin
    x, y, z = ([frame.rot[i][j] for i in range(3)] for j in range(3))
    f = lambda v: "(" + ", ".join(f"{c:.6f}" for c in v) + ")"  # noqa: E731
    return (f"Joint frame (all positions below are in it): origin {f(o)} mm in the world; "
            f"its x axis is world direction {f(x)}, y is {f(y)}, z is {f(z)}.")


def rng_for(joint: str, seed: int) -> random.Random:
    return random.Random(f"joint-{joint}-{seed}")


# --------------------------------------------------------- exact surfaces


@dataclass
class Cyl:
    loc: tuple
    axis: tuple
    r: float
    face: object
    convex: bool  # True: outer surface of a pin; False: a bore
    z0: float = 0.0  # extent along local z (joint axis)
    z1: float = 0.0

    @property
    def offset(self) -> float:
        """Distance of this cylinder's axis from the joint axis (local z)."""
        return axis_offset(self.loc, self.axis)

    @property
    def angle(self) -> float:
        """Angle between this axis and the joint axis, degrees."""
        return math.degrees(math.acos(min(1.0, abs(self.axis[2]))))


def axis_offset(loc, axis) -> float:
    """Distance between the line (loc, axis) and the z axis, measured in the
    plane z = 0 where the line crosses it (exact when parallel)."""
    if abs(axis[2]) < 1e-9:
        return math.inf
    t = -loc[2] / axis[2]
    x, y = loc[0] + t * axis[0], loc[1] + t * axis[1]
    return math.hypot(x, y)


def cylinders(solid) -> list[Cyl]:
    """Every cylindrical face, with its side (convex / concave) and its z extent."""
    out = []
    for loc, d, r, f in _cylinders(solid):
        c = f.position_at(0.5, 0.5)  # a point on the surface (a centre of mass is on the axis)
        n = tuple(f.normal_at(c))
        w = [c.X - loc[0], c.Y - loc[1], c.Z - loc[2]]
        a = sum(w[i] * d[i] for i in range(3))
        radial = [w[i] - a * d[i] for i in range(3)]
        convex = sum(radial[i] * n[i] for i in range(3)) > 0
        z0, z1 = _extent(f, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        out.append(Cyl(loc, d, r, f, convex, z0, z1))
    return out


def coaxial_group(cyls, r_near: float, convex: bool, r_window: float = 0.5,
                  off_window: float = 1.0, ang_window: float = 1.0):
    """Faces of one cylinder near radius ``r_near`` about the joint axis,
    merged (a cylinder is often split in two half faces). Returns the group
    with the longest z extent: dict(r, offset, angle, z0, z1, faces)."""
    groups = {}
    for c in cyls:
        if c.convex != convex or abs(c.r - r_near) > r_window:
            continue
        if c.angle > ang_window or c.offset > off_window:
            continue
        key = (round(c.r, 6), round(c.offset, 5))
        g = groups.setdefault(key, {"r": c.r, "offset": c.offset, "angle": c.angle,
                                    "z0": c.z0, "z1": c.z1, "faces": []})
        g["z0"], g["z1"] = min(g["z0"], c.z0), max(g["z1"], c.z1)
        g["angle"] = max(g["angle"], c.angle)
        g["faces"].append(c.face)
    if not groups:
        return None, []
    best = max(groups.values(), key=lambda g: (round(g["z1"] - g["z0"], 6), -abs(g["r"] - r_near)))
    return best, list(groups.values())


def planes(solid):
    """(point, outward normal, face) for every planar face."""
    out = []
    for f in solid.faces():
        if BRepAdaptor_Surface(f.wrapped).GetType() != GeomAbs_Plane:
            continue
        c = f.center()
        out.append((tuple(c), tuple(f.normal_at(c)), f))
    return out


def cones(solid):
    """(apex, axis, semi-angle deg, face) for every conical face."""
    out = []
    for f in solid.faces():
        ad = BRepAdaptor_Surface(f.wrapped)
        if ad.GetType() != GeomAbs_Cone:
            continue
        cn = ad.Cone()
        ap, ax = cn.Apex(), cn.Axis().Direction()
        out.append(((ap.X(), ap.Y(), ap.Z()), (ax.X(), ax.Y(), ax.Z()),
                    math.degrees(abs(cn.SemiAngle())), f))
    return out


def spheres(solid):
    """(centre, radius, face) for every spherical face."""
    out = []
    for f in solid.faces():
        ad = BRepAdaptor_Surface(f.wrapped)
        if ad.GetType() != GeomAbs_Sphere:
            continue
        s = ad.Sphere()
        c = s.Location()
        out.append(((c.X(), c.Y(), c.Z()), s.Radius(), f))
    return out


def face_z_range(face):
    """Exact z range of a face: bounding box of its vertices and edges."""
    zs = [v.Z for v in face.vertices()]
    for e in face.edges():
        for t in (0.0, 0.25, 0.5, 0.75, 1.0):
            zs.append(e.position_at(t).Z)
    return min(zs), max(zs)


def union_length(intervals) -> float:
    iv = sorted((a, b) for a, b in intervals if b > a)
    total, cur = 0.0, None
    for a, b in iv:
        if cur is None or a > cur[1]:
            if cur:
                total += cur[1] - cur[0]
            cur = [a, b]
        else:
            cur[1] = max(cur[1], b)
    if cur:
        total += cur[1] - cur[0]
    return total


# ------------------------------------------------------------------ overlap


def overlap(a, b) -> float:
    """Exact overlap volume of two solids (the core's boolean)."""
    return volume(a & b)


def overlap_shape(a, b):
    try:
        return a & b
    except Exception:  # noqa: BLE001
        return None


def fuse(parts):
    parts = list(parts)
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def rotated_z(shape, deg: float):
    return shape.moved(bd.Location((0, 0, 0), (0, 0, 1), deg))


def moved_by(shape, v):
    return shape.moved(bd.Location(tuple(v)))


def extent_along(shape, axis) -> tuple[float, float] | None:
    """Exact-enough range of a shape along a direction: its vertices and
    edge samples (exact for the straight-edged overlaps of prismatic parts)."""
    if shape is None or volume(shape) <= 0:
        return None
    vals = [sum(tuple(v)[i] * axis[i] for i in range(3)) for v in shape.vertices()]
    for e in shape.edges():
        for t in (0.0, 0.25, 0.5, 0.75, 1.0):
            p = tuple(e.position_at(t))
            vals.append(sum(p[i] * axis[i] for i in range(3)))
    return min(vals), max(vals)


# ------------------------------------------------------------------ joints


@dataclass
class Instance:
    """One generated joint: its numbers, its frame, and how it was asked."""

    joint: str
    seed: int
    frame: Placement
    p: dict  # the joint's own numbers (mm, degrees)
    rules_told: bool = True
    knobs: dict = field(default_factory=dict)

    def to_json(self) -> dict:
        return {"joint": self.joint, "seed": self.seed, "rules_told": self.rules_told,
                "knobs": self.knobs, "frame": self.frame.to_json(), "params": self.p}


def roundtrip(solid, tmpdir: str, name: str = "part.step"):
    """Write and read back a STEP, as an agent's answer would arrive."""
    path = os.path.join(tmpdir, name)
    write_step(solid, path)
    sh = read_step(path)
    s = sh.solids()
    return s[0] if len(s) == 1 else sh


def grade_step(module, inst: Instance, path: str, **kw):
    """Grade a STEP file for one instance: FILE_VALID's one-solid rule, then the joint's checks."""
    sh = read_step(path)
    sols = sh.solids()
    if len(sols) != 1:
        return [res("FILE_VALID", False, f"expected exactly 1 solid, found {len(sols)}", len(sols), "solids")]
    return module.check(sols[0], inst, **kw)


def selftest(module, seeds=(0,), verbose=True, jobs_out=None):
    """Grade the reference and every variant of ``module.SELFTEST`` for each
    seed, through a real STEP file. A row is as expected when a good part
    passes every check, or a broken part fails the named check."""
    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        for seed in seeds:
            for label, variant, expect in module.SELFTEST:
                inst = module.generate(seed, **variant.get("knobs", {}))
                t0 = time.perf_counter()
                part = module.reference(inst, **variant.get("part", {}))
                t1 = time.perf_counter()
                solid = roundtrip(to_world(part, inst.frame), tmp)
                t2 = time.perf_counter()
                results = module.check(solid, inst, **variant.get("check", {}))
                t3 = time.perf_counter()
                failed = [r.name for r in results if r.ok is False and r.required]
                good = (not failed) if expect is None else (expect in failed)
                rows.append({"seed": seed, "label": label, "expect": expect or "pass", "ok": good,
                             "failed": failed, "t_build": t1 - t0, "t_check": t3 - t2,
                             "results": results})
                if verbose:
                    mark = "as expected" if good else "NOT AS EXPECTED"
                    why = next((r.reason for r in results if r.name == expect), "") if expect else \
                        ("; ".join(r.reason for r in results if r.ok is False) or "all checks pass")
                    print(f"  s{seed:<3} {label:<34} expect {expect or 'pass':<18} {mark:<16} "
                          f"{t3 - t2:5.2f} s  {why[:110]}")
    return rows
