"""Small geometric plumbing: rigid placements and deterministic STEP files."""
from __future__ import annotations

import math
import re
from dataclasses import dataclass

import build123d as bd

Vec3 = tuple[float, float, float]


def r6(x: float) -> float:
    """Round for storage and reports, so files and verdicts are stable."""
    v = round(float(x), 6)
    return 0.0 if v == 0 else v


@dataclass
class Placement:
    """Rigid placement of a local frame in the world: p_world = R @ p_local + origin.

    ``rot`` is a 3x3 rotation matrix stored row by row."""

    origin: Vec3
    rot: tuple[Vec3, Vec3, Vec3]

    @staticmethod
    def from_yaw_flip(origin: Vec3, yaw_deg: float, flip: bool) -> "Placement":
        """Yaw about local Z, then (optionally) turn the part face-down by a
        half turn about world X. Level 1 needs nothing more."""
        c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        rz = ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))
        if not flip:
            r = rz
        else:
            fx = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
            r = tuple(tuple(sum(fx[i][k] * rz[k][j] for k in range(3)) for j in range(3)) for i in range(3))
        r = tuple(tuple(r6(v) for v in row) for row in r)
        return Placement(tuple(r6(v) for v in origin), r)

    def then(self, g: "Placement") -> "Placement":
        """This placement followed by ``g`` (g applied last)."""
        return Placement(g.point(self.origin), matmul(g.rot, self.rot))

    def vec(self, v: Vec3) -> Vec3:
        return tuple(sum(self.rot[i][k] * v[k] for k in range(3)) for i in range(3))

    def point(self, p: Vec3) -> Vec3:
        rv = self.vec(p)
        return tuple(rv[i] + self.origin[i] for i in range(3))

    def location(self) -> bd.Location:
        x = (self.rot[0][0], self.rot[1][0], self.rot[2][0])
        z = (self.rot[0][2], self.rot[1][2], self.rot[2][2])
        return bd.Location(bd.Plane(origin=self.origin, x_dir=x, z_dir=z))

    def to_json(self) -> dict:
        return {"origin": list(self.origin), "rotation": [list(r) for r in self.rot]}

    @staticmethod
    def from_json(d: dict) -> "Placement":
        return Placement(tuple(d["origin"]), tuple(tuple(r) for r in d["rotation"]))


def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def rot_axis_angle(axis: Vec3, deg: float):
    """Rotation matrix about a unit axis (Rodrigues)."""
    n = math.sqrt(sum(v * v for v in axis))
    x, y, z = (v / n for v in axis)
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    C = 1 - c
    return ((c + x * x * C, x * y * C - z * s, x * z * C + y * s),
            (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
            (z * x * C - y * s, z * y * C + x * s, c + z * z * C))


def rot_from_quaternion(w: float, x: float, y: float, z: float):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


_STAMP = re.compile(r"(FILE_NAME\('[^']*',)'[^']*'")
_NAUO = re.compile(r"(NEXT_ASSEMBLY_USAGE_OCCURRENCE\()'\d+'")


def write_step(shape: bd.Shape, path: str) -> None:
    """Export STEP with the timestamp fixed, so the same geometry always
    gives a byte-identical file."""
    solids = shape.solids()
    if len(solids) == 1:
        shape = solids[0]  # a bare solid, not a one-item assembly
    bd.export_step(shape, path)
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    txt = _STAMP.sub(r"\1'2000-01-01T00:00:00'", txt, count=1)
    # the STEP writer numbers assembly links with a process-wide counter;
    # renumber them in file order so the file depends on geometry only
    n = iter(range(1, 1_000_000))
    txt = _NAUO.sub(lambda m: f"{m.group(1)}'{next(n)}'", txt)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(txt)


def read_step(path: str) -> bd.Shape:
    return bd.import_step(path)


def convex_hull(points, eps: float = 1e-7) -> bd.Solid:
    """Convex hull of a handful of points, as a solid with flat faces only.

    Brute force over point triples, in plain arrays -- fine for the 8-24
    points a bridge or arm needs. Flat faces keep later booleans reliable;
    a twisted ruled loft between the same corners made the kernel drop whole
    solids on union."""
    import itertools

    import numpy as np
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    P = np.array([tuple(p) for p in points], dtype=float)
    faces, seen = [], set()
    for i, j, k in itertools.combinations(range(len(P)), 3):
        n = np.cross(P[j] - P[i], P[k] - P[i])
        ln = np.linalg.norm(n)
        if ln < 1e-9:
            continue
        n = n / ln
        d = (P - P[i]) @ n
        if d.max() > eps and d.min() < -eps:
            continue
        on = tuple(int(m) for m in np.nonzero(np.abs(d) <= eps)[0])
        if on in seen:
            continue
        seen.add(on)
        c = P[list(on)].mean(axis=0)
        u = P[on[0]] - c
        u = u / np.linalg.norm(u)
        v = np.cross(n, u)
        ring = sorted(on, key=lambda m: math.atan2(float((P[m] - c) @ v), float((P[m] - c) @ u)))
        faces.append(bd.Face(bd.Wire.make_polygon([tuple(P[m]) for m in ring], close=True)))
    solid = bd.Solid(bd.Shell(faces))
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(solid.wrapped, g)
    if g.Mass() < 0:
        solid = bd.Solid(solid.wrapped.Reversed())
    return solid


# The build volume every task is designed for: a Prusa MINI (180 x 180 x 180 mm). Not a rule the
# models are told; the task generators keep only setups whose reference part fits it with room to
# spare (maintainer, 13 November 2026), so a good answer can always be printed on a small common printer.
BUILD_VOLUME = (180.0, 180.0, 180.0)
BUILD_ROOM = 20.0  # mm: a reference part must fit the volume less this on every side, so answers have room


def build_box(shape: bd.Shape) -> list[float]:
    """The sides of a tight box around ``shape`` in its best orientation (an oriented bounding box,
    smallest first). A box that fits the build volume means the part fits; one that does not is
    a strong sign it will not (the box is close to, not always exactly, the smallest)."""
    from OCP.Bnd import Bnd_OBB
    from OCP.BRepBndLib import BRepBndLib

    obb = Bnd_OBB()
    BRepBndLib.AddOBB_s(shape.wrapped, obb, True, True, False)
    return sorted(round(2 * h, 3) for h in (obb.XHSize(), obb.YHSize(), obb.ZHSize()))


def fits_build_volume(shape: bd.Shape, room: float = 0.0) -> bool:
    """True if ``shape`` fits the build volume, less ``room`` on every side, in its best orientation."""
    return all(s <= v - room + 1e-6 for s, v in zip(build_box(shape), sorted(BUILD_VOLUME)))
