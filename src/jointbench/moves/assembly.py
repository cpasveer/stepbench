"""The answer's assembly file, and the catalogue fasteners it may use.

A "moves" answer is rigid parts plus an assembly file, ``assembly.json``, in
the task's world coordinates with fixture B at its start pose:

    {"parts": [{"name": "frame", "file": "frame.step", "bolts_to": "A", "insert_direction": [0, -1, 0]},
               {"name": "carrier", "file": "carrier.step", "bolts_to": "B", "insert_direction": [0, -1, 0]},
               {"name": "link1", "file": "link1.step", "bolts_to": null, "insert_direction": [1, 0, 0]}],
     "pivots": [{"name": "P1", "joins": ["frame", "link1"], "point": [x, y, z], "axis": [ax, ay, az]}, ...],
     "drive": {"pivot": "P1", "range_deg": 80},
     "fasteners": [{"kind": "bolt", "size": "M5", "length": 30, "head_at": [x, y, z],
                    "axis": [ax, ay, az], "grip": 22.8, "held_by": "frame"}]}

A part bolts to fixture A, to fixture B, or to neither (``null``: a link
that is held only by its pivots). Parts bolted to the same fixture move as
one body with it. A pivot joins two parts of different bodies about an axis
(a point on it and its direction). The drive turns the second part of its
pivot relative to the first, right-handed about the pivot's axis, from 0 to
``range_deg`` (negative turns the other way).

Parts go in first, in the order listed, each along its straight
``insert_direction`` from outside the scene; a part bolted to a fixture is
bolted as it arrives. The fasteners follow, in order: an ISO 4762 socket head
cap screw of the catalogue (``jointbench.catalogue.BOLTS``) with an ISO 4032
hex nut, clamped to the part named ``held_by`` (it moves with that part's
body). ``head_at`` is the centre of the head's bearing face, ``axis`` points
from the head into the shank, ``grip`` is the distance from the head's
bearing face to the nut's bearing face. The screw goes in along ``axis``,
shank first; the nut comes on from the other end.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass

import build123d as bd
import numpy as np

from ..catalogue import BOLTS

ASSEMBLY_FILE = "assembly.json"
MAX_PARTS = 8

# ISO 4032 hex nuts: width across flats s, height m (mm)
NUTS = {"M3": (5.5, 2.4), "M4": (7.0, 3.2), "M5": (8.0, 4.7), "M6": (10.0, 5.2), "M8": (13.0, 6.8)}
# ISO 4762 preferred lengths (mm)
LENGTHS = (6, 8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 80, 90, 100)
SEAT_SLAB = 0.02  # mm: thickness of the ring used to measure where a head or nut bears


class AssemblyError(ValueError):
    pass


def nut_corner_d(size: str) -> float:
    """The nut's diameter across corners: it is modelled as a round of this
    size, so it takes at least the room a hex nut in any angle takes."""
    s, _ = NUTS[size]
    return s / math.cos(math.radians(30))


@dataclass
class PartEntry:
    name: str
    file: str
    bolts_to: str | None
    direction: np.ndarray | None
    path: list | None = None  # [("move", vec) | ("turn", point, axis, deg)], from outside to the part's place


@dataclass
class PivotEntry:
    name: str
    joins: tuple[str, str]
    point: np.ndarray
    axis: np.ndarray
    bearing: str | None = None  # a catalogue ball bearing (ISO 15 size), pressed into ``seat_on``
    seat_on: str | None = None

    def bearing_ring(self):
        """The bearing as a solid ring (bore to outside, its width), placed: its entry face's centre at the
        pivot's point, pressed in along the axis."""
        from ..catalogue import BEARINGS

        d, D, B = BEARINGS[self.bearing]
        pl = bd.Plane(origin=tuple(self.point), z_dir=tuple(self.axis))
        return pl * bd.Pos(0, 0, B / 2) * (bd.Cylinder(D / 2, B) - bd.Cylinder(d / 2, B + 1))


@dataclass
class Fastener:
    kind: str
    size: str
    length: float
    head_at: np.ndarray
    axis: np.ndarray
    grip: float
    held_by: str
    label: str = ""

    def plane(self) -> bd.Plane:
        return bd.Plane(origin=tuple(self.head_at), z_dir=tuple(self.axis))

    def screw(self) -> bd.Part:
        """Head and shank as one solid (threads ignored: the shank is the nominal diameter)."""
        b = BOLTS[self.size]
        pl = self.plane()
        head = pl * bd.Pos(0, 0, -b.head_h / 2) * bd.Cylinder(b.head_d / 2, b.head_h)
        shank = pl * bd.Pos(0, 0, self.length / 2) * bd.Cylinder(b.d / 2, self.length)
        return head + shank

    def nut(self) -> bd.Part:
        b = BOLTS[self.size]
        _, m = NUTS[self.size]
        pl = self.plane()
        ring = bd.Cylinder(nut_corner_d(self.size) / 2, m) - bd.Cylinder(b.d / 2, m + 1)
        return pl * bd.Pos(0, 0, self.grip + m / 2) * ring

    def bodies(self) -> list:
        return [self.screw(), self.nut()]

    def to_json(self) -> dict:
        return {"kind": self.kind, "size": self.size, "length": self.length,
                "head_at": [float(v) for v in self.head_at], "axis": [float(v) for v in self.axis],
                "grip": self.grip, "held_by": self.held_by}


@dataclass
class Assembly:
    parts: list[PartEntry]
    pivots: list[PivotEntry]
    drive: str
    range_deg: float
    fasteners: list[Fastener]

    def part(self, name: str) -> PartEntry:
        return next(p for p in self.parts if p.name == name)

    def body_of(self, part_name: str) -> str:
        """The body a part moves with: "A", "B", or the part itself when it bolts to neither."""
        p = self.part(part_name)
        return p.bolts_to or p.name

    def bodies(self) -> list[str]:
        out = ["A", "B"]
        out += [p.name for p in self.parts if p.bolts_to is None]
        return out

    def to_json(self) -> dict:
        return {"parts": [{"name": p.name, "file": p.file, "bolts_to": p.bolts_to,
                           **({"insert_direction": [float(v) for v in p.direction]} if p.direction is not None else
                              {"path": [{"move": [float(x) for x in s_[1]]} if s_[0] == "move" else
                                        {"turn": {"point": [float(x) for x in s_[1]], "axis": [float(x) for x in s_[2]],
                                                  "deg": s_[3]}} for s_ in p.path]})} for p in self.parts],
                "pivots": [{"name": v.name, "joins": list(v.joins), "point": [float(x) for x in v.point],
                            "axis": [float(x) for x in v.axis]} for v in self.pivots],
                "drive": {"pivot": self.drive, "range_deg": self.range_deg},
                "fasteners": [f.to_json() for f in self.fasteners]}


def _vec(v, what) -> np.ndarray:
    try:
        a = np.array([float(x) for x in v])
    except Exception:  # noqa: BLE001
        raise AssemblyError(f"{what} must be three numbers") from None
    if a.shape != (3,):
        raise AssemblyError(f"{what} must be three numbers")
    n = float(np.linalg.norm(a))
    if n < 1e-9:
        raise AssemblyError(f"{what} has length zero")
    return a / n


def _point(v, what) -> np.ndarray:
    try:
        a = np.array([float(x) for x in v])
    except Exception:  # noqa: BLE001
        raise AssemblyError(f"{what} must be three numbers") from None
    if a.shape != (3,):
        raise AssemblyError(f"{what} must be three numbers")
    return a


def _path(steps, name) -> list:
    """A part's way in: straight moves and turns, in order, from outside the box to its place."""
    if not isinstance(steps, list) or not steps or len(steps) > 20:
        raise AssemblyError(f"{name}'s path must list 1 to 20 steps")
    out = []
    for k, st in enumerate(steps):
        what = f"{name}'s path step {k + 1}"
        if not isinstance(st, dict) or len(st) != 1 or not ({"move", "turn"} & set(st)):
            raise AssemblyError(f"{what} must be {{\"move\": [dx, dy, dz]}} or {{\"turn\": {{\"point\", \"axis\", \"deg\"}}}}")
        if "move" in st:
            v = _point(st["move"], what)
            if float(np.linalg.norm(v)) < 1e-9:
                raise AssemblyError(f"{what}: a move of length zero")
            out.append(("move", v))
        else:
            t = st["turn"]
            if not isinstance(t, dict) or not {"point", "axis", "deg"} <= set(t):
                raise AssemblyError(f"{what}: a turn needs point, axis and deg")
            try:
                deg = float(t["deg"])
            except Exception:  # noqa: BLE001
                raise AssemblyError(f"{what}: deg must be a number") from None
            if not 0 < abs(deg) <= 180:
                raise AssemblyError(f"{what}: a turn of more than 0 and at most 180 deg")
            out.append(("turn", _point(t["point"], what), _vec(t["axis"], what), deg))
    return out


def parse(d: dict, fixed: str, moving: str) -> Assembly:
    """Check an assembly file's content; raises AssemblyError with the reason."""
    if not isinstance(d, dict):
        raise AssemblyError("the assembly file must hold one JSON object")
    ps = d.get("parts")
    if not isinstance(ps, list) or not 2 <= len(ps) <= MAX_PARTS:
        raise AssemblyError(f"the assembly file must list 2 to {MAX_PARTS} parts")
    parts = []
    for i, p in enumerate(ps):
        if not isinstance(p, dict):
            raise AssemblyError(f"part {i + 1} is not an object")
        for key in ("name", "file", "bolts_to"):
            if key not in p:
                raise AssemblyError(f"part {i + 1} has no {key!r}")
        if ("insert_direction" in p) == ("path" in p):
            raise AssemblyError(f"part {i + 1}: give either an insert_direction (one straight line) or a path")
        if p["bolts_to"] not in (fixed, moving, None):
            raise AssemblyError(f"part {p['name']} bolts to {p['bolts_to']!r}; it must be {fixed}, {moving} or null")
        if str(p["name"]) in (fixed, moving):
            raise AssemblyError(f"a part may not be named {p['name']!r}, like a fixture")
        parts.append(PartEntry(str(p["name"]), str(p["file"]), p["bolts_to"],
                               _vec(p["insert_direction"], f"{p['name']}'s insert_direction") if "insert_direction" in p
                               else None, _path(p["path"], p["name"]) if "path" in p else None))
    names = [p.name for p in parts]
    if len(set(names)) != len(names):
        raise AssemblyError("every part needs its own name")
    if len(set(p.file for p in parts)) != len(parts):
        raise AssemblyError("every part needs its own file")
    for side in (fixed, moving):
        if not any(p.bolts_to == side for p in parts):
            raise AssemblyError(f"no part bolts to {side}: at least one part must bolt to each fixture")
    body = {p.name: (p.bolts_to or p.name) for p in parts}
    pvs = d.get("pivots")
    if not isinstance(pvs, list) or not pvs:
        raise AssemblyError("the assembly file must list the pivots")
    pivots = []
    for i, v in enumerate(pvs):
        if not isinstance(v, dict):
            raise AssemblyError(f"pivot {i + 1} is not an object")
        for key in ("name", "joins", "point", "axis"):
            if key not in v:
                raise AssemblyError(f"pivot {i + 1} has no {key!r}")
        j = v["joins"]
        if not isinstance(j, list) or len(j) != 2 or any(x not in body for x in j):
            raise AssemblyError(f"pivot {v['name']}: joins must name two parts of the answer")
        if body[j[0]] == body[j[1]]:
            raise AssemblyError(f"pivot {v['name']} joins {j[0]} and {j[1]}, which already move as one body")
        brg, seat = None, None
        if v.get("bearing") is not None:
            from ..catalogue import BEARINGS

            b_ = v["bearing"]
            if not isinstance(b_, dict) or b_.get("size") not in BEARINGS or b_.get("seat_on") not in j:
                raise AssemblyError(f"pivot {v['name']}: bearing must be {{\"size\": one of {', '.join(BEARINGS)}, "
                                    f"\"seat_on\": one of the two parts it joins}}")
            brg, seat = b_["size"], b_["seat_on"]
        pivots.append(PivotEntry(str(v["name"]), (j[0], j[1]), _point(v["point"], f"pivot {v['name']}'s point"),
                                 _vec(v["axis"], f"pivot {v['name']}'s axis"), brg, seat))
    if len(set(v.name for v in pivots)) != len(pivots):
        raise AssemblyError("every pivot needs its own name")
    dr = d.get("drive")
    if not isinstance(dr, dict) or "pivot" not in dr or "range_deg" not in dr:
        raise AssemblyError("the assembly file must state the drive: {\"pivot\": name, \"range_deg\": angle}")
    if dr["pivot"] not in [v.name for v in pivots]:
        raise AssemblyError(f"the drive names pivot {dr['pivot']!r}, which is not declared")
    try:
        rng = float(dr["range_deg"])
    except Exception:  # noqa: BLE001
        raise AssemblyError("the drive's range_deg must be a number") from None
    if not 0 < abs(rng) <= 360:
        raise AssemblyError("the drive's range_deg must be more than 0 and at most 360 either way")
    fs = d.get("fasteners", [])
    if not isinstance(fs, list):
        raise AssemblyError("'fasteners' must be a list")
    fasteners = []
    for i, f in enumerate(fs):
        what = f"fastener {i + 1}"
        if not isinstance(f, dict):
            raise AssemblyError(f"{what} is not an object")
        if f.get("kind") != "bolt":
            raise AssemblyError(f"{what}: kind must be \"bolt\" (a catalogue screw with a nut)")
        for key in ("size", "length", "head_at", "axis", "grip", "held_by"):
            if key not in f:
                raise AssemblyError(f"{what} has no {key!r}")
        if f["size"] not in BOLTS or f["size"] not in NUTS:
            raise AssemblyError(f"{what}: size {f['size']!r} is not in the catalogue ({', '.join(BOLTS)})")
        try:
            length, grip = float(f["length"]), float(f["grip"])
        except Exception:  # noqa: BLE001
            raise AssemblyError(f"{what}: length and grip must be numbers") from None
        if length not in LENGTHS:
            raise AssemblyError(f"{what}: length {length:g} mm is not a catalogue length "
                                f"({', '.join(str(v) for v in LENGTHS)})")
        if grip <= 0:
            raise AssemblyError(f"{what}: grip must be more than 0")
        if f["held_by"] not in body:
            raise AssemblyError(f"{what}: held_by must name a part of the answer")
        fasteners.append(Fastener("bolt", f["size"], length, _point(f["head_at"], f"{what}'s head_at"),
                                  _vec(f["axis"], f"{what}'s axis"), grip, f["held_by"],
                                  f"{f['size']} x {length:g} bolt {i + 1}"))
    return Assembly(parts, pivots, str(dr["pivot"]), rng, fasteners)


def load(path: str, fixed: str, moving: str) -> Assembly:
    if not os.path.exists(path):
        raise AssemblyError(f"no assembly file {os.path.basename(path)}")
    try:
        with open(path, encoding="utf-8") as fh:
            d = json.load(fh)
    except Exception as e:  # noqa: BLE001
        raise AssemblyError(f"the assembly file is not valid JSON: {e}") from None
    return parse(d, fixed, moving)


def describe(v) -> str:
    v = np.asarray(v, float)
    return "(" + ", ".join(f"{0.0 if abs(x) < 5e-4 else x:.3f}" for x in v) + ")"
