"""The task format: what a task is, independent of how it was generated.

``task.json`` is the single source of truth. The fixed parts are stored as
catalogue entries (interface kind + parameters) in their local frame, and
every *pose* gives a world placement for each fixed part. The STEP files in a
workspace are a convenience view of pose 0; the grader rebuilds the fixed
parts from ``task.json`` so it never depends on how well a STEP file
round-trips.

Level 1 has exactly one pose. Motion later means more entries in ``poses``;
the format and the grader loop already handle a list. A pose may place only
some of the fixed parts (the adapt family: each pose is one arrangement of
its own fixtures); a part a pose does not place is absent from it.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field

import build123d as bd

from .catalogue import BOLTS, Interface, fixed_part_solid, make_interface
from .geometry import Placement


def _x(v: float) -> float:
    """Full precision, so anyone building from these numbers lands exactly on
    the geometry the grader uses (rounding here showed up as false overlaps
    of ~0.02 mm3 on tilted faces). Only -0.0 is tidied to 0.0."""
    v = float(v)
    return 0.0 if v == 0 else v

FORMAT_VERSION = 1  # task.json carries a version number, never the project name

DEFAULT_TOLERANCES = {
    # a hole's axis may sit this far from the bolt axis (mm, radial)
    "hole_axis_offset_max": 0.1,
    # a seat face may stand off the mounting face by up to this much (mm)
    "seat_gap_max": 0.05,
    # share of the bearing ring around each bolt that must touch the face
    "seat_contact_min": 0.75,
    # largest overlap volume treated as zero (mm^3) -- numerical noise only
    "overlap_volume_max": 0.01,
    # free space kept around a bolt head, radial and above it (mm)
    "head_clearance_radial": 0.5,
    "head_clearance_axial": 1.0,
    # SECTION_MIN: required area = this x the combined bolt shank area of
    # the weaker of two mounts
    "section_factor": 2.0,
    # how the part is to be made; its rules are reported (or required, if
    # listed in "required_checks")
    "manufacturing": {"process": "fdm", "quantity": 1, "direction": None},
}


@dataclass
class ResolvedBolt:
    id: str
    part: str
    size: str
    position: tuple[float, float, float]  # on the mounting face
    axis: tuple[float, float, float]  # unit, out of the fixed part, into the connector

    def to_json(self) -> dict:
        b = BOLTS[self.size]
        return {
            "id": self.id,
            "size": self.size,
            "position": [_x(v) for v in self.position],
            "axis": [_x(v) for v in self.axis],
            "clearance_diameter_range": [b.clear_fine, b.clear_coarse],
            "grip_range": [b.grip_min, b.grip_max],
            "head_diameter": b.head_d,
            "head_height": b.head_h,
        }


@dataclass
class Obstacle:
    """A fixed keep-out body the answer must not touch (level 3 on). A box of
    ``size`` centred on its placement's origin, along the placement's axes;
    or (``kind`` "cylinder") a round bar along the placement's z axis, of
    diameter ``size[0]`` and length ``size[2]``. A box may have round holes
    through it (``holes``: [{"center", "axis", "radius", "length"}], world
    coordinates): a wall a pipe runs through. Files without ``kind`` or
    ``holes`` read as plain boxes, as before."""

    name: str
    size: tuple[float, float, float]
    placement: Placement
    kind: str = "box"
    holes: tuple = ()

    def solid(self) -> bd.Part:
        if self.kind == "cylinder":
            return bd.Cylinder(self.size[0] / 2, self.size[2]).moved(self.placement.location())
        body = bd.Box(*self.size).moved(self.placement.location())
        for h in self.holes:
            body = body - bd.Plane(origin=tuple(h["center"]), z_dir=tuple(h["axis"])) * bd.Cylinder(h["radius"], h["length"])
        return body

    def to_json(self) -> dict:
        r = self.placement.rot
        out = {
            "name": self.name,
            "kind": self.kind,
            "size": list(self.size),
            "center": [_x(v) for v in self.placement.origin],
            "axes": [[_x(r[i][j]) for i in range(3)] for j in range(3)],  # box x, y, z directions
        }
        if self.holes:
            out["holes"] = [dict(h) for h in self.holes]
        return out

    @staticmethod
    def from_json(d: dict) -> "Obstacle":
        ax = d["axes"]
        rot = tuple(tuple(ax[j][i] for j in range(3)) for i in range(3))
        return Obstacle(d["name"], tuple(d["size"]), Placement(tuple(d["center"]), rot), d.get("kind", "box"),
                        tuple(d.get("holes", ())))


@dataclass
class MountingFace:
    part: str
    origin: tuple[float, float, float]
    normal: tuple[float, float, float]


@dataclass
class Task:
    id: str
    level: int
    seed: int
    parts: dict[str, Interface]
    poses: list[dict[str, Placement]]
    tolerances: dict = field(default_factory=lambda: dict(DEFAULT_TOLERANCES))
    meta: dict = field(default_factory=dict)
    # per pose, the keep-out bodies (empty below level 3; omitted from the
    # file when empty, so level 1 and 2 task files are unchanged)
    obstacles: list = field(default_factory=list)
    # which generator made the task: the same seed gives a different task
    # under a different generator version. Files without it are version 1.
    generator_version: int = 1
    # the volume of a deliberately lean answer, for the material score (a
    # par, not an optimum); None for tasks generated before it existed
    par: dict | None = None
    # which task family the task belongs to (see jointbench.families);
    # written to the file only when it is not the default, "connect"
    family: str = "connect"
    # stiffness requirement, when the task has one: {"min": N/mm, "fraction",
    # "reference": the reference answer's figure, "precision": percent}
    stiffness: dict | None = None
    # regions where overlap with a fixed part is the point (an interference
    # fit): [{"part": "A", "kind": "cylinder", "center": [...], "axis": [...],
    # "radius": r, "length": L} or {"kind": "box", "center", "axes", "size"}]
    intended_overlaps: list = field(default_factory=list)
    # the folder the task was loaded from (not stored in the file)
    root: str | None = field(default=None, repr=False, compare=False)

    def overlap_regions(self, part: str):
        """The intended-overlap regions on fixed part ``part``, as solids."""
        out = []
        for r in self.intended_overlaps:
            if r.get("part") != part:
                continue
            if r["kind"] == "cylinder":
                pl = bd.Plane(origin=tuple(r["center"]), z_dir=tuple(r["axis"]))
                out.append(pl * bd.Cylinder(r["radius"], r["length"]))
            elif r["kind"] == "box":
                ax = r["axes"]
                rot = tuple(tuple(ax[j][i] for j in range(3)) for i in range(3))
                out.append(bd.Box(*r["size"]).moved(Placement(tuple(r["center"]), rot).location()))
            else:
                raise ValueError(f"unknown intended-overlap kind {r['kind']!r}")
        return out

    def obstacles_at(self, pose: int = 0) -> list[Obstacle]:
        return self.obstacles[pose] if pose < len(self.obstacles) else []

    def obstacle_solids(self, pose: int = 0) -> dict[str, bd.Part]:
        return {o.name: o.solid() for o in self.obstacles_at(pose)}

    # ---- derived geometry -------------------------------------------------

    def faces(self, pose: int = 0) -> list[MountingFace]:
        out = []
        for name, itf in self.parts.items():
            if not itf.has_face or name not in self.poses[pose]:
                continue
            pl = self.poses[pose][name]
            out.append(MountingFace(name, pl.point((0, 0, 0)), pl.vec((0, 0, 1))))
        return out

    def bolts(self, pose: int = 0) -> list[ResolvedBolt]:
        out = []
        for name, itf in self.parts.items():
            if name not in self.poses[pose]:
                continue
            pl = self.poses[pose][name]
            if itf.hole_frames:  # each bolt its own point and axis (a pipe)
                for i, (p, a) in enumerate(itf.hole_frames):
                    out.append(ResolvedBolt(f"{name}{i + 1}", name, itf.bolt, pl.point(p), pl.vec(a)))
                continue
            ax = pl.vec((0, 0, 1))
            for i, (x, y) in enumerate(itf.holes):
                out.append(ResolvedBolt(f"{name}{i + 1}", name, itf.bolt, pl.point((x, y, 0)), ax))
        return out

    def fixed_solids(self, pose: int = 0) -> dict[str, bd.Part]:
        return {
            name: fixed_part_solid(itf, self.root).moved(self.poses[pose][name].location())
            for name, itf in self.parts.items() if name in self.poses[pose]
        }

    def requirements(self) -> dict:
        """Which requirement groups decide the pass for this task."""
        req = set(self.tolerances.get("required_checks", []))
        return {"fits": "required",
                "sturdy": "required" if ({"SECTION_MIN", "STIFFNESS"} & req) else "reported",
                "clean": "required" if "CLEAN" in req else "reported",
                "makeable": "required" if "MANUFACTURING" in req else "reported"}

    # ---- io ---------------------------------------------------------------

    def to_json(self) -> dict:
        return {
            "format_version": FORMAT_VERSION,
            **({"family": self.family} if self.family != "connect" else {}),
            "generator_version": self.generator_version,
            "id": self.id,
            "level": self.level,
            "seed": self.seed,
            "units": "mm",
            # what decides the pass, and the difficulty settings: a readable
            # view; the source is tolerances["required_checks"] and meta
            "requirements": self.requirements(),
            "knobs": self.meta.get("knobs", {}),
            "fixed_parts": [
                {"name": n, "interface": itf.to_json(), "step": f"fixed_{n}.step"}
                for n, itf in self.parts.items()
            ],
            "tolerances": self.tolerances,
            "poses": [
                {
                    "name": f"pose{k}",
                    "placements": {n: p[n].to_json() for n in self.parts if n in p},
                    "resolved": {
                        "mounting_faces": [
                            {"part": f.part, "origin": [_x(v) for v in f.origin],
                             "normal": [_x(v) for v in f.normal]}
                            for f in self.faces(k)
                        ],
                        "bolts": [b.to_json() for b in self.bolts(k)],
                    },
                    **({"obstacles": [o.to_json() for o in self.obstacles_at(k)]}
                       if self.obstacles_at(k) else {}),
                }
                for k, p in enumerate(self.poses)
            ],
            **({"par": self.par} if self.par else {}),
            **({"stiffness": self.stiffness} if self.stiffness else {}),
            **({"intended_overlaps": self.intended_overlaps} if self.intended_overlaps else {}),
            "meta": self.meta,
        }

    def dumps(self) -> str:
        return json.dumps(self.to_json(), indent=2, sort_keys=False) + "\n"

    @staticmethod
    def from_json(d: dict) -> "Task":
        if d.get("format_version") != FORMAT_VERSION:
            raise ValueError(f"unknown task format version {d.get('format_version')!r}")
        parts = {
            fp["name"]: make_interface(fp["interface"]["kind"], fp["interface"]["params"])
            for fp in d["fixed_parts"]
        }
        poses = [
            {n: Placement.from_json(pl) for n, pl in p["placements"].items()} for p in d["poses"]
        ]
        obstacles = [[Obstacle.from_json(o) for o in p.get("obstacles", [])] for p in d["poses"]]
        if not any(obstacles):
            obstacles = []
        return Task(d["id"], d["level"], d["seed"], parts, poses, d["tolerances"], d.get("meta", {}),
                    obstacles, d.get("generator_version", 1), d.get("par"), d.get("family", "connect"),
                    d.get("stiffness"), d.get("intended_overlaps", []))

    @staticmethod
    def load(path: str) -> "Task":
        with open(path, encoding="utf-8") as f:
            t = Task.from_json(json.load(f))
        t.root = os.path.dirname(os.path.abspath(path))
        return t
