"""The mounts of a task: where the part is held and where it is loaded.

A mount is a fixed part's mounting face (a point, its two in-face axes and
its normal, and the face's size) and the bolts through it. On the answer,
the mount's *seat* is every bit of surface lying on that face, inside its
outline, plus the walls of the bolt holes. Read from ``task.json``, pose 0
(the pose the answer's STEP file is drawn in).
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Bolt:
    id: str
    position: np.ndarray
    axis: np.ndarray
    radius: float  # largest clearance radius allowed, mm
    grip: float  # longest grip, mm


@dataclass
class Mount:
    name: str
    origin: np.ndarray
    x: np.ndarray
    y: np.ndarray
    n: np.ndarray  # out of the fixed part, into the answer
    size: tuple[float, float]
    bolts: list[Bolt] = field(default_factory=list)
    bolt_size: str = ""  # e.g. "M5"
    n_bolts: int = 0
    # a held bearing instead of a bolted face: "bearing", its outside diameter
    # and width; origin is then the bearing's centre and n its axis (the way it
    # is pressed in), and the seat is the bore round it plus the shoulder
    kind: str = "face"
    axis_point: np.ndarray | None = None  # a pipe: a point on its axis (x is the axis)
    D: float = 0.0
    B: float = 0.0
    d: float = 0.0

    def seat_pred(self, plane_tol: float, hole_tol: float):
        if self.kind == "bearing":
            return self._bearing_pred(plane_tol, hole_tol)
        if self.kind == "pipe":
            return self._pipe_pred(plane_tol, hole_tol)

        def pred(P):
            d = P - self.origin
            s = d @ self.n
            on_face = (np.abs(s) <= plane_tol) & (np.abs(d @ self.x) <= self.size[0] / 2 + plane_tol) \
                & (np.abs(d @ self.y) <= self.size[1] / 2 + plane_tol)
            in_hole = np.zeros(len(P), bool)
            for b in self.bolts:
                q = P - b.position
                a = q @ b.axis
                rad = np.linalg.norm(q - a[:, None] * b.axis, axis=1)
                in_hole |= (rad <= b.radius + hole_tol) & (a >= -plane_tol) & (a <= b.grip)
            return on_face | in_hole
        return pred

    def _pipe_pred(self, plane_tol: float, hole_tol: float):
        """On a pipe: the part's surface lying on the pipe (radius D/2 about
        the pipe's axis, ``x``, through ``axis_point``), plus the bolt holes."""
        def pred(P):
            q = P - self.axis_point
            a = q @ self.x
            rad = np.linalg.norm(q - a[:, None] * self.x, axis=1)
            on = np.abs(rad - self.D / 2) <= plane_tol
            in_hole = np.zeros(len(P), bool)
            for b in self.bolts:
                w = P - b.position
                s = w @ b.axis
                r = np.linalg.norm(w - s[:, None] * b.axis, axis=1)
                in_hole |= (r <= b.radius + hole_tol) & (s >= -plane_tol) & (s <= b.grip)
            return on | in_hole
        return pred

    def _bearing_pred(self, plane_tol: float, hole_tol: float):
        """The bore round the bearing (its outside diameter, plus the fit band
        and the hole tolerance) over its width, and the shoulder ring at its
        far face: where the bearing bears on the part."""
        from ..checks.bearing_seat import FIT_MAX, inner_race_radius

        ri = inner_race_radius(self.d, self.D)

        def pred(P):
            q = P - self.origin
            a = q @ self.n  # 0 at the centre, -B/2 at the entry face, +B/2 at the shoulder
            rad = np.linalg.norm(q - a[:, None] * self.n, axis=1)
            bore = (np.abs(rad - self.D / 2) <= FIT_MAX / 2 + hole_tol) & (np.abs(a) <= self.B / 2 + plane_tol)
            shoulder = (np.abs(a - self.B / 2) <= plane_tol) & (rad >= ri - hole_tol) & (rad <= self.D / 2 + hole_tol)
            return bore | shoulder
        return pred


def from_task(task_dir: str) -> tuple[dict, list[Mount]]:
    with open(os.path.join(task_dir, "task.json"), encoding="utf-8") as fh:
        task = json.load(fh)
    pose = task["poses"][0]
    mounts = []
    bolts = pose["resolved"]["bolts"]
    for fp in task["fixed_parts"]:
        pl = pose["placements"][fp["name"]]
        R = np.array(pl["rotation"], float)
        if fp["interface"]["kind"] == "bearing":
            from ..catalogue import BEARINGS

            d, D, B = BEARINGS[fp["interface"]["params"]["size"]]
            n = R[:, 2]
            mounts.append(Mount(fp["name"], np.array(pl["origin"], float) + n * B / 2, R[:, 0], R[:, 1], n,
                                (0.0, 0.0), kind="bearing", D=D, B=B, d=d))
            continue
        m = Mount(fp["name"], np.array(pl["origin"], float), R[:, 0], R[:, 1], R[:, 2],
                  tuple(fp["interface"]["face_size"]), bolt_size=fp["interface"]["bolt"],
                  n_bolts=len(fp["interface"]["holes_local"]))
        for b in bolts:
            if b["id"].startswith(fp["name"]) and b["id"][len(fp["name"]):].isdigit():
                m.bolts.append(Bolt(b["id"], np.array(b["position"], float), np.array(b["axis"], float),
                                    max(b["clearance_diameter_range"]) / 2, max(b["grip_range"])))
        if fp["interface"]["kind"] == "pipe" and m.bolts:
            # the load acts at the bolts' centre, in their mean direction; x is the pipe's axis
            m.kind, m.D = "pipe", float(fp["interface"]["params"]["od"])
            m.axis_point = np.array(pl["origin"], float)
            m.origin = np.mean([b.position for b in m.bolts], axis=0)
            nn = np.mean([b.axis for b in m.bolts], axis=0)
            m.x = R[:, 0]
            nn = nn - m.x * (nn @ m.x)
            m.n = nn / np.linalg.norm(nn)
            m.y = np.cross(m.n, m.x)
        mounts.append(m)
    return task, mounts
