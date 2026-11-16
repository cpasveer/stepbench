"""Generating a "moves" task (level 4c) backwards from a hidden four-bar linkage.

The situation: two fixtures with flat mounting faces, parallel and facing
each other, as at level 1. Fixture A stays put. Fixture B starts where the task
places it and must be carried, by a mechanism the model designs, through three
more stated placements (poses), in order. No axis is stated, and no single
hinge can do it: the model has to find a linkage -- typically a four-bar,
its pivot places and link lengths -- from the poses alone.

How a task is made (answer first):

1. Draw the fixtures and a hidden four-bar in a vertical plane between them:
   two ground pivots above A, two coupler pivots below B, B riding on the
   coupler. The plane's direction across A, the pivot places, and the drive
   range (60-130 deg either way) are drawn from the seed.
2. Drive the crank through its range in closed form (circle meets circle,
   on the branch it is assembled in) and record B's placement at 25-40 % and
   55-75 % of the range and at the end: those are the required poses. With
   the start that is four placements of B, which leave only a handful of
   four-bars that pass through them exactly (three would leave infinitely many).
3. Reject the draw if: the linkage nears a dead point (transmission angle
   outside 35-145 deg) anywhere in the range; one fixed hinge could come
   within ``HINGE_MISS_MIN`` of every pose; consecutive poses lie too close
   (B's centre less than ``POSE_MOVE_MIN`` apart, or turned less than
   ``POSE_TURN_MIN``); the fixtures come within ``FIXTURE_CLEAR`` of each
   other; a reference part does not fit the build volume with room; or the
   reference fails the real grader.

The hidden reference, built from those numbers:

  frame    a plate on A with a clevis (two ears) at each ground pivot
  carrier  a plate on B with a clevis at each coupler pivot
  crank, rocker  flat links, one knuckle at each end, each in its own layer
  4 axle screws (catalogue M4/M5) with nuts, clamped in the clevises

Running gaps 0.4 mm (axially and round each screw): inside the 0.3 mm
printing gap and the 0.6 mm play limit with room both ways.
"""
from __future__ import annotations

import json
import math
import os
import random
import shutil

import build123d as bd
import numpy as np

from ..catalogue import BOLTS, GENERATOR_VERSION, make_interface, sample_interface
from ..geometry import BUILD_ROOM, Placement, fits_build_volume, write_step
from ..task import DEFAULT_TOLERANCES, Task
from . import assembly as AS
from . import linkage as LK
from . import motion as MO

LEVEL = 4  # level 4c: the third of the level 4s (4a assembles, 4b adapts); builds on levels 1 and 2
LEVEL_NAME = "4c"
NAME = "moves"
SITUATION = (
    "Two fixtures are given. Their mounting faces are parallel and face each other. Fixture A\n"
    "stays where it is. Fixture B starts where it is placed and must then be carried, by one\n"
    "continuous motion of a mechanism you design, through three more stated placements, in order.\n"
    "Nothing loads it."
)
KNOBS_DEFAULT = {"targets": 3}

SETBACK = 2.0  # mm: reference plates stay this far inside their fixture's face
REF_GAP = 0.4  # mm: the reference's running gap, axially and round each screw
LINK_T = 8.0  # mm: the thickness of a link knuckle, along the pivot axis
FIXTURE_CLEAR = 3.0  # mm
HINGE_MISS_MIN = 10.0  # mm: the best single fixed hinge misses a pose by at least this
POSE_MOVE_MIN = 15.0  # mm
POSE_TURN_MIN = 8.0  # deg
TRANSMISSION = (35.0, 145.0)  # deg
MOVES_TOLERANCES = {
    "gap_min": 0.3,  # mm, the printing gap between bodies that move relative to each other, at every step
    "play_max": 0.6,  # mm, free shift at each pivot, along its axis and square to it
    "tilt_max_deg": 5.0,  # deg, free tilt at each pivot
    "loose_shift": 5.0,  # mm: a shift this far without touching = the pivot comes apart
    "loose_tilt": 15.0,  # deg
    "step_max_deg": 2.0,  # deg of drive, the largest step
    "step_move_max": 1.5,  # mm, no point of any body moves more than this per step
    "pose_position": 0.5,  # mm, B's face centre within this of each required pose
    "pose_angle_deg": 0.5,  # deg, B's orientation within this of each required pose
    "pivot_offset_max": 0.05,  # mm, a pin's and a bore's axis within this of the declared pivot axis
    "pivot_angle_max_deg": 0.1,
}
REQUIRED = ["SEAT_INSIDE", "CLEAN", "MANUFACTURING", "KEY_TURN"]
TOOL_LENGTH = 30.0  # mm, the straight tool that must reach every bolt into a fixture (as at level 3)


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _local(pl: Placement, p) -> np.ndarray:
    r = np.array(pl.rot)
    return r.T @ (np.asarray(p, float) - np.asarray(pl.origin, float))


def _inside_face(itf, pl: Placement, pts, margin: float) -> bool:
    for p in pts:
        x, y, _ = _local(pl, p)
        if itf.kind == "round":
            if math.hypot(x, y) > itf.face_w / 2 - margin:
                return False
        elif abs(x) > itf.face_w / 2 - margin or abs(y) > itf.face_h / 2 - margin:
            return False
    return True


def _rect_clear(task, part, pts, extra: float) -> bool:
    """The rectangle (4 world corners on the face) keeps clear of every bolt head of ``part``."""
    c = np.mean(pts, axis=0)
    e1, e2 = np.asarray(pts[1]) - pts[0], np.asarray(pts[3]) - pts[0]
    l1, l2 = np.linalg.norm(e1), np.linalg.norm(e2)
    e1, e2 = e1 / l1, e2 / l2
    for b in task.bolts(0):
        if b.part != part:
            continue
        v = np.asarray(b.position) - c
        dx, dy = max(abs(v @ e1) - l1 / 2, 0.0), max(abs(v @ e2) - l2 / 2, 0.0)
        if math.hypot(dx, dy) < BOLTS[b.size].head_d / 2 + extra:
            return False
    return True


def _no_spigot(itf):
    p = dict(itf.params)
    p.pop("spigot", None)  # parts slide in along the faces: nothing may stand proud of them
    return make_interface(itf.kind, p)


def _seg_dist(p, a, b) -> float:
    p, a, b = (np.asarray(x, float) for x in (p, a, b))
    t = float(np.clip((p - a) @ (b - a) / ((b - a) @ (b - a)), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * (b - a))))


def relative(start: Placement, target: Placement) -> Placement:
    """The rigid motion that takes a body from ``start`` to ``target``."""
    R = np.array(target.rot) @ np.array(start.rot).T
    t = np.asarray(target.origin) - R @ np.asarray(start.origin)
    return Placement(tuple(float(v) for v in t), tuple(tuple(float(v) for v in r) for r in R))


# --------------------------------------------------------------------------
# the hidden mechanism, in its plane: s across (world u), t up (world z)


class Frame:
    def __init__(self, d: dict):
        self.u = np.array([math.cos(math.radians(d["psi"])), math.sin(math.radians(d["psi"])), 0.0])
        self.z = np.array([0.0, 0.0, 1.0])
        self.a = np.cross(self.z, self.u)  # along the pivot axes
        self.n = np.cross(self.u, self.z)  # a positive turn in (s, t) is right-handed about n

    def world(self, st, w: float = 0.0) -> np.ndarray:
        return st[0] * self.u + st[1] * self.z + w * self.a

    def plane(self, p) -> np.ndarray:
        p = np.asarray(p, float)
        return np.array([float(p @ self.u), float(p @ self.z)])


def start_b(d: dict) -> Placement:
    return Placement.from_yaw_flip((d["B"]["center"][0], d["B"]["center"][1], d["gap"]), d["B"]["yaw"], True)


def coupler_at(d: dict, deg: float):
    """(C2, C4) with the crank turned ``deg`` from its start, or None if it cannot close."""
    m = d["mech"]
    return LK.fourbar(m["O2"], m["O4"], m["r2"], m["r3"], m["r4"], m["theta0"] + math.radians(deg), m["mode"])


def b_placement(d: dict, C2, C4) -> Placement:
    """Fixture B's placement with the coupler at (C2, C4)."""
    m, F = d["mech"], Frame(d)
    c20, c40 = np.asarray(m["C2"]), np.asarray(m["C4"])
    phi0 = math.atan2((c40 - c20)[1], (c40 - c20)[0])
    v = np.asarray(C4) - np.asarray(C2)
    phi = math.atan2(v[1], v[0])
    turn = MO.about(tuple(F.world(c20)), tuple(F.n), math.degrees(phi - phi0))
    move = turn.then(MO.shift(F.world(np.asarray(C2) - c20)))
    return start_b(d).then(move)


def transmission(d: dict, C2, C4) -> float:
    m = d["mech"]
    v1, v2 = np.asarray(C4) - np.asarray(C2), np.asarray(C4) - np.asarray(m["O4"])
    c = float(v1 @ v2 / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def hinge_miss(poses: list[Placement], F: Frame) -> float:
    """The least, over every fixed hinge square to the plane, of the largest distance by which B's
    centre misses a pose. A hinge turns B by each pose's own turn, so for each pose alone the only
    centre that hits it is that pose's pole; the best compromise lies between the poles."""
    b = [F.plane(x.origin) for x in poses]
    r0 = np.array(poses[0].rot)
    poles, w = [], []
    for k in range(1, len(poses)):
        rel = np.array(poses[k].rot) @ r0.T
        dth = math.atan2(float(F.z @ rel @ F.u), float(F.u @ rel @ F.u))
        if abs(dth) < 1e-9:
            return math.inf  # a pure shift: no hinge does it
        chord = b[k] - b[0]
        half = float(np.linalg.norm(chord)) / 2
        perp = np.array([-chord[1], chord[0]]) / (2 * half)
        poles.append((b[0] + b[k]) / 2 + perp * half / math.tan(dth / 2))
        w.append(2 * abs(math.sin(dth / 2)))
    if len(poles) == 1:
        return 0.0
    if len(poles) == 2:
        D = float(np.linalg.norm(poles[0] - poles[1]))
        return w[0] * w[1] * D / (w[0] + w[1])
    # more poses: the least of the largest weighted distance to the poles. That is a convex function of the
    # centre, so a grid that shrinks round its best point finds it (to well under 0.01 mm here)
    P = np.array(poles)
    c = P.mean(axis=0)
    half = float(np.max(np.abs(P - c))) + 1.0
    f = lambda x: max(wk * float(np.linalg.norm(x - pk)) for wk, pk in zip(w, P))  # noqa: E731
    g = np.linspace(-1.0, 1.0, 41)
    for _ in range(20):
        c = min((c + half * np.array([gx, gy]) for gx in g for gy in g), key=f)
        half /= 4.0
    return f(c)


def draw(rng: random.Random) -> dict | None:
    """One candidate draw, or None if its numbers do not fit together."""
    A, B = _no_spigot(sample_interface(rng)), _no_spigot(sample_interface(rng))
    yaw_a, yaw_b = 5.0 * rng.randrange(72), 5.0 * rng.randrange(72)
    axle = rng.choice(["M4", "M5"])
    ab = BOLTS[axle]
    Rk = max(ab.head_d, AS.nut_corner_d(axle)) / 2 + 2.5
    le = max(4.0, ab.grip_min)
    W = LINK_T + 2 * REF_GAP + 2 * le  # one pivot stack along the axis
    d = {"psi": 5.0 * rng.randrange(72)}
    F = Frame(d)
    t_a, t_b = max(5.0, BOLTS[A.bolt].grip_min), max(5.0, BOLTS[B.bolt].grip_min)
    head_a, head_b = BOLTS[A.bolt].head_h, BOLTS[B.bolt].head_h
    pa = Placement.from_yaw_flip((0.0, 0.0, 0.0), yaw_a, False)
    w0 = rng.uniform(-6.0, 6.0)
    sep = W + 2.0
    wc, wr = (w0 - sep / 2, w0 + sep / 2) if rng.random() < 0.5 else (w0 + sep / 2, w0 - sep / 2)
    ext = min(A.face_w, A.face_h) / 2
    o_lo = t_a + head_a + Rk + 2.0  # ground pivots: low, over A
    O2 = np.array([rng.uniform(-ext, ext), o_lo + rng.uniform(0.0, 10.0)])
    O4 = np.array([rng.uniform(-ext, ext), o_lo + rng.uniform(0.0, 10.0)])
    if abs(O2[0] - O4[0]) < 2 * Rk + 6:
        return None
    c_lo = max(O2[1], O4[1]) + 2 * Rk + 3.0  # coupler pivots: higher, under B
    C2 = np.array([rng.uniform(-ext - 10, ext + 10), c_lo + rng.uniform(0.0, 25.0)])
    C4 = np.array([rng.uniform(-ext - 10, ext + 10), c_lo + rng.uniform(0.0, 25.0)])
    if np.linalg.norm(C2 - C4) < 2 * Rk + 10:
        return None
    gap = round(max(C2[1], C4[1]) + Rk + head_b + 2.0 + t_b + rng.uniform(0.0, 6.0), 1)
    centre = ((C2[0] + C4[0]) / 2 + rng.uniform(-8.0, 8.0)) * F.u + w0 * F.a
    d.update({"A": {"kind": A.kind, "params": A.params, "yaw": yaw_a},
              "B": {"kind": B.kind, "params": B.params, "yaw": yaw_b,
                    "center": [round(float(centre[0]), 1), round(float(centre[1]), 1)]},
              "gap": gap, "axle": axle, "Rk": Rk, "le": le, "t": {"A": t_a, "B": t_b}, "floor": t_a + head_a,
              "w": {"crank": wc, "rocker": wr, "mid": w0}})
    pb = start_b(d)

    def foot(P, wk, z):  # a clevis footprint on a face: the stack along the axis, the knuckle's width across
        return [F.world((P[0] + sx * Rk, 0), wk + sy * W / 2) + z * F.z
                for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]

    for P, wk in ((O2, wc), (O4, wr)):
        if not _inside_face(A, pa, foot(P, wk, 0.0), SETBACK + 1.0):
            return None
    for P, wk in ((C2, wc), (C4, wr)):
        if not _inside_face(B, pb, foot(P, wk, gap), SETBACK + 1.0):
            return None
    # each screw's line must miss the other stack's link (screws go in along their axis)
    nut_r = AS.nut_corner_d(axle) / 2
    for P, (a, b) in ((O2, (O4, C4)), (C2, (O4, C4)), (O4, (O2, C2)), (C4, (O2, C2))):
        if _seg_dist(P, a, b) < Rk + nut_r + 2.0:
            return None
    r2, r3, r4 = (float(np.linalg.norm(x)) for x in (C2 - O2, C4 - C2, C4 - O4))
    th0 = math.atan2((C2 - O2)[1], (C2 - O2)[0])
    got = LK.fourbar(O2, O4, r2, r3, r4, th0, 1)
    mode = 1 if got is not None and np.linalg.norm(got[1] - C4) < 1e-6 else -1
    d["mech"] = {"O2": O2.tolist(), "O4": O4.tolist(), "C2": C2.tolist(), "C4": C4.tolist(),
                 "r2": r2, "r3": r3, "r4": r4, "theta0": th0, "mode": mode}
    R = rng.choice([-1, 1]) * float(rng.randint(60, 130))
    f1, f2 = rng.uniform(0.25, 0.4), rng.uniform(0.55, 0.75)
    d["drive"] = {"range_deg": R, "target_at": [round(f1 * R, 2), round(f2 * R, 2), R]}
    return d


def sweep_hidden(d: dict, step: float = 0.5):
    """[(drive deg, C2, C4)] over the range, or None if it cannot close or nears a dead point."""
    m = d["mech"]
    R = d["drive"]["range_deg"]
    n = math.ceil(abs(R) / step)
    out = []
    for i in range(n + 1):
        deg = R * i / n
        got = coupler_at(d, deg)
        if got is None:
            return None
        C2, C4 = got
        if not TRANSMISSION[0] <= transmission(d, C2, C4) <= TRANSMISSION[1]:
            return None
        # the links keep above the frame's bolt heads, and each coupler knuckle clear of its ground knuckle
        if min(C2[1], C4[1]) - d["Rk"] < d["floor"] + 1.0:
            return None
        if min(np.linalg.norm(C2 - np.asarray(m["O2"])), np.linalg.norm(C4 - np.asarray(m["O4"]))) < 2 * d["Rk"] + 2.0:
            return None
        out.append((deg, C2, C4))
    return out


def targets(d: dict) -> list[Placement]:
    return [b_placement(d, *coupler_at(d, deg)) for deg in d["drive"]["target_at"]]


def task_from_draw(seed: int, d: dict) -> Task:
    parts = {n: make_interface(d[n]["kind"], d[n]["params"]) for n in ("A", "B")}
    pose = {"A": Placement.from_yaw_flip((0.0, 0.0, 0.0), d["A"]["yaw"], False), "B": start_b(d)}
    tol = dict(DEFAULT_TOLERANCES, required_checks=list(REQUIRED),
               manufacturing={"process": "fdm", "quantity": 1, "direction": None},
               moves=dict(MOVES_TOLERANCES), tool_length=TOOL_LENGTH, tool_diameter_extra=0.0)
    tg = [{"name": f"pose{k + 1}", "placement": {"origin": [float(v) for v in p.origin],
                                                 "rotation": [[float(v) for v in r] for r in p.rot]}}
          for k, p in enumerate(targets(d))]
    meta = {"generator": NAME, "level_name": LEVEL_NAME, "knobs": dict(KNOBS_DEFAULT),
            "motion": {"fixed": "A", "moving": "B", "targets": tg}}
    return Task(f"{NAME}-s{seed}", LEVEL, seed, parts, [pose], tol, meta, [], GENERATOR_VERSION, family=NAME)


def target_placements(task: Task) -> list[Placement]:
    return [Placement.from_json(t["placement"]) for t in task.meta["motion"]["targets"]]


def target_solids(task: Task) -> list:
    """Fixture B's solid at each required pose."""
    b0 = task.poses[0]["B"]
    fb = task.fixed_solids(0)["B"]
    return [fb.moved(relative(b0, p).location()) for p in target_placements(task)]


# --------------------------------------------------------------------------
# the reference, and deliberately broken copies of it


def _uprofile(plane: bd.Plane, R: float, y_end: float, z0: float, z1: float) -> bd.Part:
    """In ``plane``: a slab between local z0 and z1 whose section is a rectangle 2R wide from local
    y = 0 to y_end, with a round end of radius R about the local z axis."""
    sk = bd.Pos(0, y_end / 2) * bd.Rectangle(2 * R, abs(y_end)) + bd.Circle(R)
    return plane * bd.Pos(0, 0, z0) * bd.extrude(sk, z1 - z0)


def _plate(itf, pl: Placement, t: float, bolt_holes: list, hole_d: float) -> bd.Part:
    if itf.kind == "round":
        sk = bd.Circle(itf.face_w / 2 - SETBACK)
    else:
        sk = bd.Rectangle(itf.face_w - 2 * SETBACK, itf.face_h - 2 * SETBACK)
    body = bd.extrude(sk, t)
    for x, y in bolt_holes:
        body = body - bd.Pos(x, y, t / 2) * bd.Cylinder(hole_d / 2, t + 2)
    return body.moved(pl.location())


def _bar(F: Frame, P, Q, wk: float, Rk: float) -> bd.Part:
    P, Q = np.asarray(P, float), np.asarray(Q, float)
    L = float(np.linalg.norm(Q - P))
    pl = bd.Plane(origin=tuple(F.world((P + Q) / 2, wk)), x_dir=tuple(_unit(F.world(Q - P))), z_dir=tuple(F.a))
    return pl * bd.extrude(bd.SlotCenterToCenter(L, 2 * Rk), LINK_T / 2, both=True)


def _link(F: Frame, P, Q, wk: float, Rk: float, bore: float, tail=None) -> bd.Part:
    """A flat link in the layer at ``wk``: knuckles of radius Rk at plane points P and Q, bored."""
    body = _bar(F, P, Q, wk, Rk)
    if tail is not None:  # a second arm from P (a broken copy)
        body = body + _bar(F, P, tail, wk, Rk)
    for X in (P, Q):
        body = body - bd.Plane(origin=tuple(F.world(X, wk)), z_dir=tuple(F.a)) * bd.Cylinder(bore / 2, LINK_T + 2)
    return body


def _clevis(F: Frame, P, wk: float, Rk: float, le: float, y_end: float, hole: float) -> bd.Part:
    sec = bd.Plane(origin=tuple(F.world(P)), x_dir=tuple(-F.u), z_dir=tuple(F.a))
    inner = LINK_T / 2 + REF_GAP
    ears = (_uprofile(sec, Rk, y_end, wk + inner, wk + inner + le)
            + _uprofile(sec, Rk, y_end, wk - inner - le, wk - inner))
    return ears - sec * bd.Pos(0, 0, wk) * bd.Cylinder(hole / 2, 2 * (inner + le) + 2)


def build(task: Task, d: dict, *, shift_o4: float = 0.0, tail: bool = False, no_rocker: bool = False,
          drop_bolt: str | None = None) -> tuple[dict, dict]:
    """({part name: solid}, assembly dict) for the reference or a broken copy.

    shift_o4   move the rocker's ground pivot this far across (mm): the linkage still assembles,
               but carries B along another path
    tail       give the crank a second arm that swings into the frame's plate half-way through
    no_rocker  leave the rocker and its pivots out: two free motions
    drop_bolt  leave this pivot's screw out (the pivot has no pin)
    """
    F = Frame(d)
    m = d["mech"]
    Rk, le = d["Rk"], d["le"]
    ab = BOLTS[d["axle"]]
    pa, pb = task.poses[0]["A"], task.poses[0]["B"]
    A, B = task.parts["A"], task.parts["B"]
    t_a, t_b, gap = d["t"]["A"], d["t"]["B"], d["gap"]
    wc, wr, w0 = d["w"]["crank"], d["w"]["rocker"], d["w"]["mid"]
    O2, O4, C2, C4 = (np.asarray(m[k], float) for k in ("O2", "O4", "C2", "C4"))
    O4 = O4 + np.array([shift_o4, 0.0])
    frame = _plate(A, pa, t_a, A.holes, BOLTS[A.bolt].clear_medium)
    carrier = _plate(B, pb, t_b, B.holes, BOLTS[B.bolt].clear_medium)
    hole = ab.clear_medium
    pivots = [("O2", ["frame", "crank"], O2, wc), ("C2", ["crank", "carrier"], C2, wc)]
    if not no_rocker:
        pivots += [("O4", ["frame", "rocker"], O4, wr), ("C4", ["rocker", "carrier"], C4, wr)]
    for name, _, P, wk in pivots:
        if name.startswith("O"):
            frame = frame + _clevis(F, P, wk, Rk, le, -(P[1] - t_a), hole)
        else:
            carrier = carrier + _clevis(F, P, wk, Rk, le, gap - t_b - P[1], hole)
    bore = ab.d + 2 * REF_GAP
    tail_pt = None
    if tail:  # an arm that points straight down half-way through the drive and then reaches the plate
        R = d["drive"]["range_deg"]
        delta = min(10.0, abs(R) / 4)
        L = (O2[1] - t_a) / math.cos(math.radians(delta)) - Rk + 1.0
        beta0 = math.radians(-90.0 - R / 2)
        tail_pt = O2 + L * np.array([math.cos(beta0), math.sin(beta0)])
    solids = {"frame": frame, "carrier": carrier, "crank": _link(F, O2, C2, wc, Rk, bore, tail_pt)}
    if not no_rocker:
        solids["rocker"] = _link(F, O4, C4, wr, Rk, bore)
    grip = LINK_T + 2 * REF_GAP + 2 * le
    length = next(L for L in AS.LENGTHS if L >= grip + AS.NUTS[d["axle"]][1])
    fasteners = []
    for name, _, P, wk in pivots:
        if name == drop_bolt:
            continue
        out = 1.0 if wk > w0 else -1.0  # head on the stack's outer side
        head = F.world(P, wk + out * (LINK_T / 2 + REF_GAP + le))
        fasteners.append({"kind": "bolt", "size": d["axle"], "length": float(length),
                          "head_at": [float(v) for v in head], "axis": [float(v) for v in -out * F.a],
                          "grip": round(grip, 6), "held_by": "frame" if name.startswith("O") else "carrier"})
    ins = [float(v) for v in -F.u]
    parts = [{"name": "frame", "file": "frame.step", "bolts_to": "A", "insert_direction": ins},
             {"name": "carrier", "file": "carrier.step", "bolts_to": "B", "insert_direction": ins},
             {"name": "crank", "file": "crank.step", "bolts_to": None, "insert_direction": ins}]
    if not no_rocker:
        parts.append({"name": "rocker", "file": "rocker.step", "bolts_to": None, "insert_direction": ins})
    asm = {"parts": parts,
           "pivots": [{"name": n, "joins": j, "point": [float(v) for v in F.world(P, wk)],
                       "axis": [float(v) for v in F.n]} for n, j, P, wk in pivots],
           "drive": {"pivot": "O2", "range_deg": d["drive"]["range_deg"]},
           "fasteners": fasteners}
    return solids, asm


def broken_variants(d: dict) -> list[tuple[str, dict, str, str]]:
    """(folder name, build options, the check it must fail, what it shows)."""
    return [
        ("misses-pose", {"shift_o4": 3.0}, "POSES",
         "the rocker's ground pivot sits 3 mm off: the linkage still moves, along another path"),
        ("clash-mid-motion", {"tail": True}, "MOTION",
         "the crank has a second arm that swings down into the frame during the drive"),
        ("two-free-motions", {"no_rocker": True}, "MOBILITY",
         "no rocker: B hangs on the crank alone and can move two ways"),
        ("pivot-without-pin", {"drop_bolt": "C4"}, "PIVOTS",
         "the rocker-to-carrier pivot is declared but has no screw through it"),
    ]


def write_answer(folder: str, solids: dict, asm: dict) -> str:
    os.makedirs(folder, exist_ok=True)
    for p in asm["parts"]:
        write_step(solids[p["name"]], os.path.join(folder, p["file"]))
    path = os.path.join(folder, AS.ASSEMBLY_FILE)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(asm, indent=2) + "\n")
    return path


def _screen(task: Task, d: dict) -> str | None:
    """Cheap reasons to drop a draw before building anything; None if it may go on."""
    sw = sweep_hidden(d)
    if sw is None:
        return "the hidden linkage nears a dead point or cannot close in its range"
    F = Frame(d)
    tg = [task.poses[0]["B"]] + target_placements(task)
    for k in range(1, len(tg)):
        mv = float(np.linalg.norm(np.asarray(tg[k].origin) - np.asarray(tg[k - 1].origin)))
        rel = np.array(tg[k].rot) @ np.array(tg[k - 1].rot).T
        turn = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(rel) - 1) / 2))))
        if mv < POSE_MOVE_MIN or turn < POSE_TURN_MIN:
            return f"poses {k - 1} and {k} lie too close ({mv:.1f} mm, {turn:.1f} deg)"
    miss = hinge_miss(tg, F)
    if miss < HINGE_MISS_MIN:
        return f"a single hinge would come within {miss:.1f} mm of every pose"
    d["hinge_miss_mm"] = round(miss, 2)
    fx = task.fixed_solids(0)
    for deg, C2, C4 in sw[::6] + [sw[-1]]:
        mv = relative(start_b(d), b_placement(d, C2, C4))
        if MO.distance(fx["B"].moved(mv.location()), fx["A"]) < FIXTURE_CLEAR:
            return "the fixtures come too close during the motion"
    W = LINK_T + 2 * REF_GAP + 2 * d["le"]
    Rk, m = d["Rk"], d["mech"]
    for P, wk, part, z in ((m["O2"], d["w"]["crank"], "A", 0.0), (m["O4"], d["w"]["rocker"], "A", 0.0),
                           (m["C2"], d["w"]["crank"], "B", d["gap"]), (m["C4"], d["w"]["rocker"], "B", d["gap"])):
        pts = [F.world((P[0] + sx * Rk, 0), wk + sy * W / 2) + z * F.z
               for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        if not _rect_clear(task, part, pts, 1.5):
            return "a bolt head sits where a clevis meets a plate"
    return None


def generate(seed: int, *, check=True, progress=None, scratch: str | None = None):
    """(task, draw, reference solids, reference assembly) for ``seed``."""
    import tempfile

    from .grade import grade

    say = progress or (lambda s: None)
    rng = random.Random(f"{NAME}-{seed}")
    for attempt in range(100000):
        d = draw(rng)
        if d is None:
            continue
        if sweep_hidden(d) is None:
            say(f"draw {attempt}: the hidden linkage nears a dead point or cannot close in its range")
            continue
        task = task_from_draw(seed, d)
        why = _screen(task, d)
        if why:
            say(f"draw {attempt}: {why}")
            continue
        solids, asm = build(task, d)
        if not all(fits_build_volume(s, BUILD_ROOM) for s in solids.values()):
            say(f"draw {attempt}: a reference part does not fit the build volume with room")
            continue
        if check:
            with tempfile.TemporaryDirectory(dir=scratch) as tmp:
                path = write_answer(tmp, solids, asm)
                rep = grade(task, path)
            if not rep.passed:
                bad = next(c for c in rep.checks if c.name in rep.failed())
                say(f"draw {attempt}: reference fails {', '.join(rep.failed())} ({bad.name}: {bad.reason})")
                continue
        d["attempt"] = attempt
        return task, d, solids, asm
    raise RuntimeError(f"no workable draw for seed {seed}")


def write_folder(task: Task, d: dict, solids: dict, asm: dict, out_dir: str, broken: bool = True) -> str:
    """The task folder: what the model sees, and ``_solution/`` (reference and broken copies) beside it."""
    from .brief import brief

    folder = os.path.join(out_dir, task.id)
    if os.path.isdir(folder):
        shutil.rmtree(folder)
    os.makedirs(folder)
    task.root = folder
    with open(os.path.join(folder, "task.json"), "w", encoding="utf-8", newline="\n") as f:
        f.write(task.dumps())
    with open(os.path.join(folder, "README.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write(brief(task))
    fx = task.fixed_solids(0)
    for n, s in fx.items():
        write_step(s, os.path.join(folder, f"fixed_{n}.step"))
    for k, s in enumerate(target_solids(task)):
        write_step(s, os.path.join(folder, f"fixed_B_pose{k + 1}.step"))
    sol = os.path.join(folder, "_solution")
    write_answer(sol, solids, asm)
    with open(os.path.join(sol, "reference.json"), "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps({"seed": task.seed, "draw": d}, indent=2, sort_keys=True, default=float) + "\n")
    if broken:
        for name, opts, must, why in broken_variants(d):
            s2, a2 = build(task, d, **opts)
            bf = os.path.join(sol, "broken", name)
            write_answer(bf, s2, a2)
            with open(os.path.join(bf, "MUST_FAIL"), "w", encoding="utf-8", newline="\n") as f:
                f.write(must + "\n")
            with open(os.path.join(bf, "WHY"), "w", encoding="utf-8", newline="\n") as f:
                f.write(why + "\n")
    return folder
