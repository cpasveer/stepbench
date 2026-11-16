"""Generating a "locks" task (level 4d), answer first, from a hidden bayonet.

The situation: two fixtures with flat mounting faces, parallel and facing each
other, as at level 1. The brief states an axis square to the faces, that B
comes in straight along it, and an angle B must then turn about it. The model
designs one part on A and one on B such that B goes in freely, turns through
the angle freely, is then locked (it cannot be pulled back out), and is not
locked before the turn (it could still be pulled straight out).

The hidden reference is a bayonet:

  part_A  a plate on A with a tube standing on the axis; through its wall,
          for each lug, an L-shaped slot: a channel straight down from the
          top, then round the wall through the turn angle
  part_B  a plate on B with a post coming down the axis into the tube, with
          two or three lugs that run down the straight channels and round the
          turning ones

Everything prints on filament without support. A locking face that lies flat
would need a bridge as long as the turn (20 mm and more) and fail the
printing rules, so the lugs' top faces and the slot ceilings above them slope
at 45 degrees, the steepest a layer may overhang, falling away from the axis.
The slots are grooves inside the tube's bore, not cut through its wall (a
1.5 mm skin stays outside): so every edge where two faces meet is at least
135 degrees, and no knife edge is left thinner than the printing rules'
minimum wall (a slope rising away from the axis, or a slot right through the
wall, leaves a 45-degree edge 0.2 mm thick). Running gaps are 0.4 mm
(measured square to each face). Pulling B back closes the 45-degree gap after
0.4 x sqrt 2 = 0.57 mm: the stated play limit is 0.8 mm, so the reference
sits inside the 0.3 mm printing gap and the 0.8 mm play with room both ways.

Once locked, B carries a stated pull-out load (0.5-1.5 kg, drawn as the other
load levels draw theirs) straight back along the axis; the lock must carry it
in the printed plastic, judged with 4a's allowables (``grade``). The score is
the printed weight of the two parts, as at levels 3, 4a and 4b.

A draw is kept only if the tube and post fit between the fixtures' bolts and
inside their faces, every part fits the Prusa MINI with room, and the
reference passes the real grader. Deterministic per seed.
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
from ..moves import motion as MO

LEVEL = 4  # level 4d: next to 4a assembles, 4b adapts, 4c moves; builds on levels 1 and 2
LEVEL_NAME = "4d"
NAME = "locks"
SITUATION = (
    "Two fixtures are given. Their mounting faces are parallel and face each other. Fixture A\n"
    "stays where it is. Fixture B comes in straight along a stated axis, then turns about it\n"
    "through a stated angle, and must then be locked: it cannot be pulled back out, and it carries\n"
    "a stated pull. Before the turn it must still come straight out."
)
KNOBS_DEFAULT = {}

SETBACK = 2.0
REF_GAP = 0.4
WALL = 4.0  # tube wall
SKIN = 1.5  # tube wall left outside a slot
LUG_W = 5.0  # lug width at its outer face (the lug is a slice of a ring: its sides are radial)
LUG_H = 3.0  # lug height at the post
FLOOR = 3.0  # tube wall under the turning channel
ROOF = 4.0  # tube wall above the highest point of the turning channel
TOOL_LENGTH = 30.0
OVERLOAD_KG = 100.0  # the "overloaded" broken copy's pull: far past what this lock carries
G_ACC = 9.81  # m/s2: the stated mass's weight, as the stiffness analysis turns a stated load into force
LOCKS_TOLERANCES = {
    "gap_min": 0.3,  # mm, between the two sides at every step of the turn
    "play_max": 0.8,  # mm, free shift once locked (along the axis and square to it)
    "tilt_max_deg": 5.0,
    "loose_shift": 5.0,  # mm: a shift this far without touching = it comes out
    "loose_tilt": 15.0,
    "step_max_deg": 2.0,
    "step_move_max": 1.5,
}
REQUIRED = ["SEAT_INSIDE", "CLEAN", "MANUFACTURING"]
FILES = {"A": "part_A.step", "B": "part_B.step"}


def _no_spigot(itf):
    p = dict(itf.params)
    p.pop("spigot", None)
    return make_interface(itf.kind, p)


def _inside_face(itf, pl: Placement, centre, radius: float, margin: float) -> bool:
    """A circle of ``radius`` about the world point ``centre`` lies on the face, ``margin`` inside its edge."""
    r = np.array(pl.rot)
    x, y, _ = r.T @ (np.asarray(centre, float) - np.asarray(pl.origin, float))
    if itf.kind == "round":
        return math.hypot(x, y) + radius <= itf.face_w / 2 - margin
    return abs(x) + radius <= itf.face_w / 2 - margin and abs(y) + radius <= itf.face_h / 2 - margin


def start_b(d: dict) -> Placement:
    return Placement.from_yaw_flip((d["B"]["center"][0], d["B"]["center"][1], d["gap"]), d["B"]["yaw"], True)


def draw(rng: random.Random) -> dict | None:
    A, B = _no_spigot(sample_interface(rng)), _no_spigot(sample_interface(rng))
    yaw_a, yaw_b = 5.0 * rng.randrange(72), 5.0 * rng.randrange(72)
    pa = Placement.from_yaw_flip((0.0, 0.0, 0.0), yaw_a, False)
    P = np.array([rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)])
    room_a = min(math.hypot(*(np.asarray(pa.point((x, y, 0)))[:2] - P)) - BOLTS[A.bolt].head_d / 2 - 1.5
                 for x, y in A.holes)
    Ro = math.floor(min(room_a, 20.0) * 2) / 2
    if Ro < 11.0 or not _inside_face(A, pa, (P[0], P[1], 0.0), Ro, SETBACK + 1.0):
        return None
    t_a, t_b = max(5.0, BOLTS[A.bolt].grip_min), max(5.0, BOLTS[B.bolt].grip_min)
    head_b = BOLTS[B.bolt].head_h
    g = REF_GAP
    Ri = Ro - WALL
    Rlo = Ro - SKIN - g
    zb = t_a + FLOOR + g
    top = zb + LUG_H + (Rlo - Ri) + g * math.sqrt(2) + ROOF
    gap = round(top + head_b + 1.5 + t_b + rng.uniform(2.0, 12.0), 1)
    cb = P + np.array([rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)])
    d = {"A": {"kind": A.kind, "params": A.params, "yaw": yaw_a},
         "B": {"kind": B.kind, "params": B.params, "yaw": yaw_b, "center": [round(float(cb[0]), 1), round(float(cb[1]), 1)]},
         "gap": gap, "P": [round(float(P[0]), 2), round(float(P[1]), 2)], "t": {"A": t_a, "B": t_b}}
    pb = start_b(d)
    Rp = Ri - g
    if not _inside_face(B, pb, (P[0], P[1], gap), Rp, SETBACK + 1.0):
        return None
    for x, y in B.holes:  # B's bolt heads keep clear of the post
        if math.hypot(*(np.asarray(pb.point((x, y, 0)))[:2] - P)) < Rp + BOLTS[B.bolt].head_d / 2 + 1.5:
            return None
    th = rng.choice([-1, 1]) * float(rng.randint(30, 90))
    from ..requirements import draw_stated_load

    d["mass"] = draw_stated_load(rng)  # the pull-out load once locked, as the other load levels draw it
    n = 3 if abs(th) <= 60 and rng.random() < 0.5 else 2
    d.update({"Ro": Ro, "zb": zb, "top": round(top, 6), "turn_deg": th, "lugs": n,
              "phi0": round(rng.uniform(0.0, 360.0 / n), 1)})
    return d


def task_from_draw(seed: int, d: dict) -> Task:
    parts = {n: make_interface(d[n]["kind"], d[n]["params"]) for n in ("A", "B")}
    pose = {"A": Placement.from_yaw_flip((0.0, 0.0, 0.0), d["A"]["yaw"], False), "B": start_b(d)}
    tol = dict(DEFAULT_TOLERANCES, required_checks=list(REQUIRED),
               manufacturing={"process": "fdm", "quantity": 1, "direction": None},
               locks=dict(LOCKS_TOLERANCES), tool_length=TOOL_LENGTH, tool_diameter_extra=0.0,
               score="material")  # the printed weight of the two parts: lighter is better (as 3, 4a, 4b)
    meta = {"generator": NAME, "level_name": LEVEL_NAME, "knobs": dict(KNOBS_DEFAULT),
            "lock": {"fixed": "A", "moving": "B", "axis_point": [d["P"][0], d["P"][1], 0.0], "axis_direction": [0.0, 0.0, 1.0],
                     "insert_direction": [0.0, 0.0, -1.0], "turn_deg": d["turn_deg"],
                     "pull_load_kg": d["mass"], "pull_load_N": round(d["mass"] * G_ACC, 3)}}
    task = Task(f"{NAME}-s{seed}", LEVEL, seed, parts, [pose], tol, meta, [], GENERATOR_VERSION, family=NAME)
    from ..requirements import apply_key_turn

    apply_key_turn(task)  # every bolt turnable with its hex key, or the draw is rejected
    return task


# --------------------------------------------------------------------------
# the reference bayonet, and broken copies


def _plane(P, phi_deg: float) -> bd.Plane:
    """A plane through the axis at angle phi: local x radial, local y up, local z square to it."""
    c, s = math.cos(math.radians(phi_deg)), math.sin(math.radians(phi_deg))
    return bd.Plane(origin=(P[0], P[1], 0.0), x_dir=(c, s, 0.0), z_dir=(s, -c, 0.0))


def _plate(itf, pl: Placement, t: float) -> bd.Part:
    sk = bd.Circle(itf.face_w / 2 - SETBACK) if itf.kind == "round" else \
        bd.Rectangle(itf.face_w - 2 * SETBACK, itf.face_h - 2 * SETBACK)
    body = bd.extrude(sk, t)
    hd = BOLTS[itf.bolt].clear_medium
    for x, y in itf.holes:
        body = body - bd.Pos(x, y, t / 2) * bd.Cylinder(hd / 2, t + 2)
    return body.moved(pl.location())


def shapes(d: dict, g_bore: float = REF_GAP) -> dict:
    """The section lines of the reference (r, z), shared by its builder and its broken copies."""
    g = REF_GAP
    Ro = d["Ro"]
    Ri = Ro - WALL
    Rp = Ri - g_bore
    zb = d["zb"]
    r0 = Ri - g - 1.0  # where the lug starts, inside the post
    Rlo = Ro - SKIN - g  # where it ends, 0.4 mm short of the groove's back
    Rg = Ro - SKIN  # the groove's back

    def ztop(r):  # the lug's 45-degree top, falling away from the axis
        return zb + LUG_H + (Rlo - r)

    def zceil(r):  # the slot's ceiling, 0.4 mm above the lug's top (square to the face)
        return ztop(r) + g * math.sqrt(2)

    return {"Ro": Ro, "Ri": Ri, "Rp": Rp, "Rlo": Rlo, "Rg": Rg, "zb": zb, "r0": r0, "ztop": ztop, "zceil": zceil,
            "top": d["top"]}


def lug_half_angle(S: dict) -> float:
    """Half the lug's angular width (deg): its outer corners lie LUG_W / 2 from its centre plane."""
    return math.degrees(math.asin(LUG_W / 2 / S["Rlo"]))


def _end_margin(S: dict) -> float:
    """Degrees beyond the lug's centre plane at which a turning channel ends. The lug's side and the channel's
    end are both radial planes, closest at the bore: the gap there is Ri sin(angle between them)."""
    return lug_half_angle(S) + math.degrees(math.asin(REF_GAP / S["Ri"])) + 1.0


def build(task: Task, d: dict, *, bump: bool = False, open_after_turn: bool = False, locked_at_start: bool = False,
          g_bore: float = REF_GAP) -> dict:
    """{"A": part on A, "B": part on B}, both at B's start pose (pushed in, not turned).

    bump             a block in one turning channel, half-way round
    open_after_turn  a second straight channel at the end of each turn: B pulls out after turning
    locked_at_start  the straight channels at the end of the turn instead of the start
    g_bore           the post's gap in the tube's bore
    """
    P = d["P"]
    S = shapes(d, g_bore)
    g = REF_GAP
    pa, pb = task.poses[0]["A"], task.poses[0]["B"]
    t_a, t_b, gap = d["t"]["A"], d["t"]["B"], d["gap"]
    th, n, phi0 = d["turn_deg"], d["lugs"], d["phi0"]
    axis = bd.Axis((P[0], P[1], 0.0), (0, 0, 1))
    # round bodies are revolved, their seam in solid wall half-way between two channels
    seam = phi0 + th / 2 + 180.0 / n
    tube = bd.revolve(_plane(P, seam) * bd.Polygon((S["Ri"], t_a), (S["Ro"], t_a), (S["Ro"], S["top"]),
                                                    (S["Ri"], S["top"]), align=None), axis, 360)
    em = _end_margin(S)
    for k in range(n):
        phi = phi0 + k * 360.0 / n
        # from the straight channel's centre plane (inside that channel, so no sliver) round to past the turn
        lo, hi = (phi, phi + th + em) if th > 0 else (phi + th - em, phi)
        if locked_at_start:  # the straight channel sits at the turn's end: the turning one runs from the start
            lo, hi = (phi - em, phi + th) if th > 0 else (phi + th, phi + em)
        if open_after_turn:  # a second straight channel at the turn's end takes in that end
            lo, hi = (phi, phi + th) if th > 0 else (phi + th, phi)
        ring = [(S["Ri"] - 1.0, S["zb"] - g), (S["Rg"], S["zb"] - g),
                (S["Rg"], S["zceil"](S["Rg"])), (S["Ri"] - 1.0, S["zceil"](S["Ri"] - 1.0))]
        face = _plane(P, lo) * bd.Polygon(*ring, align=None)
        tube = tube - bd.revolve(face, axis, hi - lo)
        starts = []
        if not locked_at_start:
            starts.append(phi)
        if open_after_turn or locked_at_start:
            starts.append(phi + th)
        for a in starts:
            slot = [(S["Ri"] - 1.0, S["zb"] - g), (S["Rg"], S["zb"] - g), (S["Rg"], S["top"] + 1.0),
                    (S["Ri"] - 1.0, S["top"] + 1.0)]
            tube = tube - _plane(P, a) * bd.extrude(bd.Polygon(*slot, align=None), LUG_W / 2 + g, both=True)
        if bump and k == 0:
            # far enough round that the lug, at the start, is clear of it; the lug meets it early in the turn
            mid = phi + math.copysign(min(0.85 * abs(th), lug_half_angle(S) + 9.0), th)
            blk = [(S["Ri"] - 0.5, S["zb"] - g - 0.5), (S["Rg"] + 0.5, S["zb"] - g - 0.5),
                   (S["Rg"] + 0.5, S["zceil"](S["Ri"] - 0.5) + 0.5), (S["Ri"] - 0.5, S["zceil"](S["Ri"] - 0.5) + 0.5)]
            block = _plane(P, mid) * bd.extrude(bd.Polygon(*blk, align=None), 1.0, both=True)
            ring_solid = bd.revolve(_plane(P, seam) * bd.Polygon((S["Ri"], t_a), (S["Ro"], t_a), (S["Ro"], S["top"]),
                                                                  (S["Ri"], S["top"]), align=None), axis, 360)
            tube = tube + (block & ring_solid)
    part_a = _plate(task.parts["A"], pa, t_a) + tube
    post_bottom = t_a + 1.0
    post = bd.revolve(_plane(P, seam) * bd.Polygon((0, post_bottom), (S["Rp"], post_bottom), (S["Rp"], gap - t_b),
                                                    (0, gap - t_b), align=None), axis, 360)
    part_b = _plate(task.parts["B"], pb, t_b) + post
    for k in range(n):
        phi = phi0 + k * 360.0 / n
        lug = [(S["r0"], S["zb"]), (S["Rlo"], S["zb"]), (S["Rlo"], S["ztop"](S["Rlo"])), (S["r0"], S["ztop"](S["r0"]))]
        ha = lug_half_angle(S)
        part_b = part_b + bd.revolve(_plane(P, phi - ha) * bd.Polygon(*lug, align=None), axis, 2 * ha)
    return {"A": part_a, "B": part_b}


def broken_variants(d: dict) -> list[tuple[str, dict, str, str]]:
    return [
        ("collides-while-turning", {"bump": True}, "TURN", "a block in one turning channel, partway round"),
        ("pulls-out-after-turn", {"open_after_turn": True}, "LOCKED",
         "a second straight channel where each turn ends: B comes straight out again"),
        ("locked-before-turn", {"locked_at_start": True}, "UNLOCKS",
         "the straight channels sit where the turn ends, not where it starts: B is caught before it turns"),
        ("gap-too-small", {"g_bore": 0.1}, "GAP", "the post is only 0.1 mm clear of the tube's bore"),
        ("overloaded", {}, "LOCK_STRENGTH", f"the reference itself, graded under a {OVERLOAD_KG:g} kg pull instead of "
         "the task's load: the plastic where the lugs bear gives way"),
    ]


def with_load(task: Task, kg: float) -> Task:
    """The same task under another pull-out load (for the overloaded broken copy)."""
    t = Task.from_json(json.loads(task.dumps()))
    t.root = task.root
    t.meta["lock"]["pull_load_kg"] = kg
    t.meta["lock"]["pull_load_N"] = round(kg * G_ACC, 3)
    return t


def write_answer(folder: str, parts: dict) -> str:
    os.makedirs(folder, exist_ok=True)
    for k, f in FILES.items():
        write_step(parts[k], os.path.join(folder, f))
    return folder


def generate(seed: int, *, check=True, progress=None):
    """(task, draw, reference parts)."""
    import tempfile

    from .grade import grade

    say = progress or (lambda s: None)
    rng = random.Random(f"{NAME}-{seed}")
    for attempt in range(100000):
        d = draw(rng)
        if d is None:
            continue
        task = task_from_draw(seed, d)
        fx = task.fixed_solids(0)
        if MO.distance(fx["A"], fx["B"]) < 3.0:
            continue
        parts = build(task, d)
        if not all(fits_build_volume(s, BUILD_ROOM) for s in parts.values()):
            say(f"draw {attempt}: a reference part does not fit the build volume with room")
            continue
        if check:
            with tempfile.TemporaryDirectory() as tmp:
                rep = grade(task, write_answer(tmp, parts))
            if not rep.passed:
                bad = next(c for c in rep.checks if c.name in rep.failed())
                say(f"draw {attempt}: reference fails {', '.join(rep.failed())} ({bad.name}: {bad.reason})")
                continue
        d["attempt"] = attempt
        return task, d, parts
    raise RuntimeError(f"no workable draw for seed {seed}")


def write_folder(task: Task, d: dict, parts: dict, out_dir: str, broken: bool = True) -> str:
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
    for n, s in task.fixed_solids(0).items():
        write_step(s, os.path.join(folder, f"fixed_{n}.step"))
    lk = task.meta["lock"]
    turned = MO.about(lk["axis_point"], lk["axis_direction"], lk["turn_deg"])
    write_step(task.fixed_solids(0)["B"].moved(turned.location()), os.path.join(folder, "fixed_B_turned.step"))
    sol = os.path.join(folder, "_solution")
    write_answer(sol, parts)
    with open(os.path.join(sol, "reference.json"), "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps({"seed": task.seed, "draw": d}, indent=2, sort_keys=True, default=float) + "\n")
    if broken:
        for name, opts, must, why in broken_variants(d):
            bf = os.path.join(sol, "broken", name)
            write_answer(bf, build(task, d, **opts))
            if name == "overloaded":
                with open(os.path.join(bf, "LOAD_KG"), "w", encoding="utf-8", newline="\n") as f:
                    f.write(f"{OVERLOAD_KG:g}\n")
            for fn, txt in (("MUST_FAIL", must), ("WHY", why)):
                with open(os.path.join(bf, fn), "w", encoding="utf-8", newline="\n") as f:
                    f.write(txt + "\n")
    return folder
