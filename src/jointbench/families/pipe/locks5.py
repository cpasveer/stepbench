"""Clamp a pipe, level 5 (locks): level 4 with the bolted joint between the two
parts replaced by a push-and-turn lock.

Everything of level 4 holds: the closed box round both fixed parts with the
pipe through its top and bottom walls, the one opening, each part printable
and bolted to its fixed part, the ways in. Between the two parts there are no
bolts: inside the box one part is pushed along an axis and turned about it,
and is then locked to the other (free to come apart before the turn, blocked
after it), carrying the stated load through the lock's plastic (level 4d's
checks, ``assemble.locked``).

The hidden reference, level 4's two parts without their joint bolts:

* part 2 (on the pipe) goes in first and waits on the pipe a little along it
  and turned back round it; part 1 (on the flat face) goes in as at level 4
  and is bolted; then part 2 slides along the pipe (the push) and turns round
  the pipe's axis (the turn), and is bolted to the pipe;
* the lock: a lug on part 2's arm, a block swept round the pipe's axis, and
  in part 1's ear a channel swept the same way. Turning round the pipe moves
  the arm almost straight at the ear, so the turn swings the lug from clear
  of the ear into the channel; there it holds part 2 along the pipe and off
  it, and the pipe under the saddle holds it the other ways.
"""
from __future__ import annotations

import json
import math
import os

import build123d as bd
import numpy as np

from ...task import Task
from . import assembles as A
from .assembles import _r, _u

LEVEL = 5
LUG_H = 6.0        # mm, the lug across (radially from the pipe's axis)
LUG_W = 10.0       # mm, the lug along the pipe
LUG_DEPTH = 6.0    # mm the lug reaches into the ear
LUG_ROOT = 2.0     # deg the lug starts inside the arm, past the joint face
CLEAR_DEG = 1.5    # deg past the joint face the lug stands at the start of the turn
GAP = 0.4          # mm round the lug in its channel (level 4d's printing gap)
TIP = 3.0          # mm the lug is wider along the pipe than it reaches out: its tip's solid end
EAR_FLARE = 6.0    # mm part 1's ear widens each side above the flat part's keys, round the channel
MOUTH = 2.0        # mm the sharp edge of the channel's mouth is cut back, each way
PUSH = 10.0        # mm part 2 slides along the pipe before it turns
LIFT_STEP = 2.0    # mm: part 2's lift off the pipe is raised in these steps until its way in clears
LOCKS_TOLERANCES = {  # level 4d's
    "gap_min": 0.3, "play_max": 0.8, "tilt_max_deg": 5.0, "loose_shift": 5.0, "loose_tilt": 15.0,
    "step_max_deg": 2.0, "step_move_max": 1.5,
}


def _offset(poly, d: float) -> list:
    """A convex polygon with every side moved out by ``d``."""
    P = np.asarray(poly, float)
    if 0.5 * np.sum(P[:, 0] * np.roll(P[:, 1], -1) - np.roll(P[:, 0], -1) * P[:, 1]) < 0:
        P = P[::-1]
    lines = []
    for i in range(len(P)):
        a, b = P[i], P[(i + 1) % len(P)]
        e = (b - a) / np.linalg.norm(b - a)
        nrm = np.array([e[1], -e[0]])  # outward for a counter-clockwise polygon
        lines.append((nrm, float(nrm @ a) + d))
    out = []
    for i in range(len(lines)):
        (n1, c1), (n2, c2) = lines[i - 1], lines[i]
        out.append(tuple(np.linalg.solve(np.array([n1, n2]), np.array([c1, c2]))))
    return out


def build(task: Task, *, joint_up: float = A.JOINT_UP, push: float = PUSH, variant: dict | None = None):
    """(part 1, part 2, assembly file, dimensions). ``variant`` makes the broken copies:
    ``no_channel`` (the lug meets the ear: the turn clashes), ``open_channel`` (the channel runs out
    through the ear's far side: pulled along the pipe the lug comes out), ``tight`` (the channel
    0.1 mm round the lug), ``shut_at_start`` (the lug starts inside the ear: it is caught before
    the turn), ``longer`` (the saddle that much longer each end: too big for the opening)."""
    var = dict(variant or {})
    p1, p2, asm4, dims = A.build(task, joint_up=joint_up, variant={"no_joint": True, "face_gap": GAP,
                                                                   "ear_flare": EAR_FLARE,
                                                                   "longer": var.get("longer", 0.0)})
    F = dims["F"]
    e1, e3, t, C, R, crest = F["e1"], F["e3"], F["t"], F["C"], F["R"], F["crest"]
    yb = _u(np.cross(t, e1))
    # the lock's place: the joint's middle on the joint face, in the pipe's own cylindrical frame
    c = np.mean(dims["pos"], axis=0) - C
    x1, yL, tL = float(c @ e1), float(c @ yb), float(c @ t)
    rL = math.hypot(x1, yL)
    r0, r1 = rL - LUG_H / 2, rL + LUG_H / 2
    face = [math.degrees(math.acos(x1 / r)) for r in (r0, r1)]  # the joint face's angle at r (y > 0 side)
    lo = min(face) - math.degrees(LUG_DEPTH / rL)  # into the ear: the angle falls towards +e1
    hi = max(face) + LUG_ROOT  # its root, inside the arm
    theta = max(face) + CLEAR_DEG - lo  # turned back this far, the lug stands wholly clear of the ear
    if var.get("shut_at_start"):
        theta = (hi - lo) / 2  # the lug still half in the ear at the start
    g = 0.1 if var.get("tight") else GAP  # every face of the section set out by this
    gdeg = math.degrees(g / r0)

    def ring(poly, a0, sweep):
        """A section (r, t) in the half-plane through the pipe's axis at angle a0 of the profile,
        swept round the axis by ``sweep`` deg."""
        u = math.cos(math.radians(a0)) * e1 + math.sin(math.radians(a0)) * yb
        pl = bd.Plane(origin=tuple(C), x_dir=tuple(u), z_dir=tuple(np.cross(u, t)))
        return bd.revolve(pl * bd.Polygon(*[tuple(v) for v in poly], align=None), bd.Axis(tuple(C), tuple(t)), sweep)

    def at(r, a):
        return C + r * (math.cos(math.radians(a)) * e1 + math.sin(math.radians(a)) * yb)

    # the lug sticks out of the arm's side; in part 2's print (it stands on the saddle's end its arm
    # runs to) its underside rises at 45 deg from the arm out to its tip, as a bracket grows out of a
    # wall: no ceiling, no keel. It is as wide along the pipe as it reaches out, and TIP more, so the
    # tip keeps a solid end; the pull along the pipe bears on its other side, flat
    down = 1.0 if dims["ta"][1] >= dims["t01"][1] - 1e-6 else -1.0  # +1: the bed lies towards +t
    root, tip = at(rL, hi), at(rL, lo)
    reach = float(np.linalg.norm(tip - root))
    w = max(LUG_W, reach + TIP)
    tt = (tL - w / 2, tL + w / 2)
    sec = [(r0, tt[0]), (r1, tt[0]), (r1, tt[1]), (r0, tt[1])]
    lug = ring(sec, lo, hi - lo)
    t_under = tt[1] if down > 0 else tt[0]
    q0 = root + t * t_under
    q1 = tip + t * (t_under - down * reach)
    radial = _u(at(1.0, (lo + hi) / 2) - C)
    nrm = _u(np.cross(q1 - q0, radial))
    if float(nrm @ (t * down)) < 0:
        nrm = -nrm  # towards the side cut away (the bed)
    lug = lug & (bd.Plane(origin=tuple(q0), z_dir=tuple(-nrm)) * bd.Pos(0, 0, 250.0) * bd.Box(1000, 1000, 500))
    deep = lo - gdeg  # the channel's end, deep in the ear
    out = hi + theta + 3.0  # and out past the start, through the joint face
    channel = ring(_offset(sec, g), deep, out - deep)  # the plain section: the lug's sweep fits it
    if var.get("open_channel"):  # the channel runs on along the pipe, back the way it came in, out of the ear
        for k in range(1, 13):
            channel = channel + channel.moved(bd.Location(tuple(-t * 5.0 * k)))  # the way it pulls back
    part2 = p2 + lug
    part1 = p1 if var.get("no_channel") else p1 - channel
    # the channel's mouth in the ear's joint face: on its side towards the pipe the channel leaves the
    # face at a sharp angle (it turns round the pipe's axis, the face is flat): that edge is cut back,
    # a triangle along it between the channel's two end walls
    jf = float(np.mean(dims["pos"], axis=0) @ e1 - C @ e1)  # the ear's joint face, in the profile
    ri = r0 - g
    if not var.get("no_channel") and ri > jf:
        E = np.array([jf, math.sqrt(ri * ri - jf * jf)])
        tang = np.array([E[1], -E[0]]) / ri  # along the channel's inner wall, into the ear
        tri = [tuple(E), tuple(E + np.array([0.0, -MOUTH])), tuple(E + tang * MOUTH)]
        fb = bd.Plane(origin=tuple(C), x_dir=tuple(e1), z_dir=tuple(t))
        inner = [v for v in _offset(sec, g) if abs(v[0] - (r0 - g)) < 1e-6]  # the inner face's ends
        z0, z1_ = min(v[1] for v in inner), max(v[1] for v in inner)
        face = bd.Face(bd.Wire.make_polygon([fb.from_local_coords((x, y, z0)) for x, y in tri], close=True))
        part1 = part1 - bd.extrude(face, amount=z1_ - z0, dir=tuple(t))
    s1, s2 = part1.solids(), part2.solids()
    if len(s1) != 1 or len(s2) != 1:
        raise ValueError("a part falls apart with the lock in it")
    part1, part2 = s1[0], s2[0]

    # the lock's motion, about the pipe's axis: the turn swings the lug into the ear (angle falling)
    lock = {"moving": "part2", "push": _r(t * push),
            "turn": {"point": _r(C), "axis": _r(t), "degrees": round(-theta, 6)}}
    from ...moves import motion as MO

    back = MO.about(C, t, theta).then(MO.shift(-t * push))  # final -> where part 2 waits
    waiting = part2.moved(back.location())
    # part 2's way in: in along -e1, lifted off the pipe along its crest as it waits, then down onto it
    crest_w = np.array(MO.about((0, 0, 0), t, theta).vec(tuple(crest)))
    from ...assemble.sweep import collisions

    pipe = task.fixed_solids(0)[F["pipe"]]
    lift = R * (1.0 - math.cos(math.radians(60.0))) + 1.0
    for _ in range(12):
        if max(collisions(waiting.moved(bd.Location(tuple(crest_w * lift))), -e1, {"pipe": pipe}).values()) < 1e-3:
            break
        lift += LIFT_STEP
    else:
        raise ValueError("part 2 cannot be lifted clear of the pipe for its way in")
    start2 = waiting.moved(bd.Location(tuple(crest_w * lift)))
    asm = {"parts": [
        {"name": "part2", "file": "part2.step", "bolts_to": F["pipe"], "insert_direction": _r(-e1),
         "path": [{"move": _r(-crest_w * lift)}]},
        dict(asm4["parts"][1])],
        "lock": lock}
    start1 = part1.moved(bd.Location(tuple(dims["entry1"])))
    dims = dict(dims, lock=lock, theta=theta, lift=lift, starts=(start1, start2), waiting=waiting,
                pushed=part2.moved(MO.about(C, t, theta).location()), lug=(r0, r1, lo, hi, tt))
    return part1, part2, asm, dims


def task5(task3: Task, walls: list, opening: dict, load_kg: float | None = None) -> Task:
    tk = Task.from_json(json.loads(task3.dumps()))
    tk.root = task3.root
    tk.id = task3.id.replace("-L3-", "-L5-")
    tk.level = LEVEL
    tk.obstacles = [walls]
    tk.tolerances["required_checks"] = [c for c in tk.tolerances.get("required_checks", []) if c != "STIFFNESS"]
    tk.tolerances["locks"] = dict(LOCKS_TOLERANCES)
    kg = load_kg if load_kg is not None else float(next(m["mass_kg"] for m in tk.stiffness["per_mount"]))
    tk.meta = dict(tk.meta, test="locks", assembly=True, lock_joint=True, opening=opening,
                   lock={"pull_load_kg": kg, "pull_load_N": round(kg * 9.81, 3)})
    return tk


def make(task3: Task | None = None, **kw):
    """(level 5 task, (part 1, part 2, assembly), dimensions)."""
    task3 = task3 or A.setup()
    p1, p2, asm, dims = build(task3, **kw)
    walls, opening = A.box(task3, p1, p2, dims, starts=dims["starts"], extra=(dims["waiting"], dims["pushed"]))
    return task5(task3, walls, opening), (p1, p2, asm), dims


# --------------------------------------------------------------------------
# broken copies, the task folder, the view


OVERLOAD_KG = 500.0  # the overloaded copy's pull: far past what the lug's plastic takes


def broken(task3: Task, dims: dict, part1, part2, asm: dict) -> dict:
    """name -> (part 1, part 2, assembly dict, the check that must fail, load in kg or None)."""
    out = {}
    for name, var, must in (("collides-while-turning", {"no_channel": True}, "TURN"),
                            ("pulls-out-after-turn", {"open_channel": True}, "LOCKED"),
                            ("locked-before-turn", {"shut_at_start": True}, "UNLOCKS"),
                            ("gap-too-small", {"tight": True}, "GAP"),
                            ("too-big-for-the-opening", {"longer": 20.0}, "INSERTION")):
        q1, q2, a, _ = build(task3, joint_up=dims["joint_up"], variant=var)
        out[name] = (q1, q2, a, must, None)
    out["overloaded"] = (part1, part2, asm, "LOCK_STRENGTH", OVERLOAD_KG)
    return out


VIEW_NOTE = ("A ship in a bottle round a pipe, locked: no bolts between the two parts. Grey: the pipe and the "
             "flat plate. Red, see-through: the closed box, the pipe through its top and bottom walls, its one "
             "opening in the front wall. Green: part 1, on the plate; teal: part 2, on the pipe, which slides along "
             "the pipe and turns round it, swinging the lug on its arm into the channel in part 1's ear. (In the "
             "task part 2 goes in first and waits on the pipe; here part 1 is shown first, so the slider ends with "
             "the push and the turn.) Slide to take them out again; drag to turn, scroll to zoom.")


def write_folder(task: Task, ref, dims: dict, out_dir: str, *, task3: Task | None = None, view: bool = True) -> str:
    import shutil

    from ...assemble.brief import SUBMISSION
    from ...assemble.generate import write_answer
    from ...geometry import write_step
    from .brief5 import brief

    folder = os.path.join(out_dir, task.id)
    if os.path.exists(folder):
        shutil.rmtree(folder)
    os.makedirs(folder)
    with open(os.path.join(folder, "task.json"), "w", encoding="utf-8", newline="\n") as f:
        f.write(task.dumps())
    for n, s_ in task.fixed_solids(0).items():
        write_step(s_, os.path.join(folder, f"fixed_{n}.step"))
    write_step(bd.Compound([o.solid() for o in task.obstacles_at(0)]), os.path.join(folder, "obstacles.step"))
    with open(os.path.join(folder, "README.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write(brief(task))
    sub = SUBMISSION.replace(
        '"joint": {"bolt": "M4", "head_on": "part1", "axis": [0, 0, -1], "positions": [[0, -10, 20], [0, 10, 20]]},',
        '"lock": {"moving": "part2", "push": [0, 0, 5], "turn": {"point": [0, 0, 0], "axis": [0, 0, 1], "degrees": 30}},')
    with open(os.path.join(folder, "submission.py"), "w", encoding="utf-8", newline="\n") as f:
        f.write(sub)
    sol = os.path.join(folder, "_solution")
    write_answer(sol, *ref)
    with open(os.path.join(sol, "reference.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump({"level": LEVEL, "test": "locks", "situation": "pipe", "lock": ref[2]["lock"],
                   "turn_deg": round(dims["theta"], 3), "push_mm": PUSH}, f, indent=2)
        f.write("\n")
    for name, (q1, q2, a, must, kg) in broken(task3 or A.setup(), dims, *ref).items():
        bdir = os.path.join(sol, "broken", name)
        write_answer(bdir, q1, q2, a)
        with open(os.path.join(bdir, "MUST_FAIL"), "w", encoding="utf-8") as f:
            f.write(must + "\n")
        if kg is not None:
            with open(os.path.join(bdir, "LOAD_KG"), "w", encoding="utf-8") as f:
                f.write(f"{kg:g}\n")
    if view:
        write_view(task, ref, dims, os.path.join(sol, "view"))
    return folder


def write_view(task: Task, ref, dims: dict, out: str) -> str:
    """The view: part 1 first, then part 2 along its way in, its push and its turn into the lock."""
    import tempfile

    from ...assemble.generate import write_answer
    from ...assemble.view import write as write_view_

    p1, p2, asm = ref
    lk = asm["lock"]
    shown = {"parts": [dict(asm["parts"][1]), dict(asm["parts"][0], path=list(asm["parts"][0]["path"])
                                                    + [{"move": lk["push"]}, {"turn": lk["turn"]}])],
             "joint": {"bolt": "M5", "head_on": "part1", "axis": [1, 0, 0],  # (the view reads no joint)
                       "positions": [[0, 0, 0], [0, 100, 0]]}}
    F = dims["F"]
    with tempfile.TemporaryDirectory() as tmp:
        write_answer(tmp, p1, p2, shown)
        return write_view_(task, tmp, out, heading=f"Clamp a pipe, level 5 (locks): {task.id}", note=VIEW_NOTE,
                           view_dir=_u(F["e1"] + 0.45 * F["t"] - 0.2 * F["e3"]))


def main(argv=None) -> int:
    """python -m jointbench.families.pipe.locks5 OUT_DIR: the version 1 pipe setup's level 5 task folder."""
    import sys

    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) != 1:
        print(main.__doc__)
        return 2
    t3 = A.setup()
    t5, ref, dims = make(t3)
    print(write_folder(t5, ref, dims, args[0], task3=t3))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
