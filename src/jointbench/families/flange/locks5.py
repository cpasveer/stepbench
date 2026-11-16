"""Flange and block, level 5 (locks): level 4 with the bolted joint between the
two parts replaced by a push-and-turn lock.

Everything of level 4 holds: the closed box round both fixed parts, the one
opening, each part printable and bolted to its fixed part, the ways in. Between
the two parts there are no bolts: inside the box one part is pushed along an
axis and turned about it, and is then locked to the other (free to come apart
before the turn, blocked after it), carrying the stated load through the lock's
plastic (level 4d's checks, ``assemble.locked``).

The hidden reference, level 4's two parts with the side wall's two halves
meeting end to end (no lapped layers, no joint bolts, the wall ``WALL_T``
thick), the two wall ends square to the flange's axis, a gap between:

* part 2 (on the block) goes in first, as at level 4. Part 1 (on the flange)
  is the moving part: it goes in along the flange's face clear of the boss,
  turned back about the flange's own axis, then onto the boss to where it
  waits, ``push`` mm off the face; it is pushed onto the face (the push, along
  the flange's axis) and turned about that axis (the turn) on the face, which
  bears it; then both are bolted. The wall ends, square to that axis, slide
  past each other as it turns, so the gap needs only the push and the boss;
* the lock: a lug on the end of part 1's wall, near its top, reaching across
  the gap into part 2's wall end; a block swept round the flange's axis, its
  underside rising at 50 deg from the wall (it prints as a bracket grows out of
  a wall). In part 2's wall end the lug's place, ``GAP`` all round, and above
  it the wall end cut back as far as the lug may be pulled. Turned back, the
  lug stands wholly above its place; the turn swings it down into it, where its
  end bears on the far wall if part 1 is pulled back off the flange.
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
LUG_H = 5.0        # mm, the lug radially (from the flange's axis)
LUG_ROOT = 3.0     # mm the lug starts inside part 1's wall end
LUG_DEPTH = 6.0    # mm the lug reaches into part 2's wall end
LUG_CHAMFER = 1.5  # mm the lug's lower outer edge is chamfered in
TIP = 3.0          # mm the lug's end stands tall (along the turn) where it is shortest
SLOPE = 1.2        # the lug's underside rises this much per mm out from the wall (50 deg)
TOP = 3.0          # mm below the walls' top the lug's top stays
GAP = 0.4          # mm round the lug in its slot (level 4d's printing gap)
SHUT_DEG = 1.0     # deg the locked-before-turn copy is turned back: its lug's end still in its place
CLEAR_DEG = 1.5    # deg past wholly clear of part 2 the lug stands at the start of the turn
MARGIN = 1.5       # mm the wall ends stay apart at their closest, beyond what the moves need
WALL_T = 24.0      # mm, the side wall (no joint bolts through it here: as thick as the lug's slot needs)
LOCKS_TOLERANCES = {  # level 4d's
    "gap_min": 0.3, "play_max": 0.8, "tilt_max_deg": 5.0, "loose_shift": 5.0, "loose_tilt": 15.0,
    "step_max_deg": 2.0, "step_move_max": 1.5,
}
X = (1.0, 0.0, 0.0)  # the flange's axis in the arrangement frame (A's face is x = 0, its centre the origin)


def _ring(r0, r1, x0, x1, a0, sweep):
    """The block r0..r1 from the flange's axis, x0..x1 along it, from a0 deg (from +y towards +z) on
    for ``sweep`` deg round it."""
    a = math.radians(a0)
    u = (0.0, math.cos(a), math.sin(a))
    pl = bd.Plane(origin=(0, 0, 0), x_dir=u, z_dir=tuple(np.cross(u, X)))
    sec = bd.Polygon((r0, x0), (r1, x0), (r1, x1), (r0, x1), align=None)
    return bd.revolve(pl * sec, bd.Axis((0, 0, 0), X), sweep)


def _turned(shape, deg):
    return shape.rotate(bd.Axis((0, 0, 0), X), deg)


def lock_parts(ctx: dict, butt: float, *, gap: float = GAP, var: dict | None = None):
    """(lug, slot, numbers) in the arrangement frame for part 1's wall ending at ``ctx['s1']`` and
    part 2's starting ``butt`` on.

    The lug: a block swept round the flange's axis (radially LUG_H, from LUG_ROOT inside part 1's wall
    end to LUG_DEPTH inside part 2's), its underside cut to rise at SLOPE out from the wall; placed
    across the wall's middle at its root, its top TOP under the walls' top. The slot in part 2: the
    lug's place, ``gap`` all round (it bears there when pulled back), and above it part 2's wall end
    cut away altogether as far as the lug may be pulled back (level 4d's ``loose_shift``) and more:
    turned back past the lug's height, the lug stands there wholly clear."""
    var = dict(var or {})
    qa, sh, w, T, s1 = ctx["qa"], ctx["sh"], ctx["w"], ctx["T"], ctx["s1"]
    z_top = ctx["z"][1]
    mid = qa + w * T / 2  # the wall's middle line

    def y_at(x, w_=T / 2):  # across the flange's axis, where the wall's line at w_ out passes x
        p0 = qa + w * w_
        return float(p0[1] + (x - p0[0]) * sh[1] / sh[0])

    zt = z_top - TOP

    def face_x(y, s):  # x of a wall end: square to A's axis (``ctx['x_ends']``), or else square to the wall
        if ctx.get("x_ends"):
            return ctx["x_ends"][0] if abs(s - s1) < 1e-9 else ctx["x_ends"][1]
        return (float(sh @ qa) + s - sh[1] * y) / sh[0]

    del mid
    rp = math.hypot(y_at(float((qa + s1 * sh)[0])), zt - 8.0)
    for _ in range(6):  # centre the lug's reach across (y) on the wall's middle half way along the lug
        r0, r1 = rp - LUG_H / 2, rp + LUG_H / 2
        hi = math.degrees(math.asin(zt / r1))
        ys = [r0 * math.cos(math.radians(hi)), r1]
        xa = min(face_x(y, s1) for y in (min(ys) - 2.0, max(ys) + 2.0)) - LUG_ROOT
        xb = max(face_x(y, s1 + butt) for y in (min(ys) - 2.0, max(ys) + 2.0)) + LUG_DEPTH
        z_low = r0 * math.sin(math.radians(hi)) - TIP - SLOPE * (xb - xa)
        lo = math.degrees(math.asin(max(-0.99, z_low / r1))) - 1.0
        y_lo = r0 * math.cos(math.radians(max(abs(lo), abs(hi))))
        y_hi = r1 if lo < 0 < hi else r1 * math.cos(math.radians(min(abs(lo), abs(hi))))
        yp = y_at((xa + xb) / 2)
        if yp <= 0:
            raise ValueError("the lock's wall lies on the wrong side of the flange's axis")
        rp += yp - (y_lo + y_hi) / 2
    lug = _ring(r0, r1, xa, xb, lo, hi - lo)
    n = _u((-SLOPE, 0.0, 1.0))  # keep above the underside's plane through (xa, ., z_low)
    keep = bd.Plane(origin=(xa, 0.0, z_low), z_dir=tuple(n)) * bd.Pos(0, 0, 250.0) * bd.Box(1000, 1000, 500)
    lug = lug & keep
    # the edge where that underside meets the lug's outer face is chamfered (they meet at 90 deg less
    # the lug's angle round the axis: a sharp edge, which prints as a thin wall)
    def corner(x):
        zs = z_low + SLOPE * (x - xa)
        return np.array([x, math.sqrt(max(r1 * r1 - zs * zs, 0.0)), zs])

    c0, c1 = corner(xa), corner(xb)
    phi_c = math.atan2(float(c0[2] + c1[2]) / 2, float(c0[1] + c1[1]) / 2)
    out = np.array([0.0, math.cos(phi_c), math.sin(phi_c) - 1.0])  # between the two faces' outward normals
    run = _u(c1 - c0)
    out = _u(out - run * float(out @ run))
    lug = lug & (bd.Plane(origin=tuple((c0 + c1) / 2 - out * LUG_CHAMFER), z_dir=tuple(-out))
                 * bd.Pos(0, 0, 250.0) * bd.Box(1000, 1000, 500))
    gdeg = math.degrees(gap / r0)
    x_far = xb + gap + (300.0 if var.get("open_slot") else 0.0)
    slot = _ring(r0 - gap, r1 + gap, xa - 40.0, x_far, lo - gdeg, hi - lo + 2 * gdeg)
    # above the lug's place part 2's wall end is cut back, from the lowest point of the place's top up,
    # inside the place's outer face carried straight up (a floor tilted with the turn, or carried out to
    # the wall's outside, would meet a face at a sharp edge); the wall's outer skin stays full height
    z_floor = (r0 - gap) * math.sin(math.radians(hi + gdeg))
    y_v = (r1 + gap) * math.cos(math.radians(hi + gdeg))
    # its back square to the wall (square to x it would meet the wall's slanted inside at a sharp edge),
    # past where the lug's end may be pulled back to
    pull = LOCKS_TOLERANCES["loose_shift"] + 1.5 + (300.0 if var.get("open_slot") else 0.0)
    s_back = max(float((np.array([xb + pull, y_]) - qa) @ sh) for y_ in (y_lo - gap, r1 + gap)) + 1.0
    x_front = xa - 40.0
    y0_ = y_at(x_front, 0.0) - 3.0  # a little past the wall's inside

    def x_on_back(y_):
        return float(qa[0] + (s_back - (y_ - qa[1]) * sh[1]) / sh[0])

    plan = [(x_front, y0_), (x_on_back(y0_), y0_), (x_on_back(y_v), y_v), (x_front, y_v)]
    face = bd.Face(bd.Wire.make_polygon([bd.Vector(float(x_), float(y_), z_floor) for x_, y_ in plan], close=True))
    slot = slot + bd.extrude(face, amount=z_top + 20.0 - z_floor, dir=(0, 0, 1))
    return lug, slot, {"r": (r0, r1), "x": (xa, xb), "deg": (lo, hi), "z_low": z_low,
                       "theta": hi - lo + gdeg + CLEAR_DEG,
                       "margins_y": (y_lo - gap - max(y_at(xa, 0.0), y_at(xb + gap, 0.0)),
                                     min(y_at(xa, T), y_at(xb + gap, T)) - y_hi - gap)}


def build(task3: Task, d: dict, *, variant: dict | None = None):
    """(part 1, part 2, assembly file, dimensions), world coordinates. ``variant`` makes broken copies:
    ``no_slot`` (the lug meets part 2: the turn clashes), ``open_slot`` (the slot runs on along the
    flange's axis through part 2: pulled back the lug comes out), ``tight`` (the slot 0.1 mm round the
    lug), ``shut_at_start`` (turned back only SHUT_DEG: caught before the turn), ``taller``
    (part 2 that much taller: too big for the opening)."""
    var = dict(variant or {})
    lay = A.S.layout(d)
    ia = lay["itf"]["A"]
    spig = max([p.height for p in ia.protrusions] or [0.0])
    push = A.ONTO  # part 1 waits this far off A's face: its way in already took it onto A's boss
    # level 5's wall: square ends, as thick as the slot needs; no thickening beside the bolts (no stiffness
    # is asked here, and it would crowd the slot's cut-back)
    plain = dict(ends_x=True, wall_t=WALL_T, gusset=0.0)
    tall = dict(plain, **({"taller": var["taller"]} if var.get("taller") else {}))
    # the gap between the wall ends. Part 2 is in place first; part 1 comes in along A's face (square to
    # x) as far off it as the push and A's boss, and is pulled back as far as UNLOCKS tries: the wall
    # ends, square to x, must be that far apart, and a margin; then wider until part 1 comes in clear of
    # part 2 and turns clear of it
    from ...assemble.sweep import path_collisions
    from ...moves import motion as MO

    butt =math.ceil((max(push + spig, LOCKS_TOLERANCES["loose_shift"]) + MARGIN) * 2) / 2  # (UNLOCKS pulls it back)
    for _ in range(16):
        _, _, info = lock_parts(ctx_of(task3, d, butt, plain), butt)
        theta = info["theta"]
        q1, q2, _, dm = _assemble(task3, d, butt, plain, theta, GAP, {})
        if _clear(q1, q2, dm, theta, push, spig, path_collisions, MO):
            break
        butt += 1.0
    else:
        raise ValueError("no gap between the wall ends lets part 1 in past part 2 and turn")
    th = SHUT_DEG if var.get("shut_at_start") else theta
    gap = 0.1 if var.get("tight") else GAP

    p1, p2, asm4, dims = _assemble(task3, d, butt, tall, theta, gap, var)
    G = dims["frame"]
    na = np.array(G.vec(X))
    centre = np.array(G.point((0.0, 0.0, 0.0)))
    lock = {"moving": "part1", "push": _r(-na * push),
            "turn": {"point": _r(centre), "axis": _r(na), "degrees": round(-th, 6)}}
    path1 = [{"move": _r(-na * spig)}] if spig > 0 else []  # in along A's face clear of its boss, then onto it
    asm = {"parts": [dict(asm4["parts"][1]),
                     {"name": "part1", "file": "part1.step", "bolts_to": "A",
                      "insert_direction": asm4["parts"][0]["insert_direction"], "path": path1}],
           "lock": lock}
    from ...moves import motion as MO

    waiting = p1.moved(MO.about(centre, na, th).then(MO.shift(na * push)).location())
    started = p1.moved(MO.about(centre, na, th).location())
    start2 = p2.moved(bd.Location(tuple(-np.asarray(dims["moves"][1]))))
    start1 = waiting.moved(bd.Location(tuple(na * spig)))  # where its straight way in ends: off A's boss
    dims = dict(dims, lock=lock, theta=th, push=push, butt=butt, info=info, starts=(start1, start2),
                extra_poses=(waiting, started))
    return p1, p2, asm, dims


def ctx_of(task3, d, butt, tall):
    return A.build(task3, d, variant=dict(butt=butt, **tall), side=1)[3]["ctx"]


def _assemble(task3, d, butt, tall, theta, gap, var):
    """Level 4's build with the wall ends ``butt`` apart and the lock cut in."""
    def extra(part1, part2, ctx_):
        lug_, slot_, _ = lock_parts(ctx_, butt, gap=gap, var=var)
        return part1 + lug_, (part2 if var.get("no_slot") else part2 - slot_)

    return A.build(task3, d, variant=dict(butt=butt, extra=extra, **tall), side=1)


def _clear(p1, p2, dims, theta, push, spig, path_collisions, MO) -> bool:
    """With part 2 in place, part 1 comes in (along A's face, then onto A's boss) to where it waits clear
    of it, and, pushed, turns clear of it (every 1 deg)."""
    G = dims["frame"]
    na = np.array(G.vec(X))
    centre = np.array(G.point((0.0, 0.0, 0.0)))
    waiting = p1.moved(MO.about(centre, na, theta).then(MO.shift(na * push)).location())
    path = [{"move": list(-na * spig)}] if spig > 0 else []
    hits, _ = path_collisions(waiting, dims["dirs"][0], path, {"p2": p2})
    if max(hits.values()) > 1e-3:
        return False
    for a in np.arange(theta, -0.01, -1.0):
        x = p1.moved(MO.about(centre, na, float(a)).location()) & p2
        if (x.volume if x is not None else 0.0) > 1e-3:
            return False
    return True


def _local(G, v):
    """A world direction in the arrangement frame (G turns about the vertical only, so its inverse is
    its transpose)."""
    R = np.array(G.rot, float)
    return R.T @ np.asarray(v, float)


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


def make(task3: Task, d: dict, **kw):
    """(level 5 task, (part 1, part 2, assembly), dimensions)."""
    p1, p2, asm, dims = build(task3, d, **kw)
    walls, opening = A.box(task3, p1, p2, dims, starts=dims["starts"], extra=dims["extra_poses"])
    return task5(task3, walls, opening), (p1, p2, asm), dims


# --------------------------------------------------------------------------
# broken copies, the task folder, the view


TALLER = 45.0  # mm part 2 is made taller in the too-big copy (the opening is tall: part 1 comes in turned back)
OVERLOAD_KG = 500.0  # the overloaded copy's pull: far past what the lug's plastic takes


def broken(task3: Task, d: dict, part1, part2, asm: dict) -> dict:
    """name -> (part 1, part 2, assembly dict, the check that must fail, load in kg or None)."""
    out = {}
    for name, var, must in (("collides-while-turning", {"no_slot": True}, "TURN"),
                            ("pulls-out-after-turn", {"open_slot": True}, "LOCKED"),
                            ("locked-before-turn", {"shut_at_start": True}, "UNLOCKS"),
                            ("gap-too-small", {"tight": True}, "GAP"),
                            ("too-big-for-the-opening", {"taller": TALLER}, "INSERTION")):
        q1, q2, a, _ = build(task3, d, variant=var)
        out[name] = (q1, q2, a, must, None)
    out["overloaded"] = (part1, part2, asm, "LOCK_STRENGTH", OVERLOAD_KG)
    return out


VIEW_NOTE = ("A ship in a bottle round a flange and a block, locked: no bolts between the two parts. Grey: the "
             "flange A and the block B. Red, see-through: the closed box, its one opening in the front wall. Teal: "
             "part 2, on the block, which goes in first. Green: part 1, on the flange, which goes in second and waits "
             "turned back about the flange's axis, just off its face; then it is pushed onto the face and turned, "
             "swinging the lug on its wall end down into its place in part 2's wall. Slide to take them out again; "
             "drag to turn, scroll to zoom.")


def write_folder(task: Task, ref, dims: dict, out_dir: str, *, task3: Task, view: bool = True) -> str:
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
        json.dump({"level": LEVEL, "test": "locks", "situation": "flange", "seed": task.seed,
                   "draw": {k: v for k, v in dims["d"].items() if k != "timing_s"}, "lock": ref[2]["lock"],
                   "turn_deg": round(dims["theta"], 3), "push_mm": dims["push"], "wall_gap_mm": dims["butt"]},
                  f, indent=2)
        f.write("\n")
    for name, (q1, q2, a, must, kg) in broken(task3, dims["d"], *ref).items():
        bdir = os.path.join(sol, "broken", name)
        write_answer(bdir, q1, q2, a)
        with open(os.path.join(bdir, "MUST_FAIL"), "w", encoding="utf-8") as f:
            f.write(must + "\n")
        if kg is not None:
            with open(os.path.join(bdir, "LOAD_KG"), "w", encoding="utf-8") as f:
                f.write(f"{kg:g}\n")
    if view:
        write_view(task, ref, os.path.join(sol, "view"))
    return folder


def write_view(task: Task, ref, out: str) -> str:
    """The view: part 2 first, then part 1 along its way in, its push and its turn into the lock."""
    import tempfile

    from ...assemble.generate import write_answer
    from ...assemble.view import write as write_view_

    p1, p2, asm = ref
    lk = asm["lock"]
    still = next(p for p in asm["parts"] if p["name"] != lk["moving"])
    mover = next(p for p in asm["parts"] if p["name"] == lk["moving"])
    shown = {"parts": [dict(still), dict(mover, path=list(mover.get("path") or [])
                                         + [{"move": lk["push"]}, {"turn": lk["turn"]}])],
             # a placeholder joint (the view reads none), its bolts far enough apart to load
             "joint": {"bolt": "M5", "head_on": "part1", "axis": [1, 0, 0], "positions": [[0, 0, 0], [0, 100, 0]]}}
    e1 = np.asarray(task.meta["opening"]["normal"], float)
    with tempfile.TemporaryDirectory() as tmp:
        write_answer(tmp, p1, p2, shown)
        return write_view_(task, tmp, out, heading=f"Flange and block, level 5 (locks): {task.id}", note=VIEW_NOTE,
                           view_dir=_u(e1 + np.array([0.0, 0.0, 0.5])))


def main(argv=None) -> int:
    """python -m jointbench.families.flange.locks5 OUT_DIR [SEED]: the flange setup's level 5 task folder."""
    import sys

    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) not in (1, 2):
        print(main.__doc__)
        return 2
    from .level5 import generate

    t5, draw, ref = generate(int(args[1]) if len(args) > 1 else 0)
    print(write_folder(t5, ref, draw["dims"], args[0], task3=draw["task3"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
