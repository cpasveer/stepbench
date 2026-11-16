"""Facing mounts, level 5 (locks): level 4 with the bolted joint between the two
parts replaced by a push-and-turn lock.

Everything of level 4 holds: the closed box round both fixed parts, the one
opening, each part printable and bolted to its fixed part, the ways in. Between
the two parts there are no bolts: inside the box one part is pushed along an
axis and turned about it, and is then locked to the other (free to come apart
before the turn, blocked after it), carrying the stated load through the lock's
plastic (level 4d's checks, ``assemble.locked``).

The hidden reference, in the bracket's own frame (``bracket.layout``: A's face
at z = 0 facing up, B's at z = distance facing down, the back wall towards -y),
is level 3's C bracket cut across its back wall, high up, the two halves apart.
Both parts print lying on their back faces, as at level 3:

* part 1, on A: A's plate, the back wall (``T`` thick) up to ``RISE`` below
  part 2's, and the side walls; on the wall's top, at each end, a pin;
* part 2, on B: B's plate, the back wall's top (the left pin's pocket roofed
  steeper than 45 deg, as a teardrop hole: in part 2's print its front is a
  ceiling open on one side), and a skin behind part 1's wall
  (``GAP`` behind it) down to just above A, on the wall's right half only. The
  pins' tops stand ``BITE`` mm up into grooves cut in part 2's wall from below.

Part 1 is the moving part. It turns about the vertical through ``P``: on A's
face, half way between the pins, on their middle line. Turning, the pins at the
wall's two ends move along it in opposite directions: the left one (-x) from
behind part 2's wall forward into its groove, the right one from in front of
part 2's wall back into its groove, to the skin behind. Locked, each groove
holds its pin up and towards ``P``, and its end ahead of the pin; the two ends
face opposite ways, so part 1 cannot shift any way across, while turning back
(the lock's own way out) frees both. Each groove is open on its outer side (no
thin fin there): towards ``P`` is enough, the two pins pulling opposite ways.
Turned back, the wall's left half stands behind its place and its right half in
front: the skin, on the right half only, is never in the way.

Both parts come in from the side, along the wall (+x in the bracket's frame),
through an opening in the box's -x wall: part 2 first, below B by ``ONTO`` mm
(plus B's spigot), then up onto it; then part 1, lifted by the push and turned
back: its left pin passes nothing (it stops at the wall's left end, behind part
2), its right pin passes in front of part 2's wall, under B's plate. Part 1 is
pushed down onto A, turned, and both are bolted.

A's face must have no spigot (part 1 turns on it about an axis of its own);
B's may.
"""
from __future__ import annotations

import json
import math
import os

import build123d as bd
import numpy as np

from ...task import Task
from . import assembles as A
from . import bracket as Br
from .assembles import _r, _u

LEVEL = 5
T = 12.0          # mm, part 1's back wall, as at level 3
SKIN = 2.0        # mm, part 2's skin behind part 1's wall (the right pin's far end bears on it)
GAP = 0.4         # mm round each pin in its groove, and between part 1's wall and the skin (level 4d's gap)
RISE = 5.8        # mm between part 1's wall top and part 2's wall bottom (more than the 5 mm pull-back)
BITE = 3.0        # mm each pin stands up into its groove
PIN_R = 3.5       # mm, each pin's radius
PIN_FLAT = 1.5    # mm each pin is cut flat on the wall's back face (it prints lying on that face)
PIN_END = 2.5     # mm from the wall's end to its pin
PULL = 5.0        # mm part 1 is pulled back before the turn (level 4d's loose shift) ...
PULL_ROOM = 0.6   # ... and this much more room above the right pin there, under B's plate
BEHIND = 1.0      # mm the left pin waits behind part 2's wall, and the right one in front of it, at the least
OPEN = 0.0        # mm short of the left pin's centre its groove is open forward on its outer side (0: at the
#                   roof's tip, so no thin wedge is left between the opening and the roof)
ROOF = 50.0       # deg from level, the flanks of the left pin's pocket roof (in part 2's print)
SKIN_FROM = 3.0   # mm right of P the skin starts (the wall moves little near P)
LOCKS_TOLERANCES = {  # level 4d's
    "gap_min": 0.3, "play_max": 0.8, "tilt_max_deg": 5.0, "loose_shift": 5.0, "loose_tilt": 15.0,
    "step_max_deg": 2.0, "step_move_max": 1.5,
}
Z = (0.0, 0.0, 1.0)


def _box(x0, x1, y0, y1, z0, z1) -> bd.Part:
    return A._box(x0, x1, y0, y1, z0, z1)


def _sector(P, r0, r1, z0, z1, a0, sweep):
    """The block r0..r1 from the vertical through P (in plan), z0..z1, from a0 deg (from +x towards +y)
    on for ``sweep`` deg."""
    a = math.radians(a0)
    u = (math.cos(a), math.sin(a), 0.0)
    pl = bd.Plane(origin=(P[0], P[1], 0.0), x_dir=u, z_dir=tuple(np.cross(u, Z)))
    sec = bd.Polygon((r0, z0), (r1, z0), (r1, z1), (r0, z1), align=None)
    return bd.revolve(pl * sec, bd.Axis((P[0], P[1], 0.0), Z), sweep)


def _turn(shape, P, deg):
    return shape.rotate(bd.Axis((P[0], P[1], 0.0), Z), deg)


def _rot(xy, P, deg) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    v = np.asarray(xy, float) - np.asarray(P[:2])
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]]) + np.asarray(P[:2])


def _pin(cx, cy, r, y_flat, z0, z1) -> bd.Part:
    """A round pin, upright, cut flat at y_flat (behind it): a D in plan."""
    pin = bd.Pos(cx, cy, (z0 + z1) / 2) * bd.Cylinder(r, z1 - z0)
    return pin & _box(cx - r - 1, cx + r + 1, y_flat, cy + r + 1, z0 - 1, z1 + 1)


def build(task3: Task, d: dict, *, variant: dict | None = None):
    """(part 1, part 2, assembly file, dimensions), world coordinates. ``variant`` makes the broken
    copies: ``no_groove`` (the pins meet part 2's wall: the turn clashes), ``open_top`` (the grooves cut
    up through the wall: pulled back after the turn the pins come out), ``shut_at_start`` (part 2's wall
    goes on down in front over the right pin's start: it is caught before the turn), ``tight`` (0.1 mm
    round the pins), ``deeper`` (part 2's wall that much thicker behind: too big for the opening)."""
    var = dict(variant or {})
    lay = Br.layout(d)
    pl, itf, dist = lay["placements"], lay["itf"], lay["distance"]
    if itf["A"].protrusions:
        raise ValueError("level 5 of the facing setup needs A's face flat (part 1 turns on it)")
    g = 0.1 if var.get("tight") else GAP
    run = Br.STEP / math.tan(math.radians(Br.STEP_SLOPE))
    y_wall = min(Br._outline(itf[n], pl[n], Br.INSET - run - Br.LEDGE).bounding_box().min.Y for n in "AB") - Br.WALL_GAP
    pa, xa = Br._plate(itf["A"], pl["A"], 0.0, 1.0, y_wall)
    pb, xb = Br._plate(itf["B"], pl["B"], dist, -1.0, y_wall)
    x0, x1 = min(xa[0], xb[0]) - Br.WALL_SIDE, max(xa[1], xb[1]) + Br.WALL_SIDE
    heads = [pl[n].point((hx, hy, 0.0))[1] - Br.BOLTS[itf[n].bolt].head_d / 2 for n in "AB" for hx, hy in itf[n].holes]
    y_side = min(heads) - Br.SIDE_ROOM
    if y_side - y_wall < Br.SIDE_MIN:
        raise ValueError("no room for side walls behind the bolts")
    onto_a, onto_b = A.ONTO + A._spigot(itf["A"]), A.ONTO + A._spigot(itf["B"])
    yb1 = y_wall - T                 # part 1's wall's back face (its bed)
    yb2 = yb1 - GAP - SKIN           # part 2's
    # the heights: the pins' tops as high as B's plate allows (the right pin, pulled back before the turn,
    # still clears it), part 2's wall from z_l, part 1's wall top z_w
    z_lo, z_hi = Br.STEP, dist - Br.STEP
    z_pin = dist - Br.PLATE - PULL - g - PULL_ROOM
    z_l = z_pin - BITE
    z_w = z_l - RISE
    if z_w < Br.PLATE + 6.0 or z_pin + onto_a >= dist - Br.PLATE:
        raise ValueError("the faces are too close for the lock")
    # the pins and the turn
    cy = yb1 + PIN_R - PIN_FLAT
    cl, cr = x0 + PIN_END + PIN_R, x1 - PIN_END - PIN_R
    P = np.array([(cl + cr) / 2, cy, 0.0])  # on A's face, between the pins, on their middle line
    theta = None
    for deg in np.arange(3.0, 60.0, 0.25):  # the least turn back that takes the left pin behind part 2's wall
        # and the right one in front of it (turned back by +deg)
        yl = _rot((cl, cy), P, deg)[1]
        yr = _rot((cr, cy), P, deg)[1]
        if yl + PIN_R + g + BEHIND <= yb2 and yr - PIN_R - g - BEHIND >= y_wall:
            theta = float(deg)
            break
    if theta is None:
        raise ValueError("the bracket is too short for the lock's turn")
    pins = _pin(cl, cy, PIN_R, yb1, z_w - 1.0, z_pin) + _pin(cr, cy, PIN_R, yb1, z_w - 1.0, z_pin)
    # part 1: A's plate, the wall, the side walls, the pins
    part1 = pa + _box(x0, x1, yb1, y_wall, z_lo, z_w) + pins
    for xs in (x0, x1 - Br.SIDE):
        part1 = part1 + _box(xs, xs + Br.SIDE, y_wall, y_side, z_lo, z_w)
    # part 2: B's plate, the wall's top from z_l, the skin behind part 1's wall on its right half
    deep = float(var.get("deeper", 0.0))  # broken copy: part 2's wall that much thicker behind
    part2 = pb + _box(x0, x1, yb2 - deep, y_wall, z_l, z_hi)
    part2 = part2 + _box(P[0] + SKIN_FROM, x1, yb2, yb1 - GAP, onto_b + 1.0, z_l + 1.0)
    if var.get("shut_at_start"):  # broken copy: part 2's wall on down in front, over the right pin's start
        part2 = part2 + _box(cr - 15.0, x1 + 5.0, y_wall, y_wall + 15.0, z_pin + g + 1.0, z_hi)
    # the grooves: each pin (set out by g) where it is locked, where it starts, and the ring between,
    # open on the outer side; the left one open forward there too, short of the pin's centre
    top = z_pin + g if not var.get("open_top") else z_hi + 1.0
    cuts = []
    for cx, a_here in ((cl, 180.0), (cr, 0.0)):
        L = abs(cx - P[0])
        here = _pin(cx, cy, PIN_R + g, yb1 - g, z_l - 1.0, top)
        if cx == cl:  # its front is a roof in part 2's print, open on its outer side: a teardrop roof, its
            # flanks ROOF deg from level (steeper than 45: no sliver of it read as needing support)
            R_, a = PIN_R + g, math.radians(ROOF)
            kx, ky, tip = R_ * math.sin(a), R_ * math.cos(a), R_ / math.cos(a)
            roof = bd.Face(bd.Wire.make_polygon([(cx - kx, cy + ky, z_l - 1.0), (cx, cy + tip, z_l - 1.0),
                                                 (cx + kx, cy + ky, z_l - 1.0), (cx, cy, z_l - 1.0)], close=True))
            here = here + bd.extrude(roof, amount=top - z_l + 1.0, dir=Z)
        sx, sy = _rot((cx, cy), P, theta)
        start = bd.Pos(sx, sy, (z_l - 1.0 + top) / 2) * bd.Cylinder(PIN_R + g, top - z_l + 1.0)
        ring = _sector(P, L - PIN_R - g, L + 60.0, z_l - 1.0, top, a_here, theta)
        cuts += [here, start, ring]
    cuts.append(_box(x0 - 1.0, cl - OPEN, cy, y_wall + 1.0, z_l - 1.0, top))
    if not var.get("no_groove"):
        for c in cuts:
            part2 = part2 - c
    ctx = {"y_wall": y_wall, "x": (x0, x1), "y_side": y_side, "z_w": z_w, "z_l": z_l, "z_pin": z_pin,
           "P": P, "theta": theta, "dist": dist, "onto": (onto_a, onto_b), "g": g, "pins": (cl, cr, cy)}
    s1_, s2_ = part1.clean().solids(), part2.clean().solids()
    if len(s1_) != 1 or len(s2_) != 1:
        raise ValueError(f"a part falls apart ({len(s1_)} and {len(s2_)} pieces)")
    part1, part2 = s1_[0], s2_[0]

    # ---- to the world: the lock, and the ways in (both along +x, from the box's -x wall)
    G = Br.frame(d)

    def W(v):
        return np.array(G.vec(tuple(float(x) for x in v)))

    def Pw(v):
        return np.array(G.point(tuple(float(x) for x in v)))

    from ...moves import motion as MO

    loc = G.location()
    part1, part2 = part1.moved(loc), part2.moved(loc)
    up, ex = W(Z), W((1.0, 0.0, 0.0))
    Pw_ = Pw(P)
    push = -up * onto_a
    lock = {"moving": "part1", "push": _r(push), "turn": {"point": _r(Pw_), "axis": _r(up), "degrees": round(-theta, 6)}}
    back = MO.about(Pw_, up, theta).then(MO.shift(-push))  # final -> where part 1 waits
    waiting = part1.moved(back.location())
    pushed = part1.moved(MO.about(Pw_, up, theta).location())
    d_in = ex
    asm = {"parts": [
        {"name": "part2", "file": "part2.step", "bolts_to": "B", "insert_direction": _r(d_in),
         "path": [{"move": _r(up * onto_b)}]},
        {"name": "part1", "file": "part1.step", "bolts_to": "A", "insert_direction": _r(d_in)}],
        "lock": lock}
    start2 = part2.moved(bd.Location(tuple(-up * onto_b)))
    dims = {"ctx": ctx, "frame": G, "d": d, "dir": d_in, "up": up, "x": W((0.0, 1.0, 0.0)), "lock": lock,
            "theta": theta, "starts": (waiting, start2), "waiting": waiting, "pushed": pushed,
            "moves": (np.zeros(3), up * onto_b), "pos": [], "size": None}
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


def make(task3: Task, d: dict, **kw):
    """(level 5 task, (part 1, part 2, assembly), dimensions) for a level 3 facing task and its draw."""
    p1, p2, asm, dims = build(task3, d, **kw)
    walls, opening = A.box(task3, p1, p2, dims, starts=dims["starts"], extra=(dims["pushed"],),
                           dirs_in=(dims["dir"], dims["dir"]))
    return task5(task3, walls, opening), (p1, p2, asm), dims


# --------------------------------------------------------------------------
# broken copies, the task folder, the view


OVERLOAD_KG = 500.0  # the overloaded copy's pull: far past what the pins' plastic takes


def broken(task3: Task, dims: dict, part1, part2, asm: dict) -> dict:
    """name -> (part 1, part 2, assembly dict, the check that must fail, load in kg or None)."""
    out = {}
    for name, var, must in (("collides-while-turning", {"no_groove": True}, "TURN"),
                            ("pulls-out-after-turn", {"open_top": True}, "LOCKED"),
                            ("locked-before-turn", {"shut_at_start": True}, "UNLOCKS"),
                            ("gap-too-small", {"tight": True}, "GAP"),
                            ("too-big-for-the-opening", {"deeper": 20.0}, "INSERTION")):
        q1, q2, a, _ = build(task3, dims["d"], variant=var)
        out[name] = (q1, q2, a, must, None)
    out["overloaded"] = (part1, part2, asm, "LOCK_STRENGTH", OVERLOAD_KG)
    return out


VIEW_NOTE = ("A ship in a bottle between two facing plates, locked: no bolts between the two parts. Grey: the "
             "two fixed parts, A below facing up and B above facing down. Red, see-through: the closed box, its one "
             "opening in the side wall. Teal: part 2, on B, which goes in first; green: part 1, on A, which comes in "
             "turned back, is pushed down onto A and turns on it, swinging the pins on its wall's two ends into the "
             "grooves under part 2's wall. Slide to take them out again; drag to turn, scroll to zoom.")


def write_folder(task: Task, ref, dims: dict, out_dir: str, *, task3: Task, view: bool = True) -> str:
    from ...assemble.brief import SUBMISSION
    from .brief5 import brief

    folder = A.write_folder(task, ref, dims, out_dir, task3=task3, view=False, level=LEVEL, brief=brief,
                            broken_copies=broken(task3, dims, *ref),
                            reference={"level": LEVEL, "test": "locks", "situation": "facing", "seed": task.seed,
                                       "draw": {k: v for k, v in dims["d"].items() if k != "timing_s"},
                                       "lock": ref[2]["lock"], "turn_deg": round(dims["theta"], 3)})
    sub = SUBMISSION.replace(
        '"joint": {"bolt": "M4", "head_on": "part1", "axis": [0, 0, -1], "positions": [[0, -10, 20], [0, 10, 20]]},',
        '"lock": {"moving": "part2", "push": [0, 0, 5], "turn": {"point": [0, 0, 0], "axis": [0, 0, 1], "degrees": 30}},')
    with open(os.path.join(folder, "submission.py"), "w", encoding="utf-8", newline="\n") as f:
        f.write(sub)
    if view:
        write_view(task, ref, dims, os.path.join(folder, "_solution", "view"))
    return folder


def write_view(task: Task, ref, dims: dict, out: str) -> str:
    """The view: part 2 first, then part 1 along its way in, its push and its turn into the lock."""
    import tempfile

    from ...assemble.generate import write_answer
    from ...assemble.view import write as write_view_

    p1, p2, asm = ref
    lk = asm["lock"]
    pw = lk["turn"]
    shown = {"parts": [dict(asm["parts"][0]), dict(asm["parts"][1], path=[{"move": lk["push"]}, {"turn": pw}])],
             "joint": {"bolt": "M5", "head_on": "part1", "axis": [1, 0, 0],  # (the view reads no joint)
                       "positions": [[0, 0, 0], [0, 100, 0]]}}
    e1 = np.asarray(task.meta["opening"]["normal"], float)
    with tempfile.TemporaryDirectory() as tmp:
        write_answer(tmp, p1, p2, shown)
        return write_view_(task, tmp, out, heading=f"Facing mounts, level 5 (locks): {task.id}", note=VIEW_NOTE,
                           view_dir=_u(e1 + 0.5 * np.asarray(dims["up"]) - 0.6 * np.asarray(dims["x"])))


def write_task(seed: int, out_dir: str) -> str:
    """Generate seed ``seed``'s level 5 task folder in ``out_dir``; one line saying what was written."""
    t3, d = A.setup(seed)
    t5, ref, dims = make(t3, d)
    folder = write_folder(t5, ref, dims, out_dir, task3=t3)
    return f"wrote {folder}  ({d['summary']}; lock: turn {dims['theta']:.1f} deg)"


def main(argv=None) -> int:
    """python -m jointbench.families.facing.locks5 OUT_DIR [SEED]: the facing setup's level 5 task folder."""
    import sys

    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) not in (1, 2):
        print(main.__doc__)
        return 2
    print(write_task(int(args[1]) if len(args) > 1 else 0, args[0]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
