"""Facing mounts, level 4 (assembles): two parts, put in one after the other
through one small opening in a closed box, bolted together inside.

The fixed parts are the facing setup's (``bracket``) for the same seed, as at
levels 1-3: A's face horizontal facing up, B's above it facing down, offset a
little in plan. A closed box stands round both, one small opening in one of
its side walls. Everything of level 3 holds per part (fit, clean, key turn,
printable in PLA) and for the assembly (the stated load, the plastic round
the joint bolts, stiffness); level 4 adds the ways in, the joint (catalogue
through bolts and nuts: the model states size, count and length) and a key
reaching every joint bolt from outside.

Why two parts at all: a one-piece bracket is exactly as tall as the faces are
apart, and each face has to be met by a last short move onto it (over a
spigot, if the face has one). One part cannot move up and down at once; two
can, each onto its own face.

The hidden reference is level 3's C bracket cut in two through its back wall.
In the bracket's own frame (``bracket.layout``: A's face at z = 0, B's at
z = distance, the back wall towards -y), both parts print lying on the back
wall, as at level 3:

* part 1, on A: A's plate, the back wall's inner layer (``y_wall - tj`` to
  ``y_wall``) from the plate up to above the joint, and the lower halves of
  the side walls;
* part 2, on B: B's plate, the back wall's outer layer (``y_wall - 2 tj`` to
  ``y_wall - tj``) from just above A's plate up to B's, the inner layer above
  part 1's (``GAP`` above it), and the upper halves of the side walls.

The two layers lie face to face on the plane ``y = y_wall - tj``: the joint,
bolts along -y through both layers in the gap between the plates, heads on
the outer face (part 2), nuts on the inner face (part 1). Each layer is as
thick as the bolt's length allows.

Both parts come in from behind the back wall, along +y, through the opening
in the box's wall there (its outward normal is -y, the joint keys' line of
sight): part 1 first, ``ONTO`` mm (plus A's spigot) above A's face, then down
onto it; part 2 the same below B's face, then up onto it, its outer layer
sliding up behind part 1's inner layer.
"""
from __future__ import annotations

import json
import math
import os

import build123d as bd
import numpy as np

from ...catalogue import BOLTS, NUTS, lengths
from ...geometry import Placement
from ...task import Obstacle, Task
from . import bracket as Br

LEVEL = 4
WALL = 6.0        # mm, the box's walls
CLEAR = 2.5       # mm, the opening beyond what passes it, each side
ONTO = 2.0        # mm each part comes the last way onto its face (beyond a spigot)
GAP_MIN = 5.0     # mm between part 1's inner layer and part 2's (at least; more than part 2's last move)
RING = 3.0        # mm of plastic round a joint head or nut beyond its diameter
NUT_ROOM = 3.0    # mm a joint nut stands clear of a side wall, beyond its corners
REACH = 6.0       # mm part 2's side walls reach below its plate, at least
MARGIN = 30.0     # mm the box stands beyond what it holds
KEY_ROOM = 5.0    # mm the box stands beyond a fixed bolt's hex key, swung round
JOINT_SIZE, JOINT_LENGTH = "M5", 30.0


def _u(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _r(v):
    return [round(float(x), 6) for x in v]


def _rot(e1, e2, e3):
    m = np.stack([e1, e2, e3], axis=1)
    return tuple(tuple(float(m[i][j]) for j in range(3)) for i in range(3))


def _points(shape, tol=0.2) -> np.ndarray:
    v, _ = shape.tessellate(tol)
    return np.array([tuple(p) for p in v])


def _box(x0, x1, y0, y1, z0, z1) -> bd.Part:
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


# --------------------------------------------------------------------------
# the fixed parts: level 3's task and draw for a seed


def setup(seed: int = 0):
    """(level 3 task, draw): the facing setup's, exactly as level 3 generates it."""
    task, d, _ = Br.generate(seed, 3)
    return task, d


def _spigot(itf) -> float:
    return max([p.height for p in itf.protrusions] or [0.0])


# --------------------------------------------------------------------------
# the reference


def build(task3: Task, d: dict, *, size: str = JOINT_SIZE, length: float = JOINT_LENGTH,
          side: float = Br.SIDE, variant: dict | None = None):
    """(part 1, part 2, assembly file, dimensions), world coordinates. ``size``, ``length``: the joint
    bolts' (the length sets the layers' thickness); ``side``: the side walls' thickness. ``variant``
    makes a broken copy: ``wider`` (mm part 2's wall layer reaches past the bracket each side)."""
    var = dict(variant or {})
    lay = Br.layout(d)
    pl, itf, dist = lay["placements"], lay["itf"], lay["distance"]
    b, nut = BOLTS[size], NUTS[size]
    tj = (length - nut.m - 1.5 * nut.pitch) / 2  # each layer: the bolt stands out of its nut 1.5 pitches
    if not (b.grip_min <= tj <= b.grip_max):
        raise ValueError("joint layers out of range for this bolt length")
    run = Br.STEP / math.tan(math.radians(Br.STEP_SLOPE))
    y_wall = min(Br._outline(itf[n], pl[n], Br.INSET - run - Br.LEDGE).bounding_box().min.Y for n in "AB") - Br.WALL_GAP
    pa, xa = Br._plate(itf["A"], pl["A"], 0.0, 1.0, y_wall)
    pb, xb = Br._plate(itf["B"], pl["B"], dist, -1.0, y_wall)
    x0, x1 = min(xa[0], xb[0]) - Br.WALL_SIDE, max(xa[1], xb[1]) + Br.WALL_SIDE
    heads = [pl[n].point((hx, hy, 0.0))[1] - BOLTS[itf[n].bolt].head_d / 2 for n in "AB" for hx, hy in itf[n].holes]
    y_side = min(heads) - Br.SIDE_ROOM
    if y_side - y_wall < Br.SIDE_MIN:
        raise ValueError("no room for side walls behind the bolts")
    onto_a, onto_b = ONTO + _spigot(itf["A"]), ONTO + _spigot(itf["B"])
    # which part goes in first: the one whose last move onto its face is the longer (over the higher
    # spigot). It keeps the wall's inner layer; the second keeps the outer one and, above (or below) the
    # joint, the inner one too, ``gap`` beyond the first's: its own last move must clear that, so the
    # shorter move sets the gap. Heights below are measured from the first part's face (``Z`` maps them).
    flip = onto_b > onto_a  # B's part goes in first
    first, second = ("part2", "part1") if flip else ("part1", "part2")
    onto_f, onto_s = (onto_b, onto_a) if flip else (onto_a, onto_b)
    plate_f, plate_s = (pb, pa) if flip else (pa, pb)

    def Z(z0, z1):  # heights from the first part's face -> the bracket's z (a box's two ends)
        return (z0, z1) if not flip else (dist - z1, dist - z0)

    gap = max(GAP_MIN, onto_s + 3.0)
    ring = max(b.head_d, nut.e) / 2 + RING
    z_lo, z_hi = Br.STEP, dist - Br.STEP  # the back wall's height, as at level 3
    # the joint: at most half way across, and low enough that the second part's own inner layer and
    # side walls (which start ``gap`` beyond the first's, which reach ``ring`` + 1 past the joint) still
    # reach REACH past its plate (else they would be stubs on the plate, not members joining it to the wall)
    z_top1_max = dist - Br.PLATE - REACH - gap
    z_j = min(dist / 2, z_top1_max - ring - 1.0)
    if z_j - nut.e / 2 < Br.PLATE + 1.0:
        raise ValueError("no room for the joint between the plates")
    z_top1 = z_j + ring + 1.0  # the first part's inner layer and side walls end here
    z_bot2 = onto_s + 1.0      # the second part's outer layer starts here (clear of the first's face even
    # before its last move)
    yi0, yi1 = y_wall - tj, y_wall           # the inner layer
    yo0, yo1 = y_wall - 2 * tj, y_wall - tj  # the outer layer
    wide = float(var.get("wider", 0.0))  # (broken copy: part 2's outer layer wider)
    w2 = wide if second == "part2" else 0.0
    w1 = wide if first == "part2" else 0.0
    pf = plate_f + _box(x0 - w1, x1 + w1, yi0, yi1, *Z(z_lo, z_top1))
    ps = plate_s + _box(x0 - w2, x1 + w2, yo0, yo1, *Z(z_bot2, z_hi)) + _box(x0, x1, yi0, yi1, *Z(z_top1 + gap, z_hi))
    for xs in (x0, x1 - side):
        pf = pf + _box(xs, xs + side, y_wall, y_side, *Z(z_lo, z_top1))
        ps = ps + _box(xs, xs + side, y_wall, y_side, *Z(z_top1 + gap, z_hi))
    # the joint bolts: on the joint face, at z_j, as far apart as the nuts' room beside the side walls allows
    xm = side + nut.e / 2 + NUT_ROOM
    xs_j = [x0 + xm, x1 - xm]
    if xs_j[1] - xs_j[0] < 2 * ring:
        raise ValueError("the bracket is too narrow for two joint bolts")
    zj = Z(z_j, z_j)[0]
    pos = [np.array([x, y_wall - tj, zj]) for x in xs_j]
    for p in pos:
        cut = bd.Plane(origin=tuple(p + np.array([0.0, -tj - 1.0, 0.0])), z_dir=(0.0, 1.0, 0.0)) \
            * bd.Pos(0, 0, tj + 1.0) * bd.Cylinder(b.clear_medium / 2, 2 * tj + 2.0)
        pf, ps = pf - cut, ps - cut
    part1, part2 = (ps, pf) if flip else (pf, ps)
    ctx = {"y_wall": y_wall, "tj": tj, "x": (x0, x1), "y_side": y_side, "z_j": zj, "z_top1": z_top1, "gap": gap,
           "z": (z_lo, z_hi), "z_bot2": z_bot2, "dist": dist, "onto": (onto_a, onto_b), "pos": pos, "first": first}
    s1_, s2_ = part1.clean().solids(), part2.clean().solids()
    if len(s1_) != 1 or len(s2_) != 1:
        raise ValueError(f"a part falls apart ({len(s1_)} and {len(s2_)} pieces)")
    part1, part2 = s1_[0], s2_[0]

    # ---- to the world, and the ways in: both along +y (from behind the back wall), then onto their faces
    G = Br.frame(d)

    def W(v):
        return np.array(G.vec(tuple(float(x) for x in v)))

    def P(v):
        return np.array(G.point(tuple(float(x) for x in v)))

    loc = G.location()
    part1, part2 = part1.moved(loc), part2.moved(loc)
    d_in = W((0.0, 1.0, 0.0))
    move1, move2 = W((0.0, 0.0, -onto_a)), W((0.0, 0.0, onto_b))
    axis = W((0.0, -1.0, 0.0))  # nut side (the first part, the inner layer) into head side (the outer)
    entries = {"part1": {"name": "part1", "file": "part1.step", "bolts_to": "A", "insert_direction": _r(d_in),
                         "path": [{"move": _r(move1)}]},
               "part2": {"name": "part2", "file": "part2.step", "bolts_to": "B", "insert_direction": _r(d_in),
                         "path": [{"move": _r(move2)}]}}
    asm = {"parts": [entries[first], entries[second]],
           "joint": {"bolt": size, "length": float(length), "head_on": second, "axis": _r(axis),
                     "positions": [_r(P(p)) for p in pos]}}
    dims = {"ctx": ctx, "frame": G, "tj": tj, "size": size, "length": length, "d": d, "axis": axis,
            "dir": d_in, "moves": (move1, move2), "pos": [P(p) for p in pos], "ring": ring,
            "up": W((0.0, 0.0, 1.0)), "x": W((1.0, 0.0, 0.0)), "first": first, "second": second}
    return part1, part2, asm, dims


# --------------------------------------------------------------------------
# the box


def box(task: Task, part1, part2, dims: dict, *, starts=None, extra=(), dirs_in=None) -> tuple[list, dict]:
    """The closed box round everything (its walls as obstacles), square to the bracket's frame, and its
    one opening: in the wall behind the bracket's back wall (its outward normal -y in the bracket's
    frame), drawn round where each part crosses that wall on its way in and round the joint keys' line
    of sight. ``starts``: each part where its path begins (default: its place with its path undone);
    ``extra``: more poses inside the box it must hold; ``dirs_in``: each part's way in (default level
    4's)."""
    up, ex = dims["up"], dims["x"]
    e1 = -np.asarray(dims["dir"], float)  # the opening wall's outward normal
    s = np.cross(up, e1)
    m1, m2 = dims["moves"]
    if starts is None:
        starts = (part1.moved(bd.Location(tuple(-m1))), part2.moved(bd.Location(tuple(-m2))))
    dirs_in = dirs_in or (dims["dir"], dims["dir"])
    fixed = task.fixed_solids(0)
    stuff = np.concatenate([_points(x) for x in (part1, part2, *starts, *extra, *fixed.values())] + [_key_swings(task)])
    dirs = {"t+": up, "t-": -up, "e1+": e1, "e1-": -e1, "s+": s, "s-": -s}
    off = {k: float((stuff @ n).max()) + MARGIN for k, n in dirs.items()}
    axes = {"e1": e1, "s": s, "t": up}
    span = {a: (-off[a + "-"], off[a + "+"]) for a in axes}
    walls = []
    for k, n in dirs.items():
        if k == "e1+":
            continue
        a = k[:-1]
        u_, v_ = [o for o in ("e1", "s", "t") if o != a]
        if float(np.cross(axes[u_], axes[v_]) @ n) < 0:
            u_, v_ = v_, u_
        cen = n * (off[k] + WALL / 2) + sum(axes[o] * (span[o][0] + span[o][1]) / 2 for o in (u_, v_))
        size = tuple(float(span[o][1] - span[o][0] + 2 * WALL) for o in (u_, v_)) + (WALL,)
        walls.append(Obstacle({"t+": "wall_top", "t-": "wall_bottom", "e1-": "wall_back", "s+": "wall_side_1",
                               "s-": "wall_side_2"}[k], size,
                              Placement(tuple(float(x) for x in cen), _rot(axes[u_], axes[v_], n))))
    cross = []
    for part, dd in zip(starts, dirs_in):
        P_ = _points(part)
        for plane in (off["e1+"], off["e1+"] + WALL):
            t_ = (plane - P_ @ e1) / float(np.asarray(dd) @ e1)
            cross.append(P_ + np.outer(t_, dd))
    if dims.get("pos"):
        ax = np.asarray(dims["axis"], float)
        b = BOLTS[dims["size"]]
        ka, kb = _u(np.cross(ax, up)), up
        for p in dims["pos"]:
            disc = np.array([p + b.head_d / 2 * (math.cos(a) * ka + math.sin(a) * kb)
                             for a in np.linspace(0, 2 * math.pi, 24)])
            for plane in (off["e1+"], off["e1+"] + WALL):
                t_ = (plane - disc @ e1) / float(ax @ e1)
                cross.append(disc + np.outer(t_, ax))
    cross = np.concatenate(cross)
    oa_ = (float((cross @ s).min()) - CLEAR, float((cross @ s).max()) + CLEAR)
    ot = (float((cross @ up).min()) - CLEAR, float((cross @ up).max()) + CLEAR)
    S0, S1 = span["s"][0] - WALL, span["s"][1] + WALL
    T0, T1 = span["t"][0] - WALL, span["t"][1] + WALL
    if not (S0 < oa_[0] and oa_[1] < S1 and T0 < ot[0] and ot[1] < T1):
        raise ValueError("the opening would be as big as the front wall")
    rot = _rot(s, up, e1)

    def piece(name, a0, a1, z0, z1):
        cen = s * ((a0 + a1) / 2) + up * ((z0 + z1) / 2) + e1 * (off["e1+"] + WALL / 2)
        return Obstacle(name, (float(a1 - a0), float(z1 - z0), WALL), Placement(tuple(float(x) for x in cen), rot))

    walls += [piece("wall_front_below", S0, S1, T0, ot[0]), piece("wall_front_above", S0, S1, ot[1], T1),
              piece("wall_front_left", S0, oa_[0], ot[0], ot[1]), piece("wall_front_right", oa_[1], S1, ot[0], ot[1])]
    opening = {"wall": "wall_front", "normal": _r(e1), "plane": round(off["e1+"], 6), "along": _r(s), "up": _r(up),
               "along_range": [round(oa_[0], 6), round(oa_[1], 6)], "up_range": [round(ot[0], 6), round(ot[1], 6)]}
    return walls, opening


def _key_swings(task: Task) -> np.ndarray:
    """Points round every fixed part's bolt where its hex key's long leg swings (at the head's top, for
    the thickest part a seat may have): the box stands clear of them."""
    from ...catalogue import KEYS

    pts = []
    for b in task.bolts(0):
        ax = _u(b.axis)
        e = _u(np.cross(ax, (1.0, 0.0, 0.0) if abs(ax[0]) < 0.9 else (0.0, 1.0, 0.0)))
        f = np.cross(ax, e)
        c = np.asarray(b.position, float) + ax * (BOLTS[b.size].grip_max + BOLTS[b.size].head_h)
        r = KEYS[b.size].long_leg + KEY_ROOM
        pts += [c + r * (math.cos(a) * e + math.sin(a) * f) for a in np.linspace(0, 2 * math.pi, 36, endpoint=False)]
    return np.array(pts).reshape(-1, 3)


def task4(task3: Task, walls: list, opening: dict) -> Task:
    tk = Task.from_json(json.loads(task3.dumps()))
    tk.root = task3.root
    tk.id = task3.id.replace("-L3-", "-L4-")
    tk.level = LEVEL
    tk.obstacles = [walls]
    tk.tolerances["joint"] = {"length_required": True}
    tk.meta = dict(tk.meta, test="assembles", assembly=True, opening=opening)
    return tk


def make(task3: Task, d: dict, **kw):
    """(level 4 task, (part 1, part 2, assembly), dimensions) for a level 3 facing task and its draw."""
    p1, p2, asm, dims = build(task3, d, **kw)
    walls, opening = box(task3, p1, p2, dims)
    return task4(task3, walls, opening), (p1, p2, asm), dims


# --------------------------------------------------------------------------
# broken copies


WEAK_KG = 5.5  # the small-bolts copy is tested at this load: the reference holds it, the copy does not
WEAK_SIZE = "M3"  # (measured, seeds 0-2: the reference's joint 11-15 % of what its plastic takes per kg, M3's 24-33 %)


def broken(task3: Task, dims: dict, part1, part2, asm: dict) -> dict:
    """name -> (part1, part2, assembly dict, the check that must fail, load in kg or None for the task's own)."""
    swap = json.loads(json.dumps(asm))
    swap["parts"] = swap["parts"][::-1]
    out = {"wrong-order": (part1, part2, swap, "INSERTION", None)}
    heads = json.loads(json.dumps(asm))
    heads["joint"]["head_on"], heads["joint"]["axis"] = dims["first"], _r(-np.asarray(dims["axis"]))
    out["key-cannot-reach"] = (part1, part2, heads, "JOINT_REACH", None)
    short = json.loads(json.dumps(asm))
    short["joint"]["length"] = float(max(x for x in lengths(dims["size"]) if x < dims["length"]))
    out["bolt-too-short"] = (part1, part2, short, "JOINT_LENGTH", None)
    p1, p2, a, _ = build(task3, dims["d"], size=dims["size"], length=dims["length"], variant={"wider": 20.0})
    out["too-big-for-the-opening"] = (p1, p2, a, "INSERTION", None)  # part 2 20 mm wider each side
    # the joint pulls its bolts (the load hangs off the plates), so the plastic under the nuts is what gives:
    # smaller bolts, smaller nuts, less of it
    p1, p2, a, _ = build(task3, dims["d"], size=WEAK_SIZE, length=dims["length"])
    out["small-joint-bolts"] = (p1, p2, a, "JOINT_BOLTS", WEAK_KG)
    return out


# --------------------------------------------------------------------------
# the task folder


VIEW_NOTE = ("A ship in a bottle between two facing plates. Grey: the two fixed parts, A below facing up and B "
             "above facing down. Red, see-through: the closed box, its one opening in the front wall. Green: part 1, "
             "which bolts to A and goes in first; teal: part 2, which bolts to B. They meet in the back wall, bolted "
             "through it from outside. Slide to take them out again the way they went in, last in first out; the "
             "lines are each part's way in (its middle point). Drag to turn, scroll to zoom.")


def write_folder(task: Task, ref, dims: dict, out_dir: str, *, task3: Task, view: bool = True, level: int = LEVEL,
                 brief=None, broken_copies=None, note: str = VIEW_NOTE, reference: dict | None = None) -> str:
    """The task folder the model sees, plus _solution/ (never shown to it): the reference, its broken
    copies (each with the check it must fail, and its own load where it has one) and a view."""
    import shutil

    from ...assemble.brief import SUBMISSION
    from ...assemble.generate import write_answer
    from ...geometry import write_step

    if brief is None:
        from .brief4 import brief
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
    if level == LEVEL:
        with open(os.path.join(folder, "submission.py"), "w", encoding="utf-8", newline="\n") as f:
            f.write(SUBMISSION.replace('"joint": {"bolt": "M4", ', '"joint": {"bolt": "M4", "length": 20, '))
    sol = os.path.join(folder, "_solution")
    write_answer(sol, *ref)
    with open(os.path.join(sol, "reference.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(reference or {"level": LEVEL, "test": "assembles", "situation": "facing", "seed": task.seed,
                                "draw": {k: v for k, v in dims["d"].items() if k != "timing_s"},
                                "joint": {"bolt": dims["size"], "length": dims["length"], "layers": round(dims["tj"], 3)}},
                  f, indent=2)
        f.write("\n")
    copies = broken(task3, dims, *ref) if broken_copies is None else broken_copies
    for name, (q1, q2, a, must, kg) in copies.items():
        bdir = os.path.join(sol, "broken", name)
        write_answer(bdir, q1, q2, a)
        with open(os.path.join(bdir, "MUST_FAIL"), "w", encoding="utf-8") as f:
            f.write(must + "\n")
        if kg is not None:
            with open(os.path.join(bdir, "LOAD_KG"), "w", encoding="utf-8") as f:
                f.write(f"{kg:g}\n")
    if view:
        from ...assemble.view import write as write_view

        e1 = np.asarray(task.meta["opening"]["normal"], float)
        name = {4: "assembles", 5: "locks"}[level]
        write_view(task, sol, os.path.join(sol, "view"), heading=f"Facing mounts, level {level} ({name}): {task.id}",
                   note=note, view_dir=_u(e1 + 0.5 * np.asarray(dims["up"]) + 0.4 * np.asarray(dims["x"])))
    return folder


def write_task(seed: int, out_dir: str) -> str:
    """Generate seed ``seed``'s level 4 task folder in ``out_dir``; one line saying what was written."""
    t3, d = setup(seed)
    t4, ref, dims = make(t3, d)
    folder = write_folder(t4, ref, dims, out_dir, task3=t3)
    return f"wrote {folder}  ({d['summary']}; joint {dims['size']} x {dims['length']:g})"


def main(argv=None) -> int:
    """python -m jointbench.families.facing.assembles OUT_DIR [SEED]: the facing setup's level 4 task folder."""
    import sys

    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) not in (1, 2):
        print(main.__doc__)
        return 2
    print(write_task(int(args[1]) if len(args) > 1 else 0, args[0]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
