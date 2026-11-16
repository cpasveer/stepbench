"""The assembles level, facing mounts: a ship in a bottle.

The situation: fixed part A faces up from the floor at one end of a closed
box; fixed part B faces down from the ceiling at the other end. The box --
fixed walls, floor and ceiling round the space between the two mounts --
has ONE small opening, in the end wall on B's side. Every part goes in
through it, one after the other; they are joined inside; and the joint's
bolts must be reachable by a key through it.

Why it takes thought, not a cut: the opening is barely bigger than each
half of the connection and much lower than the box, so nothing that reaches
from A's mount up to B's fits through it. The part for A must go in first
and travel to the far end (once the part for B is in, the way there is
blocked). The joint must face the opening, or no key reaches its bolts. And
the plastic round the bolts must carry the load.

Answer-first, as every level: the generator builds a two-part reference
(each part a stepped profile, printable on its side):

  part 1: a plate on A and an upright wall at its end nearest the opening;
          in through the opening, along the box, down onto A;
  part 2: a plate under B, reaching back to a flange that hangs down
          against part 1's upright; in through the opening second, until
          the flange meets the upright, then up onto B.

The two bolt through upright and flange, the heads on the flange facing the
opening. The box, the opening and the heights are then placed round that,
and the draw is kept only if the reference passes the grader in full
(load, plastic round the bolts, key reach). A draw whose reference fails is
thrown away and the next draw from the same seeded stream is used.
"""
from __future__ import annotations

import json
import os
import random
import shutil
import tempfile

import build123d as bd

from ..catalogue import BOLTS, GENERATOR_VERSION, NUTS, edge_margin, make_interface
from ..geometry import Placement, write_step
from ..task import DEFAULT_TOLERANCES, Obstacle, Task

LEVEL = 4  # the number the grader works with (checks from level 3 apply); shown as 4a, "assembles"
LABEL = "4a"
NAME = "assembles"
FAMILY = "assemble"
SITUATION = (
    "Two parts are fixed in space, their mounting faces parallel and facing each other, one low at one "
    "end, the other high at the other end. Round the space between them is a closed box -- fixed walls, "
    "floor and ceiling -- with one small opening in one end wall. Everything you put in must go in "
    "through that opening, and every bolt that joins your parts must be reachable through it."
)
TOLERANCES = dict(DEFAULT_TOLERANCES, tool_length=30.0, tool_diameter_extra=0.0, seat_setback=1.0,
                  seat_overhang_max=0.5, manufacturing={"process": "fdm", "quantity": 1, "direction": None},
                  required_checks=["SEAT_INSIDE", "CLEAN", "MANUFACTURING", "STIFFNESS", "KEY_TURN"],
                  score="material")  # the printed weight of the two parts: lighter is better
SIZES = ("M3", "M4", "M5", "M6")
JOINT_BOLTS_N = 2  # the reference's joint: two bolts
PITCHES = (25, 30, 40, 50)
UP = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
DOWN = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
THICKNESS = (8.0, 12.0, 16.0, 18.0, 20.0, 22.0, 24.0)  # the reference's walls and plates, tried until stiff enough
WALL = 6.0  # mm, the box's walls, floor and ceiling
RAISE = 1.0  # mm: where a part's plate runs past its mount's face, it stands this far off the face's plane


def _mount(rng: random.Random):
    """A square mount, or a rectangular one with two columns of bolts."""
    size = rng.choice(SIZES)
    b = BOLTS[size]
    ps = [p for p in PITCHES if p >= b.head_d + 14]
    if rng.random() < 0.5:
        return make_interface("square", {"bolt": size, "pitch": rng.choice(ps)})
    return make_interface("rect", {"bolt": size, "nx": 2, "ny": rng.choice([2, 3]),
                                   "pitch_x": rng.choice(ps), "pitch_y": rng.choice([p for p in PITCHES if p >= b.head_d + 6])})


def _span(itf):
    xs = [h[0] for h in itf.holes]
    ys = [h[1] for h in itf.holes]
    return max(xs), max(ys)  # patterns are centred


def _draw(rng: random.Random) -> dict:
    a, b = _mount(rng), _mount(rng)
    return {"A": {"kind": a.kind, "params": a.params}, "B": {"kind": b.kind, "params": b.params},
            "clear": rng.choice([2.0, 2.5, 3.0]),  # opening beyond the bigger part, each side: a tight fit
            "sill": rng.choice([1.0, 2.0, 3.0]),  # the opening's bottom edge above the floor
            "joint_lift": rng.choice([1.0, 3.0, 5.0]),  # the joint above the least height its nuts allow
            "head_room": rng.choice([1.0, 3.0, 5.0]),  # above the joint heads, below B's tool reach
            "room_x": rng.choice([4.0, 8.0, 12.0]),  # box beyond the mounts, at either end
            "room_y": rng.choice([4.0, 8.0, 12.0]),  # box beyond the widest of mounts and parts, each side
            "mass": None}


def layout(d: dict, T: float, joint_bolt: str | None = None) -> dict:
    """Every world dimension, from the draw, the reference's upright/flange
    thickness ``T`` and the joint bolt size. A at the origin, facing up; B
    to the -x side, facing down; the opening in the box's -x end wall."""
    A, B = make_interface(d["A"]["kind"], d["A"]["params"]), make_interface(d["B"]["kind"], d["B"]["params"])
    ba, bb = BOLTS[A.bolt], BOLTS[B.bolt]
    jb = joint_bolt or d.get("joint_bolt") or "M5"
    jbolt, jnut = BOLTS[jb], NUTS[jb]
    tool = TOLERANCES["tool_length"]
    tp_a = max(6.0, ba.grip_min, T)  # the plates as thick as the walls: the turning load bends them most
    tp_b = max(6.0, bb.grip_min, T)
    ax, ay = _span(A)
    bx, by = _span(B)
    pa = (ax + edge_margin(ba) - 1.5, ay + edge_margin(ba) - 1.5)  # seats: 1.5 mm inside each face
    pb = (bx + edge_margin(bb) - 1.5, by + edge_margin(bb) - 1.5)
    # part 1's upright: just clear (1 mm) of the reach a key needs above A's bolts on the -x side
    xf = -(ax + ba.head_d / 2 + 1.0 + T)  # the joint face: the upright's -x face
    # part 2's flange against it; B's bolts on the +x side 1 mm clear of the flange
    xb = xf - T - 1.0 - bb.head_d / 2 - bx
    ring = max(jbolt.head_d, jnut.e) / 2 + 3.0  # each bolt's ring of plastic round it, on upright and flange
    nut_r = jnut.e / 2 + DEFAULT_TOLERANCES["head_clearance_radial"]
    head_r = jbolt.head_d / 2 + DEFAULT_TOLERANCES["head_clearance_radial"]
    # the nuts sit behind the upright, over A's -x bolts: above the 30 mm a key needs there
    zj = tp_a + ba.head_h + tool + nut_r + 0.5 + d["joint_lift"]
    # the heads face the opening, under B's +x bolts: below the 30 mm a key needs there
    H = zj + head_r + 0.5 + tool + bb.head_h + tp_b + d["head_room"]
    z_top1, z_bot2 = zj + ring, zj - ring
    h1, h2 = z_top1, H - z_bot2
    w1, w2 = 2 * pa[1], 2 * pb[1]
    half = min(pa[1], pb[1])  # the joint's width: both parts centred on y = 0
    jy = half - ring
    c, sill = d["clear"], d["sill"]
    Wo = max(w1, w2) + 2 * c
    Ho = max(h1, h2) + 2 * c
    x0 = xb - B.face_w / 2 - d["room_x"]  # the opening wall's inside face
    x1 = A.face_w / 2 + d["room_x"]
    Y = max(A.face_h, B.face_h, w1, w2, Wo) / 2 + d["room_y"]
    return {"A": A, "B": B, "tp_a": tp_a, "tp_b": tp_b, "pa": pa, "pb": pb, "T": T, "xf": xf, "xb": xb, "zj": zj,
            "H": H, "z_top1": z_top1, "z_bot2": z_bot2, "h1": h1, "h2": h2, "joint_bolt": jb, "joint_y": (-jy, jy),
            "ring": ring, "x0": x0, "x1": x1, "Y": Y, "Wo": Wo, "Ho": Ho, "sill": sill, "clear": c,
            # the reference's ways in: part 1 at the opening's height, then down; part 2 the same, then up
            "lift1": sill + c, "lift2": H - h2 - sill - c,
            "ok": jy >= jbolt.head_d / 2 + 0.5 and sill + Ho < H - WALL and H - h2 - sill - c > 0}


def _box(name, lo, hi):
    lo, hi = [float(v) for v in lo], [float(v) for v in hi]
    c = tuple((a + b) / 2 for a, b in zip(lo, hi))
    return Obstacle(name, tuple(round(b - a, 6) for a, b in zip(lo, hi)), Placement(tuple(round(v, 6) for v in c), UP))


def box_walls(L: dict) -> list:
    """The closed box: floor and ceiling (each with the mount's face left
    open, the mount filling it), the two side walls, the far end wall, and
    the near end wall round its one opening. Boxes, as every obstacle."""
    A, B = L["A"], L["B"]
    x0, x1, Y, H, w = L["x0"], L["x1"], L["Y"], L["H"], WALL
    xb = L["xb"]
    out = []

    def slab(prefix, cx, fw, fh, z0, z1):  # from x0 to x1, y -Y..Y, with the face fw x fh (centred cx, 0) left open
        out.append(_box(f"{prefix}_west", (x0, -Y, z0), (cx - fw / 2, Y, z1)))
        out.append(_box(f"{prefix}_east", (cx + fw / 2, -Y, z0), (x1, Y, z1)))
        out.append(_box(f"{prefix}_south", (cx - fw / 2, -Y, z0), (cx + fw / 2, -fh / 2, z1)))
        out.append(_box(f"{prefix}_north", (cx - fw / 2, fh / 2, z0), (cx + fw / 2, Y, z1)))

    slab("floor", 0.0, A.face_w, A.face_h, -w, 0.0)
    slab("ceiling", xb, B.face_w, B.face_h, H, H + w)
    out.append(_box("wall_south", (x0 - w, -Y - w, -w), (x1 + w, -Y, H + w)))
    out.append(_box("wall_north", (x0 - w, Y, -w), (x1 + w, Y + w, H + w)))
    out.append(_box("wall_far", (x1, -Y, -w), (x1 + w, Y, H + w)))
    s, Wo, Ho = L["sill"], L["Wo"], L["Ho"]
    out.append(_box("wall_near_below", (x0 - w, -Y, -w), (x0, Y, s)))
    out.append(_box("wall_near_above", (x0 - w, -Y, s + Ho), (x0, Y, H + w)))
    out.append(_box("wall_near_south", (x0 - w, -Y, s), (x0, -Wo / 2, s + Ho)))
    out.append(_box("wall_near_north", (x0 - w, Wo / 2, s), (x0, Y, s + Ho)))
    return out


def task_from(seed: int, d: dict, L: dict) -> Task:
    parts = {"A": L["A"], "B": L["B"]}
    pose = {"A": Placement((0.0, 0.0, 0.0), UP), "B": Placement((L["xb"], 0.0, L["H"]), DOWN)}
    t = Task(f"L4a-s{seed}", LEVEL, seed, parts, [pose], json.loads(json.dumps(TOLERANCES)),
             {"generator": "assemble.level4", "level_label": LABEL, "level_name": NAME, "knobs": {},
              "opening": {"wall_x": round(L["x0"], 6), "y": [round(-L["Wo"] / 2, 6), round(L["Wo"] / 2, 6)],
                          "z": [L["sill"], round(L["sill"] + L["Ho"], 6)]}},
             [box_walls(L)], GENERATOR_VERSION, family=FAMILY)
    from ..requirements import apply_stated_load

    apply_stated_load(t, {"B": d["mass"]})
    # the joint bolts allowed: no bigger than the smallest size the reference's joint holds with, and no more
    jb = L["joint_bolt"]
    t.tolerances["joint"] = {"sizes": [s for s in SIZES if BOLTS[s].d <= BOLTS[jb].d], "max_bolts": JOINT_BOLTS_N}
    return t


# --------------------------------------------------------------------------
# the reference: two stepped profiles, each printable lying on its side


def _profile(points_xz, y0: float, y1: float) -> bd.Part:
    pts = [(x, y1, z) for x, z in points_xz]
    face = bd.Face(bd.Wire.make_polygon(pts, close=True))
    return bd.extrude(face, amount=y1 - y0, dir=(0, -1, 0))


def build_reference(task: Task, L: dict, *, joint_shift: float = 0.0, tall: float = 0.0,
                    heads_inward: bool = False, thin_edge: bool = False) -> tuple[bd.Part, bd.Part, dict]:
    """Part 1 (plate on A, upright wall at its -x end), part 2 (plate under
    B reaching to a flange hanging against the upright) and the assembly
    file. Keyword options make the deliberately broken copies:
    ``joint_shift`` moves part 2's joint holes along y; ``tall`` raises part
    1's upright by that much; ``heads_inward`` puts the heads on part 1
    (facing the far end); ``thin_edge`` makes part 1's upright as thin as its
    bolts allow, with two M3 bolts 1 mm of plastic from its side edge."""
    T, xf, xb, H = L["T"], L["xf"], L["xb"], L["H"]
    pa, pb, zj = L["pa"], L["pb"], L["zj"]
    jb = "M3" if thin_edge else L["joint_bolt"]
    jbb = BOLTS[jb]
    t1 = jbb.grip_min if thin_edge else T  # part 1's upright thickness, from the joint face
    zt1 = L["z_top1"] + tall
    p1 = [(xf, RAISE), (-pa[0], RAISE), (-pa[0], 0.0), (pa[0], 0.0), (pa[0], L["tp_a"]),
          (xf + t1, L["tp_a"]), (xf + t1, zt1), (xf, zt1)]
    p2 = [(xb - pb[0], H), (xb + pb[0], H), (xb + pb[0], H - RAISE), (xf, H - RAISE), (xf, L["z_bot2"]),
          (xf - T, L["z_bot2"]), (xf - T, H - L["tp_b"]), (xb - pb[0], H - L["tp_b"])]
    part1 = _profile(p1, -pa[1], pa[1])
    part2 = _profile(p2, -pb[1], pb[1])
    for b in task.bolts(0):
        bb = BOLTS[b.size]
        x, y, z = b.position
        if b.part == "A":
            part1 = part1 - bd.Pos(x, y, L["tp_a"] / 2) * bd.Cylinder(bb.clear_medium / 2, L["tp_a"] + 2)
        else:
            part2 = part2 - bd.Pos(x, y, H - L["tp_b"] / 2) * bd.Cylinder(bb.clear_medium / 2, L["tp_b"] + 2)
    if thin_edge:
        ye = pa[1] - jbb.clear_medium / 2 - 1.0  # 1 mm of plastic between the holes and the upright's side
        dz = jbb.head_d / 2 + 0.25
        pos = [[xf, ye, zj - dz], [xf, ye, zj + dz]]
    else:
        pos = [[xf, y, zj] for y in L["joint_y"]]
    along = bd.Rot(0, 90, 0)  # cylinders along x
    for x, y, z in pos:
        part1 = part1 - bd.Pos(xf + t1 / 2, y, z) * along * bd.Cylinder(jbb.clear_medium / 2, t1 + 2)
        part2 = part2 - bd.Pos(xf - T / 2, y + joint_shift, z) * along * bd.Cylinder(jbb.clear_medium / 2, T + 2)
    part1, part2 = part1.solids()[0], part2.solids()[0]
    asm = {"parts": [{"name": "part1", "file": "part1.step", "bolts_to": "A", "insert_direction": [1.0, 0.0, 0.0],
                      "path": [{"move": [0.0, 0.0, -round(L["lift1"], 6)]}]},
                     {"name": "part2", "file": "part2.step", "bolts_to": "B", "insert_direction": [1.0, 0.0, 0.0],
                      "path": [{"move": [0.0, 0.0, round(L["lift2"], 6)]}]}],
           "joint": {"bolt": jb, "head_on": "part1" if heads_inward else "part2",
                     "axis": [1.0, 0.0, 0.0] if heads_inward else [-1.0, 0.0, 0.0], "positions": pos}}
    return part1, part2, asm


def write_answer(folder: str, part1, part2, asm: dict) -> str:
    os.makedirs(folder, exist_ok=True)
    write_step(part1, os.path.join(folder, "part1.step"))
    write_step(part2, os.path.join(folder, "part2.step"))
    p = os.path.join(folder, "assembly.json")
    with open(p, "w", encoding="utf-8", newline="\n") as f:
        json.dump(asm, f, indent=2)
        f.write("\n")
    return p


def _smallest_joint(seed: int, d: dict, progress=None) -> str | None:
    """The smallest joint bolt size with which the reference's joint holds
    the stated load (graded in full but for the load analysis, at the first
    thickness); None if no size works or the reference fails otherwise."""
    from .grade import grade

    for jb in SIZES:
        L = layout(d, THICKNESS[0], jb)
        if not L["ok"]:
            return None
        task = task_from(seed, d, L)
        with tempfile.TemporaryDirectory() as tmp:
            rep = grade(task, write_answer(tmp, *build_reference(task, L)), stiffness=False)
        failed = set(rep.failed())
        if progress:
            progress(f"joint {jb}: {'holds' if not failed else 'fails ' + ', '.join(sorted(failed))}")
        if not failed:
            return jb
        if failed != {"JOINT_BOLTS"}:
            return None  # not the bolts: a bigger size will not help
    return None


def generate(seed: int, knobs: dict | None = None, max_tries: int = 30, progress=None):
    """Return (task, draw, (part1, part2, assembly)). Deterministic in ``seed``."""
    from ..geometry import BUILD_ROOM, fits_build_volume
    from ..requirements import draw_stated_load
    from .grade import grade

    rng = random.Random(f"L4-{seed}")  # unchanged since before the 4a label: the same draws
    for _ in range(max_tries):
        d = _draw(rng)
        d["mass"] = draw_stated_load(rng)
        d["joint_bolt"] = _smallest_joint(seed, d, progress)
        if d["joint_bolt"] is None:
            continue
        for T in THICKNESS:
            L = layout(d, T)
            if not L["ok"]:
                break
            task = task_from(seed, d, L)
            p1, p2, asm = build_reference(task, L)
            if not (fits_build_volume(p1, BUILD_ROOM) and fits_build_volume(p2, BUILD_ROOM)):
                break
            with tempfile.TemporaryDirectory() as tmp:
                rep = grade(task, write_answer(tmp, p1, p2, asm))
            if progress:
                progress(f"draw T={T:g}: {'pass' if rep.passed else 'fail: ' + ', '.join(rep.failed())}")
            st = next((c for c in rep.checks if c.name == "STIFFNESS"), None)
            if rep.passed and (st.details or {}).get("verdict") == "marginal":
                if progress:
                    progress(f"draw T={T:g}: stiffness only marginal (within the checker's precision): thicker")
                continue  # no knife-edge task: the reference must pass clearly
            if rep.passed:
                d["T"] = T
                d["summary"] = (f"A {L['A'].description}; B {L['B'].description}; {d['mass']:g} kg on B; "
                                f"opening {L['Wo']:.1f} x {L['Ho']:.1f} mm in a box {L['H']:.1f} mm high; "
                                f"joint {len(asm['joint']['positions'])} x {asm['joint']['bolt']}")
                return task, d, (p1, p2, asm)
            if set(rep.failed()) - {"STIFFNESS", "JOINT_BOLTS"}:
                break  # thicker will not cure a fit failure: next draw
    raise RuntimeError(f"seed {seed}: no valid level 4a (assembles) draw in {max_tries} tries")


# --------------------------------------------------------------------------
# deliberately broken copies of the reference, each failing one way


def broken(task: Task, d: dict) -> dict:
    """name -> (part1, part2, assembly dict, the check that must fail, load in kg or None for the task's own)."""
    L = layout(d, d["T"])
    out = {}
    p1, p2, asm = build_reference(task, L)
    a = json.loads(json.dumps(asm))
    a["parts"] = [a["parts"][1], a["parts"][0]]  # part 2 first: then part 1 runs into it on its way to A
    out["wrong-order"] = (p1, p2, a, "INSERTION", None)
    q1, q2, a = build_reference(task, L, tall=L["Ho"] - L["h1"] + 5.0)  # part 1 too tall for the opening
    out["too-big-for-the-opening"] = (q1, q2, a, "INSERTION", None)
    q1, q2, a = build_reference(task, L, heads_inward=True)  # heads face the far end: no key gets to them
    out["key-cannot-reach"] = (q1, q2, a, "JOINT_REACH", None)
    q1, q2, a = build_reference(task, L, joint_shift=1.0)
    out["holes-do-not-line-up"] = (q1, q2, a, "part2 HOLES", None)
    q1, q2, a = build_reference(task, L, thin_edge=True)  # the plastic gives way: thin, holes at its edge
    out["thin-flange-edge-holes"] = (q1, q2, a, "JOINT_BOLTS", None)
    return out


def with_load(task: Task, kg: float) -> Task:
    """A copy of the task stating ``kg`` on B instead (for a broken copy only)."""
    from ..requirements import apply_stated_load

    t = Task.from_json(json.loads(task.dumps()))
    t.root = task.root
    apply_stated_load(t, {"B": kg})
    return t


def write_folder(task: Task, d: dict, ref, out_dir: str) -> str:
    """The task folder the model sees, plus _solution/ (never shown to it)."""
    from .brief import brief, SUBMISSION

    folder = os.path.join(out_dir, task.id)
    if os.path.exists(folder):
        shutil.rmtree(folder)
    os.makedirs(folder)
    with open(os.path.join(folder, "task.json"), "w", encoding="utf-8", newline="\n") as f:
        f.write(task.dumps())
    for n, s in task.fixed_solids(0).items():
        write_step(s, os.path.join(folder, f"fixed_{n}.step"))
    write_step(bd.Compound([o.solid() for o in task.obstacles_at(0)]), os.path.join(folder, "obstacles.step"))
    with open(os.path.join(folder, "README.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write(brief(task))
    with open(os.path.join(folder, "submission.py"), "w", encoding="utf-8", newline="\n") as f:
        f.write(SUBMISSION)
    sol = os.path.join(folder, "_solution")
    write_answer(sol, *ref)
    with open(os.path.join(sol, "reference.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump({"level": LEVEL, "seed": task.seed, "draw": d}, f, indent=2, default=str)
        f.write("\n")
    for name, (q1, q2, a, must, kg) in broken(task, d).items():
        bdir = os.path.join(sol, "broken", name)
        write_answer(bdir, q1, q2, a)
        with open(os.path.join(bdir, "MUST_FAIL"), "w", encoding="utf-8") as f:
            f.write(must + "\n")
        if kg is not None:
            with open(os.path.join(bdir, "LOAD_KG"), "w", encoding="utf-8") as f:
                f.write(f"{kg:g}\n")
    return folder
