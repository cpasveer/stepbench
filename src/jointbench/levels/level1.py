"""Level 1: two fixed parts, faces parallel and facing each other, nothing in
the way.

Generation is answer-first. We first decide the connector -- two plates,
one on each mounting face, joined by a wall standing beside the bolts -- and
only then place the fixed parts where that connector fits them. The gap is
chosen so that the bolt heads of one side cannot reach the plate of the
other. The reference connector is then graded by the same grader an agent
faces; a draw whose reference fails is discarded and the next draw (from the
same seeded stream) is used. So every published task is solvable, and the
grader has passed at least one answer for it.

Level 1 allows, as variation:
  * the two interfaces (type, bolt size, pitch),
  * the gap between the faces,
  * a lateral offset of B relative to A, up to 20 mm in X and Y,
  * a rotation of each part about its own face normal (multiples of 5 deg).
It does not allow tilt (level 2) or anything between the parts (level 3).
"""
from __future__ import annotations

import math
import os
import random
import tempfile

import build123d as bd

from ..catalogue import GENERATOR_VERSION, BOLTS, Interface, edge_margin, sample_interface
from ..geometry import Placement, write_step
from ..task import DEFAULT_TOLERANCES, Task

LEVEL = 1
KNOBS_DEFAULT = {"process": "pbf", "stiffness": 0.0}

SITUATION = (
    "Two parts are fixed in space. Their mounting faces are parallel and face\n"
    "each other, with nothing in between."
)
MAX_OFFSET = 20.0
SPINE_W = 8.0


def _draw(rng: random.Random) -> dict:
    a, b = sample_interface(rng), sample_interface(rng)
    ba, bb = BOLTS[a.bolt], BOLTS[b.bolt]
    t_choices = [4.0, 5.0, 6.0, 8.0, 10.0]
    ta = rng.choice([t for t in t_choices if t >= ba.grip_min])
    tb = rng.choice([t for t in t_choices if t >= bb.grip_min])
    ca = DEFAULT_TOLERANCES["head_clearance_axial"]
    lower = ta + tb + ba.head_h + bb.head_h + 2 * ca + 6.0
    tallest = max([pr.height for pr in a.protrusions + b.protrusions] or [0.0])
    if tallest:
        lower = max(lower, tallest + ta + tb + 4.0)
    return {
        "A": {"kind": a.kind, "params": a.params, "yaw": 5.0 * rng.randrange(72)},
        "B": {"kind": b.kind, "params": b.params, "yaw": 5.0 * rng.randrange(72)},
        "gap": round(lower + rng.uniform(0.0, 35.0) * 2) / 2,
        "offset": [round(rng.uniform(-MAX_OFFSET, MAX_OFFSET) * 2) / 2 for _ in range(2)],
        "t": {"A": ta, "B": tb},
    }


def task_from_draw(seed: int, d: dict) -> Task:
    from ..catalogue import make_interface

    parts = {n: make_interface(d[n]["kind"], d[n]["params"]) for n in ("A", "B")}
    pose = {
        "A": Placement.from_yaw_flip((0.0, 0.0, 0.0), d["A"]["yaw"], flip=False),
        "B": Placement.from_yaw_flip((d["offset"][0], d["offset"][1], d["gap"]), d["B"]["yaw"], flip=True),
    }
    return Task(f"L1-s{seed}", LEVEL, seed, parts, [pose], dict(DEFAULT_TOLERANCES),
                {"generator": "level1"}, generator_version=GENERATOR_VERSION)


# --------------------------------------------------------------------------
# the reference connector, and deliberately broken copies of it


def _rect(points, margin):
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    return [min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin]


def _union_rect(r, s):
    return [min(r[0], s[0]), min(r[1], s[1]), max(r[2], s[2]), max(r[3], s[3])]


def _plate(r, z0, t, rounded: float = 0.0):
    w, h = r[2] - r[0], r[3] - r[1]
    sk = bd.RectangleRounded(w, h, rounded) if rounded > 0 else bd.Rectangle(w, h)
    return bd.Pos((r[0] + r[2]) / 2, (r[1] + r[3]) / 2, z0) * bd.extrude(sk, t)


def build_connector(task: Task, d: dict, *, style: str = "reference", hole_shift: float = 0.0,
                    hole_d: float | None = None, push: float = 0.0, lift: float = 0.0,
                    member_area: float | None = None, thin_plates: bool = False,
                    fin: bool = False, spread: float = 1.0, shape: str | None = None,
                    chords: int = 4, lean_plates: bool = False, wrap: float | None = None) -> bd.Part:
    """The reference: a plate on each face, joined by a truss (see
    ``levels.truss``): four round chords standing square to both plates at
    the corners of a rectangle drawn round both bolt patterns, with
    diagonals in each side panel when the gap is long enough to need them.
    Spreading the chords round the bolts is what makes it stiff: a load on
    one mount pulls and pushes the chords instead of bending them.

    style      "reference" or "alt" (chords 3 mm further out, thicker plates
               and chords): same interfaces, different body.
    hole_shift move A's holes this far along world X (mm).
    hole_d     drill A's holes at this diameter instead of the medium series.
    push       move the whole part this far into fixed part A (mm).
    lift       raise A's plate this far off A's mounting face (mm).
    member_area  chords sized to exactly this section in total (mm2)
    thin_plates  plates only as thick as the bolts allow (grip minimum)
    fin        add a 0.4 mm fin to plate A: fits, but cannot be made
    spread     scale the chord rectangle about its centre (1 = round the bolts)
    shape      one of ``levels.weak``'s shapes instead (fits, not stiff)
    chords     4 (every corner) or 2 (two opposite corners): par's leaner option
    lean_plates  cut the plates' middle out, inside the bolt pattern (par)
    wrap       par's dial, 0..1: 1 puts the four chords round the bolts
               (stiff); smaller slides the two -X chords along the outside of
               the bolt rectangle towards the +X pair (0: all four on +X)
    """
    if shape:
        from . import weak

        far = {f.part: bd.Vector(*f.origin) + bd.Vector(200.0, 0.0, 0.0) for f in task.faces(0)}
        return weak.build(task, shape, far, lift=lift)
    from ..checks.section_min import required_area
    from . import rods as R
    from . import truss as T

    alt = style == "alt"
    gap = d["gap"]
    bolts = task.bolts(0)
    pa = [b.position for b in bolts if b.part == "A"]
    pb = [b.position for b in bolts if b.part == "B"]
    ba, bb = BOLTS[task.parts["A"].bolt], BOLTS[task.parts["B"].bolt]
    need = 1.25 * required_area(task, "A", "B")
    area = member_area if member_area is not None else need * (1.21 if alt else 1.0)
    # an exact member_area (self-test, par) is built as asked; the
    # reference's own chords are never thinner than R_MIN
    r = math.sqrt(area / chords / math.pi)
    r = min(R.R_MAX, max(1.0, r) if member_area is not None else max(R.R_MIN, r))
    ta = max(R.PLATE_T, ba.grip_min) + (1.0 if alt else 0.0)
    tb = max(R.PLATE_T, bb.grip_min) + (1.0 if alt else 0.0)
    if thin_plates:  # as thin as the bolts allow, but never under a third of the rod (thickness transition)
        ta, tb = max(ba.grip_min, 2 * r / 3 + 0.1), max(bb.grip_min, 2 * r / 3 + 0.1)
    ra, rb = _rect(pa, edge_margin(ba)), _rect(pb, edge_margin(bb))
    both = _union_rect(ra, rb)
    off = r + 2.0 + (3.0 if alt else 0.0)  # chords clear of every bolt ring
    cx, cy = (both[0] + both[2]) / 2, (both[1] + both[3]) / 2
    hx, hy = (both[2] - both[0]) / 2 + off, (both[3] - both[1]) / 2 + off
    hx, hy = max(hx * spread, r + 1.0), max(hy * spread, r + 1.0)
    nr = [cx - hx, cy - hy, cx + hx, cy + hy]
    if wrap is not None:  # the two -X chords slide along the outside edges towards +X
        w = 2 * r + R.GAP
        f = min(1.0, max(0.0, wrap))
        nr = [nr[2] - w - (nr[2] - w - nr[0]) * f, nr[1], nr[2], nr[3]]
        hx, hy = (nr[2] - nr[0]) / 2, (nr[3] - nr[1]) / 2
    pr = _union_rect([nr[0] - r - 3, nr[1] - r - 3, nr[2] + r + 3, nr[3] + r + 3], both)

    def flat_ok(r_, t_):  # a long thin plate warps when made: keep its diagonal under 38 x thickness
        return min(8.0, max(t_, math.hypot(r_[2] - r_[0], r_[3] - r_[1]) / 38.0))

    ta, tb = flat_ok(pr, ta), flat_ok(pr, tb)
    body = _plate(pr, lift, ta) + _plate(pr, gap - tb, tb)
    if lean_plates:
        # material inside the bolt pattern carries little: leave a frame
        # one edge margin wide inside the bolt rings
        for rr, bt, z0, t_ in ((ra, ba, lift, ta), (rb, bb, gap - tb, tb)):
            e = edge_margin(bt)
            hole = [rr[0] + 2 * e, rr[1] + 2 * e, rr[2] - 2 * e, rr[3] - 2 * e]
            if hole[2] - hole[0] > 4.0 and hole[3] - hole[1] > 4.0:
                body = body - _plate(hole, z0 - 1.0, t_ + 2.0)
    xy = bd.Plane.XY
    na = T.corners(xy, *nr, lift + ta / 2)
    nb = T.corners(xy, *nr, gap - tb / 2)
    if chords == 2:
        na, nb = [na[0], na[2]], [nb[0], nb[2]]
    elif chords != 4:
        raise ValueError("chords is 4 or 2")
    lay = T.layout(na, nb, bd.Vector(0, 0, 1), bd.Vector(0, 0, -1), ta / 2 + T.STUB, tb / 2 + T.STUB,
                   min(2 * hx, 2 * hy) if chords == 4 else 1e9)  # two chords: no diagonals
    rd = max(T.R_DIAG_MIN, 0.7 * r)  # thinner than the chord: the joint keeps an open wedge
    lay = T.prune(lay, r, rd)
    d.setdefault("truss", {}).update(bays=lay["bays"], diagonals=len(lay["diagonals"]))
    body = body + T.solid(lay, r, rd)
    if fin:
        body = body + bd.Pos(pr[0] - 3.0, (pr[1] + pr[3]) / 2, lift + ta / 2) * bd.Box(8.0, 8.0, 0.4)
    # clearance holes
    for p in pa:
        dia = hole_d if hole_d is not None else ba.clear_medium
        h = ta + 2.0
        body = body - bd.Pos(p[0] + hole_shift, p[1], lift + ta + 1.0 - h / 2) * bd.Cylinder(dia / 2, h)
    for p in pb:
        h = tb + 2.0
        body = body - bd.Pos(p[0], p[1], gap - tb - 1.0 + h / 2) * bd.Cylinder(bb.clear_medium / 2, h)
    # room for anything the fixed parts push into connector space
    for name, z0, sgn, t_ in (("A", 0.0, 1.0, ta + lift), ("B", gap, -1.0, tb)):
        itf: Interface = task.parts[name]
        pl = task.poses[0][name]
        for prt in itf.protrusions:
            c = pl.point((prt.x, prt.y, 0.0))
            h = prt.height + 1.0
            if t_ - h < 2.0:
                h = t_ + 2.0  # a pocket would leave too thin a skin: go right through
            body = body - bd.Pos(c[0], c[1], z0 + sgn * h / 2) * bd.Cylinder(prt.radius + 0.5, h + 0.02)
    if push:
        body = bd.Pos(0, 0, -push) * body
    solids = body.solids()
    return solids[0] if len(solids) == 1 else body


# --------------------------------------------------------------------------


def generate(seed: int, knobs: dict | None = None, max_tries: int = 50):
    """Return (task, draw, reference solid). Deterministic in ``seed``."""
    from ..grade import grade

    rng = random.Random(f"L1-{seed}")  # name-free: a rename keeps every task
    for _ in range(max_tries):
        d = _draw(rng)
        task = task_from_draw(seed, d)
        from ..requirements import apply_process

        process = (knobs or {}).get("process", KNOBS_DEFAULT["process"])
        apply_process(task, process)  # the reference must be makeable by it, or the draw is rejected
        from ..requirements import apply_key_turn

        apply_key_turn(task)  # every bolt turnable with its hex key, or the draw is rejected
        if process != KNOBS_DEFAULT["process"]:
            task.id += f"-{process}"
        stiff = float((knobs or {}).get("stiffness", KNOBS_DEFAULT["stiffness"]))
        if stiff not in (0.0, 1.0):
            raise ValueError("knob stiffness is 1 (required, from the task's bolts and span) or 0 (off)")
        if stiff:
            from ..requirements import apply_stiffness

            apply_stiffness(task)
            task.meta = dict(task.meta, knobs=dict(task.meta.get("knobs", {}), stiffness=stiff))
        if stiff != KNOBS_DEFAULT["stiffness"]:
            task.id += "-stiff" if stiff else "-nostiff"
        fx = task.fixed_solids(0)
        if fx["A"].distance_to(fx["B"]) < 1.0:
            continue
        ref = build_connector(task, d)
        if len(ref.solids()) != 1:
            continue
        from ..geometry import BUILD_ROOM, fits_build_volume

        if not fits_build_volume(ref, BUILD_ROOM):
            continue  # the reference does not fit a Prusa MINI with room to spare
        # keep the draw only if the reference AND the alternative body pass:
        # the alternative's plates are 1 mm thicker, so a task where 1 mm
        # decides between pass and fail (a knife-edge task) is thrown away
        ok = True
        with tempfile.TemporaryDirectory() as tmp:
            for part, skip in ((ref, frozenset()), (build_connector(task, d, style="alt"), frozenset({"MANUFACTURING", "STIFFNESS"}))):
                p = os.path.join(tmp, "candidate.step")
                write_step(part, p)
                rep = grade(task, p, optional=False, score=False, skip=skip)
                if not rep.passed:
                    ok = False
                    break
                st = next((c for c in rep.checks if c.name == "STIFFNESS" and c.ok), None)
                if st is not None and not skip:
                    d["reference_stiffness_ratio"] = st.value  # weakest mount's figure / requirement
        if ok:
            from ..requirements import par as par_for

            task.par = par_for(task, d, build_connector)
            d["summary"] = f"gap {d['gap']:g} mm" + ("" if task.par else ", no par")
            return task, d, ref
    raise RuntimeError(f"seed {seed}: no valid level-1 draw in {max_tries} tries")


# --------------------------------------------------------------------------
# what selftest does to the reference: (label, variant kwargs, check that
# must fail -- or None if the variant must pass)


def _undersized(task):
    return {"hole_d": BOLTS[task.parts["A"].bolt].clear_fine - 0.2}


SELFTEST_VARIANTS = [
    ("reference answer", lambda t: {}, None),
    ("different body, same interfaces", lambda t: {"style": "alt"}, None),
    ("holes shifted 0.05 mm (inside tol)", lambda t: {"hole_shift": 0.05}, None),
    ("holes shifted 0.25 mm", lambda t: {"hole_shift": 0.25}, "HOLES"),
    ("holes shifted 0.5 mm", lambda t: {"hole_shift": 0.5}, "HOLES"),
    ("holes shifted 1 mm", lambda t: {"hole_shift": 1.0}, "HOLES"),
    ("holes shifted 2 mm", lambda t: {"hole_shift": 2.0}, "HOLES"),
    ("holes 0.2 mm under clearance", _undersized, "HOLES"),
    ("body pushed 1 mm into part A", lambda t: {"push": 1.0}, "NO_INTERFERENCE"),
    ("seat on A lifted 1 mm", lambda t: {"lift": 1.0}, "SEATING"),
]


def _need(task):
    from ..checks.section_min import required_area

    names = list(task.parts)
    return max(required_area(task, a, b) for i, a in enumerate(names) for b in names[i + 1:])


SELFTEST_VARIANTS += [
    # a sound member, well clear of the limit, must pass (at 1.1 x the variant's
    # plates or slanted segments can themselves be the thinnest section)
    ("member at 1.5 x the required section", lambda t: {"member_area": 1.5 * _need(t)}, ("ok", "SECTION_MIN")),
    ("member at 0.9 x the required section", lambda t: {"member_area": 0.9 * _need(t)}, "SECTION_MIN"),
    ("0.4 mm fin: fits, cannot be made", lambda t: {"fin": True}, "*_WALL_MIN"),  # the task's process
]


def PAR_CANDIDATES(task):
    """Lean answers par is chosen from (the smallest volume that passes
    every required check wins): plates at minimum grip, with and without
    their middle cut out, chords at 1.15 x the required section, all on one
    side of the bolts (wrap 0), half way round (0.5) or round them (1)."""
    import math

    need = _need(task)
    area = max(1.15 * need, 4 * math.pi * R_MIN_CHORD ** 2)  # chords never thinner than the reference's floor
    return [{"member_area": round(area, 3), "thin_plates": True, "lean_plates": lean, "wrap": w}
            for w in (0.0, 0.5, 1.0) for lean in (True, False)]


R_MIN_CHORD = 3.0  # mm, as levels.rods.R_MIN


# stiffness rows (they run the stiffness analysis; see selftest): the lean
# answer par was taken from must pass, last round's slender shapes must fail
def _par_build(task):
    return dict((task.par or {}).get("build") or {"thin_plates": True})


SELFTEST_VARIANTS.append(("lean answer par was taken from (just stiff enough)", _par_build, None))
from .weak import SELFTEST_ROWS as _WEAK  # noqa: E402

SELFTEST_VARIANTS += [(label, (lambda s: lambda t: {"shape": s})(shape), "STIFFNESS") for label, shape in _WEAK]
