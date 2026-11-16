"""The facing situation's first generator (a shell reference, faces near
vertical). No longer used by the facing levels, which use ``bracket`` (the
round 5 look: faces horizontal, one above the other); kept because the pipe
situation borrows its draw constants and frame.

Facing mounts, levels 1-3: two fixed parts whose mounting faces are
parallel and face each other; one solid part bolts to both.

The three levels are one situation with requirements added:

* level 1, fits -- holes, seating, no interference, the seat inside each
  face, every bolt turnable with its hex key, cleanly modelled;
* level 2, printable -- level 1, printed in PLA on a filament printer without
  support (the stated process's rules);
* level 3, strong -- level 2, and B carries a stated load in PLA; the
  printed weight is the score.

One seed gives the same fixed parts at every level (the draw's random stream
does not depend on the level), so the levels really build on each other.

Generation works backwards with the adapts family's shell (see
``families/adapt/angled.py``): in the part's own frame a faceted shell, open
underneath, printing standing on its rim, with a pad on two opposite walls
(the mounts), two roof walls and a few closing walls in random directions. The
bolt heads sit inside, reached through the open underside. The fixtures are
placed on the pads; the whole is then moved to a random place and turned
about the vertical (not tilted: the printing check tries the axes as print
directions, and the shell prints on its rim). A draw is kept only if the
reference passes everything of level 3 -- the strictest -- so every level's
task is solvable by the same reference.
"""
from __future__ import annotations

import os
import random
import tempfile

from ...catalogue import BOLTS, GENERATOR_VERSION, sample_interface
from ...geometry import BUILD_ROOM, Placement, fits_build_volume, rot_axis_angle, write_step
from ...requirements import apply_stated_load, draw_stated_load
from ...task import DEFAULT_TOLERANCES, Task
from ..adapt import angled as A

KNOBS_DEFAULT: dict = {}
SITUATION = (
    "Two parts are fixed in space. Their mounting faces are parallel and face\n"
    "each other, with nothing in between; the arrangement stands at an arbitrary\n"
    "position, turned arbitrarily about the vertical."
)
NZ_FACING = 0.20  # |z| of the mount normals: both a mount and its opposite must stay within MOUNT_NZ
ROOF_NZ = (0.40, 0.66)   # the roof walls' upward tilt (|z| of the normal; at most NZ_MAX 0.68)
ROOF_TURN = 20.0         # deg a roof wall may turn from square to the mounts, in plan
LOADED = "B"      # A is held, B carries the stated load
REQUIRED = {
    1: ["SEAT_INSIDE", "CLEAN", "KEY_TURN"],
    2: ["SEAT_INSIDE", "CLEAN", "KEY_TURN", "MANUFACTURING"],
    3: ["SEAT_INSIDE", "CLEAN", "KEY_TURN", "MANUFACTURING", "STIFFNESS"],
}
NAMES = {1: "fits", 2: "printable", 3: "strong"}


def _draw(rng: random.Random) -> dict:
    itf = {}
    for n in "AB":
        while True:  # small enough for a wall of a part that fits the printer, and printable
            i = sample_interface(rng)
            if i.bolt in A.BOLTS_OK and A._pattern_radius(i) + BOLTS[i.bolt].head_d / 2 <= A.MAX_PATTERN \
                    and all(2 * p.radius + 1.0 <= A.BORE_MAX for p in i.protrusions):
                break
        itf[n] = i
    d = {n: {"kind": itf[n].kind, "params": itf[n].params, "yaw": 5.0 * rng.randrange(72)} for n in "AB"}
    d["mounts"], d["pairs"] = "AB", ("AB",)
    while True:  # near level, so both the mount and the opposite one face no more than a little up or down
        na = A._rand_unit(rng)
        if abs(na[2]) <= NZ_FACING:
            break
    d["normals"] = {"A": list(na), "B": list(A._neg(na))}
    # the roof: two side walls, square to the mounts in plan, tilted up, meeting like a ridge between the
    # two mount walls (with only two mounts, random walls alone rarely close the shell)
    import math

    h = math.hypot(na[0], na[1])
    side = (-na[1] / h, na[0] / h)
    closing = []
    for s in (1.0, -1.0):
        turn = math.radians(rng.uniform(-ROOF_TURN, ROOF_TURN))
        c, sn = math.cos(turn), math.sin(turn)
        px, py = s * (c * side[0] - sn * side[1]), s * (sn * side[0] + c * side[1])
        nz = rng.uniform(*ROOF_NZ)
        k = math.sqrt(1.0 - nz * nz)
        closing.append(A._unit((k * px, k * py, nz)))
    for count, (lo, hi) in ((rng.randint(*A.SKIRT), A.SKIRT_NZ), (rng.randint(0, 1), (-A.NZ_MAX, A.NZ_MAX))):
        got = 0
        for _ in range(60):
            if got >= count:
                break
            v = A._rand_unit(rng)
            if lo <= v[2] <= hi and all(A._angle(v, w) >= A.MIN_SEP for w in [na, A._neg(na)] + closing):
                closing.append(v)
                got += 1
    d["closing"] = [list(v) for v in closing]
    d["depth"] = {n: A._half(rng.uniform(*A.DEPTH)) for n in "AB"}
    d["closing_depth"] = [A._half(rng.uniform(*A.DEPTH)) for _ in closing]
    d["bed_depth"] = A._half(rng.uniform(*A.BED_DEPTH))
    d["shift"] = {n: [0.0, 0.0] for n in "AB"}
    d["load_kg"] = draw_stated_load(rng)
    # a random place and a random turn about the vertical, but no tilt: the part prints standing on its
    # rim, and the printing check always tries the axes as print directions (not a narrow rim face)
    d["frame"] = {"yaw": round(rng.uniform(0.0, 360.0), 1),
                  "origin": [A._half(rng.uniform(-150.0, 150.0)) for _ in range(3)]}
    return d


def frame(d: dict) -> Placement:
    f = d["frame"]
    return Placement(tuple(f["origin"]), rot_axis_angle((0.0, 0.0, 1.0), f["yaw"]))


def task_from_draw(seed: int, d: dict, lay: dict, level: int) -> Task:
    g = frame(d)
    pose = {n: lay["placements"][n].then(g) for n in "AB"}
    tol = dict(DEFAULT_TOLERANCES, seat_setback=1.0, seat_overhang_max=0.5, tool_length=30.0, tool_diameter_extra=0.0,
               manufacturing={"process": "fdm", "quantity": 1, "direction": None},
               required_checks=list(REQUIRED[level]))
    meta = {"generator": "facing-shell", "situation": "facing", "test": NAMES[level]}
    task = Task(f"facing-L{level}-s{seed}", level, seed, dict(lay["itf"]), [pose], tol, meta,
                generator_version=GENERATOR_VERSION, family="facing")
    if level >= 3:
        apply_stated_load(task, {LOADED: d["load_kg"]})
        task.tolerances["score"] = "material"
    return task


def _frame_ok(task: Task) -> bool:
    return all(max(abs(v) for v in f.normal) <= A.AXIS_MAX for f in task.faces(0))


def generate(seed: int, level: int, max_tries: int = 400, tally: dict | None = None):
    """Return (task, draw, reference solid in world coordinates) for ``level``
    (1, 2 or 3). Deterministic in ``seed``; the same draw at every level."""
    from ...grade import grade

    tally = tally if tally is not None else {}

    def no(why):
        tally[why] = tally.get(why, 0) + 1

    rng = random.Random(f"facing-{seed}")
    for _ in range(max_tries):
        d = _draw(rng)
        try:
            lay = A.layout(d)
        except A.Reject as e:
            no(str(e))
            continue
        strict = task_from_draw(seed, d, lay, 3)
        for _ in range(30):  # turn the frame again until no face is near a coordinate plane
            if _frame_ok(strict):
                break
            no("a face near a coordinate plane (frame redrawn)")
            d["frame"]["yaw"] = round(rng.uniform(0.0, 360.0), 1)
            strict = task_from_draw(seed, d, lay, 3)
        if not _frame_ok(strict):
            continue
        try:
            ref = A.build_reference(lay)
        except ValueError:
            no("reference could not be built")
            continue
        if len(ref.solids()) != 1 or not fits_build_volume(ref, BUILD_ROOM):
            no("reference not one solid or too big for the printer")
            continue
        ref = ref.moved(frame(d).location())
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "reference.step")
            write_step(ref, p)
            rep = grade(strict, p, optional=False, score=False, skip=frozenset({"STIFFNESS"}))  # cheap checks first
            if rep.passed:
                rep = grade(strict, p, optional=False, score=False,
                            skip=frozenset({"CLEAN", "MANUFACTURING", "SEAT_INSIDE"}))
        if not rep.passed:
            no("reference fails: " + ", ".join(sorted(set(rep.failed()))))
            continue
        st = next((c for c in rep.checks if c.name == "STIFFNESS"), None)
        if st is not None and (st.details or {}).get("verdict") == "marginal":
            no("stiffness only marginal")
            continue  # no knife-edge task
        import math
        d["distance"] = round(math.dist(lay["placements"]["A"].origin, lay["placements"]["B"].origin), 3)
        d["summary"] = (f"A {lay['itf']['A'].description}; B {lay['itf']['B'].description}; "
                        f"{d['distance']:.0f} mm apart" + (f"; {d['load_kg']:g} kg on B" if level >= 3 else ""))
        return task_from_draw(seed, d, lay, level), d, ref
    raise RuntimeError(f"seed {seed}: no valid facing draw in {max_tries} tries ({tally})")
