"""Facing mounts, levels 1-3: two fixed parts, one above the other, whose
mounting faces are horizontal and face each other; one solid part bolts to
both.

The three levels are one situation with requirements added:

* level 1, fits -- holes, seating, no interference, the seat inside each
  face, every bolt turnable with its hex key, cleanly modelled;
* level 2, printable -- level 1, printed in PLA on a filament printer without
  support (the stated process's rules);
* level 3, strong -- level 2, and B carries a stated load in PLA; the
  printed weight is the score.

One seed gives the same fixed parts at every level (the draw's random stream
does not depend on the level), so the levels really build on each other.
Seed 0 is the showcase: round 5's "two facing mounts" (a square mount
facing up, a round flange with a centre spigot facing down onto it, offset a
little), at a distance a hex key can work in.

The hidden reference is a C-shaped bracket: a plate on A, a plate under B,
one wall behind joining them. The bolt heads sit in the open space between
the plates; their keys come in from the open front and sides. It prints
lying on its wall (the wall's outer face on the bed): the plates then stand
up as fins, every bolt hole runs sideways and is narrower than the widest
hole a printer bridges, a wider cut for a fixture's spigot is a teardrop
pointing away from the bed, and each plate's seat steps down to the face
on slopes no steeper than printing allows. A draw is kept only if the
reference passes everything of level 3 -- the strictest -- so every level's
task is solvable by the same reference.
"""
from __future__ import annotations

import math
import os
import random
import tempfile

import build123d as bd

from ...catalogue import BOLTS, GENERATOR_VERSION, KEYS, rect_mount, round_mount, sample_interface, square_mount
from ...geometry import BUILD_ROOM, Placement, fits_build_volume, rot_axis_angle, write_step
from ...requirements import apply_stated_load, draw_stated_load
from ...task import DEFAULT_TOLERANCES, Task

KNOBS_DEFAULT: dict = {}
SITUATION = (
    "Two parts are fixed in space, one above the other. Their mounting faces are\n"
    "horizontal and face each other, with nothing in between; the arrangement stands\n"
    "at an arbitrary position, turned arbitrarily about the vertical."
)
LOADED = "B"      # A is held, B carries the stated load
REQUIRED = {
    1: ["SEAT_INSIDE", "CLEAN", "KEY_TURN"],
    2: ["SEAT_INSIDE", "CLEAN", "KEY_TURN", "MANUFACTURING"],
    3: ["SEAT_INSIDE", "CLEAN", "KEY_TURN", "MANUFACTURING", "STIFFNESS"],
}
NAMES = {1: "fits", 2: "printable", 3: "strong"}

# the draw
OFFSET_MAX = 20.0        # mm the two fixtures' centres lie apart in plan, at most
GAP_EXTRA = (2.0, 14.0)  # mm of distance between the faces beyond what the keys need
FACE_MAX = 90.0          # mm, largest fixture face drawn (the bracket must fit the printer)

# the reference
PLATE = 14.0             # mm, each plate's thickness: the bolts' grip (at least every size's grip_min)
WALL = 12.0              # mm, the wall behind
INSET = 1.5              # mm the seat stays inside a fixture's face (setback 1.0 + 0.5 to spare)
STEP = 2.0               # mm the plate steps up from its seat, beyond the seat's outline
STEP_SLOPE = 30.0        # deg the step's face rises from the face's plane (printing: its normal at most 45 deg off level)
LEDGE = 2.0              # mm of flat between the step's top and the plate's edge (no edges meeting at a tangent)
WALL_GAP = 2.0           # mm between the wall and the nearest plate outline
WALL_SIDE = 2.0          # mm the wall reaches past the wider plate on each side
SIDE = 7.0               # mm, each side wall's thickness
SIDE_MIN = 6.0           # mm, the shortest side wall (from the back wall forward) worth having
SIDE_ROOM = 2.0          # mm the side walls stop short of the hindmost bolt head
BORE_MAX = 10.0          # mm, widest sideways hole printed without a teardrop (fdm.toml)
BORE_ROOM = 0.5          # mm a cut stands off a fixture's spigot
RING_ROOM = 1.0          # mm a spigot's cut stays clear of a bolt head's ring


class Reject(ValueError):
    """The draw cannot be used; the reason says why (for the generator's tally)."""


def _half(x: float) -> float:
    return round(x * 2) / 2


def _key_room(itf) -> float:
    """Height above a plate a bolt's head and its hex key take: the head, then the key's short leg."""
    return BOLTS[itf.bolt].head_h + KEYS[itf.bolt].short_leg


def _need(itf: dict) -> float:
    """The least distance between the faces: both plates, and for each side the head and key under
    the other side's heads."""
    a, b = itf["A"], itf["B"]
    return 2 * PLATE + max(_key_room(a) + BOLTS[b.bolt].head_h, _key_room(b) + BOLTS[a.bolt].head_h)


def _interface(kind: str, params: dict):
    return {"square": square_mount, "rect": rect_mount, "round": round_mount}[kind](params)


SHOWCASE = {  # seed 0: round 5's "two facing mounts"
    "A": {"kind": "square", "params": {"bolt": "M5", "pitch": 30}, "yaw": 80.0},
    "B": {"kind": "round", "params": {"bolt": "M5", "n": 4, "pcd": 50, "spigot": {"d": 30.0, "h": 2.0}},
          "yaw": 295.0},
    "offset": [-6.0, 5.5],
    "gap_extra": 3.0,
    "load_kg": 1.0,
    "frame": {"yaw": 0.0, "origin": [0.0, 0.0, 0.0]},
}


def _draw(rng: random.Random, seed: int) -> dict:
    if seed == 0:
        import copy

        d = copy.deepcopy(SHOWCASE)
    else:
        d = {}
        for n in "AB":
            while True:
                i = sample_interface(rng)
                if max(i.face_w, i.face_h) <= FACE_MAX:
                    break
            d[n] = {"kind": i.kind, "params": i.params, "yaw": 5.0 * rng.randrange(72)}
        r, a = OFFSET_MAX * math.sqrt(rng.random()), rng.uniform(0.0, 2 * math.pi)
        d["offset"] = [_half(r * math.cos(a)), _half(r * math.sin(a))]
        d["gap_extra"] = _half(rng.uniform(*GAP_EXTRA))
        d["load_kg"] = draw_stated_load(rng)
        d["frame"] = {"yaw": round(rng.uniform(0.0, 360.0), 1),
                      "origin": [_half(rng.uniform(-150.0, 150.0)) for _ in range(3)]}
    itf = {n: _interface(d[n]["kind"], d[n]["params"]) for n in "AB"}
    d["distance"] = _half(_need(itf) + d["gap_extra"])
    return d


def layout(d: dict) -> dict:
    """The fixtures in the bracket's own frame: A's face at z = 0 facing up, B's at z = distance facing
    down, the wall behind (towards -y)."""
    itf = {n: _interface(d[n]["kind"], d[n]["params"]) for n in "AB"}
    ox, oy = d["offset"]
    pl = {"A": Placement.from_yaw_flip((0.0, 0.0, 0.0), d["A"]["yaw"], False),
          "B": Placement.from_yaw_flip((ox, oy, d["distance"]), d["B"]["yaw"], True)}
    return {"itf": itf, "placements": pl, "distance": d["distance"]}


# ---------------------------------------------------------------- the reference


def _outline(itf, pl: Placement, inset: float) -> bd.Face:
    """The fixture's face outline set in by ``inset``, in plan at z = 0."""
    if itf.kind == "round":  # its seam at the back, where the plate's straight sides take over
        return bd.Pos(pl.origin[0], pl.origin[1], 0.0) * bd.Rot(0, 0, -90) * \
            bd.Face(bd.Wire(bd.Edge.make_circle(itf.face_w / 2 - inset)))
    else:
        f = bd.Face(bd.Wire.make_rect(itf.face_w - 2 * inset, itf.face_h - 2 * inset))
    x = pl.vec((1.0, 0.0, 0.0))
    yaw = math.degrees(math.atan2(x[1], x[0]))
    return bd.Pos(pl.origin[0], pl.origin[1], 0.0) * bd.Rot(0, 0, yaw) * f


def _teardrop(r: float, at, height: float, z0: float) -> bd.Part:
    """A vertical cut of radius ``r`` at plan point ``at``, from z0 up ``height``: round, or a teardrop
    pointing +y (away from the bed when printed on the wall) when wider than a printer bridges."""
    with bd.BuildSketch() as sk:
        bd.Circle(r)
        if 2 * r > BORE_MAX:
            k = r / math.sqrt(2.0)
            bd.Polygon((-k, k), (0.0, r * math.sqrt(2.0)), (k, k), (0.0, 0.0), align=None)
    circ = sk.sketch.faces()[0]
    return bd.extrude(bd.Pos(at[0], at[1], z0) * circ, height, dir=(0, 0, 1))


def _plate(itf, pl: Placement, z_face: float, up: float, y_wall: float) -> tuple[bd.Part, tuple]:
    seat = _outline(itf, pl, INSET)
    run = STEP / math.tan(math.radians(STEP_SLOPE))
    top = bd.offset(seat, amount=run, kind=bd.Kind.INTERSECTION).faces()[0]
    rim = bd.offset(seat, amount=run + LEDGE, kind=bd.Kind.INTERSECTION).faces()[0]
    bb = rim.bounding_box()
    back = bd.Edge.make_line((bb.min.X, y_wall, 0.0), (bb.max.X, y_wall, 0.0))
    if itf.kind == "round":  # a D: the circle and a rectangle back to the wall (a hull would split the arc)
        c = rim.center()
        with bd.BuildSketch() as sk:
            with bd.Locations((c.X, c.Y)):
                bd.Circle((bb.max.X - bb.min.X) / 2)
            with bd.Locations(((bb.min.X + bb.max.X) / 2, (y_wall + c.Y) / 2)):
                bd.Rectangle(bb.max.X - bb.min.X, c.Y - y_wall)
        hull = sk.sketch.faces()[0]
    else:
        hull = bd.make_hull(list(rim.edges()) + [back])
    zdir = (0.0, 0.0, up)
    # the step: a loft from the seat up to its outline grown by ``run``
    pad = bd.Solid.make_loft([bd.Pos(0, 0, z_face) * seat.outer_wire(),
                              bd.Pos(0, 0, z_face + up * STEP) * top.outer_wire()])
    body = pad + bd.extrude(bd.Pos(0, 0, z_face + up * STEP) * hull, PLATE - STEP, dir=zdir)
    b = BOLTS[itf.bolt]
    z_lo = min(z_face, z_face + up * PLATE) - 1.0
    for hx, hy in itf.holes:
        p = pl.point((hx, hy, 0.0))
        body = body - _teardrop(b.clear_medium / 2, p, PLATE + 2.0, z_lo)
    for pr in itf.protrusions:
        p = pl.point((pr.x, pr.y, 0.0))
        r = pr.radius + BORE_ROOM
        cut = _teardrop(r, p, PLATE + 2.0, z_lo)
        ring = b.head_d / 2 + RING_ROOM
        for hx, hy in itf.holes:
            h = pl.point((hx, hy, 0.0))
            disc = bd.Pos(h[0], h[1], z_lo) * bd.Cylinder(ring, PLATE + 2.0, align=(bd.Align.CENTER,) * 2 + (bd.Align.MIN,))
            if (cut & disc).volume > 1e-6:
                raise Reject("a spigot's cut reaches a bolt head")
        body = body - cut
    return body, (bb.min.X, bb.max.X)


def build_reference(lay: dict) -> bd.Part:
    """The C-shaped bracket in its own frame (see the module text)."""
    pl, itf, dist = lay["placements"], lay["itf"], lay["distance"]
    run = STEP / math.tan(math.radians(STEP_SLOPE))
    y_wall = min(_outline(itf[n], pl[n], INSET - run - LEDGE).bounding_box().min.Y for n in "AB") - WALL_GAP
    pa, xa = _plate(itf["A"], pl["A"], 0.0, 1.0, y_wall)
    pb, xb = _plate(itf["B"], pl["B"], dist, -1.0, y_wall)
    x0, x1 = min(xa[0], xb[0]) - WALL_SIDE, max(xa[1], xb[1]) + WALL_SIDE
    wall = bd.Pos((x0 + x1) / 2, y_wall - WALL / 2, dist / 2) * bd.Box(x1 - x0, WALL, dist - 2 * STEP)
    # two side walls, from the back wall forward to just behind the hindmost bolt head: they stiffen the
    # bracket against B moving to and from the wall, and stay out of every key's swing (the keys come from
    # the front)
    heads = [pl[n].point((hx, hy, 0.0))[1] - BOLTS[itf[n].bolt].head_d / 2 for n in "AB" for hx, hy in itf[n].holes]
    y_side = min(heads) - SIDE_ROOM
    if y_side - y_wall < SIDE_MIN:
        raise Reject("no room for side walls behind the bolts")
    for xs in (x0 + SIDE / 2, x1 - SIDE / 2):
        wall = wall + bd.Pos(xs, (y_wall + y_side) / 2, dist / 2) * bd.Box(SIDE, y_side - y_wall, dist - 2 * STEP)
    part = pa + pb + wall
    return part.clean()


# ---------------------------------------------------------------- the task


def frame(d: dict) -> Placement:
    f = d["frame"]
    return Placement(tuple(f["origin"]), rot_axis_angle((0.0, 0.0, 1.0), f["yaw"]))


def task_from_draw(seed: int, d: dict, lay: dict, level: int) -> Task:
    g = frame(d)
    pose = {n: lay["placements"][n].then(g) for n in "AB"}
    tol = dict(DEFAULT_TOLERANCES, seat_setback=1.0, seat_overhang_max=0.5, tool_length=30.0, tool_diameter_extra=0.0,
               manufacturing={"process": "fdm", "quantity": 1, "direction": None},
               required_checks=list(REQUIRED[level]))
    meta = {"generator": "facing", "situation": "facing", "test": NAMES[level]}
    task = Task(f"facing-L{level}-s{seed}", level, seed, dict(lay["itf"]), [pose], tol, meta,
                generator_version=GENERATOR_VERSION, family="facing")
    if level >= 3:
        apply_stated_load(task, {LOADED: d["load_kg"]})
        task.tolerances["score"] = "material"
    return task


def generate(seed: int, level: int, max_tries: int = 400, tally: dict | None = None):
    """Return (task, draw, reference solid in world coordinates) for ``level``
    (1, 2 or 3). Deterministic in ``seed``; the same draw at every level."""
    from ...grade import grade

    tally = tally if tally is not None else {}

    def no(why):
        tally[why] = tally.get(why, 0) + 1

    rng = random.Random(f"facing-{seed}")
    for _ in range(max_tries if seed else 1):
        d = _draw(rng, seed)
        lay = layout(d)
        strict = task_from_draw(seed, d, lay, 3)
        try:
            ref = build_reference(lay)
        except (Reject, ValueError) as e:
            no(str(e) if isinstance(e, Reject) else "reference could not be built")
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
        d["summary"] = (f"A {lay['itf']['A'].description}; B {lay['itf']['B'].description}; faces "
                        f"{d['distance']:g} mm apart" + (f"; {d['load_kg']:g} kg on B" if level >= 3 else ""))
        return task_from_draw(seed, d, lay, level), d, ref
    raise RuntimeError(f"seed {seed}: no valid facing draw in {max_tries} tries ({tally})")
