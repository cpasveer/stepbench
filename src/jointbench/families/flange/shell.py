"""Flange and block, levels 1-3: a round flange A and a tall rectangular block
B, their mounting faces standing upright and turned a little away from facing
each other; one solid part bolts to both.

The levels as for facing: 1 fits (seat inside each face, key turn, clean), 2
adds filament printing, 3 a stated load on B with printed weight as the score.
One seed gives the same fixed parts at every level (one random stream per
seed, drawn before the level is looked at). Seed 0 is the showcase: the round 5
"Hold a bearing" arrangement without its bearing -- 6 x M6 on a 50 mm circle
with a 25 mm boss, a 36 x 96 mm block with 2 x 2 M6 at 20 x 80 mm, 70 mm
apart, bent 20 degrees. Other seeds vary sizes, bend, distance and offsets
round it.

The arrangement, in its own frame (z up, the print direction): A's face is the
plane x = 0, its normal +x, its centre the origin. B's face centre is ``D`` mm
out, ``S`` mm to the side and ``H`` mm up; its normal is A's reversed, then
turned ``BEND`` degrees about the vertical, so it looks back towards A's axis.
Both faces are exactly upright: every bolt is horizontal, every head sits on
an upright face.

The reference is built backwards, so a solution is known to exist. It prints
standing on the bed and is, apart from two pads, one vertical prism:

* two upright slabs, one behind each face (set back ``SET`` mm from it, so no
  part of a slab lies on the mounting plane outside the face), joined by two
  side walls into a ring in plan: the space between the slabs is open at top
  and bottom, the bolt heads sit on the slabs' inner faces, and a hex key
  reaches them from above or below;
* on each slab a pad ``SET`` mm proud: its front is the seat, the face's
  outline set in ``INSET`` mm (for the flange a 24-sided polygon), its sides
  drafted so the pad's underside slopes no more than 37 deg from vertical;
* the ring stands on the bed under the lower pad and ends flat level with the
  top of the higher one; each slab reaches ``SIDE_ROOM`` beyond its face, so
  a key's long leg swings past the side walls;
* the flange's boss passes through a teardrop hole (45 deg roof), the bolts
  through plain clearance holes (horizontal, under 10 mm: no roof needed).

The flange is clocked so no bolt is at the top (the teardrop's roof needs the
room). The whole arrangement is placed anywhere and turned about the vertical,
never tilted: the part prints standing on its ring. A draw is kept only if the
reference passes everything of level 3 with the real grader.
"""
from __future__ import annotations

import math
import os
import random
import tempfile

from ...catalogue import BOLTS, GENERATOR_VERSION, make_interface
from ...geometry import BUILD_ROOM, Placement, fits_build_volume, rot_axis_angle, write_step
from ...requirements import apply_stated_load, draw_stated_load
from ...task import DEFAULT_TOLERANCES, Task

KNOBS_DEFAULT: dict = {}
SITUATION = (
    "Two parts are fixed in space: a round flange and a rectangular block. Their\n"
    "mounting faces stand upright, roughly facing each other but turned at an\n"
    "angle, the block offset to one side and in height. Nothing stands between\n"
    "them; the arrangement stands at an arbitrary position, turned arbitrarily\n"
    "about the vertical."
)
REQUIRED = {
    1: ["SEAT_INSIDE", "CLEAN", "KEY_TURN"],
    2: ["SEAT_INSIDE", "CLEAN", "KEY_TURN", "MANUFACTURING"],
    3: ["SEAT_INSIDE", "CLEAN", "KEY_TURN", "MANUFACTURING", "STIFFNESS"],
}
NAMES = {1: "fits", 2: "printable", 3: "strong"}
LOADED = "B"  # A is held, B carries the stated load

# the showcase (seed 0): round 5's "Hold a bearing" without the bearing
SHOWCASE = {
    "A": {"bolt": "M6", "n": 6, "pcd": 50, "spigot": {"d": 25.0, "h": 5.0}},
    "B": {"bolt": "M6", "nx": 2, "ny": 2, "pitch_x": 20, "pitch_y": 80},
    "D": 70.0, "S": 36.0, "H": -35.0, "BEND": 20.0,
}
# the variation round it (other seeds)
FLANGE_BOLTS = ("M5", "M6")
FLANGE_N = (4, 5, 6)
FLANGE_PCD = (40, 50, 60)
SPIGOT_D = (12.0, 16.0, 20.0, 25.0)
SPIGOT_H = (3.0, 5.0)
BLOCK_BOLTS = ("M5", "M6")
BLOCK_PITCH_X = (20, 25, 30)
BLOCK_PITCH_Y = (50, 60, 70, 80)
# level 6 (moves) needs the block's two bolt rows at least this far apart: closer, two coupler pivots no longer fit
# beside the latch arm, nor the posts' nut paths on the frame's head (notes l6-flange.md, seeds 1, 3, 4: 50-60 mm).
# Every level of a seed shares one draw, so a fresh seed with a shorter block is redrawn at every level (
# 16 Nov 2026); seeds whose first draw is long enough keep it.
L6_ROW_GAP_MIN = 70
D_RANGE = (62.0, 80.0)      # mm from A's face to B's centre, along A's normal
S_RANGE = (15.0, 40.0)      # mm B's centre lies to the side
H_RANGE = (-40.0, 10.0)     # mm B's centre lies above A's (negative: below)
BEND_RANGE = (12.0, 28.0)   # deg the faces are turned from facing each other

# the reference
SET = 1.5        # mm a slab stands back from its mounting plane; the pad's height
INSET = 1.5      # mm the seat stays inside the face's outline (setback 1.0 + 0.5 to spare)
DRAFT = 2.0      # mm a pad's base reaches out beyond its front: sides at atan(SET / DRAFT) = 37 deg from upright
SLAB = 8.0       # mm, slab thickness behind the pad: grip = SET + SLAB
WALL = 5.0       # mm, the side walls
SIDE_ROOM = 14.0  # mm a slab reaches beyond its face's half width, so a key's long leg swings past the walls
POLY = 24        # sides of the flange's seat polygon
SPIGOT_ROOM = 0.75  # mm round the boss
TOOL = 30.0      # mm, the straight tool behind a head (TOOL_ACCESS, tolerances tool_length)
AXIS_MAX = 0.95  # |component| of a world face normal: above this a face is near a coordinate plane


def long_enough_for_l6(d: dict) -> bool:
    """Is the block's bolt-row spacing (upright, ``pitch_y``) enough for level 6's linkage?"""
    return float(d["B"]["pitch_y"]) >= L6_ROW_GAP_MIN


def _half(x: float) -> float:
    return round(x * 2) / 2


def _draw(rng: random.Random, seed: int) -> dict:
    """Only numbers (so the draw can be stored). Seed 0: the showcase's
    numbers, with its own random frame."""
    if seed == 0:
        d = {k: (dict(v) if isinstance(v, dict) else v) for k, v in SHOWCASE.items()}
    else:
        a = {"bolt": rng.choice(FLANGE_BOLTS), "n": rng.choice(FLANGE_N), "pcd": rng.choice(FLANGE_PCD)}
        if rng.random() < 0.6:
            a["spigot"] = {"d": rng.choice([s for s in SPIGOT_D if s <= a["pcd"] - 25]), "h": rng.choice(SPIGOT_H)}
        b = {"bolt": rng.choice(BLOCK_BOLTS), "nx": 2, "ny": 2,
             "pitch_x": rng.choice(BLOCK_PITCH_X), "pitch_y": rng.choice(BLOCK_PITCH_Y)}
        d = {"A": a, "B": b, "D": _half(rng.uniform(*D_RANGE)), "S": _half(rng.uniform(*S_RANGE)),
             "H": _half(rng.uniform(*H_RANGE)), "BEND": float(round(rng.uniform(*BEND_RANGE)))}
    d["load_kg"] = draw_stated_load(rng)
    d["frame"] = {"yaw": round(rng.uniform(0.0, 360.0), 1),
                  "origin": [_half(rng.uniform(-150.0, 150.0)) for _ in range(3)]}
    return d


# --------------------------------------------------------------------------
# the arrangement in its own frame


def _rot(x, y, z):
    """Placement rotation (stored row by row) from the local axes' columns."""
    return tuple(tuple(float(round(c[i], 12)) for c in (x, y, z)) for i in range(3))


def normals(d: dict):
    """A's and B's face normals in the arrangement frame (both horizontal)."""
    t = math.radians(d["BEND"])
    return (1.0, 0.0, 0.0), (-math.cos(t), -math.sin(t), 0.0)


def _side(n):
    """The face's horizontal in-plane direction u, so that (u, up, n) is right-handed."""
    return (-n[1], n[0], 0.0)


def layout(d: dict) -> dict:
    """The two interfaces and their placements in the arrangement frame."""
    ia, ib = make_interface("round", d["A"]), make_interface("rect", d["B"])
    na, nb = normals(d)
    up = (0.0, 0.0, 1.0)
    # A: no bolt at the top (the boss's teardrop roof needs the room): the
    # first bolt half a step past straight up
    n = d["A"]["n"]
    al = math.radians(90.0 + 180.0 / n)
    ua = _side(na)
    xa = tuple(math.cos(al) * ua[i] + math.sin(al) * up[i] for i in range(3))
    ya = tuple(-math.sin(al) * ua[i] + math.cos(al) * up[i] for i in range(3))
    pa = Placement((0.0, 0.0, 0.0), _rot(xa, ya, na))
    # B: its pitch_y upright (the tall block), its pitch_x along the face
    ub = _side(nb)
    cb = (d["D"], d["S"], d["H"])
    pb = Placement(cb, _rot(ub, up, nb))
    return {"itf": {"A": ia, "B": ib}, "placements": {"A": pa, "B": pb}, "normals": {"A": na, "B": nb}}


# --------------------------------------------------------------------------
# the reference


def _polygon_yz(points_uv, origin, u, v):
    """3D points of a polygon given in a face's (u, v) coordinates."""
    return [tuple(origin[i] + p[0] * u[i] + p[1] * v[i] for i in range(3)) for p in points_uv]


def _pad(outline_front, outline_base, origin, u, v, n):
    """A pad: ruled loft from its base (on the slab, SET mm out from the
    mounting plane) to its front (on the mounting plane)."""
    import build123d as bd

    base_o = tuple(origin[i] + SET * n[i] for i in range(3))
    base = bd.Face(bd.Wire.make_polygon(_polygon_yz(outline_base, base_o, u, v), close=True))
    front = bd.Face(bd.Wire.make_polygon(_polygon_yz(outline_front, origin, u, v), close=True))
    return bd.Solid.make_loft([base.outer_wire(), front.outer_wire()], ruled=True)


def _flange_outline(r: float):
    """The flange seat: a regular polygon of POLY sides, circumradius r, a
    vertex straight down (as (u, v) points)."""
    return [(r * math.cos(-math.pi / 2 + 2 * math.pi * k / POLY), r * math.sin(-math.pi / 2 + 2 * math.pi * k / POLY))
            for k in range(POLY)]


def _rect_outline(w: float, h: float):
    return [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]


def build_reference(d: dict, lay: dict):
    import build123d as bd
    import numpy as np
    from scipy.spatial import ConvexHull

    ia, ib = lay["itf"]["A"], lay["itf"]["B"]
    na, nb = lay["normals"]["A"], lay["normals"]["B"]
    up = (0.0, 0.0, 1.0)
    ua, ub = _side(na), _side(nb)
    ca, cb = (0.0, 0.0, 0.0), lay["placements"]["B"].origin

    # pads: the seat inside each face, the base DRAFT wider all round
    ra = ia.face_w / 2 - INSET
    grow = DRAFT / math.cos(math.pi / POLY)  # circumradius of a polygon whose sides lie DRAFT further out
    pad_a = _pad(_flange_outline(ra), _flange_outline(ra + grow), ca, ua, up, na)
    wb, hb = ib.face_w - 2 * INSET, ib.face_h - 2 * INSET
    pad_b = _pad(_rect_outline(wb, hb), _rect_outline(wb + 2 * DRAFT, hb + 2 * DRAFT), cb, ub, up, nb)

    # the ring in plan: the hull of the two slabs, less its inside (WALL in from the hull, behind both slabs)
    def slab_corners(c, n, u, half, bolt):
        # the slab, and the side walls running straight back from it as far as a tool behind a head reaches
        reach = SET + SLAB + BOLTS[bolt].head_h + TOOL + 2.0
        out = []
        for t in (SET, reach):
            for s in (-half, half):
                out.append((c[0] + t * n[0] + s * u[0], c[1] + t * n[1] + s * u[1]))
        return out

    half_a = ia.face_w / 2 + SIDE_ROOM
    half_b = ib.face_w / 2 + SIDE_ROOM
    pts = np.array(slab_corners(ca, na, ua, half_a, ia.bolt) + slab_corners(cb, nb, ub, half_b, ib.bolt))
    hull = [tuple(pts[i]) for i in ConvexHull(pts).vertices]
    z_bed = min(-(ra + grow), cb[2] - hb / 2 - DRAFT)  # under the lower pad
    z_top = max(ra + grow, cb[2] + hb / 2 + DRAFT)    # over the higher pad: one flat top (a top sloping
    # down towards a slab would leave a thin wedge on it, FDM_WALL_MIN)
    outer = bd.Polygon(*[(x, y) for x, y in hull], align=None)
    ring = bd.extrude(bd.Pos(0, 0, z_bed) * outer, z_top - z_bed, dir=(0, 0, 1))
    inner = bd.offset(outer, amount=-WALL, kind=bd.Kind.INTERSECTION)
    cavity = bd.extrude(bd.Pos(0, 0, z_bed - 1) * inner, z_top - z_bed + 2, dir=(0, 0, 1))
    # behind each slab: half-spaces
    big = 1000.0
    for c, n in ((ca, na), (cb, nb)):
        keep = bd.Plane(origin=tuple(c[i] + (SET + SLAB) * n[i] for i in range(3)), z_dir=n) \
            * bd.Pos(0, 0, big / 2) * bd.Box(big, big, big)
        cavity = cavity & keep
    ring = ring - cavity

    body = ring + pad_a + pad_b

    # holes: each bolt's clearance hole, from in front of the face into the ring's inside
    for name, itf, c, n in (("A", ia, ca, na), ("B", ib, cb, nb)):
        b = BOLTS[itf.bolt]
        pl = lay["placements"][name]
        for x, y in itf.holes:
            p = pl.point((x, y, 0.0))
            L = SET + SLAB + 2.0
            body = body - bd.Plane(origin=p, z_dir=n) * bd.Pos(0, 0, L / 2 - 1.0) * bd.Cylinder(b.clear_medium / 2, L)
    # the boss: a teardrop hole through pad and slab, its roof up
    for p in ia.protrusions:
        r = p.radius + SPIGOT_ROOM
        q = r / math.sqrt(2)
        drop = bd.Circle(r) + bd.Polygon((-q, q), (q, q), (0.0, r * math.sqrt(2)), align=None)
        L = SET + SLAB + 2.0
        pl = bd.Plane(origin=(-1.0, 0.0, 0.0), x_dir=ua, z_dir=na)
        body = body - bd.extrude(pl * drop, L, dir=na)
    solids = body.solids()
    return solids[0] if len(solids) == 1 else body


# --------------------------------------------------------------------------
# the task


def frame(d: dict) -> Placement:
    f = d["frame"]
    return Placement(tuple(f["origin"]), rot_axis_angle((0.0, 0.0, 1.0), f["yaw"]))


def task_from_draw(seed: int, d: dict, lay: dict, level: int) -> Task:
    g = frame(d)
    pose = {n: lay["placements"][n].then(g) for n in "AB"}
    tol = dict(DEFAULT_TOLERANCES, seat_setback=1.0, seat_overhang_max=0.5, tool_length=TOOL, tool_diameter_extra=0.0,
               manufacturing={"process": "fdm", "quantity": 1, "direction": None},
               required_checks=list(REQUIRED[level]))
    meta = {"generator": "flange", "situation": "flange", "test": NAMES[level]}
    task = Task(f"flange-L{level}-s{seed}", level, seed, dict(lay["itf"]), [pose], tol, meta,
                generator_version=GENERATOR_VERSION, family="flange")
    if level >= 3:
        apply_stated_load(task, {LOADED: d["load_kg"]})
        task.tolerances["score"] = "material"
    return task


def _frame_ok(task: Task) -> bool:
    return all(max(abs(v) for v in f.normal) <= AXIS_MAX for f in task.faces(0))


def generate(seed: int, level: int, max_tries: int = 60, tally: dict | None = None):
    """Return (task, draw, reference solid in world coordinates) for ``level``
    (1, 2 or 3). Deterministic in ``seed``; the same draw at every level."""
    from ...grade import grade

    tally = tally if tally is not None else {}

    def no(why):
        tally[why] = tally.get(why, 0) + 1

    rng = random.Random(f"flange-{seed}")
    for _ in range(max_tries):
        d = _draw(rng, seed)
        if not long_enough_for_l6(d):
            no("block's bolt rows too close for level 6 (redrawn)")
            continue
        lay = layout(d)
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
            ref = build_reference(d, lay)
        except Exception:  # noqa: BLE001 - a draw whose reference cannot be built is redrawn
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
        d["summary"] = (f"A {lay['itf']['A'].description}; B {lay['itf']['B'].description}; "
                        f"B {d['D']:g} mm out, {d['S']:g} mm aside, {d['H']:g} mm up, faces bent {d['BEND']:g} deg"
                        + (f"; {d['load_kg']:g} kg on B" if level >= 3 else ""))
        return task_from_draw(seed, d, lay, level), d, ref
    raise RuntimeError(f"seed {seed}: no valid flange draw in {max_tries} tries ({tally})")
