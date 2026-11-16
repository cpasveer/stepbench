"""Clamp a pipe, levels 1-3: a flat fixed part A and a round pipe B with its
bolts screwed straight into it; one solid part bolts to both.

The levels as for facing: 1 fits (seat inside A's face, key turn, clean), 2
adds filament printing, 3 a stated load on B with printed weight as the score.
One seed gives the same fixed parts at every level.

Generation works backwards with the adapts shell (``families/adapt/angled``):
the shell gets a flat pad for A and, for the pipe, a stand-in flat pad whose
top is where the pipe will sit. The pipe is then placed with its crest
``SINK`` mm below that top and carved out of the pad: a saddle that wraps the
pipe and follows its round surface. The pipe's bolts form one row along its
crest, square to the pad, so their heads sit inside the shell on the wall's
inner face, turned with a key through the open underside (rows at an angle
round the pipe: a later step). The two mounts face at an angle between
``ANGLE`` degrees apart; the whole is placed anywhere and turned about the
vertical, not tilted (the shell prints on its rim, see ``facing.shell``).
"""
from __future__ import annotations

import math
import os
import random
import tempfile

from ...catalogue import BOLTS, GENERATOR_VERSION, INTERFACES, Interface, edge_margin, fixed_part_solid, \
    make_interface, sample_interface
from ...geometry import BUILD_ROOM, Placement, fits_build_volume, rot_axis_angle, write_step
from ...requirements import apply_stated_load, draw_stated_load
from ...task import DEFAULT_TOLERANCES, Task
from ..adapt import angled as A
from ..facing import shell as F

KNOBS_DEFAULT: dict = {}
SITUATION = (
    "Two parts are fixed in space at an angle to each other. One is a round pipe\n"
    "with its bolts screwed straight into it; the other has a flat mounting face.\n"
    "Nothing stands between them; the arrangement stands at an arbitrary position,\n"
    "turned arbitrarily about the vertical."
)
ODS = (32.0, 40.0, 48.0)
PIPE_BOLTS = ("M4", "M5")
ALONG = (2, 3)
PITCHES = (25.0, 30.0, 40.0)
SINK = 6.0          # mm the pipe's crest sits below the pad's top: the saddle's depth (pad A.PAD_H, then the wall)
# 0: one row on the crest; else two rows at +- this angle round the pipe. Angled rows (20-30 deg) are built
# (a seat cut into the wall square to each bolt) but their seats still leave slivers and overhangs inside the
# shell's thin wall: off until the reference keeps their heads on a thicker block (to do).
ROWS = (0.0,)
ROWS_ANGLED = (20.0, 25.0, 30.0)
SEAT_ROOM = 1.5     # mm an angled bolt's seat (spot face) lies, at its shallowest, below the wall's inner face
ANGLE = (110.0, 180.0)  # deg between the two mounts' normals
REQUIRED = F.REQUIRED
NAMES = F.NAMES


def _seat(params: dict) -> Interface:
    """The pipe's stand-in while the shell is laid out: a flat rectangle about as
    long as the bolt rows and as wide as the saddle. It has no holes: the pipe's
    bolts are cut afterwards along their own axes (``build_reference``)."""
    b = BOLTS[params["bolt"]]
    R = params["od"] / 2
    chord = 2 * math.sqrt(max(0.0, R * R - (R - SINK) ** 2))
    xs = sorted({x for x, _ in params["holes"]})
    # shorter than the pipe (span + 2 x (head + 8)): the saddle runs out through both ends of the pad,
    # with no thin lip where the pipe would end flush with it
    length = max(xs) - min(xs) + 2 * edge_margin(b)
    return Interface(kind="rect", params=dict(params), bolt=b.name, holes=[],
                     face_w=float(length), face_h=float(max(chord + 6.0, 2 * edge_margin(b))),
                     description="the pipe's seat (stand-in)")


INTERFACES.setdefault("pipe_seat", _seat)


def _pipe_params(rng) -> dict:
    bolt = rng.choice(PIPE_BOLTS)
    n = rng.choice(ALONG)
    pitch = rng.choice(PITCHES)
    xs = [(i - (n - 1) / 2) * pitch for i in range(n)]
    a = rng.choice(ROWS)
    angles = [0.0] if a == 0 else [-a, a]
    return {"bolt": bolt, "od": rng.choice(ODS), "holes": [[x, r] for r in angles for x in xs]}


def _draw(rng: random.Random) -> dict:
    while True:  # A: a flat mount small enough for a wall of a part that fits the printer
        i = sample_interface(rng)
        if i.bolt in A.BOLTS_OK and A._pattern_radius(i) + BOLTS[i.bolt].head_d / 2 <= A.MAX_PATTERN \
                and all(2 * p.radius + 1.0 <= A.BORE_MAX for p in i.protrusions):
            break
    pipe = _pipe_params(rng)
    d = {"A": {"kind": i.kind, "params": i.params, "yaw": 5.0 * rng.randrange(72)},
         "B": {"kind": "pipe_seat", "params": pipe, "yaw": 5.0 * rng.randrange(72)},
         "pipe": pipe, "mounts": "AB", "pairs": ("AB",)}
    while True:
        na = A._rand_unit(rng)
        if abs(na[2]) <= F.NZ_FACING:
            break
    phi = math.radians(float(round(rng.uniform(*ANGLE))))
    h = math.hypot(na[0], na[1])
    ha = (na[0] / h, na[1] / h)
    # B's normal: A's turned by phi about the vertical, with a little tilt of its own
    bx, by = ha[0] * math.cos(phi) - ha[1] * math.sin(phi), ha[0] * math.sin(phi) + ha[1] * math.cos(phi)
    bz = rng.uniform(-F.NZ_FACING, F.NZ_FACING)
    k = math.sqrt(1 - bz * bz)
    nb = A._unit((k * bx, k * by, bz))
    d["normals"] = {"A": list(na), "B": list(nb)}
    d["angle"] = math.degrees(phi)
    # the roof: two walls square (in plan) to the line between the mounts, tilted up; and, unless the
    # mounts are opposite, a back wall closing the open side
    gap = (ha[0] - bx, ha[1] - by)
    g = math.hypot(*gap)
    side = (-gap[1] / g, gap[0] / g)
    closing = []
    for s in (1.0, -1.0):
        turn = math.radians(rng.uniform(-F.ROOF_TURN, F.ROOF_TURN))
        c, sn = math.cos(turn), math.sin(turn)
        px, py = s * (c * side[0] - sn * side[1]), s * (sn * side[0] + c * side[1])
        nz = rng.uniform(*F.ROOF_NZ)
        kk = math.sqrt(1.0 - nz * nz)
        closing.append(A._unit((kk * px, kk * py, nz)))
    back = (-(ha[0] + bx), -(ha[1] + by))
    if math.hypot(*back) > 0.3:
        bh = math.hypot(*back)
        nz = rng.uniform(-0.3, 0.5)
        kk = math.sqrt(1.0 - nz * nz)
        closing.append(A._unit((kk * back[0] / bh, kk * back[1] / bh, nz)))
    for count, (lo, hi) in ((rng.randint(*A.SKIRT), A.SKIRT_NZ),):
        got = 0
        for _ in range(60):
            if got >= count:
                break
            v = A._rand_unit(rng)
            if lo <= v[2] <= hi and all(A._angle(v, w) >= A.MIN_SEP for w in [na, nb] + closing):
                closing.append(v)
                got += 1
    d["closing"] = [list(v) for v in closing]
    d["depth"] = {n: A._half(rng.uniform(*A.DEPTH)) for n in "AB"}
    d["closing_depth"] = [A._half(rng.uniform(*A.DEPTH)) for _ in closing]
    d["bed_depth"] = A._half(rng.uniform(*A.BED_DEPTH))
    d["shift"] = {n: [0.0, 0.0] for n in "AB"}
    d["load_kg"] = draw_stated_load(rng)
    d["frame"] = {"yaw": round(rng.uniform(0.0, 360.0), 1),
                  "origin": [A._half(rng.uniform(-150.0, 150.0)) for _ in range(3)]}
    return d


def pipe_placement(d: dict, lay: dict) -> Placement:
    """The pipe in the part frame: on its stand-in's placement, the axis along
    the row of bolts, the crest SINK mm below the stand-in's top face."""
    s = lay["placements"]["B"]
    R = d["pipe"]["od"] / 2
    z = s.vec((0.0, 0.0, 1.0))
    return Placement(tuple(s.origin[i] + (SINK - R) * z[i] for i in range(3)), s.rot)


def build_reference(d: dict, lay: dict):
    """The shell with its pads and holes (``A.build_reference``), the pipe carved
    out of B's pad: the saddle."""
    import build123d as bd

    body = A.build_reference(lay)
    itf = make_interface("pipe", d["pipe"])
    pp = pipe_placement(d, lay)
    # the pipe's outline (no tapped holes: those would leave plugs of the part inside the pipe)
    xs = [q[0] for q, _ in itf.hole_frames]
    envelope = bd.Pos((max(xs) + min(xs)) / 2, 0, 0) * bd.Rot(0, 90, 0) * bd.Cylinder(itf.face_h / 2, itf.face_w + 40.0)
    body = body - envelope.moved(pp.location())
    # each bolt: a clearance hole along its own axis, and a flat seat for its head (a spot face, square to
    # the axis) cut in from the shell's cavity, SEAT_ROOM short of the wall's inner face
    b = BOLTS[itf.bolt]
    R = d["pipe"]["od"] / 2
    inner = A.PAD_H + A.WALL  # the wall's inner face, from the pad's top, along the stand-in's normal
    for p, ax in itf.hole_frames:
        cos_a = ax[2]
        z0 = SINK - R + p[2]  # the bolt's point on the pipe, measured from the pad's top
        t_inner = (inner - z0) / cos_a  # along the axis, from the pipe to the wall's inner face
        r_seat = b.head_d / 2 + 1.0
        P, V = pp.point(p), pp.vec(ax)
        if abs(cos_a - 1.0) < 1e-9:  # on the crest: square to the wall, the head sits on its inner face
            grip, seat = t_inner, None
        else:  # at an angle: a seat cut in so deep that its whole floor lies inside the wall
            sin_a = math.sqrt(max(0.0, 1.0 - cos_a * cos_a))
            grip = t_inner - (SEAT_ROOM + r_seat * sin_a) / cos_a
            if grip < b.grip_min:
                raise ValueError("no room for an angled bolt's seat")
            seat = bd.Plane(origin=P, z_dir=V) * bd.Pos(0, 0, grip + 20.0) * bd.Cylinder(r_seat, 40.0)
        hole = bd.Plane(origin=P, z_dir=V) * bd.Pos(0, 0, grip / 2) * bd.Cylinder(b.clear_medium / 2, grip + 2.0)
        body = body - hole
        if seat is not None:
            body = body - seat
    solids = body.solids()
    return solids[0] if len(solids) == 1 else body


def frame(d: dict) -> Placement:
    return F.frame(d)


def task_from_draw(seed: int, d: dict, lay: dict, level: int) -> Task:
    g = frame(d)
    parts = {"A": lay["itf"]["A"], "B": make_interface("pipe", d["pipe"])}
    pose = {"A": lay["placements"]["A"].then(g), "B": pipe_placement(d, lay).then(g)}
    tol = dict(DEFAULT_TOLERANCES, seat_setback=1.0, seat_overhang_max=0.5, tool_length=30.0, tool_diameter_extra=0.0,
               manufacturing={"process": "fdm", "quantity": 1, "direction": None},
               required_checks=list(REQUIRED[level]))
    meta = {"generator": "pipe-shell", "situation": "pipe", "test": NAMES[level]}
    task = Task(f"pipe-L{level}-s{seed}", level, seed, parts, [pose], tol, meta,
                generator_version=GENERATOR_VERSION, family="pipe")
    if level >= 3:
        apply_stated_load(task, {"B": d["load_kg"]})
        task.tolerances["score"] = "material"
    return task


def _frame_ok(task: Task) -> bool:
    return all(max(abs(v) for v in f.normal) <= A.AXIS_MAX for f in task.faces(0))


def generate(seed: int, level: int, max_tries: int = 400, tally: dict | None = None):
    """Return (task, draw, reference solid in world coordinates) for ``level``.
    Deterministic in ``seed``; the same draw at every level."""
    from ...grade import grade

    tally = tally if tally is not None else {}

    def no(why):
        tally[why] = tally.get(why, 0) + 1

    rng = random.Random(f"pipe-{seed}")
    for _ in range(max_tries):
        d = _draw(rng)
        try:
            lay = A.layout(d)
        except A.Reject as e:
            no(str(e))
            continue
        strict = task_from_draw(seed, d, lay, 3)
        for _ in range(30):
            if _frame_ok(strict):
                break
            no("a face near a coordinate plane (frame redrawn)")
            d["frame"]["yaw"] = round(rng.uniform(0.0, 360.0), 1)
            strict = task_from_draw(seed, d, lay, 3)
        if not _frame_ok(strict):
            continue
        try:
            ref = build_reference(d, lay)
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
            rep = grade(strict, p, optional=False, score=False, skip=frozenset({"STIFFNESS"}))
            if rep.passed:
                rep = grade(strict, p, optional=False, score=False,
                            skip=frozenset({"CLEAN", "MANUFACTURING", "SEAT_INSIDE"}))
        if not rep.passed:
            no("reference fails: " + ", ".join(sorted(set(rep.failed()))))
            continue
        st = next((c for c in rep.checks if c.name == "STIFFNESS"), None)
        if st is not None and (st.details or {}).get("verdict") == "marginal":
            no("stiffness only marginal")
            continue
        pipe = make_interface("pipe", d["pipe"])
        d["summary"] = (f"A {lay['itf']['A'].description}; B {pipe.description}; mounts {d['angle']:.0f} deg apart"
                        + (f"; {d['load_kg']:g} kg on B" if level >= 3 else ""))
        return task_from_draw(seed, d, lay, level), d, ref
    raise RuntimeError(f"seed {seed}: no valid pipe draw in {max_tries} tries ({tally})")
