"""Adapts (level 4b), angled mounts: one part for two arrangements, carrying a
stated load in each, printed in filament.

The model is given two complete arrangements of fixed parts, each in
coordinates of its own: arrangement 1 (fixed parts A and B) and arrangement 2
(fixed parts C and D). In each, the two mounting faces stand at an angle to
each other -- a different angle in each arrangement, never near parallel and
never near square -- and at a different distance. It hands in ONE solid and,
per arrangement, the rigid move that puts that solid in place. Everything of
level 3 holds in both: the part fits, is cleanly modelled, prints in PLA
without support (judged once, on the part as handed in), and in each
arrangement carries that arrangement's stated load; its printed weight is the
score. So the part needs four mounting faces at unrelated angles: no box, no
standard shape.

Generation works backwards, so a solution is guaranteed. In the part's own
frame the reference is a faceted shell, open underneath, that prints standing
on its rim:

* four mount directions are drawn: A and B at the angle of arrangement 1,
  C and D at the angle of arrangement 2, every two well apart, and a few
  closing directions; every face slopes at least 47 deg from level, so
  neither the outside nor the inside of the shell overhangs;
* the shell's walls are WALL thick; the bolt heads sit inside it, on the
  inner face of the wall, reached through the open underside;
* each mount is a pad standing PAD_H proud of its wall, cut to its fixture's
  face (set in by the seat setback), with sloped sides.

A draw is kept only if every bolt's clamped ring lies on its pad and every
head has room inside the shell, the two fixtures of each arrangement do not
overlap, the arrangements are not one pair of mounts at the same relative
pose, the part fits the Prusa MINI with room, and the reference passes every
check -- printing and both stated loads included -- in both arrangements.
Each arrangement is then moved into a random frame of its own (no mounting
face within 18 deg of a coordinate plane); that move is the reference
placement, kept in ``_solution`` only.

Known limit: only M3-M5 bolts and mounts no wider than a wall face allows.
"""
from __future__ import annotations

import math
import os
import random
import tempfile

import build123d as bd

from ...catalogue import BOLTS, GENERATOR_VERSION, fixed_part_solid, make_interface, sample_interface
from ...geometry import BUILD_ROOM, Placement, fits_build_volume, rot_axis_angle, rot_from_quaternion, write_step
from ...requirements import draw_stated_load
from ...task import DEFAULT_TOLERANCES, Task

LEVEL = 1  # the adapt family's own level; the benchmark calls it "adapts", level 4b (14 November)
KNOBS_DEFAULT: dict = {}
SITUATION = (
    "Two arrangements of fixed parts, each in coordinates of its own. In each,\n"
    "two parts are fixed in space; their mounting faces are at an angle to each\n"
    "other, with nothing in between."
)
ARRANGEMENTS = (("1", ("A", "B")), ("2", ("C", "D")))
PLACEMENT_FILE = "placement.json"

# the angle between the two mounting faces of an arrangement (between their normals), deg:
# never near parallel (facing, 180 / side by side, 0) and never near square (90)
ANGLE_BANDS = ((55.0, 78.0), (102.0, 130.0))
ANGLE_APART = 15.0      # deg the two arrangements' angles differ at least
DIST_APART = 10.0       # mm the two arrangements' mount distances differ at least
MIN_SEP = 40.0          # deg between any two wall directions of the shell
SKIRT = (1, 2)          # closing walls round the bottom, flaring out to the rim (downward-facing)
SKIRT_NZ = (-0.60, -0.25)
CLOSING = (1, 2)        # other closing walls in random directions, besides the four mount walls
RIM_CUT = 8.0           # mm a skirt wall added under an upward-facing wall cuts into the rim corner
EDGE_MAX = 90.0         # deg between the normals of two walls that meet: at most square, else chamfered
CHAMFER = 5.0           # mm a sharp outside edge is cut back
CHAMFER_ROUNDS = 8      # sharp edges chamfered at most (pipe level 4 raises it for a shell with a cut wall)
FOOT = 4.0              # mm above the bed in which the cavity's sides are vertical
KEEL_MIN = 0.45         # |z| of an edge where two downward faces meet (overhang rule: edges at most 63.4 deg from vertical)
DEPTH = (40.0, 50.0)    # distance of a wall's outer face from the shell's centre, mm
BED_DEPTH = (26.0, 36.0)  # of the open underside (the rim it prints standing on)
WALL = 6.0              # mm, the shell's wall
MAX_PATTERN = 16.0      # mm, largest bolt-pattern radius plus head radius drawn (a wall must hold it)
INSET = 1.5             # mm the seat stays inside a fixture's face (setback 1.0 + 0.5 to spare)
RING_ROOM = 0.5         # mm a bolt's clamped ring stays inside its facet
# a mount is a pad standing PAD_H proud of its wall, its face the fixture's face set in by INSET, its
# sides sloped BEVELS deg off the face (the steepest that prints on that edge). PAD_H is at least the
# modelling rules' stub length (5 mm), so the pad does not read as a member pushed through a plate.
BEVELS = (60.0, 50.0, 40.0, 30.0, 20.0)
PAD_H = 5.5
ROUND_SIDES = 16        # a round fixture's facet is a polygon of this many sides inside its face
# printing (filament, without support), built in: the shell stands on its rim, and every wall's
# normal keeps |z| <= NZ_MAX, so neither its outside nor its inside (which looks the other way)
# leans out more than the overhang rule allows (45 deg from vertical). A spigot's room is a hole
# at most 10 mm wide (the overhang rule lets a hole that small print sideways).
NZ_MIN = -0.68
NZ_MAX = 0.68
MOUNT_NZ = (-0.20, 0.68)  # a mount faces at most a little downward: its pad's underside must not hang
BOLTS_OK = ("M3", "M4", "M5", "M6", "M8")
BORE_MAX = 10.0         # mm, widest sideways hole (overhang_hole_exempt_diameter in dfm/processes/fdm.toml)
AXIS_MAX = 0.95         # |component| of a world face normal: above this a face is near a coordinate plane
TOLERANCES = dict(DEFAULT_TOLERANCES, seat_setback=1.0, seat_overhang_max=0.5,
                  manufacturing={"process": "fdm", "quantity": 1, "direction": None},
                  required_checks=["SEAT_INSIDE", "CLEAN", "MANUFACTURING", "KEY_TURN"], score="material")
BED = (0.0, 0.0, -1.0)
LOADED = {"1": "B", "2": "D"}  # in each arrangement the first fixed part is held, the other carries the load
# KEY_TURN, as for every generator: the reference's bolts turn with a standard hex key, in each
# arrangement. The shell's heads sit inside it, and a key's long leg swings out through the open
# underside.


# --------------------------------------------------------------------------
# small vector helpers (plain tuples, deterministic)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _unit(v):
    n = math.sqrt(_dot(v, v))
    return tuple(x / n for x in v)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _angle(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, _dot(a, b)))))


def _neg(v):
    return tuple(-x for x in v)


def _seg_dist(p0, p1, q0, q1) -> float:
    """Shortest distance between the segments p0-p1 and q0-q1."""
    import numpy as np

    p0, p1, q0, q1 = (np.array(v, float) for v in (p0, p1, q0, q1))
    u, v, w = p1 - p0, q1 - q0, p0 - q0
    a, b, c, d, e = u @ u, u @ v, v @ v, u @ w, v @ w
    den = a * c - b * b
    cands = []
    if den > 1e-12:
        sc, tc = (b * e - c * d) / den, (a * e - b * d) / den
        if 0 <= sc <= 1 and 0 <= tc <= 1:
            cands.append((sc, tc))
    for sc in (0.0, 1.0):  # an end of one against the other
        cands.append((sc, min(1.0, max(0.0, (e + sc * b) / c))))
    for tc in (0.0, 1.0):
        cands.append((min(1.0, max(0.0, (tc * b - d) / a)), tc))
    return min(float(np.linalg.norm(w + sc * u - tc * v)) for sc, tc in cands)


def _rand_unit(rng):
    return _unit([rng.gauss(0.0, 1.0) for _ in range(3)])


def _perp(n, rng):
    """A random unit vector square to ``n``."""
    while True:
        v = _cross(n, _rand_unit(rng))
        if _dot(v, v) > 0.05:
            return _unit(v)


def _half(x: float) -> float:
    return round(x * 2) / 2


def _thick(itf) -> float:
    """Clamped thickness under a bolt head: the pad and the wall."""
    return PAD_H + WALL


def _pattern_radius(itf) -> float:
    return max(math.hypot(x, y) for x, y in itf.holes)


# --------------------------------------------------------------------------
# the draw: only numbers (so it can be stored), everything else is derived


def _draw(rng: random.Random) -> dict:
    itf = {}
    for n in "ABCD":
        while True:  # small enough for a facet of a part that fits the printer, and printable
            i = sample_interface(rng)
            if i.bolt in BOLTS_OK and _pattern_radius(i) + BOLTS[i.bolt].head_d / 2 <= MAX_PATTERN \
                    and all(2 * p.radius + 1.0 <= BORE_MAX for p in i.protrusions):
                break
        itf[n] = i
    d = {n: {"kind": itf[n].kind, "params": itf[n].params, "yaw": 5.0 * rng.randrange(72)} for n in "ABCD"}
    angles = []
    while len(angles) < 2:  # the second angle well apart from the first
        lo, hi = ANGLE_BANDS[rng.randrange(2)]
        phi = float(round(rng.uniform(lo, hi)))
        if not angles or abs(phi - angles[0]) >= ANGLE_APART:
            angles.append(phi)
    d["angles"] = angles
    normals = {}
    for (a, b), phi in zip(("AB", "CD"), angles):
        for _ in range(500):  # printable, and every face direction well apart from every other
            na = _rand_unit(rng)
            u = _perp(na, rng)
            c, s = math.cos(math.radians(phi)), math.sin(math.radians(phi))
            nb = _unit([c * na[i] + s * u[i] for i in range(3)])
            if all(MOUNT_NZ[0] <= v[2] <= MOUNT_NZ[1] for v in (na, nb)) \
                    and all(_angle(v, w) >= MIN_SEP for v in (na, nb) for w in normals.values()):
                break
        normals[a], normals[b] = na, nb
    closing = []
    for count, (lo, hi) in ((rng.randint(*SKIRT), SKIRT_NZ), (rng.randint(*CLOSING), (-NZ_MAX, NZ_MAX))):
        got = 0
        for _ in range(60):
            if got >= count:
                break
            v = _rand_unit(rng)
            if lo <= v[2] <= hi and all(_angle(v, w) >= MIN_SEP for w in list(normals.values()) + closing):
                closing.append(v)
                got += 1
    d["normals"] = {n: list(v) for n, v in normals.items()}
    d["closing"] = [list(v) for v in closing]
    d["depth"] = {n: _half(rng.uniform(*DEPTH)) for n in "ABCD"}
    d["closing_depth"] = [_half(rng.uniform(*DEPTH)) for _ in closing]
    d["bed_depth"] = _half(rng.uniform(*BED_DEPTH))
    d["shift"] = {n: [0.0, 0.0] for n in "ABCD"}  # room is tight: each fixture sits at its roomiest point
    d["load_kg"] = {a: draw_stated_load(rng) for a, _ in ARRANGEMENTS}
    d["frames"] = [{"quat": [rng.gauss(0.0, 1.0) for _ in range(4)],
                    "origin": [_half(rng.uniform(-150.0, 150.0)) for _ in range(3)]} for _ in range(2)]
    return d


def _halfspace(n, depth) -> bd.Part:
    """Everything on the inner side of the plane at ``depth`` along ``n``."""
    return bd.Plane(origin=tuple(depth * v for v in n), z_dir=tuple(n)) * bd.Pos(0, 0, -500.0) * bd.Box(2000.0, 2000.0, 1000.0)


def _facet(body: bd.Part, n, depth):
    for f in body.faces():
        if f.geom_type != bd.GeomType.PLANE:
            continue
        if _dot(tuple(f.normal_at()), n) > 1 - 1e-9 and abs(_dot(tuple(f.center()), n) - depth) < 1e-6:
            return f
    return None


def _inset(face: bd.Face, by: float):
    try:
        got = bd.offset(bd.Face(face.outer_wire()), amount=-by, kind=bd.Kind.INTERSECTION)
    except Exception:  # noqa: BLE001 - a facet too small to set in
        return None
    fs = got.faces() if got is not None else []
    return fs[0] if fs else None


def _roomiest(face: bd.Face):
    """The point of a facet farthest from its edges (near enough: the centre of
    the facet set in as far as it will go, in 1 mm steps)."""
    lo, hi = 0.0, 200.0
    best = tuple(face.center())
    while hi - lo > 1.0:
        mid = (lo + hi) / 2
        f = _inset(face, mid)
        if f is None or f.area < 1e-6:
            hi = mid
        else:
            lo, best = mid, tuple(f.center())
    return best


def _inside(disc_r: float, centre, normal, face) -> bool:
    """True if the disc of radius ``disc_r`` round ``centre`` lies on ``face``."""
    disc = bd.Face(bd.Wire.make_circle(disc_r, bd.Plane(origin=centre, z_dir=normal)))
    return sum(x.area for x in (disc - face).faces()) <= 1e-3


class Reject(ValueError):
    """The draw cannot be used; the reason says why (for the generator's tally)."""


def _outline_edges(itf, inset: float):
    """The fixture's face outline set in by ``inset``, as straight edges: per
    edge its midpoint and outward direction, in the fixture's local x, y. A
    round face becomes a polygon inside its circle."""
    if itf.kind == "round":
        r = itf.face_w / 2 - inset
        k = ROUND_SIDES
        out = []
        for i in range(k):
            a0, a1 = 2 * math.pi * i / k, 2 * math.pi * (i + 1) / k
            am = (a0 + a1) / 2
            out.append(((r * (math.cos(a0) + math.cos(a1)) / 2, r * (math.sin(a0) + math.sin(a1)) / 2),
                        (math.cos(am), math.sin(am))))
        return out
    hw, hh = itf.face_w / 2 - inset, itf.face_h / 2 - inset
    return [((hw, 0.0), (1.0, 0.0)), ((-hw, 0.0), (-1.0, 0.0)), ((0.0, hh), (0.0, 1.0)), ((0.0, -hh), (0.0, -1.0))]


def _walls(d: dict):
    """(normal, outer depth) of every wall: the four mounts, then the closing walls."""
    out = [(tuple(d["normals"][n]), d["depth"][n]) for n in d.get("mounts", "ABCD")]
    return out + [(tuple(v), dep) for v, dep in zip(d["closing"], d["closing_depth"])]


def layout(d: dict) -> dict:
    """Everything about the draw in the part frame: the shell, each pad and
    fixture placement. Raises Reject if the draw cannot be used.

    ``d["mounts"]`` names the mounts (default ABCD) and ``d["pairs"]`` the
    arrangements (default AB, CD): the facing situation uses one pair, AB."""
    names = d.get("mounts", "ABCD")
    pairs = d.get("pairs", ("AB", "CD"))
    itf = {n: make_interface(d[n]["kind"], d[n]["params"]) for n in names}
    normals = {n: tuple(d["normals"][n]) for n in names}
    walls = _walls(d)
    for i, (a, _) in enumerate(walls):
        if abs(a[2]) > NZ_MAX + 1e-9:
            raise Reject("a wall would overhang")
        for b, _ in walls[i + 1:]:
            if _angle(a, b) < MIN_SEP:
                raise Reject("two walls too close in direction")
    def shell(ws):
        o = bd.Box(400.0, 400.0, 400.0)
        for v, dep in ws:
            o = o & _halfspace(v, dep)
        o = o & _halfspace(BED, d["bed_depth"])
        if o is None or len(o.solids()) != 1:
            raise Reject("the walls do not close the shell")
        o = o.solids()[0]
        if max(o.bounding_box().size) > 300.0:
            raise Reject("the walls do not close the shell")
        return o

    # the rim must not be undercut: wherever an upward-facing wall reaches the bed, a skirt wall
    # under it (same compass direction, facing down) cuts that corner off
    outer = shell(walls)
    for _ in range(4):
        bed = _facet(outer, BED, d["bed_depth"])
        if bed is None:
            raise Reject("no bed")
        bad = [v for v, dep in walls if v[2] > 0.02 and (f := _facet(outer, v, dep)) is not None
               and any(e1.is_same(e2) for e1 in f.edges() for e2 in bed.edges())]
        if not bad:
            break
        for v in bad:
            h = math.hypot(v[0], v[1])
            sn = _unit((v[0] / h * 0.94, v[1] / h * 0.94, -0.35))
            far = max(_dot(tuple(x), sn) for x in outer.vertices())
            walls.append((sn, round(far - RIM_CUT, 3)))
        outer = shell(walls)
    else:
        raise Reject("an upward-facing wall reaches the rim")
    # no outside edge sharper than square (the material would thin to nothing along it, which the
    # printing rules' wall measure sees as too thin): each such edge is chamfered, on the outside
    # only, the cavity left as it is
    chamfers = []
    for _ in range(CHAMFER_ROUNDS):
        allw = walls + chamfers + [(BED, d["bed_depth"])]
        fs = [(v, dep, _facet(outer, v, dep)) for v, dep in allw]
        sharp = None
        for i, (v1, d1, f1) in enumerate(fs):
            for v2, d2, f2 in fs[i + 1:]:
                if f1 is None or f2 is None or _angle(v1, v2) <= EDGE_MAX:
                    continue
                if any(e1.is_same(e2) for e1 in f1.edges() for e2 in f2.edges()):
                    sharp = (v1, d1, v2, d2)
                    break
            if sharp:
                break
        if sharp is None:
            break
        v1, d1, v2, d2 = sharp
        half = math.radians(_angle(v1, v2)) / 2
        b = _unit([v1[i] + v2[i] for i in range(3)])
        if b[2] < NZ_MIN:
            raise Reject("a chamfer would overhang")
        # the chamfer's plane, CHAMFER mm in from the edge (measured along its own normal)
        far = max(_dot(tuple(x), b) for x in outer.vertices())
        edge_at = (d1 + d2) / (2 * math.cos(half))
        chamfers.append((b, round(min(far, edge_at) - CHAMFER, 3)))
        outer = shell(walls + chamfers)
    else:
        raise Reject("an edge sharper than square")
    inner = bd.Box(400.0, 400.0, 400.0)
    for v, dep in walls:
        inner = inner & _halfspace(v, dep - WALL)
    if inner is None or len(inner.solids()) != 1:
        raise Reject("the walls do not close the shell")
    inner = inner.solids()[0]
    # the last FOOT mm above the bed the cavity runs straight down: the rim's corners are square
    # (a sloping wall cut by the bed leaves a sharp corner, too thin to print)
    upper = inner & _halfspace(BED, d["bed_depth"] - FOOT)
    sec = _facet(upper, BED, d["bed_depth"] - FOOT) if upper is not None else None
    if sec is None:
        raise Reject("the cavity does not reach the rim")
    inner = (upper + bd.extrude(sec, amount=FOOT + 2.0, dir=(0.0, 0.0, -1.0))).solids()[0]
    pl = {}
    for n in names:
        f = _facet(outer, normals[n], d["depth"][n])
        g = _facet(inner, normals[n], d["depth"][n] - WALL)
        if f is None or g is None:
            raise Reject("a mount wall leaves no face")
        # centred where the wall's outer face and its inner face (seen square to it) overlap most:
        # the pad stands on the one, the heads sit on the other
        both = f & g.moved(bd.Location(tuple(WALL * v for v in normals[n])))
        both = [h for h in (both.faces() if both is not None else []) if h.area > 1e-6]
        if not both:
            raise Reject("a mount wall's faces do not overlap")
        z = _neg(normals[n])  # the fixture's outward normal points into the part
        x0 = _unit(_cross(z, min(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), key=lambda e: abs(_dot(e, z)))))
        c = _roomiest(max(both, key=lambda h: h.area))
        c = tuple(c[i] + PAD_H * normals[n][i] for i in range(3))  # the pad's face, PAD_H proud of the wall
        room = _inset(f, 1.0)
        plane = bd.Plane(origin=tuple(d["depth"][n] * v for v in normals[n]), z_dir=normals[n])
        for k in range(12):  # from the drawn turn, in 15 deg steps, the first in which the pad fits its wall
            r = rot_axis_angle(z, d[n]["yaw"] + 15.0 * k)
            x = tuple(sum(r[i][j] * x0[j] for j in range(3)) for i in range(3))
            y = _cross(z, x)
            cand = Placement(c, tuple((x[i], y[i], z[i]) for i in range(3)))
            try:
                piece = _pad_solid(itf[n], cand, normals[n], d["depth"][n])
            except Reject:
                continue
            foot = piece & (plane * bd.Rectangle(1000.0, 1000.0)).face()
            if foot is not None and room is not None and sum(x_.area for x_ in (foot - room).faces()) <= 1e-3:
                pl[n] = cand
                break
        else:
            raise Reject("a pad does not fit on its wall")
    body = outer - inner
    if body is None or len(body.solids()) != 1:
        raise Reject("the shell is not one solid")
    body = body.solids()[0]
    for n in names:  # each pad stands wholly on its own wall's outer face, clear of its edges (above)
        body = body + _pad_solid(itf[n], pl[n], normals[n], d["depth"][n])
        if len(body.solids()) != 1:
            raise Reject("a pad does not join the shell")
        body = body.solids()[0]
    # printing: where two downward-facing walls meet outside, the edge is steep enough to print
    faces = [(v, _facet(outer, v, dep)) for v, dep in walls]
    for i, (v1, f1) in enumerate(faces):
        for v2, f2 in faces[i + 1:]:
            if f1 is None or f2 is None or v1[2] >= 0 or v2[2] >= 0:
                continue  # a keel is an outside edge between two downward faces (inside, the shell's
                # upward walls meet in an inside corner, which prints)
            if not any(e1.is_same(e2) for e1 in f1.edges() for e2 in f2.edges()):
                continue
            e = _unit(_cross(v1, v2))
            if abs(e[2]) < KEEL_MIN:
                raise Reject("a keel too flat to print")
    facets = {}
    for n in names:
        f = _facet(body, normals[n], d["depth"][n] + PAD_H)
        if f is None:
            raise Reject("a pad lost its face")
        b = BOLTS[itf[n].bolt]
        z = pl[n].vec((0.0, 0.0, 1.0))
        for hx, hy in itf[n].holes:
            p = pl[n].point((hx, hy, 0.0))
            if not _inside(b.head_d / 2 + RING_ROOM, p, z, f):  # the clamped ring on the pad
                raise Reject("pad too small for the mount")
            # the head, with its clearance and some air, inside the shell's cavity
            q = tuple(p[i] + _thick(itf[n]) * z[i] for i in range(3))
            head = bd.Plane(origin=q, z_dir=z) * bd.Pos(0, 0, (b.head_h + 3.0) / 2 + 0.01) \
                * bd.Cylinder(b.head_d / 2 + 1.5, b.head_h + 3.0)
            out = head - inner
            if out is not None and sum(x.volume for x in out.solids()) > 1e-3:
                raise Reject("no room for a bolt head inside the shell")
        facets[n] = {"face": f}
    for a, b in pairs:  # the two fixtures of one arrangement do not overlap
        sa = fixed_part_solid(itf[a]).moved(pl[a].location())
        sb = fixed_part_solid(itf[b]).moved(pl[b].location())
        inter = sa & sb
        if inter is not None and sum(s.volume for s in inter.solids()) > 1e-3:
            raise Reject("the two fixtures of an arrangement overlap")
    dist = [math.dist(pl[a].origin, pl[b].origin) for a, b in pairs]
    if len(dist) == 2 and abs(dist[0] - dist[1]) < DIST_APART:
        raise Reject("the two arrangements' mounts are as far apart")
    return {"itf": itf, "body": body, "placements": pl, "facets": facets, "t": {n: _thick(itf[n]) for n in names},
            "normals": normals, "distances": dist}


def _pad_solid(itf, pl: Placement, n, depth: float) -> bd.Part:
    """A mount's pad: its face is the fixture's face set in by INSET, PAD_H proud
    of the wall's outer face; its sides slope down to the middle of the wall,
    as steep as printing allows on each edge."""
    pad = _halfspace(n, depth + PAD_H) & _halfspace(_neg(n), -(depth - WALL / 2))
    for m, u in _outline_edges(itf, INSET):
        w = pl.vec((u[0], u[1], 0.0))
        for beta in BEVELS:
            nb = _unit([math.cos(math.radians(beta)) * n[i] + math.sin(math.radians(beta)) * w[i] for i in range(3)])
            if nb[2] >= NZ_MIN + 0.02:
                break
        else:
            raise Reject("a pad side would overhang")
        p0 = pl.point((m[0], m[1], 0.0))
        pad = pad & _halfspace(nb, _dot(p0, nb))
    return pad


def build_reference(lay: dict, *, hole_shift: float = 0.0) -> bd.Part:
    """The reference part in the part frame: the shell with its pads, a
    clearance hole through pad and wall at every bolt, room for spigots.

    hole_shift  move C's holes this far along C's local x (a broken copy)."""
    body = lay["body"]
    for n, itf in lay["itf"].items():
        loc = lay["placements"][n].location()
        t = lay["t"][n]
        clear = BOLTS[itf.bolt].clear_medium / 2
        for x, y in itf.holes:
            dx = hole_shift if n == "C" else 0.0
            body = body - (bd.Pos(x + dx, y, t / 2) * bd.Cylinder(clear, t + 2.0)).moved(loc)
        for p in itf.protrusions:  # room for a spigot
            h = p.height + 1.0
            body = body - (bd.Pos(p.x, p.y, h / 2 - 1.0) * bd.Cylinder(p.radius + 0.5, h + 2.0)).moved(loc)
    solids = body.solids()
    return solids[0] if len(solids) == 1 else body


def frames(d: dict) -> list[Placement]:
    """Each arrangement's frame: where the part frame lands in that
    arrangement's coordinates (the reference placement)."""
    return [Placement(tuple(f["origin"]), rot_from_quaternion(*f["quat"])) for f in d["frames"]]


def task_from_draw(seed: int, d: dict, lay: dict | None = None) -> Task:
    lay = lay or layout(d)
    g = frames(d)
    poses = [{n: lay["placements"][n].then(g[k]) for n in names} for k, (_, names) in enumerate(ARRANGEMENTS)]
    meta = {"generator": "adapt-angled",
            "arrangements": [{"name": a, "pose": k, "parts": list(names)} for k, (a, names) in enumerate(ARRANGEMENTS)],
            "answer": {"files": ["output.step", PLACEMENT_FILE]}}
    task = Task(f"adapts-angled-s{seed}", LEVEL, seed, dict(lay["itf"]), poses, dict(TOLERANCES), meta,
                generator_version=GENERATOR_VERSION, family="adapt")
    task.tolerances["required_checks"] = list(TOLERANCES["required_checks"]) + ["STIFFNESS"]
    _state_loads(task, d)
    return task


def _state_loads(task: Task, d: dict) -> None:
    """The stated load of each arrangement (level 3's rule, in PLA): its first
    fixed part held, the other carrying the drawn mass. Stored with the
    arrangement; the grader gives it to that arrangement's checks."""
    from ...requirements import apply_stated_load
    from .grading import sub_task

    for arr in task.meta["arrangements"]:
        sub = sub_task(task, dict(arr, stiffness=None))
        apply_stated_load(sub, {LOADED[arr["name"]]: d["load_kg"][arr["name"]]})
        arr["stiffness"] = sub.stiffness


def _rel_signature(a: Placement, b: Placement, itf_b) -> list:
    """B's bolts as seen from A's face: distance from A's normal axis, height
    above A's face. Unchanged by turning A about its own normal."""
    inv_r = tuple(tuple(a.rot[j][i] for j in range(3)) for i in range(3))
    out = []
    for x, y in itf_b.holes:
        p = b.point((x, y, 0.0))
        q = [p[i] - a.origin[i] for i in range(3)]
        loc = [sum(inv_r[i][k] * q[k] for k in range(3)) for i in range(3)]
        out.append((round(math.hypot(loc[0], loc[1]), 1), round(loc[2], 1)))
    return sorted(out)


def trivial(lay: dict) -> bool:
    """True if one arrangement is the other again: the same two mounts at the
    same relative pose, so one set of features would serve both."""
    itf, pl = lay["itf"], lay["placements"]
    same = lambda m, n: itf[m].kind == itf[n].kind and itf[m].params == itf[n].params  # noqa: E731

    def close(s, r):
        return len(s) == len(r) and all(abs(a - c) <= 0.5 and abs(b - e) <= 0.5 for (a, b), (c, e) in zip(s, r))

    for p, q in (("C", "D"), ("D", "C")):
        if same("A", p) and same("B", q) and close(_rel_signature(pl["A"], pl["B"], itf["B"]),
                                                   _rel_signature(pl[p], pl[q], itf[q])):
            return True
    return False


def _frames_ok_at(task: Task, k: int) -> bool:
    return all(max(abs(v) for v in f.normal) <= AXIS_MAX for f in task.faces(k))


def _frames_ok(task: Task) -> bool:
    return all(_frames_ok_at(task, k) for k in range(len(task.poses)))


def reference_placements(d: dict) -> dict:
    """The reference's placement file: per arrangement, rotation and translation."""
    return {a: {"rotation": [list(r) for r in g.rot], "translation": list(g.origin)}
            for (a, _), g in zip(ARRANGEMENTS, frames(d))}


def generate(seed: int, knobs: dict | None = None, max_tries: int = 400, tally: dict | None = None):
    """Return (task, draw, reference solid). Deterministic in ``seed``. The
    reference solid is in the part frame; its placements are
    ``draw["placement"]``. ``tally`` (a dict) counts why draws were rejected."""
    from .grading import grade_with_placement

    tally = tally if tally is not None else {}

    def no(why):
        tally[why] = tally.get(why, 0) + 1

    rng = random.Random(f"adapts-angled-{seed}")
    for _ in range(max_tries):
        d = _draw(rng)
        if abs(d["angles"][0] - d["angles"][1]) < ANGLE_APART:
            no("the two angles too alike")
            continue
        try:
            lay = layout(d)
        except Reject as e:
            no(str(e))
            continue
        if trivial(lay):
            no("trivial")
            continue
        task = task_from_draw(seed, d, lay)
        for _ in range(30):  # turn the frames again until no face is near a coordinate plane
            if _frames_ok(task):
                break
            no("a face near a coordinate plane (frame redrawn)")
            d["frames"] = [f if _frames_ok_at(task, k) else {"quat": [rng.gauss(0.0, 1.0) for _ in range(4)],
                                                             "origin": f["origin"]}
                           for k, f in enumerate(d["frames"])]
            task = task_from_draw(seed, d, lay)
        if not _frames_ok(task):
            continue
        try:
            ref = build_reference(lay)
        except ValueError:  # a boolean left nothing to cut from
            no("reference could not be built")
            continue
        if len(ref.solids()) != 1 or not fits_build_volume(ref, BUILD_ROOM):
            no("reference not one solid or too big for the printer")
            continue
        d["placement"] = reference_placements(d)
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "reference.step")
            write_step(ref, p)
            # the cheap checks first (fit, clean, printing); the load analysis only on a part that passes them
            rep = grade_with_placement(task, p, d["placement"], optional=False, score=False,
                                       skip=frozenset({"STIFFNESS"}))
            if rep.passed:
                rep = grade_with_placement(task, p, d["placement"], optional=False, score=False,
                                           skip=frozenset({"CLEAN", "MANUFACTURING", "SEAT_INSIDE"}))
        if not rep.passed:
            no("reference fails: " + ", ".join(sorted(set(rep.failed()))))
            continue
        a1, a2 = d["angles"]
        m = d["load_kg"]
        d["summary"] = (f"arrangement 1: faces at {a1:g} deg, {lay['distances'][0]:.0f} mm apart, {m['1']:g} kg on B; "
                        f"arrangement 2: {a2:g} deg, {lay['distances'][1]:.0f} mm apart, {m['2']:g} kg on D")
        d["distances"] = [round(v, 3) for v in lay["distances"]]
        return task, d, ref
    raise RuntimeError(f"seed {seed}: no valid adapts draw in {max_tries} tries ({tally})")


def build_connector(task: Task, d: dict, **variant) -> bd.Part:
    return build_reference(layout(d), **variant)


SELFTEST_VARIANTS: list = []  # the adapt family's broken copies are in tests/test_adapt.py
