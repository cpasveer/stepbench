"""Clamp a pipe, level 4 (assembles): two parts, put in one after the other
through one small opening in a closed box, bolted together inside.

The fixed parts: a pipe and a part with a flat face (the version 1 pipe
setup: ``setup``, from ``round5``; which is A and which B does not matter).
A closed box stands round
both: the pipe runs in through its bottom and out through its top (round
holes with a small clearance; the pipe is long enough to pass both walls);
four side walls, one with the opening. Everything of level 3 holds per part
(fit, clean, key turn, printable in PLA) and for the assembly (the stated
load, the plastic round the joint bolts, stiffness); level 4 adds the ways
in, the joint (catalogue through bolts and nuts: the model states size,
count and length) and a key reaching every joint bolt from outside.

Frames. ``e3`` is the flat face's normal (towards the pipe), ``t`` the
pipe's axis, ``e1`` square to both (the joint's bolts run along it; the
opening is in the box's +e1 wall), ``e2 = e3 x e1``. Part 2 is drawn in the
plane square to the pipe: ``x = e1``, ``y = yb = t x e1`` (towards the flat
face).

The hidden reference, two parts, each printable by how it is made:

* part 1, on the flat face (printed lying on it): a plate of the face's own
  outline with its bolt holes, a hole for a spigot if there is one, a crown
  built up on the plate wherever nothing passes (round its heads, clear of
  part 2's arm and of part 1's own way in), and an upright ear square to e1
  that carries the joint; over a spigot the ear's underside is a 40 deg cone;
* part 2, on the pipe (printed standing on its end): a saddle on the pipe's
  bolt side, a flat seat square to each bolt row, and an arm out to the ear.
  The arm is found, not drawn: a bar along the crest is searched to the joint
  between the keys' sweeps (the pipe bolts' and the flat part's), then grown
  into the free room round it -- past the joint face too, where that stays
  above all of part 1 and its way in -- and traced as one clean outline
  (nothing thinner than 2 mm, nothing at a glancing angle). It spans the
  joint along the pipe, run out to the saddle's nearer end.

Part 2 goes in first, lifted off the pipe towards the flat face just enough
that its shorter leading wrap clears the pipe, then moves onto it; part 1
goes in, ``ONTO`` mm off the flat face plus any spigot, and moves onto it,
its ear meeting the arm at the joint face.
"""
from __future__ import annotations

import json
import math
import os.path

import build123d as bd
import numpy as np

from ...catalogue import BOLTS, NUTS, lengths
from ...geometry import Placement
from ...task import Obstacle, Task

LEVEL = 4
JOINT_UP = 30.0     # mm the joint stands higher on part 1's ear than its lowest (see build)
WALL = 6.0          # mm, the box's walls
CLEAR = 2.5         # mm, the opening beyond what passes it, each side
PIPE_CLEAR = 0.5    # mm, the pipe's holes in the box's walls, radially
ONTO = 2.0          # mm part 1 comes the last way onto A's face, beyond A's spigot
INSET = 1.5         # mm a seat stays inside its fixed part's face
CONE = 40.0         # deg, the spigot pocket's roof from the vertical of its print (at most 45 prints)
GRIP_PIPE = 9.0     # mm under the pipe bolts' heads
SADDLE_WRAP = 70.0  # deg the saddle wraps round the pipe on its trailing side (from its crest)
SADDLE_LEAD = 50.0  # deg on its leading side, the side it slides in toward: less, so a short lift clears the pipe
KEY_MARGIN = 1.0    # mm between the arm and any key's sweep
CROWN = 18.0        # mm above A's face part 1's crown rises to (an L key's long leg swings above it)
BAR = 9.5           # mm: the arm's path to the joint is searched for a bar this thick
WELL = 6.0          # mm radius kept clear round each of A's heads in the crown
TOOL = 30.0         # mm a straight key needs beyond a head (the task's tool length)
RING = 3.0          # mm of seat round a head (or nut) beyond its diameter
MARGIN = {"t": 30.0, "e1-": 40.0, "e1+": 30.0, "s": 40.0}  # the box beyond what it holds


def _u(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _rot(e1, e2, e3):
    """Rotation (row by row) whose columns are e1, e2, e3."""
    m = np.stack([e1, e2, e3], axis=1)
    return tuple(tuple(float(m[i][j]) for j in range(3)) for i in range(3))


def _r(v):
    return [round(float(x), 6) for x in v]


def _points(shape, tol=0.2) -> np.ndarray:
    if not hasattr(shape, "tessellate"):  # a list of shapes (what an intersection may give)
        shape = bd.Compound(list(shape))
    v, _ = shape.tessellate(tol)
    return np.array([tuple(p) for p in v])


def _through(points: np.ndarray, d, n, off) -> np.ndarray:
    """Where points moving along d cross the plane x . n = off."""
    t = (off - points @ n) / float(np.asarray(d) @ n)
    return points + np.outer(t, d)


# --------------------------------------------------------------------------
# the fixed parts


def roles(task: Task) -> tuple[str, str]:
    """(the flat fixed part's name, the pipe's)."""
    pipe = next(n for n, itf in task.parts.items() if itf.kind == "pipe")
    return next(n for n in task.parts if n != pipe), pipe


def frames(task: Task) -> dict:
    flat, pipe = roles(task)
    fa = next(f for f in task.faces(0) if f.part == flat)
    e3 = _u(fa.normal)
    pb = task.poses[0][pipe]
    t = _u(pb.vec((1.0, 0.0, 0.0)))
    if t[2] < 0:
        t = -t  # the pipe's axis, upwards
    e1 = _u(np.cross(t, e3))
    e2 = np.cross(e3, e1)
    crest = _u(pb.vec((0.0, 0.0, 1.0)))
    q = _u(np.cross(t, e1))
    oa = np.array(fa.origin, float)
    if q @ (oa - np.array(pb.origin)) < 0:
        q = -q  # in the pipe's cross-section, towards A
    return {"e1": e1, "e2": e2, "e3": e3, "t": t, "q": q, "crest": crest, "OA": oa,
            "C": np.array(pb.origin, float), "R": task.parts[pipe].face_h / 2, "flat": flat, "pipe": pipe}


def _face_shape(task: Task, F: dict):
    """inside(x, y, inset): whether points in the flat face's plane (x along e1, y along e2 from its
    centre) lie inside its outline by ``inset``; and the plate of that outline, ``inset`` in, as a
    solid ``h`` thick on the face."""
    itf = task.parts[F["flat"]]
    pose = task.poses[0][F["flat"]]
    e1, e2, e3, oa = F["e1"], F["e2"], F["e3"], F["OA"]
    if itf.kind == "round":
        r = itf.face_w / 2

        def inside(x, y, inset):
            return np.hypot(x, y) <= r - inset

        def plate(inset, h):
            return _plane(oa, e1, e3) * bd.Pos(0, 0, h / 2) * bd.Cylinder(r - inset, h)
        return inside, plate
    lx = _u(np.asarray(pose.vec((1.0, 0.0, 0.0))))
    lx = _u(lx - e3 * float(lx @ e3))
    ly = np.cross(e3, lx)
    cx, cy = float(lx @ e1), float(lx @ e2)  # the face's own x in (e1, e2)

    def inside(x, y, inset):
        u, v = x * cx + y * cy, -x * cy + y * cx
        return (np.abs(u) <= itf.face_w / 2 - inset) & (np.abs(v) <= itf.face_h / 2 - inset)

    def plate(inset, h):
        return _plane(oa, lx, e3) * bd.Pos(0, 0, h / 2) * bd.Box(itf.face_w - 2 * inset, itf.face_h - 2 * inset, h)
    del ly
    return inside, plate


# --------------------------------------------------------------------------
# the reference


def _plane(origin, x, z) -> bd.Plane:
    return bd.Plane(origin=tuple(float(v) for v in origin), x_dir=tuple(float(v) for v in x),
                    z_dir=tuple(float(v) for v in z))


def _profile(plane: bd.Plane, pts2d, t0: float, t1: float) -> bd.Part:
    face = bd.Face(bd.Wire.make_polygon([plane.from_local_coords((x, y, t0)) for x, y in pts2d], close=True))
    return bd.extrude(face, amount=t1 - t0, dir=tuple(plane.z_dir))


def build(task: Task, *, size: str = "M5", length: float = 35.0, joint_up: float = JOINT_UP,
          variant: dict | None = None):
    """(part 1, part 2, assembly file, dimensions). ``size``, ``length``: the
    joint bolts' (the length sets the ear's and the arm's thickness: the
    thicker, the stiffer). ``joint_up``: how much higher than its lowest the joint stands on part 1's
    ear (the higher, the shorter part 2's arm: the two parts meet half way, and the box's opening,
    drawn round what passes it, is smaller). ``variant`` makes the broken copies."""
    var = dict(variant or {})
    F = frames(task)
    e1, e2, e3, t, q, oa, C, R = (F[k] for k in ("e1", "e2", "e3", "t", "q", "OA", "C", "R"))
    b, nut = BOLTS[size], NUTS[size]
    tj = (length - nut.m - 1.5 * nut.pitch) / 2  # ear and arm: the bolt stands out of its nut 1.5 pitches
    if not (b.grip_min <= tj <= b.grip_max):
        raise ValueError("joint plates out of range for this bolt length")
    flat, pipe_n = F["flat"], F["pipe"]
    A_itf = task.parts[flat]
    ba = BOLTS[A_itf.bolt]
    inside, plate = _face_shape(task, F)
    r_face = max(A_itf.face_w, A_itf.face_h) / 2  # (a bound: what is inside, ``inside`` says)
    tp = max(8.0, ba.grip_min)
    # ---- part 1: a plate on the flat face and an ear standing on it (its frame: x = e1, y = e2, z = e3)
    fa = _plane(oa, e1, e3)
    disc = plate(INSET, tp)
    a_bolts = [bt for bt in task.bolts(0) if bt.part == flat]
    room = ba.head_d / 2 + DEFAULT_RADIAL + 0.2  # a head's room (the grader's), and a hair more
    xy = [(float((np.asarray(bt.position) - oa) @ e1), float((np.asarray(bt.position) - oa) @ e2)) for bt in a_bolts]
    # the arm comes down a lane between the bolt heads level with the joint (clear of them and of their
    # keys), the ear beside it: the free gap across x nearest the middle wide enough for both
    ring = max(b.head_d, nut.e) / 2 + RING
    blocked = sorted((x - room, x + room) for x, y in xy if abs(y) < ring + room)
    xx = np.arange(-r_face, r_face, 0.1)
    xin = xx[inside(xx, np.zeros_like(xx), INSET)]  # the face across, along e1 through its centre
    gaps, edge = [], float(xin.min())
    for lo, hi in blocked:
        if lo > edge:
            gaps.append((edge, lo))
        edge = max(edge, hi)
    gaps.append((edge, float(xin.max())))
    gaps = [g for g in gaps if g[1] - g[0] >= 2 * tj + 0.2]
    if not gaps:
        raise ValueError("no lane between A's bolt heads for the arm")
    g = min(gaps, key=lambda g: abs((g[0] + g[1]) / 2))
    # the joint face: the arm on its -x side (centred in the lane if it can be), the ear on its +x side
    xj = round(min((g[0] + g[1]) / 2 + tj / 2, g[1] - tj - 0.1), 3)
    # the ear beside the lane, as wide as A's heads and the disc allow
    near = [abs(y) - room for x, y in xy if x + room > xj and x - room < xj + tj]
    hw = 0.0  # as wide as the face allows: every corner of the ear's footprint inside it
    while hw < r_face and all(inside(np.array([x_]), np.array([y_]), INSET) for x_ in (xj, xj + tj)
                              for y_ in (hw + 0.1, -hw - 0.1)):
        hw += 0.1
    half_w = min(near + [hw]) - 0.5
    if half_w < ring:
        raise ValueError("no room for the ear between A's bolt heads")
    if var.get("narrow_ear"):  # broken copy: 1 mm of plastic beside the joint holes
        half_w = b.clear_medium / 2 + 1.0
    ear_w = 2 * half_w
    spig_h = max([p.height for p in A_itf.protrusions] or [0.0])
    onto = ONTO + spig_h
    arm_low = tp + onto + 1.0 + joint_up  # the arm's end: above the plate even before part 1's last move
    pitch = b.head_d + 1.0
    z1 = arm_low + ring
    z2 = z1 + pitch
    ear_top = z2 + ring
    spig = max([p.radius for p in A_itf.protrusions] or [0.0])
    hp = spig_h + 1.0 if spig else tp  # the spigot's pocket: through the disc, roofed only under the ear
    ear = fa * bd.Pos(xj + tj / 2, 0, (hp + ear_top) / 2) * bd.Box(tj, ear_w, ear_top - hp)
    flare = float(var.get("ear_flare", 0.0))
    if flare > 0:  # above the keys of the flat part's bolts the ear may widen, at 45 deg (it prints)
        z_f = tp + ba.head_h + TOOL + 2.0
        if z_f < ear_top - 1.0:
            w_top = half_w + min(flare, ear_top - z_f)
            fy = bd.Plane(origin=tuple(oa + e1 * xj), x_dir=tuple(e2), z_dir=tuple(e1))
            sec = [(-half_w, z_f), (half_w, z_f), (w_top, z_f + (w_top - half_w)), (w_top, ear_top),
                   (-w_top, ear_top), (-w_top, z_f + (w_top - half_w))]
            # (in this plane local y is e1 x e2 = e3: the height above the flat face)
            face = bd.Face(bd.Wire.make_polygon([fy.from_local_coords((y, z, 0.0)) for y, z in sec], close=True))
            ear = ear + bd.extrude(face, amount=tj, dir=tuple(e1))
    part1 = disc + ear
    for bt in a_bolts:
        part1 = part1 - bd.Plane(origin=bt.position, z_dir=bt.axis) * bd.Pos(0, 0, tp / 2) \
            * bd.Cylinder(BOLTS[bt.size].clear_medium / 2, tp + 2)
    if spig:  # the pocket: a hole through the disc; under the ear its roof is a cone (the part prints
        # standing on A's face, and a cone needs no support)
        rp = spig + 0.5
        cone_h = rp / math.tan(math.radians(CONE))
        through = fa * bd.Pos(0, 0, tp / 2) * bd.Cylinder(rp, tp + 2.0)
        under_ear = fa * bd.Pos(xj + tj / 2, 0, tp / 2) * bd.Box(tj, ear_w, tp + 2.0)
        pocket = (through - under_ear) + fa * bd.Pos(0, 0, hp / 2 - 0.5) * bd.Cylinder(rp, hp + 1.0)
        pocket = pocket + fa * bd.Pos(0, 0, hp + cone_h / 2) * bd.Cone(rp, 0.0, cone_h)
        part1 = part1 - pocket
    # the joint bolts: on the joint face, in the ear's middle, at z1 and z2, along e1
    pos = [oa + e1 * xj + e3 * z for z in (z1, z2)]
    holes = [] if var.get("no_joint") else pos  # no_joint: level 5 joins the two by a lock instead
    for p in holes:
        part1 = part1 - bd.Plane(origin=tuple(p), z_dir=tuple(e1)) * bd.Pos(0, 0, tj / 2) \
            * bd.Cylinder(b.clear_medium / 2, tj + 2.0)
    # ---- part 2: one profile square to the pipe, extruded along it (x = e1, y = q, z = t)
    fb = _plane(C, e1, t)
    yb = _u(np.cross(t, e1))  # the profile plane's own second axis
    crest = F["crest"]
    cr = np.array([crest @ e1, crest @ yb])
    ang0 = math.atan2(cr[1], cr[0])
    rs = R + GRIP_PIPE
    pts = []
    rows = sorted({float(a_) for _, a_ in task.parts[pipe_n].params["holes"]})  # deg from the crest
    # the leading wrap: just past the outer bolts' bearing rings, at most SADDLE_LEAD (the lift it needs)
    seat_ = BOLTS[task.parts[pipe_n].bolt].head_d / 2 + RING
    corner = max(abs(r_) for r_ in rows) + math.degrees(math.atan2(seat_, rs))  # the outer seat's corner
    # as far as the lift allows: part 2 comes in lifted off the pipe along its crest, towards the flat face
    room_ = (arm_low - spig_h - 1.0) / max(abs(float(F["crest"] @ e3)), 1e-3) - 1.0
    lead_max = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - room_ / R)))) - 0.5
    lead_deg = min(SADDLE_LEAD, lead_max, max(corner + 10.5, max(abs(r_) for r_ in rows)
                   + math.degrees((BOLTS[task.parts[pipe_n].bolt].head_d / 2 + 2.0) / R) + 3.0))
    for row in (rows[0], rows[-1]):
        a = ang0 + math.radians(row)
        d, dt = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
        seat = BOLTS[task.parts[pipe_n].bolt].head_d / 2 + RING
        pts += [rs * d + seat * dt, rs * d - seat * dt]
    lead = 1.0
    for wrap in (-1.0, 1.0):
        a = ang0 + math.radians(wrap * SADDLE_WRAP)
        if math.cos(a) < 0:  # the leading side (toward -e1, where it slides in to): wrap less
            lead = wrap
            a = ang0 + math.radians(wrap * lead_deg)
        # both on the pipe and 4 mm off it: the saddle touches the pipe right to its end
        pts.append(R * np.array([math.cos(a), math.sin(a)]))
        # and 4 mm off it, an end face; not where that point would sit all but on the seat's own corner
        corner = max(abs(r_) for r_ in rows) + math.degrees(math.atan2(seat, rs))
        if abs(math.degrees(a - ang0)) - corner > 6.0:
            pts.append((R + 4.0) * np.array([math.cos(a), math.sin(a)]))
    j0 = float((oa - C) @ e1) + xj  # the joint face, in the pipe's frame
    zq = [float((p - C) @ yb) for p in pos]  # the bolts' place across, in the profile plane
    sgn = 1.0 if float(q @ yb) > 0 else -1.0  # which way along yb A lies
    ts = [float((p - C) @ t) for p in pos]
    # the saddle along the pipe: over the bolt rows, and far enough to hold the joint bolts
    xs = [p[0] for p, _ in task.parts[pipe_n].hole_frames]
    tb = [float(task.poses[0][pipe_n].point((x, 0, 0)) @ t - C @ t) for x in xs]
    seat_b = BOLTS[task.parts[pipe_n].bolt].head_d / 2 + RING
    t0 = min(min(tb) - seat_b, min(ts) - ring)
    t1 = max(max(tb) + seat_b, max(ts) + ring)
    # the arm's span: the joint's, run out to the saddle's nearer end (the part prints standing on
    # that end, the arm on the bed with it)
    ta0, ta1 = min(ts) - ring, max(ts) + ring
    end = var.get("arm_end") or ("t1" if t1 - ta1 <= ta0 - t0 else "t0")  # (or the end asked for)
    if end == "t1":
        ta1 = t1
    else:
        ta0 = t0
    # the arm must clear every key's sweep: the pipe bolts' (out along their axes) and A's (up from
    # part 1's disc). Each as (start, end, radius) in the profile plane, only the stretch level with
    # the arm's span along the pipe
    caps = []
    for bt in task.bolts(0):
        bb_ = BOLTS[bt.size]
        ax = np.asarray(bt.axis, float)
        p0 = np.asarray(bt.position, float) + ax * (GRIP_PIPE if bt.part == pipe_n else tp)
        p1 = p0 + ax * (bb_.head_h + TOOL)
        rk = bb_.head_d / 2 + KEY_MARGIN
        tq0, tq1 = float((p0 - C) @ t), float((p1 - C) @ t)
        tk = sorted([tq0, tq1])
        if tk[1] + rk < ta0 or tk[0] - rk > ta1:
            continue
        if abs(tq1 - tq0) > 1e-6:  # only the stretch of the key level with the arm's span along the pipe
            s_ = sorted([(ta0 - rk - tq0) / (tq1 - tq0), (ta1 + rk - tq0) / (tq1 - tq0)])
            s_lo, s_hi = max(0.0, s_[0]), min(1.0, s_[1])
            p0, p1 = p0 + (p1 - p0) * s_lo, p0 + (p1 - p0) * s_hi
        caps.append((np.array([(p0 - C) @ e1, (p0 - C) @ yb]), np.array([(p1 - C) @ e1, (p1 - C) @ yb]), rk))

    z_clear = ear_top + onto + 1.0  # above this (off the flat face), nothing of part 1 passes, ever

    def above(G):
        """Profile points that stay above all of part 1 and its way in, over the whole arm span: there
        the arm may reach past the joint face."""
        G = np.atleast_2d(G)
        z0 = float((C - oa) @ e3) + G[:, 1] * float(yb @ e3)
        return np.minimum(z0 + ta0 * float(t @ e3), z0 + ta1 * float(t @ e3)) >= z_clear

    forbidden = {}

    def _hits(poly, extra=()):
        """Whether a convex outline in the profile meets a key's sweep (or, given, the nuts' room):
        on a map of forbidden points every 0.5 mm, outside the pipe (its inside is cut away)."""
        key = len(extra)
        if key not in forbidden:
            gx, gy = np.meshgrid(np.arange(-90.0, x1 + 20.0, 0.5), np.arange(-10.0, arm_lo + 20.0, 0.5))
            G = np.c_[gx.ravel(), gy.ravel()]
            G = G[np.hypot(G[:, 0], G[:, 1]) > R]
            bad = np.zeros(len(G), bool)
            for a_, b_, rk in list(caps) + list(extra):
                d = b_ - a_
                u = np.clip(((G - a_) @ d) / max(d @ d, 1e-12), 0, 1)
                bad |= np.hypot(*(G - a_ - u[:, None] * d).T) < rk
            # close to the pipe, nothing past the saddle's ends (it would stand out as a stub)
            phi = np.degrees(np.arctan2(G[:, 1], G[:, 0])) - math.degrees(ang0)
            phi = (phi + 180.0) % 360.0 - 180.0
            past = np.where(phi * lead > 0, np.abs(phi) > lead_deg - 6.0, np.abs(phi) > SADDLE_WRAP - 6.0)
            bad |= past & (np.hypot(G[:, 0], G[:, 1]) < R + 14.0)
            bad |= (G[:, 0] > x1) & ~above(G)  # past the joint face only above part 1
            forbidden[key] = G[bad]
        B = forbidden[key]
        P = np.asarray(poly, float)
        # exact: no edge comes within a key's reach (a needle-thin corner slips between map points)
        E0, E1 = P, np.roll(P, -1, axis=0)
        for a_, b_, rk in list(caps) + list(extra):
            if (_seg_dist(E0, E1, a_, b_) < rk).any():
                return True
        lo, hi = P.min(0), P.max(0)
        B = B[(B[:, 0] >= lo[0]) & (B[:, 0] <= hi[0]) & (B[:, 1] >= lo[1]) & (B[:, 1] <= hi[1])]
        if not len(B):
            return False
        inside = np.ones(len(B), bool)
        sg = np.sign(_area(P))
        for i in range(len(P)):  # convex: inside is on the same side of every edge
            a_, b_ = P[i], P[(i + 1) % len(P)]
            inside &= ((b_[0] - a_[0]) * (B[:, 1] - a_[1]) - (b_[1] - a_[1]) * (B[:, 0] - a_[0])) * sg >= -1e-9
        return bool(inside.any())

    # the arm's joint face; face_gap: set back off the ear's (level 5's lock: parts that move
    # against each other keep apart)
    x0, x1 = j0 - tj, j0 - float(var.get("face_gap", 0.0))
    arm_lo = (max(zq) + ring) if sgn > 0 else (min(zq) - ring)
    y_top = (min(zq) - ring) if sgn > 0 else (max(zq) + ring)  # where the joint's rings begin
    # the arm's path: a bar up from the saddle's crest (between the two bolt rows), a slant out to the
    # joint plate, the plate itself. Search where the slant leaves the lane (ys) and where it reaches
    # the plate (yc): the first, lowest pair whose three pieces clear every key
    arm_polys = None
    if sgn > 0:
        w = min(tj, BAR)  # the path is found for a bar this thick; the web then fills out round it
        c2 = cr / np.linalg.norm(cr)  # the lane runs out along the crest, between the bolt rows
        n2 = np.array([-c2[1], c2[0]]) * w / 2

        def at(r_):
            return [tuple(c2 * r_ - n2), tuple(c2 * r_ + n2)]

        for ys in np.arange(R + w, y_top / max(c2[1], 0.3), 0.5):
            lane = at(0.0) + at(ys)[::-1]
            if _hits(lane):
                break  # further out only meets more
            for yc in np.arange(max(float(c2[1] * ys), y_top - w), arm_lo - w + 0.01, 0.5):
                pts_ = np.array(at(ys - w) + at(ys) + [(x1 - w, yc - w), (x1, yc - w), (x1, yc), (x1 - w, yc)])
                slant = [tuple(v) for v in pts_[_hull_idx(pts_)]]
                down = [(x0, yc - w), (x1, yc - w), (x1, arm_lo), (x0, arm_lo)]
                if not (_hits(slant) or _hits(down)):
                    arm_polys = [lane, slant, down]
                    break
            if arm_polys is not None:
                break
    if arm_polys is None:
        raise ValueError("no way for the arm past the keys to the joint")
    # then the room round the bar, for stiffness: not past the joint face, clear of the nuts' room, and
    # high enough that part 2's lift clears the pipe
    lift = R * (1.0 - math.cos(math.radians(lead_deg))) + 1.0
    y_min = R + 1.0 - lift
    nuts_ = [(np.array([x0, y_]), np.array([x0 - nut.m - 3.0 * nut.pitch, y_]), nut.e / 2 + 1.0) for y_ in zq]
    # the arm is trimmed square to A's face at arm_low (above part 1's disc): a plane slanting across
    # the profile along the pipe. Only the joint plate's column reaches past it (cut across its
    # straight sides); everything else stays below its lowest line, so the trim meets nothing at a
    # glancing angle
    ta_lo, ta_hi = max(ta0, t0), min(ta1, t1)
    y_clip = [(arm_low - float((C - oa) @ e3) - tt * float(t @ e3)) / float(yb @ e3) for tt in (ta_lo, ta_hi)]
    y_cap = min(y_clip) - 1.0
    # the column ends in an exact flat top a little below the trim's highest line: the trim then crosses
    # only the column's straight sides and that top (never the saddle's end face, at a sharp angle)
    y_col = max(y_clip) - 3.0
    top = None
    if y_col >= arm_lo + 1.0:
        column = [(x0, yc - w), (x1, yc - w), (x1, y_col), (x0, y_col)]
        if not _hits(column):
            arm_polys[2] = column
            top = [(x0, y_col - 5.0), (x1, y_col - 5.0), (x1, y_col), (x0, y_col)]
    # the free room round the bar is not convex (keys on both sides): convex webs, each grown out from
    # a point along the bar or beside it, joined in the profile plane into one outline
    path = [c2 * R, c2 * (ys - w / 2), np.array([x1 - w / 2, yc - w / 2]),
            np.array([x1 - tj / 2, arm_lo - 2.0])]
    seeds = []
    for pa, pb in zip(path[:-1], path[1:]):
        n_ = max(1, int(np.linalg.norm(pb - pa) / 5.0))
        nd = _u(np.r_[pb - pa, 0.0])[:2]
        side = np.array([-nd[1], nd[0]])
        for k in range(n_ + 1):
            c_ = pa + (pb - pa) * k / n_
            seeds += [c_ + side * o for o in (0.0, 8.0, -8.0, 16.0, -16.0)]
    webs = []
    for sd in seeds:
        web = sd + np.array([(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)])
        if (sd[0] > x1 - 1 and not above(sd)[0]) or sd[1] < y_min + 1 or sd[1] > y_cap - 1 or _hits(web, nuts_):
            continue
        for ang in np.radians(np.arange(0.0, 360.0, 15.0)):
            d = np.array([math.cos(ang), math.sin(ang)])
            r_ = 1.0
            while r_ < 80.0:
                r_ += 1.0
                pt = sd + d * r_
                if (pt[0] > x1 and not above(pt)[0]) or pt[1] < y_min or pt[1] > y_cap:
                    break
                cand = np.vstack([web, pt])
                cand = cand[_hull_idx(cand)]
                if _hits(cand, nuts_):
                    break
                web = cand
        webs.append([tuple(v) for v in _simplify(np.array(web))])
    pieces = arm_polys + webs
    P2 = np.array(pts)
    hull = P2[_hull_idx(P2)]
    longer = float(var.get("longer", 0.0))  # broken copy: a saddle too long for the opening
    block = _profile(fb, [tuple(p) for p in hull], t0 - longer, t1 + longer)
    # the arm's pieces as one clean outline: drawn on a fine grid, kept out of every key's sweep, the
    # nuts' room and the room past the saddle's ends, with nothing thinner than 2 mm and no hairline
    # notch left; it overlaps the saddle by a band 2.5 mm wide rather than meeting its faces edge to edge
    nut_room = [(a_, b_, rk, x0) for a_, b_, rk in nuts_]
    arm_polys = [_outline(pieces, hull, R, caps, nut_room, lead, ang0, x1,
                          (y_cap, x0, y_col - 3.0 if top else None), (x0, yc - w), lead_deg, above)]
    if top:
        arm_polys.append(top)
    # corners of the arm a hair off the saddle's outline go onto it (no hairline face between them);
    # where the arm comes down to the saddle at a narrow angle (a crevice), the crevice's tip is filled
    arm_polys[0] = _snap_to(arm_polys[0], hull, keep_x=(x0, x1))
    arm_polys += _crevices(arm_polys[0], hull, lambda tri: not _hits(tri))
    # the arm, along the pipe only over the joint's span, and kept above part 1's disc
    armp = None
    for poly in arm_polys:
        r_ = _profile(fb, poly, max(ta0, t0), min(ta1, t1))
        armp = r_ if armp is None else armp + r_
    armp = armp & (fa * bd.Pos(0, 0, arm_low + 500.0) * bd.Box(2000, 2000, 1000))
    pipe_cut = bd.Plane(origin=tuple(C), z_dir=tuple(t)) * bd.Cylinder(R, 2000.0)
    part2 = (block + armp) - pipe_cut
    for bt in task.bolts(0):
        if bt.part != pipe_n:
            continue
        bb = BOLTS[bt.size]
        part2 = part2 - bd.Plane(origin=bt.position, z_dir=bt.axis) * bd.Pos(0, 0, GRIP_PIPE / 2) \
            * bd.Cylinder(bb.clear_medium / 2, GRIP_PIPE + 2)
    for p in holes:
        part2 = part2 - bd.Plane(origin=tuple(p), z_dir=tuple(-e1)) * bd.Pos(0, 0, tj / 2) \
            * bd.Cylinder(b.clear_medium / 2, tj + 2.0)
    # ---- part 1's crown: the disc built up thick (its give was half the whole), everywhere but where
    # part 2's arm lies or part 1's way in (ONTO mm up, then across along -e1) would sweep through it,
    # and round A's heads (their keys). Its footprint is drawn on a grid and opened, so no edge meets
    # another, the rim or a head's clearance at a glancing angle
    crown_top = min(CROWN, z1 - b.head_d / 2 - 1.5)
    near = part2 & (fa * bd.Pos(0, 0, crown_top + onto + 1.0) * bd.Box(400, 400, 2 * (crown_top + onto + 1.0)))
    Q = _points(near) - oa if near.volume > 1e-6 else np.zeros((0, 3))
    rd = 1.45 * r_face  # the grid covers the face, corners and all
    g = OUTLINE_GRID
    xs = ys = np.arange(-rd, rd, g)
    gx, gy = np.meshgrid(xs, ys)
    ok = inside(gx, gy, INSET + 2.0)  # a ledge 2 mm wide left round the rim
    shadow = None
    if len(Q):
        qx, qy = Q @ e1, Q @ e2
        sx = min(float(qx.max()) + 0.5, xj)  # never past the joint face: part 1 there only moves away from part 2
        y0_, y1_ = float(qy.min()) - 1.0, float(qy.max()) + 1.0
        ok &= ~((gx <= sx + 0.3) & (gy >= y0_ - 0.5) & (gy <= y1_ + 0.5))
        shadow = fa * bd.Pos((sx - 200.0) / 2, (y0_ + y1_) / 2, 100.0 + tp) * bd.Box(sx + 200.0, y1_ - y0_, 200.0)
    for bt in a_bolts:
        bp = np.asarray(bt.position, float) - oa
        bx, by = float(bp @ e1), float(bp @ e2)
        # clear round the head; where that edge would pass close by the ear, cut back clear past it
        de = math.hypot(max(xj - bx, 0.0, bx - (xj + tj)), max(-half_w - by, 0.0, by - half_w))
        rx = WELL + 0.5
        if rx - 2.5 < de < rx + 2.5:
            rx = de + 2.5
        ok &= np.hypot(gx - bx, gy - by) > rx
    # (its edge at the joint face snapped onto that face's plane: the two merge, no sliver between)
    for poly in _polys(ok, xs, ys, nut_face=(xj, -1e9) if len(Q) and sx == xj else None):
        part1 = part1 + _profile(fa, poly, tp - 0.5, crown_top)
    if spig:  # the pocket again, its cone roof now inside the crown; open right up where the crown is not
        pocket = fa * bd.Pos(0, 0, hp / 2 - 0.5) * bd.Cylinder(rp, hp + 1.0)
        pocket = pocket + fa * bd.Pos(0, 0, hp + cone_h / 2) * bd.Cone(rp, 0.0, cone_h)
        if shadow is not None:
            pocket = pocket + ((fa * bd.Pos(0, 0, 50.0) * bd.Cylinder(rp, 102.0)) & (shadow + fa * bd.Pos((sx - 200.0) / 2, (y0_ + y1_) / 2, tp / 2) * bd.Box(sx + 200.0, y1_ - y0_, tp + 2.0)))
        part1 = part1 - pocket
    for bt in a_bolts:
        part1 = part1 - bd.Plane(origin=bt.position, z_dir=bt.axis) * bd.Pos(0, 0, tp / 2) \
            * bd.Cylinder(BOLTS[bt.size].clear_medium / 2, tp + 2)
    s1, s2 = part1.solids(), part2.solids()
    if len(s1) != 1 or len(s2) != 1:
        raise ValueError(f"a part falls apart ({len(s1)} and {len(s2)} pieces; {[round(x.volume, 1) for x in s1]})")
    part1, part2 = s1[0], s2[0]
    # ---- ways in: part 2 first, in at A's end, then straight onto the pipe along its crest normal;
    # part 1 ONTO mm off A, then onto A's face
    a1 = _proj(part1, e3)
    a2 = _proj(part2, e3)
    drop = -crest
    shift = R * (1.0 - math.cos(math.radians(lead_deg))) + 1.0  # lifted off the pipe so its short wrap clears it
    room = float(a2[0] - a1[0]) - spig_h - 1.0  # part 2 above A's spigot, at its place
    if shift * abs(float(drop @ e3)) > room and not var.get("longer"):  # (a broken copy may meet it)
        raise ValueError(f"part 2 cannot lift clear of the pipe ({shift:.1f} mm) without meeting the flat part ({room:.1f} mm)")
    entry2 = drop * shift  # part 2's whole way, from where it comes in to its place
    asm = {"parts": [
        {"name": "part2", "file": "part2.step", "bolts_to": pipe_n, "insert_direction": _r(-e1),
         "path": [{"move": _r(drop * shift)}]},
        {"name": "part1", "file": "part1.step", "bolts_to": flat, "insert_direction": _r(-e1),
         "path": [{"move": _r(-e3 * onto)}]}],
        "joint": {"bolt": size, "length": float(length), "head_on": "part1", "axis": _r(e1),
                  "positions": [_r(p) for p in holes]}}
    dims = {"tj": tj, "xj": xj, "z": [z1, z2], "ear_w": ear_w, "entry2": entry2, "entry1": e3 * onto,
            "size": size, "length": length, "joint_up": joint_up, "F": F, "pos": pos, "ring": ring,
            "ta": (max(ta0, t0), min(ta1, t1)), "t01": (t0, t1), "part1_first_lift": e3 * onto,
            "profile": {"caps": caps, "nuts": nuts_, "outline": arm_polys[0], "hull": hull, "R": R}}
    return part1, part2, asm, dims


DEFAULT_RADIAL = 0.5  # = the head clearance the grader keeps round a head


def _area(P) -> float:
    return 0.5 * float(np.sum(P[:, 0] * np.roll(P[:, 1], -1) - np.roll(P[:, 0], -1) * P[:, 1]))


def _simplify(P, min_deg: float = 3.0):
    """A convex outline without its nearly straight corners (they would print as faint creases);
    dropping a corner only shrinks the outline."""
    P = [np.asarray(v, float) for v in P]
    changed = True
    while changed and len(P) > 3:
        changed = False
        for i in range(len(P)):
            a, b, c = P[i - 1], P[i], P[(i + 1) % len(P)]
            u, v = b - a, c - b
            turn = abs(math.degrees(math.atan2(u[0] * v[1] - u[1] * v[0], u @ v)))
            if turn < min_deg or np.linalg.norm(u) < 0.3:
                P.pop(i)
                changed = True
                break
    return np.array(P)


OUTLINE_GRID = 0.25  # mm: the grid the arm's outline is drawn on
OUTLINE_MIN = 1.0    # mm: half the thinnest fin left (opening), and the notch closing radius (closing: 0.4)
OUTLINE_TOL = 0.3    # mm: how far the simplified outline may stray from the grid's


def _inside(G, poly):
    """Points G inside a polygon (any shape; even-odd)."""
    P = np.asarray(poly, float)
    inside = np.zeros(len(G), bool)
    for i in range(len(P)):
        a, b = P[i - 1], P[i]
        cond = (a[1] > G[:, 1]) != (b[1] > G[:, 1])
        dy = np.where(b[1] - a[1] == 0, 1e-12, b[1] - a[1])
        inside ^= cond & (G[:, 0] < a[0] + (G[:, 1] - a[1]) * (b[0] - a[0]) / dy)
    return inside


def _trace(mask):
    """The outer boundary of the one region in a boolean grid, as (row, col) pixel centres in order
    (Moore neighbour tracing)."""
    rows, cols = np.nonzero(mask)
    start = (rows[0], cols[0])  # topmost, then leftmost
    nb = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]
    out, cur, back = [start], start, 6  # came in from the left
    H, W = mask.shape
    first = None
    for _ in range(4 * mask.size):
        for k in range(8):
            d = (back + 1 + k) % 8
            r, c = cur[0] + nb[d][0], cur[1] + nb[d][1]
            if 0 <= r < H and 0 <= c < W and mask[r, c]:
                step = (cur, (r, c))
                back = (d + 4) % 8
                cur = (r, c)
                break
        else:
            break  # a single pixel
        if first is None:
            first = step
        elif step == first:
            break  # round once: leaving the start the way it first did
        out.append(cur)
    return np.array(out[:-1] if len(out) > 1 and out[-1] == start else out, float)


def _dp(P, tol):
    """Douglas-Peucker on an open polyline."""
    if len(P) < 3:
        return P
    a, b = P[0], P[-1]
    d = b - a
    n = np.linalg.norm(d)
    dist = np.abs(d[0] * (P[:, 1] - a[1]) - d[1] * (P[:, 0] - a[0])) / n if n > 1e-12 else np.hypot(*(P - a).T)
    i = int(np.argmax(dist))
    if dist[i] <= tol:
        return np.array([a, b])
    return np.vstack([_dp(P[:i + 1], tol)[:-1], _dp(P[i:], tol)])


def _outline(pieces, hull, R, caps, nut_room, lead, ang0, x_max, cap=None, nut_face=None, lead_deg=SADDLE_LEAD,
             above=None):
    """The arm's outline in the profile plane (see build)."""
    from scipy import ndimage

    g = OUTLINE_GRID
    allp = np.vstack([np.asarray(p, float) for p in pieces])
    lo, hi = allp.min(0) - 4.0, allp.max(0) + 4.0
    xs, ys = np.arange(lo[0], hi[0], g), np.arange(lo[1], hi[1], g)
    gx, gy = np.meshgrid(xs, ys)
    G = np.c_[gx.ravel(), gy.ravel()]
    M = np.zeros(len(G), bool)
    for p in pieces:
        M |= _inside(G, p)
    bad = np.zeros(len(G), bool)
    for a_, b_, rk in caps:
        d = b_ - a_
        u = np.clip(((G - a_) @ d) / max(d @ d, 1e-12), 0, 1)
        bad |= np.hypot(*(G - a_ - u[:, None] * d).T) < rk
    for a_, b_, rk, xf in nut_room:  # only on the nut's own side of the joint face
        d = b_ - a_
        u = np.clip(((G - a_) @ d) / max(d @ d, 1e-12), 0, 1)
        bad |= (np.hypot(*(G - a_ - u[:, None] * d).T) < rk) & (G[:, 0] < xf)
    r = np.hypot(G[:, 0], G[:, 1])
    phi = (np.degrees(np.arctan2(G[:, 1], G[:, 0])) - math.degrees(ang0) + 180.0) % 360.0 - 180.0
    bad |= np.where(phi * lead > 0, np.abs(phi) > lead_deg - 6.0, np.abs(phi) > SADDLE_WRAP - 6.0) & (r < R + 14.0)
    bad |= (G[:, 0] > x_max) & (~above(G) if above is not None else True)  # past the joint face: only above part 1
    if cap is not None:  # above y_cap only the joint plate's column (x from cap[1]), and that up to cap[2]
        bad |= (G[:, 1] > cap[0]) & (G[:, 0] < cap[1])
        if cap[2] is not None:
            bad |= G[:, 1] > cap[2]
    shape = (len(ys), len(xs))
    M, bad = M.reshape(shape), bad.reshape(shape)
    H = _inside(G, hull).reshape(shape)
    M |= H  # with the saddle's own section, so a hairline beside its edge is opened away too

    def disk(rad):
        k = int(math.ceil(rad / g))
        yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
        return (xx * xx + yy * yy) * g * g <= rad * rad + 1e-9

    barred = ndimage.binary_dilation(bad, disk(g)) & ~H
    M = ndimage.binary_closing(M, structure=disk(0.4), border_value=0) & ~barred
    # the arm is what lies outside the saddle's section (and the pipe), opened on its own so no
    # hairline is left along the saddle's faces, then grown 1 mm back: where it meets the saddle it
    # runs 1 mm into it, so its outline crosses the saddle's edge or lies inside, never along it
    own = H | (r.reshape(shape) < R)
    M = ndimage.binary_opening(M & ~own, structure=disk(OUTLINE_MIN))
    M = ndimage.binary_dilation(M, structure=disk(1.0)) & ~barred
    got = _polys(M, xs, ys, opened=False, largest=True, x_snap=x_max, nut_face=nut_face)
    if not got:
        raise ValueError("the arm has no room")
    return got[0]


def _polys(M, xs, ys, *, opened=True, largest=False, x_snap=None, nut_face=None, min_area=20.0):
    """Clean polygons from a grid: each region's outer boundary (holes filled), simplified to within
    OUTLINE_TOL, with no edge under 0.6 mm and no corner turning under 3 deg; opened first (nothing
    thinner than 2 * OUTLINE_MIN) unless ``opened`` is False. ``x_snap``: corners within 1 mm of
    that x go onto it; ``nut_face`` (x, y from): corners within 0.6 mm of that x, from that y up, go
    onto it (the joint plate's other face, where the nuts bear: its thickness sets the bolt's length)."""
    from scipy import ndimage

    g = float(xs[1] - xs[0])
    if opened:
        k = int(math.ceil(OUTLINE_MIN / g))
        yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
        M = ndimage.binary_opening(M, structure=(xx * xx + yy * yy) * g * g <= OUTLINE_MIN ** 2 + 1e-9)
    lab, n = ndimage.label(M)
    order = sorted(range(1, n + 1), key=lambda c: -int((lab == c).sum()))
    out = []
    for c in order[:1] if largest else order:
        C = ndimage.binary_fill_holes(lab == c)
        if C.sum() * g * g < min_area:
            continue
        B = _trace(C)
        pts = np.c_[xs[0] + B[:, 1] * g, ys[0] + B[:, 0] * g]
        k = int(np.argmax(np.hypot(*(pts - pts[0]).T)))  # split the loop at the point farthest from its first
        loop = np.vstack([_dp(pts[:k + 1], OUTLINE_TOL)[:-1], _dp(np.vstack([pts[k:], pts[:1]]), OUTLINE_TOL)[:-1]])
        if x_snap is not None:
            loop[np.abs(loop[:, 0] - x_snap) < 1.0, 0] = x_snap
        if nut_face is not None:
            near_ = (np.abs(loop[:, 0] - nut_face[0]) < 0.6) & (loop[:, 1] >= nut_face[1] - 0.5)
            loop[near_, 0] = nut_face[0]
        P = [np.array(v) for v in loop]
        changed = True
        while changed and len(P) > 3:
            changed = False
            for i in range(len(P)):
                a_, b_, c_ = P[i - 1], P[i], P[(i + 1) % len(P)]
                u, v = b_ - a_, c_ - b_
                if np.linalg.norm(v) < 0.6:  # b and c into one corner; one on the snapped line stays there
                    on = (lambda q: (x_snap is not None and q[0] == x_snap) or (nut_face is not None and q[0] == nut_face[0]))
                    P[i] = c_ if on(c_) else (b_ if on(b_) else (b_ + c_) / 2)
                    P.pop((i + 1) % len(P))
                    changed = True
                    break
                if abs(math.degrees(math.atan2(u[0] * v[1] - u[1] * v[0], u @ v))) < 3.0:
                    P.pop(i)
                    changed = True
                    break
        out.append([tuple(v) for v in P])
    return out


CREVICE = 50.0  # deg: an inside corner narrower than this between the arm and the saddle is filled at its tip
CREVICE_FILL = 6.0  # mm along each side the fill reaches


def _crevices(outline, hull, ok) -> list:
    """Triangles filling the tips of the narrow inside corners where the arm (``outline``) and the
    saddle (``hull``) meet, each along the two sides it fills between; only those ``ok`` allows."""
    f = _face2(outline).fuse(_face2(hull)).clean()
    face = max(f.faces(), key=lambda x: x.area)
    P = [np.array([e.start_point().X, e.start_point().Y]) for e in face.outer_wire().order_edges()]
    sg = np.sign(_area(np.array(P)))
    out = []
    for i in range(len(P)):
        a, b, c = P[i - 1], P[i], P[(i + 1) % len(P)]
        u, v = b - a, c - b
        cross = float(u[0] * v[1] - u[1] * v[0])
        if cross * sg >= 0:
            continue  # not an inside corner
        gap = 180.0 - abs(math.degrees(math.atan2(cross, u @ v)))
        if gap >= CREVICE:
            continue
        L = min(np.linalg.norm(u), np.linalg.norm(v), CREVICE_FILL)
        tri = [tuple(b + (a - b) * L / np.linalg.norm(u)), tuple(b), tuple(b + (c - b) * L / np.linalg.norm(v))]
        if ok(tri):
            out.append(tri)
    return out


def _snap_to(poly, onto, tol: float = 0.6, keep_x=()) -> list:
    """``poly``'s corners within ``tol`` of ``onto``'s outline moved onto it (the nearest point); corners
    exactly at an x of ``keep_x`` (faces that must stay where they are) only slide along that x."""
    Q = np.asarray(onto, float)
    A0, A1 = Q, np.roll(Q, -1, axis=0)
    out = []
    for p in np.asarray(poly, float):
        d = A1 - A0
        u = np.clip(np.einsum("ij,ij->i", p - A0, d) / np.maximum(np.einsum("ij,ij->i", d, d), 1e-12), 0, 1)
        near = A0 + u[:, None] * d
        dist = np.hypot(*(near - p).T)
        k = int(np.argmin(dist))
        q = p
        if dist[k] < tol:
            if not any(abs(p[0] - kx) < 1e-9 for kx in keep_x):
                q = near[k]
            elif abs(d[k][0]) > 1e-9:  # on a face that stays: slide along it (x fixed) onto that edge
                f = (p[0] - A0[k][0]) / d[k][0]
                if 0.0 <= f <= 1.0:
                    q = np.array([p[0], A0[k][1] + f * d[k][1]])
        if not out or np.linalg.norm(q - out[-1]) > 0.05:
            out.append(q)
    if len(out) > 1 and np.linalg.norm(out[0] - out[-1]) <= 0.05:
        out.pop()
    return [tuple(v) for v in out]


def _face2(poly):
    return bd.Face(bd.Wire.make_polygon([bd.Vector(float(x), float(y), 0) for x, y in poly], close=True))


def _seg_dist(A0, A1, b0, b1):
    """Distance from each segment A0[i]-A1[i] to the segment b0-b1 (all in a plane)."""
    A0, A1 = np.atleast_2d(A0), np.atleast_2d(A1)
    b0, b1 = np.asarray(b0, float), np.asarray(b1, float)

    def pt_seg(P, a, b):  # points P to segment(s) a-b
        d = b - a
        dd = np.einsum("ij,ij->i", d, d) if d.ndim == 2 else float(d @ d)
        u = np.clip(np.einsum("ij,ij->i", P - a, d if d.ndim == 2 else np.broadcast_to(d, P.shape))
                    / np.maximum(dd, 1e-12), 0, 1)
        q = a + u[:, None] * (d if d.ndim == 2 else d[None, :])
        return np.hypot(*(P - q).T)

    def cross(o, a, b):
        return (a[:, 0] - o[:, 0]) * (b[:, 1] - o[:, 1]) - (a[:, 1] - o[:, 1]) * (b[:, 0] - o[:, 0])

    B0, B1 = np.broadcast_to(b0, A0.shape), np.broadcast_to(b1, A0.shape)
    d1, d2 = cross(B0, B1, A0), cross(B0, B1, A1)
    d3, d4 = cross(A0, A1, B0), cross(A0, A1, B1)
    meet = (d1 * d2 < 0) & (d3 * d4 < 0)
    dist = np.minimum.reduce([pt_seg(A0, B0[0], B1[0]), pt_seg(A1, B0[0], B1[0]),
                              pt_seg(B0, A0, A1), pt_seg(B1, A0, A1)])
    return np.where(meet, 0.0, dist)


def _hull_idx(P):
    from scipy.spatial import ConvexHull

    return ConvexHull(P).vertices


def _proj(shape, n):
    p = _points(shape) @ np.asarray(n, float)
    return float(p.min()), float(p.max())


# --------------------------------------------------------------------------
# the box


def box(task: Task, part1, part2, dims: dict, *, starts=None, extra=()) -> tuple[list, dict]:
    """The closed box round both fixed parts and the two parts, the pipe through its top and bottom
    walls, the one opening in the +e1 wall drawn round where the parts cross it on their way in.
    ``starts``: the two parts where their ways in begin (by default level 4's); ``extra``: more
    poses the box must hold (a lock's push and turn)."""
    F = dims["F"]
    e1, t, C, R = F["e1"], F["t"], F["C"], F["R"]
    s = _u(np.cross(t, e1))  # the box's third direction, square to the pipe and to e1
    if starts is None:
        in1 = part1.moved(bd.Location(tuple(dims["entry1"])))
        in2 = part2.moved(bd.Location(tuple(-dims["entry2"])))
    else:
        in1, in2 = starts
    fixed = task.fixed_solids(0)
    pb = task.poses[0][F["pipe"]]
    itf = task.parts[F["pipe"]]
    xs = [p[0] for p, _ in itf.hole_frames]
    mid = (max(xs) + min(xs)) / 2
    pipe_ends = [np.array(pb.point((mid + k * itf.face_w / 2, 0, 0))) for k in (-1, 1)]
    stuff = np.concatenate([_points(x) for x in (part1, part2, in1, in2, fixed[F["flat"]], *extra)])
    dirs = {"t+": t, "t-": -t, "e1+": e1, "e1-": -e1, "s+": s, "s-": -s}
    off = {k: float((stuff @ n).max()) + MARGIN.get(k, MARGIN.get(k[0], 30.0)) for k, n in dirs.items()}
    # the pipe passes the top and bottom walls: it must be long enough
    for k, sign in (("t+", 1), ("t-", -1)):
        if max(float(e @ dirs[k]) for e in pipe_ends) < off[k] + WALL + 2.0:
            raise ValueError("the pipe is too short to pass the box's walls")
    rim = np.array([C + R * (math.cos(a) * e1 + math.sin(a) * s) + t * float(k)
                    for a in np.linspace(0, 2 * math.pi, 24, endpoint=False)
                    for k in (off["t+"] - float(C @ t), -off["t-"] - float(C @ t))])
    for k, n in dirs.items():
        if k[0] != "t":
            off[k] = max(off[k], float((rim @ n).max()) + WALL + 5.0)
    # the box in its own axes (e1, s, t): from -off[k-] to off[k+] along each; every wall exactly as
    # big as the box's side, the four round a side overlapping at the edges
    axes = {"e1": e1, "s": s, "t": t}
    span = {a: (-off[a + "-"], off[a + "+"]) for a in axes}
    walls = []
    for k, n in dirs.items():
        if k == "e1+":
            continue
        a, sg = k[:-1], (1.0 if k[-1] == "+" else -1.0)
        u_, v_ = [o for o in ("e1", "s", "t") if o != a]
        if float(np.cross(axes[u_], axes[v_]) @ n) < 0:
            u_, v_ = v_, u_  # (u, v, n) right-handed
        cen = n * (off[k] + WALL / 2) + sum(axes[o] * (span[o][0] + span[o][1]) / 2 for o in (u_, v_))
        size = tuple(float(span[o][1] - span[o][0] + 2 * WALL) for o in (u_, v_)) + (WALL,)
        o = Obstacle({"t+": "wall_top", "t-": "wall_bottom", "e1-": "wall_back", "s+": "wall_side_1",
                      "s-": "wall_side_2"}[k], size,
                     Placement(tuple(float(x) for x in cen), _rot(axes[u_], axes[v_], n)))
        if a == "t":
            tt = (off[k] + WALL / 2 - float(C @ n)) / float(t @ n)
            o.holes = ({"center": _r(C + t * tt), "axis": _r(t), "radius": round(R + PIPE_CLEAR, 6),
                        "length": round(4 * WALL + 20.0, 6)},)
        walls.append(o)
    # the opening: where both parts cross the +e1 wall on their way in, and the keys for the joint bolts
    cross = [_through(_points(in1), -e1, e1, off["e1+"]), _through(_points(in2), -e1, e1, off["e1+"])]
    cross = np.concatenate(cross)
    up = t  # the opening wall's axes: t and s
    oa_ = (float((cross @ s).min()) - CLEAR, float((cross @ s).max()) + CLEAR)
    ot = (float((cross @ up).min()) - CLEAR, float((cross @ up).max()) + CLEAR)
    rot = _rot(s, t, e1)
    S0, S1 = span["s"][0] - WALL, span["s"][1] + WALL
    T0, T1 = span["t"][0] - WALL, span["t"][1] + WALL

    def piece(name, a0, a1, z0, z1):
        cen = s * ((a0 + a1) / 2) + t * ((z0 + z1) / 2) + e1 * (off["e1+"] + WALL / 2)
        return Obstacle(name, (float(a1 - a0), float(z1 - z0), WALL), Placement(tuple(float(x) for x in cen), rot))

    walls += [piece("wall_front_below", S0, S1, T0, ot[0]), piece("wall_front_above", S0, S1, ot[1], T1),
              piece("wall_front_left", S0, oa_[0], ot[0], ot[1]), piece("wall_front_right", oa_[1], S1, ot[0], ot[1])]
    opening = {"wall": "wall_front", "normal": _r(e1), "plane": round(off["e1+"], 6), "along": _r(s), "up": _r(t),
               "along_range": [round(oa_[0], 6), round(oa_[1], 6)], "up_range": [round(ot[0], 6), round(ot[1], 6)]}
    return walls, opening


def task4(task3: Task, walls: list, opening: dict) -> Task:
    tk = Task.from_json(json.loads(task3.dumps()))
    tk.root = task3.root
    tk.id = task3.id.replace("-L3-", "-L4-")
    tk.level = LEVEL
    tk.obstacles = [walls]
    tk.tolerances["joint"] = {"length_required": True}
    tk.meta = dict(tk.meta, test="assembles", assembly=True, opening=opening)
    return tk


THIN_EDGE_KG = 7.5  # the thin-edge copy is tested at this load: the reference holds it, the copy does not


def broken(task: Task, dims: dict, part1, part2, asm: dict) -> dict:
    """name -> (part1, part2, assembly dict, the check that must fail, load in kg or None for the
    task's own)."""
    swap = json.loads(json.dumps(asm))
    swap["parts"] = swap["parts"][::-1]
    out = {"wrong-order": (part1, part2, swap, "INSERTION", None)}
    heads = json.loads(json.dumps(asm))
    heads["joint"]["head_on"], heads["joint"]["axis"] = "part2", _r(-dims["F"]["e1"])
    out["key-cannot-reach"] = (part1, part2, heads, "JOINT_REACH", None)
    short = json.loads(json.dumps(asm))
    short["joint"]["length"] = float(max(x for x in lengths(dims["size"]) if x < dims["length"]))
    out["bolt-too-short"] = (part1, part2, short, "JOINT_LENGTH", None)
    p1, p2, a, _ = build(task, size=dims["size"], length=dims["length"], joint_up=dims["joint_up"], variant={"longer": 20.0})
    out["too-big-for-the-opening"] = (p1, p2, a, "INSERTION", None)  # the saddle 20 mm longer each end
    p1, p2, a, _ = build(task, size=dims["size"], length=dims["length"], joint_up=dims["joint_up"], variant={"narrow_ear": True})
    out["thin-edge-holes"] = (p1, p2, a, "JOINT_BOLTS", THIN_EDGE_KG)
    return out


# --------------------------------------------------------------------------
# the task folder


VIEW_NOTE = ("A ship in a bottle round a pipe. Grey: the two fixed parts, the pipe and the flat plate. Red, "
             "see-through: the closed box, the pipe passing its top and bottom walls, its one opening in the "
             "front wall. Green: part 1, which bolts to the plate; teal: part 2, which bolts to the pipe and goes in "
             "first. Slide to take them out again the way they went in, last in first out; the lines are each "
             "part's way in (its middle point). Drag to turn, scroll to zoom.")


def setup(seed: int = 0) -> Task:
    """The version 1 pipe setup (round 5's look) as level 3: the pipe A, the square plate B."""
    from .round5 import showcase_draw, task_from_draw

    return task_from_draw(seed, showcase_draw(), 3)


def make(task3: Task | None = None, *, size: str = "M5", length: float = 35.0, joint_up: float = JOINT_UP):
    """(level 4 task, (part 1, part 2, assembly), dimensions) for a level 3 pipe task (the version 1
    setup's by default)."""
    task3 = task3 or setup()
    p1, p2, asm, dims = build(task3, size=size, length=length, joint_up=joint_up)
    walls, opening = box(task3, p1, p2, dims)
    return task4(task3, walls, opening), (p1, p2, asm), dims


def write_folder(task: Task, ref, dims: dict, out_dir: str, *, task3: Task | None = None, view: bool = True) -> str:
    """The task folder the model sees, plus _solution/ (never shown to it): the reference, its broken
    copies (each with the check it must fail, and its own load where it has one) and a view."""
    import shutil

    from ...assemble.brief import SUBMISSION
    from ...assemble.generate import write_answer
    from ...geometry import write_step
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
    with open(os.path.join(folder, "submission.py"), "w", encoding="utf-8", newline="\n") as f:
        f.write(SUBMISSION.replace('"joint": {"bolt": "M4", ', '"joint": {"bolt": "M4", "length": 20, '))
    sol = os.path.join(folder, "_solution")
    write_answer(sol, *ref)
    with open(os.path.join(sol, "reference.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump({"level": LEVEL, "test": "assembles", "situation": "pipe",
                   "joint": {"bolt": dims["size"], "length": dims["length"], "plates": round(dims["tj"], 3)}},
                  f, indent=2)
        f.write("\n")
    for name, (q1, q2, a, must, kg) in broken(task3 or setup(), dims, *ref).items():
        bdir = os.path.join(sol, "broken", name)
        write_answer(bdir, q1, q2, a)
        with open(os.path.join(bdir, "MUST_FAIL"), "w", encoding="utf-8") as f:
            f.write(must + "\n")
        if kg is not None:
            with open(os.path.join(bdir, "LOAD_KG"), "w", encoding="utf-8") as f:
                f.write(f"{kg:g}\n")
    if view:
        from ...assemble.view import write as write_view

        F = dims["F"]  # looking in from the opening's side, a little from above
        write_view(task, sol, os.path.join(sol, "view"), heading=f"Clamp a pipe, level 4 (assembles): {task.id}",
                   note=VIEW_NOTE, view_dir=_u(F["e1"] + 0.45 * F["t"] - 0.2 * F["e3"]))
    return folder


def main(argv=None) -> int:
    """python -m jointbench.families.pipe.assembles OUT_DIR: the version 1 pipe setup's level 4 task folder."""
    import sys

    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) != 1:
        print(main.__doc__)
        return 2
    t3 = setup()
    t4, ref, dims = make(t3)
    print(write_folder(t4, ref, dims, args[0], task3=t3))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
