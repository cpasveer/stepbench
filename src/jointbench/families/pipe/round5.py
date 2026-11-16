"""The pipe setup of version 1, levels 1-3: a long pipe with six bolts in two
rows round it (A) and a square plate (B) at an angle to it, as in round 5.

Print frame. The reference is printed with "up" chosen so that B's plate
stands exactly upright and the pipe leans at most about 40 degrees from the
vertical (up = the pipe's axis projected onto B's face plane). The arrangement
is placed in the world with that up along world +z, turned about it at random,
so the print direction is a world axis (the printing check always tries the
axes).

The reference, every face of it upright or leaning less than 45 degrees in
print:

* a saddle on the pipe: a prism along the pipe whose outline, seen along the
  axis, is the wedge between two flat faces square to the two bolt rows (the
  heads sit on them), ended by faces square to the pipe's surface, the pipe
  carved out; it runs along the pipe down to the print bed;
* a plate on B, upright, inside B's face by the setback, standing on a foot
  (an upright column the size of the plate's own footprint) down to the bed;
* an arm from the foot to the saddle, upright, low: it stays below the plate,
  so the space behind the plate's bolt heads is free for the key and the tool.
"""
from __future__ import annotations

import math
import os
import random
import tempfile

import build123d as bd
import numpy as np

from ...catalogue import BOLTS, GENERATOR_VERSION, make_interface
from ...geometry import Placement, write_step
from ...requirements import apply_stated_load
from ...task import DEFAULT_TOLERANCES, Task
from ..facing.shell import NAMES, REQUIRED
from .bracket import _convex, _halfspace, _prism_yz, _unit

KNOBS_DEFAULT: dict = {}
SITUATION = (
    "Two parts are fixed in space at an angle to each other. One is a round pipe\n"
    "with its bolts screwed straight into it, in two rows round it; the other has\n"
    "a flat mounting face. Nothing stands between them."
)
G_SADDLE = 10.0     # mm from the pipe's surface to each row's flat head face, along the bolt
WRAP = 20.0         # deg the saddle runs past each row, round the pipe
PLATE_T = 14.0      # mm, the plate on B
SETBACK = 1.5       # mm the plate stays inside B's face
ARM_W = 30.0        # mm, the arm's width
FOOT_D = 18.0       # mm, how far the plate's foot reaches back below the plate
FOOT_W = 0.0        # mm the foot is wider than the plate on each side
ARM_GAP = 3.0       # mm the arm's top stays below the plate's lowest bolt head zone
CHAMFER = 2.0       # mm, the chamfer on the arm's top edges that lean out
BAND = 3.0          # mm at the bed that rise straight up
EDGE = 6.0          # mm of saddle beyond the outermost heads, along the pipe (upward)


def _rot_cols(x, y, z):
    return tuple(tuple(float(v) for v in row) for row in np.column_stack([x, y, z]))


def showcase_draw() -> dict:
    """The round 5 pipe setup (round 5's ``step-b1/gen/pipe-pair-0``), B a square 4-bolt plate. Its numbers are
    written here (B's centre and face normal in the pipe's own frame), so the kit needs no outside file."""
    pipe = {"bolt": "M6", "od": 76.0,
            "holes": [[-40.0, -30.0], [0.0, -30.0], [40.0, -30.0], [-40.0, 30.0], [0.0, 30.0], [40.0, 30.0]]}
    # full precision, exactly as derived from the round 5 task (the reference's cuts are sensitive to the last digit)
    centre = [-11.471528727020921, -16.383040885779813, 101.50000000000001]
    normal = [0.6408563820557881, 0.2988362387301198, -0.7071067811865475]
    return {"pipe": pipe, "B": {"kind": "square", "params": {"bolt": "M5", "pitch": 30}},
            "b_centre": centre, "b_normal": normal,
            "yaw": 30.0, "origin": [0.0, 0.0, 0.0], "load_kg": 1.0}


def layout(d: dict):
    """(pipe interface, B interface, B's placement in the pipe's frame, up in the pipe's frame)."""
    itf_b = make_interface(d["B"]["kind"], d["B"]["params"])
    nb = _unit(d["b_normal"])
    ax = np.array([1.0, 0.0, 0.0])
    up = _unit(ax - nb * float(ax @ nb))  # the pipe's axis seen in B's plane: up in print
    side = np.cross(nb, up)
    pl_b = Placement(tuple(d["b_centre"]), _rot_cols(up, side, nb))  # B's long side (local x) upright
    # the pipe long enough to carry the saddle down to the bed and past B
    pipe = dict(d["pipe"], length=float(d["pipe"].get("length") or 260.0))
    itf_a = make_interface("pipe", pipe)
    return itf_a, itf_b, pl_b, up


def build(d: dict) -> bd.Part:
    """The reference in the pipe's own frame."""
    itf_a, itf_b, pl_b, up = layout(d)
    R = itf_a.face_h / 2
    pb = BOLTS[itf_a.bolt]
    nb = np.array(pl_b.vec((0, 0, 1)))
    side = np.array(pl_b.vec((0, 1, 0)))
    F = np.array(pl_b.origin)

    # heights in print (along up)
    a_half = itf_b.face_w / 2 - SETBACK
    b_half = itf_b.face_h / 2 - SETBACK
    z_plate_lo = float(F @ up) - a_half
    # the bed: below the plate's lower edge and below the pipe's lowest bolt ring (with the saddle round it)
    z_rings = min(float(np.asarray(p) @ up) for p, _ in itf_a.hole_frames) - pb.head_d / 2 - EDGE
    z_bed = min(z_plate_lo, z_rings)
    # the saddle: wedge outline in the pipe's cross-section (y, z), along the pipe (x)
    rows = sorted({round(math.degrees(math.atan2(a[1], a[2])), 6) for _, a in itf_a.hole_frames})
    amax = max(abs(r) for r in rows)
    xs = sorted({p[0] for p, _ in itf_a.hole_frames})
    d_face = R + G_SADDLE

    def u(a):
        return np.array([math.sin(a), math.cos(a)])

    lo, hi = math.radians(min(rows)), math.radians(max(rows))
    ridge = np.linalg.solve(np.array([u(lo), u(hi)]), np.array([d_face, d_face]))
    a_side = math.radians(amax + WRAP)

    def out_to(a, fa):
        dd = np.array([math.sin(a), math.cos(a)])
        return tuple(dd * (d_face / float(u(fa) @ dd)))

    poly = [(R * math.sin(-a_side), R * math.cos(-a_side)), out_to(-a_side, lo), tuple(ridge), out_to(a_side, hi),
            (R * math.sin(a_side), R * math.cos(a_side)), (0.0, 0.0)]
    x_hi = max(xs) + pb.head_d / 2 + EDGE if up[0] > 0 else min(xs) - pb.head_d / 2 - EDGE
    # down along the pipe until the whole section is below the bed
    reach = max(abs(float(np.array([0.0, y, z]) @ up)) for y, z in poly) + 5.0
    x_bed = (z_bed - reach) / up[0] if abs(up[0]) > 1e-6 else min(xs) - 200.0
    x0, x1 = sorted([x_bed, x_hi])
    saddle = _prism_yz(poly, x0, x1)

    # the plate on B, upright, and its foot down to the bed: one upright column of the plate's footprint
    e1 = nb
    e2 = np.cross(up, e1)  # the plan plane's own y axis (z x x), so plan points are not mirrored
    plane = bd.Plane(origin=tuple(F - up * (a_half + 300.0)), x_dir=tuple(e1), z_dir=tuple(up))
    foot_rect = bd.Polygon((0.0, -b_half), (PLATE_T, -b_half), (PLATE_T, b_half), (0.0, b_half), align=None)
    column = bd.extrude(plane * foot_rect, 300.0 + 2 * a_half, dir=tuple(up))
    column = column - _halfspace(F + up * a_half, -up)  # no higher than the plate's top
    # below the plate the foot is deeper (FOOT_D behind the face) and wider, down to the bed; it stays below
    # the plate's lower edge, clear of the bolt heads behind the plate
    deep = bd.Polygon((0.0, -b_half - FOOT_W), (FOOT_D, -b_half - FOOT_W), (FOOT_D, b_half + FOOT_W),
                      (0.0, b_half + FOOT_W), align=None)
    foot_col = bd.extrude(plane * deep, 300.0 + 2 * a_half, dir=tuple(up))
    column = column + (foot_col - _halfspace(F + up * (-a_half), -up))
    # the arm: from the column to the saddle's ridge, low (below the lowest bolt head zone of the plate)
    bolts_b = [np.array(pl_b.point((hx, hy, 0.0))) for hx, hy in itf_b.holes]
    z_heads = min(float(p @ up) for p in bolts_b) - (BOLTS[itf_b.bolt].head_d / 2 + 1.2) - ARM_GAP
    # level with the foot's top (the plate's lower edge) when that is still below the heads: one flat top, no sliver
    if z_plate_lo <= z_heads + ARM_GAP:
        z_heads = z_plate_lo
    # the arm: a prism along the pipe, like the saddle, so its sides and the head faces meet in one cross-section
    # (their angle is then set in 2D); it reaches from the saddle's ridge to the plate's foot and is cut level
    # below the plate's lowest bolt head zone
    corners = [F + e1 * de1 + e2 * de2 + up * (z_plate_lo - float(F @ up) - 1.0)
               for de1 in (0.0, PLATE_T) for de2 in (-b_half, b_half)]
    yz_plate = [(float(c[1]), float(c[2])) for c in corners]
    ray_lo = _unit(np.array(out_to(-a_side, lo)) - ridge)
    ray_hi = _unit(np.array(out_to(a_side, hi)) - ridge)

    # the arm's foot on each head face stays short of the heads (they sit d_face * tan(row) from the ridge)
    s_max = d_face * math.tan(math.radians(amax)) - pb.head_d / 2 - 1.5

    def foot(ray):
        best, best_ang = None, -1.0
        for s_ in np.arange(2.0, s_max, 0.5):
            q = ridge + ray * s_
            hull_ = _convex([tuple(q), tuple(ridge)] + yz_plate)
            # the arm's side leaving q: the hull edge from q that is not along the face
            k = [i_ for i_, pp in enumerate(hull_) if np.allclose(pp, q, atol=1e-6)]
            if not k:
                continue
            k = k[0]
            for nb_ in (hull_[(k + 1) % len(hull_)], hull_[k - 1]):
                edge = _unit(np.array(nb_) - q)
                if abs(float(edge @ ray)) > 0.999 or np.allclose(nb_, ridge, atol=1e-6):
                    continue
                ang = math.degrees(math.acos(max(-1.0, min(1.0, float(ray @ edge)))))
                if ang >= 50.0:
                    return q
                if ang > best_ang:
                    best, best_ang = q, ang
        return best if best is not None else ridge + ray * min(10.0, s_max)

    arm_yz = _convex([tuple(foot(ray_lo)), tuple(ridge), tuple(foot(ray_hi))] + yz_plate)
    arm = _prism_yz(arm_yz, x0, x1 + 400.0 if up[0] > 0 else x1)
    arm = arm - _halfspace(up * z_heads, -up)  # level below the plate's lowest head zone
    # where a side of the arm leans out under that level top, the two meet at under 90 degrees (a thin edge):
    # chamfer that edge, CHAMFER mm back, so it becomes two open angles, the chamfer facing up
    n_ = len(arm_yz)
    for i_ in range(n_):
        a2, b2 = np.array(arm_yz[i_]), np.array(arm_yz[(i_ + 1) % n_])
        t2 = _unit(b2 - a2)
        n3 = np.array([0.0, t2[1], -t2[0]])  # outward, for a counter-clockwise outline (y, z)
        if float(n3 @ up) > -0.05:
            continue
        x_e = (z_heads - a2[0] * up[1] - a2[1] * up[2]) / up[0]
        p_e = np.array([x_e, a2[0], a2[1]])  # a point on that edge (on the side, at the top's level)
        m = _unit(up + n3)
        arm = arm - _halfspace(p_e - m * CHAMFER, -m)
    # below the plate the foot slopes back at 45 degrees from the plate's lower front edge: the seat stops at
    # the plate's edge, and the slope prints. Cut from the foot and the arm only, never from the saddle.
    P0 = F + up * (z_plate_lo - float(F @ up))
    slope = _halfspace(P0, _unit(nb + up)) & _halfspace(F + nb * (FOOT_D + 2.0), nb)
    # the pipe bolts' heads and their tool's way out are cut from the arm from the head face outward (before the
    # union, so the saddle's head faces stay whole): the arm threads between the two rows
    for p_, a_ in itf_a.hole_frames:
        P_, V_ = np.asarray(p_, float), _unit(a_)
        rt_ = pb.head_d / 2 + 1.2
        w_up_ = _unit(up - V_ * float(up @ V_))
        w_side_ = np.cross(V_, w_up_)
        k_ = rt_ * math.sqrt(0.5)
        tear_ = bd.Circle(rt_) + bd.Polygon((-k_, -k_), (k_, -k_), (0.0, -rt_ * math.sqrt(2.0)), align=None)
        tpl_ = bd.Plane(origin=tuple(P_ + V_ * (G_SADDLE - 0.5)), x_dir=tuple(w_side_), z_dir=tuple(V_))
        zone_ = bd.extrude(tpl_ * tear_, 200.0, dir=tuple(V_))
        arm = arm - zone_
        column = column - zone_
    # the saddle and the arm stop 1 mm short of B's face plane: only the plate may touch B
    # the arm ends at the back of the foot (FOOT_D behind B's face): only the foot carries the slope, so the
    # arm's leaning sides are never cut by it at a sharp angle
    arm = arm - _halfspace(F + nb * FOOT_D, nb)
    body = saddle + (column - slope) + arm
    body = body - _halfspace(up * z_bed, -up) if False else body - _halfspace(up * z_bed + np.zeros(3), up)
    # nothing on B's side of its face, near B (B's outline and a margin); far away the saddle may pass the plane
    near_b = bd.Plane(origin=tuple(F), x_dir=tuple(up), z_dir=tuple(nb)) * bd.Box(
        itf_b.face_w + 20.0, itf_b.face_h + 20.0, 400.0)
    body = body - (_halfspace(F, nb) & near_b)
    # every bolt into the pipe: its head's way out and the straight tool's room, a teardrop channel along it
    # (round below, a 45-degree roof above in print)
    for p, a in itf_a.hole_frames:
        P, V = np.asarray(p, float), _unit(a)
        head_at = P + V * (G_SADDLE + 0.3)  # just off the head face: the face itself is left whole
        rt = pb.head_d / 2 + 1.2
        w_up = _unit(up - V * float(up @ V))  # up, seen across the bolt
        w_side = np.cross(V, w_up)
        k = rt * math.sqrt(0.5)
        tear = bd.Circle(rt) + bd.Polygon((-k, -k), (k, -k), (0.0, -rt * math.sqrt(2.0)), align=None)
        tpl = bd.Plane(origin=tuple(head_at), x_dir=tuple(w_side), z_dir=tuple(V))
        # in tpl, local +y is V x w_side = -w_up: the roof (at -y) points up
        body = body - bd.extrude(tpl * tear, pb.head_h + 32.0, dir=tuple(V))
    # the pipe carved out, its bolts' holes along their axes
    body = body - bd.Pos((x0 + x1) / 2, 0, 0) * bd.Rot(0, 90, 0) * bd.Cylinder(R, abs(x1 - x0) + 600.0)
    for p, a in itf_a.hole_frames:
        P, V = np.asarray(p, float), np.asarray(a, float)
        body = body - bd.Plane(origin=tuple(P), z_dir=tuple(V)) * bd.Pos(0, 0, G_SADDLE / 2) * bd.Cylinder(
            pb.clear_medium / 2, G_SADDLE + 4)
    # B's bolts: holes through the plate along its normal
    fb = BOLTS[itf_b.bolt]
    for p in bolts_b:
        body = body - bd.Plane(origin=tuple(p), z_dir=tuple(nb)) * bd.Pos(0, 0, PLATE_T / 2) * bd.Cylinder(
            fb.clear_medium / 2, PLATE_T + 4)
    # the bottom BAND mm rise straight up from the bed (the section just above, extruded down): no knife edge
    # where a leaning face would meet the bed
    cut = bd.Plane(origin=tuple(up * (z_bed + BAND)), x_dir=tuple(e1), z_dir=tuple(up))
    sec = body.intersect(cut * bd.Rectangle(4000.0, 4000.0)) if False else body & (cut * bd.Pos(0, 0, -0.001) * bd.Box(4000.0, 4000.0, 0.002))
    faces = [f for f in sec.faces() if abs(float(np.array(tuple(f.normal_at(f.center()))) @ up)) > 0.99] if sec else []
    if faces:
        band = None
        for f in faces:
            if f.center().dot(bd.Vector(*up)) > z_bed + BAND - 0.002:
                piece = bd.extrude(f, BAND + 0.5, dir=tuple(-up))
                band = piece if band is None else band + piece
        if band is not None:
            body = (body - _halfspace(up * (z_bed + BAND), up)) + band
            body = body - _halfspace(up * z_bed, up)
            body = body - bd.Pos((x0 + x1) / 2, 0, 0) * bd.Rot(0, 90, 0) * bd.Cylinder(R, abs(x1 - x0) + 600.0)
    body = body.clean()  # faces on one surface merged (the arm and the foot share sides)
    solids = body.solids()
    return max(solids, key=lambda s: s.volume) if len(solids) > 1 else solids[0]


def placements(d: dict):
    """(pipe placement, B placement) in the world: up along world +z, turned by d["yaw"]."""
    itf_a, itf_b, pl_b, up = layout(d)
    e3 = up
    e1 = _unit(np.cross(np.array([0.0, 0.0, 1.0]) if abs(e3[2]) < 0.9 else np.array([1.0, 0.0, 0.0]), e3))
    e2 = np.cross(e3, e1)
    G = np.array([e1, e2, e3])  # rows: the pipe frame's coordinates of world x, y, z
    yaw = math.radians(d["yaw"])
    Z = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1.0]])
    M = Z @ G  # pipe frame -> world
    o = np.array(d["origin"], float)
    pipe_w = Placement(tuple(o), tuple(tuple(float(v) for v in r) for r in M))
    b_w = Placement(tuple(M @ np.array(pl_b.origin) + o),
                    tuple(tuple(float(v) for v in r) for r in M @ np.array(pl_b.rot)))
    return pipe_w, b_w


def task_from_draw(seed: int, d: dict, level: int) -> Task:
    itf_a, itf_b, _, _ = layout(d)
    pa, pb = placements(d)
    tol = dict(DEFAULT_TOLERANCES, seat_setback=1.0, seat_overhang_max=0.5, tool_length=30.0, tool_diameter_extra=0.0,
               manufacturing={"process": "fdm", "quantity": 1, "direction": None}, required_checks=list(REQUIRED[level]))
    task = Task(f"pipe-L{level}-s{seed}", level, seed, {"A": itf_a, "B": itf_b}, [{"A": pa, "B": pb}], tol,
                {"generator": "pipe", "situation": "pipe", "test": NAMES[level]},
                generator_version=GENERATOR_VERSION, family="pipe")
    if level >= 3:
        apply_stated_load(task, {"B": d["load_kg"]})
        task.tolerances["score"] = "material"
    return task


def reference_world(d: dict):
    pa, _ = placements(d)
    return build(d).moved(pa.location())
