"""Clamp a pipe, the showcase setup: a round flange (A) and a pipe (B) with
bolts in two rows round it; one solid part bolts to both.

The reference is a bracket printed standing on its end, along the pipe's axis
(the print direction is the pipe's axis, local +x of the pipe):

* a saddle: a prism along the pipe whose outline, seen along the axis, is the
  wedge between two flat faces square to the two bolt rows (the heads sit on
  them) and the pipe itself carved out of it;
* a plate on the flange: the flange's face outline set in by the seat setback,
  ``PLATE_T`` thick, its rim square to the print direction (a prism of the
  outline seen along the axis, cut by the two faces of the plate), with a
  teardrop recess for the flange's boss (round, with a 45-degree roof on the
  upper side, so it prints without support) and a hole at every bolt;
* an arm from the saddle's ridge to the back of the plate, a prism along the
  axis from the print bed up.

Every face of it is vertical, or leans less than 45 degrees, in the print
direction; every hole runs square to the print direction and is under 10 mm,
or is the teardrop. The geometry is built in the pipe's own frame (axis local
x, the bolt side round local +z) and then placed with the pipe.
"""
from __future__ import annotations

import math

import build123d as bd
import numpy as np

from ...catalogue import BOLTS
from ...geometry import Placement

PLATE_T = 9.0       # mm, the plate on the flange
SADDLE_G = 9.0      # mm from the pipe's surface to each row's flat head face, along the bolt
SADDLE_WRAP = 22.0  # deg the saddle runs past each outer row, round the pipe
ARM_W = 26.0        # mm, the arm's width across
ARM_X = 30.0        # mm, how far up the print direction the arm reaches past the plate's centre
SETBACK = 2.6       # mm the plate's round part stays inside the flange's face
TIP_SIDE = 42.0     # deg from the print direction, the sides of the plate's teardrop point
TIP_RECESS = 5.0    # mm the teardrop's point stands back from the flange
SEAT_SLOPE = 1.0    # the step round the seat: mm in per mm back (45 degrees), so it does not overhang
ARM_JOIN = 50.0     # deg, the least angle at which the arm's side leaves a head face
ARM_NEAR = 24.0     # mm behind the plate in which the arm keeps inside the plate's outline
TOOL_ROOM = 30.5    # mm behind each flange bolt's head kept free: the straight tool (30 mm) with room
BOSS_ROOM = 0.6     # mm round and beyond the flange's boss
EDGE = 4.0          # mm of saddle beyond the outermost heads, along the pipe


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _halfspace(point, normal, size=2000.0):
    """Everything on the side of the plane opposite ``normal`` (normal points out)."""
    n = _unit(normal)
    return bd.Plane(origin=tuple(np.asarray(point, float) - n * size / 2), z_dir=tuple(n)) * bd.Box(
        size, size, size)


def _prism_yz(pts_yz, x0, x1):
    """A prism along local x from x0 to x1 over a polygon in the (y, z) plane."""
    plane = bd.Plane(origin=(x0, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    face = plane * bd.Polygon(*[(float(y), float(z)) for y, z in pts_yz], align=None)
    return bd.extrude(face, x1 - x0, dir=(1, 0, 0))  # always toward +x, whichever way the outline winds


def build(pipe_itf, flange_itf, flange_local: Placement) -> bd.Part:
    """The reference in the pipe's own frame. ``flange_local`` places fixture A
    (the flange) in that frame."""
    R = pipe_itf.face_h / 2
    pb = BOLTS[pipe_itf.bolt]
    xs = sorted({p[0] for p, _ in pipe_itf.hole_frames})
    rows = sorted({round(math.degrees(math.atan2(a[1], a[2])), 6) for _, a in pipe_itf.hole_frames})
    amax = max(abs(r) for r in rows)
    x_lo = min(xs) - pb.head_d / 2 - EDGE
    x_hi = max(xs) + pb.head_d / 2 + EDGE

    # the saddle: the wedge between the rows' head faces, down past the outer rows, the pipe carved out
    faces = [(math.radians(r), R + SADDLE_G) for r in rows]
    def u(a):
        return np.array([math.sin(a), math.cos(a)])
    a_side = math.radians(amax + SADDLE_WRAP)
    z_cut = R * math.cos(a_side)
    # the polygon's corners: where each head face meets the bottom cut, and the ridge(s) between faces
    pts = []
    lo_face, hi_face = min(faces), max(faces)
    for (a, d), sign in ((lo_face, -1.0), (hi_face, 1.0)):
        ua = u(a)
        # point on the face line with z = z_cut: ua . p = d
        y = (d - ua[1] * z_cut) / ua[0] if abs(ua[0]) > 1e-9 else sign * R
        pts.append((y, z_cut))
    ridge = np.linalg.solve(np.array([u(lo_face[0]), u(hi_face[0])]), np.array([lo_face[1], hi_face[1]]))
    # end the wrap with faces square to the pipe's surface (radial): no knife edge where the seat meets the pipe
    def on_circle(a):
        return (R * math.sin(a), R * math.cos(a))

    def out_to(a, face):
        fa, fd = face
        ua = u(fa)
        d = np.array([math.sin(a), math.cos(a)])
        t_ = fd / float(ua @ d)
        return tuple(t_ * d)
    poly = [on_circle(-a_side), out_to(-a_side, lo_face), tuple(ridge), out_to(a_side, hi_face), on_circle(a_side),
            (0.0, 0.0)]
    saddle = _prism_yz(poly, x_lo, x_hi)

    # the plate on the flange
    F = np.asarray(flange_local.origin, float)
    nf = _unit(flange_local.vec((0, 0, 1)))  # into the part
    fx, fy = _unit(flange_local.vec((1, 0, 0))), _unit(flange_local.vec((0, 1, 0)))
    rp = flange_itf.face_w / 2 - SETBACK
    # The plate stands nearly upright in print, so a round plate's lower rim would overhang. Its outline
    # is a teardrop instead: the circle, plus a point below it (in the print direction) whose sides slope
    # TIP_SIDE from the vertical. Only the circle touches the flange; the point stands TIP_RECESS back from
    # it, so the seat stays inside the flange's face.
    up = _unit(np.array([1.0, 0.0, 0.0]) - nf * nf[0])
    side = np.cross(nf, up)
    h = math.radians(TIP_SIDE)
    tip = 1.03 * rp / math.sin(h)  # just past tangent: its sides clear the circle
    pl0 = bd.Plane(origin=tuple(F), x_dir=tuple(side), z_dir=tuple(nf))
    # in this plane local +y is nf x side = -up: down in print
    w = tip * math.tan(h)  # the triangle's half-width at the circle's centre line
    drop = bd.Circle(rp) + bd.Polygon((-w, 0.0), (w, 0.0), (0.0, tip), align=None)
    drop0 = drop
    # the seat: the round part, inside the flange's outline; the point stands TIP_RECESS back from the flange,
    # the step between them sloping so that it prints (cut from the whole part, last)
    r_seat = flange_itf.face_w / 2 - 1.5  # half a millimetre inside the 1 mm setback line
    drop0 = drop
    plate = bd.extrude(pl0 * drop, PLATE_T, dir=tuple(nf))
    fb_ring0 = BOLTS[flange_itf.bolt].head_d / 2 + max(math.hypot(hx, hy) for hx, hy in flange_itf.holes)
    x_bed_plate = float(F[0]) - (flange_itf.face_w / 2 + 0.5) * abs(float(up[0]))
    # its holes: each bolt square to the flange, and the boss's teardrop recess (cut from the whole part, last)
    recesses = []
    fb = BOLTS[flange_itf.bolt]
    for hx, hy in flange_itf.holes:
        p = flange_local.point((hx, hy, 0.0))
        recesses.append(bd.Plane(origin=tuple(p), z_dir=tuple(nf)) * bd.Pos(0, 0, PLATE_T / 2) * bd.Cylinder(
            fb.clear_medium / 2, PLATE_T + 4))
        # the head's way out and the straight tool's room: a teardrop channel along the bolt behind the plate
        rt = fb.head_d / 2 + 1.2
        k = rt * math.sqrt(0.5)
        tool = bd.Circle(rt) + bd.Polygon((-k, -k), (k, -k), (0.0, -rt * math.sqrt(2.0)), align=None)
        low = float((p + nf * PLATE_T)[0]) - rt * abs(float(up[0]))  # the channel's lowest point, in print
        if low - x_bed_plate < 3.0:  # just above the base: open it downward (local +y is down), no thin floor
            tool = tool + bd.Pos(0, 40.0) * bd.Rectangle(2 * rt, 80.0)
        tpl = bd.Plane(origin=tuple(p + nf * PLATE_T), x_dir=tuple(side), z_dir=tuple(nf))
        recesses.append(bd.extrude(tpl * tool, fb.head_h + TOOL_ROOM, dir=tuple(nf)))
    for pr in flange_itf.protrusions:
        c = flange_local.point((pr.x, pr.y, 0.0))
        r = pr.radius + BOSS_ROOM
        depth = pr.height + BOSS_ROOM
        # a teardrop recess: round, with a 45-degree roof on the upper side (local -y is up here, see above)
        pl = bd.Plane(origin=tuple(c - nf * 1.0), x_dir=tuple(side), z_dir=tuple(nf))
        k = r * math.sqrt(0.5)
        drop = bd.Circle(r) + bd.Polygon((-k, -k), (k, -k), (0.0, -r * math.sqrt(2.0)), align=None)
        recesses.append(bd.extrude(pl * drop, depth + 1.0, dir=tuple(nf)))

    # the arm: from the saddle's ridge to the back of the plate, a prism from the bed up
    back = F - nf * 20.0  # aimed past the plate's centre; cut flush on its back face below (no end face left)
    # the arm's outline: its plate end ARM_W wide, its foot on the two head faces, each foot point far enough
    # down its face that the arm's side leaves the face at ARM_JOIN degrees or more (no narrow crevice)
    end = np.array([back[1], back[2]])
    along = _unit(end - np.array(ridge))
    across = np.array([-along[1], along[0]]) * ARM_W / 2
    e_lo, e_hi = end - across, end + across
    if (e_lo - np.array(ridge)) @ (np.array(pts[0]) - np.array(ridge)) < (e_hi - np.array(ridge)) @ (np.array(pts[0]) - np.array(ridge)):
        e_lo, e_hi = e_hi, e_lo  # e_lo: the end corner on the lo face's side

    def foot(face_pt, end_pt):
        ray = _unit(np.array(face_pt) - np.array(ridge))  # down the face, away from the ridge
        best = None
        for s_ in np.arange(2.0, 40.0, 1.0):
            q = np.array(ridge) + ray * s_
            edge = _unit(end_pt - q)
            ang = math.degrees(math.acos(max(-1.0, min(1.0, float(ray @ edge)))))
            if ang >= ARM_JOIN:  # the angle outside, between the face beyond the foot and the arm's side
                best = q
                break
        return best if best is not None else np.array(ridge) + ray * 20.0

    q_lo, q_hi = foot(pts[0], e_lo), foot(pts[-1] if len(pts) > 1 else pts[0], e_hi)
    arm_pts = _convex([tuple(q_lo), tuple(ridge), tuple(q_hi), tuple(e_hi), tuple(e_lo)])
    arm = _prism_yz(arm_pts, x_lo, min(x_hi, float(back[0]) + ARM_X / 2))

    # one flat base for the whole part: at the plate's lowest bolt ring, or the saddle's foot if lower
    fb_ring = BOLTS[flange_itf.bolt].head_d / 2 + max(math.hypot(hx, hy) for hx, hy in flange_itf.holes)
    x_bed = min(x_lo, float(F[0]) - (flange_itf.face_w / 2 + 0.5) * abs(float(up[0])))  # 1.5 mm below the round seat's lowest point
    if x_bed < x_lo:  # the saddle and the arm reach down to it
        saddle = saddle + _prism_yz(poly, x_bed, x_lo + 1.0)
        arm = arm + _prism_yz(arm_pts, x_bed, x_lo + 1.0)
    # the arm stops flush on the plate's back face, and near the plate it keeps inside the plate's outline
    # (its sides there continue the plate's rim): no end face or edge of it is left beside the plate
    arm = arm - _halfspace(F + nf * PLATE_T, nf)
    near = (_halfspace(F + nf * (PLATE_T + ARM_NEAR), nf) - _halfspace(F + nf * PLATE_T, nf))
    rim = bd.extrude(pl0 * drop0, PLATE_T + ARM_NEAR + 1.0, dir=tuple(nf))
    arm = arm - (near - rim)
    body = saddle + arm + plate
    body = body - _halfspace(np.array([x_bed, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))  # nothing below the base
    body = body - _halfspace(F, nf)  # nothing on the flange's side of its face
    for rc in recesses:
        body = body - rc
    near = _halfspace(F + nf * TIP_RECESS, nf) - _halfspace(F, nf)
    # the seat's edge at the flange is the circle r_seat; the step widens away from the flange at 45 degrees
    seat = bd.Plane(origin=tuple(F - nf * 0.5), z_dir=tuple(nf)) * bd.Pos(0, 0, (TIP_RECESS + 1.0) / 2) * bd.Cone(
        r_seat - 0.5 * SEAT_SLOPE, r_seat + (TIP_RECESS + 0.5) * SEAT_SLOPE, TIP_RECESS + 1.0)
    body = body - (near - seat)
    body = body - bd.Pos((x_lo + x_hi) / 2, 0, 0) * bd.Rot(0, 90, 0) * bd.Cylinder(R, x_hi - x_lo + 400.0)
    # the pipe's bolts: a clearance hole along each, the head on its row's flat face
    for p, a in pipe_itf.hole_frames:
        P, V = np.asarray(p, float), np.asarray(a, float)
        body = body - bd.Plane(origin=tuple(P), z_dir=tuple(V)) * bd.Pos(0, 0, SADDLE_G / 2) * bd.Cylinder(
            pb.clear_medium / 2, SADDLE_G + 4)
    solids = body.solids()
    return max(solids, key=lambda s: s.volume) if len(solids) > 1 else solids[0]


def _convex(pts):
    """The convex hull of 2D points, counter-clockwise (monotone chain)."""
    P = sorted(set((round(x, 6), round(y, 6)) for x, y in pts))
    if len(P) < 3:
        return P

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in P:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(P):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]
