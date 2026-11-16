"""The pipe setup's reference, levels 1-3 (see ``round5`` for the fixed parts).

Built in the pipe's frame (axis local x); "up" is the print direction, B's
plate stands exactly upright and the pipe leans about 40 degrees from it.
Every face is upright, leans along the pipe (any face containing the pipe's
axis leans at most 40 degrees), or faces up, so the part prints without
support; and edges where two faces meet at a sharp angle are avoided by
construction (two shapes are only joined where one buries the other's edge):

* the saddle: a prism along the pipe. Its outline, seen along the axis, is the
  pipe's upper side, two flat faces square to the two bolt rows (the heads sit
  on them; each ends ``WRAP`` degrees past its row, in a face running
  straight down to the pipe, the corner between them cut back ``CORNER`` mm) and, between the rows, a web that reaches out towards B. The web is
  kept clear of every pipe bolt's tool (a band along the bolt from its head
  face outward). The prism ends at its top in a face square to the pipe
  (facing up) and runs down along the pipe to the bed; its last ``BAND`` mm
  rise straight up from the bed, so no leaning face meets the bed at a knife
  edge.
* B's slab: an upright column from the bed to B's top, ``SET`` mm behind B's
  face, with a drafted pad ``SET`` mm proud carrying the seat (as the flange
  setup). The web runs into it and is cut off inside it.
* holes: the pipe carved out, a clearance hole along every bolt, B's bolts
  through the pad and slab.
"""
from __future__ import annotations

import math

import build123d as bd
import numpy as np

from ...catalogue import BOLTS, KEYS
from .bracket import _halfspace, _prism_yz, _unit

G_SADDLE = 10.0   # mm from the pipe's surface to each row's flat head face, along the bolt
WRAP = 20.0       # deg the saddle runs round the pipe past each row
CORNER = 2.5      # mm the corner at each head face's end is cut back
EDGE = 6.0        # mm of saddle beyond the outermost heads, along the pipe
BAND = 3.0        # mm at the bed that rise straight up
SET = 1.5         # mm B's slab stands back from B's face; the pad's height
INSET = 1.5       # mm the seat stays inside B's face
DRAFT = 2.0       # mm the pad's base reaches beyond its front (sides 37 deg from upright)
SLAB = 10.0       # mm, B's slab behind the pad
SLAB_SIDE = 4.0   # mm the slab reaches beyond the pad's base on each side
TOOL = 30.0       # mm, the straight tool behind a head
TOOL_ROOM = 1.0   # mm the web keeps from a tool's band
WEB_HALF = 7.0    # mm, half the web's width
WEB_TOP_ROOM = 2.0  # mm the web's top stays below the space behind B's lowest heads, along the pipe
FOOT_OUT = 20.0   # mm the foot reaches out from the ridge towards B (the web, full width, reaches on)
NARROW = 0.6      # a head face whose normal has a component under -NARROW along B's normal meets B's slab narrowly
FOOT_ROOM = 1.0   # mm the foot's top stays below the lowest heads, along the pipe
WEB_BURY = 4.0    # mm behind B's face where the web is cut off (inside the slab)


def _poly_area(p):
    return 0.5 * sum(p[i][0] * p[(i + 1) % len(p)][1] - p[(i + 1) % len(p)][0] * p[i][1] for i in range(len(p)))


def _rows(itf_a):
    R = itf_a.face_h / 2
    rows = sorted({round(math.degrees(math.atan2(a[1], a[2])), 6) for _, a in itf_a.hole_frames})
    lo, hi = math.radians(min(rows)), math.radians(max(rows))
    d_face = R + G_SADDLE
    ridge = np.linalg.solve(np.array([_u(lo), _u(hi)]), np.array([d_face, d_face]))
    return R, lo, hi, d_face, ridge


def _u(a):
    return np.array([math.sin(a), math.cos(a)])


def outline(itf_a):
    """The saddle's outline in the pipe's cross-section (y, z), counter-clockwise."""
    R, lo, hi, d_face, ridge = _rows(itf_a)
    a_lo, a_hi = lo - math.radians(WRAP), hi + math.radians(WRAP)

    def out_to(a, fa):
        dd = _u(a)
        return dd * (d_face / float(_u(fa) @ dd))

    def corner(q, fa):
        # the head face's end: a radial face down to the pipe (square to it), the 70 deg corner where it meets the
        # head face cut back CORNER mm both ways (a knife edge under 75 deg would print thinner than a wall)
        r_hat = q / np.linalg.norm(q)
        towards_ridge = (ridge - q) / np.linalg.norm(ridge - q)
        return [q - r_hat * CORNER, q + towards_ridge * CORNER]

    q_lo, q_hi = out_to(a_lo, lo), out_to(a_hi, hi)
    c_lo, c_hi = corner(q_lo, lo), corner(q_hi, hi)
    pts = [R * _u(a_lo), c_lo[0], c_lo[1], ridge, c_hi[1], c_hi[0], R * _u(a_hi), np.zeros(2)]
    pts = [tuple(float(v) for v in p) for p in pts]
    return pts[::-1] if _poly_area(pts) < 0 else pts


def web_outline(itf_a, web_dir, web_len, web_half):
    """The web's outline in the cross-section: the hull of the ridge, the two
    points on the head faces where the heads' tool bands begin (seen from the
    ridge), and the web's far end, ``web_len`` mm out along ``web_dir``,
    ``web_half`` mm either side. The tool bands run outward along the rows,
    which spread apart, so the hull stays clear of them."""
    from .bracket import _convex

    R, lo, hi, d_face, ridge = _rows(itf_a)
    band = BOLTS[itf_a.bolt].head_d / 2 + TOOL_ROOM
    pts = [ridge]
    for row in (lo, hi):
        n = _u(row)
        t = np.array([n[1], -n[0]])
        head = n * d_face
        s = 1.0 if float((ridge - head) @ t) > 0 else -1.0
        pts.append(head + s * t * band)
    w = np.asarray(web_dir, float) / np.linalg.norm(web_dir)
    side = np.array([-w[1], w[0]])
    tip = ridge + w * web_len
    pts += [tip + side * web_half, tip - side * web_half]
    return _convex([tuple(float(v) for v in p) for p in pts])


def web_params(itf_a, itf_b, pl_b) -> dict:
    """The web, from the fixed parts: aimed at B's centre (seen along the pipe),
    long enough to run into B's slab, its top along the pipe below the space
    behind B's lowest bolt heads (their heads, keys and tools)."""
    F = np.array(pl_b.origin)
    nb = np.array(pl_b.vec((0, 0, 1)))
    Fyz = np.array([F[1], F[2]])
    _, _, _, _, ridge = _rows(itf_a)
    fb = BOLTS[itf_b.bolt]
    sx = math.sqrt(max(0.0, 1.0 - nb[0] ** 2))  # how far a round section across nb reaches along the pipe, per mm
    lows = [pl_b.point((hx, hy, 0.0))[0] + (SET + SLAB) * nb[0] for hx, hy in itf_b.holes]
    if nb[0] >= 0:
        x_top = min(lows) - (fb.head_d / 2 + 1.0) * sx - WEB_TOP_ROOM
    else:
        x_top = max(lows) + (fb.head_d / 2 + 1.0) * sx + WEB_TOP_ROOM
    return {"dir": list(Fyz / np.linalg.norm(Fyz)), "len": float(np.linalg.norm(Fyz) - np.linalg.norm(ridge)),
            "half": WEB_HALF, "x_top": float(x_top)}


def build(d: dict, lay) -> bd.Part:
    itf_a, itf_b, pl_b, up = lay
    up = np.asarray(up, float)
    R = itf_a.face_h / 2
    pb = BOLTS[itf_a.bolt]
    nb = np.array(pl_b.vec((0, 0, 1)))
    side = np.array(pl_b.vec((0, 1, 0)))
    F = np.array(pl_b.origin)

    # B: the pad (seat) and the slab. B's local x is up, local y is side
    a_half = itf_b.face_w / 2 - INSET  # along up
    b_half = itf_b.face_h / 2 - INSET  # along side
    z_b_lo = float(F @ up) - a_half - DRAFT
    z_b_hi = float(F @ up) + a_half + DRAFT
    xs = sorted({p[0] for p, _ in itf_a.hole_frames})
    z_rings = min(float(np.asarray(p) @ up) for p, _ in itf_a.hole_frames) - pb.head_d / 2 - EDGE
    z_bed = min(z_b_lo, z_rings)

    # the saddle with its web towards B
    wd = dict(web_params(itf_a, itf_b, pl_b), **d.get("web", {}))
    web_dir = wd["dir"]
    pts = outline(itf_a)
    x_hi = max(xs) + pb.head_d / 2 + EDGE if up[0] > 0 else min(xs) - pb.head_d / 2 - EDGE
    reach = max(abs(float(np.array([0.0, y, z]) @ up)) for y, z in pts) + 5.0
    x_bed = (z_bed - reach) / up[0]
    x0, x1 = sorted([x_bed, x_hi])
    saddle = _prism_yz(pts, x0, x1)
    if wd.get("len"):
        # the web: from the bed up to x_top along the pipe, below B's lowest tools
        w_out = web_outline(itf_a, web_dir, wd["len"], wd["half"])
        saddle = saddle + _prism_yz(w_out, x0, wd["x_top"])
        # the foot: below the lowest heads, the saddle and the web as one convex block, so no face of the saddle
        # that meets B's slab at a narrow angle reaches it
        from .bracket import _convex

        x_f = min(xs) - pb.head_d / 2 - FOOT_ROOM if up[0] > 0 else max(xs) + pb.head_d / 2 + FOOT_ROOM
        _, _, _, _, ridge = _rows(itf_a)
        w = np.asarray(web_dir, float) / np.linalg.norm(web_dir)
        sd = np.array([-w[1], w[0]])
        stub = [tuple(ridge + w * FOOT_OUT + sd * k * wd["half"]) for k in (-1.0, 1.0)]
        # it covers the head face of each row that would meet B's slab at a narrow angle (its normal within
        # NARROW of facing B's), from the ridge to exactly where that face ends (one of the saddle's corners, so
        # no face of the foot runs nearly along one of the saddle's: a sliver)
        _, lo, hi, _, _ = _rows(itf_a)
        nb_yz = np.array([nb[1], nb[2]])
        ends = [q for q in pts if abs(math.hypot(*q) - R) > 1e-6 and math.hypot(*q) > 1e-6]
        ends = sorted(ends, key=lambda q: math.atan2(q[0], q[1]))
        head_ends = [e for e, row in ((ends[1], lo), (ends[-2], hi)) if float(_u(row) @ nb_yz) < -NARROW]
        saddle = saddle + _prism_yz(_convex(head_ends + [tuple(ridge)] + stub), x0, x_f)
    # everything stops inside B's slab
    saddle = saddle - _halfspace(F + nb * WEB_BURY, nb)

    # B's slab: an upright column from the bed to the pad's top
    sb = b_half + DRAFT + SLAB_SIDE
    plane = bd.Plane(origin=tuple(F - up * (float(F @ up) - z_bed)), x_dir=tuple(nb), z_dir=tuple(up))
    rect = bd.Polygon((SET, -sb), (SET + SLAB, -sb), (SET + SLAB, sb), (SET, sb), align=None)
    slab = bd.extrude(plane * rect, z_b_hi - z_bed, dir=tuple(up))
    # the pad: loft from its base (on the slab) to the seat
    def quad(half_u, half_s, off):
        return [tuple(F + nb * off + up * su * half_u + side * ss * half_s)
                for su, ss in ((-1, -1), (1, -1), (1, 1), (-1, 1))]

    base = bd.Wire.make_polygon(quad(a_half + DRAFT, b_half + DRAFT, SET), close=True)
    front = bd.Wire.make_polygon(quad(a_half, b_half, 0.0), close=True)
    pad = bd.Solid.make_loft([base, front], ruled=True)

    body = saddle + slab + pad
    # the bed
    body = body - _halfspace(up * z_bed, up)
    # the pipe carved out (before and after the band, which must not fill it), its bolts' holes along their axes
    pipe = bd.Pos((x0 + x1) / 2, 0, 0) * bd.Rot(0, 90, 0) * bd.Cylinder(R, abs(x1 - x0) + 600.0)
    body = body - pipe
    body = _band(body, up, nb, z_bed) - pipe
    for p, a in itf_a.hole_frames:
        body = body - bd.Plane(origin=tuple(p), z_dir=tuple(a)) * bd.Pos(0, 0, G_SADDLE / 2) * bd.Cylinder(
            pb.clear_medium / 2, G_SADDLE + 4)
    fb = BOLTS[itf_b.bolt]
    for hx, hy in itf_b.holes:
        p = np.array(pl_b.point((hx, hy, 0.0)))
        L = SET + SLAB + 2.0
        body = body - bd.Plane(origin=tuple(p - nb), z_dir=tuple(nb)) * bd.Pos(0, 0, L / 2) * bd.Cylinder(
            fb.clear_medium / 2, L)
    solids = body.solids()
    return max(solids, key=lambda s: s.volume) if len(solids) > 1 else solids[0]


def _band(body, up, e1, z_bed):
    """The bottom BAND mm: the section at BAND above the bed, extruded straight down to it."""
    cut = bd.Plane(origin=tuple(up * (z_bed + BAND)), x_dir=tuple(e1), z_dir=tuple(up))
    sec = body & (cut * bd.Pos(0, 0, -0.001) * bd.Box(4000.0, 4000.0, 0.002))
    faces = [f for f in sec.faces() if abs(float(np.array(tuple(f.normal_at(f.center()))) @ up)) > 0.99
             and float(np.array(tuple(f.center())) @ up) > z_bed + BAND - 0.002]
    if not faces:
        return body
    band = None
    for f in faces:
        piece = bd.extrude(f, BAND + 0.5, dir=tuple(-up))
        band = piece if band is None else band + piece
    body = (body - _halfspace(up * (z_bed + BAND), up)) + band
    return body - _halfspace(up * z_bed, up)
