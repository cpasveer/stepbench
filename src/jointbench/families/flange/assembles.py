"""Flange and block, level 4 (assembles): two parts, put in one after the other
through one small opening in a closed box, bolted together inside.

The fixed parts are the flange setup's (``shell``) for the same seed, as at
levels 1-3: a round flange A and a tall block B, faces upright and bent in
plan. A closed box stands round both, one small opening in its front wall.
Everything of level 3 holds per part (fit, clean, key turn, printable in PLA)
and for the assembly (the stated load, the plastic round the joint bolts,
stiffness); level 4 adds the ways in, the joint (catalogue through bolts and
nuts: the model states size, count and length) and a key reaching every joint
bolt from outside.

The hidden reference is level 3's ring cut in two and opened on one side. Both
parts are upright prisms in plan (they print standing on the bed, as at level
3), each with its drafted seat pad:

* part 1, on A: A's slab and, from its end on one side, a straight side wall
  running towards B's slab;
* part 2, on B: B's slab and the rest of that side wall.

The side wall runs from the back corner of A's slab to the back corner of B's
slab on the same side (the line ``Q_A -> Q_B``; ``s`` along it, ``w`` square
to it, outwards). It is two layers thick: the inner layer (``0 <= w <= tj``)
and the outer (``tj <= w <= 2 tj``), each as thick as the joint bolt's length
allows. Near A both layers are part 1's, near B both are part 2's; in the
middle part 1 keeps the inner layer, part 2 the outer, and the two lie face to
face on the plane ``w = tj``: the joint, two bolts one above the other through
both layers, heads outside on part 2, nuts inside on part 1. Where a layer
changes hands there is a gap ``GAP`` mm wide, so the parts touch only at the
joint face. The other side of the ring is left open.

An open ring is about half as stiff as level 3's closed one, so: B's slab (B
carries the load) is ``EXTRA_B`` mm thicker than level 3's, its heads sunk in
teardrop pockets (the grip stays level 3's); each slab is thickened behind,
beside its bolts, towards the wall (``GUSSET``), clear of its own heads and of
the other part's keys; the joint sits in the middle of the stretch of wall
that both slabs leave free. Where a part would stand on the bed too tall for
the width of its outline, a low foot at its slab's far end widens the stance.

Part 1 goes in first: along A's face (square to A's normal, in from the
opening's side), ``ONTO`` mm plus A's boss off it, then onto it. Part 2 goes
in along B's face, square to B's normal, and then onto B within the joint plane
(its outer layer slides over part 1's inner layer onto it). The box's opening
is drawn round what passes it and round the joint keys' line of sight.
"""
from __future__ import annotations

import json
import math
import os

import build123d as bd
import numpy as np

from ...catalogue import BOLTS, NUTS, lengths
from ...geometry import Placement
from ...task import Obstacle, Task
from . import shell as S

LEVEL = 4
WALL = 6.0       # mm, the box's walls
CLEAR = 2.5      # mm, the opening beyond what passes it, each side
ONTO = 2.0       # mm each part comes the last way onto its face (beyond A's boss)
GAP = 5.0        # mm where a layer of the side wall changes hands (part 2's last move, and room to spare)
RING = 3.0       # mm of plastic round a joint head or nut beyond its diameter
MARGIN = 30.0    # mm the box stands beyond what it holds
JOINT_SIZE, JOINT_LENGTH = "M5", 25.0
EXTRA_B = 6.0     # mm B's slab is thicker than level 3's, behind the heads' seat (stiffness: the ring is open)
EXTRA = 0.0       # mm each slab is thicker than level 3's, behind the heads' seat (stiffness: the ring is open)
POCKET_ROOM = 1.0  # mm round a sunk head, radially
GUSSET = 22.0     # mm at most a slab is thickened behind, beside its bolts, towards the wall (stiffness: the ring is open)
GUSSET_ROOM = 2.0  # mm the thickening stays clear of the outermost head
JOINT_AT = 0.0    # where in the free stretch of wall the joint sits, 0 at A's end: part 2's wall the longer
CHAMFER = 3.0     # mm along each side, a sharp corner (under 90 deg) of a level 5 wall end
FOOT_H = 3.0      # mm, a foot on the bed where part 1 would stand too tall for its outline (FDM_STANCE)
FOOT_W = 8.0      # mm, the foot's width
STANCE_MAX = 3.0 / 1.08  # height over stance width the reference keeps to (the printing rule's 3, with room)


def _u(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _r(v):
    return [round(float(x), 6) for x in v]


def _rot(e1, e2, e3):
    m = np.stack([e1, e2, e3], axis=1)
    return tuple(tuple(float(m[i][j]) for j in range(3)) for i in range(3))


def _points(shape, tol=0.2) -> np.ndarray:
    v, _ = shape.tessellate(tol)
    return np.array([tuple(p) for p in v])


def _two(v) -> np.ndarray:
    return np.array([float(v[0]), float(v[1])])


def _prism(poly, z0: float, z1: float) -> bd.Part:
    face = bd.Face(bd.Wire.make_polygon([bd.Vector(float(x), float(y), z0) for x, y in poly], close=True))
    return bd.extrude(face, amount=z1 - z0, dir=(0, 0, 1))


def _width(pts) -> float:
    """The narrowest width of points in plan (between two parallel lines holding them all)."""
    from scipy.spatial import ConvexHull

    P = np.asarray(pts, float)
    H = P[ConvexHull(P).vertices]
    best = math.inf
    for i in range(len(H)):
        e = H[(i + 1) % len(H)] - H[i]
        nrm = np.array([-e[1], e[0]]) / np.linalg.norm(e)
        proj = H @ nrm
        best = min(best, float(proj.max() - proj.min()))
    return best


def _meet(p, d, q, e) -> np.ndarray:
    """Where the line p + a d meets the line q + b e (in plan)."""
    a, _ = np.linalg.solve(np.array([d, -e]).T, q - p)
    return p + a * d


# --------------------------------------------------------------------------
# the fixed parts: level 3's task and draw for a seed


def setup(seed: int = 0):
    """(level 3 task, draw): the flange setup's, exactly as level 3 generates it."""
    task, d, _ = S.generate(seed, 3)
    return task, d


# --------------------------------------------------------------------------
# the reference


def build(task3: Task, d: dict, *, size: str = JOINT_SIZE, length: float = JOINT_LENGTH, side: int = 1,
          variant: dict | None = None):
    """(part 1, part 2, assembly file, dimensions), world coordinates. ``size``, ``length``: the joint
    bolts' (the length sets the layers' thickness). ``side``: +1 puts the side wall at the end of A's
    slab towards +u (A's own in-plane horizontal), -1 at the other. ``variant`` makes broken copies.

    B's slab (B carries the load) is made thicker than level 3's by EXTRA_B, its heads sunk in pockets,
    or by less where the side wall then has no room for the joint."""
    var = dict(variant or {})
    if var.get("butt") is None and "slab_extra_B" not in var and "slab_extra" not in var:
        for e in (EXTRA_B, EXTRA_B - 2.0, EXTRA_B - 4.0, 0.0):
            try:
                return _build(task3, d, size=size, length=length, side=side, variant=dict(var, slab_extra_B=e))
            except WallTooShort:
                if e == 0.0:
                    raise
    return _build(task3, d, size=size, length=length, side=side, variant=var)


class WallTooShort(ValueError):
    pass


def _build(task3: Task, d: dict, *, size: str, length: float, side: int, variant: dict):
    var = dict(variant)
    lay = S.layout(d)
    ia, ib = lay["itf"]["A"], lay["itf"]["B"]
    na3, nb3 = (np.array(v, float) for v in (lay["normals"]["A"], lay["normals"]["B"]))
    ua3, ub3 = np.array(S._side(na3)), np.array(S._side(nb3))
    na, nb, ua, ub = _two(na3), _two(nb3), _two(ua3), _two(ub3)
    ca = np.zeros(2)
    cb3 = np.array(lay["placements"]["B"].origin, float)
    cb = _two(cb3)
    b, nut = BOLTS[size], NUTS[size]
    tj = (length - nut.m - 1.5 * nut.pitch) / 2  # each layer: the bolt stands out of its nut 1.5 pitches
    if not (b.grip_min <= tj <= b.grip_max):
        raise ValueError("joint layers out of range for this bolt length")
    seat = S.SET + S.SLAB  # the heads' seat behind the face: level 3's grip
    extras = {k: float(var.get(f"slab_extra_{k}", var.get("slab_extra", EXTRA))) for k in "AB"}
    backs = {k: seat + e for k, e in extras.items()}  # each slab's back: as level 3's, or thicker, its heads sunk
    half_a = ia.face_w / 2 + S.SIDE_ROOM
    half_b = ib.face_w / 2 + S.SIDE_ROOM
    # the side wall's inner line: from the back corner of A's slab to the back corner of B's on the same side
    qa = ca + seat * na + side * half_a * ua
    sb = min((1.0, -1.0), key=lambda k: float(np.linalg.norm(cb + k * half_b * ub - qa)))  # B's end nearer it
    qb = cb + seat * nb + sb * half_b * ub
    sh = _u(qb - qa)
    w = np.array([-sh[1], sh[0]])
    if float(w @ (qa - (ca + cb) / 2)) < 0:
        w = -w  # outwards
    L = float((qb - qa) @ sh)
    T = float(var.get("wall_t", 2 * tj))  # the side wall: the two joint layers (level 5: as thick as it needs)
    ring = max(b.head_d, nut.e) / 2 + RING

    def at(s_, w_):
        return qa + s_ * sh + w_ * w

    # each slab, from its far end to where its back and front faces meet the wall's outer line
    def slab(c, n, u, half, q, sgn, back):
        f0 = c + S.SET * n - sgn * half * u
        b0 = c + back * n - sgn * half * u
        b1 = _meet(c + back * n, sgn * u, at(0, T), sh)
        f1 = _meet(c + S.SET * n, sgn * u, at(0, T), sh)
        return [tuple(f0), tuple(f1), tuple(b1), tuple(b0)]

    slab_a = slab(ca, na, ua, half_a, qa, side, backs["A"])
    slab_b = slab(cb, nb, ub, half_b, qb, sb, backs["B"])
    # the slabs' reach along the wall's outer line: the layers change hands clear of them
    def near(poly):  # its corners by the wall (not the slab's far end)
        return [np.array(p) for p in poly if float((np.array(p) - qa) @ w) > -1.0]

    s_a_hi = max(float((p - qa) @ sh) for p in near(slab_a))
    s_b_lo = min(float((p - qa) @ sh) for p in near(slab_b))
    def end_max(w_lo, w_hi):
        """How far along the wall part 1's wall (between w_lo and w_hi) may end: part 2 comes in along
        B's face, its slab sweeping the whole strip behind B's face, so part 1 stays 1 mm in front of
        that slab's back face."""
        k = -float(sh @ nb)
        return min((float((qa - cb) @ nb) + w_ * float(w @ nb) - backs["B"] - 1.0) / k for w_ in (w_lo, w_hi))

    def corner(c, n, q, s0, step, slab_poly):
        """The wall's end joined to its slab: the hull of the slab's end on the wall's outer line and the
        wall's end, which starts at s0 or as far along (``step``) as keeps its outer corner behind the
        slab's front (so nothing stands out in front of the slab)."""
        from scipy.spatial import ConvexHull

        e = 0.0
        while float((at(s0 + step * e, T) - c) @ n) < S.SET + 0.5:
            e += 0.25
            if e > L / 2:
                raise ValueError("the side wall does not meet its slab")
        s_end = s0 + step * e
        pts = np.array([q, at(s_end, 0.0), at(s_end, T), np.array(slab_poly[1]), np.array(slab_poly[2])])
        return [tuple(pts[i]) for i in ConvexHull(pts).vertices], s_end

    lj = 2 * ring + 4.0  # the joint face along the wall
    if var.get("narrow"):  # broken copy: the joint face 1 mm wider than the holes, along the wall
        lj = b.clear_medium + 2.0
    butt = var.get("butt")  # level 5: the wall's two halves meet end to end, this far apart (no joint)
    g2 = butt if butt is not None else GAP
    if butt is not None:
        lj = 0.0
    ja, sa0 = corner(ca, na, qa, 0.0, 1.0, slab_a)
    jb, sb0 = corner(cb, nb, qb, L, -1.0, slab_b)
    # where the wall changes hands: s1, where part 1's full wall ends (part 2's outer layer, or at level 5
    # its whole wall, starts g2 on), and s2 = s1 + g2 + lj, where part 1's inner layer ends (part 2's full
    # wall starts g2 on). Each part's wall reaches from its own slab, clear of the other part's slab: in
    # the middle of the room that leaves
    lo = max(sa0 + 1.0, s_a_hi + 1.0)  # part 1's full wall from its corner, covering its slab's tip
    if butt is None:  # part 1's inner layer short of B's slab; part 2's full wall from its corner, over its tip
        hi = min(end_max(0.0, tj) - GAP - lj, s_b_lo - 1.0 - 2 * GAP - lj, sb0 - 1.0 - 2 * GAP - lj)
    else:
        hi = min(end_max(0.0, T), s_b_lo - 1.0 - butt, sb0 - 1.0 - butt)
    if lo > hi:
        raise WallTooShort(f"the side wall is too short for the joint ({L:.1f} mm; the room along it runs from "
                         f"{lo:.1f} to {hi:.1f})")
    s1 = lo + float(var.get("joint_at", JOINT_AT)) * (hi - lo)
    s2 = s1 if butt is not None else s1 + GAP + lj
    lay_z = S_z(lay)
    z_bed, z_top = lay_z
    top2 = z_top + float(var.get("taller", 0.0))  # broken copy: part 2 too tall for the opening

    def quad(s0, s1_, w0, w1):
        return [tuple(at(s0, w0)), tuple(at(s1_, w0)), tuple(at(s1_, w1)), tuple(at(s0, w1))]

    # part 1: A's slab, the full wall up to s1, the inner layer on to s2
    x_ends = None
    if butt is not None and var.get("ends_x"):
        # level 5: the wall ends square to A's axis (x), the gap ``butt`` along it: a turn about that axis
        # slides them past each other. Where an end meets the wall's side at a sharp angle, chamfered
        x1 = float(at(s1, T / 2)[0])
        x_ends = (x1, x1 + butt)

        def end_poly(s_far, x_e):
            pi = _meet(qa, sh, np.array([x_e, 0.0]), np.array([0.0, 1.0]))
            po = _meet(at(0.0, T), sh, np.array([x_e, 0.0]), np.array([0.0, 1.0]))
            bi, bo = at(s_far, 0.0), at(s_far, T)
            out = [bi]
            for p_, prev, nxt in ((pi, bi, po), (po, pi, bo)):
                a_, c_ = _u(prev - p_), _u(nxt - p_)
                if float(a_ @ c_) > 0.0:  # under 90 deg: chamfered
                    out += [p_ + a_ * CHAMFER, p_ + c_ * CHAMFER]
                else:
                    out.append(p_)
            out.append(bo)
            return [tuple(v) for v in out]

        p1_faces = [slab_a, ja, end_poly(sa0, x_ends[0])]
        p2_faces = [slab_b, jb, end_poly(sb0, x_ends[1])]
    elif butt is not None:
        p1_faces = [slab_a, ja, quad(sa0, s1, 0.0, T)]
        p2_faces = [slab_b, jb, quad(s1 + butt, sb0, 0.0, T)]
    else:
        p1_faces = [slab_a, ja, quad(sa0, s1, 0.0, T), quad(s1 - 1.0, s2, 0.0, tj)]
        p2_faces = [slab_b, jb, quad(s2 + GAP, sb0, 0.0, T), quad(s1 + GAP, s2 + GAP + 1.0, tj, T)]

    def gusset(c, n, u, sgn, name, itf, q, s_lim):
        """The slab thickened behind, from beside its bolts (clear of their heads, tools and keys by
        GUSSET_ROOM) to the wall, at most ``gus`` deep and only as far along the wall as ``s_lim``."""
        from scipy.spatial import ConvexHull

        pl = lay["placements"][name]
        ub_ = max(float((_two(pl.point((x, y, 0.0))) - c) @ (sgn * u)) for x, y in itf.holes)
        u_g = ub_ + BOLTS[itf.bolt].head_d / 2 + GUSSET_ROOM
        along = float(sh @ n) if name == "A" else -float(sh @ n)  # the wall's run away from this slab
        depth = min(gus, abs(s_lim) * abs(along) - (backs[name] - seat))  # (a thicker slab uses up some)
        if u_g > float((q - c) @ (sgn * u)) - 2.0:
            return None
        back = backs[name]
        while depth >= 2.0:
            p0 = c + back * n + sgn * u_g * u
            p1 = c + (back + depth) * n + sgn * u_g * u
            p2 = _meet(c + (back + depth) * n, sgn * u, q, sh)
            pts = np.array([p0, p1, p2, q])
            poly = [tuple(pts[i]) for i in ConvexHull(pts).vertices]
            pr = _prism(poly, z_bed, z_top)
            if all(((pr & k).volume if (pr & k) is not None else 0.0) < 1e-6 for k in keys[name]):
                return poly
            depth -= 1.0  # in the other part's keys' way: shallower
        return None

    # each fixed bolt's key, as TOOL_ACCESS takes it (the head's diameter, the tool's length beyond the
    # head) and 1 mm more all round: a thickening of the other part stays out of it
    keys = {"A": [], "B": []}
    for name_, itf_, n_ in (("A", ia, na3), ("B", ib, nb3)):
        bb_ = BOLTS[itf_.bolt]
        for x, y in itf_.holes:
            p_ = np.asarray(lay["placements"][name_].point((x, y, 0.0)), float) + n_ * seat
            L_ = bb_.head_h + S.TOOL + 2.0
            cyl = bd.Plane(origin=tuple(p_), z_dir=tuple(n_)) * bd.Pos(0, 0, L_ / 2) * bd.Cylinder(bb_.head_d / 2 + 1.0, L_)
            keys["B" if name_ == "A" else "A"].append(cyl)  # A's keys bar B's thickening and the other way

    gus = float(var.get("gusset", GUSSET))
    if gus > 0:
        if x_ends is not None:  # part 1's wall end square to x: where it meets the wall's inside, less its chamfer
            pi_ = _meet(qa, sh, np.array([x_ends[0], 0.0]), np.array([0.0, 1.0]))
            a_end = float((pi_ - qa) @ sh) - CHAMFER
        else:
            a_end = s1 if butt is not None else s1 + GAP
        for faces, args in ((p1_faces, (ca, na, ua, side, "A", ia, qa, a_end - 1.0)),
                            (p2_faces, (cb, nb, ub, sb, "B", ib, qb, L - (s2 + g2 + 1.0)))):
            g_ = gusset(*args)
            if g_ is not None:
                faces.append(g_)

    def body(faces, z0, z1):
        """The upright prisms of the plan pieces, fused into one (their seams merged)."""
        out = None
        for poly in faces:
            pr = _prism(poly, z0, z1)
            out = pr if out is None else out + pr
        out = out.clean()
        if len(out.solids()) != 1:
            raise ValueError(f"a part's plan falls apart: {[[tuple(round(float(c), 1) for c in v) for v in p] for p in faces]}")
        return out

    # a foot: where part 1 would stand on the bed too tall for the width of its outline, a low plate on
    # the bed at its slab's far end, reaching into the ring along A's normal as far as the stance needs
    # Part 2's foot is at its slab's wall end, reaching back past B's face plane beside B's block (a foot
    # into the ring would sweep through part 1's wall as part 2 comes in along B's face; this corner
    # comes in from outside and never passes the block). The joint sits towards A: part 2's wall is the
    # longer, its foot the shorter
    def foot(faces, c, n, u, sgn, back, height):
        pts = np.array([v for f in faces for v in f], float)
        if height / _width(pts) <= STANCE_MAX:
            return None
        far = -sgn  # the slab's end away from the wall
        for reach in np.arange(4.0, 80.0, 2.0):
            ft = [c + back * n + far * half_a * u, c + (back + reach) * n + far * half_a * u,
                  c + (back + reach) * n + far * (half_a - FOOT_W) * u, c + back * n + far * (half_a - FOOT_W) * u]
            if height / _width(np.vstack([pts, np.array(ft)])) <= STANCE_MAX:
                return [tuple(v) for v in ft]
        raise ValueError("no foot gives part 1 a wide enough stance")

    def foot_b(faces, height):
        pts = np.array([v for f in faces for v in f], float)
        if height / _width(pts) <= STANCE_MAX:
            return None
        u_in = half_b - FOOT_W
        if u_in < ib.face_w / 2 + 1.0:
            raise ValueError("no room for part 2's foot beside B's block")
        for reach in np.arange(4.0, 60.0, 2.0):
            ft = [cb + S.SET * nb + sb * u_in * ub, cb + S.SET * nb + sb * half_b * ub,
                  cb - reach * nb + sb * half_b * ub, cb - reach * nb + sb * u_in * ub]
            if height / _width(np.vstack([pts, np.array(ft)])) <= STANCE_MAX:
                return [tuple(v) for v in ft]
        raise ValueError("no foot gives part 2 a wide enough stance")

    feet = [None, None] if var.get("no_feet") else [foot(p1_faces, ca, na, ua, side, backs["A"], z_top - z_bed),
                                                    foot_b(p2_faces, top2 - z_bed)]
    part1 = body(p1_faces, z_bed, z_top)
    part2 = body(p2_faces, z_bed, top2)
    if feet[0] is not None:
        part1 = (part1 + _prism(feet[0], z_bed, z_bed + FOOT_H)).clean()
    if feet[1] is not None:
        part2 = (part2 + _prism(feet[1], z_bed, z_bed + FOOT_H)).clean()
    # the pads and the fixed parts' holes: as at level 3
    ra = ia.face_w / 2 - S.INSET
    grow = S.DRAFT / math.cos(math.pi / S.POLY)
    part1 = part1 + S._pad(S._flange_outline(ra), S._flange_outline(ra + grow), tuple(ca) + (0.0,), tuple(ua3),
                           (0.0, 0.0, 1.0), tuple(na3))
    wb, hb = ib.face_w - 2 * S.INSET, ib.face_h - 2 * S.INSET
    part2 = part2 + S._pad(S._rect_outline(wb, hb), S._rect_outline(wb + 2 * S.DRAFT, hb + 2 * S.DRAFT), tuple(cb3),
                           tuple(ub3), (0.0, 0.0, 1.0), tuple(nb3))
    for name, itf, n in (("A", ia, na3), ("B", ib, nb3)):
        bb = BOLTS[itf.bolt]
        back, extra = backs[name], extras[name]
        pl = lay["placements"][name]
        for x, y in itf.holes:
            p = pl.point((x, y, 0.0))
            hole = bd.Plane(origin=p, z_dir=tuple(n)) * bd.Pos(0, 0, (back + 2.0) / 2 - 1.0) \
                * bd.Cylinder(bb.clear_medium / 2, back + 2.0)
            if extra > 0:  # the head's pocket: a teardrop (roof up) from the seat through the slab's back
                rp = bb.head_d / 2 + POCKET_ROOM
                q = rp / math.sqrt(2)
                drop = bd.Circle(rp) + bd.Polygon((-q, q), (q, q), (0.0, rp * math.sqrt(2)), align=None)
                zc = float(p[2])
                if zc + rp * math.sqrt(2) > z_top - 2.0:  # its roof all but at the top: open through it
                    drop = drop + bd.Pos(0, (z_top + 5.0 - zc) / 2) * bd.Rectangle(2 * rp, z_top + 5.0 - zc)
                if zc - rp < z_bed + 2.0:  # its floor all but at the bed: open through that
                    drop = drop + bd.Pos(0, -(zc - z_bed + 5.0) / 2) * bd.Rectangle(2 * rp, zc - z_bed + 5.0)
                u = np.array(S._side(n))
                pl_ = bd.Plane(origin=tuple(np.asarray(p) + np.asarray(n) * seat), x_dir=tuple(u), z_dir=tuple(n))
                hole = hole + bd.extrude(pl_ * drop, extra + 1.0, dir=tuple(n))
            if name == "A":
                part1 = part1 - hole
            else:
                part2 = part2 - hole
    for p in ia.protrusions:  # the boss: a teardrop hole, its roof up
        r = p.radius + S.SPIGOT_ROOM
        q = r / math.sqrt(2)
        drop = bd.Circle(r) + bd.Polygon((-q, q), (q, q), (0.0, r * math.sqrt(2)), align=None)
        pl = bd.Plane(origin=tuple(-1.0 * na3), x_dir=tuple(ua3), z_dir=tuple(na3))
        part1 = part1 - bd.extrude(pl * drop, backs["A"] + 2.0, dir=tuple(na3))
    # the joint bolts: on the joint face (w = tj), in the joint's middle along the wall, one low and one high
    s_mid = s1 + GAP + lj / 2
    zs = (z_bed + ring + 2.0, z_top - ring - 2.0)
    pos = [np.r_[at(s_mid, tj), z] for z in zs]
    w3 = np.r_[w, 0.0]
    for p in (pos if butt is None else []):
        cut = bd.Plane(origin=tuple(p - w3 * (tj + 1.0)), z_dir=tuple(w3)) * bd.Pos(0, 0, tj + 1.0) \
            * bd.Cylinder(b.clear_medium / 2, 2 * tj + 2.0)
        part1, part2 = part1 - cut, part2 - cut
    ctx = {"qa": qa, "sh": sh, "w": w, "T": T, "s1": s1, "x_ends": x_ends, "L": L, "z": (z_bed, z_top), "na": na3, "nb": nb3,
           "ua": ua3, "ub": ub3, "cb": cb3}
    if var.get("extra") is not None:  # level 5's lock, cut and added in the arrangement's own frame
        part1, part2 = var["extra"](part1, part2, ctx)
    s1_, s2_ = part1.solids(), part2.solids()
    if len(s1_) != 1 or len(s2_) != 1:
        raise ValueError(f"a part falls apart ({len(s1_)} and {len(s2_)} pieces)")
    part1, part2 = s1_[0], s2_[0]

    # ---- ways in (arrangement frame): part 1 along A's face from the wall's side, then onto A; part 2
    # along B's face from that side, then onto B within the joint plane
    spig = max([p.height for p in ia.protrusions] or [0.0])
    onto_a = ONTO + spig
    d1 = -side * ua3
    d2 = (ub3 if float(np.r_[ub, 0.0] @ w3) < 0 else -ub3)
    dB = -(nb3 - float(nb3 @ w3) * w3)
    dB = dB / np.linalg.norm(dB)
    move1 = -na3 * onto_a
    move2 = dB * ONTO
    G = S.frame(d)

    def W(v):  # a direction to the world
        return np.array(G.vec(tuple(float(x) for x in v)))

    def P(v):
        return np.array(G.point(tuple(float(x) for x in v)))

    loc = G.location()
    part1, part2 = part1.moved(loc), part2.moved(loc)
    asm = {"parts": [
        {"name": "part1", "file": "part1.step", "bolts_to": "A", "insert_direction": _r(W(d1)),
         "path": [{"move": _r(W(move1))}]},
        {"name": "part2", "file": "part2.step", "bolts_to": "B", "insert_direction": _r(W(d2)),
         "path": [{"move": _r(W(move2))}]}],
        "joint": {"bolt": size, "length": float(length), "head_on": "part2", "axis": _r(W(w3)),
                  "positions": [_r(P(p)) for p in pos]}}
    dims = {"ctx": ctx, "frame": G, "tj": tj, "lj": lj, "L": L, "side": side, "size": size, "length": length, "d": d,
            "w": W(w3), "dirs": (W(d1), W(d2)), "moves": (W(move1), W(move2)), "z": (z_bed, z_top),
            "pos": [P(p) for p in pos], "ring": ring}
    return part1, part2, asm, dims


def S_z(lay: dict) -> tuple[float, float]:
    """Level 3's ring heights: from under the lower pad to over the higher."""
    ia, ib = lay["itf"]["A"], lay["itf"]["B"]
    ra = ia.face_w / 2 - S.INSET
    grow = S.DRAFT / math.cos(math.pi / S.POLY)
    cb = lay["placements"]["B"].origin
    hb = ib.face_h - 2 * S.INSET
    return min(-(ra + grow), cb[2] - hb / 2 - S.DRAFT), max(ra + grow, cb[2] + hb / 2 + S.DRAFT)


# --------------------------------------------------------------------------
# the box


def box(task: Task, part1, part2, dims: dict, *, starts=None, extra=()) -> tuple[list, dict]:
    """The closed box round everything (its walls as obstacles) and its one opening: in the front wall,
    square to the parts' two ways in, drawn round where each part crosses that wall on its way and round
    the joint keys' line of sight. ``starts``: each part where its path begins (default: its place with
    its path undone); ``extra``: more poses inside the box it must hold."""
    d1, d2 = dims["dirs"]
    m1, m2 = dims["moves"]
    up = np.array([0.0, 0.0, 1.0])
    e1 = _u(-(d1 + d2))  # the front wall's outward normal: between the two ways in
    e1 = _u(e1 - up * float(e1 @ up))
    s = np.cross(up, e1)
    if starts is None:
        starts = (part1.moved(bd.Location(tuple(-m1))), part2.moved(bd.Location(tuple(-m2))))
    fixed = task.fixed_solids(0)
    stuff = np.concatenate([_points(x) for x in (part1, part2, *starts, *extra, *fixed.values())])
    dirs = {"t+": up, "t-": -up, "e1+": e1, "e1-": -e1, "s+": s, "s-": -s}
    off = {k: float((stuff @ n).max()) + MARGIN for k, n in dirs.items()}
    axes = {"e1": e1, "s": s, "t": up}
    span = {a: (-off[a + "-"], off[a + "+"]) for a in axes}
    walls = []
    for k, n in dirs.items():
        if k == "e1+":
            continue
        a = k[:-1]
        u_, v_ = [o for o in ("e1", "s", "t") if o != a]
        if float(np.cross(axes[u_], axes[v_]) @ n) < 0:
            u_, v_ = v_, u_
        cen = n * (off[k] + WALL / 2) + sum(axes[o] * (span[o][0] + span[o][1]) / 2 for o in (u_, v_))
        size = tuple(float(span[o][1] - span[o][0] + 2 * WALL) for o in (u_, v_)) + (WALL,)
        walls.append(Obstacle({"t+": "wall_top", "t-": "wall_bottom", "e1-": "wall_back", "s+": "wall_side_1",
                               "s-": "wall_side_2"}[k], size,
                              Placement(tuple(float(x) for x in cen), _rot(axes[u_], axes[v_], n))))
    # the opening: where each part crosses the front wall (both its faces) along its own way in, and the
    # joint keys: each head's disc carried out along the joint axis
    cross = []
    for part, dd in ((starts[0], d1), (starts[1], d2)):
        P_ = _points(part)
        for plane in (off["e1+"], off["e1+"] + WALL):
            t_ = (plane - P_ @ e1) / float(dd @ e1)
            cross.append(P_ + np.outer(t_, dd))
    ax = np.asarray(dims["w"], float)
    b = BOLTS[dims["size"]]
    ex, ey = np.cross(ax, up), up
    ex = ex / np.linalg.norm(ex)
    for p in dims["pos"]:
        disc = np.array([p + b.head_d / 2 * (math.cos(a) * ex + math.sin(a) * ey) for a in np.linspace(0, 2 * math.pi, 24)])
        for plane in (off["e1+"], off["e1+"] + WALL):
            t_ = (plane - disc @ e1) / float(ax @ e1)
            cross.append(disc + np.outer(t_, ax))
    cross = np.concatenate(cross)
    oa_ = (float((cross @ s).min()) - CLEAR, float((cross @ s).max()) + CLEAR)
    ot = (float((cross @ up).min()) - CLEAR, float((cross @ up).max()) + CLEAR)
    S0, S1 = span["s"][0] - WALL, span["s"][1] + WALL
    T0, T1 = span["t"][0] - WALL, span["t"][1] + WALL
    if not (S0 < oa_[0] and oa_[1] < S1 and T0 < ot[0] and ot[1] < T1):
        raise ValueError("the opening would be as big as the front wall")
    rot = _rot(s, up, e1)

    def piece(name, a0, a1, z0, z1):
        cen = s * ((a0 + a1) / 2) + up * ((z0 + z1) / 2) + e1 * (off["e1+"] + WALL / 2)
        return Obstacle(name, (float(a1 - a0), float(z1 - z0), WALL), Placement(tuple(float(x) for x in cen), rot))

    walls += [piece("wall_front_below", S0, S1, T0, ot[0]), piece("wall_front_above", S0, S1, ot[1], T1),
              piece("wall_front_left", S0, oa_[0], ot[0], ot[1]), piece("wall_front_right", oa_[1], S1, ot[0], ot[1])]
    opening = {"wall": "wall_front", "normal": _r(e1), "plane": round(off["e1+"], 6), "along": _r(s), "up": _r(up),
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


def make(task3: Task, d: dict, **kw):
    """(level 4 task, (part 1, part 2, assembly), dimensions) for a level 3 flange task and its draw."""
    p1, p2, asm, dims = build(task3, d, **kw)
    walls, opening = box(task3, p1, p2, dims)
    return task4(task3, walls, opening), (p1, p2, asm), dims


# --------------------------------------------------------------------------
# broken copies


NARROW_KG = 6.0  # the narrow-joint copy is tested at this load: the reference holds it (64%), the copy does not (160%)


def broken(task3: Task, dims: dict, part1, part2, asm: dict) -> dict:
    """name -> (part1, part2, assembly dict, the check that must fail, load in kg or None for the task's own)."""
    swap = json.loads(json.dumps(asm))
    swap["parts"] = swap["parts"][::-1]
    out = {"wrong-order": (part1, part2, swap, "INSERTION", None)}
    heads = json.loads(json.dumps(asm))
    heads["joint"]["head_on"], heads["joint"]["axis"] = "part1", _r(-np.asarray(dims["w"]))
    out["key-cannot-reach"] = (part1, part2, heads, "JOINT_REACH", None)
    short = json.loads(json.dumps(asm))
    short["joint"]["length"] = float(max(x for x in lengths(dims["size"]) if x < dims["length"]))
    out["bolt-too-short"] = (part1, part2, short, "JOINT_LENGTH", None)
    kw = dict(size=dims["size"], length=dims["length"], side=dims["side"])
    p1, p2, a, _ = build(task3, dims["d"], variant={"taller": 20.0}, **kw)
    out["too-big-for-the-opening"] = (p1, p2, a, "INSERTION", None)  # part 2 20 mm taller
    p1, p2, a, _ = build(task3, dims["d"], variant={"narrow": True}, **kw)
    out["narrow-joint"] = (p1, p2, a, "JOINT_BOLTS", NARROW_KG)  # the nuts pull through: too little face
    return out


# --------------------------------------------------------------------------
# the task folder


VIEW_NOTE = ("A ship in a bottle round a flange and a block. Grey: the two fixed parts, the round flange A and "
             "the tall block B. Red, see-through: the closed box, its one opening in the front wall. Green: part 1, "
             "which bolts to the flange and goes in first; teal: part 2, which bolts to the block. They meet in the "
             "side wall, bolted through it from outside. Slide to take them out again the way they went in, last in "
             "first out; the lines are each part's way in (its middle point). Drag to turn, scroll to zoom.")


def write_folder(task: Task, ref, dims: dict, out_dir: str, *, task3: Task, view: bool = True) -> str:
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
        json.dump({"level": LEVEL, "test": "assembles", "situation": "flange", "seed": task.seed,
                   "draw": {k: v for k, v in dims["d"].items() if k != "timing_s"},
                   "joint": {"bolt": dims["size"], "length": dims["length"], "layers": round(dims["tj"], 3)}},
                  f, indent=2)
        f.write("\n")
    for name, (q1, q2, a, must, kg) in broken(task3, dims, *ref).items():
        bdir = os.path.join(sol, "broken", name)
        write_answer(bdir, q1, q2, a)
        with open(os.path.join(bdir, "MUST_FAIL"), "w", encoding="utf-8") as f:
            f.write(must + "\n")
        if kg is not None:
            with open(os.path.join(bdir, "LOAD_KG"), "w", encoding="utf-8") as f:
                f.write(f"{kg:g}\n")
    if view:
        from ...assemble.view import write as write_view

        e1 = np.asarray(task.meta["opening"]["normal"], float)
        write_view(task, sol, os.path.join(sol, "view"), heading=f"Flange and block, level 4 (assembles): {task.id}",
                   note=VIEW_NOTE, view_dir=_u(e1 + np.array([0.0, 0.0, 0.5])))
    return folder


def main(argv=None) -> int:
    """python -m jointbench.families.flange.assembles OUT_DIR [SEED]: the flange setup's level 4 task folder."""
    import sys

    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) not in (1, 2):
        print(main.__doc__)
        return 2
    from .level4 import generate

    t4, draw, ref = generate(int(args[1]) if len(args) > 1 else 0)
    print(write_folder(t4, ref, draw["dims"], args[0], task3=draw["task3"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
