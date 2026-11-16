"""Level 6 on the flange setup: what the setup supplies to the shared generator.

The fixed parts are the flange setup's own (``families/flange/shell``, the
same draw as its levels 1-3 for the same seed): the round flange A stays
fixed, the tall block B is carried. B's end pose is its pose from levels 1-3:
its home.

Why A is the fixed one: the frame must print standing on a flat bed with
world up as its print direction, its pivot posts rising straight up. Bolted to
the flange it is the levels 1-3 slab standing on the flange's upright face,
and the mechanism's plane (through B's centre, along B's face normal) runs
straight into that slab. Bolted to the block, the frame would have to reach
some 38 mm sideways to that plane and down past the block's lower bolts,
80 mm below its upper ones.

The frame (``base`` + ``finish``):

* the slab behind the flange, stood back 1.5 mm from its mounting plane, the
  seat a drafted pad inside the face's outline, the boss through a teardrop
  hole, plain clearance holes for the bolts; it runs down past the flange to
  the print bed (lower where the flange sits low, ``bed_t``) and is wide
  enough that the mechanism's layers, which cross it at 20 degrees, find it
  under them;
* the head: an upright plate in each of the four clevis-ear layers, from the
  bed to above the flange's top bolts, reaching HEAD_OUT towards B, its front
  square to the mechanism's plane. The ground pivots' posts stand on the
  plates' tops (flush with their front, or POST_SET back: a shorter run-on is
  a stub); each link swings in the gap between its post's two plates, the
  latch's lug arm between the inner two. Nothing overhangs, so nothing needs
  a chamfer (a chamfer square to the flange's axis, cut by slots square to
  the mechanism, leaves sharp 76 degree edges that print as thin walls). The
  flange's bolts' key ways (head and a 30 mm straight tool, ``keep_out``) run
  through the plates as tunnels; the hex key's long leg swings beyond the
  plates' ends (31-36 mm out from the slab);
* two side feet on the bed, just outside the outer layers, parallel to the
  mechanism's plane: the tall frame then stands on an outline at least a
  third of its height wide (the printing rule's stance);
* the latch's foot: a low block on the slab's back at the bed, flush with the
  inner plates' outer faces; the ledge's post rises from it, between the
  flange's bolt heads (low, or high on a short block, ``_latch_band``).

Seed 0's flange has a 25 mm boss inside six bolts on a 50 mm circle: a part
bolted to it cannot come straight down onto it (no slot under the boss would
spare the lower bolts' seats). The frame therefore comes in along a path
(``insert_path``): straight down held 10 degrees open about a vertical hinge
line at its outer edge on the flange's face plane, then swung shut, so its
last millimetres run along the flange's axis. A flange without a boss takes
the frame straight down. Every other part comes straight down.

The linkage is drawn by this setup (``draw``) rather than by the shared draw,
which fixes B's start first and finds each ground pivot on a bisector: here
those run into the key ways or past the head. This draw goes forwards: the
carrier's pivots in front of B, short of the plates' front (the carrier comes
straight down past them) and of the flange's bolt heads, at heights clear of
B's bolt heads and of the latch's arm (the nearer one higher); the ground
pivots on the plates, at least a screw's line apart (each axle's screw and nut
come along its line through the other stack); a crank turn of 35-90 degrees
whose start leaves B at least 8 mm above home. The start is where the linkage
then puts B. The layers are fixed (the plates are built for them). Everything
after the draw -- the 2D screens, the reference, the box, the grading -- is the
shared generator's.

The block is narrower (36 mm) than the mechanism's three layers (about 36 mm
with their running gaps), so the carrier's outer clevis ears stand up to about
2 mm past the plate's edge (``layers_half``); they are joined to the plate
along most of their width and do not touch B's face.
"""
from __future__ import annotations

import functools
import json
import math

import build123d as bd
import numpy as np

from ...catalogue import BOLTS, fixed_part_solid
from .. import assembly as AS
from ...families.flange import shell as SH
from ...families.pipe.bracket import _halfspace, _unit

NAME = "flange"
BED_T = -10.0  # mm, the print bed below B's home centre at most: lower where the flange sits lower (``bed_t``), so the
# slab runs down past the flange's pad
DOOR_OPEN = 10.0  # deg the frame is held open on its way down (flange with a boss; see ``insert_path``)
DOOR_DROP = 500.0  # mm of its straight way down (it starts above the box)
LATCH_SPAN = 2.5  # mm, the latch lug's height range (the lowest room clear of the flange's bolt heads)
SIDE_FOOT = 6.0  # mm, each side foot's width (see ``base``)
FOOT_ROOM = 31.5  # mm between the mechanism's outer layers and a side foot: beyond every axle's line as the screens
# check it (30 mm out) and the carrier's low clevis ears
POST_ABOVE = 34.0  # mm a ground post's top stands above the head's top at most (R + 8 + O_RISE + R, rounded up)
FOOT_LEN, FOOT_H = 18.0, 2.0  # mm, the latch's foot on the slab's back, at the bed (the ledge's post rises from it)
HEAD_OUT = 22.0  # mm the slab's head reaches out towards B over the flange's top bolts (a hex key's long leg swings
# beyond it: socket 3 mm, short leg 28 mm, so the long leg lies 31-36 mm out from the head's top)
HEAD_CLEAR = 1.0  # mm between the top bolts' key ways and the head's chamfered underside
HEAD_TOP = 10.0  # mm of upright front above the chamfer (a post is set 6 mm into the plate's top)
KEY_WAY = 30.0 + 10.0  # mm a bolt's key way runs out from the slab: the head and a 30 mm straight tool, with room
O_RISE = 11.0
POST_SET = 5.5  # mm a post's front face stands back from the head's front at least (Q_STUB's 5 mm, with room)
DRIVE = (35, 90)  # deg, the crank's turn from the start to home (the shared least is 35)
START_RISE = 8.0  # mm B's start lies at least this far above its home (it then comes down onto the latch)
SLAB_ROOM = 2.5  # mm the slab reaches beyond the pad's base outline


@functools.lru_cache(maxsize=None)
def _levels(seed: int):
    """The flange setup's draw for ``seed`` exactly as its levels 1-3 have it (their generator, graded)."""
    task, d, _ = SH.generate(seed, 1)
    return task, d


def _seg_dist(p, a, b) -> float:
    """The distance from point p to the segment a-b (in the plane)."""
    p, a, b = (np.asarray(x, float) for x in (p, a, b))
    ab = b - a
    k = float(np.clip((p - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + k * ab)))


class Setup:
    """The flange setup (seed 0: the public showcase draw)."""

    name = NAME
    fixed = "A"
    carried = "B"
    posts_clear_keep_out = True  # the generator draws the pivot and latch posts clear of ``keep_out``
    post_lean = 0.0  # the posts stand upright on the plates' flat tops (leaning sides would overhang their fronts)
    post_webs = False  # no web between a post's ears: the gap there is the link's slot between the plates

    def __init__(self, seed: int = 0):
        self.seed = seed
        task, d = _levels(seed)
        self.d = d
        self.lay = SH.layout(d)
        self.g = SH.frame(d)  # the arrangement frame in the world
        self.itf_a, self.itf_b = self.lay["itf"]["A"], self.lay["itf"]["B"]
        self.home = {n: task.poses[0][n] for n in "AB"}
        self.load_kg = float(d["load_kg"])
        # the carrier's layers may reach a few mm past the block's plate (see the module's note)
        self.layers_half = self.itf_b.face_w / 2 + 3.0

    def parts(self) -> dict:
        return {"A": self.itf_a, "B": self.itf_b}

    def frame(self) -> dict:
        """origin (B's home centre), s (B's face normal, level: from B towards the flange), t (world up),
        axis (every pivot's direction: along B's face, level)."""
        O = np.array(self.home["B"].origin, float)
        t = np.array([0.0, 0.0, 1.0])
        s = np.array(self.home["B"].vec((0, 0, 1)), float)
        s = _unit(s - t * float(s @ t))
        return {"origin": O, "s": s, "t": t, "axis": _unit(np.cross(s, t))}

    # ---- heights
    def heights(self, P) -> tuple[float, float]:
        """(print bed, top) in world z: the bed ``bed_t`` below B's home centre, the top the head's."""
        return float(P.O[2]) + self.bed_t(P), float(P.O[2]) + self.head_top_t(P)

    def bed_t(self, P) -> float:
        """The print bed, as t: BED_T, or lower so the slab reaches 1 mm below the flange's pad."""
        ra, grow = self._r_pad()
        pad_lo = float(P.st(self.home["A"].origin)[1]) - (ra + grow)
        return min(BED_T, pad_lo - 1.0)

    # ---- the frame's base on the flange, built in the arrangement frame (A's face x = 0, normal +x, z up)
    def _r_pad(self):
        ra = self.itf_a.face_w / 2 - SH.INSET
        grow = SH.DRAFT / np.cos(np.pi / SH.POLY)
        return ra, grow

    def head_top_t(self, P) -> float:
        """The head's flat top, as t in the mechanism's plane."""
        z = self._z_head_foot() + HEAD_OUT + HEAD_TOP + float(self.g.origin[2])
        return z - float(P.O[2])

    def _z_head_foot(self) -> float:
        """Where the head's chamfer leaves the slab's back (arrangement z): just over the top bolts' key ways."""
        pb = BOLTS[self.itf_a.bolt]
        z_bolt = max(float(self.lay["placements"]["A"].point((x, y, 0.0))[2]) for x, y in self.itf_a.holes)
        return max(z_bolt + pb.head_d / 2 + 1.7 + HEAD_CLEAR, self._r_slab() - HEAD_OUT)

    def _slab_y(self) -> tuple[float, float]:
        """The slab's (and head's) extent across the flange (arrangement y): the flange's outline with room, and
        wide enough that the mechanism's layers, which cross it at an angle, find the head under them."""
        from . import generate as G

        rs = self._r_slab()
        fr = self.frame()
        O, s_, t_, a_ = (np.asarray(fr[k], float) for k in ("origin", "s", "t", "axis"))
        L = self.layers()
        R = np.array(self.g.rot, float)
        P_ = G.Plane(fr)
        # where the side feet and the head's plates run along s (the feet end nearest B; the slab's front farthest)
        s_span = (min(self._feet_s_end(P_), 25.0) - 3.0,
                  -self._s_plane(fr, 0.0, [(L["lo"] - 40.0, L["hi"] + 40.0)], SH.SET, most=True) + 3.0)
        ys = [float((R.T @ (O + s * s_ + w * a_ - np.asarray(self.g.origin, float)))[1])
              for s in s_span for w in (L["lo"] - max(G.rk(), FOOT_ROOM + SIDE_FOOT + 2.0),
                                         L["hi"] + max(G.rk(), FOOT_ROOM + SIDE_FOOT + 2.0))]
        return min(-rs, min(ys)), max(rs, max(ys))

    def _r_slab(self) -> float:
        ra, grow = self._r_pad()
        return ra + grow + SLAB_ROOM

    def base(self, z_bed: float, z_top: float, **_) -> bd.Part:  # (newer shared options, e.g. under=, not used)
        """The slab behind the flange (no pad, no holes yet: ``finish`` adds them), the side feet, the latch's foot
        and the head's plates, in the world."""
        g = self.g
        zb = float(z_bed - g.origin[2])  # the arrangement frame only turns about the vertical
        rs = self._r_slab()
        x0, x1 = SH.SET, SH.SET + SH.SLAB
        # the slab from the bed to the head's top
        z_hi = self._z_head_foot() + HEAD_OUT + HEAD_TOP
        prof = [(x0, zb), (x1, zb), (x1, z_hi), (x0, z_hi)]
        from . import generate as G

        y0, y1 = self._slab_y()
        pl_xz = bd.Plane(origin=(0.0, y0, 0.0), x_dir=(1.0, 0.0, 0.0), z_dir=(0.0, 1.0, 0.0))  # local y = -z
        slab = bd.extrude(pl_xz * bd.Polygon(*[(x, -z) for x, z in prof], align=None), y1 - y0, dir=(0.0, 1.0, 0.0))
        body = slab.moved(g.location())
        # two feet on the print bed, at the slab's two ends, out from it towards B, FOOT_H thick: the tall frame then
        # stands on a stance (the outline round all it stands on) at least a third of its height, the printing
        # rule; the space under the mechanism stays free
        reach = self._reach(G.Plane(self.frame()))
        frp = self.frame()
        Pp = G.Plane(frp)
        Lp = self.layers()
        n_a = np.asarray(self.home["A"].vec((0, 0, 1)), float)
        c_a = np.asarray(self.home["A"].origin, float)
        for w0f, w1f in self._feet_w():
            s_far = self._s_plane(frp, self.bed_t(Pp), [(w0f, w1f)], SH.SET)  # where it meets the slab's front
            s_end = self._feet_s_end(Pp)
            ft = bd.Plane(origin=tuple(Pp.world(((s_end + s_far + 40.0) / 2, self.bed_t(Pp) + FOOT_H - 5.0), (w0f + w1f) / 2)),
                          x_dir=tuple(Pp.s), z_dir=tuple(Pp.a)) * bd.Box(s_far + 40.0 - s_end, 10.0, w1f - w0f)
            ft = ft - _halfspace(c_a + n_a * SH.SET, n_a)  # back to the slab's front
            body = body + ft
        # the latch's foot: a low block on the slab's back in the latch's layer, from the bed up FOOT_H (the
        # ledge's post rises from it), cut back to the slab's front (x = SET)
        fr = self.frame()
        O, s, t, a = (np.asarray(fr[k], float) for k in ("origin", "s", "t", "axis"))
        L = self.layers()
        from . import generate as G

        bed = self.bed_t(G.Plane(fr))
        s_back = self._s_plane(fr, bed, [(L["latch"] - G.LUG_T / 2, L["latch"] + G.LUG_T / 2)], SH.SET + SH.SLAB)
        c = O + s * (s_back - FOOT_LEN + 30.0) + t * (bed + FOOT_H - 10.0) + a * L["latch"]
        # as wide as the latch's layer and the two inner ear layers beside it: flush with the inner plates' outer
        # faces, so no face of it ends partway across a plate
        SLf = G.slabs(L)
        fw0, fw1 = SLf["crank_ear_hi"][0] if L["crank"] < L["latch"] else SLf["rocker_ear_lo"][0], \
            SLf["rocker_ear_lo"][1] if L["rocker"] > L["latch"] else SLf["crank_ear_hi"][1]
        c = c + a * ((fw0 + fw1) / 2 - L["latch"])
        foot = bd.Plane(origin=tuple(c), x_dir=tuple(s), z_dir=tuple(a)) * bd.Box(60.0, 20.0, fw1 - fw0)
        n_a = np.asarray(self.home["A"].vec((0, 0, 1)), float)  # the flange's face normal, towards B
        c_a = np.asarray(self.home["A"].origin, float)
        foot = foot - _halfspace(c_a + n_a * SH.SET, n_a)  # nothing nearer the flange than the slab's front
        body = body + foot
        # the head: a plate in each of the posts' ear layers, out from the slab's back to HEAD_OUT (its front square
        # to the mechanism's plane), its underside rising at 45 degrees in its own plane from the slab's back just
        # over the flange's top bolts' key ways. The posts stand on the plates' tops (flush with their front, or
        # POST_SET back); each link swings in the gap between its post's two plates, the latch's lug arm between
        # the inner two; a hex key's long leg swings beyond the plates' ends. The underside meets the plates' faces
        # square (a chamfer square to the flange's axis, cut by gaps square to the mechanism, would leave sharp
        # 76 degree edges that print as thin walls)
        for plate in self._plates():
            body = body + plate
        return body

    def carrier_plate(self, t_b: float, L: dict):
        """The carrier's plate, in B's own axes (its face z = 0, x across it, y up): its seat 1 mm inside B's face
        across, then widening at 45 degrees to the mechanism's outer layers (the clevises' outer ears stand there,
        past B's 36 mm: a plate as narrow as the seat would leave their ends as stubs)."""
        seat = self.itf_b.face_w / 2 - 1.0
        wide = max(seat, abs(L["lo"]), abs(L["hi"]))
        rise = wide - seat
        h = self.itf_b.face_h - 3.0
        prof = [(-seat, 0.0), (seat, 0.0), (wide, rise), (wide, t_b), (-wide, t_b), (-wide, rise)]
        pl = bd.Plane(origin=(0.0, -h / 2, 0.0), x_dir=(1.0, 0.0, 0.0), z_dir=(0.0, 1.0, 0.0))  # local y = -z
        return bd.extrude(pl * bd.Polygon(*[(x, -z) for x, z in prof], align=None), h, dir=(0.0, 1.0, 0.0))

    def _plates(self) -> list:
        """The head's four plates, in the world (see ``base``)."""
        from . import generate as G

        fr = self.frame()
        O, s, t, a = (np.asarray(fr[k], float) for k in ("origin", "s", "t", "axis"))
        n_a = np.asarray(self.home["A"].vec((0, 0, 1)), float)
        c_a = np.asarray(self.home["A"].origin, float)
        Pf = G.Plane(fr)
        t_head = self.head_top_t(Pf)
        t_ch = self._z_head_foot() + float(self.g.origin[2]) - float(O[2])  # the chamfer's foot at the slab's back
        SLm = G.slabs(self.layers())
        out = []
        for k in ("crank_ear_lo", "crank_ear_hi", "rocker_ear_lo", "rocker_ear_hi"):
            w0, w1 = SLm[k]
            # both plates of a post share one front (a post flush with one would leave a stub of the other)
            stk = k.split("_")[0]
            s_front = -self._s_plane(fr, t_head, [SLm[stk + "_ear_lo"], SLm[stk + "_ear_hi"]], SH.SET + SH.SLAB + HEAD_OUT,
                                     most=True)
            s_ref = -self._s_plane(fr, t_head, [(w0, w1)], SH.SET + SH.SLAB, most=True)  # the slab's back, deepest
            s_in = self._s_plane(fr, t_head, [(w0, w1)], SH.SET) + 10.0  # well into the slab (cut back below)
            prof = [(s_front, t_head), (s_front, t_ch + (s_ref - s_front)), (s_ref, t_ch), (s_in, t_ch),
                    (s_in, t_head)]
            pl = bd.Plane(origin=tuple(O + a * w0), x_dir=tuple(s), z_dir=tuple(a))
            plate = bd.extrude(pl * bd.Polygon(*prof, align=None), w1 - w0, dir=tuple(a))
            out.append(plate - _halfspace(c_a + n_a * SH.SET, n_a))  # nothing nearer the flange than the slab's front
        return out

    def post_keep_out(self, **_) -> list:  # (newer shared options, e.g. open_along=, not used)
        """Cut from each post (the shared build sets them 25 mm down into the base): between the plates' underside
        and the head's top, everything but the plates, so a post merges into its two plates and leaves no faces of
        its own there."""
        from . import generate as G

        fr = self.frame()
        O, t_ = np.asarray(fr["origin"], float), np.asarray(fr["t"], float)
        P = G.Plane(fr)
        t_head = self.head_top_t(P)
        t_ch = self._z_head_foot() + float(self.g.origin[2]) - float(O[2])
        band = _halfspace(O + t_ * t_head, t_, 2000.0) - _halfspace(O + t_ * (t_ch - 1.0), t_, 2000.0)
        for plate in self._plates():
            band = band - plate
        return [band]

    def insert_path(self, name: str, solid) -> list | None:
        """The frame's way in when the flange has a boss: straight down held DOOR_OPEN degrees open about a
        vertical hinge line at the frame's outer edge on the flange's face plane, then swung shut, so its last
        millimetres run along the flange's axis onto the boss. Every other part (and a frame without a boss to
        clear) comes straight down."""
        if name != "frame" or not self.itf_a.protrusions:
            return None
        R = np.array(self.g.rot, float)
        o = np.asarray(self.g.origin, float)
        y_h = max(float(((np.array(tuple(v)) - o) @ R)[1]) for v in solid.vertices())
        hinge = o + R @ np.array([0.0, y_h, 0.0])
        # open far enough that the boss clears the frame on the way down (with a degree to spare)
        need = max(math.degrees(math.asin(min(1.0, (pr.height + SH.SPIGOT_ROOM + 1.0) / max(1.0, y_h - pr.radius))))
                   for pr in self.itf_a.protrusions)
        deg = max(DOOR_OPEN, need + 2.0)
        return [{"move": [0.0, 0.0, -DOOR_DROP]},
                {"turn": {"point": [float(v) for v in hinge], "axis": [0.0, 0.0, 1.0], "deg": -float(deg)}}]

    def layers(self) -> dict:
        """The mechanism's layers (fixed for this setup: the head is slotted for the links)."""
        from . import generate as G

        return G.layers(0.0, True)

    def keep_out(self) -> list:
        """Bodies the frame must not grow into, in the world: each flange bolt's key way, from the slab's back
        face straight out along the bolt."""
        pb = BOLTS[self.itf_a.bolt]
        pa = self.home["A"]
        n = np.asarray(pa.vec((0, 0, 1)), float)
        out = []
        for x, y in self.itf_a.holes:
            P = np.asarray(pa.point((x, y, 0.0)), float) + n * (SH.SET + SH.SLAB)
            out.append(bd.Plane(origin=tuple(P), z_dir=tuple(n)) * bd.Pos(0, 0, KEY_WAY / 2)
                       * bd.Cylinder(pb.head_d / 2 + 1.7, KEY_WAY))
        return out

    def finish(self, body: bd.Part, z_bed: float) -> bd.Part:
        """Stand the frame back from the flange's mounting plane, add the seat pad, drill the bolts' holes and
        the boss's teardrop, keep the key ways free and cut it flat at the print bed."""
        pa = self.home["A"]
        n = np.asarray(pa.vec((0, 0, 1)), float)
        c = np.asarray(pa.origin, float)
        body = body - _halfspace(c + n * SH.SET, n)  # no frame material on or near the mounting plane
        g = self.g
        na, ua, up = self.lay["normals"]["A"], SH._side(self.lay["normals"]["A"]), (0.0, 0.0, 1.0)
        ra, grow = self._r_pad()
        pad = SH._pad(SH._flange_outline(ra), SH._flange_outline(ra + grow), (0.0, 0.0, 0.0), ua, up, na)
        body = body + pad.moved(g.location())
        b = BOLTS[self.itf_a.bolt]
        L = SH.SET + SH.SLAB + 2.0
        for x, y in self.itf_a.holes:
            p = pa.point((x, y, 0.0))
            body = body - bd.Plane(origin=tuple(p), z_dir=tuple(n)) * bd.Pos(0, 0, L / 2 - 1.0) * bd.Cylinder(
                b.clear_medium / 2, L)
        for pr in self.itf_a.protrusions:
            r = pr.radius + SH.SPIGOT_ROOM
            q = r / np.sqrt(2)
            drop = bd.Circle(r) + bd.Polygon((-q, q), (q, q), (0.0, r * np.sqrt(2)), align=None)
            pl = bd.Plane(origin=(-1.0, 0.0, 0.0), x_dir=ua, z_dir=na)
            body = body - bd.extrude(pl * drop, L, dir=na).moved(g.location())
        body = body - _halfspace(np.array([0.0, 0.0, z_bed]), np.array([0.0, 0.0, 1.0]))  # all below the bed
        # the key ways through the plates: cut square to each plate (straight walls, no thin rims where a round
        # tunnel would enter it at 20 degrees), shaped as the hull of two teardrops (a flat top under 10 mm: prints
        # without support)
        for k in self.keep_out():
            body = body - k
        return body

    def draw(self, rng, P, scene) -> dict | None:
        """The hidden four-bar for this setup, in the shared draw's format (see ``generate.draw``), drawn the
        other way round: the ground pivots first, low on the shelf under the flange's bolts' key ways (the only
        room the frame's posts have here), the carrier's pivots in front of B, the crank's drive; the start pose
        is where the linkage then puts B. (The shared draw fixes the start first, above and behind home, and
        finds each ground pivot on a bisector: here those run up into the key ways.)"""
        from .. import linkage as LK
        from . import generate as G
        from . import geom2d as G2

        R_ = G.rk()
        bb = BOLTS[self.itf_b.bolt]
        t_b = max(5.0, bb.grip_min)
        half_b = min(self.itf_b.face_w, self.itf_b.face_h) / 2 - 1.5
        L = self.layers()  # fixed: the head is slotted for the links
        if L["hi"] > self.layers_half or L["lo"] < -self.layers_half:
            return G._no("layers wider than B")
        SL = G.slabs(L)
        # the carrier's pivots in front of B, apart by a knuckle and 6 mm (each screw's line misses the other
        # knuckle), their clevises short of the flange's bolt heads (the slab stands at an angle: nearer on one
        # side) and of B's own bolt heads
        hb = max(abs(y) for _, y in self.itf_b.holes) - bb.head_d / 2 - 1.0
        Cs = {}
        fr0 = {"origin": P.O, "s": P.s, "t": P.t, "axis": P.a}
        t_head0 = self.head_top_t(P) - HEAD_TOP  # the head's front face, at its top
        # their heights: clear of B's bolt heads, and above or below the latch's arm (no axle's line may cross it)
        nut_r = AS.nut_corner_d(G.AXLE) / 2
        # (a clevis arm a knuckle high, beside the latch arm in the next layer, would leave a 0.4 mm gap: too
        # narrow to print; and no axle's nut may cross the latch arm)
        room = max(nut_r, R_) + 1.5
        sep = R_ + nut_r + 1.0  # two pivots' heights apart, so neither screw's line crosses the other's clevis arm

        def c_bands(la):
            bs = [b for b in ((la[1] + G.LUG_H / 2 + room, hb - R_ - 1.0),
                              (-hb + R_ + 1.0, la[0] - G.LUG_H / 2 - room)) if b[1] - b[0] >= 2.0]
            return bs if bs and max(b[1] for b in bs) - min(b[0] for b in bs) >= sep else []

        # the latch low on the slab; on a short block where the pivots then find no heights, high instead
        latch_high = not c_bands(self._latch_band(P)) and bool(c_bands(self._latch_band(P, high=True)))
        la_lo, la_hi = self._latch_band(P, high=latch_high)
        bands = c_bands((la_lo, la_hi))
        if not bands:
            return G._no("no height for the carrier's pivots")
        # with two bands, both pivots may share the wider one when it holds them ``sep`` apart (the coupler then lies
        # flatter, away from a dead point), else one in each
        wide = max(bands, key=lambda b: b[1] - b[0])
        same_band = len(bands) > 1 and wide[1] - wide[0] >= sep + 1.0 and rng.random() < 0.5
        # one nearer B, one farther: the farther for the stack whose posts stand farther out on the head (the links
        # then run side by side, not crossing)
        t_h_ = self.head_top_t(P)
        mids = {st_: sum(self._post_band(P, SL, st_, t_h_)) / 2 for st_ in ("crank", "rocker")}
        near_first = mids["crank"] < mids["rocker"]
        for name, stack in (("C2", "crank"), ("C4", "rocker")):
            near = (name == "C2") == near_first
            for _ in range(40):
                # the nearer pivot higher (the nearer link then ends above the farther link's lower end instead of
                # running past it): with two bands it takes the upper one
                if len(bands) > 1 and not same_band:
                    bnd = max(bands, key=lambda b: b[0]) if near else min(bands, key=lambda b: b[0])
                else:  # one band: the nearer in its upper half, the farther in its lower
                    b0_, b1_ = max(bands, key=lambda b: b[1] - b[0])
                    m_ = (b0_ + b1_) / 2
                    bnd = (m_, b1_) if near else (b0_, m_)
                c_tt = rng.uniform(*bnd)
                ear_w = [SL[stack + "_ear_lo"], SL[stack + "_ear_hi"]]
                # in front of B's bolt heads, short of the flange's bolt heads and of the frame's head (the carrier
                # comes straight down past it)
                s_hi = min(self._s_heads(P, c_tt, *ear_w),
                           self._s_plane(fr0, t_head0, ear_w, SH.SET + SH.SLAB + HEAD_OUT)) - R_ - 2.0
                s_lo = t_b + bb.head_h + R_ + 2.0
                if s_hi - s_lo < 2.0:
                    continue
                mid = (s_lo + s_hi) / 2
                C = np.array([rng.uniform(s_lo, mid) if near else rng.uniform(mid, s_hi), c_tt])
                # (the side feet stand beyond every axle's line: a low pivot's screw passes them)
                if (C[0] <= s_hi
                        and abs(C[1]) + R_ + 1.0 <= hb
                        and all(np.linalg.norm(C - X) >= 2 * R_ for X in Cs.values())):  # (in their own layers;
                    # each screw's line missing the other clevis is checked below)
                    Cs[name] = C
                    break
            else:
                return G._no("carrier pivots out of their room")
        C2, C4 = Cs["C2"], Cs["C4"]
        t_head = self.head_top_t(P)
        fr_ = {"origin": P.O, "s": P.s, "t": P.t, "axis": P.a}
        nut_c = AS.nut_corner_d(G.AXLE) / 2
        # the posts stand in their own layers. Each axle's screw or nut comes along its line through the other
        # stack's layers: there it must miss the other link and the other post (its circle and the column under it,
        # down to the head), so the posts stand at least ``clear`` apart along the plane
        clear = R_ + nut_c + 1.5
        o_t = (t_head + R_ + 8.0, t_head + R_ + 8.0 + O_RISE)

        def band(stack):
            return self._post_band(P, SL, stack, t_head)

        def why(k):
            G.WHY["o: " + k] = G.WHY.get("o: " + k, 0) + 1

        def post_ok(O, C, stack):
            ws = G._probe_ws(SL[stack + "_ear_lo"]) + G._probe_ws(SL[stack + "_ear_hi"])
            if np.linalg.norm(O - C) < 2 * R_ + 4.0:
                return why("too near its coupler pivot")
            # the link at home clear of the frame and the flange (its bolt heads included), in its own layer
            lk_ = [G2.stadium(O, C, R_, 16)]
            if any(G2.shape_dist(lk_, scene.at(w_)["base"] + scene.at(w_)["fixed"]) < G.SCREEN_GAP + 0.5
                   for w_ in G._probe_ws(SL[stack + "_link"])):
                return why("link meets the frame at home")
            # (sampled 0.6 mm in from the post's sides: a post flush with the plates' front stands on its very edge)
            top = G.saddle_top(scene, O[0] - R_ + 0.6, O[0] + R_ - 0.6, ws, O[1] - R_ - 3.0)
            high = G.base_high(scene, O[0] - R_ + 0.6, O[0] + R_ - 0.6, ws + G._probe_ws(SL[stack + "_link"]))
            if top is None or O[1] - high < R_ + 8.0:
                return why("no head under the post")
            if G._hits_keep(scene, [G2.circle(O, R_), G2.rect(O[0] - R_, O[0] + R_, top - 6.0, O[1])], ws):
                return why("post in a key way")
            return top

        b2, b4 = band("crank"), band("rocker")
        if b2[1] < b2[0] or b4[1] < b4[0]:
            return G._no("no room on the head for a post")
        Os, posts = {}, {}
        # the links slope down towards B: the post nearer B stands in the upper half of the heights, the farther in the
        # lower half (each then stands farther off the other's link)
        mid_t = (o_t[0] + o_t[1]) / 2
        crank_far = sum(b2) > sum(b4)
        t2 = (o_t[0], mid_t) if crank_far else (mid_t, o_t[1])
        t4 = (mid_t, o_t[1]) if crank_far else (o_t[0], mid_t)
        for _ in range(12):
            O2 = np.array([self._post_s(rng.uniform(*b2), b2, P, SL, "crank"), rng.uniform(*t2)])
            top2 = post_ok(O2, C2, "crank")
            if top2 is None:
                continue
            # the second post: along the plane at least ``clear`` from the first
            opts = [r for r in ((b4[0], min(b4[1], O2[0] - clear)), (max(b4[0], O2[0] + clear), b4[1])) if r[1] > r[0]]
            # on the same side of the first as its coupler pivot (the links side by side)
            opts = [r for r in opts if (r[0] < O2[0]) == (C4[0] < C2[0])]
            if not opts:
                why("no room for the second post")
                continue
            for _ in range(10):
                O4 = np.array([self._post_s(rng.uniform(*rng.choice(opts)), b4, P, SL, "rocker"), rng.uniform(*t4)])
                if abs(O4[0] - O2[0]) < clear:
                    continue
                top4 = post_ok(O4, C4, "rocker")
                if top4 is None:
                    continue
                if min(_seg_dist(X, O4, C4) for X in (O2, C2)) < clear or \
                        min(_seg_dist(X, O2, C2) for X in (O4, C4)) < clear:
                    why("axle line crosses the other link")
                    continue
                Os, posts = {"O2": O2, "O4": O4}, {"O2": top2, "O4": top4}
                break
            if Os:
                break
        if not Os:
            return G._no("no ground pivots on the head")
        O2, O4 = Os["O2"], Os["O4"]
        # each carrier pivot's screw and nut come along its line: it must miss the other clevis's arm (from B's
        # plate out to that pivot, a knuckle high)
        nut_r = AS.nut_corner_d(G.AXLE) / 2 + 1.0
        for X, Y in ((C2, C4), (C4, C2)):
            if X[0] <= Y[0] + R_ + nut_r and abs(X[1] - Y[1]) < R_ + nut_r:
                return G._no("a carrier pivot's screw line crosses the other clevis")

        r2, r3, r4 = (float(np.linalg.norm(x)) for x in (C2 - O2, C4 - C2, C4 - O4))
        th = math.atan2(*(C2 - O2)[::-1])
        got = LK.fourbar(O2, O4, r2, r3, r4, th, 1)
        mode = 1 if got is not None and np.linalg.norm(got[1] - C4) < 1e-6 else -1
        d = {"layers": L, "Rk": R_, "t_b": t_b, "mech": {"O2": O2.tolist(), "O4": O4.tolist(), "C2": C2.tolist(),
                                                         "C4": C4.tolist(), "r2": r2, "r3": r3, "r4": r4,
                                                         "theta_home": th, "mode": mode},
             "post_tops": posts}
        # the drive: the crank turned back from home by R, whichever way lifts B (it then comes down home, onto
        # the latch's ledge, and starts clear of the shelf), the linkage continuous all the way
        # each way: as far as the linkage turns continuously (up to DRIVE's top), then a turn drawn among those
        # that leave B at least START_RISE above home
        R = None
        for sgn in rng.sample([-1, 1], 2):
            d["drive"] = {"range_deg": sgn * DRIVE[1]}
            ok_k = []
            for k in range(1, int(DRIVE[1]) + 1):
                got = G.coupler(d, -sgn * k)
                if got is None or not G.TRANSMISSION[0] <= G.transmission(d, *got) <= G.TRANSMISSION[1]:
                    break
                if k >= DRIVE[0] and G.coupler_move(d, *got)[1][1] >= START_RISE:
                    ok_k.append(k)
            if ok_k:
                R = float(sgn * rng.choice(ok_k))
                break
        if R is None:
            m_ = d["mech"]
            t_home = G.transmission(d, np.asarray(m_["C2"]), np.asarray(m_["C4"]))
            return G._no("the linkage does not lift B continuously to a start" + (
                " (dead near home)" if not G.TRANSMISSION[0] <= t_home <= G.TRANSMISSION[1] else ""))
        d["drive"] = {"range_deg": R}
        d["drive"]["targets_at"] = [round(rng.uniform(0.25, 0.4) * R, 2), round(rng.uniform(0.55, 0.75) * R, 2), R]
        # the latch: the lug on the carrier's arm over a ledge whose post rises from the foot on the slab's back,
        # below the flange's lowest bolt heads; kept where the arm stays clear of the post all the way in (sampled
        # as the screen samples the drive)
        ws = G._probe_ws(SL["latch"])
        pth = G.path(d)
        if pth is None:
            return G._no("the hidden linkage nears a dead point")
        pts = [G.coupler_move(d, C2_, C4_) for _, C2_, C4_ in pth[::3] if _ != pth[-1][0]]
        fr_ = {"origin": P.O, "s": P.s, "t": P.t, "axis": P.a}
        s_back = self._s_plane(fr_, self.bed_t(P), [SL["latch"]], SH.SET + SH.SLAB)
        t_lo, t_hi = la_lo, la_hi
        # short of the flange's bolt heads in the latch's layer (the carrier's arm comes straight down past them)
        s_heads_l = self._s_plane(fr_, 0.0, [SL["latch"]], SH.SET + SH.SLAB + BOLTS[self.itf_a.bolt].head_h)
        s_top = min(s_back - G.LUG_S / 2 - 1.5, s_heads_l - G.LUG_S / 2 - 1.5 if self._latch_heads(P) else math.inf)
        for _ in range(15):
            X = np.array([rng.uniform(s_back - FOOT_LEN + G.LUG_S / 2 + 1.0, s_top),
                          rng.uniform(t_lo, max(t_lo, t_hi))])
            top = G.saddle_top(scene, X[0] - G.LUG_S / 2, X[0] + G.LUG_S / 2, ws, X[1] - G.LUG_H / 2 - G.REF_GAP - 3.0)
            if top is None or top > X[1] - G.LUG_H / 2 - G.REF_GAP - 3.0 - 1e-6:
                continue
            ledge = float(X[1] - G.LUG_H / 2 - G.REF_GAP)
            post = [G2.rect(X[0] - G.LUG_S / 2, X[0] + G.LUG_S / 2, top - 2.0, ledge)]
            if G._hits_keep(scene, [G2.rect(X[0] - G.LUG_S / 2, X[0] + G.LUG_S / 2, top - 6.0, ledge)], ws):
                continue
            arm = G2.rect(t_b - 0.5, X[0] + G.LUG_S / 2, X[1] - G.LUG_H / 2, X[1] + G.LUG_H / 2)
            if all(G2.shape_dist([G2.move(arm, a_, sh_)], post) >= G.SCREEN_GAP for a_, sh_ in pts):
                d["latch"] = {"X": X.tolist(), "ledge": ledge, "post_foot": top}
                return d
        return G._no("no place for the latch")

    def _post_band(self, P, SL, stack, t_head) -> tuple[float, float]:
        """Where a ground pivot may stand along s: its post's foot wholly on the head's top in every ear layer, 1.5
        mm in from the slab's side and POST_SET back from the head's slanted front (the head's fingers then run on
        well past the post's front face, not a stub's few millimetres)."""
        from . import generate as G

        R_ = G.rk()
        fr = {"origin": P.O, "s": P.s, "t": P.t, "axis": P.a}
        ear_w = [SL[stack + "_ear_lo"], SL[stack + "_ear_hi"]]
        # (the band starts flush with the plates' front: a post is then either flush with it or POST_SET back from it,
        # see ``_post_s``)
        # (and POST_SET short of the plates' slanted back end, the slab's front plane: the plates stand free above
        # the slab, and a post nearer their back would leave a stub of plate behind it)
        return (-self._s_plane(fr, t_head, ear_w, SH.SET + SH.SLAB + HEAD_OUT, most=True) + R_,
                self._s_plane(fr, t_head, ear_w, SH.SET) - R_ - POST_SET)

    def _post_s(self, s_o: float, band: tuple, P=None, SL=None, stack=None) -> float:
        """A post's s: flush with the plates' front, or at least POST_SET back from it (a shorter run-on of the
        plate past the post's front face would be a stub); and its back face clear of the slab's slanted back face
        (where that face meets a post's back face within a millimetre, the edges between them are too short)."""
        from . import generate as G

        s_o = band[0] if s_o < band[0] + POST_SET else s_o
        if P is not None:
            R_ = G.rk()
            fr = {"origin": P.O, "s": P.s, "t": P.t, "axis": P.a}
            ear_w = [SL[stack + "_ear_lo"], SL[stack + "_ear_hi"]]
            t_head = self.head_top_t(P)
            # the slab's back edge (exposed in the gaps beside the plates) meets the post's back face where the
            # ears' side faces are: keep the back face 0.25 mm off each of those four points (the edges between them
            # then stay over Q_SHORT_EDGE's 0.1 mm)
            sbs = [self._s_plane(fr, t_head, [(w, w)], SH.SET + SH.SLAB) for sl in ear_w for w in sl]
            bad = lambda v: any(abs(v + R_ - x) < 0.25 for x in sbs)  # noqa: E731
            if bad(s_o):
                opts = [v for x in sbs for v in (x - 0.3 - R_, x + 0.3 - R_) if band[0] <= v <= band[1] and not bad(v)
                        and (v == band[0] or v >= band[0] + POST_SET)]
                s_o = min(opts, key=lambda v: abs(v - s_o)) if opts else band[0]
        return s_o

    def _reach(self, P) -> float:
        """How far the side feet run out from the slab (along the flange's axis): a third of the frame's height."""
        return (self.head_top_t(P) + POST_ABOVE - self.bed_t(P)) / 3.0 + 4.0

    def _feet_w(self) -> list:
        """The side feet's layers: just outside the mechanism's outer layers, either side."""
        L = self.layers()
        return [(L["lo"] - FOOT_ROOM - SIDE_FOOT, L["lo"] - FOOT_ROOM), (L["hi"] + FOOT_ROOM, L["hi"] + FOOT_ROOM + SIDE_FOOT)]

    def _feet_s_end(self, P) -> float:
        """Where both side feet end, towards B (one s for both, square to the mechanism's plane): ``reach`` short of
        the farther of the two's meeting with the slab (the outline round both and the slab is then that wide)."""
        fr = {"origin": P.O, "s": P.s, "t": P.t, "axis": P.a}
        end = -self._s_plane(fr, self.bed_t(P), self._feet_w(), SH.SET, most=True) - self._reach(P)
        # at least 6 mm past the outer plates' fronts (they merge: a shorter run-on would be a stub)
        from . import generate as G

        SL = G.slabs(self.layers())
        t_head = self.head_top_t(P)
        fronts = [-self._s_plane(fr, t_head, [SL[k]], SH.SET + SH.SLAB + HEAD_OUT, most=True)
                  for k in ("crank_ear_lo", "crank_ear_hi", "rocker_ear_lo", "rocker_ear_hi")]
        return min(end, min(fronts) - 6.0)

    def _latch_heads(self, P) -> bool:
        """Whether any of the flange's bolt heads stands in the latch's layer."""
        from . import generate as G

        L = self.layers()
        w0, w1 = L["latch"] - G.LUG_T / 2 - 1.0, L["latch"] + G.LUG_T / 2 + 1.0
        pa = self.home["A"]
        n_ = np.asarray(pa.vec((0, 0, 1)), float)
        pb = BOLTS[self.itf_a.bolt]
        for x, y in self.itf_a.holes:
            c = np.asarray(pa.point((x, y, 0.0)), float)
            ws = [float((c + n_ * dx - P.O) @ P.a) for dx in (SH.SET + SH.SLAB, SH.SET + SH.SLAB + pb.head_h)]
            if min(ws) - pb.head_d / 2 < w1 and max(ws) + pb.head_d / 2 > w0:
                return True
        return False

    def _latch_band(self, P, high: bool = False) -> tuple[float, float]:
        """The latch lug's height range (t of its centre): over the foot by a post, on B's plate, and the lowest
        LATCH_SPAN of room clear of the flange's bolt heads that stand in the latch's layer."""
        from . import generate as G

        L = self.layers()
        w0, w1 = L["latch"] - G.LUG_T / 2 - 1.0, L["latch"] + G.LUG_T / 2 + 1.0
        pa = self.home["A"]
        n_ = np.asarray(pa.vec((0, 0, 1)), float)
        pb = BOLTS[self.itf_a.bolt]
        bars = []  # the heads' heights in the latch's layer, with room
        for x, y in self.itf_a.holes:
            c = np.asarray(pa.point((x, y, 0.0)), float)
            ws = [float((c + n_ * dx - P.O) @ P.a) for dx in (SH.SET + SH.SLAB, SH.SET + SH.SLAB + pb.head_h)]
            if min(ws) - pb.head_d / 2 < w1 and max(ws) + pb.head_d / 2 > w0:
                tc = float(P.st(c)[1])
                bars.append((tc - pb.head_d / 2 - 1.0, tc + pb.head_d / 2 + 1.0))
        t_lo = max(self.bed_t(P) + FOOT_H + G.LUG_H / 2 + G.REF_GAP + 3.0 + 0.5,
                   -(self.itf_b.face_h - 3.0) / 2 + G.LUG_H / 2 + 1.0)
        t_top = (self.itf_b.face_h - 3.0) / 2 - G.LUG_H / 2 - 1.0
        free = lambda t: all(t + G.LUG_H / 2 < lo or t - G.LUG_H / 2 > hi for lo, hi in bars)  # noqa: E731
        if high:  # the highest LATCH_SPAN of room instead (on a short block: the carrier's pivots then go below)
            t = t_top
            while t >= t_lo and not free(t):
                t -= 0.5
            if t < t_lo:
                return t_top, t_top
            lo = t
            while lo - 0.5 >= max(t - LATCH_SPAN, t_lo) and free(lo - 0.5):
                lo -= 0.5
            return lo, t
        t = t_lo
        while t <= t_top and not free(t):
            t += 0.5
        if t > t_top:
            return t_lo, t_lo
        hi = t
        while hi + 0.5 <= min(t + LATCH_SPAN, 6.0 if t <= 6.0 else t + LATCH_SPAN) and free(hi + 0.5):
            hi += 0.5
        return t, hi

    def _s_heads(self, P, t: float, *slabs_w) -> float:
        """The least s, over the layers ``slabs_w`` [(w0, w1)] at height t, of the flange's bolt heads' outer faces
        (the slab's back face plus a head): the carrier's clevises stay short of it."""
        fr = {"origin": P.O, "s": P.s, "t": P.t, "axis": P.a}
        return self._s_plane(fr, t, slabs_w, SH.SET + SH.SLAB + BOLTS[self.itf_a.bolt].head_h)

    def _s_plane(self, fr: dict, t: float, slabs_w, depth: float, most: bool = False) -> float:
        """The least s (``most``: minus the greatest), over the layers ``slabs_w`` [(w0, w1)] at height t, of the
        plane ``depth`` mm out from the flange's mounting face (towards B)."""
        pa = self.home["A"]
        n = np.asarray(pa.vec((0, 0, 1)), float)
        Pb = np.asarray(pa.origin, float) + n * depth
        O, s_, t_, a_ = (np.asarray(fr[k], float) for k in ("origin", "s", "t", "axis"))
        out = math.inf
        for w0, w1 in slabs_w:
            for w in (w0, w1):
                v = float(n @ (Pb - (O + t * t_ + w * a_))) / float(n @ s_)
                out = min(out, -v if most else v)
        return out

    def fixed_solids(self) -> dict:
        """The fixtures in the world; the flange with its bolts' heads standing on the frame's slab, so the 2D
        screens keep the links and the carrier clear of them (the grader measures them with the frame)."""
        out = {n: fixed_part_solid(itf).moved(self.home[n].location()) for n, itf in self.parts().items()}
        pa = self.home["A"]
        n_ = np.asarray(pa.vec((0, 0, 1)), float)
        pb = BOLTS[self.itf_a.bolt]
        heads = [bd.Plane(origin=tuple(np.asarray(pa.point((x, y, 0.0)), float) + n_ * (SH.SET + SH.SLAB)),
                          z_dir=tuple(n_)) * bd.Pos(0, 0, pb.head_h / 2) * bd.Cylinder(pb.head_d / 2, pb.head_h)
                 for x, y in self.itf_a.holes]
        out["A"] = bd.Compound([out["A"]] + heads)
        return out
