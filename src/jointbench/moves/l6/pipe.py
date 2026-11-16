"""Level 6 on the pipe setup: what the setup supplies to the shared generator.

The fixed parts are the pipe setup's own (``families/pipe/round5``, the same
draw as its levels 1-3): the pipe A stays fixed (in level 4 it runs through
the box), the square plate B is carried. B's end pose is its own pose from
levels 1-3: its home.

The setup supplies:

* the fixtures and their home placements, and the stated load (level 3's);
* the mechanism frame: B moves in the vertical plane through its centre that
  holds its face's normal; every pivot is horizontal, along the face;
* the frame's base on the pipe (``base``): round5's saddle -- the wedge between
  the two bolt rows' head faces, the pipe carved out, a key's way out along
  every bolt -- run down along the pipe to a flat print bed, so the whole frame
  prints standing on that bed with world up as its print direction. Pivot
  posts and the latch's ledge rise straight up from it.
"""
from __future__ import annotations

import math

import build123d as bd
import numpy as np

from ...catalogue import BOLTS, fixed_part_solid
from ...families.pipe import round5 as R5
from ...families.pipe.bracket import _halfspace, _prism_yz, _unit

NAME = "pipe"


class Setup:
    """The pipe setup (seed 0: the public showcase draw)."""

    name = NAME
    fixed = "A"
    carried = "B"
    pivot_s_max = 115.0  # mm from B's face: the frame's base (cut level above B's home) still stands under a post
    layer_bias = 3.5  # mm along the pivots' axis: the pipe lies 24 mm to that side of B's middle, so the saddle
    # under the mechanism is widest there

    def __init__(self, seed: int = 0):
        if seed != 0:
            raise NotImplementedError("the pipe setup's level 6 is built on its showcase draw (seed 0) for now")
        self.seed = seed
        self.d = R5.showcase_draw()
        self.itf_a, self.itf_b, _, _ = R5.layout(self.d)
        self.home = dict(zip(("A", "B"), R5.placements(self.d)))
        self.load_kg = float(self.d["load_kg"])

    def parts(self) -> dict:
        return {"A": self.itf_a, "B": self.itf_b}

    def frame(self) -> dict:
        """origin (B's home centre), s (B's face normal made level: from B towards the pipe), t (world up),
        axis (every pivot's direction: along B's face, level)."""
        O = np.array(self.home["B"].origin, float)
        t = np.array([0.0, 0.0, 1.0])
        s = np.array(self.home["B"].vec((0, 0, 1)), float)
        s = _unit(s - t * float(s @ t))
        return {"origin": O, "s": s, "t": t, "axis": _unit(np.cross(s, t))}

    # ---- the frame's base on the pipe
    def _section(self):
        itf = self.itf_a
        R = itf.face_h / 2
        rows = sorted({round(math.degrees(math.atan2(a[1], a[2])), 6) for _, a in itf.hole_frames})
        amax = max(abs(r) for r in rows)
        d_face = R + R5.G_SADDLE

        def u(a):
            return np.array([math.sin(a), math.cos(a)])

        lo, hi = math.radians(min(rows)), math.radians(max(rows))
        ridge = np.linalg.solve(np.array([u(lo), u(hi)]), np.array([d_face, d_face]))
        a_side = math.radians(amax + R5.WRAP)

        def out_to(a, fa):
            dd = np.array([math.sin(a), math.cos(a)])
            return tuple(dd * (d_face / float(u(fa) @ dd)))

        return [(R * math.sin(-a_side), R * math.cos(-a_side)), out_to(-a_side, lo), tuple(ridge), out_to(a_side, hi),
                (R * math.sin(a_side), R * math.cos(a_side)), (0.0, 0.0)]

    def base(self, z_bed: float, z_top: float) -> bd.Part:
        """The saddle on the pipe, in the world, from below the print bed ``z_bed`` to above ``z_top``, with a
        hole and a key's way out (a teardrop channel, roof up) along every bolt. Cut to the bed and carved by
        ``finish`` once the posts are joined."""
        itf = self.itf_a
        pb = BOLTS[itf.bolt]
        pa = self.home["A"]
        poly = self._section()
        ax_w = np.array(pa.vec((1, 0, 0)))
        o_w = np.array(pa.origin)
        reach = max(abs(float(pa.vec((0.0, y, z))[2])) for y, z in poly) + 5.0
        # down along the pipe until the whole section is below the bed (cut flat there); up only to just past the
        # highest bolt's head (round5's EDGE), ended square to the pipe: the saddle no longer than it must be
        bx = [p_[0] for p_, _ in itf.hole_frames]
        up_x = ax_w[2] > 0
        x_end = (max(bx) if up_x else min(bx)) + (1 if up_x else -1) * (pb.head_d / 2 + R5.EDGE)
        x_bed = (z_bed - reach - float(o_w[2])) / ax_w[2]
        body = _prism_yz(poly, min(x_bed, x_end), max(x_bed, x_end))
        up = _unit(np.array(pa.rot).T @ np.array([0.0, 0.0, 1.0]))  # world up, in the pipe's frame
        for p, a in itf.hole_frames:
            P, V = np.asarray(p, float), _unit(a)
            rt = pb.head_d / 2 + 1.2
            w_up = _unit(up - V * float(up @ V))
            w_side = np.cross(V, w_up)
            k = rt * math.sqrt(0.5)
            tear = bd.Circle(rt) + bd.Polygon((0.0, -rt * math.sqrt(2.0)), (k, -k), (-k, -k), align=None)
            tpl = bd.Plane(origin=tuple(P + V * (R5.G_SADDLE + 0.3)), x_dir=tuple(w_side), z_dir=tuple(V))
            body = body - bd.extrude(tpl * tear, pb.head_h + 60.0, dir=tuple(V))
            body = body - bd.Plane(origin=tuple(P), z_dir=tuple(V)) * bd.Pos(0, 0, R5.G_SADDLE / 2) * bd.Cylinder(
                pb.clear_medium / 2, R5.G_SADDLE + 4)
        return body.moved(pa.location())

    def below_roof(self, sink: float = 1.5) -> bd.Part:
        """Everything under the saddle's top faces less ``sink`` (the axis side of its four outer faces), in the
        world: a post on the saddle is cut to it, so it stands on the saddle's own faces and ends nowhere else."""
        poly = self._section()
        roof = poly[:5]  # outer chain: side wing, head face, head face, side wing
        R = self.itf_a.face_h / 2
        region = bd.Rot(0, 90, 0) * bd.Cylinder(4 * R, 3000.0)  # a big cap round the pipe's axis
        for (y0, z0), (y1, z1) in zip(roof[:-1], roof[1:]):
            n = np.array([0.0, z1 - z0, -(y1 - y0)])  # in the section, square to the segment
            n /= np.linalg.norm(n)
            mid = np.array([0.0, (y0 + y1) / 2, (z0 + z1) / 2])
            if float(n @ mid) < 0:
                n = -n  # pointing away from the axis
            region = region - _halfspace(mid - n * sink, -n)  # keep the axis side
        return region.moved(self.home["A"].location())

    def shadow(self) -> bd.Part:
        """The pipe and everything straight under it, in the world: a post is cut to stay out of it, since material
        under the pipe could never come down past it (every part goes in from above)."""
        R = self.itf_a.face_h / 2
        pa = self.home["A"]
        c = np.asarray(pa.origin, float)
        u = _unit(np.asarray(pa.vec((1.0, 0.0, 0.0)), float))  # the pipe's axis
        w = np.cross(np.array([0.0, 0.0, 1.0]), u)
        w = _unit(w)  # level, square to the axis: the silhouette seen from above is the band within R of the axis
        band = bd.Plane(origin=tuple(c), x_dir=tuple(u), z_dir=tuple(np.cross(u, w))) * bd.Rectangle(3000.0, 2 * R)
        under = bd.extrude(band, 600.0, dir=(0.0, 0.0, -1.0))
        pipe = (bd.Rot(0, 90, 0) * bd.Cylinder(R, 3000.0)).moved(pa.location())
        return under + pipe

    def keep_out(self) -> list:
        """Bodies the frame's posts must not grow into, in the world: each bolt's key channel (as cut above)."""
        itf = self.itf_a
        pb = BOLTS[itf.bolt]
        pa = self.home["A"]
        out = []
        for p, a in itf.hole_frames:
            P, V = np.asarray(p, float), _unit(a)
            rt = pb.head_d / 2 + 1.2
            out.append((bd.Plane(origin=tuple(P + V * R5.G_SADDLE), z_dir=tuple(V)) * bd.Pos(0, 0, 32.0)
                        * bd.Cylinder(rt + 0.5, 64.0)).moved(pa.location()))
        return out

    def post_keep_out(self) -> list:
        """Round each bolt, from under the saddle up along its key's way, in the world: a post is cut back from it
        before it is joined to the saddle, so it neither fills the bolt's hole nor crowds its head (one clean pocket
        round the bolt, never a second copy of the hole's own surface)."""
        itf = self.itf_a
        pb = BOLTS[itf.bolt]
        pa = self.home["A"]
        out = []
        for p, a in itf.hole_frames:
            P, V = np.asarray(p, float), _unit(a)
            rt = pb.head_d / 2 + 1.2
            out.append((bd.Plane(origin=tuple(P - V * 2.0), z_dir=tuple(V)) * bd.Pos(0, 0, 40.0)
                        * bd.Cylinder(rt + 0.5, 80.0)).moved(pa.location()))
        return out

    def finish(self, body: bd.Part, z_bed: float) -> bd.Part:
        """Carve the pipe out of the frame and cut it flat at the print bed."""
        R = self.itf_a.face_h / 2
        pipe = bd.Rot(0, 90, 0) * bd.Cylinder(R, 3000.0)
        body = body - pipe.moved(self.home["A"].location())
        body = body - _halfspace(np.array([0.0, 0.0, z_bed]), np.array([0.0, 0.0, 1.0]))  # all below the bed
        for k in self.keep_out():
            body = body - k
        return body

    def fixed_solids(self) -> dict:
        return {n: fixed_part_solid(itf).moved(self.home[n].location()) for n, itf in self.parts().items()}
