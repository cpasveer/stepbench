"""Exact overlap between parts placed at one pose, with cheap tests first.

Three filters run before any exact boolean, each of them exact in what it
rules out:

1. **Height bands.** Motion is in parallel planes, so two parts whose
   height ranges along the normal do not overlap can never touch.
2. **Fixed pairs.** Two parts that never move (frame, housing, obstacles)
   are tested once, not per pose.
3. **Plan outlines.** Each part's outline seen along the normal is wrapped
   in a convex polygon once; two separated polygons mean no contact.

Whatever is left goes to the core's exact measure: the volume of the
boolean intersection of the two solids (``jointbench.checks.base.volume``,
the same call the core NO_INTERFERENCE check makes).
"""
from __future__ import annotations

import math

import build123d as bd
import numpy as np

from ..checks.base import volume
from .kin import Pose2, compose, inverse
from .parts3d import primitives


def plan_hull(solid) -> np.ndarray:
    pts = []
    for e in solid.edges():
        n = 2 if e.geom_type == bd.GeomType.LINE else 48
        for i in range(n + 1):
            p = e.position_at(i / n)
            pts.append((p.X, p.Y))
    P = np.unique(np.round(np.array(pts), 9), axis=0)
    P = P[np.lexsort((P[:, 1], P[:, 0]))]

    def cr(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def half(points):
        h = []
        for p in points:
            while len(h) >= 2 and cr(h[-2], h[-1], p) <= 0:
                h.pop()
            h.append(p)
        return h

    lower, upper = half(P), half(P[::-1])
    hull = np.array(lower[:-1] + upper[:-1])
    # curved edges are sampled in 48 chords; the hull can sit inside the true
    # outline by the chord's sag (under 0.11 mm for a 50 mm radius), which the
    # separation test's margin covers
    return hull


def _move(hull: np.ndarray, pose: Pose2) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    R = np.array([[c, -s], [s, c]])
    return hull @ R.T + np.array(pose[:2])


def _normals(H: np.ndarray) -> np.ndarray:
    e = np.roll(H, -1, axis=0) - H
    n = np.stack([e[:, 1], -e[:, 0]], axis=1)
    ln = np.hypot(n[:, 0], n[:, 1])
    keep = ln > 1e-12
    return n[keep] / ln[keep, None]


def _rot(N: np.ndarray, a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return N @ np.array([[c, s], [-s, c]])


def _separated(A: np.ndarray, B: np.ndarray, margin: float, N: np.ndarray | None = None) -> bool:
    """Separating-axis test for two convex polygons, all axes at once."""
    if N is None:
        N = np.concatenate([_normals(A), _normals(B)])
    pa, pb = A @ N.T, B @ N.T
    gap = np.maximum(pb.min(axis=0) - pa.max(axis=0), pa.min(axis=0) - pb.max(axis=0))
    return bool(gap.max() > margin)


def location(pose: Pose2) -> bd.Location:
    return bd.Location((pose[0], pose[1], 0.0), (0.0, 0.0, math.degrees(pose[2])))


def _curved(solid) -> bool:
    return any(e.geom_type != bd.GeomType.LINE for e in solid.edges())


class Piece:
    __slots__ = ("hull", "z", "c", "r", "margin", "normals")

    def __init__(self, solid):
        bb = solid.bounding_box()
        self.z = (bb.min.Z, bb.max.Z)
        self.hull = plan_hull(solid)
        self.c = self.hull.mean(axis=0)
        self.r = float(np.max(np.hypot(*(self.hull - self.c).T)))
        self.margin = 0.12 if _curved(solid) else 1e-6
        self.normals = _normals(self.hull)


def slab_pieces(solid, cuts: list[float], cell: float = 15.0) -> list:
    """A solid cut into height slices at ``cuts`` and each slice into plan
    cells no larger than ``cell`` mm: pieces whose union is the solid. Small
    pieces hug the shape far more tightly than one outline round the whole."""
    bb = solid.bounding_box()
    zs = sorted({z for z in cuts if bb.min.Z + 1e-6 < z < bb.max.Z - 1e-6} | {bb.min.Z, bb.max.Z})
    pad = 0.5
    x0, y0 = bb.min.X - pad, bb.min.Y - pad
    lx, ly = bb.max.X - bb.min.X + 2 * pad, bb.max.Y - bb.min.Y + 2 * pad
    nx, ny = max(1, math.ceil(lx / cell)), max(1, math.ceil(ly / cell))
    if len(zs) == 2 and nx == ny == 1:
        return [solid]
    out = []
    zs = [z for i, z in enumerate(zs) if i == 0 or z - zs[i - 1] > 1e-3 or i == len(zs) - 1]
    for a, b in zip(zs, zs[1:]):
        if b - a < 1e-4:
            continue
        for i in range(nx):
            for k in range(ny):
                box = bd.Pos(x0 + (i + 0.5) * lx / nx, y0 + (k + 0.5) * ly / ny, (a + b) / 2) * bd.Box(
                    lx / nx, ly / ny, b - a)
                piece = solid & box
                if volume(piece) > 1e-9:
                    out.extend(piece.solids())
    return out


class Scene:
    """Parts with their solids prepared once; ``clashes(poses)`` per pose.

    Filters before the exact boolean, all of them sound:

    * height bands that do not overlap (motion is in parallel planes);
    * each part split into pieces (a recipe part into the shapes it is
      built from, a submitted part into height slices) and each piece's plan
      outline wrapped in a circle and a convex polygon; separated pieces
      cannot touch;
    * a pin sitting in its own hole: a piece of the pin-side part that lies
      wholly inside the hole's radius, against material of the hole-side part
      only at heights where the hole is open, cannot touch it (the solver
      keeps the two axes on one line). That the hole is really empty over its
      height is checked once, exactly.
    """

    def __init__(self, parts: dict, solids: dict, limit: float = 0.01, joints: dict | None = None,
                 cuts: list[float] | None = None, recipes: dict | None = None):
        """``recipes``: the recipe each solid was built from, where it differs
        from ``parts[k].recipe``; a part with no recipe is cut into height
        slices at ``cuts``."""
        self.limit = limit
        self.ids = list(solids)
        self.solids = solids
        self.kind = {k: parts[k].kind for k in solids}
        self.label = {k: (parts[k].label or k) for k in solids}
        self.default = {k: tuple(parts[k].shown) for k in solids if parts[k].kind != "link"}
        self.z = {}
        self.hull = {}
        self.pieces = {}
        cuts = sorted(set(cuts or []))
        for k, s in solids.items():
            bb = s.bounding_box()
            self.z[k] = (bb.min.Z, bb.max.Z)
            self.hull[k] = plan_hull(s)
            rec = (recipes or {}).get(k, parts[k].recipe)
            if rec is not None:
                prims = primitives(rec)
            else:
                from .outline import z_levels
                # flat plates are settled by their outlines; no need for fine plan cells
                prims = slab_pieces(s, cuts, cell=1e9 if z_levels(s) is not None else 15.0)
            self.pieces[k] = [Piece(p) for p in prims]
        # revolute joints usable for the pin-in-hole rule
        self.rules = {}
        for j in (joints or {}).values():
            if j.kind != "revolute" or j.a[0] not in solids or j.b[0] not in solids:
                continue
            fa = parts[j.a[0]].features.get(j.a[1])
            fb = parts[j.b[0]].features.get(j.b[1])
            if fa is None or fb is None or {fa.kind, fb.kind} != {"pin", "hole"}:
                continue
            (pp, pf), (hp, hf) = ((j.a[0], fa), (j.b[0], fb)) if fa.kind == "pin" else ((j.b[0], fb), (j.a[0], fa))
            rh = hf.size / 2
            void = bd.Pos(hf.xy[0], hf.xy[1], hf.z[0]) * bd.Cylinder(
                rh - 0.01, hf.z[1] - hf.z[0], align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            if volume(solids[hp] & void) > limit:
                continue  # something inside the hole: no shortcut for this joint
            inside = [bool(np.all(np.hypot(*(pc.hull - np.array(pf.xy)).T) <= rh - 1e-6))
                      for pc in self.pieces[pp]]
            self.rules[frozenset((pp, hp))] = (pp, hp, inside, hf.z)
        # straight channels (slots) a pin or tongue rides in
        self.channels = {}
        for j in (joints or {}).values():
            if j.kind not in ("slide", "pin_slot") or j.a[0] not in solids or j.b[0] not in solids:
                continue
            fa = parts[j.a[0]].features.get(j.a[1])
            fb = parts[j.b[0]].features.get(j.b[1])
            if fa is None or fb is None:
                continue
            if fb.kind == "slot":
                (op, of), (sp, sf) = (j.a[0], fa), (j.b[0], fb)
            elif fa.kind == "slot":
                (op, of), (sp, sf) = (j.b[0], fb), (j.a[0], fa)
            else:
                continue
            if sf.along is None or sf.dir is None:
                continue
            lo, hi = sf.along
            L = hi - lo
            mid = (lo + hi) / 2
            u = (math.cos(sf.dir), math.sin(sf.dir))
            cx, cy = sf.xy[0] + mid * u[0], sf.xy[1] + mid * u[1]
            void = bd.Pos(cx, cy, (sf.z[0] + sf.z[1]) / 2) * bd.Rot(0, 0, math.degrees(sf.dir)) * bd.Box(
                L - 0.02, sf.size - 0.02, sf.z[1] - sf.z[0] - 0.02)
            if volume(solids[sp] & void) > limit:
                continue
            self.channels.setdefault(frozenset((op, sp)), []).append((op, sp, sf, lo, hi))
        self.pairs = []
        for i, a in enumerate(self.ids):
            for b in self.ids[i + 1:]:
                if self.kind[a] != "link" and self.kind[b] != "link":
                    continue
                if min(self.z[a][1], self.z[b][1]) - max(self.z[a][0], self.z[b][0]) <= 1e-6:
                    continue
                pp = []
                for ia, pa in enumerate(self.pieces[a]):
                    for ib, pb in enumerate(self.pieces[b]):
                        lo, hi = max(pa.z[0], pb.z[0]), min(pa.z[1], pb.z[1])
                        if hi - lo > 1e-6:
                            pp.append((ia, ib, lo, hi))
                if pp:
                    arr = np.array(pp)
                    self.pairs.append((a, b, pp, arr[:, 0].astype(int), arr[:, 1].astype(int)))
        self.n_exact = 0
        from .outline import z_levels
        self.levels = {k: z_levels(s) for k, s in solids.items()}
        self._outlines: dict = {}
        self.C = {k: np.array([p.c for p in v]) for k, v in self.pieces.items()}
        self.R = {k: np.array([p.r + p.margin for p in v]) for k, v in self.pieces.items()}

    def _centres(self, k, pose, cache):
        c = cache.get(k)
        if c is None:
            c = cache[k] = _move(self.C[k], pose)
        return c

    def _needs_exact(self, a, b, pp, poses, IA=None, IB=None, cache=None) -> bool:
        rule = self.rules.get(frozenset((a, b)))
        chans = self.channels.get(frozenset((a, b)))
        Pa, Pb = poses[a], poses[b]
        if IA is not None:
            cache = {} if cache is None else cache
            ca_, cb_ = self._centres(a, Pa, cache)[IA], self._centres(b, Pb, cache)[IB]
            d = np.hypot(*(ca_ - cb_).T)
            near = np.nonzero(d <= self.R[a][IA] + self.R[b][IB])[0]
            if len(near) == 0:
                return False
            todo = [pp[i] for i in near]
        else:
            todo = pp
        for ia, ib, lo, hi in todo:
            qa, qb = self.pieces[a][ia], self.pieces[b][ib]
            m = qa.margin + qb.margin
            if rule is not None:
                pin_part, _, inside, zh = rule
                k = ia if pin_part == a else ib
                if inside[k] and lo >= zh[0] - 1e-6 and hi <= zh[1] + 1e-6:
                    continue
            if chans is not None and self._in_channel(chans, a, ia, b, ib, lo, hi, poses):
                continue
            N = np.concatenate([_rot(qa.normals, Pa[2]), _rot(qb.normals, Pb[2])])
            if _separated(_move(qa.hull, Pa), _move(qb.hull, Pb), m, N):
                continue
            return True
        return False

    def _in_channel(self, chans, a, ia, b, ib, lo, hi, poses) -> bool:
        """The rider's piece lies wholly inside the slot's empty channel."""
        for op, sp, sf, alo, ahi in chans:
            if not (lo >= sf.z[0] - 1e-6 and hi <= sf.z[1] + 1e-6):
                continue
            piece = self.pieces[a][ia] if op == a else self.pieces[b][ib]
            rel = compose(inverse(poses[sp]), poses[op])
            H = _move(piece.hull, rel) - np.array(sf.xy)
            u = np.array([math.cos(sf.dir), math.sin(sf.dir)])
            n = np.array([-u[1], u[0]])
            m = piece.margin + 1e-6
            t, d = H @ u, H @ n
            if np.max(np.abs(d)) <= sf.size / 2 - m and t.min() >= alo + m and t.max() <= ahi - m:
                return True
        return False

    def _plates_clear(self, a, b, poses) -> bool:
        """Both parts are flat plates: compare their outlines in every height
        slab they share, coarse outlines first (sampled within 0.001 mm: clear
        if the gap exceeds their error), then fine ones (within 0.0001 mm:
        clear if they do not cross). Outlines that do not cross can hide a true
        overlap no deeper than 0.0002 mm; over a contact under 0.25 mm long
        (a radius up to 100 mm) and 50 mm tall that is under 0.005 mm3, half
        the overlap limit. Parts that merely touch, like a cam and its roller,
        are settled here without an exact boolean."""
        from .outline import outline_gap
        la, lb = self.levels.get(a), self.levels.get(b)
        if la is None or lb is None:
            return False
        lo, hi = max(self.z[a][0], self.z[b][0]), min(self.z[a][1], self.z[b][1])
        if hi - lo > 50.0 or self.limit < 0.01 - 1e-12:
            return False
        cuts = sorted({lo, hi} | {z for z in la + lb if lo < z < hi})
        for z0, z1 in zip(cuts, cuts[1:]):
            if z1 - z0 < 1e-6:
                continue
            zm = (z0 + z1) / 2
            oa, ob = self._outline(a, zm, 1e-3), self._outline(b, zm, 1e-3)
            g, _, _ = outline_gap(oa, poses[a], ob, poses[b], reach=1.0)
            if g > 2 * (oa.deflection + ob.deflection):
                continue
            oa, ob = self._outline(a, zm, 1e-4), self._outline(b, zm, 1e-4)
            g, _, _ = outline_gap(oa, poses[a], ob, poses[b], reach=1.0)
            if g <= 0.0:
                return False
        return True

    def _outline(self, k, z, deflection):
        from .outline import Outline
        key = (k, round(z, 6), deflection)
        o = self._outlines.get(key)
        if o is None:
            o = self._outlines[key] = Outline(self.solids[k], z, deflection)
        return o

    def clashes(self, poses: dict, only: set | None = None, margin: float = 0.0) -> list[tuple]:
        """[(a, b, volume)] for every pair overlapping by more than the limit.
        ``only``: restrict to pairs touching these parts."""
        out = []
        cache: dict = {}
        poses = {**self.default, **poses}
        for a, b, pp, IA, IB in self.pairs:
            if only is not None and a not in only and b not in only:
                continue
            if not self._needs_exact(a, b, pp, poses, IA, IB, cache):
                continue
            if self._plates_clear(a, b, poses):
                continue
            rel = compose(inverse(poses[b]), poses[a])
            self.n_exact += 1
            v = volume(self.solids[a].moved(location(rel)) & self.solids[b])
            if v > self.limit:
                out.append((a, b, v))
        return out
