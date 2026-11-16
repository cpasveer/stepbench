"""KEY_TURN: every bolt can be turned with its hex key, not just reached.

TOOL_ACCESS asks whether a straight tool reaches the head. A bolt can pass
that and still be impossible to tighten: an L-shaped key has to swing its
long leg round the bolt, and a wall a few millimetres away stops it. This
check swings the real key.

The key (catalogue ``KEYS``: ISO 4762 socket, ISO 2936 key lengths) is two
hexagonal bars at right angles. Its short leg sits in the socket, its tip
at the socket depth, its axis on the bolt axis; the long leg sticks out
sideways from the top of the short leg. The key may lean up to
``KEY_TILT_MAX`` off the bolt axis (a ball-end key), pivoting where the
short leg enters the socket. Only the key above the top of the head is
tested: the part in the socket is inside the bolt head.

Pass, per bolt: for some lean and some starting angle, the key can turn
``KEY_SWING`` (60 degrees, one hex step) about the bolt axis without
touching the part, the fixed parts or the obstacles. The nut end of a
through bolt is not judged: a nut is held with a spanner while the head is
turned. Graded for the verdict alone (a generator checking its reference),
the check stops at the first bolt that cannot be turned.

The grid searched, fixed so the result is deterministic:
  lean        0 degrees, then 5, 10, 15, 20, 25 degrees, each toward
              8 directions (every 45 degrees, measured from the long leg)
  swing       sampled every 2 degrees round the bolt axis (``FINE``);
              a swing of 60 degrees is 31 positions in a row, all free
  touching    the key's overlap with a body is more than the task's
              ``overlap_volume_max`` (the same noise limit as every other
              overlap check), found by exact solid intersection

How it stays fast. Most positions are settled by a point test, never by a
guess:
  1. Bodies whose bounding box is out of the key's reach are dropped.
  2. The remaining bodies' surfaces are sampled as points (on a grid of half
     the key size over a 0.05 mm triangulation of a copy of each body; finer would only be slower,
     never change a verdict). A key position with a surface point at
     least ``MARGIN`` inside the key certainly touches that body. Every lean
     is swept first every 10 degrees, then every 2 degrees inside the
     openings that could hold 60 degrees. This finds where the key cannot
     be; it can miss contact, never invent it.
  3. A swing the point test leaves open is then confirmed position by
     position (every 2 degrees) by exact solid intersection of the key with
     each body whose box it enters. Only a confirmed swing passes.
So a pass is always exact-checked; a fail means every lean was blocked
within 60 degrees by real surface points inside the key.
"""
from __future__ import annotations

from functools import lru_cache

import build123d as bd
import numpy as np

from ..catalogue import BOLTS, KEY_SWING, KEY_TILT_MAX, KEYS, is_nut
from .base import CheckResult, Context, volume

NAME = "KEY_TURN"
TILT_STEP = 5.0     # lean grid, degrees
LEAN_DIRS = 8       # lean directions, every 360 / 8 degrees
COARSE = 10.0       # first sweep, degrees
FINE = 2.0          # final sweep and exact confirmation, degrees
MARGIN = 0.2        # a surface point this deep inside the key proves contact, mm
SPACING = 0.5       # surface sample spacing, as a fraction of the key size s
DEFLECTION = 0.05   # triangulation tolerance (well inside MARGIN), mm


# --------------------------------------------------------------------------
# the key


@lru_cache(maxsize=None)
def key_legs(size: str) -> tuple:
    """The key's two legs as convex solids in the key's own frame: pivot (the
    socket mouth, top of the head) at the origin, short leg up +Z, long leg
    along +X. Each leg is a hexagonal bar ``s`` across flats."""
    k = KEYS[size]
    s = k.s
    top = k.short_leg - k.socket_depth  # outer end of the short leg above the pivot
    hexagon = bd.RegularPolygon(s / 2, 6, major_radius=False)
    short = bd.extrude(hexagon, top)
    # long leg: a flat on top, flush with the top of the short leg
    long_ = bd.Pos(-s / 2, 0, top - s / 2) * bd.Rot(0, 90, 0) * bd.extrude(bd.Rot(0, 0, 30) * hexagon, k.long_leg)
    return short.solids()[0], long_.solids()[0]


@lru_cache(maxsize=None)
def _leg_data(size: str):
    """Per leg: outward face planes (normals, offsets) and corner points."""
    out = []
    for leg in key_legs(size):
        n, d = [], []
        for f in leg.faces():
            c = f.center()
            v = f.normal_at(c)
            n.append((v.X, v.Y, v.Z))
            d.append(v.X * c.X + v.Y * c.Y + v.Z * c.Z)
        n, d = np.array(n), np.array(d)
        corners = np.array([tuple(v) for v in leg.vertices()])
        # outward: the leg's own centre is inside every plane
        mid = corners.mean(axis=0)
        flip = (n @ mid) > d
        n[flip] *= -1
        d[flip] *= -1
        out.append((n, d, corners))
    return out


def _rz(deg: float) -> np.ndarray:
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _lean(tilt: float, direction: float) -> np.ndarray:
    """Rotation that leans the short leg by ``tilt`` toward ``direction``
    (degrees from the long leg, about the short leg's own axis)."""
    if tilt == 0:
        return np.eye(3)
    a = np.radians(direction)
    k = np.array([-np.sin(a), np.cos(a), 0.0])  # z x (cos a, sin a, 0)
    t = np.radians(tilt)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * (K @ K)


def leans() -> list[tuple[float, float]]:
    """The lean grid: straight first, then increasing lean, each direction."""
    out = [(0.0, 0.0)]
    n = int(round(KEY_TILT_MAX / TILT_STEP))
    for i in range(1, n + 1):
        for j in range(LEAN_DIRS):
            out.append((i * TILT_STEP, j * 360.0 / LEAN_DIRS))
    return out


def reach(size: str) -> float:
    """Radius round the pivot that holds every possible key position."""
    k = KEYS[size]
    return float(np.hypot(k.long_leg, k.short_leg)) + k.s


# --------------------------------------------------------------------------
# bodies as surface points


def _surface_points(solid, centre: np.ndarray, radius: float, h: float) -> np.ndarray:
    """Points on the solid's surface within ``radius`` of ``centre``, no more
    than about ``h`` apart: the corners of a fine triangulation, and on
    every triangle larger than ``h`` a square grid of that spacing in
    the triangle's plane, plus points along its edges."""
    import copy

    from ._meshcut import mesh

    # a copy: the triangulation is stored on the shape, and other checks
    # mesh the same solid at their own tolerance
    T = mesh(copy.deepcopy(solid), DEFLECTION)
    if len(T) == 0:
        return np.zeros((0, 3))
    lo, hi = T.min(axis=1), T.max(axis=1)
    T = T[np.all(hi >= centre - radius, axis=1) & np.all(lo <= centre + radius, axis=1)]
    if len(T) == 0:
        return np.zeros((0, 3))
    pts = [T.reshape(-1, 3)]
    edge = np.max(np.linalg.norm(T - np.roll(T, 1, axis=1), axis=2), axis=1)
    T = T[edge > h]
    if len(T):
        A, Bv, C = T[:, 0], T[:, 1], T[:, 2]
        for p, q in ((A, Bv), (Bv, C), (C, A)):  # along the edges
            n = np.ceil(np.linalg.norm(q - p, axis=1) / h).astype(int)
            for m in np.unique(n[n > 1]):
                sel = n == m
                f = np.arange(1, m) / m
                pts.append((p[sel, None, :] + f[None, :, None] * (q - p)[sel, None, :]).reshape(-1, 3))
        # each triangle's plane frame: u along its first edge, v square to it
        U = Bv - A
        U /= np.linalg.norm(U, axis=1)[:, None]
        AC = C - A
        cu = np.einsum("ij,ij->i", AC, U)
        V = AC - cu[:, None] * U
        cv = np.linalg.norm(V, axis=1)
        ok = cv > 1e-9
        V[ok] /= cv[ok, None]
        bu = np.einsum("ij,ij->i", Bv - A, U)
        ou = np.einsum("ij,ij->i", centre - A, U)
        ov = np.einsum("ij,ij->i", centre - A, V)
        u0 = np.maximum(np.minimum(0.0, cu), ou - radius)
        u1 = np.minimum(np.maximum(bu, cu), ou + radius)
        v1 = np.minimum(cv, ov + radius)
        for i in np.nonzero(ok & (u0 <= u1) & (v1 >= 0))[0]:
            gu = np.arange(np.floor(u0[i] / h), np.ceil(u1[i] / h) + 1) * h
            gv = np.arange(0, np.ceil(v1[i] / h) + 1) * h
            Gu = np.repeat(gu, len(gv))
            Gv = np.tile(gv, len(gu))
            # inside: above the first edge, and on the inner side of the other two
            keep = (Gv >= 0) & ((cu[i] - bu[i]) * Gv - cv[i] * (Gu - bu[i]) >= 0) & (cv[i] * Gu - cu[i] * Gv >= 0)
            if keep.any():
                pts.append(A[i] + np.outer(Gu[keep], U[i]) + np.outer(Gv[keep], V[i]))
    P = np.concatenate(pts)
    P = P[np.linalg.norm(P - centre, axis=1) <= radius]
    return np.unique(np.round(P, 6), axis=0)


# --------------------------------------------------------------------------
# one bolt


def _sampled(solid, h: float, cache: dict | None) -> np.ndarray:
    """All of a body's surface points at spacing ``h``, kept in ``cache``
    (one grading) so a body is sampled once, not once per bolt."""
    key = (id(solid), h)
    if cache is not None and key in cache:
        return cache[key][1]
    bb = solid.bounding_box()
    lo, hi = np.array(tuple(bb.min)), np.array(tuple(bb.max))
    P = _surface_points(solid, (lo + hi) / 2, float(np.linalg.norm(hi - lo)) / 2 + 1.0, h)
    if cache is not None:
        cache[key] = (solid, P)  # the solid kept alive so its id stays its own
    return P


def _runs(free: np.ndarray) -> list[tuple[int, int]]:
    """Runs of consecutive True on a circle: (start index, length), longest
    first, ties by start. A circle all True is one run of len(free)."""
    n = len(free)
    if free.all():
        return [(0, n)]
    if not free.any():
        return []
    s = int(np.argmin(free))  # start the walk on a blocked sample
    out, i = [], 0
    while i < n:
        j = (s + i) % n
        if free[j]:
            k = i
            while k < n and free[(s + k) % n]:
                k += 1
            out.append(((s + i) % n, k - i))
            i = k
        else:
            i += 1
    return sorted(out, key=lambda r: (-r[1], r[0]))


class _Bolt:
    def __init__(self, size: str, pivot, axis, bodies: dict, lim: float, samples: dict | None = None):
        self.size = size
        self.c = np.array(pivot, dtype=float)
        pl = bd.Plane(origin=tuple(self.c), z_dir=tuple(axis))
        self.B = np.column_stack([tuple(pl.x_dir), tuple(pl.y_dir), tuple(pl.z_dir)])  # bolt frame
        self.lim = lim
        R = reach(size)
        self.bodies = {}
        for name, s in bodies.items():  # 1. bounding-box rejection
            bb = s.bounding_box()
            lo, hi = np.array(tuple(bb.min)), np.array(tuple(bb.max))
            if np.linalg.norm(np.clip(self.c, lo, hi) - self.c) <= R:
                self.bodies[name] = (s, lo, hi)
        pts, lab = [], []
        self.names = list(self.bodies)
        for i, name in enumerate(self.names):  # 2. surface points, in the bolt frame
            P = _sampled(self.bodies[name][0], KEYS[size].s * SPACING, samples)
            P = P[np.linalg.norm(P - self.c, axis=1) <= R]
            pts.append((P - self.c) @ self.B)
            lab.append(np.full(len(P), i))
        self.P = np.concatenate(pts) if pts else np.zeros((0, 3))
        self.lab = np.concatenate(lab) if lab else np.zeros(0, int)
        self.legs = _leg_data(size)
        self.exact_calls = 0

    # ---- point test ------------------------------------------------------

    def _near(self, L: np.ndarray) -> None:
        """Keep, for this lean, only the points the key can reach in any swing:
        turning about the bolt axis keeps height and distance from the axis,
        so the key's corners bound both (the legs are convex)."""
        C = np.concatenate([c for _, _, c in self.legs]) @ L.T
        r = np.hypot(self.P[:, 0], self.P[:, 1])
        keep = (self.P[:, 2] >= C[:, 2].min() - 1e-9) & (self.P[:, 2] <= C[:, 2].max() + 1e-9) & \
               (r <= np.hypot(C[:, 0], C[:, 1]).max() + 1e-9)
        self.Q, self.qlab = self.P[keep], self.lab[keep]

    def _hits(self, L: np.ndarray, theta: float) -> np.ndarray:
        """Labels of bodies with a surface point inside the key (by more
        than MARGIN) at lean rotation L and swing angle theta."""
        R = _rz(theta) @ L
        for n, d, corners in self.legs:
            C = corners @ R.T
            lo, hi = C.min(axis=0) - 1e-9, C.max(axis=0) + 1e-9
            P = self.Q
            box = (P[:, 0] >= lo[0]) & (P[:, 0] <= hi[0]) & (P[:, 1] >= lo[1]) & (P[:, 1] <= hi[1]) & \
                  (P[:, 2] >= lo[2]) & (P[:, 2] <= hi[2])
            if not box.any():
                continue
            inside = np.all(P[box] @ (n @ R.T).T < d - MARGIN, axis=1)
            if inside.any():  # the short leg is tested first; one blocked leg is enough
                return np.unique(self.qlab[box][inside])
        return np.zeros(0, int)

    # ---- exact test ------------------------------------------------------

    def _exact(self, L: np.ndarray, theta: float) -> list[tuple[str, float]]:
        """Bodies the key overlaps by more than the noise limit, by exact
        solid intersection (bodies outside the key's box skipped)."""
        R = self.B @ _rz(theta) @ L  # key frame -> world
        tr = bd.Location(_trsf(R, self.c))
        out = []
        for leg, (_, _, corners) in zip(key_legs(self.size), self.legs):
            C = corners @ R.T + self.c
            lo, hi = C.min(axis=0), C.max(axis=0)
            moved = None
            for name, (s, blo, bhi) in self.bodies.items():
                if np.any(hi < blo) or np.any(lo > bhi):
                    continue
                if moved is None:
                    moved = leg.moved(tr)
                self.exact_calls += 1
                v = volume(s & moved)
                if v > self.lim:
                    out.append((name, v))
        return out

    # ---- search ----------------------------------------------------------

    def run(self) -> dict:
        nc, nf = int(round(360 / COARSE)), int(round(360 / FINE))
        per = int(round(COARSE / FINE))
        need = int(round(KEY_SWING / FINE)) + 1  # fine positions in a row
        best = (-1.0, 0.0, 0.0, 0.0, [])  # swing, tilt, direction, start, blockers
        for tilt, direction in leans():
            L = _lean(tilt, direction)
            self._near(L)
            coarse = [self._hits(L, i * COARSE) for i in range(nc)]
            if best[0] == -1.0 and not any(len(h) == 0 for h in coarse):  # blocked all round
                best = (-0.5, tilt, direction, 0.0, sorted({self.names[i] for h in coarse for i in h}))
            cfree = np.array([len(h) == 0 for h in coarse])
            # a blocked coarse position is blocked in the fine sweep too; the
            # fine sweep covers each opening between blocked coarse positions
            # that could hold the swing, and always the widest one (for the report)
            hits = {i * per: coarse[i] for i in range(nc) if not cfree[i]}
            runs = _runs(cfree)
            for n, (start, length) in enumerate(runs):
                if n and (length + 1) * per - 1 < need:
                    break
                lo, hi = (0, nf - 1) if length == nc else (start * per - per + 1, (start + length) * per + per - 1)
                for j in range(lo, hi + 1):
                    jj = j % nf
                    if jj not in hits:
                        hits[jj] = coarse[jj // per] if jj % per == 0 else self._hits(L, jj * FINE)
            ffree = np.array([jj in hits and len(hits[jj]) == 0 for jj in range(nf)])
            for start, length in _runs(ffree):
                if length < need:  # longest first: nothing left can hold the swing
                    if (length - 1) * FINE > best[0]:
                        edge = [hits.get((start - 1) % nf, ()), hits.get((start + length) % nf, ())]
                        best = ((length - 1) * FINE, tilt, direction, start * FINE,
                                sorted({self.names[i] for h in edge for i in h}))
                    break
                # 3. confirm by exact intersection, position by position,
                # until KEY_SWING in a row is proven
                opening = 360.0 if length == nf else (length - 1) * FINE
                ok = np.zeros(length, bool)
                blockers: set[str] = set()
                run = 0
                for k in range(length):
                    hit = self._exact(L, ((start + k) % nf) * FINE)
                    if hit:
                        blockers.update(n_ for n_, _ in hit)
                        run = 0
                        continue
                    ok[k] = True
                    run += 1
                    if run >= need:
                        return {"ok": True, "swing": (need - 1) * FINE, "open": opening, "tilt": tilt,
                                "direction": direction, "start": ((start + k - need + 1) % nf) * FINE,
                                "blocked_by": []}
                sub = sorted(_runs_line(ok), key=lambda r: (-r[1], r[0]))
                if sub and (sub[0][1] - 1) * FINE > best[0]:
                    best = ((sub[0][1] - 1) * FINE, tilt, direction, ((start + sub[0][0]) % nf) * FINE,
                            sorted(blockers))
        swing, tilt, direction, start, by = best
        return {"ok": False, "swing": max(swing, 0.0), "open": max(swing, 0.0), "tilt": tilt, "direction": direction, "start": start,
                "blocked_by": by}


def _runs_line(free: np.ndarray) -> list[tuple[int, int]]:
    out, i, n = [], 0, len(free)
    while i < n:
        if free[i]:
            k = i
            while k < n and free[k]:
                k += 1
            out.append((i, k - i))
            i = k
        else:
            i += 1
    return out


def _trsf(R: np.ndarray, t: np.ndarray):
    from OCP.gp import gp_Trsf

    tr = gp_Trsf()
    tr.SetValues(*R[0], t[0], *R[1], t[1], *R[2], t[2])
    return tr


def turn(size: str, pivot, axis, bodies: dict, lim: float = 0.01, samples: dict | None = None) -> dict:
    """Search one bolt: ``pivot`` is the centre of the top of its head,
    ``axis`` points out of the head, ``bodies`` are name -> solid.

    Returns ``ok``; ``swing``, the swing in degrees proven free by exact
    intersection (a pass stops proving at ``KEY_SWING``; a fail gives the
    best found, which the point test bounds from above); ``open``, the
    opening the point test saw at that lean (up to 360); the lean
    (``tilt``, ``direction`` from the long leg) and ``start`` angle; and
    ``blocked_by``, the bodies that stop the best swing of a failing bolt."""
    return _Bolt(size, pivot, axis, bodies, lim, samples).run()


def run(ctx: Context) -> CheckResult:
    t = ctx.task
    lim = t.tolerances["overlap_volume_max"]
    rows, problems, samples = [], [], {}
    for k in ctx.poses():
        bodies = {"the part itself": ctx.solid_at(k)}
        bodies.update({f"fixed part {n}": s for n, s in ctx.fixed(k).items()})
        bodies.update({f"obstacle {n}": s for n, s in ctx.obstacles(k).items()})
        for bolt in t.bolts(ctx.stored(k)):
            if is_nut(bolt.size):  # the nut end of a through bolt: held with a spanner, the head is turned
                continue
            if problems and ctx.verdict_only:  # one bolt that cannot be turned decides the verdict
                break
            b = BOLTS[bolt.size]
            grip = ctx.grips.get((k, bolt.id), b.grip_min)
            ax = np.array(tuple(bolt.axis), dtype=float)
            ax /= np.linalg.norm(ax)
            pivot = np.array(tuple(bolt.position), dtype=float) + ax * (grip + b.head_h)
            r = turn(bolt.size, pivot, ax, bodies, lim, samples)
            row = {"bolt": bolt.id, "pose": k, "ok": r["ok"], "swing_deg": r["swing"], "open_deg": r["open"],
                   "tilt_deg": r["tilt"],
                   "lean_direction_deg": r["direction"], "start_deg": r["start"], "blocked_by": r["blocked_by"]}
            rows.append(row)
            if not r["ok"]:
                by = f", blocked by {' and '.join(r['blocked_by'])}" if r["blocked_by"] else ""
                problems.append(f"{bolt.id}: its {KEYS[bolt.size].s:g} mm hex key turns at most "
                                f"{r['swing']:g} of the {KEY_SWING:g} degrees needed{by}")
    worst = min((r["swing_deg"] for r in rows), default=360.0)
    details = {"bolts": rows, "grid": {"tilt_max": KEY_TILT_MAX, "tilt_step": TILT_STEP, "lean_directions": LEAN_DIRS,
                                       "coarse_deg": COARSE, "fine_deg": FINE, "swing_needed": KEY_SWING}}
    if problems:
        more = f" (+{len(problems) - 1} more)" if len(problems) > 1 else ""
        return CheckResult(NAME, False, problems[0] + more, worst, "deg", dict(details, problems=problems))
    return CheckResult(NAME, True, f"a standard hex key swings at least {KEY_SWING:g} degrees on every bolt "
                       f"(leaning at most {KEY_TILT_MAX:g} degrees)", worst, "deg", details)
