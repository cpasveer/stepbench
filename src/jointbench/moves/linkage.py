"""Planar linkages built from declared pivots: mobility, and the pose at any drive angle.

The answer's rigid bodies are: the *ground* (fixture A and every part bolted
to it), *B* (fixture B and every part bolted to it), and each part bolted to
neither (a link). Pivots join two bodies about a common axis. All pivot axes
must be parallel: the linkage is planar, every body moves in planes square to
that axis, and its place is three numbers in the plane, (x, y, turn).

Each body's pose is kept relative to where it is assembled (pose 0, all
zeros). A pivot k between bodies i and j at plane point p says T_i(p) =
T_j(p): two equations. The drive says body j turns relative to body i, about
the drive pivot, by the drive angle: one equation.

*Mobility* is counted exactly as the number of free motions the pivot
equations leave at the assembled pose: 3 x (bodies - 1) minus the rank of
their Jacobian (Gruebler's count, corrected for redundant pivots). It must be
exactly 1, and the drive must take that one motion up.

*Posing* follows the drive in small steps and solves the closure equations
by Gauss-Newton from the previous step's pose, so the mechanism stays on the
branch it was assembled in; it cannot jump to the other way a four-bar can
close. A step that does not close (the links cannot reach) or where the
equations turn singular (a dead point: the drive no longer controls the
motion) stops the motion with that angle named. For a four-bar this is the
same answer the circle-meets-circle construction gives (tested).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..geometry import Placement
from . import motion as MO

CLOSE_TOL = 1e-9  # mm: closure equations met to this
DEAD_RATIO = 1e-5  # smallest / largest singular value below this: a dead point
RANK_TOL = 1e-7


class LinkageError(ValueError):
    pass


@dataclass
class Pivot:
    name: str
    i: int  # body index
    j: int
    p: np.ndarray  # plane coordinates at the assembled pose


@dataclass
class Linkage:
    bodies: list[str]  # bodies[0] is the ground
    pivots: list[Pivot]
    drive: int  # index into pivots
    origin: np.ndarray  # a point on the plane (3D)
    normal: np.ndarray  # the common pivot axis direction (3D, unit)
    e1: np.ndarray
    e2: np.ndarray
    drive_sign: float = 1.0  # +1 if the drive pivot's declared axis points along ``normal``, else -1

    @property
    def n(self) -> int:
        return 3 * (len(self.bodies) - 1)

    def to_plane(self, p) -> np.ndarray:
        v = np.asarray(p, float) - self.origin
        return np.array([v @ self.e1, v @ self.e2])

    # ---- equations

    def _pose(self, q, b):
        if b == 0:
            return 0.0, 0.0, 0.0
        k = 3 * (b - 1)
        return q[k], q[k + 1], q[k + 2]

    def residual(self, q, theta=None) -> np.ndarray:
        out = []
        for pv in self.pivots:
            a = _apply(self._pose(q, pv.i), pv.p)
            b = _apply(self._pose(q, pv.j), pv.p)
            out += [a[0] - b[0], a[1] - b[1]]
        if theta is not None:
            d = self.pivots[self.drive]
            out.append(self._pose(q, d.j)[2] - self._pose(q, d.i)[2] - self.drive_sign * theta)
        return np.array(out)

    def jacobian(self, q, drive=True) -> np.ndarray:
        rows = []
        for pv in self.pivots:
            for comp in (0, 1):
                r = np.zeros(self.n)
                for b, sgn in ((pv.i, 1.0), (pv.j, -1.0)):
                    if b == 0:
                        continue
                    k = 3 * (b - 1)
                    _, _, t = self._pose(q, b)
                    c, s = math.cos(t), math.sin(t)
                    dp = (-s * pv.p[0] - c * pv.p[1], c * pv.p[0] - s * pv.p[1])
                    r[k + comp] += sgn
                    r[k + 2] += sgn * dp[comp]
                rows.append(r)
        if drive:
            d = self.pivots[self.drive]
            r = np.zeros(self.n)
            if d.j:
                r[3 * (d.j - 1) + 2] += 1.0
            if d.i:
                r[3 * (d.i - 1) + 2] -= 1.0
            rows.append(r)
        return np.array(rows)

    # ---- mobility

    def mobility(self) -> tuple[int, int]:
        """(free motions the pivots leave, rank with the drive added)."""
        q = np.zeros(self.n)
        J = self.jacobian(q, drive=False)
        rank = _rank(J) if J.size else 0
        Jd = self.jacobian(q, drive=True)
        return self.n - rank, _rank(Jd)

    # ---- posing

    def solve(self, theta: float, q0) -> tuple[np.ndarray, float, float]:
        """(q, residual norm, conditioning) at drive angle ``theta`` (rad), from ``q0``."""
        q = np.array(q0, float)
        for _ in range(60):
            r = self.residual(q, theta)
            J = self.jacobian(q)
            dq = np.linalg.lstsq(J, -r, rcond=None)[0]
            q = q + dq
            if float(np.linalg.norm(dq)) < 1e-13:
                break
        r = float(np.linalg.norm(self.residual(q, theta)))
        sv = np.linalg.svd(self.jacobian(q), compute_uv=False)
        return q, r, float(sv[-1] / sv[0]) if sv[0] > 0 else 0.0

    def sweep(self, end_deg: float, step_deg: float = 0.25):
        """[(drive deg, q)] from 0 towards ``end_deg``, and None or (deg, why) if it stops early."""
        n = max(1, math.ceil(abs(end_deg) / step_deg - 1e-9))
        q = np.zeros(self.n)
        out = [(0.0, q.copy())]
        for i in range(1, n + 1):
            deg = end_deg * i / n
            # predict along the last step, then correct
            guess = q if len(out) < 2 else 2 * out[-1][1] - out[-2][1]
            q2, res, cond = self.solve(math.radians(deg), guess)
            if res > CLOSE_TOL:
                return out, (deg, f"the mechanism cannot close at {deg:.2f} deg of drive (the links cannot reach; "
                                  f"{res:.3g} mm short)")
            if cond < DEAD_RATIO:
                return out, (deg, f"the mechanism reaches a dead point at {deg:.2f} deg of drive (the drive no "
                                  "longer controls it)")
            if float(np.max(np.abs(q2 - q)[2::3])) > math.radians(30):
                return out, (deg, f"the mechanism jumps to another way of closing at {deg:.2f} deg of drive")
            q = q2
            out.append((deg, q.copy()))
        return out, None

    def placement(self, q, b: int) -> Placement:
        """Body ``b``'s 3D motion from its assembled place, as a Placement."""
        x, y, t = self._pose(q, b)
        rot = MO.about(tuple(self.origin), tuple(self.normal), math.degrees(t))
        return rot.then(MO.shift(x * self.e1 + y * self.e2))


def _apply(pose, p):
    x, y, t = pose
    c, s = math.cos(t), math.sin(t)
    return (c * p[0] - s * p[1] + x, s * p[0] + c * p[1] + y)


def _rank(J) -> int:
    sv = np.linalg.svd(J, compute_uv=False)
    if not sv.size or sv[0] == 0:
        return 0
    return int(np.sum(sv > RANK_TOL * max(1.0, sv[0])))


def build(bodies: list[str], pivots: list[tuple[str, str, str, np.ndarray, np.ndarray]], drive: str,
          parallel_deg: float = 0.01) -> Linkage:
    """``pivots``: (name, body i, body j, point 3D, axis 3D). Raises LinkageError if not planar."""
    if not pivots:
        raise LinkageError("no pivots")
    n0 = np.asarray(pivots[0][4], float)
    n0 = n0 / np.linalg.norm(n0)
    for name, _, _, _, ax in pivots:
        a = np.asarray(ax, float) / np.linalg.norm(ax)
        ang = math.degrees(math.acos(min(1.0, abs(float(a @ n0)))))
        if ang > parallel_deg:
            raise LinkageError(f"pivot {name}'s axis is {ang:.3f} deg off pivot {pivots[0][0]}'s: all pivot axes "
                               f"must be parallel (within {parallel_deg:g} deg); only planar linkages are graded")
    a1, e1, e2 = MO.frame(n0)
    origin = np.asarray(pivots[0][3], float)
    idx = {b: k for k, b in enumerate(bodies)}
    L = Linkage(bodies, [], 0, origin, a1, e1, e2)
    for name, bi, bj, p, _ in pivots:
        L.pivots.append(Pivot(name, idx[bi], idx[bj], L.to_plane(p)))
    names = [p[0] for p in pivots]
    if drive not in names:
        raise LinkageError(f"the drive names pivot {drive!r}, which is not declared")
    L.drive = names.index(drive)
    dax = np.asarray(pivots[L.drive][4], float)
    L.drive_sign = 1.0 if float(dax @ a1) > 0 else -1.0
    return L


# ---------------------------------------------------------------- the four-bar in closed form


def fourbar(O2, O4, r2: float, r3: float, r4: float, theta: float, mode: int):
    """Crank tip and rocker tip of a four-bar (plane points), crank at angle ``theta`` (rad) about O2;
    ``mode`` +-1 picks which of the two closures. None if it cannot close."""
    O2, O4 = np.asarray(O2, float), np.asarray(O4, float)
    A = O2 + r2 * np.array([math.cos(theta), math.sin(theta)])
    d = O4 - A
    L = float(np.linalg.norm(d))
    if L > r3 + r4 or L < abs(r3 - r4) or L == 0:
        return None
    x = (r3 ** 2 - r4 ** 2 + L ** 2) / (2 * L)
    h = math.sqrt(max(r3 ** 2 - x ** 2, 0.0))
    u = d / L
    perp = np.array([-u[1], u[0]])
    Bp = A + x * u + mode * h * perp
    return A, Bp
