"""STRENGTH: nothing breaks under the design load (option A).

Two rules, on the same six design load cases the stiffness check solves
(the part is analysed once):

* **Radius at loaded inside corners.** An inside corner -- a concave edge
  where the surface turns by at least 30 deg, or a concave fillet -- is
  *loaded* if the stress beside it (within ``corner_reach``) reaches
  ``loaded_share`` of the allowable under some load case. Every loaded
  inside corner must be rounded by at least ``min_radius``; a sharp one has
  radius 0 and fails. Unloaded corners (cosmetic) do not count. This is
  what forces fillets: at a sharp corner the stress of an elastic model has
  no finite value, so no stress figure could judge it.
* **Averaged stress.** At every surface point, the von Mises stress averaged
  over a ball of ``average_radius`` (a fixed volume, so the figure does not
  hinge on one element), highest over the six cases, against the allowable
  = strength / safety factor of the process's material. Points within
  ``mount_zone`` of a mount's seat or bolt holes are left out (the rigid
  clamp is an idealisation and makes its own peaks), and so are points at
  sharp inside corners (judged by the radius rule).

Stress under a fixed load does not depend on the modulus (only a little on
Poisson's ratio), so the stiffness solve's stresses serve every material.
"""
from __future__ import annotations

import os
import time
from dataclasses import asdict, dataclass, field

import numpy as np

from . import settings


@dataclass
class Corner:
    kind: str  # "sharp edge" or "fillet"
    radius: float  # mm (0 for a sharp edge)
    stress: float  # MPa, highest beside it over the six cases
    loaded: bool
    ok: bool
    point: list


@dataclass
class Strength:
    file: str
    process: str
    material: str
    allowable: float  # MPa
    verdict: str = ""  # pass / fail / marginal / error
    reason: str = ""
    radius_ok: bool = True
    smallest_loaded_radius: float | None = None
    stress: float | None = None  # MPa, highest averaged stress
    stress_ratio: float | None = None  # stress / allowable
    stress_verdict: str = ""
    stress_point: list | None = None
    precision: float = 0.0
    corners: list = field(default_factory=list)
    design_loads: dict = field(default_factory=dict)
    seconds: float = 0.0
    note: str = ("a reported check for now; the stress figure is the average over a ball of the stated radius, "
                 "and sharp loaded inside corners fail the radius rule instead of being judged by stress")

    @property
    def passed(self) -> bool:
        return self.verdict in ("pass", "marginal")

    def to_json(self) -> dict:
        d = asdict(self)
        d["passed"] = self.passed
        return d

    def text(self) -> str:
        out = [f"STRENGTH  {self.verdict.upper()}  {self.material}, allowable {self.allowable:g} MPa  ({self.file})",
               f"  {self.reason}"]
        for c in self.corners:
            if c.loaded:
                out.append(f"  {'ok  ' if c.ok else 'FAIL'} loaded {c.kind} r = {c.radius:.2f} mm, "
                           f"{c.stress:.3g} MPa beside it, at {c.point}")
        return "\n".join(out)


TOP_POINTS = 300  # per load case: the surface points with the highest corner stress at which the ball is averaged


def _ball_pattern(r: float):
    """Sample offsets in a ball of radius r, and their volume weights (over
    4 pi / 3): 26 directions on three shells, at r/6, r/2 and 5r/6, each
    standing for the layer around it (0-r/3, r/3-2r/3, 2r/3-r)."""
    dirs = [np.array(v, float) for v in np.ndindex(3, 3, 3) if v != (1, 1, 1)]
    dirs = [(d - 1) / np.linalg.norm(d - 1) for d in dirs]
    pts, w = [], []
    for f in (1 / 6, 1 / 2, 5 / 6):
        shell = ((f + 1 / 6) ** 3 - (f - 1 / 6) ** 3) * r ** 3 / len(dirs)
        for d in dirs:
            pts.append(d * f * r)
            w.append(shell)
    return np.array(pts), np.array(w)


class _Locator:
    """Finds the tetrahedron holding a point, and interpolates a corner field
    linearly inside it."""

    def __init__(self, P, T):
        from scipy.spatial import cKDTree

        self.P, self.T = P, T
        self.tree = cKDTree(P[T].mean(axis=1))
        a = P[T[:, 0]]
        self.a = a
        M = np.stack([P[T[:, 1]] - a, P[T[:, 2]] - a, P[T[:, 3]] - a], axis=2)  # (n, 3, 3)
        self.inv = np.linalg.inv(M)

    def average(self, field, pts, wts) -> float:
        _, cand = self.tree.query(pts, k=24)
        tot = wsum = 0.0
        for q, row, w in zip(pts, cand, wts):
            lam = np.einsum("nij,nj->ni", self.inv[row], q - self.a[row])
            l0 = 1 - lam.sum(axis=1)
            ok = (lam >= -1e-9).all(axis=1) & (l0 >= -1e-9)
            if not ok.any():
                continue  # outside the material
            k = int(np.flatnonzero(ok)[0])
            t = self.T[row[k]]
            b = np.array([l0[k], *lam[k]])
            tot += w * float(b @ field[t])
            wsum += w
        return tot / wsum if wsum else 0.0


def material(process: str) -> dict:
    m = settings()["materials"][process]
    return {**m, "allowable": m["strength"] / m["safety"]}


def _envelope(rep) -> dict:
    """Per case and envelope, von Mises at every mesh node under the design load."""
    from .viz import design_load

    per = {c.label: rep.stresses[c.label] * design_load(rep, c) for c in rep.cases if c.label in rep.stresses}
    env = np.max(np.stack(list(per.values())), axis=0)
    return per, env


def _node_volumes(P, T) -> np.ndarray:
    a, b, c, d = (P[T[:, k]] for k in range(4))
    v = np.abs(np.einsum("ij,ij->i", np.cross(b - a, c - a), d - a)) / 6
    return np.bincount(T.ravel(), weights=np.repeat(v / 4, 4), minlength=len(P))


def judge(rep, step_path: str, process: str, average_radius: float | None = None) -> Strength:
    """Judge strength from a stiffness report made with ``keep_fields=True``
    (which carries the per-case stresses), for ``process``'s material.
    ``average_radius`` overrides the settings' ball radius (COMPLIANCE: thin
    blades, where a 0.5 mm ball would reach past the blade's middle)."""
    from scipy.spatial import cKDTree

    from ..dfm import part_for

    t0 = time.perf_counter()
    cfg = settings()["strength"]
    mat = material(process)
    allow = mat["allowable"]
    res = Strength(os.path.basename(step_path), process, mat["name"], round(allow, 3), precision=cfg["precision"],
                   design_loads=rep.design_loads)
    if not rep.ok:
        q = getattr(rep, "quality", None)
        if q is not None and not q.get("passed", True):
            res.verdict = "fail"  # an untidy model that cannot be analysed is the modeller's doing
            res.reason = f"could not be analysed: {rep.error}; the model is untidy: {', '.join(q.get('failed') or [])}"
        else:
            res.verdict, res.reason = "error", f"could not be measured: {rep.error}"
        return res
    P, BT = rep.surface
    surf = np.unique(BT)
    per, env = _envelope(rep)
    tree_s = cKDTree(P[surf])

    # mounts: nodes of the seat and bolt-hole triangles
    seat = np.zeros(len(BT), bool)
    for flag, _held in rep.regions.values():
        seat |= flag
    seat_nodes = np.unique(BT[seat])
    near_mount = np.zeros(len(P), bool)
    if seat_nodes.size:
        dist, _ = cKDTree(P[seat_nodes]).query(P, distance_upper_bound=cfg["mount_zone"])
        near_mount = np.isfinite(dist)

    # inside corners, from the solid itself
    part = part_for(step_path, 1.5)
    reach = cfg["corner_reach"]
    near_sharp = np.zeros(len(P), bool)

    def beside(points):
        idx = set()
        for q in np.atleast_2d(points):
            idx.update(surf[i] for i in tree_s.query_ball_point(q, reach))
        idx = np.array(sorted(i for i in idx if not near_mount[i]), dtype=int)
        return idx, (float(env[idx].max()) if idx.size else 0.0)

    for e in part.edges():
        if not e.concave or e.turn < 30.0:
            continue
        line = part.edge_polyline(e.index)
        idx, s = beside(line)
        near_sharp[idx] = True
        loaded = s >= cfg["loaded_share"] * allow
        res.corners.append(Corner("sharp edge", 0.0, round(s, 4), loaded, not loaded, [round(float(c), 3) for c in e.mid]))
    for x in part.rounds():
        if not x.concave or x.full:
            continue
        pts = part.P[np.isin(part.F, x.faces)]
        if not len(pts):
            continue
        idx, s = beside(pts)
        loaded = s >= cfg["loaded_share"] * allow
        ok = (not loaded) or x.radius >= cfg["min_radius"] - 1e-9
        res.corners.append(Corner("fillet", round(float(x.radius), 4), round(s, 4), loaded, ok,
                                  [round(float(c), 3) for c in x.where]))
    loaded = [c for c in res.corners if c.loaded]
    res.radius_ok = all(c.ok for c in res.corners)
    res.smallest_loaded_radius = min((c.radius for c in loaded), default=None)

    # stress averaged over a ball: the stress field (linear within each
    # element, from the smoothed corner values) is sampled at a fixed pattern
    # of points in the ball, those inside the material kept, each weighted by
    # the volume it stands for -- so the figure does not depend on how many
    # mesh points happen to fall in the ball
    cand_all = surf[~near_mount[surf] & ~near_sharp[surf]]
    radius = cfg["average_radius"] if average_radius is None else average_radius
    probe, wts = _ball_pattern(radius)
    loc = _Locator(P, rep.tets)
    best, where = 0.0, None
    for s in per.values():
        if not cand_all.size:
            break
        top = cand_all[np.argsort(-s[cand_all], kind="stable")[:TOP_POINTS]]
        for i in top:
            if s[i] <= best:  # the average cannot exceed the nearby peak by much; checked in order
                break
            # radius 0: the smoothed corner value itself (a ball of no volume has no weight)
            v = float(s[i]) if radius <= 0 else loc.average(s, P[i] + probe, wts)
            if v > best:
                best, where = v, [round(float(c), 3) for c in P[i]]
    res.stress, res.stress_point = round(best, 4), where
    res.stress_ratio = round(best / allow, 4) if allow else None
    p = cfg["precision"] / 100
    res.stress_verdict = "pass" if best <= allow * (1 - p) else "fail" if best > allow * (1 + p) else "marginal"

    if not res.radius_ok:
        worst = min((c for c in loaded if not c.ok), key=lambda c: c.radius)
        res.verdict = "fail"
        res.reason = (f"a loaded inside corner is rounded {worst.radius:.2f} mm (need {cfg['min_radius']:g}); "
                      f"{worst.stress:.3g} MPa beside it at {worst.point}")
    else:
        res.verdict = res.stress_verdict
        res.reason = (f"highest stress {best:.3g} MPa (averaged over a {radius:g} mm ball) is "
                      f"{100 * best / allow:.0f} % of the {allow:g} MPa allowable"
                      + (f" at {where}" if where else "") + f"; {len(loaded)} loaded inside corner(s), all rounded")
    res.seconds = round(time.perf_counter() - t0, 2)
    return res


def check(task_dir: str, file: str | None = None, process: str = "fdm", h=None):
    """Stiffness and strength from one analysis (at the strength mesh,
    32 elements per circle): returns (stiffness report, strength result)."""
    from . import analyse, find_answer
    from .mounts import from_task

    from . import judge_spec

    task, mounts = from_task(task_dir)
    path = file or find_answer(task_dir)
    rep = analyse(path, mounts, h=h, task=task.get("id", ""), curvature=int(settings()["strength"]["curvature_points"]),
                  process=process)
    if rep.ok:
        try:
            rep.requirement = judge_spec(rep, mounts)  # the stiffness verdict from the same analysis
        except (KeyError, AttributeError):
            pass  # no bolt data: no specification
    return rep, judge(rep, path, process)

