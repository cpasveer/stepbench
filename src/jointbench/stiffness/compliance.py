"""Compliance: a part that lets one mount make a stated small motion easily
and resists every other (a flexure, test E1).

One analysis, in the printed material (PLA): mount A is held at its seat and
bolt holes, mount B is loaded by a unit force along each world axis and a
unit moment about each, as for the stiffness figure, giving B's full 6 x 6
flexibility matrix C (world axes, about B's mount point). Moments are put on
the force scale through the span L between the mounts, as the stiffness
figure does (a moment M counts as a force M / L, a rotation r as a movement
r L), so every way of loading B is compared in N/mm.

The stated motion ("travel") is a unit vector e in that scaled space: a
slide along a direction d is (d, 0); a turn about an axis a through a point
P moves B's mount point by a x (B - P) per radian and turns it by a, scaled
(a x (B - P), a L). Three figures, read off C alone:

* **travel stiffness**: push B along the travel, read its movement along the
  travel: k = 1 / (e' C e). For a slide in N/mm; for a turn in N mm/rad.
* **ratio**: every other way of loading B, the six-dimensional space at
  right angles to the travel, is at least ``ratio`` times stiffer: the
  softest of them, 1 / (largest eigenvalue of C restricted to that space),
  divided by k. This also bounds how far a push along the travel may move B
  off it: for a ratio of 50, by under 1/sqrt(50) = 14 % of the travel.
* **stress at full travel**: the push that moves B the full stated travel
  along it, with B otherwise free, its stress field (superposed from the six
  unit solutions) judged by the same rules as STRENGTH: loaded inside
  corners rounded, the highest stress against the material's allowable.

The same analysis gives the stiffness figure with the travel left out (the
STIFFNESS requirement on a task with a travel): the softest way of loading B
at right angles to the travel, against the task's specification.
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import asdict, dataclass, field
from types import SimpleNamespace

import numpy as np

from . import fe, judge, material_for, settings, spec_requirements
from .mounts import Mount, from_task

NAMES = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")


@dataclass
class Travel:
    """The stated motion of mount B relative to mount A (world axes)."""

    kind: str  # "slide" or "turn"
    amount: float  # mm (slide) or degrees (turn), the full travel
    band: tuple  # (low, high): N/mm (slide) or N mm per degree (turn)
    ratio: float  # every other way at least this many times stiffer
    direction: list | None = None  # slide: unit vector
    axis: list | None = None  # turn: unit vector
    point: list | None = None  # turn: a point on the axis

    @staticmethod
    def from_task(t: dict) -> "Travel | None":
        c = (t.get("tolerances") or {}).get("compliance")
        if not c:
            return None
        return Travel(c["kind"], float(c["travel"]), tuple(c["band"]), float(c["ratio"]),
                      c.get("direction"), c.get("axis"), c.get("point"))

    def vector(self, origin: np.ndarray, span: float) -> tuple[np.ndarray, float]:
        """The travel as a unit vector in scaled motion space, and the length
        of the unscaled motion per unit of travel (1 for a slide, per radian
        for a turn: |(a x r, a L)|)."""
        if self.kind == "slide":
            d = np.asarray(self.direction, float)
            m = np.concatenate([d / np.linalg.norm(d), np.zeros(3)])
        else:
            a = np.asarray(self.axis, float)
            a = a / np.linalg.norm(a)
            m = np.concatenate([np.cross(a, origin - np.asarray(self.point, float)), a * span])
        n = float(np.linalg.norm(m))
        return m / n, n


@dataclass
class Compliance:
    file: str
    ok: bool = False
    error: str = ""
    fallback: str = ""
    material: str = ""
    E: float = 0.0
    kind: str = ""
    travel: float = 0.0  # mm or degrees
    band: tuple = ()
    ratio_required: float = 0.0
    stiffness: float = 0.0  # along the travel: N/mm (slide) or N mm per degree (turn)
    unit: str = ""
    band_verdict: str = ""
    other: float = 0.0  # N/mm, the softest way at right angles to the travel (scaled)
    other_description: str = ""
    ratio: float = 0.0  # other / travel stiffness, both in scaled N/mm
    ratio_verdict: str = ""
    off_travel: float = 0.0  # a push along the travel moves B off it by this share of its movement along it
    push: list = field(default_factory=list)  # Fx Fy Fz (N), Mx My Mz (N mm) for the full travel
    stress: dict | None = None  # STRENGTH's judgement of the full-travel push
    stress_verdict: str = ""
    stiffness_rest: dict | None = None  # STIFFNESS with the travel left out
    precision: float = 0.0
    span: float = 0.0
    C: list = field(default_factory=list)  # 6 x 6, world axes, mm and rad per N and N mm
    stages: list = field(default_factory=list)
    seconds: float = 0.0

    @property
    def passed(self) -> bool:
        return self.ok and all(v in ("pass", "marginal") for v in (self.band_verdict, self.ratio_verdict,
                                                                     self.stress_verdict))

    def to_json(self) -> dict:
        d = asdict(self)
        d["passed"] = self.passed
        return d

    def text(self) -> str:
        if not self.ok:
            return f"COMPLIANCE  NO RESULT  {self.file}: {self.error}"
        lo, hi = self.band
        t = f"{self.travel:g} {'mm' if self.kind == 'slide' else 'deg'}"
        out = [f"COMPLIANCE  {'PASS' if self.passed else 'FAIL'}  {self.file}  ({self.material}, E {self.E:g} MPa)",
               f"  along the travel ({self.kind}, {t}): {self.stiffness:.4g} {self.unit}, band {lo:g} to {hi:g}: "
               f"{self.band_verdict}",
               f"  every other way: softest {self.other:.4g} N/mm ({self.other_description}), "
               f"{self.ratio:.3g} x the travel, need {self.ratio_required:g} x: {self.ratio_verdict}",
               f"  a push along the travel moves B off it by {100 * self.off_travel:.1f} % of the travel",
               f"  full-travel push: " + ", ".join(f"{n} {v:.3g}" for n, v in zip(NAMES, self.push)),
               f"  stress at full travel: {self.stress_verdict}: {self.stress['reason'] if self.stress else ''}"]
        if self.stiffness_rest:
            s = self.stiffness_rest
            out.append(f"  STIFFNESS without the travel: {s['figure']:.4g} N/mm, need {s['required']:.4g}: {s['verdict']}")
        if self.fallback:
            out.append(f"  note: {self.fallback}")
        return "\n".join(out)


def _describe(v: np.ndarray) -> str:
    v = v * np.sign(v[np.argmax(np.abs(v))])
    return ", ".join(f"{NAMES[i]} {v[i] ** 2 * 100:.0f} %" for i in np.argsort(-np.abs(v)) if v[i] ** 2 >= 0.05)


def _band(x: float, lo: float, hi: float, p: float) -> str:
    if lo * (1 + p) <= x <= hi * (1 - p):
        return "pass"
    if x < lo * (1 - p) or x > hi * (1 + p):
        return "fail"
    return "marginal"


def analyse(task_dir: str, step_path: str, *, h: float | None = None, process: str = "fdm",
            curvature: int | None = None, average_radius: float | None = None) -> Compliance:
    """The compliance of the part in ``step_path`` on the task in ``task_dir``."""
    need = fe.available()
    if need:
        raise RuntimeError(need)
    with open(os.path.join(task_dir, "task.json"), encoding="utf-8") as fh:
        import json

        task = json.load(fh)
    travel = Travel.from_task(task)
    if travel is None:
        raise ValueError("the task states no travel (tolerances.compliance)")
    _, mounts = from_task(task_dir)
    cfg = settings()
    mat = material_for(process)
    msh = cfg["mesh"]
    if h == "screen":
        h, prec = msh["screen_element_size"], msh["screen_precision"]
    else:
        h = h or msh["element_size"]
        prec = msh["precision"] if h <= msh["element_size"] else math.nan
    # the stress needs the finer curvature; the coarse screen takes the stiffness check's
    curvature = int(curvature or (msh["curvature_points"] if h == msh["screen_element_size"]
                                  else cfg["strength"]["curvature_points"]))
    res = Compliance(os.path.basename(step_path), material=mat["name"], E=mat["E"], kind=travel.kind,
                     travel=travel.amount, band=tuple(travel.band), ratio_required=travel.ratio,
                     precision=prec)
    t0 = time.perf_counter()
    from ..geometry import read_step

    try:
        solid = read_step(step_path)
        if len(solid.solids()) != 1:
            raise fe.MeshError(f"the answer is {len(solid.solids())} separate solids, not one part")
        mesh = fe.mesh_step(step_path, h, curvature, int(msh.get("element_order", 2)))
    except fe.MeshError as e:
        res.error = str(e)
        return res
    from threadpoolctl import threadpool_limits

    with threadpool_limits(1):
        try:
            return _run(res, mesh, mounts, travel, mat, cfg, step_path, process, average_radius, t0, task.get("stiffness"))
        except Exception as e:  # noqa: BLE001
            if "Newton iteration" not in str(e) and "no middle node" not in str(e):
                raise
        mesh = fe.mesh_step(step_path, h, curvature, 1)
        res.fallback = "straight elements (curved ones failed to map): round members read up to 6 % too flexible"
        return _run(res, mesh, mounts, travel, mat, cfg, step_path, process, average_radius, t0, task.get("stiffness"))


def _run(res, mesh, mounts: list[Mount], travel: Travel, mat, cfg, step_path, process, average_radius, t0, stated=None):
    from . import strength

    if len(mounts) != 2:
        res.error = f"a travel is defined between two mounts; this task has {len(mounts)}"
        return res
    held, moved = mounts
    model = fe.Model(mesh, mat["E"], mat["nu"])
    regions = {}
    for m in mounts:
        facets = model.facets_where(m.seat_pred(cfg["seat"]["plane_tolerance"], cfg["seat"]["hole_tolerance"]))
        regions[m.name] = model.region(facets, m.origin)
        if regions[m.name].area <= 0:
            res.error = f"no seat found on the answer at mount {m.name}"
            return res
    model.clamp(regions[held.name])
    reg = regions[moved.name]
    C = np.zeros((6, 6))
    U = []
    for k in range(6):
        q = np.zeros(6)
        q[k] = 1.0
        u = model.solve(reg.load(q))
        U.append(u)
        C[:, k] = reg.motion(u)
    C = (C + C.T) / 2
    span = float(np.linalg.norm(moved.origin - held.origin))
    D = np.diag([1, 1, 1, span, span, span])
    Cs = D @ C @ D
    e, per_unit = travel.vector(moved.origin, span)
    c_e = float(e @ Cs @ e)
    k_s = 1 / c_e  # scaled N/mm along the travel
    # every way at right angles to the travel
    Q = np.eye(6) - np.outer(e, e)
    lam, vec = np.linalg.eigh(Q @ Cs @ Q)
    other = 1 / lam[-1]
    p = res.precision / 100 if math.isfinite(res.precision) else 0.0
    if travel.kind == "slide":
        res.stiffness, res.unit = k_s, "N/mm"
    else:  # N mm per degree: work-conjugate to the turn angle
        res.stiffness, res.unit = k_s * per_unit ** 2 * math.pi / 180, "N mm/deg"
    res.band_verdict = _band(res.stiffness, *travel.band, p)
    res.other, res.other_description = float(other), _describe(vec[:, -1])
    res.ratio = float(other / k_s)
    r = travel.ratio
    res.ratio_verdict = "pass" if res.ratio >= r * (1 + 2 * p) else "fail" if res.ratio < r * (1 - 2 * p) else "marginal"
    resp = Cs @ e
    res.off_travel = float(np.linalg.norm(resp - c_e * e) / c_e)
    # the push for the full travel: scaled amplitude s along e
    s = travel.amount if travel.kind == "slide" else math.radians(travel.amount) * per_unit
    qs = e * (s / c_e)
    q = D @ qs  # back to N and N mm
    res.push = [round(float(v), 4) for v in q]
    u = sum(qk * uk for qk, uk in zip(q, U))
    vm = model.von_mises(u)
    case = SimpleNamespace(label="full travel", loaded=moved.name, kind="force")
    fake = SimpleNamespace(ok=True, surface=(model.mesh.p.T.copy(), model.boundary_triangles()),
                           tets=model.mesh.t.T.copy(), cases=[case], stresses={case.label: vm},
                           design_loads={moved.name: {"force_N": 1.0, "moment_Nmm": 1.0}},
                           regions={k: (np.isin(model._bf, rg.facets), k == held.name) for k, rg in regions.items()})
    st = strength.judge(fake, step_path, process, average_radius=average_radius)
    res.stress = st.to_json()
    res.stress_verdict = st.verdict
    if st.verdict in ("pass", "fail") and st.stress_ratio is not None and st.radius_ok and p > st.precision / 100:
        # a coarse mesh resolves the stress no better than its own precision
        r_ = st.stress_ratio
        res.stress_verdict = "pass" if r_ <= 1 - p else "fail" if r_ > 1 + p else "marginal"
    # STIFFNESS with the travel left out, against the task's specification
    try:
        if (stated or {}).get("mode") == "stated":  # a stated load, already in the real material
            req = {x["mount"]: x for x in stated["per_mount"]}[moved.name]
        else:  # the bolt-based specification, scaled from aluminium to this material
            req = {x["mount"]: x for x in spec_requirements(mounts, mat["E"])}[moved.name]
        j = judge(float(other), req["required"], res.precision, "")
        res.stiffness_rest = {"figure": float(other), "required": req["required"], "verdict": j.verdict,
                              "margin_percent": round(j.margin, 1), "design_load_N": req["design_load_N"],
                              "allowed_deflection_mm": req["allowed_deflection_mm"], "description": res.other_description}
    except (KeyError, AttributeError):
        res.stiffness_rest = None
    res.span = span
    res.C = [[float(x) for x in row] for row in C]
    res.ok = True
    res.seconds = round(time.perf_counter() - t0, 1)
    return res


def screened(task_dir: str, step_path: str, **kw) -> Compliance:
    """Coarse elements first (the stiffness check's screen size); the full
    analysis only when a figure falls within the coarse precision of its
    limit, as STIFFNESS does."""
    rep = analyse(task_dir, step_path, h="screen", **kw)
    if not rep.ok:
        return rep
    sr = rep.stiffness_rest["verdict"] if rep.stiffness_rest else "pass"
    if "marginal" in (rep.band_verdict, rep.ratio_verdict, rep.stress_verdict, sr):
        full = analyse(task_dir, step_path, **kw)
        full.stages = [{"stage": "screen", "seconds": rep.seconds}, {"stage": "full", "seconds": full.seconds}]
        return full
    rep.stages = [{"stage": "screen", "seconds": rep.seconds}]
    return rep
