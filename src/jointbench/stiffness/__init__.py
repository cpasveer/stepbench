"""Stiffness score: how stiff a part is between its mounts, per unit of
material, against the task's reference answer.

A score, never a pass/fail gate. One mount is held fixed at its seat and
bolt holes; each other mount in turn gets a unit force along each of its
three axes and a unit moment about each, spread over its seat and bolt
holes as through a rigid washer; the mount's movement is read back the
same way. Stiffness per case is load / movement along the load. The figure
is the weakest case ("stiff whichever way you load it"). Linear static,
one isotropic material (``settings.toml``).

Needs the optional ``stiffness`` extra (gmsh, scikit-fem); the rest of the
benchmark runs without it.
"""
from __future__ import annotations

import math
import os
import time
import tomllib
from dataclasses import asdict, dataclass, field

import numpy as np

from . import fe
from .mounts import Mount, from_task

SETTINGS = os.path.join(os.path.dirname(__file__), "settings.toml")
AXES = ("x", "y", "z")


def settings() -> dict:
    with open(SETTINGS, "rb") as fh:
        return tomllib.load(fh)


@dataclass
class Case:
    fixed: str
    loaded: str
    kind: str  # "force" or "moment"
    axis: str  # the loaded mount's own x, y (along its face) or z (along its bolts)
    stiffness: float  # N/mm or N mm/rad
    unit: str
    movement: list  # rigid motion of the loaded mount: ux, uy, uz (mm), rx, ry, rz (rad)

    @property
    def label(self) -> str:
        return f"{'F' if self.kind == 'force' else 'M'}{self.axis} at {self.loaded}"

    def to_json(self) -> dict:
        d = asdict(self)
        d["label"] = self.label
        return d


@dataclass
class Weakest:
    """The weakest way to load a mount: the combination of force and moment
    that moves it most per unit of load. Moments are put on the force scale
    through the span L between the held and the loaded mount (a moment M
    counts as a force M / L, a rotation r as a movement r L), so the figure
    is in N/mm and does not depend on how the axes were chosen."""

    mount: str
    stiffness: float  # N/mm
    span: float  # mm, the lever L
    direction: list  # unit (Fx, Fy, Fz, Mx/L, My/L, Mz/L) in the mount's axes
    description: str


@dataclass
class Par:
    file: str
    volume: float
    ratios: dict  # case label -> stiffness / reference stiffness
    weakest: str  # the answer's weakest way of loading, described
    stiffness: float  # weakest stiffness / the reference's weakest stiffness
    material: float  # volume / reference volume
    specific: float  # stiffness / material: stiffness per unit material, against the reference
    efficiency: float  # stiffness / material**2: see docs/stiffness.md


@dataclass
class Requirement:
    """Stiffness as a requirement: the weakest-way figure must reach
    ``required``. The figure is only known to +/- ``precision`` percent, so
    a verdict is given only when the gap is larger than that: "pass" above
    required x (1 + p), "fail" below required x (1 - p), "marginal" in
    between (counted as a pass, flagged)."""

    required: float  # N/mm
    source: str  # where the number comes from
    fraction: float | None  # of the reference's figure, when derived that way
    precision: float  # percent
    verdict: str = ""
    margin: float = 0.0  # figure / required - 1, percent
    per_mount: list = field(default_factory=list)  # spec mode: every loaded mount's requirement and verdict

    @property
    def passed(self) -> bool:
        return self.verdict in ("pass", "marginal")


def judge(figure: float, required: float, precision: float, source: str, fraction=None) -> Requirement:
    p = precision / 100
    v = "pass" if figure >= required * (1 + p) else "fail" if figure < required * (1 - p) else "marginal"
    return Requirement(required, source, fraction, precision, v, 100 * (figure / required - 1))


def requirement_for(task: dict, ref: "Report | None", fraction: float | None, precision: float):
    """The requirement for a task: a stated minimum in ``task.json``
    (``"stiffness": {"min": N/mm}``), else a stated or given fraction of the
    reference answer's figure (``"stiffness": {"fraction": 0.5}``). A
    fraction of the reference keeps every task solvable by construction:
    the reference itself passes with room to spare."""
    st = task.get("stiffness") or {}
    if "min" in st:
        return float(st["min"]), "stated in task.json", None
    f = fraction if fraction is not None else st.get("fraction", settings()["requirement"]["fraction"])
    if ref is None or not ref.ok:
        return None
    return f * ref.figure.stiffness, f"{f:g} x the reference answer's figure", f


def material_for(process: str | None) -> dict:
    """The material the analysis uses: the process's material
    (``[materials.<process>]``) when a process is given, else the reference
    material (``[material]``, aluminium) the requirement was calibrated in."""
    cfg = settings()
    if process:
        m = cfg["materials"][process]
        return {"name": m["name"], "E": m["E"], "nu": m["nu"], "density": m["density"]}
    return {k: cfg["material"][k] for k in ("name", "E", "nu", "density")}


def deflection_ratio(E: float) -> float:
    """span / allowed deflection for a part of modulus E. The requirement was
    calibrated with span/250 in the reference material; required stiffness
    scales with the modulus, so a material E times as stiff is allowed
    E_ref / E times the deflection -- every verdict stays as calibrated
    (only Poisson's ratio differs, which moves figures by under 1 %)."""
    cfg = settings()
    return cfg["requirement"]["deflection_ratio"] * E / cfg["material"]["E"]


def spec_requirements(mounts: list[Mount], E: float | None = None) -> list[dict]:
    """The task-only requirement at each loaded mount (mounts[0] is held):
    design load F = working stress x the bolts' tensile stress area at the
    weaker of the two mounts; allowed deflection = span / ratio; required
    stiffness = F / allowed deflection, in N/mm, for the weakest way to load
    that mount (forces and moments, as the figure). Every number comes from
    task.json: bolt size and count per mount, and where the mounts are."""
    cfg = settings()["requirement"]
    areas = cfg["stress_area"]
    ratio = deflection_ratio(E) if E else cfg["deflection_ratio"]
    held = mounts[0]
    out = []
    for m in mounts[1:]:
        def area(x):
            return x.n_bolts * areas[x.bolt_size.replace(".", "_")] if x.n_bolts else 0.0
        # a held bearing has no bolts: it is loaded with what the held mount's bolts carry
        a = area(held) if m.kind == "bearing" else min(area(held), area(m))
        span = float(np.linalg.norm(m.origin - held.origin))
        load = cfg["working_stress"] * a
        allowed = span / ratio
        out.append({"mount": m.name, "bolts": (f"{held.n_bolts} x {held.bolt_size} / bearing" if m.kind == "bearing"
                                               else f"{held.n_bolts} x {held.bolt_size} / {m.n_bolts} x {m.bolt_size}"),
                    "bolt_area_mm2": round(a, 2), "span_mm": round(span, 2), "design_load_N": round(load, 1),
                    "deflection_ratio": round(ratio, 2),
                    "allowed_deflection_mm": round(allowed, 4), "required": load / allowed})
    return out


def design_loads(mounts: list[Mount]) -> dict:
    """The loads the stress pictures are drawn for, per loaded mount: the
    specification's design force (as in the requirement), and as moment
    that force times the span. Without bolt data, 1 N and 1 N x span."""
    out = {}
    try:
        for r in spec_requirements(mounts):
            out[r["mount"]] = {"force_N": r["design_load_N"], "moment_Nmm": round(r["design_load_N"] * r["span_mm"], 1),
                               "span_mm": r["span_mm"], "basis": "design load of the specification"}
    except (KeyError, AttributeError):
        held = mounts[0]
        for m in mounts[1:]:
            span = round(float(np.linalg.norm(m.origin - held.origin)), 2)
            out[m.name] = {"force_N": 1.0, "moment_Nmm": span, "span_mm": span, "basis": "unit force (no bolt data)"}
    return out


def stated_requirements(mounts: list[Mount], loads: dict, ratio: float | None = None) -> list[dict]:
    """A stated load at each loaded mount: ``loads`` = {mount: mass in kg}.
    The weight (g = 9.81) may move that mount at most span / ``ratio``
    (default the settings' deflection ratio), in whichever direction the
    part is weakest. Judged in the process's real material."""
    cfg = settings()["requirement"]
    ratio = ratio or cfg["deflection_ratio"]
    held = mounts[0]
    out = []
    for m in mounts[1:]:
        if m.name not in loads:
            continue
        span = float(np.linalg.norm(m.origin - held.origin))
        load = float(loads[m.name]) * 9.81
        allowed = span / ratio
        out.append({"mount": m.name, "mass_kg": float(loads[m.name]), "design_load_N": round(load, 2),
                    "span_mm": round(span, 2), "deflection_ratio": ratio,
                    "allowed_deflection_mm": round(allowed, 4), "required": load / allowed, "bolts": "stated load"})
    return out


def judge_stated(rep: "Report", stated: dict) -> "Requirement":
    """Judge each loaded mount against its stated load; the worst decides."""
    reqs = {r["mount"]: r for r in stated["per_mount"]}
    per = []
    for w in rep.per_mount:
        if w.mount not in reqs:
            continue
        r = reqs[w.mount]
        j = judge(w.stiffness, r["required"], rep.precision,
                  f"mount {w.mount}: {r['mass_kg']:g} kg ({r['design_load_N']:g} N) over at most "
                  f"{r['allowed_deflection_mm']:.3g} mm (span {r['span_mm']:.1f} mm / {r['deflection_ratio']:g}; {rep.material})")
        per.append((j, {**r, "figure": w.stiffness, "verdict": j.verdict, "margin_percent": round(j.margin, 1)}))
    rank = {"fail": 0, "marginal": 1, "pass": 2}
    worst = min(per, key=lambda t: (rank[t[0].verdict], t[0].margin))[0]
    worst.per_mount = [d for _, d in per]
    return worst


def judge_spec(rep: "Report", mounts: list[Mount]) -> "Requirement":
    """Judge every loaded mount against its own requirement; the verdict is
    the worst of them (fail beats marginal beats pass)."""
    reqs = {r["mount"]: r for r in spec_requirements(mounts, rep.E)}
    per = []
    for w in rep.per_mount:
        r = reqs[w.mount]
        j = judge(w.stiffness, r["required"], rep.precision,
                  f"mount {w.mount}: {r['design_load_N']:g} N over {r['allowed_deflection_mm']:.3g} mm "
                  f"(span {r['span_mm']:.1f} mm / {r['deflection_ratio']:g}; bolts {r['bolts']}; {rep.material})")
        per.append((j, {**r, "figure": w.stiffness, "verdict": j.verdict, "margin_percent": round(j.margin, 1)}))
    rank = {"fail": 0, "marginal": 1, "pass": 2}
    worst = min(per, key=lambda t: (rank[t[0].verdict], t[0].margin))[0]
    worst.per_mount = [d for _, d in per]
    return worst


@dataclass
class Report:
    file: str
    task: str
    ok: bool = False
    error: str = ""
    fallback: str = ""  # set when the analysis had to fall back from curved to straight elements
    quality: dict | None = None  # the quality checker's verdict, given when meshing fails
    material: str = ""
    E: float = 0.0
    nu: float = 0.0
    element_size: float = 0.0
    precision: float = 0.0  # percent, from the convergence study
    nodes: int = 0
    elements: int = 0
    curved: bool = False  # edges follow the curved surfaces (second-order geometry)
    straightened: int = 0  # curved elements made straight again because bending them folded them
    dofs: int = 0
    volume: float = 0.0  # cm3, exact (from the solid)
    mesh_volume: float = 0.0  # cm3, of the meshed solid
    mass: float = 0.0  # g
    seats: dict = field(default_factory=dict)  # mount -> seat area found, mm2
    cases: list = field(default_factory=list)
    figure: Weakest | None = None  # THE figure: the weakest way to load any mount
    per_mount: list = field(default_factory=list)  # the weakest way at each loaded mount
    symmetry: float = 0.0  # largest relative asymmetry of a flexibility matrix
    stages: list = field(default_factory=list)  # requirement screening: what ran, how long, what it said
    seconds: float = 0.0
    timing: dict = field(default_factory=dict)
    par: Par | None = None
    requirement: Requirement | None = None
    # kept for pictures and fields, not written to JSON
    surface: object = None
    displacements: dict = field(default_factory=dict)
    stresses: dict = field(default_factory=dict)  # von Mises per mesh corner, MPa per unit load (for pictures only)
    design_loads: dict = field(default_factory=dict)  # mount -> design force (N) and moment (N mm), for stress pictures
    regions: dict = field(default_factory=dict)

    def weakest(self, kind: str) -> Case | None:
        c = [x for x in self.cases if x.kind == kind]
        return min(c, key=lambda x: x.stiffness) if c else None

    def text(self) -> str:
        if not self.ok:
            out = [f"{self.file}: NO FIGURE -- {self.error}"]
            if self.quality:
                q = self.quality
                out.append(f"  modelling quality: {'CLEAN' if q['passed'] else 'UNTIDY'}"
                           + (f" ({', '.join(q['failed'])})" if q["failed"] else ""))
            return "\n".join(out)
        p = f"+/- {self.precision:g} %"
        out = [f"{self.file}: stiffness (score, not a gate)",
               f"  material {self.material}, E {self.E:g} MPa; {self.elements} quadratic tetrahedra "
               f"of at most {self.element_size:g} mm, {self.dofs} unknowns; {self.seconds:.1f} s",
               f"  volume {self.volume:.2f} cm3 ({self.mass:.1f} g)",
               "  seats found: " + ", ".join(f"{k} {v:.0f} mm2" for k, v in self.seats.items())]
        w = self.figure
        out.append(f"  STIFFNESS, weakest way to load it: {w.stiffness:.4g} N/mm ({p}), at mount {w.mount}: "
                   f"{w.description} (moments over the {w.span:.1f} mm span)")
        out.append(f"  per unit material: {w.stiffness / self.volume:.4g} N/mm per cm3")
        out.append("  every case:")
        for c in self.cases:
            r = f"   x{self.par.ratios[c.label]:.3g} of reference" if self.par else ""
            out.append(f"    held {c.fixed}, {c.label:<10} {c.stiffness:12.4g} {c.unit}{r}")
        for st in self.stages:
            out.append(f"  stage: {st['stage']}: {st['seconds']} s" + (f", {st.get('verdict')}" if st.get("verdict") else ""))
        if self.requirement:
            q = self.requirement
            if q.per_mount:
                out.append(f"  REQUIREMENT (task specification): {q.verdict.upper()}")
                for d in q.per_mount:
                    out.append(f"    mount {d['mount']}: needs {d['required']:.4g} N/mm ({d['design_load_N']:g} N, "
                               f"deflection at most {d['allowed_deflection_mm']:.3g} mm), has {d['figure']:.4g}: "
                               f"{d['verdict']} ({d['margin_percent']:+.1f} %)")
            else:
                out.append(f"  REQUIREMENT: at least {q.required:.4g} N/mm ({q.source}): {q.verdict.upper()} "
                           f"({q.margin:+.1f} %, precision +/- {q.precision:g} %)")
        if self.par:
            q = self.par
            out += [f"  against the reference ({q.file}, {q.volume:.2f} cm3):",
                    f"    stiffness, weakest way to load it: x{q.stiffness:.3f}",
                    f"    material: x{q.material:.3f}",
                    f"    SPECIFIC STIFFNESS (stiffness / material): x{q.specific:.3f}",
                    f"    shape efficiency (stiffness / material^2): x{q.efficiency:.3f}"]
        return "\n".join(out)

    def to_json(self) -> dict:
        d = {k: v for k, v in asdict(self).items()
             if k not in ("surface", "displacements", "stresses", "regions", "cases", "par", "figure", "requirement",
                          "per_mount")}
        d["per_mount"] = [asdict(w) for w in self.per_mount]
        d["figure"] = asdict(self.figure) if self.figure else None
        d["units"] = {"figure": "N/mm", "force": "N/mm", "moment": "N mm/rad", "volume": "cm3", "mass": "g",
                      "movement": "mm and rad per unit load", "precision": "percent"}
        d["cases"] = [c.to_json() for c in self.cases]
        if self.cases:
            d["weakest_force"] = self.weakest("force").to_json()
            d["weakest_moment"] = self.weakest("moment").to_json()
        d["par"] = asdict(self.par) if self.par else None
        d["requirement"] = asdict(self.requirement) if self.requirement else None
        if self.requirement:
            d["requirement"]["passed"] = self.requirement.passed
        return d


def analyse(step_path: str, mounts: list[Mount], *, h: float | None = None, keep_fields=True,
            task: str = "", quality_on_failure=True, curvature: int | None = None, order: int | None = None,
            process: str | None = None) -> Report:
    """The stiffness of the solid in ``step_path`` between ``mounts``: the
    first is held, every other is loaded in turn."""
    need = fe.available()
    if need:
        raise RuntimeError(need)
    cfg = settings()
    mat, msh, seat = material_for(process), cfg["mesh"], cfg["seat"]
    if h == "quick":
        h, prec = msh["quick_element_size"], msh["quick_precision"]
    elif h == "screen":
        h, prec = msh["screen_element_size"], msh["screen_precision"]
    elif h:
        prec = msh["precision"] if h <= msh["element_size"] else math.nan
    else:
        h, prec = msh["element_size"], msh["precision"]
    rep = Report(os.path.basename(step_path), task, material=mat["name"], E=mat["E"], nu=mat["nu"],
                 element_size=h, precision=prec)
    t0 = time.perf_counter()
    from ..geometry import read_step

    try:
        solid = read_step(step_path)
        rep.volume = solid.volume / 1000
        nsol = len(solid.solids())
        if nsol != 1:
            raise fe.MeshError(f"the answer is {nsol} separate solids, not one part")
        mesh = fe.mesh_step(step_path, h, int(curvature or msh["curvature_points"]), int(order or msh.get("element_order", 2)))
    except fe.MeshError as e:
        rep.error = str(e)
        rep.seconds = time.perf_counter() - t0
        if quality_on_failure:
            rep.quality = _quality(step_path)
        return rep
    rep.timing["mesh"] = time.perf_counter() - t0
    rep.mass = rep.volume * mat["density"]
    rep.mesh_volume = fe.tet_volume(mesh) / 1000
    from threadpoolctl import threadpool_limits

    with threadpool_limits(1):
        try:
            return _analyse(rep, mesh, mounts, cfg, keep_fields, t0)
        except Exception as e:  # noqa: BLE001
            # curved elements can fail to map on a badly shaped part ("Newton
            # iteration didn't converge"); straight elements always map. They read
            # round members up to 6 % too flexible: against the part, never for it
            if "Newton iteration" not in str(e) or int(order or msh.get("element_order", 2)) == 1:
                raise
        mesh = fe.mesh_step(step_path, h, int(curvature or msh["curvature_points"]), 1)
        rep.fallback = "straight elements (curved ones failed to map): round members read up to 6 % too flexible"
        return _analyse(rep, mesh, mounts, cfg, keep_fields, t0)


def _analyse(rep, mesh, mounts, cfg, keep_fields, t0):
    mat, seat = {"E": rep.E, "nu": rep.nu}, cfg["seat"]
    t1 = time.perf_counter()
    model = fe.Model(mesh, mat["E"], mat["nu"])
    rep.curved, rep.straightened = model.curved, model.straightened
    rep.timing["assemble"] = time.perf_counter() - t1
    rep.nodes, rep.elements, rep.dofs = int(model.sbasis.N), len(mesh.t), int(model.K.shape[0])

    regions = {}
    for m in mounts:
        facets = model.facets_where(m.seat_pred(seat["plane_tolerance"], seat["hole_tolerance"]))
        reg = model.region(facets, m.origin)
        rep.seats[m.name] = round(reg.area, 1)
        regions[m.name] = reg
        if reg.area <= 0:
            rep.error = f"no seat found on the answer at mount {m.name}"
            rep.seconds = time.perf_counter() - t0
            return rep
    t2 = time.perf_counter()
    held = mounts[0]
    model.clamp(regions[held.name])
    rep.timing["factor"] = time.perf_counter() - t2
    t3 = time.perf_counter()
    asym = 0.0
    weak = []
    for m in mounts[1:]:
        reg = regions[m.name]
        T = np.zeros((6, 6))  # loads and readings in the mount's own axes
        T[:3, :3] = T[3:, 3:] = np.stack([m.x, m.y, m.n])
        C = np.zeros((6, 6))
        # a bearing passes forces from its shaft, not moments: it turns, and a
        # single row of balls tilts rather than carry a moment; forces only there
        for k in (range(3) if m.kind == "bearing" else range(6)):
            q = np.zeros(6)
            q[k] = cfg["loads"]["force"] if k < 3 else cfg["loads"]["moment"]
            u = model.solve(reg.load(T.T @ q))
            mov = T @ reg.motion(u)
            C[:, k] = mov / q[k]
            kind = "force" if k < 3 else "moment"
            rep.cases.append(fe_case(held.name, m.name, kind, AXES[k % 3], C[k, k], mov / q[k]))
            if keep_fields:
                rep.displacements[rep.cases[-1].label] = model.nodal(u) / q[k]
                rep.stresses[rep.cases[-1].label] = model.von_mises(u) / q[k]
        asym = max(asym, float(np.abs(C - C.T).max() / np.abs(np.diag(C)).max()))
        span = float(np.linalg.norm(m.origin - held.origin))
        weak.append(_weakest_force(m.name, C, span) if m.kind == "bearing" else _weakest(m.name, C, span))
    rep.symmetry = asym
    rep.figure = min(weak, key=lambda w: w.stiffness)
    rep.per_mount = weak
    rep.timing["solve"] = time.perf_counter() - t3
    if keep_fields:
        rep.design_loads = design_loads(mounts)
        rep.surface = (model.mesh.p.T.copy(), model.boundary_triangles())
        rep.tets = model.mesh.t.T.copy()
        rep.regions = {k: (np.isin(model._bf, r.facets), k == held.name) for k, r in regions.items()}
    rep.ok = True
    rep.seconds = time.perf_counter() - t0
    return rep


def _weakest(name: str, C: np.ndarray, span: float) -> Weakest:
    D = np.diag([1, 1, 1, span, span, span])
    Cs = D @ ((C + C.T) / 2) @ D
    lam, vec = np.linalg.eigh(Cs)
    v = vec[:, -1]
    v = v * np.sign(v[np.argmax(np.abs(v))])
    names = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
    parts = [f"{names[i]} {v[i] ** 2 * 100:.0f} %" for i in np.argsort(-np.abs(v)) if v[i] ** 2 >= 0.05]
    return Weakest(name, float(1 / lam[-1]), span, [round(float(a), 4) for a in v], ", ".join(parts))


def _weakest_force(name: str, C: np.ndarray, span: float) -> Weakest:
    """The weakest direction of a force at a bearing (forces only)."""
    Cs = (C[:3, :3] + C[:3, :3].T) / 2
    lam, vec = np.linalg.eigh(Cs)
    v = vec[:, -1]
    v = v * np.sign(v[np.argmax(np.abs(v))])
    names = ["Fx", "Fy", "Fz"]
    parts = [f"{names[i]} {v[i] ** 2 * 100:.0f} %" for i in np.argsort(-np.abs(v)) if v[i] ** 2 >= 0.05]
    return Weakest(name, float(1 / lam[-1]), span, [round(float(a), 4) for a in v] + [0.0, 0.0, 0.0], ", ".join(parts))


def fe_case(fixed, loaded, kind, axis, flex, mov) -> Case:
    unit = "N/mm" if kind == "force" else "N mm/rad"
    k = 1.0 / flex if flex > 0 else math.inf
    return Case(fixed, loaded, kind, axis, float(k), unit, [float(v) for v in mov])


def _quality(step_path: str) -> dict | None:
    try:
        from ..quality import check as qcheck

        q = qcheck(step_path)
        return {"passed": q.passed, "failed": q.failed()}
    except Exception as e:  # noqa: BLE001
        return {"passed": False, "failed": [f"quality check failed too: {e}"]}


def compare(rep: Report, ref: Report) -> Par | None:
    if not (rep.ok and ref.ok):
        return None
    refk = {c.label: c.stiffness for c in ref.cases}
    ratios = {c.label: round(c.stiffness / refk[c.label], 4) for c in rep.cases}
    s, m = rep.figure.stiffness / ref.figure.stiffness, rep.volume / ref.volume
    return Par(ref.file, ref.volume, ratios, f"at {rep.figure.mount}: {rep.figure.description}", s, m, s / m, s / m**2)


def find_answer(task_dir: str) -> str:
    for p in ("output.step", os.path.join("_solution", "reference.step")):
        if os.path.exists(os.path.join(task_dir, p)):
            return os.path.join(task_dir, p)
    raise FileNotFoundError(f"no output.step in {task_dir}; give one with --file")


def check(task_dir: str, file: str | None = None, reference: str | None = None, *, par: bool = True,
          h: float | None = None, ref_report: Report | None = None, require: float | None = None,
          fraction: float | None = None, process: str | None = None) -> Report:
    """Score ``file`` (default ``<task>/output.step``) on the task in
    ``task_dir``, against ``reference`` (default
    ``<task>/_solution/reference.step``, if there)."""
    task, mounts = from_task(task_dir)
    path = file or find_answer(task_dir)
    rep = analyse(path, mounts, h=h, task=task.get("id", ""), process=process)
    if par and rep.ok:
        ref = reference or os.path.join(task_dir, "_solution", "reference.step")
        if ref_report is None and os.path.exists(ref):
            ref_report = (rep if os.path.abspath(ref) == os.path.abspath(path)
                          else analyse(ref, mounts, h=h, keep_fields=False, task=rep.task, process=process))
        if ref_report is not None:
            rep.par = compare(rep, ref_report)
    if rep.ok:
        st = task.get("stiffness") or {}
        if require:
            rep.requirement = judge(rep.figure.stiffness, require, rep.precision, "given on the command line")
        elif fraction is None and "min" not in st and "fraction" not in st and settings()["requirement"]["mode"] == "spec":
            rep.requirement = judge_spec(rep, mounts)
        else:
            req = requirement_for(task, ref_report, fraction, rep.precision)
            if req:
                rep.requirement = judge(rep.figure.stiffness, req[0], rep.precision, req[1], req[2])
    return rep


def screened(task_dir: str, file: str | None = None, *, require: float | None = None,
             fraction: float | None = None, reference: str | None = None, process: str | None = None) -> Report:
    """Requirement mode, cheapest first. The requirement comes from
    ``require``, else ``task.json`` (``"stiffness": {"min": ...}`` or
    ``{"fraction": ...}``), else the task-only specification (settings,
    ``mode = "spec"``: needs no reference at all), else a fraction of the
    reference answer's figure (the reference is then analysed in full). The answer is first analysed with coarse elements
    (``screen_element_size``); if its figure is clear of the requirement by
    more than that size's claimed precision, that verdict stands. Only a
    part inside the band gets the full analysis."""
    task, mounts = from_task(task_dir)
    path = file or find_answer(task_dir)
    cfg = settings()["mesh"]
    stages = []
    st = task.get("stiffness") or {}
    stated = st if st.get("mode") == "stated" else None
    if stated:  # a stated load is judged in the stated process's real material
        process = process or ((task.get("tolerances") or {}).get("manufacturing") or {}).get("process")
    spec = (not stated and not require and fraction is None and "min" not in st and "fraction" not in st
            and settings()["requirement"]["mode"] == "spec")
    req = None
    if stated:
        pass
    elif require:
        req = (require, "given on the command line", None)
    elif not spec:
        ref_rep = None
        if "min" not in st:
            ref = reference or os.path.join(task_dir, "_solution", "reference.step")
            if not os.path.exists(ref):
                raise FileNotFoundError("no requirement: give --require, state it in task.json, or provide the reference")
            ref_rep = analyse(ref, mounts, keep_fields=False, task=task.get("id", ""), process=process)
            stages.append({"stage": "reference, full", "seconds": round(ref_rep.seconds, 1),
                           "figure": ref_rep.figure.stiffness if ref_rep.ok else None})
        req = requirement_for(task, ref_rep, fraction, cfg["precision"])
        if req is None:
            raise RuntimeError("the reference answer could not be analysed, so there is no requirement")

    def verdict(r):
        if stated:
            return judge_stated(r, stated)
        return judge_spec(r, mounts) if spec else judge(r.figure.stiffness, req[0], r.precision, req[1], req[2])

    rep = analyse(path, mounts, h="screen", keep_fields=False, task=task.get("id", ""), process=process)
    if rep.ok:
        rep.requirement = verdict(rep)
    stages.append({"stage": f"screen, {cfg['screen_element_size']:g} mm", "seconds": round(rep.seconds, 1),
                   "figure": rep.figure.stiffness if rep.ok else None,
                   "verdict": rep.requirement.verdict if rep.requirement else None})
    if rep.ok and rep.requirement.verdict == "marginal":
        rep = analyse(path, mounts, keep_fields=False, task=task.get("id", ""), process=process)
        if rep.ok:
            rep.requirement = verdict(rep)
        stages.append({"stage": f"full, {cfg['element_size']:g} mm", "seconds": round(rep.seconds, 1),
                       "figure": rep.figure.stiffness if rep.ok else None,
                       "verdict": rep.requirement.verdict if rep.requirement else None})
    rep.stages = stages
    return rep


def write_fields(rep: Report, out_dir: str) -> str:
    from .viz import write_fields as wf

    return wf(rep, out_dir)


def write_images(rep: Report, out_dir: str) -> list[str]:
    from .viz import write_images as wi

    return wi(rep, out_dir)
