"""Where the task's own numbers come from: the stiffness requirement and par.

Each is decided in exactly one function here, so changing how it is
derived -- for example a stiffness computed from the mounts' bolts and the
span instead of a fraction of the reference answer -- is a one-place change.
Generators call these; the numbers are written into ``task.json`` and the
brief; the grader only reads them.
"""
from __future__ import annotations

import os
import tempfile

STIFFNESS_RULE = "from the task alone: bolt-area design load over span / 250 (the stiffness checker's spec mode)"
PAR_RULE = "volume of the leanest generator-built answer that meets every required check, stiffness included"


def stiffness_requirement(task, reference=None, fraction: float | None = None) -> dict | None:
    """The stiffness the answer must reach, written into task.json, or None.

    Current rule, from the task alone (the stiffness checker's spec mode):
    at each loaded mount, design load = working stress x the bolts' stress
    area at the weaker of the two mounts, allowed deflection = span / 250,
    required stiffness = load / deflection, N/mm. The reference merely has
    to meet it like any answer; the generator checks that it does.
    ``reference`` and ``fraction`` are unused under this rule."""
    from .stiffness import settings as st_settings, spec_requirements
    from .stiffness.mounts import from_task

    with tempfile.TemporaryDirectory() as tmp:
        with open(os.path.join(tmp, "task.json"), "w", encoding="utf-8") as f:
            f.write(task.dumps())
        _, mounts = from_task(tmp)
    per = spec_requirements(mounts)
    cfg = st_settings()["requirement"]
    held = mounts[0].name
    for m in per:  # the requirement in plain words, as the brief states it
        m["statement"] = (
            f"Mount {held} is held at its seat and bolt holes. A load of {m['design_load_N']:g} N on mount "
            f"{m['mount']}, through its seat and bolt holes and in whichever direction the part is weakest "
            f"(a moment counts as a force at the {m['span_mm']:g} mm span between the mounts), may move "
            f"mount {m['mount']} by at most {m['allowed_deflection_mm']:.3g} mm: at least "
            f"{m['required']:.1f} N/mm.")
    return {"mode": "spec", "held": held, "per_mount": per,
            "precision": st_settings()["mesh"]["precision"],
            "how": (f"design load {cfg['working_stress']:g} N per mm2 of bolt stress area at the weaker mount, "
                    f"deflection at most span / {cfg['deflection_ratio']:g}")}


def par(task, draw, build) -> dict | None:
    """The par volume for the material score, or None. Current rule: the
    volume of a lean answer the generator builds (see ``jointbench.par``)."""
    from .par import compute

    return compute(task, draw, build)


def apply_stiffness(task) -> None:
    """Make STIFFNESS a requirement of the task and write its figures into
    task.json. From the task alone, so it is known before anything is
    graded; the reference must meet it like any answer, or the draw is
    rejected."""
    req = list(task.tolerances.get("required_checks", []))
    if "STIFFNESS" not in req:
        req.append("STIFFNESS")
    task.tolerances["required_checks"] = req
    task.stiffness = stiffness_requirement(task)


def apply_key_turn(task) -> None:
    """Make KEY_TURN a requirement: every bolt must be turnable with its
    standard L-shaped hex key (a 60 degree swing, leaning at most 25
    degrees), not just reachable. Required for every task made from 14
    November 2026 on; older task files do not list it and keep their
    verdicts."""
    req = list(task.tolerances.get("required_checks", []))
    if "KEY_TURN" not in req:
        req.append("KEY_TURN")
    task.tolerances["required_checks"] = req


PROCESSES = ("pbf", "fdm")  # processes references can be built for (moulding, milling later)
PROCESS_DEFAULT = "pbf"


def apply_process(task, process: str) -> None:
    """State the manufacturing process on the task and make its rules a
    requirement: a part that cannot be made fails, at every level."""
    if process not in PROCESSES:
        raise ValueError(f"process must be one of {', '.join(PROCESSES)} (references cannot be built for others yet)")
    task.tolerances["manufacturing"] = dict(task.tolerances.get("manufacturing", {}), process=process)
    req = list(task.tolerances.get("required_checks", []))
    if "MANUFACTURING" not in req:
        req.append("MANUFACTURING")
    task.tolerances["required_checks"] = req
    task.meta = dict(task.meta, knobs=dict(task.meta.get("knobs", {}), process=process))


STATED_LOAD_KG = (0.5, 1.5)  # the mass a loaded fixture carries, drawn per task (maintainer, 13 November)


def draw_stated_load(rng) -> float:
    """A mass in kg from ``STATED_LOAD_KG``, in 0.1 kg steps, from the task's
    own seeded random stream."""
    lo, hi = STATED_LOAD_KG
    return round(lo + 0.1 * rng.randrange(int(round((hi - lo) / 0.1)) + 1), 1)


def apply_stated_load(task, loads: dict, ratio: float | None = None) -> None:
    """Make STIFFNESS a requirement from a stated load: ``loads`` = {mount:
    mass in kg}, analysed in the task's process's real material (PLA for
    filament, PA12 for powder), the mount allowed to move span / ``ratio``."""
    from .stiffness import settings as st_settings, stated_requirements
    from .stiffness.mounts import from_task

    req = list(task.tolerances.get("required_checks", []))
    if "STIFFNESS" not in req:
        req.append("STIFFNESS")
    task.tolerances["required_checks"] = req
    with tempfile.TemporaryDirectory() as tmp:
        with open(os.path.join(tmp, "task.json"), "w", encoding="utf-8") as f:
            f.write(task.dumps())
        _, mounts = from_task(tmp)
    per = stated_requirements(mounts, loads, ratio)
    held = mounts[0].name
    proc = (task.tolerances.get("manufacturing") or {}).get("process")
    mats = st_settings().get("materials", {})
    mat = mats[proc]["name"] if proc in mats else "the process's material"
    for m in per:
        m["statement"] = (
            f"Fixed part {m['mount']} carries {m['mass_kg']:g} kg ({m['design_load_N']:g} N). With {held} held, that "
            f"weight may move {m['mount']} by at most {m['allowed_deflection_mm']:.3g} mm, whichever way it pulls "
            f"(a turning load counts as the weight at the {m['span_mm']:g} mm span): at least {m['required']:.1f} N/mm, "
            f"in {mat}.")
    task.stiffness = {"mode": "stated", "held": held, "per_mount": per,
                      "precision": st_settings()["mesh"]["precision"],
                      "how": (f"a stated load, the movement at most span / {per[0]['deflection_ratio']:g}, in {mat}"
                              if per else "")}
