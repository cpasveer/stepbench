"""The package's function API: what a task family wrapper would call.

    generate_folder(kind, seed, knobs, out)  -> folder path
    verify_folder(folder, step_path=None)    -> grade.Report
    make(kind, seed, knobs)                  -> (task dict, full mechanism, reference solid)
    grade_solid(task, mechanism, solid)      -> grade.Report
"""
from __future__ import annotations

import json
import os

from . import generate, grade, taskio
from .kin import Mechanism

PLANAR = ("fourbar", "slider", "yoke", "oldham")
CONTACT = ("cam", "geneva", "ratchet")


def make(kind: str, seed: int, knobs: dict | None = None):
    if kind in CONTACT:
        from . import contact
        return contact.make(kind, seed, knobs)
    return generate.make(kind, seed, knobs)


def generate_folder(kind: str, seed: int, knobs: dict | None, out: str) -> str:
    task, m, ref = make(kind, seed, knobs)
    if kind in CONTACT:
        from . import contact
        return contact.write(task, m, ref, out)
    return taskio.write(task, m, ref, out)


def verify_folder(folder: str, step_path: str | None = None, step_deg: float | None = None) -> grade.Report:
    with open(os.path.join(folder, "motion.json")) as f:
        kind = json.load(f)["type"]
    if kind in CONTACT:
        from . import contact
        return contact.grade_task(folder, step_path)
    return grade.grade_task(folder, step_path, step_deg)


def hidden_mechanism(task: dict, solids: dict | None = None) -> Mechanism:
    m = Mechanism.from_json(task["mechanism"])
    for pid, p in m.parts.items():
        if solids and pid in solids and pid != task["removed"]:
            p.solid = solids[pid]
    return m


def grade_solid(task: dict, solid_world, solids: dict | None = None) -> grade.Report:
    return grade.grade_solid(task, hidden_mechanism(task, solids), solid_world)


def show(kind: str, seed: int, knobs: dict | None, path: str) -> None:
    from . import render
    task, m, ref = make(kind, seed, knobs)
    solids = m.meta["solids"]
    for pid, p in m.parts.items():
        if pid in solids:
            p.solid = solids[pid]
    sols = None
    if kind in CONTACT:
        from . import contact, kin
        sols, _ = contact.solve_any(m, solids, kin.input_values(m, 2.0))
    render.picture(m, solids, path, removed=task["removed"],
                   title=f"{task['id']}: {m.parts[task['removed']].label} (red) removed", sols=sols)
