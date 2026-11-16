"""Task families: the boundary new kinds of task are built against.

A family is a sub-package of ``jointbench.families`` that defines a module
attribute ``FAMILY = Family(...)``. It is found by scanning this folder, so
adding a family needs no edit to any core file: ``gen``, ``verify``,
``selftest``, ``solve``, ``baselines``, ``freeze``, the attempt archive and
``traces`` all look families up here. See ``docs/families.md``.

The connecting-part levels are the first family, ``connect``. Task files
without a ``family`` field belong to it, so every task generated before
families existed still loads and grades unchanged.
"""
from __future__ import annotations

import importlib
import pkgutil
from dataclasses import dataclass, field
from typing import Callable

DEFAULT = "connect"


@dataclass
class Family:
    name: str
    title: str
    # level number -> level module. A level module provides:
    #   LEVEL, SITUATION (brief text), generate(seed[, knobs]) -> (task, draw, reference),
    #   build_connector(task, draw, **variant) -> solid, SELFTEST_VARIANTS,
    #   and optionally KNOBS_DEFAULT
    levels: dict
    # extra checks: [(check module, first level)]; each module has NAME, run(ctx)
    # and optionally GROUP ("fits", "clean", "makeable", "sturdy") and OPTIONAL
    checks: list = field(default_factory=list)
    # name -> function(task) -> solid: scripted answers, the difficulty floor
    baselines: dict = field(default_factory=dict)
    # motion: function(task, ctx, pose) -> Placement of the answer at that pose,
    # or None if the answer cannot reach it. None here: the answer stays put.
    pose_answer: Callable | None = None
    # motion: function(task, ctx, k) -> {part: Placement} placing the fixed
    # parts at check pose k as the answer actually drives them (overrides
    # task.json's stored placements; also turns on fixed-against-fixed checks)
    pose_scene: Callable | None = None
    # motion: function(task) -> int, how many poses the checks run at (may be
    # finer than the poses stored in task.json); None = the stored poses
    check_poses: Callable | None = None
    # names of the core checks that apply (FILE_VALID always does); None =
    # all of them. A family without bolts leaves out HOLES and SEATING.
    core_checks: set | None = None
    # function(task) -> str: the family's own brief (README.md of a task);
    # None = the connecting-part brief
    brief: Callable | None = None
    # function(task) -> str: the family's submission.py starting point;
    # None = the connecting-part stub
    submission: Callable | None = None
    # function(task) -> dict: regenerate the grader-only data (see
    # grader_data) from the task's seed and knobs, when _solution is absent
    grader: Callable | None = None
    # function(task, step_path, **grade options) -> Report: the family grades
    # an answer its own way (the adapt family: one part placed into several
    # arrangements, each graded by the core grader); None = the core grader
    grade: Callable | None = None
    doc: str = ""


_REGISTRY: dict | None = None


def registry() -> dict[str, Family]:
    global _REGISTRY
    if _REGISTRY is None:
        found = {}
        for m in sorted(pkgutil.iter_modules(__path__), key=lambda m: m.name):
            if not m.ispkg:
                continue
            mod = importlib.import_module(f"{__name__}.{m.name}")
            fam = getattr(mod, "FAMILY", None)
            if isinstance(fam, Family):
                if fam.name in found:
                    raise ValueError(f"two families named {fam.name!r}")
                found[fam.name] = fam
        _REGISTRY = found
    return _REGISTRY


def grader_data(task) -> dict | None:
    """Data the grader needs but the agent must not see (a mechanism's
    exact table, hidden tolerances). Convention: a family returns it in its
    draw as ``draw["grader"]``; ``write_workspace`` stores the draw in
    ``_solution/reference.json``, which travels with the grader and never
    with the agent (it is part of a frozen set's fingerprint; an agent's
    copy of a task is the folder without ``_solution``). Looked up: in
    memory on the task; else the family's ``grader`` hook, which rebuilds
    it deterministically from seed and knobs and so cannot be loosened by
    editing a file; only a family without that hook reads the file."""
    import json
    import os

    cached = getattr(task, "_grader", None)
    if cached is not None:
        return cached
    data = None
    fam = of(task)
    if fam.grader is not None:
        data = fam.grader(task)
    elif task.root:
        p = os.path.join(task.root, "_solution", "reference.json")
        if os.path.exists(p):
            with open(p, encoding="utf-8") as f:
                data = (json.load(f).get("draw") or {}).get("grader")
    task._grader = data
    return data


def register(fam: Family) -> None:
    """Add a family that does not live in this folder (tests, experiments)."""
    reg = registry()
    if fam.name in reg and reg[fam.name] is not fam:
        raise ValueError(f"a family named {fam.name!r} is already registered")
    reg[fam.name] = fam


def get(name: str | None) -> Family:
    reg = registry()
    name = name or DEFAULT
    if name not in reg:
        raise KeyError(f"unknown task family {name!r}; known: {', '.join(sorted(reg))}")
    return reg[name]


def of(task) -> Family:
    return get(getattr(task, "family", None))


def level(family: str | None, lv: int):
    """The level module ``lv`` of a family, with a clear error if absent."""
    fam = get(family)
    if lv not in fam.levels:
        raise KeyError(f"family {fam.name!r} has levels {sorted(fam.levels)}, not {lv}")
    return fam.levels[lv]


def level_module(task_or_family, level: int | None = None):
    fam = task_or_family if isinstance(task_or_family, Family) else of(task_or_family)
    lv = level if level is not None else task_or_family.level
    return fam.levels[lv]
