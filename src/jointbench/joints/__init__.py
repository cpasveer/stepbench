"""A catalogue of joint types: ways two parts connect, each with its own exact check.

Every joint is a module with the same small API, built ahead of the task
family interface (``docs/families.md``) so it can be wrapped as a family:

    generate(seed, **knobs) -> Instance      answer-first numbers and frame
    fixed_parts(inst) -> {name: solid}       the given part(s), world coordinates
    reference(inst, **variant) -> solid      the answer (joint frame), or a broken copy
    brief(inst) -> str                       every number the agent needs
    check(solid, inst) -> [CheckResult]      grades a submitted solid (world coordinates)
    SELFTEST                                 [(label, variant, expected failing check | None)]
    TASK, KNOBS                              the full task's shape and its difficulty settings

Run ``python -m jointbench.joints <joint> --selftest``. See ``docs/joints.md``.
"""
from __future__ import annotations

import importlib

ORDER = ["revolute", "interference", "keyed", "spline", "involute", "kinematic", "snapfit"]


def get(name: str):
    if name not in ORDER:
        raise KeyError(f"unknown joint {name!r}; known: {', '.join(ORDER)}")
    return importlib.import_module(f"{__name__}.{name}")


def all_joints():
    return [get(n) for n in ORDER]
