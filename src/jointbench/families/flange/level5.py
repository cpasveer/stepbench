"""Flange and block, level 5 (locks): level 4 with the joint between the two parts a
push-and-turn lock. The seed's fixed parts are levels 1-3's (``shell``); the reference and
its box are ``locks5``'s."""
from __future__ import annotations

import tempfile

from . import assembles, locks5
from .brief5 import SITUATION  # noqa: F401

LEVEL = 5


def generate(seed: int, knobs: dict | None = None, check: bool = True):
    """(task, draw, (part 1, part 2, assembly)). With ``check``, the reference must pass the grader (all
    but each part's printing check, which is level 4's design and takes minutes)."""
    t3, d = assembles.setup(seed)
    t5, ref, dims = locks5.make(t3, d)
    if check:
        from ...assemble.generate import write_answer
        from ...assemble.locked import grade

        with tempfile.TemporaryDirectory() as tmp:
            rep = grade(t5, write_answer(tmp, *ref), manufacturing=False)
        if not rep.passed:
            raise RuntimeError(f"flange L5 seed {seed}: the reference fails: {', '.join(rep.failed())}")
    op = t5.meta["opening"]
    draw = dict(d, task3=t3, dims=dims,
                summary=f"{d['summary']}; opening {op['along_range'][1] - op['along_range'][0]:.0f} x "
                        f"{op['up_range'][1] - op['up_range'][0]:.0f} mm, push {dims['push']:g} mm, "
                        f"turn {dims['theta']:.1f} deg")
    return t5, draw, ref


def write_folder(task, draw: dict, ref, out_dir: str) -> str:
    return locks5.write_folder(task, ref, draw["dims"], out_dir, task3=draw["task3"])
