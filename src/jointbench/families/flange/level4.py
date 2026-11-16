"""Flange and block, level 4 (assembles): two parts through one opening of a closed box,
bolted together inside. The seed's fixed parts are levels 1-3's (``shell``); the reference
and its box are ``assembles``'s."""
from __future__ import annotations

import tempfile

from . import assembles
from .brief4 import SITUATION  # noqa: F401

LEVEL = 4


def generate(seed: int, knobs: dict | None = None, check: bool = True):
    """(task, draw, (part 1, part 2, assembly)). The seed's level 3 fixed parts, level 4's reference and
    box; with ``check``, the reference must pass the whole grader (the stiffness analysis included: an
    open ring is where this setup's level 4 is weakest)."""
    t3, d = assembles.setup(seed)
    t4, ref, dims = assembles.make(t3, d)
    if check:
        from ...assemble.generate import write_answer
        from ...assemble.grade import grade

        with tempfile.TemporaryDirectory() as tmp:
            rep = grade(t4, write_answer(tmp, *ref))
        if not rep.passed:
            raise RuntimeError(f"flange L4 seed {seed}: the reference fails: {', '.join(rep.failed())}")
    op = t4.meta["opening"]
    draw = dict(d, task3=t3, dims=dims,
                summary=f"{d['summary']}; opening {op['along_range'][1] - op['along_range'][0]:.0f} x "
                        f"{op['up_range'][1] - op['up_range'][0]:.0f} mm, joint {dims['size']} x {dims['length']:g}")
    return t4, draw, ref


def write_folder(task, draw: dict, ref, out_dir: str) -> str:
    return assembles.write_folder(task, ref, draw["dims"], out_dir, task3=draw["task3"])
