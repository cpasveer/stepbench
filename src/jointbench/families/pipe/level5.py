"""Clamp a pipe, level 5 (locks): level 4 with the bolted joint between the two
parts replaced by a push-and-turn lock. The seed's fixed parts are levels 1-3's
(``levels``); the reference and its box are ``locks5``'s."""
from __future__ import annotations

import tempfile

from . import levels, locks5
from .levels import KNOBS_DEFAULT  # noqa: F401
from .brief5 import SITUATION  # noqa: F401

LEVEL = 5


def generate(seed: int, knobs: dict | None = None, max_tries: int = 40, check: bool = True):
    """(task, draw, (part 1, part 2, assembly)). The seed's level 3 fixed parts, level 5's reference
    and box; with ``check``, the reference must pass the grader (all but each part's printing check,
    which takes minutes)."""
    t3, d, _ = levels.generate(seed, 3, max_tries=max_tries)
    t5, ref, dims = locks5.make(t3)
    t5.id = f"pipe-L{LEVEL}-s{seed}"
    if check:
        from ...assemble.generate import write_answer
        from ...assemble.locked import grade

        with tempfile.TemporaryDirectory() as tmp:
            rep = grade(t5, write_answer(tmp, *ref), manufacturing=False)
        if not rep.passed:
            raise RuntimeError(f"pipe L5 seed {seed}: the reference fails: {', '.join(rep.failed())}")
    op = t5.meta["opening"]
    draw = dict(d, task3=t3, dims=dims,
                summary=f"opening {op['along_range'][1] - op['along_range'][0]:.0f} x "
                        f"{op['up_range'][1] - op['up_range'][0]:.0f} mm, push {locks5.PUSH:g} mm, "
                        f"turn {dims['theta']:.1f} deg")
    return t5, draw, ref


def write_folder(task, draw: dict, ref, out_dir: str) -> str:
    return locks5.write_folder(task, ref, draw["dims"], out_dir, task3=draw["task3"])
