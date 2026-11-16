"""Facing mounts, level 5 (locks): level 4 with the joint between the two parts a
push-and-turn lock. See ``locks5``."""
from . import assembles, locks5
from .brief5 import SITUATION  # noqa: F401

LEVEL = 5
SELFTEST_VARIANTS: list = []


def generate(seed: int, knobs: dict | None = None):
    """(level 5 task, draw, (part 1, part 2, assembly file))."""
    t3, d = assembles.setup(seed)
    t5, ref, _ = locks5.make(t3, d)
    return t5, d, ref


def write_task(seed: int, out_dir: str) -> str:
    return locks5.write_task(seed, out_dir)
