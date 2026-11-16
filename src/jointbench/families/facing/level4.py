"""Facing mounts, level 4 (assembles): two parts in through one small opening, bolted
together inside. See ``assembles``."""
from . import assembles
from .brief4 import SITUATION  # noqa: F401

LEVEL = 4
SELFTEST_VARIANTS: list = []


def generate(seed: int, knobs: dict | None = None):
    """(level 4 task, draw, (part 1, part 2, assembly file))."""
    t3, d = assembles.setup(seed)
    t4, ref, _ = assembles.make(t3, d)
    return t4, d, ref


def write_task(seed: int, out_dir: str) -> str:
    return assembles.write_task(seed, out_dir)
