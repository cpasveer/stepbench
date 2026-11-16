"""Facing mounts, level 2 (printable). See ``bracket``."""
from .bracket import KNOBS_DEFAULT, SITUATION  # noqa: F401
from . import bracket

LEVEL = 2


def generate(seed: int, knobs: dict | None = None, max_tries: int = 400):
    return bracket.generate(seed, LEVEL, max_tries=max_tries)
