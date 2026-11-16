"""Facing mounts, level 3 (strong). See ``bracket``."""
from .bracket import KNOBS_DEFAULT, SITUATION  # noqa: F401
from . import bracket

LEVEL = 3


def generate(seed: int, knobs: dict | None = None, max_tries: int = 400):
    return bracket.generate(seed, LEVEL, max_tries=max_tries)
