"""Clamp a pipe, level 2 (printable). See ``levels``."""
from .levels import KNOBS_DEFAULT, SITUATION  # noqa: F401
from . import levels

LEVEL = 2


def generate(seed: int, knobs: dict | None = None, max_tries: int = 40):
    return levels.generate(seed, LEVEL, max_tries=max_tries)
