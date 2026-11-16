"""Flange and block, level 1 (fits). See ``shell``."""
from .shell import KNOBS_DEFAULT, SITUATION  # noqa: F401
from . import shell

LEVEL = 1


def generate(seed: int, knobs: dict | None = None, max_tries: int = 60):
    return shell.generate(seed, LEVEL, max_tries=max_tries)
