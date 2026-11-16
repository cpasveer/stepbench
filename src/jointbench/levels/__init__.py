"""Difficulty levels. Each level is a module that provides:

    generate(seed) -> (task, draw, reference_solid)   answer-first generator
    build_connector(task, draw, **variant)           reference + broken copies
    SELFTEST_VARIANTS                                what selftest perturbs

Level 2 (tilt and offset between the faces) is a new module here; it reuses
every check unchanged, because the checks work on any axis and plane.
"""
from . import level1, level2, level3

LEVELS = {1: level1, 2: level2, 3: level3}
