"""The named checks, in the order they run.

Each check is a module with a ``NAME`` and ``run(ctx) -> CheckResult``.
``CHECKS`` lists them with the first level at which they apply, so a later
level adds a row here and nothing else changes:

    level 2  ANGLE/OFFSET needs no new check: HOLES/SEATING already work on
             any axis and plane; only the generator changes.
    level 3  CLEAR_PATH  (obstacles)   -- part vs obstacle solids
             TOOL_ACCESS (per bolt)    -- a straight hex-key / driver
                                          envelope along the axis is free
    level 1  KEY_TURN    (per bolt)    -- the bolt's L-shaped hex key can
                                          swing 60 degrees (where required)
    level 4  PRINTABLE   (rules)       -- overhang, min wall, min hole,
                                          bed contact, checked on the solid
    motion   every check already loops over task.poses
"""
from __future__ import annotations

from . import bearing_seat, clean, clear_path, compliance, stiffness, file_valid, holes, interference, key_turn, manufacturing, seat_inside, section_min, seating, tool_access
from .base import CheckResult, Context

# (module, first level it applies to). Every check listed runs at every level
# from its first; the core checks always decide the verdict.
CHECKS = [
    (file_valid, 1),
    (holes, 1),
    (seating, 1),
    (seat_inside, 1),  # only where a task requires it
    (bearing_seat, 1),  # only where a task holds a bearing and requires it
    (interference, 1),
    (clear_path, 3),
    (tool_access, 3),
    (key_turn, 1),  # only where a task requires it (tasks made from 14 November 2026 on)
    (clean, 1),  # one result per modelling-quality rule
    (section_min, 1),
    (stiffness, 1),  # only where a task states a stiffness requirement
    (compliance, 1),  # only where a task states a travel and requires it
    (manufacturing, 1),  # one result per rule of the stated process
]

# Checks that are reported at every level but decide the verdict only where
# the task lists them in tolerances["required_checks"]. Task files written
# before a check existed do not list it, so their verdicts do not change.
OPTIONAL = {"CLEAN", "SECTION_MIN", "STIFFNESS", "MANUFACTURING", "SEAT_INSIDE", "BEARING_SEAT", "COMPLIANCE", "KEY_TURN"}

# The requirements a part must meet, in order. A task switches each on or
# off (fits is always on); material use is the one score among parts that
# pass. Check module name -> requirement group.
GROUPS = ("fits", "clean", "makeable", "sturdy")
GROUP_OF = {"FILE_VALID": "fits", "HOLES": "fits", "SEATING": "fits", "NO_INTERFERENCE": "fits", "SEAT_INSIDE": "fits", "BEARING_SEAT": "fits",
            "CLEAR_PATH": "fits", "TOOL_ACCESS": "fits", "KEY_TURN": "fits", "CLEAN": "clean",
            "MANUFACTURING": "makeable", "SECTION_MIN": "sturdy", "STIFFNESS": "sturdy",
            "COMPLIANCE": "sturdy"}
GROUP_SWITCH = {"clean": "CLEAN", "makeable": "MANUFACTURING", "sturdy": "SECTION_MIN"}

__all__ = ["CHECKS", "CheckResult", "Context"]
