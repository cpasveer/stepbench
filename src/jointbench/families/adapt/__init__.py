"""The adapt family, the benchmark level "adapts" (level 4b in the
structure of 14 November; code and ids use the name): one part that serves two
arrangements of fixed parts, with everything of level 3 in both -- filament
printing (the part printed once, judged as handed in), a stated load in each
arrangement, printed weight as the score.

A task holds every fixed part of every arrangement; each stored pose is one
arrangement and places only its own fixed parts. The answer is
``output.step`` plus ``placement.json`` (per arrangement, a rigid move). The
family's grader checks the part itself once, then places it into each
arrangement and runs the core checks there (see ``grading``); the brief states
both arrangements in full (see ``brief``). Mounts are angled (a first, facing
version was too easy: a plain box served both arrangements).
"""
from .. import Family
from . import angled
from .brief import brief, submission
from .grading import grade_adapt

FAMILY = Family(
    name="adapt",
    title="One part for two arrangements",
    levels={1: angled},  # the family's own first level; named "adapts" in task ids
    checks=[],
    baselines={},
    brief=brief,
    submission=submission,
    grade=grade_adapt,
    doc="Two arrangements of fixed parts; one solid, placed rigidly into each, must bolt to both.",
)
