"""Level 4d, "locks": B comes in along an axis, turns, and is then locked.

Declared as a family so a "locks" task.json is recognised everywhere; generator, brief and
grader live in ``jointbench.locks``. The answer is two STEP files (part_A.step, part_B.step),
graded with ``python -m jointbench.locks verify <folder>``.
"""
from .. import Family
from ...locks import generate as locks
from ...locks.brief import brief as _brief

FAMILY = Family(
    name="locks",
    title="Make it lock",
    levels={4: locks},  # level 4d
    brief=_brief,
    doc="Facing mounts; B pushes in along a stated axis and turns a stated angle to lock: two printed parts.",
)
