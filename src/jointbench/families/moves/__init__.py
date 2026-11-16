"""Level 4c, "moves": a linkage of rigid parts that carries fixture B through stated poses.

Declared as a family so a "moves" task.json is recognised everywhere; its
generator, brief and grader live in ``jointbench.moves``. The answer is
several STEP files and an assembly file, not one output.step, so it is graded
with ``python -m jointbench.moves verify <folder>``.
"""
from .. import Family
from ...moves import generate as moves
from ...moves.brief import brief as _brief

FAMILY = Family(
    name="moves",
    title="Make it move",
    levels={4: moves},  # level 4c
    brief=_brief,
    doc="Facing mounts; one must turn about a stated axis through a stated range: rigid parts, assembled.",
)
