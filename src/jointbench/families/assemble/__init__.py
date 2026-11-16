"""Level 4a, "assembles": two parts of the answer, bolted to each other.

Declared as a family so that a level 4a task.json (family "assemble") is
recognised everywhere; its generator, brief and grader live in
``jointbench.assemble``. The grader is a different shape from the others
(two STEP files and an assembly file), so ``jointbench verify`` hands such a
folder to ``jointbench.assemble.grade``.
"""
from .. import Family
from ...assemble import generate as level4
from ...assemble.brief import brief as _brief

FAMILY = Family(
    name="assemble",
    title="Design the connection, in two parts",
    levels={4: level4},
    brief=_brief,
    doc="Facing mounts with a closed frame between them: two parts that bolt to each other, put in along straight paths.",
)
