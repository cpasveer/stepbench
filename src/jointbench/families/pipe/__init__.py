"""The pipe situation, levels 1-3 (fits, printable, strong): a round pipe with
bolts screwed straight into it in two rows (A) and a flat square plate (B); one
solid bolts to both, seated on the pipe's round surface. Same seed, same fixed
parts at every level; seed 0 is round 5's pipe. See ``levels`` (``shell`` is
the earlier generator, kept for the record)."""
from .. import Family
from ..facing.brief import brief
from . import level1, level2, level3, level4, level5

FAMILY = Family(
    name="pipe",
    title="Clamp a pipe",
    levels={1: level1, 2: level2, 3: level3, 4: level4, 5: level5},
    checks=[],
    baselines={},
    brief=brief,
    doc="A flat fixed part and a round pipe; one solid bolts to both, seated on the pipe. Levels 1-3; "
        "4 (assembles: two parts in through one opening of a box, bolted inside) and 5 (locks: joined by "
        "push-and-turn).",
)
