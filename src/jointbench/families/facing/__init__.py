"""The facing situation, levels 1-3 (fits, printable, strong): two fixed parts
whose mounting faces are parallel and face each other; one solid bolts to both.
Level 4 (assembles): two parts in through one small opening of a closed box,
bolted together inside (``assembles``); level 5 (locks): the two joined by a
push-and-turn lock instead (``locks5``). The same seed gives the same fixed
parts at every level. See ``bracket``."""
from .. import Family
from . import level1, level2, level3, level4, level5
from .brief import brief

FAMILY = Family(
    name="facing",
    title="Facing mounts",
    levels={1: level1, 2: level2, 3: level3, 4: level4, 5: level5},
    checks=[],
    baselines={},
    brief=brief,
    doc="Two fixed parts, faces parallel and facing each other; one solid bolts to both. Levels 1-5.",
)
