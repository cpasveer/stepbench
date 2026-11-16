"""The flange situation, levels 1-3 (fits, printable, strong): a round flange
and a tall rectangular block, their faces upright and at an angle; one solid
bolts to both. Level 4 (assembles): two parts in through one small opening of
a closed box, bolted together inside (``assembles``); level 5 (locks): the two
joined by a push-and-turn lock instead (``locks5``). The same seed gives the
same fixed parts at every level; seed 0 is round 5's arrangement without its
bearing. See ``shell``."""
from .. import Family
from . import level1, level2, level3, level4, level5
from .brief import brief

FAMILY = Family(
    name="flange",
    title="Flange and block",
    levels={1: level1, 2: level2, 3: level3, 4: level4, 5: level5},
    checks=[],
    baselines={},
    brief=brief,
    doc="A round flange and a tall block, faces upright at an angle; one solid bolts to both. Levels 1-3; 4 (assembles: "
        "two parts in through one opening of a box, bolted inside) and 5 (locks: joined by push-and-turn).",
)
