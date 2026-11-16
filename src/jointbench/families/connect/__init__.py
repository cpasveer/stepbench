"""The connecting-part family: model one solid that bolts to the given parts.

Levels 1-3 live in ``jointbench.levels`` (where they were before families
existed); this module only declares them. Their checks are the core checks,
so ``checks`` is empty here, and they are static: no motion.
"""
from .. import Family
from ...levels import level1, level2, level3


def _baselines():
    from ...baseline import level1_template, naive_block, straight_hull, straight_rod

    return {"naive": naive_block, "template": level1_template, "hull": straight_hull, "rod": straight_rod}


FAMILY = Family(
    name="connect",
    title="Design the connecting part",
    levels={1: level1, 2: level2, 3: level3},
    checks=[],
    baselines=_baselines(),
    pose_answer=None,
    doc="Fixed parts with bolt patterns; one solid must bolt to all of them.",
)
