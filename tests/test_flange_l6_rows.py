"""Flange seeds whose block is too short for level 6 (bolt rows under 70 mm apart) are redrawn, at every level
alike (one draw per seed for the whole ladder): every fresh seed gives a level 6 task that can be solved."""
import random

import pytest

from jointbench.families.flange import shell as SH


def test_limit_and_showcase():
    assert SH.L6_ROW_GAP_MIN == 70
    assert SH.long_enough_for_l6(SH._draw(random.Random("flange-0"), 0))  # the showcase: 80 mm
    assert not SH.long_enough_for_l6({"B": {"pitch_y": 60}}) and SH.long_enough_for_l6({"B": {"pitch_y": 70}})


class _Stop(Exception):
    pass


@pytest.mark.parametrize("seed", range(1, 25))
def test_every_draw_built_is_long_enough(seed, monkeypatch):
    """The generator never goes on to build a draw with a short block: the first draw it lays out has rows at
    least 70 mm apart, and a short one before it is tallied as redrawn."""
    got = {}

    def layout(d):
        got["d"] = d
        raise _Stop

    monkeypatch.setattr(SH, "layout", layout)
    tally = {}
    with pytest.raises(_Stop):
        SH.generate(seed, 1, tally=tally)
    assert got["d"]["B"]["pitch_y"] >= 70
    first = SH._draw(random.Random(f"flange-{seed}"), seed)
    if first["B"]["pitch_y"] < 70:
        assert tally.get("block's bolt rows too close for level 6 (redrawn)", 0) >= 1
    else:
        assert got["d"] == first  # a long enough first draw is kept as it was
