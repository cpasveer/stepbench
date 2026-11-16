"""The ladder's front door: every version 1 test maps to a family and level, or says plainly why not."""
import pytest

from jointbench import families, ladder
from jointbench.cli import main


def test_every_built_test_maps_to_a_family_level_that_exists():
    for (setup, level), where in ladder.WHERE.items():
        assert (setup, level) in ladder.PLANNED
        if where is not None:
            fam, lv = ladder.resolve(setup, level)
            if (fam, lv) == ("moves", 6):  # level 6 has its own generator, one for every setup (cli routes it)
                from jointbench.moves.l6 import generate as L6

                assert L6.setup_for(setup, 0) is not None
            else:
                assert families.level(fam, lv) is not None


def test_version_1_is_eighteen_tests():
    assert len(ladder.PLANNED) == 18
    for s in ladder.SETUPS:
        assert {lv for s2, lv in ladder.PLANNED if s2 == s} == {1, 2, 3, 4, 5, 6}


@pytest.mark.parametrize("setup,level,why", [("facing", 6, "not built yet"), ("pipe", 6, "not built yet"),
                                              ("tube", 1, "no setup"), ("pipe", 7, "no level")])
def test_a_test_that_does_not_exist_says_why(setup, level, why):
    with pytest.raises(ladder.NotBuilt, match=why):
        ladder.resolve(setup, level)


def test_gen_with_setup_refuses_a_missing_test(tmp_path, capsys):
    assert main(["gen", "--setup", "facing", "--level", "6", "--out", str(tmp_path)]) == 2
    assert "not built yet" in capsys.readouterr().out


def test_the_table_lists_every_setup_and_level():
    t = ladder.table()
    for s in ladder.SETUPS:
        assert s in t
    for lv, name in ladder.LEVELS.items():
        assert f"L{lv} {name}" in t
