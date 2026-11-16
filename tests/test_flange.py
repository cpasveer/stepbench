"""The flange situation, levels 1-3: one draw for every level, each level its own
requirements, seed 0 the round 5 arrangement, and a reference that passes the
strictest (level 3)."""
import math
import os

import pytest

from jointbench.families.flange import shell as S
from jointbench.geometry import write_step
from jointbench.grade import grade

SEED = 0


@pytest.fixture(scope="module")
def made():
    return {L: S.generate(SEED, L) for L in (1, 2, 3)}


def test_one_seed_gives_the_same_fixed_parts_at_every_level(made):
    p1, p3 = made[1][0].poses[0], made[3][0].poses[0]
    for n in "AB":
        assert p1[n].origin == p3[n].origin and p1[n].rot == p3[n].rot
    assert made[1][0].parts.keys() == made[2][0].parts.keys() == {"A", "B"}


def test_each_level_states_its_own_requirements(made):
    req = {L: set(made[L][0].tolerances["required_checks"]) for L in (1, 2, 3)}
    assert req[1] == {"SEAT_INSIDE", "CLEAN", "KEY_TURN"}
    assert req[2] == req[1] | {"MANUFACTURING"}
    assert req[3] == req[2] | {"STIFFNESS"}
    assert made[1][0].stiffness is None and made[3][0].stiffness["per_mount"]
    assert made[3][0].tolerances.get("score") == "material" and "score" not in made[1][0].tolerances
    assert made[3][0].tolerances["tool_length"] == 30.0 and made[3][0].tolerances["tool_diameter_extra"] == 0.0


def test_seed_0_is_the_round_5_arrangement(made):
    task = made[1][0]
    a, b = task.parts["A"], task.parts["B"]
    assert (a.kind, a.params["n"], a.params["pcd"], a.bolt) == ("round", 6, 50, "M6")
    assert a.params["spigot"] == {"d": 25.0, "h": 5.0}
    assert (b.kind, b.params["pitch_x"], b.params["pitch_y"], b.bolt) == ("rect", 20, 80, "M6")
    fa, fb = task.faces(0)
    cos = sum(x * y for x, y in zip(fa.normal, fb.normal))
    assert math.degrees(math.acos(-cos)) == pytest.approx(20.0, abs=1e-6)  # faces bent 20 deg from facing
    # both faces upright (the part prints standing), the block standing tall
    assert abs(fa.normal[2]) < 1e-9 and abs(fb.normal[2]) < 1e-9
    assert abs(task.poses[0]["B"].vec((0, 1, 0))[2]) == pytest.approx(1.0)


def test_other_seeds_vary_round_the_showcase():
    import random

    draws = [S._draw(random.Random(f"flange-{s}"), s) for s in range(1, 6)]
    assert len({(d["D"], d["S"], d["H"], d["BEND"]) for d in draws}) == 5
    for d in draws:
        assert S.D_RANGE[0] <= d["D"] <= S.D_RANGE[1] and S.BEND_RANGE[0] <= d["BEND"] <= S.BEND_RANGE[1]


def test_the_reference_passes_level_3(made, tmp_path):
    task, _, ref = made[3]
    p = os.path.join(tmp_path, "reference.step")
    write_step(ref, p)
    rep = grade(task, p, optional=False, score=False)
    assert rep.passed, rep.text()


def test_the_brief_states_the_process_only_where_required(made):
    from jointbench.families.flange.brief import brief

    assert "printed in PLA" not in brief(made[1][0])
    assert "printed in PLA" in brief(made[2][0])
    assert "STIFFNESS" in brief(made[3][0]) and "STIFFNESS" not in brief(made[2][0])
    assert "spigot 25 mm dia x 5 mm" in brief(made[1][0])
