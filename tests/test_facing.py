"""The facing situation, levels 1-3: one draw for every level, each level its own
requirements, and a reference that passes the strictest (level 3). Seed 0 is
the showcase (round 5's look); seeds 0-4 generate and verify at every level."""
import os

import pytest

from jointbench.families.facing import bracket as K
from jointbench.geometry import write_step
from jointbench.grade import grade

SEED = 0


@pytest.fixture(scope="module")
def made():
    return {L: K.generate(SEED, L) for L in (1, 2, 3)}


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


def test_faces_are_horizontal_one_above_the_other(made):
    a, b = made[1][0].faces(0)
    assert a.normal == pytest.approx((0, 0, 1), abs=1e-6) and b.normal == pytest.approx((0, 0, -1), abs=1e-6)
    assert b.origin[2] - a.origin[2] == pytest.approx(made[1][1]["distance"])


def test_seed_0_is_round_5s_look(made):
    t = made[1][0]
    assert t.parts["A"].kind == "square" and t.parts["A"].params == {"bolt": "M5", "pitch": 30}
    assert t.parts["B"].kind == "round" and t.parts["B"].params["spigot"] == {"d": 30.0, "h": 2.0}
    a, b = t.faces(0)
    assert (b.origin[0] - a.origin[0], b.origin[1] - a.origin[1]) == pytest.approx((-6.0, 5.5))


def test_the_reference_passes_level_3(made, tmp_path):
    task, _, ref = made[3]
    p = os.path.join(tmp_path, "reference.step")
    write_step(ref, p)
    rep = grade(task, p, optional=False, score=False)
    assert rep.passed, rep.text()


def test_the_brief_states_the_process_only_where_required(made):
    from jointbench.families.facing.brief import brief

    assert "printed in PLA" not in brief(made[1][0])
    assert "printed in PLA" in brief(made[2][0])
    assert "STIFFNESS" in brief(made[3][0]) and "STIFFNESS" not in brief(made[2][0])


@pytest.mark.parametrize("seed", [1, 2, 3, 4])
def test_random_seeds_give_a_task_the_reference_passes(seed, tmp_path):
    task, d, ref = K.generate(seed, 3)
    assert d["distance"] > 0 and max(abs(v) for v in d["offset"]) <= K.OFFSET_MAX
    p = os.path.join(tmp_path, "reference.step")
    write_step(ref, p)
    assert grade(task, p, optional=False, score=False).passed
