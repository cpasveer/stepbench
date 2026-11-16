"""The pipe situation, levels 1-3: one draw for every level, each level its own
requirements, seed 0 the round 5 pipe, and a reference that passes the
strictest (level 3)."""
import math
import os
import random

import pytest

from jointbench.families.pipe import levels as S
from jointbench.families.pipe import round5
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
    assert made[1][0].parts["A"].params == made[3][0].parts["A"].params


def test_each_level_states_its_own_requirements(made):
    req = {L: set(made[L][0].tolerances["required_checks"]) for L in (1, 2, 3)}
    assert req[1] == {"SEAT_INSIDE", "CLEAN", "KEY_TURN"}
    assert req[2] == req[1] | {"MANUFACTURING"}
    assert req[3] == req[2] | {"STIFFNESS"}
    assert made[1][0].stiffness is None and made[3][0].stiffness["per_mount"]
    assert made[3][0].tolerances.get("score") == "material" and "score" not in made[1][0].tolerances


def test_seed_0_is_the_round_5_pipe(made):
    task = made[1][0]
    ref = round5.task_from_draw(0, round5.showcase_draw(), 1)
    for n in "AB":
        assert task.poses[0][n].origin == ref.poses[0][n].origin and task.poses[0][n].rot == ref.poses[0][n].rot
    a = task.parts["A"]
    assert a.kind == "pipe" and a.params["od"] == 76.0 and len(a.params["holes"]) == 6
    assert [f.part for f in task.faces(0)] == ["B"]  # only B has a flat face
    # B stands upright, the pipe leans about 40 deg from the vertical
    (fb,) = task.faces(0)
    assert abs(fb.normal[2]) < 1e-9
    axis = task.poses[0]["A"].vec((1, 0, 0))
    assert math.degrees(math.acos(abs(axis[2]))) == pytest.approx(40.0, abs=0.5)


def test_other_seeds_vary_round_the_showcase():
    draws = [S.random_draw(random.Random(f"pipe-v1-{s}")) for s in range(1, 6)]
    assert len({tuple(d["b_centre"]) for d in draws}) == 5
    for d in draws:
        lean = math.degrees(math.asin(abs(d["b_normal"][0])))
        assert S.LEAN[0] - 1e-6 <= lean <= S.LEAN[1] + 1e-6


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
