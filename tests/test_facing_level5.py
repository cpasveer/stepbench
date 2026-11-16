"""The facing setup, level 5 ("locks"): two parts in the closed box round the two facing mounts,
joined to each other not by bolts but by a push-and-turn lock (level 4d's checks). Seed 0 is the
showcase; seeds 1-4 generate and their references pass."""
from __future__ import annotations

import json
import os

import numpy as np
import pytest

from jointbench.assemble import joint as J
from jointbench.assemble import locked as LK
from jointbench.assemble.generate import write_answer
from jointbench.families.facing import locks5 as L
from jointbench.task import Task


@pytest.fixture(scope="module")
def made(tmp_path_factory):
    t3, d = L.A.setup(0)
    t5, ref, dims = L.make(t3, d)
    return t3, d, t5, ref, dims, tmp_path_factory.mktemp("facing-l5")


def test_the_reference_passes_the_whole_grader(made):
    """Each part as at level 3 (printing included), the ways in, and every lock check: free before the
    turn, blocked after it, the stated load through the lock's plastic."""
    t3, d, t5, ref, dims, tmp = made
    rep = LK.grade(t5, write_answer(str(tmp / "ref"), *ref))
    assert rep.passed, rep.text()
    names = [c.name for c in rep.checks]
    for n in ("part1 FDM_OVERHANG", "part2 FDM_WALL_MIN", "part1 KEY_TURN", "INSERTION", "SEATED", "PUSH", "TURN",
              "GAP", "LOCKED", "UNLOCKS", "INTERLOCK", "PLAY", "LOCK_STRENGTH"):
        assert n in names, n
    assert "STIFFNESS" not in names and not any(n.startswith("JOINT_") for n in names)
    assert rep.info["material_used"]["grams"] > 0


def test_the_same_seed_gives_level_3s_fixed_parts(made):
    t3, d, t5, ref, dims, tmp = made
    for n in "AB":
        assert t5.poses[0][n].origin == t3.poses[0][n].origin and t5.poses[0][n].rot == t3.poses[0][n].rot
    assert t5.level == 5 and t5.id == "facing-L5-s0" and t5.meta["lock"]["pull_load_kg"] == d["load_kg"]


def test_broken_copies_fail_where_they_are_broken(made):
    t3, d, t5, ref, dims, tmp = made
    why = {"collides-while-turning": "runs into", "pulls-out-after-turn": "does not lock",
           "locked-before-turn": "locked before the turn", "gap-too-small": "comes within",
           "too-big-for-the-opening": "runs into obstacle wall_front", "overloaded": "too weak"}
    got = L.broken(t3, dims, *ref)
    assert set(got) == set(why)
    for name, (q1, q2, a, must, kg) in got.items():
        r = LK.grade(t5, write_answer(str(tmp / name), q1, q2, a), load_kg=kg, manufacturing=False)
        assert not r.passed and must in r.failed(), (name, r.text())
        assert why[name] in next(c for c in r.checks if c.name == must).reason, (name, r.text())


def test_the_lock_is_stated_along_its_axis(made, tmp_path):
    t3, d, t5, ref, dims, tmp = made
    a = json.loads(json.dumps(ref[2]))
    ax = np.asarray(a["lock"]["turn"]["axis"], float)
    side = np.cross(ax, [1.0, 0.0, 0.0])
    a["lock"]["push"] = list(10.0 * side / np.linalg.norm(side))
    p = write_answer(str(tmp_path / "skew"), ref[0], ref[1], a)
    with pytest.raises(J.AssemblyError, match="along the turn's axis"):
        LK.load(p, list(t5.parts))


def test_the_reference_is_the_same_every_time(made):
    t3, d, t5, ref, dims, tmp = made
    p1, p2, asm, _ = L.build(t3, d)
    assert asm == ref[2]
    for a, b in ((p1, ref[0]), (p2, ref[1])):
        assert a.volume == pytest.approx(b.volume, abs=1e-6)


def test_a_spigot_on_a_is_refused_plainly(made):
    """Part 1 turns on A's face about an axis of its own: with a spigot on A it could not."""
    t3, d, t5, ref, dims, tmp = made
    import copy

    d2 = copy.deepcopy(d)
    d2["A"], d2["B"] = d["B"], d["A"]  # seed 0's B has a spigot
    with pytest.raises(ValueError, match="A's face flat"):
        L.build(t3, d2)


def test_the_folder_brief_and_view(made, tmp_path):
    t3, d, t5, ref, dims, tmp = made
    folder = L.write_folder(t5, ref, dims, str(tmp_path), task3=t3)
    for f in ("task.json", "README.md", "submission.py", "obstacles.step", "_solution/assembly.json",
              "_solution/view/index.html", "_solution/view/ways.json", "_solution/broken/overloaded/LOAD_KG"):
        assert os.path.getsize(os.path.join(folder, f)) > 0, f
    t = Task.load(os.path.join(folder, "task.json"))
    assert t.level == 5 and t.meta["lock_joint"] and t.tolerances["locks"]["gap_min"] == 0.3
    text = open(os.path.join(folder, "README.md"), encoding="utf-8").read()
    for word in ('"lock"', '"push"', "LOCKED", "UNLOCKS", "INTERLOCK", "LOCK_STRENGTH", "The one opening",
                 "no bolts", "least material"):
        assert word in text, word
    assert "/Users/" not in text and "/home/" not in text and "pipe" not in text
    assert str(ref[2]["lock"]["turn"]["degrees"]) not in text and "JOINT_BOLTS" not in text
    w = json.load(open(os.path.join(folder, "_solution", "view", "ways.json")))
    assert w["order"] == ["part2", "part1"]  # the slider ends with part 1's push and turn
    assert len(w["ways"][1]["frames"]) > len(w["ways"][0]["frames"])
    assert np.allclose(w["ways"][1]["frames"][-1], np.eye(4).ravel(), atol=1e-6)


@pytest.mark.parametrize("seed", [1, 2, 3, 4])
def test_random_seeds_give_a_task_the_reference_passes(seed, tmp_path):
    t3, d = L.A.setup(seed)
    t5, ref, dims = L.make(t3, d)
    rep = LK.grade(t5, write_answer(str(tmp_path / "ref"), *ref))
    assert rep.passed, rep.text()
