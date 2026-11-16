"""The facing setup, level 4 ("assembles"): two parts, put in one after the other through one small
opening in a closed box round the two facing mounts, bolted together inside. Seed 0 is the showcase;
seeds 1-4 generate and their references pass."""
from __future__ import annotations

import json
import os

import numpy as np
import pytest

from jointbench.assemble.generate import with_load, write_answer
from jointbench.assemble.grade import grade
from jointbench.families.facing import assembles as F
from jointbench.task import Task


@pytest.fixture(scope="module")
def made(tmp_path_factory):
    t3, d = F.setup(0)
    t4, ref, dims = F.make(t3, d)
    return t3, d, t4, ref, dims, tmp_path_factory.mktemp("facing-l4")


def test_the_reference_passes_the_whole_grader(made):
    """Every check of level 3 on each part, the ways in, the joint (room, reach, length, the plastic
    round the bolts) and the stated load's stiffness of both parts together: clearly, not marginally."""
    t3, d, t4, ref, dims, tmp = made
    rep = grade(t4, write_answer(str(tmp / "ref"), *ref))
    assert rep.passed, rep.text()
    names = {c.name: c for c in rep.checks}
    for n in ("part1 KEY_TURN", "part2 KEY_TURN", "part1 FDM_OVERHANG", "part2 FDM_STANCE", "part1 Q_STUB",
              "INSERTION", "JOINT_ROOM", "JOINT_REACH", "JOINT_LENGTH", "JOINT_BOLTS", "STIFFNESS"):
        assert n in names, n
    assert names["STIFFNESS"].details["verdict"] != "marginal"


def test_the_same_seed_gives_level_3s_fixed_parts(made):
    t3, d, t4, ref, dims, tmp = made
    assert t4.parts.keys() == t3.parts.keys() == {"A", "B"}
    for n in "AB":
        assert t4.poses[0][n].origin == t3.poses[0][n].origin and t4.poses[0][n].rot == t3.poses[0][n].rot
    assert t4.stiffness == t3.stiffness and t4.level == 4 and t4.id == "facing-L4-s0"


def test_broken_copies_fail_where_they_are_broken(made):
    t3, d, t4, ref, dims, tmp = made
    why = {"wrong-order": f"{dims['first']} runs into {dims['second']}", "too-big-for-the-opening": "runs into obstacle wall_front",
           "key-cannot-reach": "no key reaches it from outside", "bolt-too-short": "stands out",
           "small-joint-bolts": "the plastic gives way"}
    got = F.broken(t3, dims, *ref)
    assert set(got) == set(why)
    for name, (q1, q2, a, must, kg) in got.items():
        r = grade(with_load(t4, kg) if kg else t4, write_answer(str(tmp / name), q1, q2, a), stiffness=False)
        assert not r.passed and must in r.failed(), (name, r.text())
        assert why[name] in next(c for c in r.checks if c.name == must).reason, (name, r.text())
    # the small-bolts copy's load: the reference holds it
    r = grade(with_load(t4, F.WEAK_KG), write_answer(str(tmp / "ref-heavy"), *ref), stiffness=False)
    assert "JOINT_BOLTS" not in r.failed(), r.text()


def test_the_part_with_the_longer_last_move_goes_in_first(made):
    """Seed 0's B has a 2 mm spigot: B's part comes the longer way onto its face, so it goes in first and
    keeps the wall's inner layer; A's part, second, keeps the outer layer and carries the joint's heads."""
    t3, d, t4, ref, dims, tmp = made
    assert [p["name"] for p in ref[2]["parts"]] == ["part2", "part1"] and ref[2]["joint"]["head_on"] == "part1"
    assert dims["ctx"]["gap"] == F.GAP_MIN


def test_the_reference_is_the_same_every_time(made):
    t3, d, t4, ref, dims, tmp = made
    p1, p2, asm, _ = F.build(t3, d)
    assert asm == ref[2]
    for a, b in ((p1, ref[0]), (p2, ref[1])):
        assert a.volume == pytest.approx(b.volume, abs=1e-6)
        assert np.allclose(tuple(a.center()), tuple(b.center()), atol=1e-6)
    assert F.make(t3, d)[0].dumps() == t4.dumps()


def test_the_box_is_closed_and_holds_the_keys(made):
    """The reference parts in place touch no wall; the one opening is in the front wall, smaller than
    the box; every fixed bolt's hex key swings clear of the walls."""
    t3, d, t4, ref, dims, tmp = made
    walls = t4.obstacle_solids(0)
    assert set(walls) == {"wall_top", "wall_bottom", "wall_back", "wall_side_1", "wall_side_2",
                          "wall_front_below", "wall_front_above", "wall_front_left", "wall_front_right"}
    for n, w in walls.items():
        for part in ref[:2]:
            x = part & w
            assert (x.volume if x is not None else 0.0) < 1e-6, n
    op = t4.meta["opening"]
    assert op["wall"] == "wall_front" and op["normal"] == pytest.approx(list(-np.asarray(dims["dir"])), abs=1e-6)
    side = walls["wall_side_1"].bounding_box().size
    assert op["up_range"][1] - op["up_range"][0] < max(side) - 10.0


def test_the_two_parts_as_one_cannot_get_in(made):
    """The reference's two parts fused into one: along the way either part takes, it meets a fixed part
    or the box."""
    from jointbench.assemble.sweep import path_collisions

    t3, d, t4, ref, dims, tmp = made
    one = ref[0].fuse(ref[1]).clean().solids()[0]
    bodies = dict(t4.fixed_solids(0), **t4.obstacle_solids(0))
    for entry in ref[2]["parts"]:
        hits, _ = path_collisions(one, entry["insert_direction"], entry["path"], bodies)
        assert max(hits.values()) > 1.0, entry["name"]


def test_the_folder_brief_and_view(made, tmp_path):
    t3, d, t4, ref, dims, tmp = made
    folder = F.write_folder(t4, ref, dims, str(tmp_path), task3=t3)
    for f in ("task.json", "README.md", "submission.py", "fixed_A.step", "fixed_B.step", "obstacles.step",
              "_solution/assembly.json", "_solution/part1.step", "_solution/reference.json",
              "_solution/view/index.html", "_solution/view/ways.json", "_solution/broken/small-joint-bolts/LOAD_KG"):
        assert os.path.getsize(os.path.join(folder, f)) > 0, f
    t = Task.load(os.path.join(folder, "task.json"))
    assert t.level == 4 and t.family == "facing" and t.meta["assembly"] and t.tolerances["joint"]["length_required"]
    text = open(os.path.join(folder, "README.md"), encoding="utf-8").read()
    for word in ("facing mounts", "assembly.json", '"length"', "JOINT_LENGTH", "JOINT_REACH", "INSERTION",
                 "wall_front_above", "The one opening", "uses the least material wins"):
        assert word in text, word
    assert "/Users/" not in text and "/home/" not in text
    asm = ref[2]
    for p in asm["joint"]["positions"]:
        assert f"{p[0]:.2f}" not in text
    w = json.load(open(os.path.join(folder, "_solution", "view", "ways.json")))
    assert w["order"] == [dims["first"], dims["second"]] and w["opening"] == t4.meta["opening"]
    e1, plane = np.asarray(t4.meta["opening"]["normal"]), t4.meta["opening"]["plane"]
    for way in w["ways"]:
        assert np.allclose(way["frames"][-1], np.eye(4).ravel(), atol=1e-6)  # the last pose is the part in place
        assert float(np.asarray(way["trail"][0]) @ e1) > plane  # it starts outside, beyond the opening


def test_gen_writes_the_level_through_the_ladder(tmp_path):
    from jointbench.cli import main

    assert main(["gen", "--setup", "facing", "--level", "4", "--seed", "2", "--out", str(tmp_path)]) == 0
    folder = tmp_path / "facing-L4-s2"
    assert (folder / "README.md").exists() and (folder / "_solution" / "assembly.json").exists()


@pytest.mark.parametrize("seed", [1, 2, 3, 4])
def test_random_seeds_give_a_task_the_reference_passes(seed, tmp_path):
    t3, d = F.setup(seed)
    t4, ref, dims = F.make(t3, d)
    rep = grade(t4, write_answer(str(tmp_path / "ref"), *ref))
    assert rep.passed, rep.text()
