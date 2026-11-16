"""The flange situation, level 4 ("assembles"): two parts, one on the flange and one on the block, put in
one after the other through one small opening in a closed box, bolted together inside."""
from __future__ import annotations

import json
import os

import numpy as np
import pytest

from jointbench.assemble.generate import with_load, write_answer
from jointbench.assemble.grade import grade
from jointbench.families.flange import assembles as F
from jointbench.task import Task


@pytest.fixture(scope="module")
def made(tmp_path_factory):
    t3, d = F.setup(0)
    t4, ref, dims = F.make(t3, d)
    return t3, d, t4, ref, dims, tmp_path_factory.mktemp("flange-l4")


def test_the_fixed_parts_are_level_3s(made):
    """Same seed, same fixed parts: the level 4 task is the level 3 task plus the box and the joint."""
    t3, d, t4, ref, dims, tmp = made
    assert t4.parts.keys() == t3.parts.keys() and t4.poses == t3.poses and t4.stiffness == t3.stiffness
    assert t4.id == "flange-L4-s0" and t4.level == 4 and t4.family == "flange"


def test_the_reference_passes_the_whole_grader(made):
    """Every check of level 3 on each part, the ways in, the joint (room, reach, length, the plastic
    round the bolts) and the stated load's stiffness of both parts together: clearly, not marginally."""
    t3, d, t4, ref, dims, tmp = made
    rep = grade(t4, write_answer(str(tmp / "ref"), *ref))
    assert rep.passed, rep.text()
    names = {c.name: c for c in rep.checks}
    for n in ("part1 KEY_TURN", "part2 KEY_TURN", "part1 FDM_OVERHANG", "part2 FDM_STANCE", "INSERTION",
              "JOINT_ROOM", "JOINT_REACH", "JOINT_LENGTH", "JOINT_BOLTS", "STIFFNESS"):
        assert n in names, n
    assert names["STIFFNESS"].details["verdict"] != "marginal"


def test_broken_copies_fail_where_they_are_broken(made):
    t3, d, t4, ref, dims, tmp = made
    why = {"wrong-order": "part1 runs into part2", "too-big-for-the-opening": "runs into obstacle wall_front",
           "key-cannot-reach": "no key reaches it from outside", "bolt-too-short": "stands out",
           "narrow-joint": "the plastic gives way"}
    got = F.broken(t3, dims, *ref)
    assert set(got) == set(why)
    for name, (q1, q2, a, must, kg) in got.items():
        r = grade(with_load(t4, kg) if kg else t4, write_answer(str(tmp / name), q1, q2, a), stiffness=False)
        assert not r.passed and must in r.failed(), (name, r.text())
        assert why[name] in next(c for c in r.checks if c.name == must).reason, (name, r.text())
    # the narrow-joint copy's load: the reference holds it
    r = grade(with_load(t4, F.NARROW_KG), write_answer(str(tmp / "ref-heavy"), *ref), stiffness=False)
    assert "JOINT_BOLTS" not in r.failed(), r.text()


def test_the_reference_is_the_same_every_time(made):
    t3, d, t4, ref, dims, tmp = made
    p1, p2, asm, _ = F.build(t3, d)
    assert asm == ref[2]
    for a, b in ((p1, ref[0]), (p2, ref[1])):
        assert a.volume == pytest.approx(b.volume, abs=1e-6)
        assert np.allclose(tuple(a.center()), tuple(b.center()), atol=1e-6)
    assert F.make(t3, d)[0].dumps() == t4.dumps()


def test_the_box_is_closed_round_both(made):
    """Nothing in place touches a wall; the one opening is in the front wall, well inside it."""
    t3, d, t4, ref, dims, tmp = made
    walls = t4.obstacle_solids(0)
    assert set(walls) == {"wall_top", "wall_bottom", "wall_back", "wall_side_1", "wall_side_2",
                          "wall_front_below", "wall_front_above", "wall_front_left", "wall_front_right"}
    for n, w in walls.items():
        for body in (*t4.fixed_solids(0).values(), *ref[:2]):
            x = body & w
            assert (x.volume if x is not None else 0.0) < 1e-6, n
    op = t4.meta["opening"]
    front = [o for o in t4.obstacles_at(0) if o.name == "wall_front_above"][0]
    assert op["along_range"][1] - op["along_range"][0] < front.size[0] - 40.0


def test_the_two_parts_as_one_cannot_get_in(made):
    """The reference's two parts fused into one: along the way either part takes, it meets the box or a
    fixed part."""
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
              "_solution/assembly.json", "_solution/part1.step", "_solution/view/index.html",
              "_solution/view/ways.json", "_solution/broken/narrow-joint/LOAD_KG"):
        assert os.path.getsize(os.path.join(folder, f)) > 0, f
    for name in F.broken(t3, dims, *ref):
        assert open(os.path.join(folder, "_solution", "broken", name, "MUST_FAIL")).read().strip()
    t = Task.load(os.path.join(folder, "task.json"))
    assert t.level == 4 and t.family == "flange" and t.meta["assembly"] and t.tolerances["joint"]["length_required"]
    text = open(os.path.join(folder, "README.md"), encoding="utf-8").read()
    for word in ("join the flange and the block", "assembly.json", '"length"', "JOINT_LENGTH", "JOINT_REACH",
                 "INSERTION", "wall_front_above", "The one opening", "boss", "uses the least material wins"):
        assert word in text, word
    asm = ref[2]  # the brief states requirements, never the answer
    for p in asm["joint"]["positions"]:
        assert f"{p[0]:.2f}" not in text
    assert str(asm["parts"][0]["insert_direction"]) not in text
    w = json.load(open(os.path.join(folder, "_solution", "view", "ways.json")))
    assert w["order"] == ["part1", "part2"] and w["opening"] == t4.meta["opening"] and "flange" in w["note"]
    e1, plane = np.asarray(t4.meta["opening"]["normal"]), t4.meta["opening"]["plane"]
    for way in w["ways"]:
        assert np.allclose(way["frames"][-1], np.eye(4).ravel(), atol=1e-6)  # the last pose is the part in place
        assert float(np.asarray(way["trail"][0]) @ e1) > plane  # it starts outside, beyond the opening


@pytest.mark.parametrize("seed", [1, 2])
def test_other_seeds_pass(seed, tmp_path):
    """Seeds 1 and 2 (random draws round the showcase): the reference passes the whole grader."""
    t3, d = F.setup(seed)
    t4, ref, dims = F.make(t3, d)
    rep = grade(t4, write_answer(str(tmp_path / "ref"), *ref))
    assert rep.passed, rep.text()
