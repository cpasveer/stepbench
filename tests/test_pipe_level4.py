"""The pipe situation, level 4 ("assembles"): two parts round the pipe, put in one after the other
through one small opening in a closed box the pipe runs through, bolted together inside."""
from __future__ import annotations

import json
import os

import build123d as bd
import numpy as np
import pytest

from jointbench.assemble.generate import with_load, write_answer
from jointbench.assemble.grade import grade
from jointbench.families.pipe import assembles as P
from jointbench.task import Obstacle, Task


@pytest.fixture(scope="module")
def made(tmp_path_factory):
    t3 = P.setup()
    t4, ref, dims = P.make(t3)
    return t3, t4, ref, dims, tmp_path_factory.mktemp("pipe-l4")


def test_the_reference_passes_the_whole_grader(made):
    """Every check of level 3 on each part, the ways in, the joint (room, reach, length, the plastic
    round the bolts) and the stated load's stiffness of both parts together: clearly, not marginally."""
    t3, t4, ref, dims, tmp = made
    rep = grade(t4, write_answer(str(tmp / "ref"), *ref))
    assert rep.passed, rep.text()
    names = {c.name: c for c in rep.checks}
    for n in ("part1 KEY_TURN", "part2 KEY_TURN", "part1 FDM_OVERHANG", "part2 FDM_STANCE", "part1 Q_THIN_WEDGE",
              "INSERTION", "JOINT_ROOM", "JOINT_REACH", "JOINT_LENGTH", "JOINT_BOLTS", "STIFFNESS"):
        assert n in names, n
    assert names["STIFFNESS"].details["verdict"] != "marginal"


def test_broken_copies_fail_where_they_are_broken(made):
    t3, t4, ref, dims, tmp = made
    why = {"wrong-order": "part2 runs into part1", "too-big-for-the-opening": "runs into obstacle wall_front",
           "key-cannot-reach": "no key reaches it from outside", "bolt-too-short": "stands out",
           "thin-edge-holes": "tear-out"}
    got = P.broken(t3, dims, *ref)
    assert set(got) == set(why)
    for name, (q1, q2, a, must, kg) in got.items():
        r = grade(with_load(t4, kg) if kg else t4, write_answer(str(tmp / name), q1, q2, a), stiffness=False)
        assert not r.passed and must in r.failed(), (name, r.text())
        assert why[name] in next(c for c in r.checks if c.name == must).reason, (name, r.text())
    # the thin-edge copy's load: the reference holds it
    r = grade(with_load(t4, P.THIN_EDGE_KG), write_answer(str(tmp / "ref-heavy"), *ref), stiffness=False)
    assert "JOINT_BOLTS" not in r.failed(), r.text()


def test_the_reference_is_the_same_every_time(made):
    t3, t4, ref, dims, tmp = made
    p1, p2, asm, _ = P.build(t3, size=dims["size"], length=dims["length"])
    assert asm == ref[2]
    for a, b in ((p1, ref[0]), (p2, ref[1])):
        assert a.volume == pytest.approx(b.volume, abs=1e-6)
        assert np.allclose(tuple(a.center()), tuple(b.center()), atol=1e-6)
    assert P.make(t3)[0].dumps() == t4.dumps()


def test_the_box_is_closed_round_the_pipe(made):
    """The pipe runs through the top and bottom walls with a little clearance and touches no wall; the
    reference parts in place touch no wall; the one opening is in the front wall, smaller than the box."""
    t3, t4, ref, dims, tmp = made
    walls = t4.obstacle_solids(0)
    assert set(walls) == {"wall_top", "wall_bottom", "wall_back", "wall_side_1", "wall_side_2",
                          "wall_front_below", "wall_front_above", "wall_front_left", "wall_front_right"}
    flat, pipe_n = P.roles(t4)
    pipe = t4.fixed_solids(0)[pipe_n]
    for n, w in walls.items():
        x = pipe & w
        assert (x.volume if x is not None else 0.0) < 1e-6, n
        for part in ref[:2]:
            x = part & w
            assert (x.volume if x is not None else 0.0) < 1e-6, n
    holes = {o.name: o.holes for o in t4.obstacles_at(0)}
    assert len(holes["wall_top"]) == len(holes["wall_bottom"]) == 1 and not holes["wall_back"]
    R = t4.parts[pipe_n].face_h / 2
    assert holes["wall_top"][0]["radius"] == pytest.approx(R + P.PIPE_CLEAR)
    t = np.asarray(dims["F"]["t"])
    ends = [float(np.asarray(t4.poses[0][pipe_n].point((k * t4.parts[pipe_n].face_w / 2, 0, 0))) @ t) for k in (-1, 1)]
    op = t4.meta["opening"]
    assert op["wall"] == "wall_front" and op["normal"] == pytest.approx(list(dims["F"]["e1"]), abs=1e-5)
    box_t = [float(np.asarray(o.to_json()["center"]) @ t) for o in t4.obstacles_at(0) if o.name in ("wall_top", "wall_bottom")]
    assert min(ends) < min(box_t) - P.WALL and max(ends) > max(box_t) + P.WALL  # the pipe passes both walls
    assert op["up_range"][1] - op["up_range"][0] < max(box_t) - min(box_t) - 10.0


def test_the_two_parts_as_one_cannot_get_in(made):
    """The reference's two parts fused into one: along the way either part takes, it meets the box,
    a fixed part or the pipe."""
    from jointbench.assemble.sweep import path_collisions

    t3, t4, ref, dims, tmp = made
    one = ref[0].fuse(ref[1]).clean().solids()[0]
    bodies = dict(t4.fixed_solids(0), **t4.obstacle_solids(0))
    for entry in ref[2]["parts"]:
        hits, _ = path_collisions(one, entry["insert_direction"], entry["path"], bodies)
        assert max(hits.values()) > 1.0, entry["name"]


def test_a_face_nearly_along_its_move_sweeps_next_to_nothing(made, tmp_path):
    """A part read back from its file: a face that runs all but along a move makes a nearly flat prism,
    which the kernel can intersect as if it filled all space. The sweep never counts more than the
    prism holds: part 1's last move, onto A, meets none of the walls."""
    from jointbench.assemble.sweep import collisions
    from jointbench.geometry import write_step

    t3, t4, ref, dims, tmp = made
    write_step(ref[0], str(tmp_path / "p1.step"))
    p1 = bd.import_step(str(tmp_path / "p1.step")).solids()[0]
    move = np.asarray(ref[2]["parts"][1]["path"][0]["move"], float)
    hits = collisions(p1, move, t4.obstacle_solids(0), float(np.linalg.norm(move)))
    assert max(hits.values()) < 1e-3, hits


def test_the_folder_brief_and_view(made, tmp_path):
    t3, t4, ref, dims, tmp = made
    folder = P.write_folder(t4, ref, dims, str(tmp_path), task3=t3)
    for f in ("task.json", "README.md", "submission.py", "fixed_A.step", "fixed_B.step", "obstacles.step",
              "_solution/assembly.json", "_solution/part1.step", "_solution/view/index.html",
              "_solution/view/ways.json", "_solution/broken/thin-edge-holes/LOAD_KG"):
        assert os.path.getsize(os.path.join(folder, f)) > 0, f
    for name in P.broken(t3, dims, *ref):
        assert open(os.path.join(folder, "_solution", "broken", name, "MUST_FAIL")).read().strip()
    t = Task.load(os.path.join(folder, "task.json"))
    assert t.level == 4 and t.family == "pipe" and t.meta["assembly"] and t.tolerances["joint"]["length_required"]
    text = open(os.path.join(folder, "README.md"), encoding="utf-8").read()
    for word in ("clamp the pipe", "assembly.json", '"length"', "JOINT_LENGTH", "JOINT_REACH", "INSERTION",
                 "wall_front_above", "The one opening", "through the top and bottom walls", "uses the least material wins"):
        assert word in text, word
    # the brief states requirements, never the answer
    asm = ref[2]
    for p in asm["joint"]["positions"]:
        assert f"{p[0]:.2f}" not in text
    assert str(asm["parts"][0]["insert_direction"]) not in text
    w = json.load(open(os.path.join(folder, "_solution", "view", "ways.json")))
    assert w["order"] == ["part2", "part1"] and w["opening"] == t4.meta["opening"] and "pipe" in w["note"]
    e1, plane = np.asarray(t4.meta["opening"]["normal"]), t4.meta["opening"]["plane"]
    for way in w["ways"]:
        assert np.allclose(way["frames"][-1], np.eye(4).ravel(), atol=1e-6)  # the last pose is the part in place
        assert float(np.asarray(way["trail"][0]) @ e1) > plane  # it starts outside, beyond the opening


def test_an_obstacle_with_holes_round_trips():
    o = Obstacle("wall", (40.0, 30.0, 6.0), P.Placement((1.0, 2.0, 3.0), ((1, 0, 0), (0, 1, 0), (0, 0, 1))))
    o.holes = ({"center": [1.0, 2.0, 3.0], "axis": [0.0, 0.0, 1.0], "radius": 5.0, "length": 20.0},)
    j = o.to_json()
    back = Obstacle.from_json(j)
    assert back.holes == o.holes and back.kind == "box"
    s = back.solid()
    assert s.volume == pytest.approx(40 * 30 * 6 - np.pi * 25 * 6, rel=1e-6)
