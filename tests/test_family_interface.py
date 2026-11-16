"""The family interface: solid fixed parts, per-family core checks,
intended overlap. A toy 'press fit' family: a pin pressed into a block."""
import json
import os

import build123d as bd

from jointbench import families
from jointbench.catalogue import make_interface
from jointbench.families import Family
from jointbench.geometry import Placement, write_step
from jointbench.grade import grade
from jointbench.task import DEFAULT_TOLERANCES, Task
from jointbench.workspace import write_workspace

SITUATION = "A block is fixed; press a pin into its bore."


class _Level:
    LEVEL = 1
    SITUATION = SITUATION
    SELFTEST_VARIANTS = []


def _task():
    block = (bd.Pos(0, 0, -10) * bd.Box(40, 40, 20) - bd.Pos(0, 0, -5) * bd.Cylinder(4.95, 10)).solids()[0]
    itf = make_interface("solid", {"step": "fixed_A.local.step", "description": "block with a 9.9 mm bore"})
    itf.solid = block
    pose = {"A": Placement((0.0, 0.0, 0.0), ((1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0)))}
    t = Task("press-L1-s0", 1, 0, {"A": itf}, [pose], dict(DEFAULT_TOLERANCES), {}, family="press")
    # the bore is meant to overlap a pressed pin by 0.05 mm radially
    t.intended_overlaps = [{"part": "A", "kind": "cylinder", "center": [0, 0, -5], "axis": [0, 0, 1],
                            "radius": 5.1, "length": 10.2}]
    return t


def setup_module():
    families.register(Family(name="press", title="Press fit", levels={1: _Level()},
                             core_checks={"NO_INTERFERENCE"}, brief=lambda task: SITUATION + "\n"))


def _grade(task, part, tmp_path):
    p = str(tmp_path / "pin.step")
    write_step(part, p)
    return grade(task, p, optional=False, score=False)


def test_solid_part_round_trips_through_a_task_folder(tmp_path):
    t = _task()
    ws = write_workspace(t, bd.Box(1, 1, 1), str(tmp_path))
    assert os.path.exists(os.path.join(ws, "fixed_A.local.step"))
    j = json.load(open(os.path.join(ws, "task.json")))
    assert j["family"] == "press" and j["intended_overlaps"][0]["part"] == "A"
    loaded = Task.load(os.path.join(ws, "task.json"))
    assert loaded.parts["A"].kind == "solid" and not loaded.parts["A"].has_face
    assert abs(loaded.fixed_solids(0)["A"].volume - t.parts["A"].solid.volume) < 1e-6
    assert open(os.path.join(ws, "README.md")).read().startswith(SITUATION)


def test_only_declared_core_checks_run_and_intended_overlap_is_allowed(tmp_path):
    t = _task()
    pin = bd.Pos(0, 0, 0) * bd.Cylinder(5.0, 20)  # 0.05 mm into the bore wall: intended
    rep = _grade(t, pin, tmp_path)
    names = [c.name for c in rep.checks]
    assert "HOLES" not in names and "SEATING" not in names and "NO_INTERFERENCE" in names
    assert rep.passed, rep.text()
    ni = next(c for c in rep.checks if c.name == "NO_INTERFERENCE")
    assert any("intended overlap" in r["what"] and r["volume"] > 0 for r in ni.details["overlaps"])


def test_overlap_outside_the_declared_region_still_fails(tmp_path):
    t = _task()
    pin = bd.Pos(0, 0, 0) * bd.Cylinder(5.0, 20) + bd.Pos(6, 0, 0.5) * bd.Box(4, 4, 3)  # a lug dipping 1 mm into the block top
    rep = _grade(t, pin.solids()[0] if len(pin.solids()) == 1 else pin, tmp_path)
    assert "NO_INTERFERENCE" in rep.failed()


def test_pose_scene_moves_neighbours_and_checks_them_against_each_other(tmp_path):
    """A toy mechanism: two blocks the family moves per check pose; at the
    last of five check poses they collide."""
    from jointbench.geometry import Placement as Pl

    def block():
        return bd.Box(10, 10, 10).solids()[0]

    a, b = make_interface("solid", {"step": "a.step"}), make_interface("solid", {"step": "b.step"})
    a.solid, b.solid = block(), block()
    ident = ((1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0))
    t = Task("swing-L1-s0", 1, 0, {"A": a, "B": b},
             [{"A": Pl((0.0, 0.0, 0.0), ident), "B": Pl((30.0, 0.0, 0.0), ident)}],
             dict(DEFAULT_TOLERANCES), {}, family="swing")
    seen = []

    def scene(task, ctx, k):
        seen.append(k)
        return {"A": Pl((0.0, 0.0, 0.0), ident), "B": Pl((30.0 - 6.0 * k, 0.0, 0.0), ident)}  # B closes in

    families.register(Family(name="swing", title="Swing", levels={1: _Level()}, core_checks={"NO_INTERFERENCE"},
                             pose_scene=scene, check_poses=lambda task: 5, brief=lambda task: "swing\n"))
    rep = _grade(t, bd.Pos(0, 40, 0) * bd.Box(5, 5, 5), tmp_path)  # an answer well clear of both
    assert sorted(set(seen)) == [0, 1, 2, 3, 4]  # the checks ran at every check pose
    ni = next(c for c in rep.checks if c.name == "NO_INTERFERENCE")
    assert ni.ok is False and "collide at pose 4" in ni.reason


def test_grader_data_travels_in_solution_and_regenerates_without_it(tmp_path):
    import shutil

    calls = []
    families.register(Family(name="hidden", title="Hidden", levels={1: _Level()}, core_checks={"NO_INTERFERENCE"},
                             brief=lambda task: "hidden\n", submission=lambda task: "# my stub\n",
                             grader=lambda task: calls.append(task.seed) or {"table": [task.seed, 2]}))
    t = _task()
    t.family, t.id = "hidden", "hidden-L1-s0"
    ws = write_workspace(t, bd.Box(1, 1, 1), str(tmp_path), {"draw": {"grader": {"table": [9, 9]}}})
    assert "table" not in open(os.path.join(ws, "task.json")).read()  # not for the agent
    assert open(os.path.join(ws, "submission.py")).read() == "# my stub\n"
    loaded = Task.load(os.path.join(ws, "task.json"))
    # the family's hook wins over the file (an edited file cannot loosen grading)
    assert families.grader_data(loaded) == {"table": [0, 2]} and calls == [0]
    shutil.rmtree(os.path.join(ws, "_solution"))  # an agent's copy grades the same
    again = Task.load(os.path.join(ws, "task.json"))
    assert families.grader_data(again) == {"table": [0, 2]}
    # a family without the hook falls back to the file
    families.registry()["hidden"].grader = None
    ws2 = write_workspace(t, bd.Box(1, 1, 1), str(tmp_path / "b"), {"draw": {"grader": {"table": [9, 9]}}})
    assert families.grader_data(Task.load(os.path.join(ws2, "task.json"))) == {"table": [9, 9]}
