import json
import os

import build123d as bd
import pytest

from jointbench.baseline import straight_hull, straight_rod
from jointbench.catalogue import make_interface
from jointbench.checks.section_min import smallest_cut
from jointbench.geometry import Placement, write_step
from jointbench.grade import grade
from jointbench.levels import level1, level3
from jointbench.task import Task
from jointbench.workspace import write_workspace


KNOBS = {"parts": 2, "obstacles": 2}  # lighter than the default, to keep the tests quick


@pytest.fixture(scope="module")
def l3():
    return level3.generate(0, KNOBS)


def _grade(task, part, tmp_path, name="p.step", **kw):
    p = str(tmp_path / name)
    write_step(part, p)
    return grade(task, p, manufacturing=False, **kw)


def test_level3_is_deterministic_and_records_knobs(l3):
    task, d, _ = l3
    again, _, _ = level3.generate(0, KNOBS)
    assert again.dumps() == task.dumps()
    j = json.loads(task.dumps())
    assert j["meta"]["knobs"] == level3.knobs_with_defaults(KNOBS)
    assert j["generator_version"] == 5 and j["par"]["volume"] > 0
    assert len(j["poses"][0]["obstacles"]) == KNOBS["obstacles"]
    assert "SECTION_MIN" in j["tolerances"]["required_checks"]


def test_level3_reference_passes_and_straight_answers_collide(l3, tmp_path):
    task, d, ref = l3
    rep = _grade(task, ref, tmp_path)
    assert rep.passed, rep.text()
    names = [c.name for c in rep.checks]
    assert "CLEAR_PATH" in names and "TOOL_ACCESS" in names
    for base in (straight_hull, straight_rod):
        r = _grade(task, base(task), tmp_path, "b.step")
        assert not r.passed
        assert {"CLEAR_PATH", "NO_INTERFERENCE"} & set(r.failed())


def test_clear_path_and_tool_access_fail_cases(l3, tmp_path):
    task, d, _ = l3
    r1 = _grade(task, level3.build_connector(task, d, bump=True), tmp_path)
    assert "CLEAR_PATH" in r1.failed() and "O1" in next(c for c in r1.checks if c.name == "CLEAR_PATH").reason
    r2 = _grade(task, level3.build_connector(task, d, tool_block=True), tmp_path)
    assert "TOOL_ACCESS" in r2.failed()


def test_knobs_change_the_task_and_its_id():
    t2, d2, _ = level3.generate(1, {"parts": 2, "obstacles": 1})
    assert t2.id == "L3-s1-p2o1m4d25" and len(t2.parts) == 2
    assert level3.task_id(1, level3.KNOBS_DEFAULT) == "L3-s1"
    assert len(t2.obstacles_at(0)) == 1


def test_brief_names_obstacles_and_tool(l3, tmp_path):
    task, d, ref = l3
    ws = write_workspace(task, ref, str(tmp_path))
    txt = open(os.path.join(ws, "README.md")).read()
    assert "Obstacles -- keep clear" in txt and "TOOL_ACCESS" in txt and "O1" in txt
    assert os.path.exists(os.path.join(ws, "obstacles.step"))


def test_section_min_finds_a_neck():
    """Two plates joined by a rod with a notch: the smallest cut is the neck."""
    a = bd.Box(30, 30, 5)
    b = bd.Pos(0, 0, 60) * bd.Box(30, 30, 5)
    rod = bd.Pos(0, 0, 30) * bd.Cylinder(5, 60)
    notch = bd.Pos(5, 0, 30) * bd.Box(14, 20, 3)  # removes x > -2: a neck 3 mm deep
    part = (a + b + rod - notch).solids()[0]
    pa = [bd.Vector(x, y, 0) for x in (-10, 10) for y in (-10, 10)]
    pb = [bd.Vector(x, y, 60) for x in (-10, 10) for y in (-10, 10)]
    area, centre, normal = smallest_cut(part, pa, pb)
    full = 3.14159 * 25
    assert area < 0.35 * full  # the neck, not the full rod
    assert 28.0 <= centre[2] <= 32.0 and abs(normal[2]) > 0.99


def test_old_generator_tasks_still_load_and_grade(tmp_path):
    """A version-1 task (NEMA 17 face) still loads and grades."""
    task, d, ref = level1.generate(0)
    j = json.loads(task.dumps())
    j.pop("generator_version")
    j.pop("par", None)
    j["fixed_parts"][0]["interface"] = {"kind": "nema17", "params": {}}
    old = Task.from_json(j)
    assert old.generator_version == 1 and old.parts["A"].kind == "nema17"
    rep = _grade(old, bd.Box(5, 5, 5), tmp_path)
    assert not rep.passed and rep.checks[0].ok is True  # graded, not crashed
