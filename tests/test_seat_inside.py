"""SEAT_INSIDE: the seat stays a set distance inside each mounting face's edge."""
import build123d as bd
import pytest

from jointbench.geometry import write_step
from jointbench.grade import grade
from jointbench.levels import level1

FAST = {"stiffness": 0}


@pytest.fixture(scope="module")
def made():
    return level1.generate(0, FAST)


def _plate_on(task, part, grow, t=6.0):
    """A plate exactly the fixture's outline grown by ``grow`` mm (negative: set back), holes at the bolts."""
    itf = task.parts[part]
    pp = task.poses[0][part]
    if itf.kind == "round":
        body = bd.Pos(0, 0, t / 2) * bd.Cylinder(itf.face_w / 2 + grow, t)
    else:
        body = bd.Pos(0, 0, t / 2) * bd.Box(itf.face_w + 2 * grow, itf.face_h + 2 * grow, t)
    return body.moved(pp.location())


def _rep(task, solid, tmp_path, required=True):
    p = str(tmp_path / "part.step")
    write_step(solid, p)
    if required:
        task.tolerances["required_checks"] = sorted(set(task.tolerances.get("required_checks", [])) | {"SEAT_INSIDE"})
    else:
        task.tolerances["required_checks"] = [c for c in task.tolerances.get("required_checks", []) if c != "SEAT_INSIDE"]
    rep = grade(task, p, optional=True, score=False, skip=frozenset({"MANUFACTURING", "STIFFNESS"}))
    return next((c for c in rep.checks if c.name == "SEAT_INSIDE"), None)


@pytest.mark.parametrize("grow, ok", [(-1.2, True), (-0.8, False), (0.0, False), (2.0, False)])
def test_setback_of_one_millimetre(made, tmp_path, grow, ok):
    task, draw, ref = made
    c = _rep(task, _plate_on(task, "A", grow), tmp_path)
    assert c is not None and c.ok is ok, c.reason


def test_not_judged_where_not_required(made, tmp_path):
    task, draw, ref = made
    assert _rep(task, _plate_on(task, "A", 2.0), tmp_path, required=False) is None


def test_brief_states_the_fixture_rule_and_clean_modelling(made):
    from jointbench.workspace import brief, required_lines

    task, draw, ref = made
    task.tolerances["required_checks"] = ["SEAT_INSIDE", "CLEAN"]
    lines = "\n".join(required_lines(task))
    assert "1 mm" in lines and "knife edges" in lines and "min 5 deg" in lines
    text = brief(task)
    assert "`SEAT_INSIDE`" in text and "`CLEAN`" in text
