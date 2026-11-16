"""Level 3's score: the material the stated process uses, among parts that pass."""
import build123d as bd
import pytest

from jointbench.geometry import write_step
from jointbench.levels import level1
from jointbench.material_score import brief_line, material_used


@pytest.fixture(scope="module")
def task():
    t, d, ref = level1.generate(0, {"stiffness": 0})
    t.tolerances["score"] = "material"
    return t


def _blocks(tmp_path):
    solid = bd.Box(40, 40, 40)
    holed = solid - bd.Box(28, 28, 50)  # the same outline with a hole through: less material
    a, b = str(tmp_path / "a.step"), str(tmp_path / "b.step")
    write_step(solid, a)
    write_step(holed, b)
    return a, b


@pytest.mark.parametrize("process", ["fdm", "pbf"])
def test_less_material_scores_lower(task, tmp_path, process):
    task.tolerances["manufacturing"] = {"process": process, "quantity": 1, "direction": None}
    a, b = _blocks(tmp_path)
    ma, mb = material_used(task, a, [0, 0, 1]), material_used(task, b, [0, 0, 1])
    assert mb["used_cm3"] < ma["used_cm3"]
    if process == "pbf":
        assert abs(ma["used_cm3"] - 64.0) < 0.01  # solid volume, 40 mm cube
    else:
        assert ma["used_cm3"] < 64.0  # printed: a shell and sparse infill, not solid
    assert "least material" in brief_line(task)
