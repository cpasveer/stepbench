import os

import build123d as bd
import pytest

from jointbench import selftest
from jointbench.baseline import solve
from jointbench.geometry import write_step
from jointbench.grade import grade
from jointbench.levels import level1, level2
from jointbench.workspace import write_workspace

FAST = {"stiffness": 0}  # see test_jointbench: tests not about stiffness skip its analysis


@pytest.fixture(scope="module")
def l2():
    return level2.generate(0, FAST)


def _status(rep, check):
    return next(c for c in rep.checks if c.name == check).ok


def test_level2_same_seed_identical(tmp_path):
    t1, d1, r1 = level2.generate(1, FAST)
    t2, d2, r2 = level2.generate(1, FAST)
    assert t1.dumps() == t2.dumps()
    write_step(r1, str(tmp_path / "a.step"))
    write_step(r2, str(tmp_path / "b.step"))
    assert (tmp_path / "a.step").read_bytes() == (tmp_path / "b.step").read_bytes()


def test_level2_nothing_is_axis_aligned(l2):
    task, d, _ = l2
    fa, fb = task.faces(0)
    for f in (fa, fb):
        assert max(abs(c) for c in f.normal) < 0.999, "a face normal is along a world axis"
    # the faces are tilted relative to each other: not anti-parallel
    cos = sum(a * b for a, b in zip(fa.normal, fb.normal))
    assert cos > -0.95
    assert 20 <= d["tilt"] <= 70


def test_level2_reference_and_alt_pass(l2, tmp_path):
    task, d, ref = l2
    for i, part in enumerate((ref, level2.build_connector(task, d, style="alt"))):
        p = str(tmp_path / f"p{i}.step")
        write_step(part, p)
        assert grade(task, p).passed


def test_level2_selftest_rows():
    rows = selftest.run([2], level=2)
    assert all(r.ok for r in rows), selftest.table(rows)


def test_seating_tolerates_tiny_tilt(l2, tmp_path):
    """A seat face a hair off parallel (as another CAD program might write
    it) still counts; NO_INTERFERENCE, not SEATING, judges penetration."""
    task, d, ref = l2
    fa = task.faces(0)[0]
    pl = bd.Plane(origin=fa.origin, z_dir=fa.normal)
    tilted = ref.rotate(bd.Axis(fa.origin, pl.x_dir), 2e-5)  # degrees
    p = str(tmp_path / "t.step")
    write_step(tilted, p)
    assert _status(grade(task, p), "SEATING") is True


def test_template_baseline_passes_level1_fails_level2(l2, tmp_path):
    t1, d1, r1 = level1.generate(0, FAST)
    ws1 = write_workspace(t1, r1, str(tmp_path))
    rep1, out1 = solve(ws1, "template")
    assert rep1.passed and out1.endswith("baseline_template.step")
    task, d, ref = l2
    ws2 = write_workspace(task, ref, str(tmp_path))
    rep2, _ = solve(ws2, "template")
    assert not rep2.passed
    assert "SEATING" in rep2.failed()  # plate B is parallel to A, not to face B


def test_level2_brief_names_the_situation(l2, tmp_path):
    task, d, ref = l2
    ws = write_workspace(task, ref, str(tmp_path))
    txt = open(os.path.join(ws, "README.md")).read()
    assert "at an angle" in txt and "Level 2" in txt
