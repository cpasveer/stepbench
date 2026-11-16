import filecmp
import json
import os

import build123d as bd
import pytest

from jointbench import selftest
from jointbench.baseline import solve
from jointbench.cli import main
from jointbench.geometry import write_step
from jointbench.grade import grade
from jointbench.levels import level1
from jointbench.task import Task
from jointbench.workspace import write_workspace

# the stiffness analysis takes minutes per part: tests that are not about it
# generate with it switched off (the default for powder tasks)
FAST = {"stiffness": 0}


@pytest.fixture(scope="module")
def gen0():
    return level1.generate(0, FAST)


def _grade_part(task, part, tmp_path, name="part.step"):
    p = os.path.join(tmp_path, name)
    write_step(part, p)
    return grade(task, p)


def _status(rep, check):
    return next(c for c in rep.checks if c.name == check).ok


# ---- generator --------------------------------------------------------------


def test_same_seed_gives_byte_identical_workspace(tmp_path):
    for sub in ("a", "b"):
        for seed in (3, 4):
            task, draw, ref = level1.generate(seed, FAST)
            write_workspace(task, ref, str(tmp_path / sub), {"draw": draw})
    cmp = filecmp.dircmp(tmp_path / "a", tmp_path / "b")
    assert sorted(cmp.common_dirs) == ["L1-s3", "L1-s4"]
    for d in cmp.common_dirs:
        for root, _, files in os.walk(tmp_path / "a" / d):
            for f in files:
                pa = os.path.join(root, f)
                pb = pa.replace(str(tmp_path / "a"), str(tmp_path / "b"))
                assert open(pa, "rb").read() == open(pb, "rb").read(), f


def test_different_seeds_differ():
    t0, _, _ = level1.generate(0, FAST)
    t1, _, _ = level1.generate(1, FAST)
    assert t0.dumps() != t1.dumps()


def test_task_json_round_trips(gen0):
    task, _, _ = gen0
    again = Task.from_json(json.loads(task.dumps()))
    assert again.dumps() == task.dumps()
    assert len(again.poses) == 1  # level 1: one pose, in a list


def test_grading_is_deterministic(gen0, tmp_path):
    task, _, ref = gen0
    p = os.path.join(tmp_path, "r.step")
    write_step(ref, p)
    assert grade(task, p).to_json() == grade(task, p).to_json()


# ---- each check: one pass, one fail -----------------------------------------


def test_reference_passes_every_check(gen0, tmp_path):
    task, _, ref = gen0
    rep = _grade_part(task, ref, tmp_path)
    assert rep.passed, rep.text()
    req = [c.name for c in rep.checks if c.required]
    assert req[:4] == ["FILE_VALID", "HOLES", "SEATING", "NO_INTERFERENCE"]
    assert any(n.startswith("PBF_") for n in req)  # the task's process: a part that cannot be made fails
    assert "SECTION_MIN" in [c.name for c in rep.checks if not c.required]  # reported at level 1
    assert rep.info["volume"] > 0


def test_file_valid_fails_on_two_solids(gen0, tmp_path):
    task, _, _ = gen0
    two = bd.Compound([bd.Box(5, 5, 5).solid(), bd.Pos(20, 0, 0) * bd.Box(5, 5, 5).solid()])
    rep = _grade_part(task, two, tmp_path)
    assert _status(rep, "FILE_VALID") is False
    assert "found 2" in rep.checks[0].reason
    assert all(c.ok is None for c in rep.checks[1:])  # the rest are skipped


def test_file_valid_fails_on_missing_file(gen0, tmp_path):
    task, _, _ = gen0
    rep = grade(task, os.path.join(tmp_path, "nothing.step"))
    assert not rep.passed and _status(rep, "FILE_VALID") is False


def test_holes_fail_when_shifted(gen0, tmp_path):
    task, d, _ = gen0
    rep = _grade_part(task, level1.build_connector(task, d, hole_shift=0.5), tmp_path)
    c = next(c for c in rep.checks if c.name == "HOLES")
    assert c.ok is False and abs(c.value - 0.5) < 1e-6


def _pad_with_rounded_corners(hole_at=(0.0, 0.0)):
    """A 13.6 mm square M6 pad, 10 mm thick, on a bolt at the origin (axis +z):
    a 6.6 mm hole at ``hole_at`` and its four outside vertical corners rounded
    r 3 mm -- each round's axis 5.374 mm from the bolt, a convex cylinder of
    6 mm diameter (Fable flange L1, try 3)."""
    pad = bd.Box(13.6, 13.6, 10, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    pad = bd.fillet(pad.edges().filter_by(bd.Axis.Z), 3)
    return pad - bd.Pos(*hole_at, 0) * bd.Cylinder(3.3, 10, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def _m6_bolt():
    from jointbench.task import ResolvedBolt

    return ResolvedBolt("B1", "B", "M6", (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))


def test_holes_a_rounded_outside_corner_is_not_the_hole():
    from jointbench.checks import holes

    m = holes.measure(_pad_with_rounded_corners(), _m6_bolt())
    assert m is not None and m["offset"] < 1e-6 and abs(m["diameter"] - 6.6) < 1e-6 and abs(m["end"] - 10) < 1e-6


def test_holes_off_axis_still_found_off_axis_beside_rounded_corners():
    from jointbench.checks import holes

    m = holes.measure(_pad_with_rounded_corners(hole_at=(0.5, 0.0)), _m6_bolt())
    assert abs(m["offset"] - 0.5) < 1e-6 and abs(m["diameter"] - 6.6) < 1e-6  # the hole, not a round
    solid = bd.fillet(bd.Box(13.6, 13.6, 10, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
                      .edges().filter_by(bd.Axis.Z), 3)
    assert holes.measure(solid, _m6_bolt()) is None  # no hole at all: the rounds do not count


def test_holes_fail_when_undersized(gen0, tmp_path):
    task, d, _ = gen0
    from jointbench.catalogue import BOLTS
    small = BOLTS[task.parts["A"].bolt].clear_fine - 0.2
    rep = _grade_part(task, level1.build_connector(task, d, hole_d=small), tmp_path)
    assert _status(rep, "HOLES") is False
    assert "diameter" in next(c for c in rep.checks if c.name == "HOLES").reason


def test_seating_fails_when_lifted(gen0, tmp_path):
    task, d, _ = gen0
    rep = _grade_part(task, level1.build_connector(task, d, lift=1.0), tmp_path)
    assert _status(rep, "SEATING") is False
    assert _status(rep, "HOLES") is True and _status(rep, "NO_INTERFERENCE") is True


def test_no_interference_fails_when_pushed_in(gen0, tmp_path):
    task, d, _ = gen0
    rep = _grade_part(task, level1.build_connector(task, d, push=1.0), tmp_path)
    c = next(c for c in rep.checks if c.name == "NO_INTERFERENCE")
    assert c.ok is False and c.value > 100


def test_different_body_passes(gen0, tmp_path):
    task, d, ref = gen0
    alt = level1.build_connector(task, d, style="alt")
    assert abs(alt.volume - ref.volume) > 1.0  # genuinely a different body
    assert _grade_part(task, alt, tmp_path).passed


# ---- selftest, baseline, command line ---------------------------------------


def test_selftest_table_all_as_expected():
    rows = selftest.run([0])
    assert len(rows) == len(level1.SELFTEST_VARIANTS)
    assert all(r.ok for r in rows), selftest.table(rows)
    txt = selftest.table(rows)
    for label, _, _ in level1.SELFTEST_VARIANTS:
        assert label in txt
    assert "1/1" in txt


def test_baseline_runs_and_is_graded(gen0, tmp_path):
    task, d, ref = gen0
    ws = write_workspace(task, ref, str(tmp_path))
    rep, out = solve(ws)
    assert os.path.exists(out)
    assert _status(rep, "FILE_VALID") is True
    assert not rep.passed  # the naive block cannot clear the bolt heads here


def test_cli_verify_exit_code_and_log(gen0, tmp_path, capsys):
    task, d, ref = gen0
    ws = write_workspace(task, ref, str(tmp_path))
    assert main(["verify", ws]) == 1  # no output.step yet
    capsys.readouterr()
    write_step(ref, os.path.join(ws, "output.step"))
    assert main(["verify", ws, "--json"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["passed"] is True and out["attempt"] == 2
    assert {c["name"]: c["status"] for c in out["checks"]}["HOLES"] == "pass"
    lines = open(os.path.join(ws, "attempts.jsonl")).read().splitlines()
    assert len(lines) == 2
    assert [json.loads(x)["passed"] for x in lines] == [False, True]


def test_cli_verify_build_runs_submission_stub(gen0, tmp_path):
    task, d, ref = gen0
    ws = write_workspace(task, ref, str(tmp_path))
    assert main(["verify", ws, "--build", "--no-log"]) == 1  # the stub cube fails
    assert os.path.exists(os.path.join(ws, "output.step"))


# ---- grader errors: the kernel could not measure ------------------------------


def _manufacturing_raises(monkeypatch):
    from jointbench.checks import manufacturing
    from jointbench.dfm import MeasurementError

    def boom(ctx):
        raise MeasurementError("pbf, direction-free rules: Standard_Failure")

    monkeypatch.setattr(manufacturing, "run", boom)


def _required_manufacturing(task):
    t = Task.from_json(task.to_json())
    t.tolerances["required_checks"] = sorted(set(t.tolerances.get("required_checks", [])) | {"MANUFACTURING"})
    return t


def test_measurement_error_is_error_not_fail(gen0, tmp_path, monkeypatch):
    task, d, ref = gen0
    _manufacturing_raises(monkeypatch)
    p = os.path.join(tmp_path, "part.step")
    write_step(ref, p)
    rep = grade(_required_manufacturing(task), p, optional="all")
    j = rep.to_json()
    assert rep.outcome() == "error" and j["outcome"] == "error" and j["counts_as_attempt"] is False
    assert rep.passed is False and rep.failed() == []
    m = next(c for c in j["checks"] if c["name"] == "MANUFACTURING")
    assert m["status"] == "error" and "Standard_Failure" in m["reason"]
    assert j["verdict"]["requirements"]["makeable"]["result"] == "not measured"
    assert j["verdict"]["errors"] == ["MANUFACTURING"]
    assert rep.text().startswith("ERROR")


def test_measurement_error_in_a_reported_check_leaves_the_verdict(gen0, tmp_path, monkeypatch):
    task, d, ref = gen0
    _manufacturing_raises(monkeypatch)
    t = Task.from_json(task.to_json())
    t.tolerances["required_checks"] = [c for c in t.tolerances.get("required_checks", []) if c != "MANUFACTURING"]
    p = os.path.join(tmp_path, "part.step")
    write_step(ref, p)
    rep = grade(t, p, optional="all")
    assert rep.outcome() == "pass" and rep.counts_as_attempt()
    assert rep.verdict()["errors"] == ["MANUFACTURING"]


def test_real_failure_outranks_a_measurement_error(gen0, tmp_path, monkeypatch):
    task, d, ref = gen0
    _manufacturing_raises(monkeypatch)
    p = os.path.join(tmp_path, "part.step")
    write_step(bd.Pos(0, 0, 5) * ref, p)  # lifted off the mounts: SEATING fails
    rep = grade(_required_manufacturing(task), p, optional="all")
    assert rep.outcome() == "fail" and rep.counts_as_attempt()


def test_cli_error_is_kept_apart_and_not_an_attempt(gen0, tmp_path, monkeypatch, capsys):
    task, d, ref = gen0
    _manufacturing_raises(monkeypatch)
    t = _required_manufacturing(task)
    ws = write_workspace(t, ref, str(tmp_path))
    write_step(ref, os.path.join(ws, "output.step"))
    assert main(["verify", ws, "--json"]) == 3
    out = json.loads(capsys.readouterr().out)
    assert out["outcome"] == "error" and out["attempt"] is None and out["error_record"] == 1
    assert not os.path.exists(os.path.join(ws, "attempts.jsonl"))
    assert len(open(os.path.join(ws, "errors.jsonl")).read().splitlines()) == 1
    assert os.path.exists(os.path.join(ws, "errors", "0001", "output.step"))


def _stiffness_mesh_fails(monkeypatch, clean: bool):
    import types

    import jointbench.stiffness as st

    def fake(task_dir, step, require=None):
        return types.SimpleNamespace(ok=False, requirement=None, error="gmsh could not mesh the solid: test",
                                     quality={"passed": clean, "failed": [] if clean else ["Q_SLIVER_FACE"]})

    monkeypatch.setattr(st, "screened", fake)
    import jointbench.checks.stiffness as chk

    monkeypatch.setattr(chk, "ISOLATE", False)  # the stand-in lives in this process


def _required_stiffness(task):
    t = Task.from_json(task.to_json())
    t.tolerances["required_checks"] = sorted(set(t.tolerances.get("required_checks", [])) | {"STIFFNESS"})
    t.stiffness = {"min": 1.0}
    return t


@pytest.mark.parametrize("clean,outcome", [(True, "error"), (False, "fail")])
def test_stiffness_mesh_failure_is_error_only_on_a_clean_model(gen0, tmp_path, monkeypatch, clean, outcome):
    task, d, ref = gen0
    _stiffness_mesh_fails(monkeypatch, clean)
    p = os.path.join(tmp_path, "part.step")
    write_step(ref, p)
    rep = grade(_required_stiffness(task), p, manufacturing=False, optional={"STIFFNESS"}, score=False)
    s = next(c for c in rep.checks if c.name == "STIFFNESS")
    assert rep.outcome() == outcome
    assert (s.error, s.ok) == ((True, None) if clean else (False, False))
    if not clean:
        assert "Q_SLIVER_FACE" in s.reason


def test_stiffness_over_its_cpu_cap_cannot_be_measured_and_fails(gen0, tmp_path, monkeypatch):
    import jointbench.checks.stiffness as chk

    task, d, ref = gen0
    monkeypatch.setattr(chk, "CPU_CAP_S", 1)  # one second of analysis: far too little
    p = os.path.join(tmp_path, "part.step")
    write_step(ref, p)
    rep = grade(_required_stiffness(task), p, manufacturing=False, optional={"STIFFNESS"}, score=False)
    s = next(c for c in rep.checks if c.name == "STIFFNESS")
    assert (s.ok, s.error) == (False, False) and "could not be measured within" in s.reason


def test_stiffness_over_its_wall_cap_is_a_grader_error_not_a_verdict(gen0, tmp_path, monkeypatch):
    import jointbench.checks.stiffness as chk

    task, d, ref = gen0
    monkeypatch.setattr(chk, "WALL_CAP_S", 0.5)
    p = os.path.join(tmp_path, "part.step")
    write_step(ref, p)
    rep = grade(_required_stiffness(task), p, manufacturing=False, optional={"STIFFNESS"}, score=False)
    s = next(c for c in rep.checks if c.name == "STIFFNESS")
    assert (s.ok, s.error) == (None, True) and rep.outcome() == "error"
