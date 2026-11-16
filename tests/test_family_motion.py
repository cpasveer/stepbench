"""The planar mechanisms as task families: generation through the core, grading
through the core grader, one pass and one fail per check."""
import filecmp
import os

import pytest

from jointbench import families
from jointbench.geometry import write_step
from jointbench.grade import grade
from jointbench.task import Task
from jointbench.workspace import write_workspace

MECHS = ["fourbar", "slider", "yoke", "oldham"]


@pytest.mark.parametrize("name", MECHS)
def test_registered(name):
    fam = families.get(name)
    assert fam.pose_answer is not None and fam.core_checks == {"NO_INTERFERENCE"}


def _folder(name, level, seed, out):
    mod = families.level(name, level)
    task, draw, ref = mod.generate(seed)
    return write_workspace(task, ref, str(out), {"draw": draw}), mod, task, draw


def test_folder_is_deterministic(tmp_path):
    a, *_ = _folder("fourbar", 1, 2, tmp_path / "a")
    b, *_ = _folder("fourbar", 1, 2, tmp_path / "b")
    cmp = filecmp.dircmp(a, b)
    assert not cmp.diff_files and not cmp.left_only and not cmp.right_only


@pytest.fixture(scope="module")
def slider_task(tmp_path_factory):
    out = tmp_path_factory.mktemp("slider")
    folder, mod, task, draw = _folder("slider", 2, 1, out)
    return folder, mod, Task.load(os.path.join(folder, "task.json")), draw


def _grade(folder, mod, task, draw, tmp, **variant):
    path = os.path.join(tmp, "v.step")
    write_step(mod.build_connector(task, draw, **variant), path)
    return grade(task, path, manufacturing=False, optional=False, score=False)


def test_reference_passes_from_the_folder(slider_task, tmp_path):
    rep = _grade(*slider_task, tmp_path)
    assert rep.passed, [c.reason for c in rep.checks if c.ok is False]
    names = [c.name for c in rep.checks]
    for n in ("POSES", "NO_INTERFERENCE", "JOINTS", "ASSEMBLES", "TRANSMISSION", "OUTPUT", "MOTION_CLEAR"):
        assert n in names


@pytest.mark.parametrize("variant,check", [
    ({"variant": "shift", "d": 2.0}, "OUTPUT"),
    ({"variant": "fouls"}, "MOTION_CLEAR"),
    ({"variant": "jams"}, "POSES"),
    ({"variant": "straight"}, "MOTION_CLEAR"),
])
def test_broken_variants_fail_on_their_check(slider_task, tmp_path, variant, check):
    rep = _grade(*slider_task, tmp_path, **variant)
    assert not rep.passed and check in rep.failed()


def test_neighbours_follow_the_answer(slider_task, tmp_path):
    """With pose_scene the core sees the scene the answer makes: a rod 2 mm
    long misses the table, but its neighbours move with it, so nothing collides."""
    rep = _grade(*slider_task, tmp_path, variant="shift", d=2.0)
    failed = rep.failed()
    assert "OUTPUT" in failed and "NO_INTERFERENCE" not in failed


def _numbers(obj):
    if isinstance(obj, dict):
        for v in obj.values():
            yield from _numbers(v)
    elif isinstance(obj, list):
        for v in obj:
            yield from _numbers(v)
    elif isinstance(obj, (int, float)) and not isinstance(obj, bool):
        yield float(obj)


def test_level3_task_json_withholds_the_answer(tmp_path):
    """At level 3 (path synthesis, the rocker drawn parked) task.json holds only
    what the brief states: no joint graph, no generator record, one drawn pose,
    and none of the numbers the agent must find."""
    import json
    import math

    from jointbench.motion import family as F, kin

    folder, mod, task, draw = _folder("fourbar", 3, 4, tmp_path)
    with open(os.path.join(folder, "task.json")) as f:
        raw = f.read()
    t = json.loads(raw)
    mt = t["meta"]["motion"]
    assert "mechanism" not in mt and "generation" not in mt and "_full" not in mt
    assert "via" not in raw and "h1-mark" not in raw
    assert len(t["poses"]) == 1
    # the withheld numbers: the coupler's pin spacing and mark offset, and the
    # rocker's true pin position and angle at the first input value
    m = F._mechanism("fourbar", task)
    cf = m.parts["coupler"].features
    secret = [math.dist(cf["h1"].xy, cf["h2"].xy), math.dist(cf["h1"].xy, cf["mark"].xy),
              math.dist(cf["h2"].xy, cf["mark"].xy)]
    sol, _ = kin.sweep(m, [m.range[0]])
    pin = kin.world_pt(m, sol[0].poses, "rocker", "pin")
    secret += list(m.frame.point((pin[0], pin[1], 10.0)))[:3] + list(pin)
    secret.append(sol[0].poses["rocker"][2])
    nums = list(_numbers(t))
    for v in secret:
        assert not any(abs(n - v) < 1e-6 for n in nums), v
    # and the grader rebuilds what it needs from seed, knobs and attempt alone
    F._CACHE.clear()
    F._GRADER.clear()
    rebuilt = F.grader_data(Task.load(os.path.join(folder, "task.json")))
    assert rebuilt["table"]["world"] == mt["table"]["world"]


def test_grading_ignores_the_solution_folder(tmp_path):
    """The grader reads nothing from _solution: with it removed (a redacted
    copy), the reference still grades from task.json alone."""
    import shutil

    from jointbench.motion import family as F

    folder, mod, task, draw = _folder("slider", 3, 2, tmp_path)
    ref = os.path.join(folder, "_solution", "reference.step")
    out = os.path.join(folder, "output.step")
    shutil.copy(ref, out)
    shutil.rmtree(os.path.join(folder, "_solution"))
    F._CACHE.clear()
    F._GRADER.clear()
    rep = grade(Task.load(os.path.join(folder, "task.json")), out, manufacturing=False, optional=False, score=False)
    assert rep.passed, [c.reason for c in rep.checks if c.ok is False]


def test_submission_stub_is_the_mechanism_one(slider_task):
    folder = slider_task[0]
    with open(os.path.join(folder, "submission.py")) as f:
        text = f.read()
    assert "meta" in text and "motion" in text and "bolt" not in text.lower()
    assert not os.path.exists(os.path.join(folder, "_solution", "grader.json"))


def test_motion_poses_for_the_viewer(slider_task):
    """The viewer's motion: the reference motion from task.json alone matches
    the stored pose, and the motion the reference answer drives matches it at
    every table row, the moved part included."""
    import math

    from jointbench.motion import family as F

    folder = slider_task[0]
    t = Task.load(os.path.join(folder, "task.json"))
    rem = t.meta["motion"]["removed"]
    ref = F.motion_poses(t)
    assert len(ref) == len(t.meta["motion"]["table"]["inputs"]) and rem in ref[0]
    q = (13.0, -7.0, 4.0)
    for p in t.parts:
        assert math.dist(ref[0][p].point(q), t.poses[0][p].point(q)) < 1e-9
    ans = F.motion_poses(t, answer=os.path.join(folder, "_solution", "reference.step"))
    v = F.reference_solid(t).vertices()[0]
    v = (v.X, v.Y, v.Z)
    for k in (0, len(ref) // 2):
        for p in t.parts:
            assert math.dist(ref[k][p].point(q), ans[k][p].point(q)) < 1e-6
        assert math.dist(ref[k][rem].point(v), ans[k][rem].point(ref[0][rem].point(v))) < 1e-6
    assert len(F.motion_poses(t, values=[0.0, 90.0, 180.0])) == 3


def test_guided_at_its_limit(tmp_path):
    """GUIDED, two guide blocks: a bar just inside both passes; one that
    leaves a block during the stroke fails GUIDED (it still poses). The brief
    states the blocks and the rule."""
    folder, mod, task, draw = _folder("yoke", 2, 1, tmp_path)
    t = Task.load(os.path.join(folder, "task.json"))
    ok = _grade(folder, mod, t, draw, tmp_path, variant="guided", d=1.0)
    assert ok.passed, [c.reason for c in ok.checks if c.ok is False]
    bad = _grade(folder, mod, t, draw, tmp_path, variant="guided", d=-1.0)
    assert not bad.passed and bad.failed() == ["GUIDED"], bad.failed()
    with open(os.path.join(folder, "README.md")) as f:
        text = f.read()
    assert "`GUIDED`" in text and "2 separate blocks" in text and "1.5 x its width" in text


def test_old_slider_folders_keep_their_geometry():
    """Version 3 lengthens the crank-slider's slider block; a task made at
    version 2 is rebuilt with version 2, so its fixed parts stay as drawn."""
    from jointbench.motion import generate
    _, m2, _ = generate.make("slider", 1, {"remove": "rod"}, version=2)
    _, m3, _ = generate.make("slider", 1, {"remove": "rod"})
    assert m2.parts["slider"].features["tongue"].along[1] == 9.0
    assert m3.parts["slider"].features["tongue"].along[1] == 12.5
