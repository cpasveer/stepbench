"""The tool layer: starters, notes and builds per CAD tool (jointbench.tools)."""
import json
import os
import stat

import build123d as bd
import pytest

from jointbench import tools
from jointbench.cli import main
from jointbench.geometry import write_step
from jointbench.levels import level1
from jointbench.tools.__main__ import main as tools_main
from jointbench.workspace import write_workspace

needs_freecad = pytest.mark.skipif(tools.freecadcmd() is None, reason="FreeCAD not installed")


@pytest.fixture()
def ws(tmp_path):
    task, _, ref = level1.generate(0)
    path = write_workspace(task, ref, str(tmp_path / "tasks"))
    ref_step = str(tmp_path / "reference.step")
    write_step(ref, ref_step)
    return path, ref_step


def _surfaces(step):
    s = bd.import_step(step)
    return len(s.solids()), {str(f.geom_type).split(".")[-1].lower() for f in s.faces()}


def test_task_scad_carries_every_bolt_at_full_precision(ws):
    path, _ = ws
    task = json.load(open(os.path.join(path, "task.json")))
    text = tools.task_scad(task)
    for b in task["poses"][0]["resolved"]["bolts"]:
        assert f'"{b["id"]}"' in text
        assert repr(float(b["position"][0])) in text
    assert text.count("[") > 10 and "FACES = [" in text and "BOLTS = [" in text


@pytest.mark.parametrize("tool,starter", [("freecad", "submission_freecad.py"), ("openscad", "submission.scad")])
def test_prepare_writes_one_starting_point_and_a_note(ws, tool, starter):
    path, _ = ws
    assert tools_main(["prepare", path, "--tool", tool, "--runner", "uv run --project /x"]) == 0
    names = set(os.listdir(path))
    assert starter in names and "TOOL.md" in names
    assert "submission.py" not in names  # the untouched build123d stub is gone
    note = open(os.path.join(path, "TOOL.md")).read()
    assert f"uv run --project /x python -m jointbench.tools build . --tool {tool}" in note
    assert f"uv run --project /x jointbench verify . --tool {tool} --script {starter}" in note
    assert "BUILD_CMD" not in open(os.path.join(path, starter)).read()
    if tool == "openscad":
        assert "task.scad" in names and "include <task.scad>" in open(os.path.join(path, starter)).read()


def test_prepare_keeps_an_edited_build123d_script(ws):
    path, _ = ws
    sub = os.path.join(path, "submission.py")
    with open(sub, "a") as f:
        f.write("# my edit\n")
    tools.prepare(path, "freecad")
    assert os.path.exists(sub)


def test_build123d_prepare_writes_only_the_note(ws):
    path, _ = ws
    written = tools.prepare(path, "build123d")
    assert [os.path.basename(p) for p in written] == ["TOOL.md"]
    assert os.path.exists(os.path.join(path, "submission.py"))


def test_unprovided_tool_says_why():
    with pytest.raises(tools.ToolMissing, match="STEP"):
        tools.get("anvil")


def test_build123d_build_then_grade_passes(ws):
    path, ref_step = ws
    with open(os.path.join(path, "submission.py"), "w") as f:
        f.write("import pathlib, shutil\n"
                f"shutil.copy({ref_step!r}, pathlib.Path(__file__).parent / 'output.step')\n")
    assert tools_main(["build", path, "--tool", "build123d"]) == 0
    assert main(["verify", path, "--tool", "build123d", "--no-log"]) == 0


@needs_freecad
def test_freecad_route_passes_the_grader(ws):
    """The reference answer read into FreeCAD and written out by FreeCAD still passes."""
    path, ref_step = ws
    tools.prepare(path, "freecad")
    starter = os.path.join(path, "submission_freecad.py")
    text = open(starter).read().replace(
        "    return Part.makeBox(10, 10, 10)", f"    return Part.read({ref_step!r})")
    open(starter, "w").write(text)
    assert tools_main(["build", path, "--tool", "freecad"]) == 0
    assert main(["verify", path, "--tool", "freecad", "--no-log"]) == 0
    rec = json.loads(open(os.path.join(path, "attempts.jsonl")).read().splitlines()[-1]) \
        if os.path.exists(os.path.join(path, "attempts.jsonl")) else None
    assert rec is None  # --no-log


@needs_freecad
def test_freecad_failure_gives_a_traceback_and_leaves_no_file(ws, capsys):
    path, _ = ws
    tools.prepare(path, "freecad")
    with open(os.path.join(path, "submission_freecad.py"), "a") as f:
        f.write("\nraise ValueError('deliberate')\n")
    assert tools_main(["build", path, "--tool", "freecad"]) == 1
    err = capsys.readouterr().err
    assert "line" in err and "ValueError: deliberate" in err
    assert not os.path.exists(os.path.join(path, "output.step"))


CSG = """difference() {
\tcube(size = [20, 20, 5], center = false);
\tmultmatrix([[1, 0, 0, 10], [0, 1, 0, 10], [0, 0, 1, -1], [0, 0, 0, 1]]) {
\t\tcylinder($fn = FN, $fa = 12, $fs = 2, h = 7, r1 = 2, r2 = 2, center = false);
\t}
}
"""


@needs_freecad
@pytest.mark.parametrize("fn,round_", [(0, True), (12, False), (16, False), (17, True), (64, True)])
def test_openscad_csg_conversion_keeps_circles_round_unless_fn_is_small(ws, tmp_path, monkeypatch, fn, round_):
    """OpenSCAD's own step (.scad -> .csg) is stood in for by a script that
    writes a CSG file in OpenSCAD's export format; the FreeCAD half is real."""
    path, _ = ws
    tools.prepare(path, "openscad")
    csg = tmp_path / "in.csg"
    csg.write_text(CSG.replace("FN", str(fn)))
    fake = tmp_path / "openscad"
    fake.write_text(f'#!/bin/sh\ncp "{csg}" "$2"\n')
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    monkeypatch.setenv("JOINTBENCH_OPENSCAD", str(fake))
    out = tools.build(path, "openscad")
    n, kinds = _surfaces(out)
    assert n == 1
    assert ("cylinder" in kinds) is round_


needs_solidcraft = pytest.mark.skipif(tools.solidcraft() is None, reason="SolidCraft not installed")


@needs_solidcraft
@pytest.mark.parametrize("plate_turn,axis,kind", [
    (None, [0, 0, 1], "cylinder"),  # hole square to the plate
    ("30 deg", [0, -0.5, 0.8660254], "cylinder"),  # plate and hole tilted together: still square
    (None, [0, 0.5, 0.8660254], "bspline"),  # hole at a slant to the plate: SolidCraft's STEP writer
])
def test_solidcraft_route_and_its_slanted_hole_limit(ws, plate_turn, axis, kind):
    """Known limit, SolidCraft 0.30.1: a hole that meets a flat face at a slant
    is a cylinder inside SolidCraft but is written to STEP as a B-spline
    surface, which HOLES rejects. If this test starts failing on 'bspline',
    SolidCraft has fixed it; update docs/tools.md and the note."""
    path, _ = ws
    tools.prepare(path, "solidcraft")
    assert "submission.py" not in os.listdir(path)
    note = open(os.path.join(path, "TOOL.md")).read()
    assert "SOLIDCRAFT_CLI" not in note and "solidcraft-cli" in note
    script = [{"command": "solid.box", "params": {"length": 30, "width": 30, "height": 5,
                                                  "center": [0, 0, 0], "body_name": "part"}}]
    if plate_turn:
        script.append({"command": "solid.move", "params": {"bodies": ["part"], "axis": [1, 0, 0],
                                                           "angle": plate_turn}})
    base = [-10 * a for a in axis]
    script.append({"command": "solid.cylinder", "params": {
        "radius": 2, "height": 20, "base": base, "axis": axis, "operation": "cut", "targets": ["part"]}})
    json.dump(script, open(os.path.join(path, "submission_solidcraft.json"), "w"))
    n, kinds = _surfaces(tools.build(path, "solidcraft"))
    assert n == 1 and kinds == {"plane", kind}


@needs_solidcraft
def test_solidcraft_failure_names_the_step_and_leaves_no_file(ws, capsys):
    path, _ = ws
    tools.prepare(path, "solidcraft")
    json.dump([{"command": "solid.cylinder", "params": {"radius": 2, "height": 5, "operation": "cut",
                                                        "targets": ["nobody"]}}],
              open(os.path.join(path, "submission_solidcraft.json"), "w"))
    assert tools_main(["build", path, "--tool", "solidcraft"]) == 1
    assert "targets" in capsys.readouterr().err
    assert not os.path.exists(os.path.join(path, "output.step"))
