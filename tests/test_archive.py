import json
import os

import build123d as bd
import pytest

from jointbench.cli import main
from jointbench.geometry import write_step
from jointbench.levels import level1
from jointbench.traces import records
from jointbench.workspace import write_workspace


@pytest.fixture()
def ws(tmp_path):
    task, d, ref = level1.generate(0, {"stiffness": 0})
    run = tmp_path / "runs" / "model-x"
    path = write_workspace(task, ref, str(run))
    return path, ref


def _index(ws_path):
    with open(os.path.join(ws_path, "attempts.jsonl")) as f:
        return [json.loads(x) for x in f if x.strip()]


def test_every_attempt_is_archived_with_its_own_file(ws, tmp_path):
    path, ref = ws
    write_step(bd.Box(10, 10, 10), os.path.join(path, "output.step"))
    assert main(["verify", path, "--by", "agent"]) == 1
    write_step(ref, os.path.join(path, "output.step"))  # overwrites the failed try
    assert main(["verify", path, "--by", "agent"]) == 0
    idx = _index(path)
    assert [r["archive"] for r in idx] == ["attempts/0001", "attempts/0002"]
    a1, a2 = (os.path.join(path, r["archive"], "output.step") for r in idx)
    assert open(a1, "rb").read() != open(a2, "rb").read()  # the failed try survived
    assert idx[0]["sha256"] != idx[1]["sha256"]
    fb = open(os.path.join(path, "attempts/0001/feedback.txt")).read()
    assert fb.startswith("FAIL") and "HOLES" in fb
    assert json.load(open(os.path.join(path, "attempts/0002/result.json")))["passed"] is True


def test_untouched_stub_is_not_archived_as_code(ws):
    path, ref = ws
    write_step(ref, os.path.join(path, "output.step"))
    main(["verify", path])
    assert "script" not in _index(path)[0]["files"]


def test_note_tool_and_reported_figures_stored_as_given(ws, tmp_path):
    path, ref = ws
    write_step(ref, os.path.join(path, "output.step"))
    note = tmp_path / "note.txt"
    note.write_text("first I placed the plates\nthen the bridge")
    main(["verify", path, "--by", "agent", "--tool", "cadquery", "--note-file", str(note),
          "--tokens-in", "1500", "--cost", "0.02", "--currency", "EUR"])
    r = _index(path)[0]
    assert r["tool"] == "cadquery"
    assert r["reported"] == {"tokens_in": 1500, "cost": 0.02, "currency": "EUR"}  # nothing invented
    assert open(os.path.join(path, r["files"]["note"])).read() == note.read_text()
    assert r["step_header"]["originating_system"] == "build123d"  # read from the file, not the flag


def test_traces_orders_attempts_and_reads_old_logs(ws, tmp_path):
    path, ref = ws
    old = {"n": 1, "time": "2026-11-01T10:00:00+00:00", "by": "agent", "task": "L1-s0", "passed": False,
           "checks": [{"name": "FILE_VALID", "status": "fail", "reason": "no file at output.step",
                       "value": None, "unit": ""}], "info": {}}
    with open(os.path.join(path, "attempts.jsonl"), "w") as f:
        f.write(json.dumps(old) + "\n")
    sub = os.path.join(path, "submission.py")
    with open(sub, "a") as f:
        f.write("\n# edited by the agent\n")
    write_step(ref, os.path.join(path, "output.step"))
    main(["verify", path, "--by", "agent"])
    main(["verify", path, "--by", "human", "--tool", "freecad", "--no-script"])
    (tmp_path / "runs" / "model-x" / "run.json").write_text('{"name": "Model X"}')
    recs = list(records(str(tmp_path / "runs")))
    assert [r["by"] for r in recs] == ["agent", "human"]
    agent = recs[0]
    assert agent["run"] == {"name": "Model X"}
    assert [a["n"] for a in agent["attempts"]] == [1, 2]
    assert agent["attempts"][0]["feedback_reconstructed"] is True
    assert agent["attempts"][0]["feedback"].startswith("FAIL")
    assert agent["attempts"][1]["code"].endswith("# edited by the agent\n")
    assert agent["attempts"][1]["code_link"] == "found"
    assert agent["summary"]["first_pass_attempt"] == 2
    assert recs[1]["attempts"][0]["code"] is None
    assert agent["brief"].startswith("# Task L1-s0")
    # the reference is never exported (the brief only warns that it exists)
    dump = json.dumps(recs)
    assert "reference.step" not in dump and "reference.json" not in dump


def test_check_limit_refuses_the_check_after_the_last(ws, monkeypatch, capsys):
    """With a limit in force, verify grades up to the limit and then refuses
    before measuring anything; an unlogged check is refused outright."""
    path, ref = ws
    write_step(bd.Box(10, 10, 10), os.path.join(path, "output.step"))
    assert main(["verify", path, "--max-checks", "2", "--skip-reported"]) == 1
    assert main(["verify", path, "--max-checks", "2", "--no-log"]) == 4
    monkeypatch.setenv("JOINTBENCH_MAX_CHECKS", "2")
    write_step(ref, os.path.join(path, "output.step"))
    assert main(["verify", path, "--skip-reported"]) == 0
    capsys.readouterr()
    assert main(["verify", path]) == 4  # the third: refused, nothing graded
    assert "refused" in capsys.readouterr().out
    assert len(_index(path)) == 2
    os.remove(os.path.join(path, "attempts.jsonl"))  # deleting the log gives nothing back
    assert main(["verify", path]) == 4
    monkeypatch.delenv("JOINTBENCH_MAX_CHECKS")
    assert main(["verify", path, "--no-log", "--skip-reported"]) == 0  # no limit: as before
