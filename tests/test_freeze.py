import json
import os

import pytest

from jointbench import freeze as fz
from jointbench.traces import records

FAST = {1: {"stiffness": 0}}  # tests not about stiffness skip its analysis


def test_freeze_writes_manifest_marks_and_detects_changes(tmp_path):
    out = str(tmp_path / "trial")
    man = fz.freeze(out, "trial", 1, {1: 2}, FAST, seed=7000)
    assert man["visibility"] == "public" and [t["id"] for t in man["tasks"]] == ["L1-s7000", "L1-s7001"]
    assert fz.check(out) == []
    # a grading the grader could not complete is kept under errors/; the task is unchanged
    os.makedirs(os.path.join(out, "L1-s7000", "errors", "0001"))
    with open(os.path.join(out, "L1-s7000", "errors.jsonl"), "w") as f:
        f.write("{}\n")
    assert fz.check(out) == []
    mark = json.load(open(os.path.join(out, "L1-s7000", "set.json")))
    assert mark["set"] == "trial" and mark["fingerprint"] == man["tasks"][0]["fingerprint"]
    with open(os.path.join(out, "L1-s7001", "task.json"), "a") as f:
        f.write(" ")
    assert fz.check(out) == ["L1-s7001: files changed: task.json"]
    with pytest.raises(FileExistsError):
        fz.freeze(out, "trial", 1, {1: 1}, FAST)


def test_hidden_seeds_come_from_the_secret_and_avoid_public_sets(tmp_path):
    pub = str(tmp_path / "pub")
    fz.freeze(pub, "p", 1, {1: 1}, FAST, seed=7100)
    a = [fz.hidden_seed("s1", 1, i) for i in range(3)]
    assert a == [fz.hidden_seed("s1", 1, i) for i in range(3)] and a != [fz.hidden_seed("s2", 1, i) for i in range(3)]
    assert min(a) >= 10_000_000
    with pytest.raises(ValueError):
        fz.freeze(str(tmp_path / "clash"), "c", 1, {1: 1}, FAST, seed=7100, avoid=[pub])


def test_set_membership_reaches_the_archive_and_traces(tmp_path):
    from jointbench.cli import main

    out = str(tmp_path / "trial")
    fz.freeze(out, "trial", 3, {1: 1}, FAST, seed=7200)
    ws = os.path.join(out, "L1-s7200")
    main(["verify", ws])
    rec = json.loads(open(os.path.join(ws, "attempts.jsonl")).readline())
    assert rec["task_set"]["set"] == "trial" and rec["task_set"]["version"] == 3
    assert fz.check(out) == []  # grading does not disturb the fingerprint
    r = next(records(out))
    assert r["task_set"]["set"] == "trial"
