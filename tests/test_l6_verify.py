"""Level 6 through the benchmark's own commands: ``jointbench verify`` grades an L6 folder, counts the check
against the limit and archives it (assembly file and every part file it lists) as at levels 1-5; the folder
gets a starter ``submission.py`` that is not recorded as the model's code; the brief names that command and
no reference folder; ``gen --setup flange --level 6`` has a route. The grader itself is replaced by a stand-in
here (it takes minutes and has its own tests)."""
import json
import os
import shutil

import pytest

from jointbench import ladder
from jointbench.checks.base import CheckResult
from jointbench.cli import main
from jointbench.grade import Report
from jointbench.moves.l6 import generate as G
from jointbench.moves.l6 import grade as L6G
from jointbench.moves.l6.starter import submission


@pytest.fixture(scope="module")
def folder(tmp_path_factory):
    """Flange L6 seed 0 from its stored draw (not graded), with the reference moved out as for a model run."""
    out = tmp_path_factory.mktemp("l6")
    task, d, start, asm = G.generate("flange", 0, check=False)
    f = G.write_folder(task, d, start, asm, str(out))
    sol = os.path.join(str(out), "_solutions", os.path.basename(f))
    shutil.move(os.path.join(f, "_solution"), sol)
    return f, sol


@pytest.fixture()
def ws(folder, tmp_path):
    """A fresh copy of the folder for each test, with the reference's answer in it."""
    f, sol = folder
    w = os.path.join(str(tmp_path), os.path.basename(f))
    shutil.copytree(f, w)
    for n in os.listdir(sol):
        if n.endswith(".step") or n == "assembly.json":
            shutil.copyfile(os.path.join(sol, n), os.path.join(w, n))
    return w


@pytest.fixture()
def fake_grader(monkeypatch):
    seen = []

    def grade_folder(task_dir, assembly=None, **kw):
        seen.append(assembly)
        ok = os.path.exists(assembly)
        return Report("t", ok, [CheckResult("ASSEMBLY_FILE", ok, "stand-in grader")])

    monkeypatch.setattr(L6G, "grade_folder", grade_folder)
    return seen


def _index(w):
    with open(os.path.join(w, "attempts.jsonl")) as f:
        return [json.loads(x) for x in f if x.strip()]


def test_brief_names_the_benchmark_command_and_no_reference(folder):
    text = open(os.path.join(folder[0], "README.md")).read()
    assert "_solution" not in text
    assert "jointbench verify <this folder>" in text and "python -m jointbench.moves" not in text
    assert "`submission.py`" in text


def test_starter_is_written_and_runs(folder, tmp_path):
    f = folder[0]
    assert open(os.path.join(f, "submission.py")).read() == submission()
    w = shutil.copytree(f, os.path.join(str(tmp_path), "w"))
    import subprocess
    import sys

    r = subprocess.run([sys.executable, "submission.py"], cwd=w, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    asm = json.load(open(os.path.join(w, "assembly.json")))
    assert asm["parts"][0]["file"] == "p1.step" and os.path.exists(os.path.join(w, "p1.step"))


def test_verify_counts_and_archives_every_part(ws, fake_grader, monkeypatch, capsys):
    monkeypatch.setenv("JOINTBENCH_MAX_CHECKS", "2")
    assert main(["verify", ws, "--by", "test"]) == 0
    assert fake_grader == [os.path.join(ws, "assembly.json")]
    rows = _index(ws)
    assert len(rows) == 1 and rows[0]["passed"] is True and rows[0]["file"] == "assembly.json"
    arch = os.path.join(ws, rows[0]["archive"])
    asm = json.load(open(os.path.join(ws, "assembly.json")))
    assert os.path.exists(os.path.join(arch, "assembly.json"))
    for p in asm["parts"]:
        assert os.path.exists(os.path.join(arch, p["file"]))
    assert "script" not in rows[0]["files"]  # the untouched starter is nobody's code
    assert main(["verify", ws, "--by", "test"]) == 0
    capsys.readouterr()
    assert main(["verify", ws, "--by", "test"]) == 4  # the third: refused before grading
    assert "refused" in capsys.readouterr().out
    assert len(fake_grader) == 2 and len(_index(ws)) == 2


def test_verify_without_an_answer_fails_and_counts(ws, fake_grader):
    os.remove(os.path.join(ws, "assembly.json"))
    assert main(["verify", ws, "--max-checks", "6"]) == 1
    assert len(_index(ws)) == 1


def test_own_script_is_archived(ws, fake_grader):
    with open(os.path.join(ws, "submission.py"), "a") as f:
        f.write("# changed\n")
    main(["verify", ws])
    assert "script" in _index(ws)[0]["files"]


def test_gen_routes_level_6(monkeypatch, tmp_path, capsys):
    assert ladder.resolve("flange", 6) == ("moves", 6)
    calls = []
    monkeypatch.setattr(G, "generate", lambda setup, seed, **kw: calls.append((setup, seed)) or ("t", {}, {}, {}))
    monkeypatch.setattr(G, "write_folder", lambda *a, **kw: "folder")
    monkeypatch.setattr(G, "broken_variants", lambda *a, **kw: [])
    assert main(["gen", "--setup", "flange", "--level", "6", "--seed", "0", "--out", str(tmp_path)]) == 0
    assert calls == [("flange", 0)]
