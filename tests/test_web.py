"""The viewer's Python side: mesh export, data documents, play server, static export.

None of this grades. These tests check that what the page is given is
complete and faithful: meshes with true edges, verdicts copied unchanged
from attempts.jsonl, uploads graded by the real grader, and a static export
that holds every file the page asks for.
"""
import json
import os
import shutil
import threading
import urllib.error
import urllib.request
from base64 import b64decode
from array import array

import build123d as bd
import pytest

from jointbench.cli import main
from jointbench.web import data
from jointbench.web.export import export
from jointbench.web.mesh import MeshCache, file_sha256, mesh_shape, mesh_step
from jointbench.web.server import State, make_handler, site_dir


def _floats(s):
    return array("f", b64decode(s))


def _ints(s):
    return array("I", b64decode(s))


@pytest.fixture(scope="module")
def tasks(tmp_path_factory):
    """Two tasks; s0 has a failing baseline and a passing reference attempt,
    s1 has none."""
    root = tmp_path_factory.mktemp("tasks")
    assert main(["gen", "--seed", "0", "--n", "2", "--out", str(root)]) == 0
    t0 = os.path.join(root, "L1-s0")
    main(["solve", "-q", t0])  # fails by design, logs attempt 1
    ref = os.path.join(t0, "_solution", "reference.step")
    assert main(["verify", t0, "--file", ref, "--who", "reference"]) == 0  # attempt 2
    return str(root)


# ---- mesh ------------------------------------------------------------------


def test_mesh_has_triangles_edges_and_bbox():
    m = mesh_shape(bd.Box(10, 20, 30))
    pos, idx, edges = _floats(m["positions"]), _ints(m["indices"]), _floats(m["edges"])
    assert m["counts"]["faces"] == 6
    assert len(idx) % 3 == 0 and max(idx) < len(pos) // 3
    assert len(edges) == 12 * 6  # 12 straight edges, one segment each
    assert m["bbox"] == [[-5, -10, -15], [5, 10, 15]]


def test_cylinder_seam_is_not_drawn():
    """A cylinder has two circular edges and one seam; the seam is not a
    real edge, so every edge point lies on the top or bottom circle."""
    m = mesh_shape(bd.Cylinder(5, 10))
    e = _floats(m["edges"])
    zs = {round(e[i], 3) for i in range(2, len(e), 3)}
    assert zs == {-5.0, 5.0}


def test_mesh_triangles_face_outward():
    """Winding follows face orientation, so shading is not inside out."""
    m = mesh_shape(bd.Box(10, 10, 10))
    pos, idx = _floats(m["positions"]), _ints(m["indices"])
    vol = 0.0
    for t in range(0, len(idx), 3):
        a, b, c = (pos[3 * idx[t + k]:3 * idx[t + k] + 3] for k in range(3))
        vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6
    assert vol == pytest.approx(1000, rel=1e-3)


def test_mesh_cache_by_content(tmp_path, tasks):
    step = os.path.join(tasks, "L1-s0", "fixed_A.step")
    cache = MeshCache(str(tmp_path / "c"))
    sha = cache.put_step(step)
    assert sha == file_sha256(step) and cache.has(sha)
    assert json.loads(cache.get(sha))["counts"]["triangles"] > 0
    assert cache.put_step(str(tmp_path / "missing.step")) is None


def test_unreadable_step_gives_error_not_crash(tmp_path):
    bad = tmp_path / "bad.step"
    bad.write_text("not a step file")
    cache = MeshCache(str(tmp_path / "c"))
    sha = cache.put_step(str(bad))
    assert "error" in json.loads(cache.get(sha))


# ---- data documents ----------------------------------------------------------


def test_find_tasks(tasks):
    assert [os.path.basename(t) for t in data.find_tasks(tasks)] == ["L1-s0", "L1-s1"]
    one = os.path.join(tasks, "L1-s1")
    assert data.find_tasks(one) == [one]


def test_index_entry(tasks):
    e = data.index_entry(os.path.join(tasks, "L1-s0"))
    assert e["id"] == "L1-s0" and e["attempts"] == 2 and e["solved"] and e["solved_by"] == ["reference"]
    assert len(e["parts"]) == 2 and e["bolts"] == sum(p["holes"] for p in e["parts"])
    assert len(e["plan"]["bolts"]) == e["bolts"]
    assert data.index_entry(os.path.join(tasks, "L1-s1"))["solved"] is False


def test_scene_reads_counts_from_the_task(tasks, tmp_path):
    d = os.path.join(tasks, "L1-s0")
    tj = json.load(open(os.path.join(d, "task.json")))
    sc = data.scene(d, MeshCache(str(tmp_path / "c")))
    assert len(sc["poses"]) == len(tj["poses"])
    p0 = sc["poses"][0]
    assert sorted(p0["fixed"]) == sorted(fp["name"] for fp in tj["fixed_parts"])
    assert len(p0["bolts"]) == len(tj["poses"][0]["resolved"]["bolts"])
    assert {b["part"] for b in p0["bolts"]} == {"A", "B"}
    assert all(len(f["outline"]) == 4 for f in p0["faces"])
    assert sc["reference"] and "_solution" not in " ".join(sc["files"])
    assert "fixed_A.step" in sc["files"]


def test_attempts_copy_the_verdict_unchanged(tasks, tmp_path):
    d = os.path.join(tasks, "L1-s0")
    raw = data.read_attempts(d)
    doc = data.attempts_doc(d, MeshCache(str(tmp_path / "c")))
    for r, a in zip(raw, doc):
        assert a["checks"] == r["checks"] and a["passed"] == r["passed"]
        assert a["mesh"] == r["sha256"]
    assert [a["passed"] for a in doc] == [False, True]


def test_marks_only_for_reported_overlaps(tasks):
    d = os.path.join(tasks, "L1-s0")
    fail, ok = data.read_attempts(d)
    m = data.marks(d, fail, data.find_step(d, fail))
    reported = {r["what"] for c in fail["checks"] if c["name"] == "NO_INTERFERENCE"
                for r in c["details"]["overlaps"] if r["volume"] > 0.01}
    assert m["regions"] and {r["what"] for r in m["regions"]} <= reported
    assert data.marks(d, ok, data.find_step(d, ok)) == {"regions": []}


# ---- play server ---------------------------------------------------------------


@pytest.fixture()
def server(tasks, tmp_path):
    root = tmp_path / "copy"
    shutil.copytree(tasks, root)
    tdirs = data.find_tasks(str(root))
    state = State(str(root), tdirs)
    from http.server import ThreadingHTTPServer

    httpd = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(state, site_dir()))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{httpd.server_port}", str(root)
    httpd.shutdown()
    httpd.server_close()


def _get(url):
    with urllib.request.urlopen(url, timeout=60) as r:
        return r.status, r.read()


def test_server_serves_page_and_data(server):
    base, _ = server
    assert b"js/app.js" in _get(base + "/")[1]
    assert json.loads(_get(base + "/data/site.json")[1])["mode"] == "live"
    idx = json.loads(_get(base + "/data/index.json")[1])
    assert [t["id"] for t in idx] == ["L1-s0", "L1-s1"]
    sc = json.loads(_get(base + "/data/tasks/L1-s0/scene.json")[1])
    atts = json.loads(_get(base + "/data/tasks/L1-s0/attempts.json")[1])
    mesh = json.loads(_get(base + f"/data/mesh/{atts[-1]['mesh']}.json")[1])
    assert mesh["counts"]["triangles"] > 0 and sc["id"] == "L1-s0"
    assert _get(base + "/data/tasks/L1-s0/files/fixed_A.step")[1].startswith(b"ISO-10303-21")


@pytest.mark.parametrize("path", [
    "/data/tasks/L1-s0/files/..%2F..%2Fetc%2Fpasswd",
    "/data/tasks/L1-s0/files/_solution%2Freference.step",
    "/data/mesh/notasha.json",
    "/../pyproject.toml",
    "/.play/x",
])
def test_server_refuses_paths_outside_what_it_serves(server, path):
    base, _ = server
    with pytest.raises(urllib.error.HTTPError) as e:
        _get(base + path)
    assert e.value.code == 404


def _post(url, body=b""):
    req = urllib.request.Request(url, data=body, method="POST")
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.loads(r.read())


def test_upload_is_graded_and_kept_as_the_persons_own_run(server):
    """A person's uploads never enter the task's own log (in a runs folder
    that log is a model's). They form a run of their own, in the layout of a
    model run, with the time from starting the clock to the first pass
    measured by the server; `score` reads it as a human run."""
    from jointbench.web import score as sc

    base, root = server
    d = os.path.join(root, "L1-s1")
    before = data.read_attempts(d)
    st = _post(base + "/api/start?task=L1-s1&who=Ada%20T&tool=FreeCAD")
    key = st["key"]
    assert key == "human-ada-t__L1-s1" and st["clock"]["started"]
    hd = os.path.join(root, "human-runs", "human-ada-t", "L1-s1")
    assert sorted(os.listdir(hd)) == sorted(data.task_files(d) + [])  # what a person is given, nothing more
    assert not os.path.exists(os.path.join(hd, "_solution"))

    bad = _post(base + "/api/upload?task=L1-s1&who=Ada%20T&tool=FreeCAD", b"junk")
    assert bad["passed"] is False and bad["n"] == 1 and bad["key"] == key and "wall_seconds" not in bad["clock"]
    assert bad["checks"][0]["name"] == "FILE_VALID" and bad["checks"][0]["status"] == "fail"
    body = open(os.path.join(d, "_solution", "reference.step"), "rb").read()
    # trying again from the person's own copy of the task goes to the same run
    rec = _post(base + f"/api/upload?task={key}&who=Ada%20T&tool=FreeCAD", body)
    assert rec["passed"] is True and rec["n"] == 2 and rec["by"] == "Ada T" and rec["mesh"]
    assert rec["clock"]["first_pass"] == 2 and rec["clock"]["wall_seconds"] >= 0
    assert data.read_attempts(d) == before            # the task's own log is untouched
    logged = data.read_attempts(hd)
    assert [a["n"] for a in logged] == [1, 2] and logged[1]["tool"] == "FreeCAD"

    run = json.loads(open(os.path.join(root, "human-runs", "human-ada-t", "run.json")).read())
    assert run["human"] is True and run["name"] == "Ada T" and run["tool"] == "FreeCAD"
    scored = sc.score(os.path.join(root, "human-runs"))["runs"][0]
    t = scored["tasks"][0]
    assert scored["summary"]["passed_within"] == 1 and t["tries"] == 2
    assert t["reported"]["seconds"]["total"] == run["per_task"]["L1-s1"]["wall_seconds"]
    # the scores page now has the person's run, live
    assert json.loads(_get(base + "/data/scores.json")[1])["runs"][0]["id"] == "human-ada-t"
    # a start without a name is refused
    try:
        _post(base + "/api/start?task=L1-s1")
        raise AssertionError("expected 400")
    except urllib.error.HTTPError as e:
        assert e.code == 400


# ---- static export ---------------------------------------------------------------


def test_static_export_holds_every_file_the_page_asks_for(tasks, tmp_path):
    out = tmp_path / "site"
    assert main(["site", tasks, "--out", str(out)]) == 0
    assert (out / "index.html").exists()
    vers = list((out / "v").iterdir())
    assert len(vers) == 1 and (vers[0] / "vendor" / "three" / "LICENSE").exists()
    assert f'src="v/{vers[0].name}/js/app.js"' in (out / "index.html").read_text(encoding="utf-8")
    assert not (out / "screenshots").exists()
    site = json.loads((out / "data" / "site.json").read_text(encoding="utf-8"))
    assert site["mode"] == "static"
    idx = json.loads((out / "data" / "index.json").read_text(encoding="utf-8"))
    for t in idx:
        tdir = out / "data" / "tasks" / t["id"]
        sc = json.loads((tdir / "scene.json").read_text(encoding="utf-8"))
        if sc["reference"]:
            assert (out / "data" / "mesh" / f"{sc['reference']}.json").exists()
        for f in sc["files"]:
            assert (tdir / "files" / f).exists()
        for a in json.loads((tdir / "attempts.json").read_text(encoding="utf-8")):
            if a["mesh"]:
                assert (out / "data" / "mesh" / f"{a['mesh']}.json").exists()
            if a["marks"]:
                assert (tdir / "marks" / f"{a['n']}.json").exists()


def test_static_export_can_leave_the_reference_out(tasks, tmp_path):
    out = tmp_path / "site"
    assert export([tasks], str(out), include_reference=False, log=lambda *a: None) == 0
    sc = json.loads((out / "data" / "tasks" / "L1-s1" / "scene.json").read_text(encoding="utf-8"))
    assert sc["reference"] is None
    # s1 has no attempts, so with the reference left out no mesh of it remains
    ref_sha = file_sha256(os.path.join(tasks, "L1-s1", "_solution", "reference.step"))
    assert not (out / "data" / "mesh" / f"{ref_sha}.json").exists()


def test_mesh_step_reads_a_fixed_part(tasks):
    m = mesh_step(os.path.join(tasks, "L1-s0", "fixed_B.step"))
    assert m["counts"]["edge_segments"] > 0


# ---- scores ----------------------------------------------------------------------


def _log(task_dir, records):
    with open(os.path.join(task_dir, "attempts.jsonl"), "w") as f:
        for r in records:
            f.write(json.dumps(r) + "\n")


def _att(n, passed, failed=(), **extra):
    names = ["FILE_VALID", "HOLES", "SEATING", "NO_INTERFERENCE"]
    return {"n": n, "time": "2026-11-11T10:00:00+00:00", "by": "agent", "file": "output.step", "sha256": None,
            "task": "x", "passed": passed,
            "checks": [{"name": c, "status": "fail" if c in failed else "pass", "reason": "", "value": None,
                        "unit": "", "details": {}} for c in names], **extra}


@pytest.fixture(scope="module")
def runs(tasks, tmp_path_factory):
    """Two model runs over the same two tasks, with hand-written logs."""
    root = tmp_path_factory.mktemp("runs")
    for run in ("alpha", "beta"):
        for t in ("L1-s0", "L1-s1"):
            d = root / run / t
            shutil.copytree(os.path.join(tasks, t), d, ignore=shutil.ignore_patterns("attempts.jsonl", ".play"))
    # alpha: s0 first try (and a later try that must not count); s1 passes on try 3
    _log(root / "alpha" / "L1-s0", [_att(1, True, seconds=10, tool="build123d"), _att(2, False, ["HOLES"])])
    _log(root / "alpha" / "L1-s1", [_att(1, False, ["HOLES", "NO_INTERFERENCE"], seconds=5),
                                     _att(2, False, ["HOLES"], seconds=6), _att(3, True, seconds=7)])
    # beta: tries_allowed 2, so s1's pass on try 3 does not count; s0 never attempted
    (root / "beta" / "run.json").write_text(json.dumps({"name": "Beta model", "tries_allowed": 2, "tool": "cadquery",
                                                         "currency": "EUR"}))
    _log(root / "beta" / "L1-s1", [_att(1, False, ["SEATING"], cost=0.5), _att(2, False, ["SEATING"]),
                                    _att(3, True)])
    return str(root)


def test_score_counts_from_the_logs(runs):
    from jointbench.web.score import score

    s = score(runs)
    a, b = {r["id"]: r for r in s["runs"]}["alpha"], {r["id"]: r for r in s["runs"]}["beta"]
    sa = a["summary"]
    assert (sa["attempted"], sa["passed_first"], sa["passed_within"]) == (2, 1, 2)
    assert sa["avg_tries"] == 2.0  # 1 and 3
    assert sa["failed_checks"] == {"HOLES": 2, "NO_INTERFERENCE": 1}  # try 2 of s0 came after the pass
    assert sa["by_level"] == {"1": {"attempted": 2, "passed_first": 1, "passed_within": 2}}
    sb = b["summary"]
    assert b["name"] == "Beta model" and b["tries_allowed"] == 2 and b["tools"] == ["cadquery"]
    assert (sb["tasks"], sb["attempted"], sb["passed_within"], sb["avg_tries"]) == (2, 1, 0, None)


def test_reported_figures_stay_missing_not_zero(runs):
    from jointbench.web.score import score

    a = {r["id"]: r for r in score(runs)["runs"]}["alpha"]
    t1 = next(t for t in a["tasks"] if t["id"] == "L1-s1")
    assert t1["reported"]["seconds"] == {"total": 18.0, "of": 3, "attempts": 3}
    assert t1["reported"]["cost"]["total"] is None
    assert a["summary"]["reported"]["cost"]["total"] is None
    # time to pass is only summed for tasks where every counted try reported it
    assert a["summary"]["time_to_pass"] == {"total": 28.0, "tasks_with": 2, "tasks": 2}
    b = {r["id"]: r for r in score(runs)["runs"]}["beta"]
    assert b["currency"] == "EUR"


def test_score_command_and_scores_in_static_export(runs, tmp_path):
    out = tmp_path / "scores.json"
    assert main(["score", runs, "--out", str(out)]) == 0
    s = json.loads(out.read_text(encoding="utf-8"))
    assert [r["id"] for r in s["runs"]] == ["alpha", "beta"]
    site = tmp_path / "site"
    assert main(["site", runs, "--out", str(site)]) == 0
    exported = json.loads((site / "data" / "scores.json").read_text(encoding="utf-8"))
    assert "root" not in exported and json.loads((site / "data" / "site.json").read_text(encoding="utf-8"))["scores"]
    idx = json.loads((site / "data" / "index.json").read_text(encoding="utf-8"))
    assert {t["key"] for t in idx} == {"alpha__L1-s0", "alpha__L1-s1", "beta__L1-s0", "beta__L1-s1"}
    site = tmp_path / "site-dev"
    assert main(["site", runs, "--out", str(site), "--dev-runs"]) == 0
    exported = json.loads((site / "data" / "scores.json").read_text(encoding="utf-8"))
    assert len(exported["runs"]) == 2
    for r in exported["runs"]:
        for t in r["tasks"]:
            assert (site / "data" / "tasks" / t["key"] / "scene.json").exists()


def test_play_serves_live_scores_for_a_runs_folder(runs):
    from http.server import ThreadingHTTPServer

    keyed, src = data.collect([runs])
    state = State(runs, keyed, src)
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(state, site_dir()))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        base = f"http://127.0.0.1:{httpd.server_port}"
        s = json.loads(_get(base + "/data/scores.json")[1])
        assert len(s["runs"]) == 2
        atts = json.loads(_get(base + "/data/tasks/alpha__L1-s1/attempts.json")[1])
        assert [a["passed"] for a in atts] == [False, False, True]
        assert atts[0]["reported"]["seconds"] == 5 and atts[0]["reported"]["cost"] is None
    finally:
        httpd.shutdown()
        httpd.server_close()


# ---- attempt archive ---------------------------------------------------------------


def _pre_archive(task_dir):
    """Turn a task folder into what an old log looked like before the
    archive existed: no attempts/ folder, no archive fields in the index."""
    shutil.rmtree(os.path.join(task_dir, "attempts"), ignore_errors=True)
    recs = data.read_attempts(task_dir)
    with open(os.path.join(task_dir, "attempts.jsonl"), "w") as f:
        for r in recs:
            for k in ("archive", "files", "feedback_format"):
                r.pop(k, None)
            f.write(json.dumps(r) + "\n")
    return recs


def test_archive_gives_each_attempt_its_own_geometry_and_code(tasks, tmp_path):
    """Two attempts graded by `verify` as an agent does: the second
    overwrites output.step, yet each shows its own part, script, note and
    reported figures, and the STEP header's own originating system."""
    from http.server import ThreadingHTTPServer

    d = tmp_path / "L1-s1"
    shutil.copytree(os.path.join(tasks, "L1-s1"), d, ignore=shutil.ignore_patterns(".play"))
    ref = d / "_solution" / "reference.step"
    out = d / "output.step"
    scripts, notes = [], []
    for k, src in enumerate([os.path.join(tasks, "L1-s0", "fixed_A.step"), str(ref)], start=1):
        shutil.copy(src, out)  # the agent re-saves output.step each try
        sp = tmp_path / f"try{k}.py"
        sp.write_text(f"# try {k}\n")
        np_ = tmp_path / f"note{k}.txt"
        np_.write_text(f"reasoning for try {k}")
        assert main(["verify", str(d), "--by", "agent", "--script", str(sp), "--note-file", str(np_),
                     "--tool", "build123d", "--wall-seconds", str(10 * k), "--tokens-in", "1000",
                     "--cost", "0.01", "--currency", "USD", "--model", "m1"]) == (1 if k == 1 else 0)
    keyed = [("L1-s1", str(d))]
    state = State(str(d), keyed)
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(state, site_dir()))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        base = f"http://127.0.0.1:{httpd.server_port}"
        atts = json.loads(_get(base + "/data/tasks/L1-s1/attempts.json")[1])
        assert [a["passed"] for a in atts] == [False, True]
        assert atts[0]["mesh"] and atts[1]["mesh"] and atts[0]["mesh"] != atts[1]["mesh"]
        for k, a in enumerate(atts, start=1):
            arc = a["archive"]
            assert arc["dir"] == os.path.join("attempts", f"{k:04d}")
            assert arc["code"] == ["submission.py"] and arc["script_link"] == "named"
            assert arc["texts"] == ["feedback.txt"] and arc["note"] == "note.txt"
            assert _get(base + f"/data/tasks/L1-s1/archive/{k}/submission.py")[1] == f"# try {k}\n".encode()
            assert _get(base + f"/data/tasks/L1-s1/archive/{k}/note.txt")[1] == f"reasoning for try {k}".encode()
            assert b"PASS" in _get(base + f"/data/tasks/L1-s1/archive/{k}/feedback.txt")[1] or k == 1
            r = a["reported"]
            assert (r["tool"], r["seconds"], r["tokens_in"], r["tokens_out"], r["cost"], r["currency"], r["model"]) == \
                ("build123d", 10.0 * k, 1000.0, None, 0.01, "USD", "m1")
            assert a["step_origin"]  # measured from the file header
        with pytest.raises(urllib.error.HTTPError):
            _get(base + "/data/tasks/L1-s1/archive/1/output.step")  # only text is served from the archive
    finally:
        httpd.shutdown()
        httpd.server_close()


def test_lost_geometry_is_reported_as_missing(tasks, tmp_path):
    """Only a log from before the archive, whose file was overwritten since,
    has no shape; its verdict is kept."""
    d = tmp_path / "t"
    shutil.copytree(os.path.join(tasks, "L1-s0"), d, ignore=shutil.ignore_patterns(".play"))
    recs = _pre_archive(str(d))
    with open(d / recs[0]["file"], "w") as f:
        f.write("overwritten")
    atts = data.attempts_doc(str(d), MeshCache(str(tmp_path / "c")))
    assert atts[0]["mesh"] is None and atts[0]["archive"] is None
    assert atts[0]["checks"] == recs[0]["checks"]
    assert atts[-1]["mesh"]  # the reference file was not overwritten


# ---- manufacturing check fields -------------------------------------------------------


def test_dfm_fields_are_served_and_exported_never_invented(tasks, tmp_path):
    """The viewer shows a field file only when the checker wrote one; it is
    found by the graded file's sha256 and copied into a static export."""
    root = tmp_path / "copy"
    shutil.copytree(tasks, root, ignore=shutil.ignore_patterns(".play"))
    d = str(root / "L1-s0")
    att = data.read_attempts(d)[-1]
    procs = data.dfm_processes()
    assert "fdm" in procs
    assert data.dfm_available(d, att) == []
    fake = {"format": data.DFM_FORMAT, "version": 1, "process": "fdm", "passed": True,
            "direction": {"name": "build direction", "vector": [0, 0, 1], "chosen": "stated"},
            "mesh": {"vertices": [[0, 0, 0], [1, 0, 0], [0, 1, 0]], "triangles": [[0, 1, 2]], "edges": []},
            "fields": []}
    os.makedirs(data.dfm_cache_dir(d))
    with open(os.path.join(data.dfm_cache_dir(d), f"{att['sha256']}.fdm.fields.json"), "w") as f:
        json.dump(fake, f)
    assert data.dfm_available(d, att) == ["fdm"]
    doc = data.attempts_doc(d, MeshCache(str(tmp_path / "c")))
    assert doc[-1]["dfm"] == ["fdm"] and doc[0]["dfm"] == []
    out = tmp_path / "site"
    assert export([str(root)], str(out), log=lambda *a: None) == 0
    got = json.loads((out / "data" / "tasks" / "L1-s0" / "dfm" / f"{att['n']}.fdm.json").read_text(encoding="utf-8"))
    assert got == fake


def test_run_dfm_refuses_unknown_process_and_lost_files(tasks, tmp_path):
    d = str(tmp_path / "t")
    shutil.copytree(os.path.join(tasks, "L1-s0"), d, ignore=shutil.ignore_patterns(".play"))
    att = data.read_attempts(d)[-1]
    assert data.run_dfm(d, att, "laser-cutting")["ok"] is False
    att = _pre_archive(d)[-1]
    with open(os.path.join(d, att["file"]), "w") as f:
        f.write("overwritten")
    r = data.run_dfm(d, att, "fdm")
    assert r["ok"] is False and "not kept" in r["error"]


def test_a_folder_of_unattempted_tasks_is_not_a_run(runs, tasks, tmp_path):
    from jointbench.web.score import find_runs

    root = tmp_path / "runs"
    shutil.copytree(runs, root)
    shutil.copytree(os.path.join(tasks, "L1-s1"), root / "gen" / "L1-s1",
                    ignore=shutil.ignore_patterns("attempts.jsonl", "attempts", ".play"))
    assert [os.path.basename(r) for r in find_runs(str(root))] == ["alpha", "beta"]


def test_angled_faces_are_described_by_angle_not_gap(tasks, tmp_path):
    d = tmp_path / "t"
    shutil.copytree(os.path.join(tasks, "L1-s0"), d)
    e = data.index_entry(str(d))
    assert e["gap"] is not None and e["face_angle"] is None
    tj = json.loads((d / "task.json").read_text(encoding="utf-8"))
    tj["poses"][0]["resolved"]["mounting_faces"][1]["normal"] = [0.0, 1.0, 0.0]  # turned 90 degrees
    (d / "task.json").write_text(json.dumps(tj))
    e = data.index_entry(str(d))
    assert e["gap"] is None and e["face_angle"] == 90.0 and e["face_distance"] > 0


# ---- downloads ----------------------------------------------------------------------


def test_downloads_are_exact_step_and_valid_meshes(tasks, tmp_path):
    """STEP is the graded file byte for byte; STL and 3MF re-open as meshes
    of the same size, in millimetres."""
    import hashlib

    import build123d as bd

    from jointbench.geometry import read_step
    from jointbench.web import downloads

    d = os.path.join(tasks, "L1-s0")
    att = data.read_attempts(d)[-1]
    cache = str(tmp_path / "dl")
    step = downloads.get(d, f"attempt-{att['n']}.step", cache)
    assert hashlib.sha256(open(step, "rb").read()).hexdigest() == att["sha256"]
    solid = read_step(step).solids()[0]
    size = [round(v, 1) for v in solid.bounding_box().size]
    stl = downloads.get(d, f"attempt-{att['n']}.stl", cache)
    assert [round(v, 1) for v in bd.import_stl(stl).bounding_box().size] == size
    m = bd.Mesher()
    shapes = m.read(downloads.get(d, f"attempt-{att['n']}.3mf", cache))
    assert m.model_unit == bd.Unit.MM and [round(v, 1) for v in shapes[0].bounding_box().size] == size
    assert downloads.get(d, "fixed_A.stl", cache) and downloads.get(d, "fixed_B.3mf", cache)
    for bad in ("fixed_A.obj", "attempt-99.step", "../task.json", "fixed_Z.step"):
        assert downloads.get(d, bad, cache) is None


def test_static_export_carries_the_downloads(tasks, tmp_path):
    out = tmp_path / "site"
    assert main(["site", os.path.join(tasks, "L1-s0"), "--out", str(out)]) == 0
    dl = out / "data" / "tasks" / "L1-s0" / "download"
    names = sorted(p.name for p in dl.iterdir())
    for base in ("fixed_A", "fixed_B", "attempt-2"):
        for fmt in ("step", "stl", "3mf"):
            assert f"{base}.{fmt}" in names
    assert not (out / ".dlcache").exists()


def test_server_sends_downloads_with_a_file_name(server):
    base, _ = server
    with urllib.request.urlopen(base + "/data/tasks/L1-s0/download/fixed_A.3mf", timeout=120) as r:
        assert r.headers["Content-Type"] == "model/3mf"
        assert 'filename="L1-s0_fixed_A.3mf"' in r.headers["Content-Disposition"]
        assert r.read(2) == b"PK"  # a 3MF is a zip package
    with pytest.raises(urllib.error.HTTPError):
        _get(base + "/data/tasks/L1-s0/download/fixed_A.obj")


# ---- compare: submissions and thumbnails ----------------------------------------------


def test_submission_is_first_pass_within_allowed_else_last_allowed(runs, tmp_path):
    """Never the best attempt picked afterwards: a pass after the allowed
    tries does not count, and a later pass does not replace the first."""
    from jointbench.web.score import score

    s = score(runs)
    alpha = {t["id"]: t for t in {r["id"]: r for r in s["runs"]}["alpha"]["tasks"]}
    beta = {t["id"]: t for t in {r["id"]: r for r in s["runs"]}["beta"]["tasks"]}
    assert alpha["L1-s0"]["submission"]["n"] == 1 and alpha["L1-s0"]["submission"]["passed"]
    assert alpha["L1-s1"]["submission"]["n"] == 3 and alpha["L1-s1"]["submission"]["passed"]
    # beta allows 2 tries; its pass came on try 3, so its answer is try 2, a fail
    sub = beta["L1-s1"]["submission"]
    assert (sub["n"], sub["passed"], sub["failed"]) == (2, False, ["SEATING"])
    assert beta["L1-s0"]["submission"] is None  # never attempted
    assert {r["id"]: r["model"] for r in s["runs"]} == {"alpha": "alpha", "beta": "Beta model"}


def test_thumbnails_are_coarse_meshes_served_and_exported(tasks, tmp_path, server):
    base, _ = server
    t = json.loads(_get(base + "/data/tasks/L1-s0/thumb.json")[1])
    assert len(t["fixed"]) == 2 and all(m["counts"]["triangles"] > 0 for m in t["fixed"])
    full = data.scene(os.path.join(tasks, "L1-s0"), MeshCache(str(tmp_path / "c")))["poses"][0]["fixed"]
    # never finer than the full mesh (equal for fixed parts with no curved faces)
    assert sum(m["counts"]["triangles"] for m in t["fixed"]) <= sum(m["counts"]["triangles"] for m in full.values())
    atts = json.loads(_get(base + "/data/tasks/L1-s0/attempts.json")[1])
    m = json.loads(_get(base + f"/data/tasks/L1-s0/thumb/{atts[-1]['n']}.json")[1])
    assert m["counts"]["triangles"] > 0
    out = tmp_path / "site"
    assert main(["site", os.path.join(tasks, "L1-s0"), "--out", str(out)]) == 0
    assert (out / "data" / "tasks" / "L1-s0" / "thumb.json").exists()
    assert (out / "data" / "tasks" / "L1-s0" / "thumb" / f"{atts[-1]['n']}.json").exists()
    assert not (out / ".thumbcache").exists()


def test_assets_are_fingerprinted_so_no_stale_mix(server, tmp_path):
    """The page names a fingerprint of the scripts and styles; a change to
    any of them changes the path, so no cache can serve an old copy."""
    from jointbench.web.assets import fingerprint

    base, _ = server
    html = _get(base + "/")[1].decode()
    fp = fingerprint(site_dir())
    assert f'src="v/{fp}/js/app.js"' in html and f'href="v/{fp}/css/style.css"' in html
    with urllib.request.urlopen(base + f"/v/{fp}/js/app.js", timeout=30) as r:
        assert "immutable" in r.headers["Cache-Control"]
    with urllib.request.urlopen(base + "/", timeout=30) as r:
        assert r.headers["Cache-Control"] == "no-store"
    site = tmp_path / "s"
    shutil.copytree(site_dir(), site, ignore=shutil.ignore_patterns("screenshots"))
    before = fingerprint(str(site))
    (site / "js" / "md.js").write_text((site / "js" / "md.js").read_text(encoding="utf-8") + "\n// changed\n")
    assert fingerprint(str(site)) != before


def test_front_page_showcase_lists_real_attempts_and_falls_back_to_references(tasks, runs, tmp_path):
    """The front page shows real attempts with geometry, failures among them;
    a curated list wins; with no attempts, tasks with their reference."""
    keyed, _ = data.collect([runs])
    show = data.showcase(keyed, runs)
    assert show and all(e["n"] and e["key"] for e in show)
    assert any(e["passed"] is False and e["failed"] for e in show)
    # a curated list, by task id and attempt number
    some = show[-1]
    (tmp_path / "pick").mkdir()
    shutil.copytree(runs, tmp_path / "pick" / "r")
    (tmp_path / "pick" / "r" / "showcase.json").write_text(json.dumps([{"task": some["key"], "n": some["n"]}]))
    k2, _ = data.collect([str(tmp_path / "pick" / "r")])
    cur = data.showcase(k2, str(tmp_path / "pick" / "r"))
    assert [(e["key"], e["n"], e.get("curated")) for e in cur] == [(some["key"], some["n"], True)]
    # the static copy lists only what it carries, and falls back to references
    out = tmp_path / "site"
    assert main(["site", runs, "--out", str(out)]) == 0
    exported = json.loads((out / "data" / "showcase.json").read_text(encoding="utf-8"))
    for e in exported:
        a = next(x for x in json.loads((out / "data" / "tasks" / e["key"] / "attempts.json").read_text(encoding="utf-8")) if x["n"] == e["n"])
        assert (out / "data" / "mesh" / f"{a['mesh']}.json").exists()
    fresh = tmp_path / "fresh"
    shutil.copytree(os.path.join(tasks, "L1-s1"), fresh / "L1-s1", ignore=shutil.ignore_patterns("attempts.jsonl", "attempts", ".play"))
    k3, _ = data.collect([str(fresh)])
    ref = data.showcase(k3, str(fresh))
    assert ref and all(e.get("reference") for e in ref)


def test_scripts_and_meshes_are_sent_compressed_when_asked(server):
    """gzip for text, scripts and meshes, only when the browser accepts it;
    the bytes unpack to exactly what is served uncompressed."""
    import gzip
    import re

    base, _ = server
    fp = re.search(rb"v/([0-9a-f]{12})/js/app\.js", _get(base + "/")[1]).group(1).decode()
    url = f"{base}/v/{fp}/vendor/three/three.module.js"
    plain = _get(url)[1]
    req = urllib.request.Request(url, headers={"Accept-Encoding": "gzip"})
    with urllib.request.urlopen(req, timeout=60) as r:
        assert r.headers["Content-Encoding"] == "gzip"
        packed = r.read()
    assert len(packed) < len(plain) / 3 and gzip.decompress(packed) == plain


def test_run_json_figures_fill_in_where_the_log_has_none(runs, tmp_path):
    """Totals and per-task figures from run.json are used only where the
    attempts log reported nothing; a figure nobody reported stays null."""
    from jointbench.web import score as sc

    root = tmp_path / "r"
    shutil.copytree(runs, root)
    rj = root / "alpha" / "run.json"
    # run-level figures alone: read as the mean per task
    rj.write_text(json.dumps({"name": "Alpha", "wall_seconds": 900, "usage": {"input_tokens": 12000}}))
    alpha = next(r for r in sc.score(str(root))["runs"] if r["id"] == "alpha")
    rr = alpha["summary"]["run_reported"]
    assert rr["seconds"] == 900 and rr["tokens_in"] == 12000 and rr["cost"] is None
    # per-task figures ("tasks" is accepted beside "per_task")
    rj.write_text(json.dumps({"name": "Alpha", "tasks": {"L1-s0": {"tokens_out": 345}}}))
    alpha = next(r for r in sc.score(str(root))["runs"] if r["id"] == "alpha")
    s0 = next(t for t in alpha["tasks"] if t["id"] == "L1-s0")
    assert s0["reported"]["tokens_out"]["total"] == 345 and s0["reported"]["tokens_out"]["source"] == "run.json"
    # the log's own figure wins: s0's attempt 1 reported seconds=10
    assert s0["reported"]["seconds"]["total"] == 10 and "source" not in s0["reported"]["seconds"]


def test_tokens_total_and_per_task_figures_from_run_json(runs, tmp_path):
    """The model-run harness's run.json: run-level means, a per_task object,
    tokens only as a total. The total stays out of the input/output columns;
    run means come from the per-task figures; no cost is made up."""
    from jointbench.web import score as sc

    root = tmp_path / "r"
    shutil.copytree(runs, root)
    (root / "beta" / "run.json").write_text(json.dumps({
        "name": "Beta", "tries_allowed": 2, "wall_seconds": 999, "tokens_total": 1,
        "per_task": {"L1-s1": {"wall_seconds": 120.5, "tokens_total": 50000, "tool_calls": 7}}}))
    beta = next(r for r in sc.score(str(root))["runs"] if r["id"] == "beta")
    s1 = next(t for t in beta["tasks"] if t["id"] == "L1-s1")
    rep = s1["reported"]
    assert rep["tokens_total"]["total"] == 50000 and rep["tokens_total"]["source"] == "run.json"
    assert rep["tokens_in"]["total"] is None and rep["tokens_out"]["total"] is None
    assert rep["seconds"]["total"] == 120.5
    assert "source" not in rep["cost"]   # the log's own cost stays; run.json adds none
    rr = beta["summary"]["run_reported"]
    assert beta["summary"]["run_reported_basis"] == "mean per task"
    assert rr["tokens_total"] == 50000 and rr["seconds"] == 120.5 and rr["cost"] is None


def test_level3_obstacles_reach_the_scene_and_the_pictures(tasks, tmp_path):
    """Keep-out boxes are passed through exactly as the task file states
    them, to the 3D scene and to the card picture."""
    d = tmp_path / "L1-s0"
    shutil.copytree(os.path.join(tasks, "L1-s0"), d, ignore=shutil.ignore_patterns(".play", "attempts*"))
    tj = json.loads((d / "task.json").read_text(encoding="utf-8"))
    box = {"name": "O1", "kind": "box", "size": [12.0, 23.0, 24.0], "center": [1.0, 2.0, 3.0],
           "axes": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}
    tj["poses"][0]["obstacles"] = [box]
    (d / "task.json").write_text(json.dumps(tj))
    sc = data.scene(str(d), MeshCache(str(tmp_path / "c")))
    assert sc["poses"][0]["obstacles"] == [box]
    assert data.task_thumb(str(d))["obstacles"] == [box]
    # a level 1 task has none
    assert data.scene(os.path.join(tasks, "L1-s1"), MeshCache(str(tmp_path / "c")))["poses"][0]["obstacles"] == []
    # the obstacles' STEP is given to a person, and offered for download
    from jointbench.web import downloads
    shutil.copyfile(d / "fixed_A.step", d / "obstacles.step")   # stands in for the generator's file
    assert "obstacles.step" in data.task_files(str(d))
    assert downloads.available(str(d), [])["obstacles"] is True
    assert downloads.source(str(d), "obstacles.stl") == (str(d / "obstacles.step"), "stl")
    assert downloads.available(os.path.join(tasks, "L1-s1"), [])["obstacles"] is False


def test_material_figure_is_the_graders_and_only_when_it_counts(runs, tmp_path):
    from jointbench.web import score as sc

    root = tmp_path / "r"
    shutil.copytree(runs, root)
    _log(root / "alpha" / "L1-s0", [
        _att(1, True, info={"material": {"volume": 10.0, "par": 8.0, "ratio": 1.25, "counts": True}}),
    ])
    _log(root / "alpha" / "L1-s1", [
        _att(1, True, info={"material": {"ratio": 0.95, "counts": False, "why_not": "minimum wall not met"}}),
    ])
    alpha = next(r for r in sc.score(str(root))["runs"] if r["id"] == "alpha")
    by = {t["id"]: t["rows"][0] for t in alpha["tasks"]}
    assert by["L1-s0"]["material_ratio"] == 1.25 and by["L1-s0"]["material_note"] is None
    assert by["L1-s1"]["material_ratio"] is None and "minimum wall not met" in by["L1-s1"]["material_note"]


def test_runs_under_earlier_rules_are_development_data(runs, tmp_path):
    """A part counts under the current rules when the grader applied exactly
    the requirements its own task states (recorded in verdict.requirements)
    on a task from a generator that states them. A log without that record,
    or graded with other requirements than the task states now, is
    development data; a static copy leaves such runs off its scoreboard, and
    a results file is relabelled from the data when served."""
    from jointbench.web.rules import refresh_rules, task_rules
    from jointbench.web.score import score

    root = tmp_path / "runs"
    shutil.copytree(os.path.join(runs, "alpha"), root / "gamma")
    shutil.copytree(os.path.join(runs, "beta"), root / "beta")
    # gamma: every attempt records the requirements its task states (fits, makeable)
    for t in ("L1-s0", "L1-s1"):
        p = root / "gamma" / t / "task.json"
        tj = json.loads(p.read_text(encoding="utf-8"))
        tj["requirements"] = {"fits": "required", "clean": "reported", "makeable": "required", "sturdy": "reported"}
        tj["generator_version"] = 4
        p.write_text(json.dumps(tj))
        lp = root / "gamma" / t / "attempts.jsonl"
        recs = [json.loads(x) for x in lp.read_text(encoding="utf-8").splitlines() if x.strip()]
        for r in recs:
            r["verdict"] = {"passed": r.get("passed"), "requirements": {
                "fits": {"required": True, "result": "pass"}, "makeable": {"required": True, "result": "pass"}}}
        lp.write_text("".join(json.dumps(r) + "\n" for r in recs))
    by = {r["id"]: r for r in score(str(root))["runs"]}
    assert by["gamma"]["rules"] == {"current": True, "why": []}
    # beta's hand-written log has no requirement record
    assert by["beta"]["rules"]["current"] is False
    assert "graded before the grader recorded requirements" in by["beta"]["rules"]["why"]
    # graded with other requirements than the task states now
    tj = json.loads((root / "gamma" / "L1-s0" / "task.json").read_text(encoding="utf-8"))
    att = {"verdict": {"requirements": {"fits": {"required": True}}}}
    assert "the task states" in task_rules(tj, att)["why"][0]
    # a connecting-part task from before generator 4
    assert task_rules({**tj, "generator_version": 3}, None)["current"] is False
    # a joint task is judged by its own requirements: clean required and recorded is current
    joint = {"family": "revolute", "generator_version": 1, "requirements": {"fits": "required", "clean": "required"}}
    assert task_rules(joint, {"verdict": {"requirements": {"fits": {"required": True}, "clean": {"required": True}}}})["current"]
    # a results file scored by older code is relabelled from the data
    stale = score(str(root))
    for r in stale["runs"]:
        r["rules"] = {"current": False, "why": ["makeable not required"]}
    fresh = refresh_rules(stale, dict(data.collect([str(root)])[0]).get)
    assert {r["id"]: r["rules"]["current"] for r in fresh["runs"]} == {"gamma": True, "beta": False}
    entry = data.index_entry(str(root / "beta" / "L1-s1"), "beta__L1-s1")
    assert entry["rules"]["current"] is False
    assert data.index_entry(str(root / "gamma" / "L1-s1"), "gamma__L1-s1")["rules"]["current"] is True
    # the static copy: only the current run on the scoreboard, the rest counted as left out
    site = tmp_path / "site"
    assert export([str(root)], str(site), log=lambda *a: None) == 0
    sc = json.loads((site / "data" / "scores.json").read_text(encoding="utf-8"))
    assert [r["id"] for r in sc["runs"]] == ["gamma"]
    assert sc["left_out"]["runs"] == 1
    show = json.loads((site / "data" / "showcase.json").read_text(encoding="utf-8"))
    assert {e["rules_current"] for e in show if e["run"] == "beta"} <= {False}


def test_play_opens_a_runs_folder_before_anything_is_graded(tasks, tmp_path):
    """<run>/<task>/task.json with no attempts log yet: play lists the tasks,
    and picks up a run added later without a restart."""
    root = tmp_path / "runs"
    shutil.copytree(os.path.join(tasks, "L1-s1"), root / "alpha" / "L1-s1", ignore=shutil.ignore_patterns("attempts*", ".play"))
    keyed, src = data.collect([str(root)])
    assert [k for k, _ in keyed] == ["alpha__L1-s1"] and src == str(root)
    st = State(str(root), keyed, src)
    st.targets = [str(root)]
    shutil.copytree(os.path.join(tasks, "L1-s1"), root / "beta" / "L1-s1", ignore=shutil.ignore_patterns("attempts*", ".play"))
    assert st.rescan() == ["beta__L1-s1"] and "beta__L1-s1" in st.tasks


def test_zoom_in_and_out_returns_the_identical_view():
    """Trackpad pinch (ctrl + small fractional deltas) and two-finger scroll
    (with a sideways component) zoom about the point under the cursor; zooming
    in and back out by the same amount returns the camera exactly, and the
    point under the cursor never moves on screen. Drives site/js/nav.js."""
    import subprocess

    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    script = os.path.join(os.path.dirname(__file__), "js", "zoom_returns.mjs")
    r = subprocess.run([node, script], capture_output=True, text=True, encoding="utf-8", timeout=60)
    assert r.returncode == 0, r.stderr
    for case in json.loads(r.stdout.strip().splitlines()[-1]):
        assert case["err"] < 1e-6, case          # mm and rad: the same view
        assert case.get("pixels", 0) < 1e-6, case  # the cursor point stays put


def _node(script):
    import subprocess

    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    r = subprocess.run([node, os.path.join(os.path.dirname(__file__), "js", script)], capture_output=True, text=True, encoding="utf-8", timeout=60)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout.strip().splitlines()[-1])


def test_check_list_groups_limits_and_how_the_part_is_made():
    """Checks group by requirement with their own counts; each line shows
    its limit on the side the checker states; the manufacturing setup names
    the face the part stands on and its area, and for moulding says the
    parting plane is assumed."""
    d = _node("checks_list.mjs")
    assert d["groups"] == [["fits", "1/2", True, True], ["clean", "1/2", False, False],
                           ["makeable", "2/3", False, False], ["sturdy", "1/1", False, True], ["keyed", "1/1", False, True]]
    assert d["limits"]["HOLES"] == "≤ 0.1" and d["limits"]["SEATING"] == "≥ 0.75"
    assert d["limits"]["Q_TINY_FACE"] == "≥ 0.1" and d["limits"]["FDM_OVERHANG"] == "≤ 45"
    assert d["limits"]["FDM_SEALED_HOLLOW"] == "limit 0"   # no word, value at the limit: no side claimed
    assert d["sources"][2:7] == ["quality", "quality", "fdm", "fdm", "fdm"]
    assert d["failRowHasReason"] and not d["passRowHasReason"] and d["openRowHasReason"]
    assert d["fdm"]["face"] == 4 and d["fdm"]["text"].startswith("Stands on face 4, 100 mm²")
    assert "stands on face 4, 100 mm²" in d["fdm"]["labels"]
    assert "assumed" in d["mould"]["text"] and "parting plane (assumed, mid-height)" in d["mould"]["labels"]
    assert d["stressCounted"] == [1, 1, 2] and d["stressSource"] == "stiffness"   # stress: shown, not a check
    assert d["crease"] == [6, 12]


def test_quality_field_is_made_served_and_exported(tasks, tmp_path):
    """The modelling-quality checker's field file is made like a
    manufacturing one, listed under the attempt's fields (not its processes),
    served, and carried into a static copy."""
    t = tmp_path / "tasks"
    shutil.copytree(tasks, t)
    d = str(t / "L1-s0")
    att = data.read_attempts(d)[-1]
    assert "quality" in data.field_sources() and "stiffness" in data.field_sources()
    assert data.run_dfm(d, att, "nonsense")["ok"] is False
    res = data.run_dfm(d, att, "quality")
    assert res["ok"], res
    meshes = MeshCache(str(tmp_path / "cache"))
    rec = next(r for r in data.attempts_doc(d, meshes) if r["n"] == att["n"])
    assert "quality" in rec["fields"] and "quality" not in rec["dfm"]
    p = data.dfm_fields_path(d, att, "quality")
    assert p and json.load(open(p))["process"] == "quality"
    assert "quality" in data.fields_available(d, att) and "quality" not in data.dfm_available(d, att)
    site = tmp_path / "site"
    assert export([str(t)], str(site), log=lambda *a: None) == 0
    assert (site / "data" / "tasks" / "L1-s0" / "dfm" / f"{att['n']}.quality.json").exists()


def test_static_copy_can_precompute_fields_without_writing_to_the_runs(tasks, tmp_path):
    """--precompute measures each model run's submission for the given
    processes so the compare page can switch process offline; the field
    files go into the copy only, never into the run folders."""
    runs = tmp_path / "runs"
    shutil.copytree(os.path.join(tasks, "L1-s0"), runs / "gamma" / "L1-s0", ignore=shutil.ignore_patterns(".play"))
    runs = str(runs)
    site = tmp_path / "site"
    # the meshes and downloads are made into the copy's own caches; measure what the run folder holds after
    assert export([runs], str(site), log=lambda *a: None, dev_runs=True) == 0
    before = {p for p in _all_files(runs)}
    assert export([runs], str(site), log=lambda *a: None, dev_runs=True, precompute=["quality"]) == 0
    assert {p for p in _all_files(runs)} == before
    atts = json.loads((site / "data" / "tasks" / "gamma__L1-s0" / "attempts.json").read_text(encoding="utf-8"))
    sub = next(a for a in atts if a.get("passed"))
    assert "quality" in sub["fields"]
    assert (site / "data" / "tasks" / "gamma__L1-s0" / "dfm" / f"{sub['n']}.quality.json").exists()


def _all_files(root):
    for d, _, fs in os.walk(root):
        for f in fs:
            yield os.path.relpath(os.path.join(d, f), root)


def test_requirement_ladder_reads_the_graders_record_and_counts_reach():
    """Rungs in a fixed order, each met / not met / not measured and required
    or not, from the grader's verdict; a part reaches a rung when it meets it
    and every measured rung below; an unmeasured rung neither counts nor blocks."""
    from jointbench.web.ladder import ladder, reach

    att = {"checks": [{"name": "FILE_VALID", "status": "pass", "group": "fits"}],
           "verdict": {"requirements": {"fits": {"required": True, "result": "pass"},
                                        "makeable": {"required": True, "result": "fail"},
                                        "sturdy": {"required": False, "result": "pass"}}}}
    lad = ladder(att, {})
    assert [r["rung"] for r in lad] == ["valid", "fits", "clean", "makeable", "sturdy"]
    assert [r["state"] for r in lad] == ["met", "met", "not measured", "not met", "met"]
    assert [r["required"] for r in lad] == [True, True, False, True, False]
    ok = ladder({"checks": [{"name": "FILE_VALID", "status": "pass"}],
                 "verdict": {"requirements": {"fits": {"required": True, "result": "pass"},
                                              "makeable": {"required": True, "result": "pass"},
                                              "sturdy": {"required": False, "result": "pass"}}}}, {})
    r = {x["rung"]: x for x in reach([lad, ok])}
    assert (r["fits"]["reached"], r["fits"]["measured"]) == (2, 2)
    assert (r["clean"]["reached"], r["clean"]["measured"]) == (0, 0)      # not measured: no share
    assert (r["makeable"]["reached"], r["makeable"]["measured"]) == (1, 2)
    assert (r["sturdy"]["reached"], r["sturdy"]["measured"]) == (1, 2)     # stopped below at makeable
    # an older log without verdict.requirements: read from the checks of each group
    old = ladder({"checks": [{"name": "FILE_VALID", "status": "pass"}, {"name": "HOLES", "status": "fail"},
                             {"name": "Q_CREASE", "status": "fail", "required": False}]}, {"requirements": {"clean": "reported"}})
    assert [x["state"] for x in old][:3] == ["met", "not met", "not met"] and old[2]["required"] is False


def test_process_versions_share_a_base_id():
    from jointbench.web.rules import task_version

    v = task_version({"id": "L2-s3000-moulding", "tolerances": {"manufacturing": {"process": "moulding"}}})
    assert v == {"base": "L2-s3000", "process": "moulding", "versioned": True}
    assert task_version({"id": "L2-s3000"})["versioned"] is False


FAMILIES = {"revolute": (2, 2000), "interference": (2, 2000), "keyed": (2, 2000), "spline": (2, 2000),
            "involute": (2, 2000), "kinematic": (2, 2000), "snapfit": (2, 2000),
            "fourbar": (2, 4000), "slider": (2, 4000), "yoke": (2, 4000), "oldham": (2, 4000)}


@pytest.fixture(scope="module")
def family_tasks(tmp_path_factory):
    """One generated task of every joint and mechanism family."""
    root = tmp_path_factory.mktemp("families")
    for fam, (level, seed) in FAMILIES.items():
        assert main(["gen", "--family", fam, "--level", str(level), "--seed", str(seed), "--n", "1", "--out", str(root)]) == 0, fam
    return str(root)


@pytest.mark.parametrize("family", sorted(FAMILIES))
def test_every_family_opens(family_tasks, family, tmp_path):
    """Joint and mechanism tasks give their fixed parts as STEP files beside
    task.json: the scene, the card picture and the server's scene request all
    read them from the task folder."""
    d = next(os.path.join(family_tasks, n) for n in sorted(os.listdir(family_tasks))
             if json.loads(open(os.path.join(family_tasks, n, "task.json")).read()).get("family") == family)
    sc = data.scene(d, MeshCache(str(tmp_path / "c")))
    assert sc["poses"] and all(sc["poses"][k]["fixed"] for k in range(len(sc["poses"])))
    assert data.task_thumb(d)["fixed"]
    if family in ("fourbar", "slider", "yoke", "oldham"):
        # the whole motion, even from a folder that stores one pose, and the answer moves with it
        assert len(sc["poses"]) > 1 and sc["part_motion"] and len(sc["part_motion"]) == len(sc["poses"])
        assert sc["part_motion"][0] == pytest.approx([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], abs=1e-6)
    st = State(d, data.find_tasks(d))
    srv = __import__("http.server").server.ThreadingHTTPServer(("127.0.0.1", 0), make_handler(st, site_dir()))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        tid = next(iter(st.tasks))
        code, body = _get(f"http://127.0.0.1:{srv.server_address[1]}/data/tasks/{tid}/scene.json")
        assert code == 200 and json.loads(body)["poses"]
    finally:
        srv.shutdown()


def test_a_failing_request_answers_with_its_reason(family_tasks, tmp_path):
    """No request ends as a dropped connection ("network error" on the page):
    the server answers 500 with the reason."""
    src = next(os.path.join(family_tasks, n) for n in sorted(os.listdir(family_tasks)) if n.startswith("revolute"))
    d = tmp_path / "broken"
    shutil.copytree(src, d)
    for f in os.listdir(d):
        if f.endswith(".step") and not f.startswith("_"):
            os.remove(d / f)
    st = State(str(d), data.find_tasks(str(d)))
    srv = __import__("http.server").server.ThreadingHTTPServer(("127.0.0.1", 0), make_handler(st, site_dir()))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        tid = next(iter(st.tasks))
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{srv.server_address[1]}/data/tasks/{tid}/scene.json", timeout=120)
            raise AssertionError("expected an error answer")
        except urllib.error.HTTPError as e:
            assert e.code == 500 and json.loads(e.read())["error"]
    finally:
        srv.shutdown()
