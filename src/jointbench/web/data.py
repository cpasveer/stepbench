"""What the front end reads, as plain JSON documents.

The live server and the static export produce the *same* documents at the
*same* paths, so the page has one way to load data and does not care which
of the two is behind it:

    data/site.json                      mode (live or static), version
    data/index.json                     every task: level, seed, parts, attempts, solved
    data/tasks/<id>/scene.json          brief, task.json, fixed-part meshes per pose,
                                        mounting faces and bolts per pose
    data/tasks/<id>/attempts.json       the attempts log, newest last, with mesh links
    data/tasks/<id>/marks/<n>.json      overlap regions for attempt n (when it failed
                                        NO_INTERFERENCE)
    data/mesh/<sha256>.json             a mesh of one STEP file, by content

Nothing here grades. Verdicts are copied from ``attempts.jsonl`` as the
grader wrote them. The one geometric computation is ``marks``: to *show*
where a clash is, the overlap solid is rebuilt for the overlaps the grader
already reported as over the limit, with the grader's own keep-out shape.
It decides nothing; a mark is drawn only where the verdict already failed.
"""
from __future__ import annotations

import json
import os

from ..task import Task
from .mesh import MeshCache, file_sha256, mesh_shape
from .rules import task_rules, task_version
from .ladder import ladder as _ladder

VIEW_DIR = ".play"  # per task: uploads and anything the viewer stores


def task_files(task_dir: str) -> list[str]:
    """The files a person needs to attempt the task in their own CAD
    program: the brief, task.json, the fixed parts, the obstacles (level 3),
    the starter script. Never the reference answer."""
    out = []
    for name in sorted(os.listdir(task_dir)):
        if name in ("README.md", "task.json", "submission.py", "obstacles.step") or (
                name.startswith("fixed_") and name.lower().endswith((".step", ".stp"))):
            out.append(name)
    return out


def find_tasks(path: str) -> list[str]:
    """A task folder, or a folder of task folders (one level deep)."""
    path = os.path.abspath(path)
    if os.path.exists(os.path.join(path, "task.json")):
        return [path]
    out = []
    if os.path.isdir(path):
        for name in sorted(os.listdir(path), key=_natural):
            d = os.path.join(path, name)
            if os.path.exists(os.path.join(d, "task.json")):
                out.append(d)
    return out


def collect(targets: list[str], scores_file: str | None = None):
    """Every task the viewer should know, as (key, task dir), plus where its
    scores come from.

    A target may be a task folder, a folder of tasks, or a runs folder
    (<run>/<task>/). Tasks of a run get the key '<run>__<task>', because the
    same task id appears once per model. Scores come from a results file
    when one is given (its runs folder is then loaded too, if it still
    exists), otherwise they are computed live from the first runs folder."""
    from . import score

    keyed: list[tuple[str, str]] = []
    seen: set[str] = set()
    runs_root = None
    loaded = None
    if scores_file:
        with open(scores_file, encoding="utf-8") as f:
            loaded = json.load(f)
        root = loaded.get("root")
        if root and os.path.isdir(root) and root not in targets:
            targets = list(targets) + [root]
    for t in targets:
        if score.is_runs_folder(t) or score.looks_like_runs(t):
            runs_root = runs_root or os.path.abspath(t)
            pairs = score.run_tasks(t)
        else:
            pairs = [(os.path.basename(d), d) for d in find_tasks(t)]
        for k, d in pairs:
            if d not in seen:
                seen.add(d)
                keyed.append((k, d))
    return keyed, (loaded if loaded is not None else runs_root)


def task_key_run(key: str) -> str | None:
    return key.split("__", 1)[0] if "__" in key else None


def _natural(s: str):
    import re

    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", s)]


def read_attempts(task_dir: str) -> list[dict]:
    p = os.path.join(task_dir, "attempts.jsonl")
    out = []
    if not os.path.exists(p):
        return out
    with open(p, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:  # a line being written right now
                continue
    return out


def _task(tj: dict, task_dir: str) -> Task:
    """The task with its folder: fixed parts given as solids (the joint and
    mechanism families) are read from STEP files beside task.json."""
    t = Task.from_json(tj)
    t.root = os.path.abspath(task_dir)
    return t


def _load_task_json(task_dir: str) -> dict:
    with open(os.path.join(task_dir, "task.json"), encoding="utf-8") as f:
        return json.load(f)


def index_entry(task_dir: str, key: str | None = None) -> dict:
    tj = _load_task_json(task_dir)
    atts = read_attempts(task_dir)
    parts = []
    for fp in tj["fixed_parts"]:
        itf = fp["interface"]
        parts.append({"name": fp["name"], "kind": itf["kind"], "bolt": itf["bolt"],
                      "holes": len(itf.get("holes_local", [])),
                      "description": itf.get("description", "")})
    gap = angle = dist = None
    try:
        faces = tj["poses"][0]["resolved"]["mounting_faces"]
        if len(faces) == 2:
            a, b = faces[0]["origin"], faces[1]["origin"]
            n, m = faces[0]["normal"], faces[1]["normal"]
            dot = max(-1.0, min(1.0, sum(n[i] * m[i] for i in range(3))))
            dist = round(sum((b[i] - a[i]) ** 2 for i in range(3)) ** 0.5, 1)
            if dot < -0.9999:  # facing each other: the gap between them is the one number
                gap = round(abs(sum((b[i] - a[i]) * n[i] for i in range(3))), 3)
            else:  # angled: how far the faces turn from facing each other, and how far apart
                import math
                angle = round(math.degrees(math.acos(-dot)), 1)
    except (KeyError, IndexError, TypeError):
        pass
    passed_by = sorted({a.get("by", "?") for a in atts if a.get("passed")})
    last = atts[-1] if atts else None
    return {
        "id": tj["id"], "key": key or tj["id"], "run": task_key_run(key or ""),
        # graded under the current rules, or development data (see rules.py)
        "rules": task_rules(tj, atts[-1] if atts else None),
        # the process version of a challenge (base id, stated process)
        "version": task_version(tj),
        "level": tj["level"], "seed": tj["seed"], "units": tj.get("units", "mm"),
        "parts": parts,
        "bolts": sum(p["holes"] for p in parts),
        "poses": len(tj["poses"]),
        "obstacles": len(tj["poses"][0].get("obstacles") or []),
        "gap": gap, "face_angle": angle, "face_distance": dist,
        "plan": plan(tj),
        "attempts": len(atts),
        "solved": bool(passed_by),
        "solved_by": passed_by,
        "last": None if last is None else {
            "n": last.get("n"), "time": last.get("time"), "by": last.get("by"),
            "passed": last.get("passed"),
            "failed": [c["name"] for c in last.get("checks", []) if c.get("status") == "fail" and c.get("required", True)],
        },
    }


def plan(tj: dict) -> dict:
    """A plan view of pose 0 for the overview thumbnails: every mounting face
    outline and bolt, projected onto the first mounting face's plane."""
    pj = tj["poses"][0]
    faces = pj["resolved"]["mounting_faces"]
    if not faces:   # a task with no mounting faces (some joint and mechanism families): no plan
        return {"faces": [], "bolts": []}
    parts = {fp["name"]: fp for fp in tj["fixed_parts"]}
    n = faces[0]["normal"]
    # any unit vector not parallel to n, then u, v spanning the plane
    a = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = _unit(_cross(a, n))
    v = _cross(n, u)

    def pr(p):
        return [round(sum(p[i] * u[i] for i in range(3)), 3), round(sum(p[i] * v[i] for i in range(3)), 3)]

    return {
        "faces": [{"part": f["part"], "outline": [pr(c) for c in face_outline(parts[f["part"]], pj["placements"][f["part"]])]}
                  for f in faces],
        "bolts": [{"id": b["id"], "at": pr(b["position"]), "d": b.get("head_diameter", 5)}
                  for b in pj["resolved"]["bolts"]],
    }


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(a):
    m = sum(x * x for x in a) ** 0.5
    return tuple(x / m for x in a)


def face_outline(tj_part: dict, placement: dict) -> list[list[float]]:
    """The mounting face rectangle in world coordinates (4 corners)."""
    w, h = tj_part["interface"]["face_size"]
    R, o = placement["rotation"], placement["origin"]
    pts = []
    for x, y in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)):
        pts.append([round(R[i][0] * x + R[i][1] * y + o[i], 4) for i in range(3)])
    return pts


def scene(task_dir: str, meshes: MeshCache) -> dict:
    """Everything needed to draw the task, for every pose."""
    tj = _load_task_json(task_dir)
    task = _task(tj, task_dir)
    brief = ""
    rp = os.path.join(task_dir, "README.md")
    if os.path.exists(rp):
        with open(rp, encoding="utf-8") as f:
            brief = f.read()
    poses = []
    parts_by_name = {fp["name"]: fp for fp in tj["fixed_parts"]}
    # mechanisms: every pose of the motion, from the motion module, when the
    # folder keeps one stored pose (newer folders); and, for any mechanism, how
    # the missing part moves, so the answer moves with the pose slider
    stored = len(tj["poses"])
    motion, part_motion = _motion(task, stored)
    if motion and stored == 1:
        task.poses = [{n: p[n] for n in task.parts} for p in motion]
    blank = {"resolved": {"mounting_faces": [], "bolts": []}}
    for k in range(len(task.poses)):
        pj = tj["poses"][k] if k < stored else {**blank, "name": f"pose{k}"}
        fixed = {}
        for name, solid in task.fixed_solids(k).items():
            fixed[name] = mesh_shape(solid)
        faces = []
        for f in pj["resolved"]["mounting_faces"]:
            faces.append({**f, "outline": face_outline(parts_by_name[f["part"]], pj["placements"][f["part"]])})
        owner = {b.id: b.part for b in task.bolts(k)} if k < stored else {}
        poses.append({"name": pj.get("name", f"pose{k}"), "fixed": fixed, "faces": faces,
                      "bolts": [{**b, "part": owner.get(b["id"])} for b in pj["resolved"]["bolts"]],
                      # level 3 keep-out boxes, exactly as the task file states them
                      # (centre, size along the box's own x, y, z directions)
                      "obstacles": pj.get("obstacles") or []})
    ref = None
    rs = os.path.join(task_dir, "_solution", "reference.step")
    if os.path.exists(rs):
        ref = meshes.put_step(rs)
    from .downloads import FORMATS, tolerance_note

    fixed_dl = [f[len("fixed_"):-len(".step")] for f in task_files(task_dir)
                if f.startswith("fixed_") and f.lower().endswith(".step")]
    return {"id": tj["id"], "level": tj["level"], "seed": tj["seed"], "units": tj.get("units", "mm"),
            "downloads": {"fixed": fixed_dl, "formats": list(FORMATS), "note": tolerance_note()},
            "brief": brief, "files": task_files(task_dir), "task": tj, "poses": poses, "reference": ref,
            "rules": task_rules(tj),
            # per pose, the missing part's move from where it sits in pose 1 (4x4,
            # column-major); the answer is modelled in pose 1, so this carries it along
            "part_motion": part_motion}


def _motion(task: Task, stored: int):
    """(poses, part_motion) of a mechanism task from jointbench.motion, or
    (None, None). The motion module rebuilds the mechanism from the task file
    (0.2 to 2 s the first time); the reference motion agrees with the motion a
    working answer drives (motion session, 2026-11-12)."""
    mo = (task.meta or {}).get("motion") if hasattr(task, "meta") else None
    if not mo:
        return None, None
    try:
        from ..motion.family import motion_poses
        motion = motion_poses(task)
    except Exception:  # noqa: BLE001 -- no motion: the stored poses are shown as they are
        return None, None
    if stored > 1 and len(motion) != stored:
        return None, None
    rem = mo.get("removed")
    if not rem or rem not in motion[0]:
        return motion, None
    p0 = motion[0][rem]
    r0t = [[p0.rot[j][i] for j in range(3)] for i in range(3)]   # inverse rotation
    out = []
    for p in motion:
        pk = p[rem]
        r = [[sum(pk.rot[i][m] * r0t[m][j] for m in range(3)) for j in range(3)] for i in range(3)]
        t = [pk.origin[i] - sum(r[i][j] * p0.origin[j] for j in range(3)) for i in range(3)]
        out.append([round(v, 9) for v in (r[0][0], r[1][0], r[2][0], 0, r[0][1], r[1][1], r[2][1], 0,
                                          r[0][2], r[1][2], r[2][2], 0, t[0], t[1], t[2], 1)])
    return motion, out


# ---- the front page's 3D showcase -------------------------------------------

SHOWCASE_MAX = 40


def showcase(keyed: list, root: str | None = None) -> list[dict]:
    """Attempts to show on the front page, one after another.

    A curated list is used when the folder holds ``showcase.json`` (a list
    of {"task": <key or task id>, "n": <attempt>}); otherwise every attempt
    whose geometry is kept is a candidate (the page picks at random, and
    puts failures in on purpose). With no attempts at all, tasks with a
    reference answer are offered instead."""
    by_key = dict(keyed)
    by_id = {}
    for k, d in keyed:
        by_id.setdefault(os.path.basename(d), k)
    out = []

    def entry(key, att):
        # the checks that decided the verdict (advisory ones, e.g. Q_*, carry required: false)
        failed = [c["name"] for c in att.get("checks", []) if c.get("status") == "fail" and c.get("required", True)]
        return {"key": key, "task": os.path.basename(by_key[key]), "run": task_key_run(key), "n": att["n"],
                "rules_current": task_rules(_load_task_json(by_key[key]), att)["current"],
                "by": att.get("by"), "passed": bool(att.get("passed")), "failed": failed,
                "marks": "NO_INTERFERENCE" in failed}

    cur = os.path.join(root, "showcase.json") if root else None
    if cur and os.path.isfile(cur):
        with open(cur, encoding="utf-8") as f:
            for item in json.load(f):
                key = item.get("task")
                key = key if key in by_key else by_id.get(key)
                if not key:
                    continue
                att = next((a for a in read_attempts(by_key[key]) if a.get("n") == item.get("n")), None)
                if att and find_step(by_key[key], att):
                    out.append({**entry(key, att), "curated": True})
        if out:
            return out
    for key, d in keyed:
        for att in read_attempts(d):
            if find_step(d, att):
                out.append(entry(key, att))
    if not out:
        for key, d in keyed:
            if os.path.exists(os.path.join(d, "_solution", "reference.step")):
                out.append({"key": key, "task": os.path.basename(d), "run": task_key_run(key), "reference": True})
    # keep it small, but keep the failures: they show what the benchmark does
    fails = [e for e in out if e.get("passed") is False]
    passes = [e for e in out if e not in fails]
    return (fails[: SHOWCASE_MAX // 2] + passes)[:SHOWCASE_MAX]


# ---- thumbnails: coarse meshes for the small 3D pictures (task cards,
# compare grid). One shared 3D context in the browser draws them.

def task_thumb(task_dir: str) -> dict:
    """The fixed parts of pose 0 as coarse meshes, for a task card."""
    from .mesh import THUMB_ANGULAR, THUMB_LINEAR

    tj = _load_task_json(task_dir)
    task = _task(tj, task_dir)
    fixed = [mesh_shape(s, THUMB_LINEAR, THUMB_ANGULAR) for s in task.fixed_solids(0).values()]
    # level 3 keep-out boxes, as the task file states them (drawn exactly in the page)
    return {"fixed": fixed, "obstacles": (tj.get("poses") or [{}])[0].get("obstacles") or []}


def attempt_thumb(task_dir: str, att: dict, cache_dir: str) -> dict | None:
    """An attempt's part as a coarse mesh (cached by its sha256), or None."""
    from ..geometry import read_step
    from .mesh import THUMB_ANGULAR, THUMB_LINEAR

    step = find_step(task_dir, att)
    if not step:
        return None
    sha = att.get("sha256") or file_sha256(step)
    path = os.path.join(cache_dir, f"{sha}.json")
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    try:
        m = mesh_shape(read_step(step), THUMB_LINEAR, THUMB_ANGULAR)
    except Exception:  # noqa: BLE001 - no thumbnail for an unreadable file
        return None
    os.makedirs(cache_dir, exist_ok=True)
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(m, f, separators=(",", ":"))
    os.replace(tmp, path)
    return m


def attempts_doc(task_dir: str, meshes: MeshCache, make_missing: bool = True) -> list[dict]:
    """The attempts log, with a mesh link for every attempt whose graded
    file can still be found (archive, upload folder, or the logged path with
    unchanged content), the archive listing when there is one, and the
    submitter-reported figures read out.

    An attempt whose file is gone keeps its verdict and gets ``mesh: None``
    -- the page shows it as "geometry not kept"."""
    from .score import reported, step_origin

    tj = _load_task_json(task_dir)
    out = []
    for a in read_attempts(task_dir):
        sha = a.get("sha256")
        mesh = None
        if sha and meshes.has(sha):
            mesh = sha
        elif make_missing:
            step = find_step(task_dir, a)
            if step:
                mesh = meshes.put_step(step, sha)
        a = _with_section_outline(task_dir, a)
        out.append({**a, "mesh": mesh,
                    "marks": bool(mesh) and _has_overlap_fail(a),
                    "archive": archive_info(task_dir, a),
                    "dfm": dfm_available(task_dir, a),
                    # every field file made for it: processes, quality, stiffness
                    "fields": fields_available(task_dir, a),
                    "ladder": _ladder(a, tj),
                    "reported": reported(a), "step_origin": step_origin(a)})
    return out


def archive_file(task_dir: str, att: dict, name: str) -> str | None:
    """Path of one archived file of an attempt, only if it is listed."""
    arc = archive_info(task_dir, att)
    if arc and name in arc["files"]:
        return os.path.join(task_dir, arc["dir"], name)
    return None


# The per-attempt archive (attempts/0001/ ...) is being added by the core.
# Until its format is fixed, the folder is found from the log record if it
# names one, else by the attempt number, and its files are listed as found.
ARCHIVE_KEYS = ("archive", "archive_dir", "dir", "folder")
TEXT_SUFFIXES = (".py", ".scad", ".fcmacro", ".txt", ".md", ".json", ".log", ".js", ".ts")
MAX_TEXT = 512 * 1024


def archive_dir(task_dir: str, att: dict) -> str | None:
    for k in ARCHIVE_KEYS:
        v = att.get(k)
        if isinstance(v, str) and v:
            d = os.path.normpath(os.path.join(task_dir, v))
            if d.startswith(os.path.abspath(task_dir)) and os.path.isdir(d):
                return d
    n = att.get("n")
    if isinstance(n, int):
        for name in (f"{n:04d}", f"{n:03d}", str(n)):
            d = os.path.join(task_dir, "attempts", name)
            if os.path.isdir(d):
                return d
    return None


def archive_files(adir: str) -> list[str]:
    out = []
    for name in sorted(os.listdir(adir)):
        p = os.path.join(adir, name)
        if os.path.isfile(p) and not name.startswith("."):
            out.append(name)
    return out


def archive_info(task_dir: str, att: dict) -> dict | None:
    """What an archived attempt kept (format: the core's README, "Every
    attempt is kept"; archive.py). The record's ``files`` map names the
    step, script, feedback and note; for folders without it, files are
    recognised by extension."""
    adir = archive_dir(task_dir, att)
    if adir is None:
        return None
    files = archive_files(adir)
    rel = os.path.relpath(adir, task_dir)
    fmap = att.get("files") if isinstance(att.get("files"), dict) else {}

    def named(key):
        v = fmap.get(key)
        if isinstance(v, str):
            b = os.path.basename(v)
            if b in files and os.path.normpath(os.path.dirname(v)) == os.path.normpath(rel):
                return b
        return None

    step = named("step") or next((f for f in files if f.lower().endswith((".step", ".stp"))), None)
    script = named("script")
    code = [script] if script else [f for f in files if f.lower().endswith((".py", ".scad", ".fcmacro", ".js", ".ts"))]
    feedback = named("feedback") or ("feedback.txt" if "feedback.txt" in files else None)
    note = named("note") or ("note.txt" if "note.txt" in files else None)
    if fmap:
        texts = [feedback] if feedback else []
    else:
        texts = [f for f in files if f.lower().endswith((".txt", ".log", ".md")) and f != note]
        if feedback and feedback in texts:
            texts.remove(feedback)
            texts.insert(0, feedback)
    results = [f for f in files if f.lower().endswith(".json")]
    meta = {}
    for f in results:
        try:
            with open(os.path.join(adir, f), encoding="utf-8") as fh:
                j = json.load(fh)
            if isinstance(j, dict):
                for k in ("note", "notes", "submitter", "by", "time", "timestamp", "tool", "cad_tool"):
                    if k in j and k not in meta and not isinstance(j[k], (dict, list)):
                        meta[k] = j[k]
        except (OSError, ValueError):
            continue
    return {"dir": rel, "files": files, "step": step, "code": code, "results": results, "texts": texts,
            "feedback_format": att.get("feedback_format"), "note": note,
            "script_link": fmap.get("script_link"), "meta": meta}


# ---- manufacturing check (src/jointbench/dfm, owned by another module) ----
# The viewer only shows its field files (format: docs/dfm.md, "Field file");
# it never judges manufacturability itself.

DFM_FORMAT = "jointbench-dfm-fields"


def dfm_processes() -> list[str]:
    try:
        from ..dfm import available
        return list(available())
    except Exception:  # noqa: BLE001 - the checker is optional for the viewer
        return []


def dfm_cache_dir(task_dir: str) -> str:
    return os.path.join(task_dir, VIEW_DIR, "dfm")


def dfm_fields_path(task_dir: str, att: dict, process: str) -> str | None:
    """A field file for this attempt and process: archived with the attempt,
    or made earlier by the viewer (named by the graded file's sha256)."""
    arc = archive_info(task_dir, att)
    if arc:
        for f in arc["files"]:
            if f.endswith(f".{process}.fields.json"):
                return os.path.join(task_dir, arc["dir"], f)
    sha = att.get("sha256")
    if sha:
        p = os.path.join(dfm_cache_dir(task_dir), f"{sha}.{process}.fields.json")
        if os.path.exists(p):
            return p
    return None


def dfm_available(task_dir: str, att: dict) -> list[str]:
    return [p for p in dfm_processes() if dfm_fields_path(task_dir, att, p)]


# Every checker that writes a field file in the same layout: the manufacturing
# processes, the modelling-quality checker and the stiffness analysis. A check
# in the verdict list is drawn from its source's file (the field whose "rule"
# is the check's name); quality and stiffness are kept apart from the
# manufacturing processes, so the "Made how?" tab lists processes only.
EXTRA_SOURCES = ("quality", "stiffness")


def field_sources() -> list[str]:
    return dfm_processes() + list(EXTRA_SOURCES)


def fields_available(task_dir: str, att: dict) -> list[str]:
    return [p for p in field_sources() if dfm_fields_path(task_dir, att, p)]


def run_dfm(task_dir: str, att: dict, process: str, timeout: int = 600, out_path: str | None = None) -> dict:
    """Run the project's manufacturing checker on an attempt's graded file
    (`python -m jointbench.dfm --fields`), caching the field file by sha256.
    out_path: write the field file there instead (a static copy; the task
    folder is then never written to)."""
    import shutil
    import subprocess
    import sys
    import tempfile

    if process not in field_sources():
        return {"ok": False, "error": f"unknown process {process!r}"}
    step = find_step(task_dir, att)
    sha = att.get("sha256")
    if not step or not sha:
        return {"ok": False, "error": "the graded file of this attempt was not kept"}
    out = dfm_cache_dir(task_dir)
    if not out_path:
        os.makedirs(out, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, f"{sha}.step")
        shutil.copy(step, src)
        if process == "quality":
            cmd = [sys.executable, "-m", "jointbench.quality", src, "--fields", tmp]
        elif process == "stiffness":  # needs the task (mounts, loads); --quick keeps it to about a minute
            cmd = [sys.executable, "-m", "jointbench.stiffness", task_dir, "--file", src, "--quick", "--fields", tmp]
        else:
            cmd = [sys.executable, "-m", "jointbench.dfm", src, "--process", process, "--fields", tmp]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        made = os.path.join(tmp, f"{sha}.{process}.fields.json")
        if not os.path.exists(made):
            return {"ok": False, "error": (r.stderr or r.stdout)[-1500:] or f"exit {r.returncode}"}
        if out_path:
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            shutil.move(made, out_path)
        else:
            shutil.move(made, os.path.join(out, f"{sha}.{process}.fields.json"))
    return {"ok": True, "report": r.stdout[-4000:]}


def find_step(task_dir: str, att: dict) -> str | None:
    """The STEP file an attempt graded, if it still has the graded content:
    from the attempt's archive, the viewer's uploads, or the logged path."""
    sha = att.get("sha256")
    arc = archive_info(task_dir, att)
    cands = []
    if arc and arc["step"]:
        cands.append(os.path.join(task_dir, arc["dir"], arc["step"]))
    if sha:
        cands.append(os.path.join(task_dir, VIEW_DIR, "uploads", f"{sha}.step"))
    cands.append(os.path.join(task_dir, att.get("file") or ""))
    for cand in cands:
        if os.path.isfile(cand) and (not sha or file_sha256(cand) == sha):
            return cand
    return None


def _has_overlap_fail(att: dict) -> bool:
    for c in att.get("checks", []):
        if c.get("name") == "NO_INTERFERENCE" and c.get("status") == "fail":
            return True
    return False


def marks(task_dir: str, att: dict, step_path: str) -> dict:
    """Overlap regions for one failed attempt, as meshes, labelled with the
    grader's own row names. Only rows the grader reported over the limit
    are rebuilt. The order of the grader's rows (per pose: part vs each
    fixed part, then per bolt: bolt vs part, bolt vs each fixed part) gives
    the pose of each row."""
    import build123d as bd

    from ..catalogue import BOLTS
    from ..checks.interference import keepout
    from ..geometry import read_step

    task = Task.load(os.path.join(task_dir, "task.json"))
    lim = task.tolerances["overlap_volume_max"]
    checks = {c["name"]: c for c in att.get("checks", [])}
    ni = checks.get("NO_INTERFERENCE")
    if not ni or ni.get("status") != "fail":
        return {"regions": []}
    rows = ni.get("details", {}).get("overlaps", [])
    grips_rows = checks.get("HOLES", {}).get("details", {}).get("holes", [])
    n_parts = len(task.parts)
    per_pose = n_parts + sum(1 + n_parts for _ in task.bolts(0))
    solids = read_step(step_path).solids()
    if len(solids) != 1:
        return {"regions": []}
    part = solids[0]
    regions = []
    for i, r in enumerate(rows):
        if r.get("volume", 0) <= lim:
            continue
        k = i // per_pose if per_pose else 0
        what = r["what"]
        fixed = task.fixed_solids(k)
        bolts = {b.id: b for b in task.bolts(k)}
        grips = {g["bolt"]: g["grip"] for g in grips_rows}
        try:
            if what.startswith("part vs fixed "):
                shape = part & fixed[what.split()[-1]]
            else:
                bid = what.split()[1]
                b = bolts[bid]
                ko = keepout(b, grips.get(bid, BOLTS[b.size].grip_min), task.tolerances)
                other = part if what.endswith("vs part") else fixed[what.split()[-1]]
                shape = ko & other
            if not isinstance(shape, bd.Shape) or not shape.faces():
                continue
            m = mesh_shape(shape)
        except Exception:  # noqa: BLE001 - a mark we cannot rebuild is left out
            continue
        regions.append({"what": what, "volume": r["volume"], "pose": k, "mesh": m})
    return {"regions": regions}


def _with_section_outline(task_dir: str, a: dict) -> dict:
    """A record graded before SECTION_MIN kept the cut's edges gets them
    from its graded file (cached beside the viewer's other files), so the
    site draws the real cut, not a circle of equal area."""
    import copy
    import json as _json

    checks = a.get("checks") or []
    sec = next((c for c in checks if c.get("name") == "SECTION_MIN"), None)
    rows = ((sec or {}).get("details") or {}).get("sections") or []
    if not rows or all("outline" in r for r in rows) or not a.get("sha256"):
        return a
    cache = os.path.join(task_dir, VIEW_DIR, "section", f"{a['sha256']}.json")
    got = None
    if os.path.exists(cache):
        with open(cache, encoding="utf-8") as f:
            got = _json.load(f)
    else:
        step = find_step(task_dir, a)
        if not step:
            return a
        try:
            from ..checks.section_min import outline
            from ..geometry import read_step

            shape = read_step(step)
            solid = shape.solids()[0]
            got = [outline(solid, r["at"], r["normal"]) for r in rows]
        except Exception:  # noqa: BLE001 - a drawing aid only
            return a
        os.makedirs(os.path.dirname(cache), exist_ok=True)
        with open(cache, "w", encoding="utf-8") as f:
            _json.dump(got, f)
    a = copy.deepcopy(a)
    sec = next(c for c in a["checks"] if c.get("name") == "SECTION_MIN")
    for r, o in zip(sec["details"]["sections"], got):
        r.setdefault("outline", o)
    return a
