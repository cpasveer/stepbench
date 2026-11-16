"""A person's tries, kept as a run of its own.

When someone uses "Try it yourself" on a task page, their uploads are never
written into the task's own attempts log (in a runs folder that log belongs
to a model). Each person gets a run folder in exactly the layout a model
run has, so `jointbench score` reads it like any other:

    <runs>/human-<name>/run.json
    <runs>/human-<name>/<task-id>/task.json, README.md, fixed_*.step, obstacles.step
    <runs>/human-<name>/<task-id>/attempts.jsonl and attempts/NNNN/  (written by the grader)

run.json says who and with what:

    {"name": "alex", "model": "human", "human": true, "tool": "FreeCAD",
     "tries_allowed": 6, "clock": {"L3-s950": {"started": "...Z"}},
     "per_task": {"L3-s950": {"wall_seconds": 1834.2, "first_pass": 2}}}

Time is measured by this server, not reported: from the moment the person
starts the clock on the task page (their first download, or the start
button) to their first passing upload. A task they never started has no
time, and no time is ever estimated.

Where <runs> is: the runs folder `play` was given; for a folder of tasks,
<tasks>/human-runs; for one task folder, <task>/.play/human-runs.
"""
from __future__ import annotations

import datetime as _dt
import json
import os
import re
import shutil
import threading

from . import data

PREFIX = "human-"
_lock = threading.Lock()


def slug(name: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "-", (name or "").lower()).strip("-")[:30]
    return s or "person"


def _now() -> str:
    return _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds")


def runs_root(root: str, tasks: dict, runs_folder: str | None) -> str:
    if runs_folder:
        return runs_folder
    dirs = list(tasks.values())
    if len(dirs) == 1 and os.path.abspath(root) == os.path.abspath(dirs[0]):
        return os.path.join(dirs[0], data.VIEW_DIR, "human-runs")
    return os.path.join(root, "human-runs")


def _read(p: str) -> dict:
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _write(p: str, obj: dict) -> None:
    tmp = p + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=1)
    os.replace(tmp, p)


def task_dir(root: str, src_dir: str, who: str, tool: str | None) -> tuple[str, str, str]:
    """The person's copy of a task (made on first use): (run id, run dir,
    task dir). The copy holds only what a person is given; never the
    reference answer, never anyone else's attempts."""
    run_id = PREFIX + slug(who)
    rdir = os.path.join(root, run_id)
    name = os.path.basename(os.path.normpath(src_dir))
    tdir = os.path.join(rdir, name)
    with _lock:
        if not os.path.exists(os.path.join(tdir, "task.json")):
            os.makedirs(tdir, exist_ok=True)
            for f in data.task_files(src_dir):
                shutil.copyfile(os.path.join(src_dir, f), os.path.join(tdir, f))
        rp = os.path.join(rdir, "run.json")
        run = _read(rp)
        if not run:
            src_run = _read(os.path.join(os.path.dirname(os.path.normpath(src_dir)), "run.json"))
            run = {"name": who, "model": "human", "human": True, "date": _now()[:10],
                   "tries_allowed": src_run.get("tries_allowed"),
                   "notes": "A person, through Try it yourself on the task page. Time is measured by "
                            "the server from starting the clock to the first passing upload.",
                   "clock": {}, "per_task": {}}
        if tool:
            run["tool"] = tool
        _write(rp, run)
    return run_id, rdir, tdir


def peek(root: str, src_dir: str, who: str) -> dict | None:
    """This person's clock for a task, if they have one; never creates anything."""
    rdir = os.path.join(root, PREFIX + slug(who))
    tdir = os.path.join(rdir, os.path.basename(os.path.normpath(src_dir)))
    if not os.path.exists(os.path.join(tdir, "task.json")):
        return None
    run = _read(os.path.join(rdir, "run.json"))
    tid = task_id(tdir)
    c = (run.get("clock") or {}).get(tid)
    return {**(c or {}), **(run.get("per_task") or {}).get(tid, {})} if c or tid in (run.get("per_task") or {}) else None


def task_id(tdir: str) -> str:
    return _read(os.path.join(tdir, "task.json")).get("id") or os.path.basename(tdir)


def start(rdir: str, tdir: str) -> dict:
    """Start the clock for this task, once; returns the task's clock."""
    tid = task_id(tdir)
    with _lock:
        rp = os.path.join(rdir, "run.json")
        run = _read(rp)
        clock = run.setdefault("clock", {})
        if tid not in clock:
            clock[tid] = {"started": _now()}
            _write(rp, run)
        return {**clock[tid], **(run.get("per_task") or {}).get(tid, {})}


def graded(rdir: str, tdir: str, n: int, passed: bool) -> dict:
    """After an upload: on the first pass, record the time since the clock started."""
    tid = task_id(tdir)
    with _lock:
        rp = os.path.join(rdir, "run.json")
        run = _read(rp)
        per = run.setdefault("per_task", {})
        started = (run.get("clock") or {}).get(tid, {}).get("started")
        if passed and "first_pass" not in per.get(tid, {}):
            rec = per.setdefault(tid, {})
            rec["first_pass"] = n
            if started:
                t0 = _dt.datetime.fromisoformat(started)
                rec["wall_seconds"] = round((_dt.datetime.now(_dt.timezone.utc) - t0).total_seconds(), 1)
            _write(rp, run)
        return {"started": started, **per.get(tid, {})}
