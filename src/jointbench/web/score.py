"""`jointbench score <runs-folder> --out <file>` -- one results file for the scores page.

A runs folder holds one sub-folder per model run, each holding task folders
as `jointbench gen` writes them, with the attempts log the run produced:

    <runs>/<run>/run.json                 optional: name, date, tries_allowed, notes, tool, ...
    <runs>/<run>/<task-id>/task.json
    <runs>/<run>/<task-id>/attempts.jsonl

Nothing is re-graded. Every pass or fail is read from the attempts log as
the grader wrote it; this file only counts.

Definitions (stated on the page too):

* tries for a task -- attempts up to and including the first pass, or all
  attempts when it never passed; attempts after the first pass are kept in
  the log but do not count.
* passed first try -- attempt 1 passed.
* passed within allowed -- some attempt numbered <= tries_allowed passed
  (every attempt counts when run.json gives no limit).
* average tries -- mean tries over the tasks that passed within allowed.

Two kinds of figure are kept apart. *Measured* figures come from the
grader (pass/fail, checks, attempt counts). *Reported* figures (time,
tokens, cost, tool calls, CAD tool) are whatever the submitter wrote into
the log or run.json; a missing one stays missing (``None``), never zero.
"""
from __future__ import annotations

import datetime as _dt
import json
import os

from . import data
from .ladder import ladder as _ladder, reach as _reach

SCORES_VERSION = 1

from .rules import task_version  # noqa: E402
from .rules import CURRENT_RULES, current_generator as _current_generator, task_rules  # noqa: F401


# reported figures: names as `verify` records them (archive.py: REPORTED_*,
# under "reported", the tool at top level), plus a few older aliases;
# first match wins
REPORTED_KEYS = {
    "seconds": ("wall_seconds", "seconds", "wall_clock_seconds", "elapsed_seconds", "duration_seconds"),
    "tokens_in": ("tokens_in", "input_tokens", "prompt_tokens"),
    "tokens_out": ("tokens_out", "output_tokens", "completion_tokens"),
    # input and output together, when the submitter could not split them;
    # never added to either column above
    "tokens_total": ("tokens_total", "total_tokens"),
    "cost": ("cost", "cost_usd", "price"),
    "tool_calls": ("tool_calls", "n_tool_calls"),
}
TEXT_KEYS = {
    "tool": ("tool", "cad_tool"),
    "currency": ("currency", "cost_currency"),
    "model": ("model",),
    "model_version": ("model_version",),
    "note": ("note", "notes"),
}


def _pick(rec: dict, names):
    for src in (rec, rec.get("reported") or {}, rec.get("usage") or {}, rec.get("submitter") or {}):
        if not isinstance(src, dict):
            continue
        for k in names:
            v = src.get(k)
            if v is not None and v != "":
                return v
    return None


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def reported(rec: dict, run: dict | None = None) -> dict:
    """The submitter-reported figures of one attempt, None where absent."""
    out = {k: _num(_pick(rec, names)) for k, names in REPORTED_KEYS.items()}
    for k, names in TEXT_KEYS.items():
        v = _pick(rec, names)
        if v is None and run is not None and k in ("tool", "currency", "model", "model_version"):
            v = _pick(run, names)
        out[k] = None if v is None else str(v)
    return out


def _material(rec: dict):
    m = (rec.get("info") or {}).get("material") or {}
    r = m.get("ratio")
    if not isinstance(r, (int, float)) or not rec.get("passed"):
        return None, None
    if m.get("counts") is False:
        return None, f"material score {r:.3f}, not counted: {m.get('why_not') or 'no reason given'}"
    return float(r), None


def step_origin(rec: dict) -> str | None:
    """What the STEP file's own header says wrote it. Measured from the file
    (not reported by the caller), but a program can write anything there."""
    h = rec.get("step_header")
    if isinstance(h, dict) and h.get("originating_system"):
        return str(h["originating_system"])
    return None


def find_runs(root: str) -> list[str]:
    """Sub-folders of root that hold task folders with an attempts log."""
    root = os.path.abspath(root)
    out = []
    for name in sorted(os.listdir(root)):
        d = os.path.join(root, name)
        if not os.path.isdir(d) or name.startswith(".") or os.path.exists(os.path.join(d, "task.json")):
            continue
        tasks = data.find_tasks(d)
        # a run is a folder of tasks that someone attempted; a folder of
        # freshly generated tasks (no attempts log anywhere) is not a run
        if tasks and any(os.path.exists(os.path.join(t, "attempts.jsonl")) for t in tasks):
            out.append(d)
    return out


def is_runs_folder(path: str) -> bool:
    return os.path.isdir(path) and not os.path.exists(os.path.join(path, "task.json")) and bool(find_runs(path))


def run_dirs(root: str) -> list[str]:
    """Sub-folders of root laid out as runs (<run>/<task>/task.json), whether
    or not anything has been graded in them yet."""
    root = os.path.abspath(root)
    out = []
    if not os.path.isdir(root):
        return out
    for name in sorted(os.listdir(root)):
        d = os.path.join(root, name)
        if os.path.isdir(d) and not name.startswith(".") and not os.path.exists(os.path.join(d, "task.json")) and data.find_tasks(d):
            out.append(d)
    return out


def looks_like_runs(path: str) -> bool:
    """A runs folder by its layout: no task.json of its own, no task folders
    directly in it, and sub-folders that hold task folders. Used by `play`
    so a runs folder opens before anything is graded."""
    return (os.path.isdir(path) and not os.path.exists(os.path.join(path, "task.json"))
            and not data.find_tasks(path) and bool(run_dirs(path)))


def run_tasks(root: str) -> list[tuple[str, str]]:
    """(key, task dir) for every task of every run, attempted or not; key is
    '<run>__<task>'. (Scoring counts only runs with an attempts log: find_runs.)"""
    out = []
    for r in run_dirs(root):
        for t in data.find_tasks(r):
            out.append((f"{os.path.basename(r)}__{os.path.basename(t)}", t))
    return out


def _sum(vals):
    vals = [v for v in vals if v is not None]
    return (sum(vals), len(vals)) if vals else (None, 0)


def task_score(task_dir: str, tries_allowed: int | None, run: dict) -> dict:
    with open(os.path.join(task_dir, "task.json"), encoding="utf-8") as f:
        tj = json.load(f)
    atts = data.read_attempts(task_dir)
    first_pass = next((a["n"] for a in atts if a.get("passed")), None)
    rules = None   # set below, from the submission (the attempt the rules are judged on)
    limit = tries_allowed if tries_allowed else None
    within = first_pass is not None and (limit is None or first_pass <= limit)
    # the submission: what counts as this run's answer to the task -- the
    # first pass within the allowed tries, or else the last allowed attempt.
    # Never the best attempt picked afterwards.
    allowed = [a for a in atts if limit is None or a["n"] <= limit]
    sub = next((a for a in allowed if a.get("passed")), None) or (allowed[-1] if allowed else None)
    rules = task_rules(tj, sub)
    used = atts if first_pass is None else [a for a in atts if a["n"] <= first_pass]
    fails: dict[str, int] = {}
    for a in used:
        for c in a.get("checks", []):
            # only checks that decide the verdict; advisory ones (required: false,
            # e.g. Q_* and FDM_* below level 4) are not why a task failed
            if c.get("status") == "fail" and c.get("required", True):
                fails[c["name"]] = fails.get(c["name"], 0) + 1
    rows = []
    for a in atts:
        rep = reported(a, run)
        rows.append({
            "n": a.get("n"), "passed": a.get("passed"), "time": a.get("time"), "by": a.get("by"),
            "sha256": a.get("sha256"),
            "counted": first_pass is None or a["n"] <= first_pass,
            "checks": [{"name": c["name"], "status": c["status"]} for c in a.get("checks", [])],
            "reported": rep, "step_origin": step_origin(a),
            # the material score (par volume / part volume, higher is leaner), as the grader wrote it
            # (info.material); only when it counts, with the reason when not
            "material_ratio": _material(a)[0], "material_note": _material(a)[1],
        })
    figs = {}
    # run.json may give a task's figures when the log does not:
    # {"per_task": {"<task id>": {"tokens_total": .., "wall_seconds": ..}}}
    # (also accepted as "tasks") -- for the task as a whole, so they count as
    # covering every counted try
    pt = run_per_task(run)
    per_task = pt.get(tj["id"]) if pt else None
    for k, names in REPORTED_KEYS.items():
        total, n = _sum(r["reported"][k] for r in rows if r["counted"])
        figs[k] = {"total": total, "of": n, "attempts": len(used)}
        if total is None and isinstance(per_task, dict):
            v = _num(_pick(per_task, names))
            if v is not None:
                figs[k] = {"total": v, "of": len(used), "attempts": len(used), "source": "run.json"}
    tools = sorted({r["reported"]["tool"] for r in rows if r["reported"]["tool"]})
    return {
        "id": tj["id"], "level": tj["level"], "seed": tj["seed"],
        "attempts": len(atts), "tries": len(used), "first_pass": first_pass,
        "submission": None if sub is None else {
            "n": sub["n"], "passed": bool(sub.get("passed")), "sha256": sub.get("sha256"),
            "failed": [c["name"] for c in sub.get("checks", []) if c.get("status") == "fail" and c.get("required", True)],
            "info": sub.get("info") or {},
            # the requirement ladder of the submission (ladder.py): how far it got, not a pass
            "ladder": _ladder(sub, tj)},
        # the process the task states (its version, when one challenge exists per process)
        "process": ((tj.get("tolerances") or {}).get("manufacturing") or {}).get("process"),
        "family": tj.get("family") or "connect",
        "version": task_version(tj),
        "passed_first": first_pass == 1, "passed_within": within,
        "failed_checks": fails, "reported": figs, "tools": tools, "rows": rows,
        "rules": rules,
    }


def run_per_task(run: dict) -> dict | None:
    for k in ("per_task", "tasks"):
        if isinstance(run.get(k), dict):
            return run[k]
    return None


def run_means(run: dict) -> dict:
    """Run-level figures in run.json, read as the mean per task (as the
    model-run harness writes them). With per-task figures present, the mean
    is computed from those instead, so the two can never disagree."""
    pt = run_per_task(run)
    out = {}
    for k, names in REPORTED_KEYS.items():
        if pt:
            vals = [_num(_pick(v, names)) for v in pt.values() if isinstance(v, dict)]
            vals = [v for v in vals if v is not None]
            out[k] = sum(vals) / len(vals) if vals else None
        else:
            out[k] = _num(_pick(run, names))
    return out


def score_run(run_dir: str) -> dict:
    run = {}
    rp = os.path.join(run_dir, "run.json")
    if os.path.exists(rp):
        with open(rp, encoding="utf-8") as f:
            run = json.load(f)
    rid = os.path.basename(run_dir)
    tries_allowed = run.get("tries_allowed")
    tasks = []
    for t in data.find_tasks(run_dir):
        ts = task_score(t, tries_allowed, run)
        ts["key"] = f"{rid}__{os.path.basename(t)}"
        tasks.append(ts)
    attempted = [t for t in tasks if t["attempts"] > 0]
    passed = [t for t in attempted if t["passed_within"]]
    by_level: dict = {}
    for t in attempted:
        b = by_level.setdefault(str(t["level"]), {"attempted": 0, "passed_first": 0, "passed_within": 0})
        b["attempted"] += 1
        b["passed_first"] += int(t["passed_first"])
        b["passed_within"] += int(t["passed_within"])
    fails: dict[str, int] = {}
    for t in attempted:
        for k, v in t["failed_checks"].items():
            fails[k] = fails.get(k, 0) + v
    figs = {}
    for k in REPORTED_KEYS:
        total, n = _sum(t["reported"][k]["total"] for t in attempted)
        figs[k] = {"total": total, "tasks_with": n, "tasks": len(attempted)}
    # time to pass: reported seconds summed over the tries of passed tasks
    tp, tpn = _sum(t["reported"]["seconds"]["total"] for t in passed
                   if t["reported"]["seconds"]["of"] == t["tries"])
    tools = sorted({x for t in tasks for x in t["tools"]} | ({run["tool"]} if run.get("tool") else set()))
    currency = next((r["reported"]["currency"] for t in tasks for r in t["rows"] if r["reported"]["currency"]), None)
    return {
        # current only when every task it ran was graded under the current rules
        "rules": {"current": bool(tasks) and all(t["rules"]["current"] for t in tasks),
                  "why": sorted({w for t in tasks for w in t["rules"]["why"]})},
        "id": rid,
        "name": run.get("name") or run.get("display_name") or rid,
        # runs of the same model are grouped by this (run.json "model", else the name)
        "model": run.get("model") or run.get("name") or run.get("display_name") or rid,
        "date": run.get("date"), "notes": run.get("notes"), "reported_note": run.get("reported_note"),
        "tries_allowed": tries_allowed,
        "model_version": run.get("model_version"),
        "tools": tools, "currency": currency or run.get("currency"),
        "run_json": run,
        "summary": {
            "tasks": len(tasks), "attempted": len(attempted),
            "passed_first": sum(int(t["passed_first"]) for t in attempted),
            "passed_within": len(passed),
            "avg_tries": round(sum(t["tries"] for t in passed) / len(passed), 3) if passed else None,
            "attempts": sum(t["attempts"] for t in tasks),
            "by_level": by_level,
            "failed_checks": dict(sorted(fails.items(), key=lambda kv: -kv[1])),
            # per rung: of the parts where it was measured, how many met it and every measured rung below
            "ladder": _reach([t["submission"]["ladder"] for t in attempted if t.get("submission")]),
            "most_failed": max(fails, key=fails.get) if fails else None,
            "reported": figs,
            "time_to_pass": {"total": tp, "tasks_with": tpn, "tasks": len(passed)},
            # mean per task as run.json states it (top level, or under
            # "reported"/"usage"), or from its per_task figures; used only
            # where the log has nothing
            "run_reported": run_means(run),
            "run_reported_basis": "mean per task",
        },
        "tasks": tasks,
    }


def score(root: str) -> dict:
    root = os.path.abspath(root)
    runs = [score_run(r) for r in find_runs(root)]
    return {
        "version": SCORES_VERSION,
        "generated": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"),
        "root": root,
        "definitions": {
            "tries": "attempts up to and including the first pass; all attempts if it never passed",
            "passed_first": "attempt 1 passed",
            "passed_within": "an attempt numbered <= tries_allowed passed (all attempts count if no limit)",
            "avg_tries": "mean tries over tasks passed within the allowed tries",
            "measured": "pass/fail, checks and attempt counts come from the grader's attempts log",
            "reported": "time, tokens, cost, tool calls and CAD tool are as the submitter reported them; "
                        "missing figures are null, never zero",
        },
        "current_rules": {"label": CURRENT_RULES["label"], "since_generator": CURRENT_RULES["since_generator"]},
        "runs": runs,
    }


def write(root: str, out: str, log=print) -> int:
    if not is_runs_folder(root):
        log(f"{root}: no <run>/<task>/task.json folders found")
        return 2
    s = score(root)
    os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(s, f, indent=1)
    for r in s["runs"]:
        m = r["summary"]
        avg = "-" if m["avg_tries"] is None else f"{m['avg_tries']:.2f}"
        log(f"  {r['name']:<24} {m['passed_within']}/{m['attempted']} passed, "
            f"{m['passed_first']} first try, avg tries {avg}"
            + ("" if r["rules"]["current"] else "  [earlier rules: development data]"))
    log(f"wrote {out}")
    return 0
