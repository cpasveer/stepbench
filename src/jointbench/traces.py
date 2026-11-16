"""`traces <folder> --out <file.jsonl>`: attempt sequences as training records.

One record per (task, submitter): the brief and task data the submitter saw,
then every attempt in order -- the code, the grader's feedback exactly as it
was printed, the verdict. Code, feedback, revised code, ..., pass: that
sequence is the training example.

The folder may be one task, a folder of tasks, or a runs folder
(<runs>/<run>/<task>/ with an optional <runs>/<run>/run.json). Nothing is
re-graded; verdicts are read from the attempt log as the grader wrote them.
The reference answer in ``_solution/`` is never included.
"""
from __future__ import annotations

import datetime as _dt
import json
import os

from .archive import INDEX

TRACE_FORMAT_VERSION = 1


def _membership(d):
    from .freeze import membership

    return membership(d)


def _read(path: str) -> str | None:
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        return f.read()


def _task_dirs(target: str):
    for root, dirs, files in os.walk(target):
        dirs[:] = sorted(d for d in dirs if not d.startswith((".", "_")) and d != "attempts")
        if "task.json" in files:
            yield root
            dirs[:] = []


def _time(s: str | None):
    if not s:
        return None
    try:
        return _dt.datetime.fromisoformat(s)
    except ValueError:
        return None


def _reconstructed_feedback(rec: dict) -> str:
    """Old log lines (before the archive) did not keep the printed text. Rebuild
    it from the stored checks; marked as reconstructed in the record."""
    from .checks.base import CheckResult
    from .grade import Report

    checks = [CheckResult(c["name"], {"pass": True, "fail": False}.get(c.get("status")), c.get("reason", ""),
                          c.get("value"), c.get("unit", "")) for c in rec.get("checks", [])]
    return Report(rec.get("task", ""), bool(rec.get("passed")), checks, rec.get("info") or {}).text()


def _attempt(task_dir: str, rec: dict, prev_time) -> dict:
    files = rec.get("files") or {}
    feedback = _read(os.path.join(task_dir, files["feedback"])) if files.get("feedback") else None
    reconstructed = feedback is None
    if reconstructed:
        feedback = _reconstructed_feedback(rec)
    t = _time(rec.get("time"))
    return {
        "n": rec.get("n"),
        "time": rec.get("time"),
        "seconds_since_previous": (t - prev_time).total_seconds() if (t and prev_time) else None,
        "code": _read(os.path.join(task_dir, files["script"])) if files.get("script") else None,
        "code_link": files.get("script_link"),
        "feedback": feedback,
        "feedback_format": rec.get("feedback_format", "text"),
        "feedback_reconstructed": reconstructed,
        "passed": rec.get("passed"),
        "verdict": rec.get("verdict"),  # requirements pass/fail and the material score; absent in old logs
        "failed_checks": [c["name"] for c in rec.get("checks", []) if c.get("status") == "fail"],
        "checks": rec.get("checks", []),
        "info": rec.get("info", {}),
        "sha256": rec.get("sha256"),
        "step_header": rec.get("step_header"),
        "tool": rec.get("tool"),
        "reported": rec.get("reported", {}),
        "note": _read(os.path.join(task_dir, files["note"])) if files.get("note") else None,
        "archive": rec.get("archive"),
    }


def records(target: str):
    for d in _task_dirs(target):
        log = os.path.join(d, INDEX)
        if not os.path.exists(log):
            continue
        rows = []
        with open(log, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    continue  # a torn line is skipped, never guessed at
        run_json = _read(os.path.join(os.path.dirname(d), "run.json"))
        run = json.loads(run_json) if run_json else None
        task = json.loads(_read(os.path.join(d, "task.json")))
        brief = _read(os.path.join(d, "README.md"))
        by_sub: dict = {}
        for r in rows:
            by_sub.setdefault(r.get("by", "unknown"), []).append(r)
        for who in sorted(by_sub):
            seq = sorted(by_sub[who], key=lambda r: r.get("n") or 0)
            atts, prev = [], None
            for r in seq:
                atts.append(_attempt(d, r, prev))
                prev = _time(r.get("time")) or prev
            first = next((a for a in atts if a["passed"]), None)
            t0, tp = _time(atts[0]["time"]), _time(first["time"]) if first else None
            allowed = (run or {}).get("tries_allowed")
            score = ((first or {}).get("verdict") or {}).get("score") or {}
            yield {
                "trace_format_version": TRACE_FORMAT_VERSION,
                "task_id": task.get("id"),
                "family": task.get("family", "connect"),
                "level": task.get("level"),
                "seed": task.get("seed"),
                "by": who,
                "task_dir": os.path.relpath(d, target),
                "task_set": _membership(d),  # frozen set name, version, fingerprint; None if not in a set
                "run": run,
                "brief": brief,
                "task": task,
                "attempts": atts,
                "summary": {
                    "n_attempts": len(atts),
                    "passed": first is not None,
                    "first_pass_attempt": first["n"] if first else None,
                    "passed_first_try": bool(atts and atts[0]["passed"]),
                    "passed_within_allowed": (None if allowed is None else
                                              any(a["passed"] for a in atts[: int(allowed)])),
                    # the one score: material at the first passing attempt
                    # (None if it does not pass or does not count)
                    "material": score.get("material"),
                    "seconds_first_attempt_to_pass": (tp - t0).total_seconds() if (t0 and tp) else None,
                    "tools": sorted({a["tool"] for a in atts if a["tool"]}),
                },
            }


def write(target: str, out: str) -> tuple[int, int]:
    n = n_att = 0
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        for rec in records(target):
            f.write(json.dumps(rec) + "\n")
            n += 1
            n_att += len(rec["attempts"])
    return n, n_att
