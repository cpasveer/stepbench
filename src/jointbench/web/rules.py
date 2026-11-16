"""The current rules, and whether a part was graded under them.

"Current rules" means: the part was graded against exactly the requirements
its task states (task.json "requirements": fits always; clean, makeable,
sturdy where the task says "required"), by a grader that records them, on a
task from a generator version that follows them. Nothing else is fixed here:
a joint task that requires clean modelling and a connecting-part task that
requires makeable both count, each by its own statement.

A part is graded under earlier rules when:
* its attempt has no requirement record (graded before the grader wrote
  verdict.requirements), or
* the record's required groups differ from what the task states now (the
  rules changed after grading), or
* its task comes from a generator version before the rules held for that
  family (CURRENT_RULES["since_generator"]: a minimum, never "the latest").

The site shows such runs as development data, never as the benchmark's
results; a static copy leaves them off its scoreboard unless asked. The label
is always recomputed from the task files and attempts logs, also when a
results file was scored earlier, so it follows the data, not the scorer.
"""
from __future__ import annotations

CURRENT_RULES = {
    # 4: the process is required at every level (core 72de47a). Families not
    # listed (joints, mechanisms) have their own generators and are judged by
    # their stated requirements alone.
    "since_generator": {"connect": 4},
    "label": "each part graded against exactly the requirements its task states "
             "(fits always; clean, makeable, sturdy where the task requires them), "
             "by the current grader, on a task from a generator that states them",
}

GROUP_NAME = {"MANUFACTURING": "makeable", "CLEAN": "clean", "SECTION_MIN": "sturdy", "STIFFNESS": "sturdy"}
GROUPS = ("fits", "clean", "makeable", "sturdy")


def current_generator(family: str = "connect") -> int | None:
    """The first generator version of a family whose tasks follow the current rules."""
    return CURRENT_RULES["since_generator"].get(family)


def task_requires(tj: dict) -> set[str]:
    """The requirement groups the task states as required."""
    req = tj.get("requirements")
    if isinstance(req, dict) and req:
        return {g for g, v in req.items() if v == "required"} | {"fits"}
    checks = set((tj.get("tolerances") or {}).get("required_checks") or [])
    return {"fits"} | {GROUP_NAME[c] for c in checks if c in GROUP_NAME}


def task_rules(tj: dict, att: dict | None = None) -> dict:
    """Whether a task (and, given one, an attempt on it) follows the current
    rules, and if not, why."""
    why = []
    fam = tj.get("family") or "connect"
    gv, since = tj.get("generator_version"), current_generator(fam)
    if since is not None and (gv or 1) < since:
        why.append(f"generator version {gv or 1}, the rules hold from {since}")
    wants = task_requires(tj)
    if att is not None:
        rec = ((att.get("verdict") or {}).get("requirements"))
        if not rec:
            why.append("graded before the grader recorded requirements")
        else:
            graded = {g for g, v in rec.items() if isinstance(v, dict) and v.get("required")} | {"fits"}
            if graded != wants:
                why.append(f"graded with {', '.join(sorted(graded))} required; the task states {', '.join(sorted(wants))}")
    return {"current": not why, "why": why, "generator_version": gv or 1, "required": sorted(wants)}


def refresh_rules(sc: dict, task_dir_of) -> dict:
    """Recompute every run's and task's rules in a results dict from the task
    files and attempts logs (a results file keeps the label of the code that
    scored it). task_dir_of(key) -> task folder or None (then left as scored)."""
    import json
    import os

    from .data import read_attempts

    for run in sc.get("runs", []):
        for t in run.get("tasks", []):
            d = task_dir_of(t.get("key"))
            if not d or not os.path.exists(os.path.join(d, "task.json")):
                continue
            with open(os.path.join(d, "task.json"), encoding="utf-8") as f:
                tj = json.load(f)
            sub = t.get("submission")
            att = next((a for a in read_attempts(d) if sub and a.get("n") == sub.get("n")), None)
            t["rules"] = task_rules(tj, att)
        ts = run.get("tasks", [])
        run["rules"] = {"current": bool(ts) and all((t.get("rules") or {}).get("current") for t in ts),
                        "why": sorted({w for t in ts for w in (t.get("rules") or {}).get("why", [])})}
    sc["current_rules"] = {"label": CURRENT_RULES["label"], "since_generator": CURRENT_RULES["since_generator"]}
    return sc


# Process versions of one challenge: the same situation (seed, fixed parts)
# made for filament printing, powder bed fusion and injection moulding. Its id
# carries the process last (L2-s3000-fdm, L3-s5-p2o2m4d25-moulding); task.json
# states the process in tolerances.manufacturing.process and knobs.process,
# and names the group in "situation" (identical across the versions). The
# declared build or pull direction is not in task.json: each graded attempt
# records it in the MANUFACTURING check's details (core session, 2026-11-12).
import re as _re

PROCESS_SUFFIXES = ("fdm", "pbf", "moulding")
_VERSION_RE = _re.compile(r"-(" + "|".join(PROCESS_SUFFIXES) + r")$")


def task_version(tj: dict) -> dict:
    """{"base": id without the process suffix, "process": the stated process,
    "versioned": whether the id carries a process suffix}."""
    tid = tj.get("id") or ""
    m = _VERSION_RE.search(tid)
    proc = (((tj.get("tolerances") or {}).get("manufacturing") or {}).get("process")
            or (tj.get("knobs") or {}).get("process"))
    base = tj.get("situation") or (tid[: m.start()] if m else tid)
    return {"base": base, "process": proc or (m.group(1) if m else None), "versioned": bool(m or tj.get("situation"))}
