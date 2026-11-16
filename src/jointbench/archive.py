"""Keeping every graded attempt.

Each graded attempt gets its own folder inside the task folder, and one line
in ``attempts.jsonl`` that points to it:

    <task>/attempts.jsonl            the index, one JSON line per attempt
    <task>/attempts/0001/
        output.step                  the exact file that was graded
        submission.py                a script, if one was linked to the attempt; how
                                     it is linked is recorded: "ran" (verify --build
                                     ran it), "named" (--script), or "found" (it sat
                                     in the task folder; not proof it made the file)
        feedback.txt                 the grader's output exactly as printed
        result.json                  full grader result plus everything below
        note.txt                     free text from the caller, as given (optional)

Two kinds of information are recorded and kept apart:

* **measured** -- by the grader itself: verdict, every check with reason and
  number, the grader's own timestamp, the file's SHA-256, and what the STEP
  file's own header says about the program that wrote it (read verbatim,
  never graded on);
* **reported** -- by the caller, stored as given and never estimated: the
  CAD tool used, wall-clock seconds, tokens in/out, cost and currency, model
  name and version, number of tool calls, a note.
"""
from __future__ import annotations

import datetime as _dt
import hashlib
import json
import os
import re
import shutil

ARCHIVE_DIR = "attempts"
INDEX = "attempts.jsonl"
ERROR_DIR = "errors"          # gradings the grader could not complete; not attempts
ERROR_INDEX = "errors.jsonl"

# reported figures the caller may give; names match what `jointbench score`
# reads. Numbers stay numbers, text stays text; absent stays absent.
REPORTED_NUMBERS = ("wall_seconds", "tokens_in", "tokens_out", "cost", "tool_calls")
REPORTED_TEXT = ("currency", "price_date", "model", "model_version")
SUGGESTED_TOOLS = ("build123d", "cadquery", "openscad", "freecad", "anvil", "other")


def sha256(path: str) -> str | None:
    if not path or not os.path.exists(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


_STR = re.compile(r"'((?:[^']|'')*)'")


def step_header(path: str) -> dict | None:
    """The STEP header's own account of who wrote the file, verbatim.

    FILE_NAME carries (name, time_stamp, author, organization,
    preprocessor_version, originating_system, authorization). We keep those
    strings exactly as written, plus the raw FILE_NAME and FILE_DESCRIPTION
    statements. Unreadable or absent: None. Never used for grading."""
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path, "rb") as f:
            head = f.read(16384).decode("latin-1")
    except OSError:
        return None
    end = head.find("ENDSEC;")
    head = head[: end if end >= 0 else len(head)]
    out: dict = {}
    for key in ("FILE_DESCRIPTION", "FILE_NAME", "FILE_SCHEMA"):
        i = head.find(key + "(")
        if i < 0:
            continue
        j = head.find(");", i)
        stmt = head[i: j + 2 if j >= 0 else len(head)]
        out[key.lower()] = " ".join(stmt.split())
    fn = out.get("file_name")
    if fn:
        # top-level fields in order; author and organization are lists
        body = fn[len("FILE_NAME("):-2]
        parts, depth, cur, q = [], 0, "", False
        for ch in body:
            if ch == "'" :
                q = not q
            if not q and ch == "(":
                depth += 1
            if not q and ch == ")":
                depth -= 1
            if not q and depth == 0 and ch == ",":
                parts.append(cur.strip())
                cur = ""
                continue
            cur += ch
        parts.append(cur.strip())
        names = ("name", "time_stamp", "author", "organization",
                 "preprocessor_version", "originating_system", "authorization")
        for k, v in zip(names, parts):
            vals = [s.replace("''", "'") for s in _STR.findall(v)]
            out[k] = vals if k in ("author", "organization") else (vals[0] if vals else v)
    return out or None


def _membership(task_dir: str):
    from .freeze import membership

    return membership(task_dir)


def _next_number(task_dir: str, index: str = INDEX, archive_dir: str = ARCHIVE_DIR) -> int:
    n = 1
    p = os.path.join(task_dir, index)
    if os.path.exists(p):
        with open(p, encoding="utf-8") as f:
            n = sum(1 for line in f if line.strip()) + 1
    while os.path.exists(os.path.join(task_dir, archive_dir, f"{n:04d}")):
        n += 1  # never overwrite an archived attempt
    return n


def _tool_check(tool: str, step_path: str) -> str | None:
    from .env import tool_vs_header

    return tool_vs_header(tool, step_header(step_path))


def checks_used(task_dir: str) -> int:
    """Graded attempts on record for a task folder: the larger of the index
    lines and the archived attempt folders, so emptying one of the two does
    not give checks back. Gradings the grader could not complete (errors/)
    are not attempts and are not counted."""
    lines = 0
    p = os.path.join(task_dir, INDEX)
    if os.path.exists(p):
        with open(p, encoding="utf-8") as f:
            lines = sum(1 for line in f if line.strip())
    folders = 0
    d = os.path.join(task_dir, ARCHIVE_DIR)
    if os.path.isdir(d):
        folders = sum(1 for n in os.listdir(d) if re.fullmatch(r"\d{4}", n))
    return max(lines, folders)


def clean_reported(reported: dict | None) -> dict:
    """Keep only values the caller actually gave."""
    out = {}
    for k, v in (reported or {}).items():
        if v is None or v == "":
            continue
        out[k] = v
    return out


def record_attempt(task_dir: str, step_path: str, report, *, by: str = "verify",
                   script: str | None = None, script_link: str | None = None, feedback: str | None = None,
                   feedback_format: str = "text", note: str | None = None,
                   tool: str | None = None, reported: dict | None = None) -> int:
    """Archive one graded attempt and append its index line. Returns its number.

    A grading the grader could not complete (outcome "error": a check could
    not measure the part) is no attempt: it is kept under ``errors/`` with
    its own ``errors.jsonl``, numbered apart, and never enters the attempts
    log that scores and traces count."""
    is_error = getattr(report, "outcome", lambda: None)() == "error"
    index, archive_dir = (ERROR_INDEX, ERROR_DIR) if is_error else (INDEX, ARCHIVE_DIR)
    n = _next_number(task_dir, index, archive_dir)
    rel = f"{archive_dir}/{n:04d}"
    folder = os.path.join(task_dir, rel)
    os.makedirs(folder)
    files = {}
    if step_path and os.path.exists(step_path):
        name = os.path.basename(step_path)
        shutil.copyfile(step_path, os.path.join(folder, name))
        files["step"] = f"{rel}/{name}"
        place = os.path.join(os.path.dirname(step_path), "placement.json")
        if os.path.exists(place):  # an adapt task's answer is the part and where it goes
            shutil.copyfile(place, os.path.join(folder, "placement.json"))
            files["placement"] = f"{rel}/placement.json"
    if script and os.path.exists(script):
        shutil.copyfile(script, os.path.join(folder, "submission.py"))
        files["script"] = f"{rel}/submission.py"
        files["script_link"] = script_link or "named"
    shown = feedback if feedback is not None else report.text()
    with open(os.path.join(folder, "feedback.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write(shown if shown.endswith("\n") else shown + "\n")
    files["feedback"] = f"{rel}/feedback.txt"
    if note is not None:
        with open(os.path.join(folder, "note.txt"), "w", encoding="utf-8", newline="\n") as f:
            f.write(note)
        files["note"] = f"{rel}/note.txt"
    rec = {
        "n": n,
        "time": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="milliseconds"),
        "by": by,
        "file": os.path.relpath(step_path, task_dir) if step_path else None,
        "sha256": sha256(step_path),
        "archive": rel,
        "files": files,
        "feedback_format": feedback_format,
        "step_header": step_header(step_path),
        "task_set": _membership(task_dir),
        **({"tool": tool} if tool else {}),
        **({"tool_vs_header": _tool_check(tool, step_path)} if tool else {}),
        **({"reported": clean_reported(reported)} if clean_reported(reported) else {}),
        **report.to_json(),
    }
    with open(os.path.join(folder, "result.json"), "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(rec, indent=2) + "\n")
    with open(os.path.join(task_dir, index), "a", encoding="utf-8") as f:
        f.write(json.dumps(rec) + "\n")
    return n


# --------------------------------------------------------------------------
# helpers for the command line


def submission_script(task_dir: str, given: str | None) -> str | None:
    """The script to archive: the one named, else <task>/submission.py --
    but not the untouched stub, which would be recorded as code nobody wrote."""
    if given:
        return given
    p = os.path.join(task_dir, "submission.py")
    if not os.path.exists(p):
        return None
    from . import NAME
    from .workspace import SUBMISSION_STUB

    from .moves.l6.starter import submission as l6_starter

    with open(p, encoding="utf-8") as f:
        if f.read() in (SUBMISSION_STUB.replace("CLI_NAME", NAME), l6_starter()):
            return None
    return p


def read_note(path: str | None) -> str | None:
    if not path:
        return None
    with open(path, encoding="utf-8") as f:
        return f.read()


def reported_from_args(a) -> dict:
    rep = {}
    if getattr(a, "reported_file", None):
        with open(a.reported_file, encoding="utf-8") as f:
            rep.update(json.load(f))
    for k in REPORTED_NUMBERS + REPORTED_TEXT:
        v = getattr(a, k, None)
        if v is not None:
            rep[k] = v
    return clean_reported(rep)
