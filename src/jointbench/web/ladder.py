"""The requirement ladder of one graded part, and how far a model's parts get.

Rungs, in a fixed order: valid solid, fits, cleanly modelled, makeable (by
the task's stated process), stiff (the sturdy group: the minimum section and,
where a task states one, stiffness). Each rung is met, not met, or not
measured at that level, and required or not by the task. A part passes only
if it meets every required rung -- that is the grader's verdict, unchanged;
the ladder only shows how far a part got.

Read from the grader's own record: verdict.requirements per group where the
log has it (round 2 on), else the checks of that group; the valid-solid rung
is the FILE_VALID check. Nothing is re-graded.

Per model: a part *reaches* a rung when it meets that rung and every measured
rung below it. A rung not measured for a part neither counts for it nor
blocks the rungs above; its share is taken over the parts where it was
measured.
"""
from __future__ import annotations

RUNGS = (("valid", "valid solid"), ("fits", "fits"), ("clean", "cleanly modelled"),
         ("makeable", "makeable"), ("sturdy", "sturdy"))
MET, NOT_MET, NOT_MEASURED = "met", "not met", "not measured"


def _group_of(c: dict) -> str:
    if c.get("group"):
        return c["group"]
    n = c.get("name", "")
    if n.startswith("Q_"):
        return "clean"
    if n.split("_")[0] in ("FDM", "PBF", "IM", "MILL"):
        return "makeable"
    if n in ("SECTION_MIN", "STIFFNESS"):
        return "sturdy"
    return "fits"


def ladder(att: dict | None, tj: dict | None = None) -> list[dict]:
    """[{rung, label, state, required}] for one attempt record."""
    if not att:
        return [{"rung": r, "label": l, "state": NOT_MEASURED, "required": r in ("valid", "fits")} for r, l in RUNGS]
    checks = att.get("checks") or []
    req_v = ((att.get("verdict") or {}).get("requirements")) or {}
    req_t = (tj or {}).get("requirements") or {}
    out = []
    for r, label in RUNGS:
        if r == "valid":
            fv = next((c for c in checks if c.get("name") == "FILE_VALID"), None)
            st = NOT_MEASURED if not fv or fv.get("status") == "skipped" else MET if fv.get("status") == "pass" else NOT_MET
            out.append({"rung": r, "label": label, "state": st, "required": True})
            continue
        v = req_v.get(r)
        if v:
            res = v.get("result")
            st = MET if res == "pass" else NOT_MET if res == "fail" else NOT_MEASURED
            required = bool(v.get("required"))
        else:
            own = [c for c in checks if _group_of(c) == r and c.get("name") != "FILE_VALID" and c.get("status") != "skipped"]
            if not own:
                st = NOT_MEASURED
            else:
                # a group is met when none of its measured checks failed
                st = NOT_MET if any(c.get("status") == "fail" for c in own) else MET
            required = r == "fits" or req_t.get(r) == "required" or any(
                c.get("required") is True for c in checks if _group_of(c) == r)
        out.append({"rung": r, "label": label, "state": st, "required": required})
    return out


def reach(ladders: list[list[dict]]) -> list[dict]:
    """Per rung over several parts: measured, met, reached, required (counts)."""
    out = [{"rung": r, "label": l, "measured": 0, "met": 0, "reached": 0, "required": 0} for r, l in RUNGS]
    for lad in ladders:
        ok_below = True
        for k, step in enumerate(lad):
            o = out[k]
            if step["required"]:
                o["required"] += 1
            if step["state"] == NOT_MEASURED:
                continue
            o["measured"] += 1
            if step["state"] == MET:
                o["met"] += 1
                if ok_below:
                    o["reached"] += 1
            else:
                ok_below = False
    return out
