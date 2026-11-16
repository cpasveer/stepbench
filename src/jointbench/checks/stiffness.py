"""STIFFNESS: the part is stiff enough between its mounts (where required).

Calls the project's stiffness checker, ``jointbench.stiffness``, through its
public function ``screened`` (coarse elements first; the full analysis only
when the result falls inside the precision band); nothing of it is copied
here. The required figure is written into ``task.json`` at generation
(``"stiffness": {"min": N/mm, ...}``), a fraction of the reference answer's
own figure, so grading analyses only the answer. A "marginal" result -- too
close to the limit to tell at the checker's precision -- counts as a pass
and is flagged in the reason.

Slow (tens of seconds to minutes per part), so it runs only where a task
requires it or a run asks for it.
"""
from __future__ import annotations

import json
import os
import shutil
import tempfile

from .base import CheckResult, Context

NAME = "STIFFNESS"
GROUP = "sturdy"

# Time caps (maintainer, 13 November 2026). The analysis runs in a child process. CPU_CAP_S counts the
# processor time the analysis itself uses, so it does not depend on how busy the machine is: a part
# that needs more cannot be measured, and fails like any part that cannot be measured. WALL_CAP_S is a
# safety net under an agent's 10-minute command limit: past it (an overloaded machine) the result is a
# grader error, not a verdict, so the try is not counted.
CPU_CAP_S = 360
WALL_CAP_S = 540
ISOLATE = True  # False: analyse in this process, uncapped (tests that replace the analysis)


def _child_main(job: str) -> None:
    """Entry point of the analysis process: reads its job, runs screened(), writes the report.
    Started as its own command (``python -c``), so it imports only the grader, never the script
    that called it (a multiprocessing child would re-run a calling script that lacks a main guard)."""
    import pickle
    import resource

    with open(job, "rb") as f:
        task_dir, p, require, cap = pickle.load(f)
    resource.setrlimit(resource.RLIMIT_CPU, (cap, cap + 30))
    from ..stiffness import screened

    try:
        rep = screened(task_dir, p) if require is None else screened(task_dir, p, require=require)
        out = ("ok", rep)
    except Exception as e:  # noqa: BLE001 - reported back as no result
        out = ("raised", f"{type(e).__name__}: {e}")
    with open(job + ".out", "wb") as f:
        pickle.dump(out, f)


def _capped(task_dir, p, require):
    """screened() in its own process under the caps. Returns (report or None, why)."""
    import multiprocessing as mp
    import pickle
    import signal
    import subprocess
    import sys

    if not ISOLATE or mp.current_process().daemon:  # a pool worker: analyse here, uncapped
        from ..stiffness import screened

        return (screened(task_dir, p) if require is None else screened(task_dir, p, require=require)), ""
    job = os.path.join(task_dir, "stiffness-job.pkl")
    with open(job, "wb") as f:
        pickle.dump((task_dir, p, require, CPU_CAP_S), f)
    cmd = [sys.executable, "-c", "import sys; from jointbench.checks.stiffness import _child_main; _child_main(sys.argv[1])", job]
    proc = subprocess.Popen(cmd, start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        _, err = proc.communicate(timeout=WALL_CAP_S)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)  # its own group: any helper it started goes too
        except Exception:  # noqa: BLE001
            proc.kill()
        proc.communicate()
        return None, "wall"
    if proc.returncode in (-signal.SIGXCPU, -signal.SIGKILL):
        return None, "cpu"  # stopped by its CPU limit: the analysis needed too long
    if not os.path.exists(job + ".out"):
        tail = (err or b"").decode("utf-8", "replace").strip().splitlines()[-1:] or [""]
        return None, f"stopped (exit {proc.returncode}) {tail[0][:200]}"
    with open(job + ".out", "rb") as f:
        got = pickle.load(f)
    if got[0] == "raised":
        raise RuntimeError(got[1])
    return got[1], ""


def run(ctx: Context) -> CheckResult:
    st = ctx.task.stiffness or {}
    if "min" not in st and st.get("mode") not in ("spec", "stated"):
        return CheckResult(NAME, None, "no stiffness requirement stated for this task")
    if ctx.task.tolerances.get("compliance"):
        return _without_travel(ctx)
    with tempfile.TemporaryDirectory() as tmp:
        with open(os.path.join(tmp, "task.json"), "w", encoding="utf-8") as f:
            f.write(ctx.task.dumps())
        p = os.path.join(tmp, "output.step")
        shutil.copyfile(ctx.step_path, p)
        # spec mode reads the requirement from the task itself (per mount)
        # spec and stated modes read the requirement from the task itself
        rep, why = _capped(tmp, p, None if st.get("mode") in ("spec", "stated") else float(st["min"]))
    if why == "cpu":
        return CheckResult(NAME, False, f"could not be measured within {CPU_CAP_S // 60} minutes of analysis: "
                                        f"a part that cannot be measured fails (maintainer's rule)")
    if why:
        return CheckResult(NAME, None, f"could not be measured (grader error, not a verdict): the analysis was "
                                       f"{'stopped after ' + str(WALL_CAP_S // 60) + ' minutes on a busy machine' if why == 'wall' else why}",
                           error=True)
    if not rep.ok or rep.requirement is None:
        q = getattr(rep, "quality", None)
        if q is not None and rep.error and "separate solids" not in rep.error:
            if q.get("passed"):
                # a clean model the mesher could not handle: the tool's fault, no verdict
                return CheckResult(NAME, None, f"could not be measured (grader error, not a verdict): "
                                               f"{rep.error}", error=True)
            return CheckResult(NAME, False, f"could not be analysed: {rep.error}; the model is untidy: "
                                            f"{', '.join(q.get('failed') or []) or 'quality faults'}")
        return CheckResult(NAME, False, f"could not be analysed: {rep.error or 'no result'}")
    req = rep.requirement
    flag = " -- marginal, within the checker's precision; counted as a pass" if req.verdict == "marginal" else ""
    if getattr(rep, "fallback", ""):
        flag += f" -- {rep.fallback}"
    per = getattr(req, "per_mount", None)
    if per:  # spec mode: the worst mount decides
        w = min(per, key=lambda m: m["figure"] / m["required"])
        reason = (f"weakest mount {w['mount']}: {w['figure']:.1f} N/mm, need {w['required']:.1f} N/mm "
                  f"({req.source}; +-{req.precision:g}%){flag}")
        value = round(w["figure"] / w["required"], 4)
    else:
        fig = rep.figure.stiffness
        reason = (f"weakest way of loading ({rep.figure.mount}: {rep.figure.description}) {fig:.1f} N/mm, "
                  f"need {req.required:.1f} N/mm (+-{req.precision:g}%){flag}")
        value = round(fig / req.required, 4)
    return CheckResult(NAME, req.passed, reason, value, "ratio",
                       {"verdict": req.verdict, "per_mount": per, "precision": req.precision,
                        "stages": getattr(rep, "stages", None)})


def _without_travel(ctx: Context) -> CheckResult:
    """A task with a stated travel (COMPLIANCE): the part is meant to be soft
    along it, so the weakest way of loading is read with the travel left out,
    from the compliance analysis (in the printed material; the specification
    scales with the modulus, so the verdict is as calibrated)."""
    from .compliance import analysed

    rep = analysed(ctx)
    if not rep.ok:
        return CheckResult(NAME, False, f"could not be analysed: {rep.error}")
    s = rep.stiffness_rest
    if not s:
        return CheckResult(NAME, None, "no stiffness specification for this task")
    flag = " -- marginal, within the checker's precision; counted as a pass" if s["verdict"] == "marginal" else ""
    reason = (f"leaving out the travel, weakest way of loading ({s['description']}) {s['figure']:.1f} N/mm in "
              f"{rep.material}, need {s['required']:.1f} N/mm ({s['design_load_N']:g} N, at most "
              f"{s['allowed_deflection_mm']:.3g} mm; +-{rep.precision:g}%){flag}")
    return CheckResult(NAME, s["verdict"] in ("pass", "marginal"), reason, round(s["figure"] / s["required"], 4),
                       "ratio", {"verdict": s["verdict"], "without_travel": True, "material": rep.material})
