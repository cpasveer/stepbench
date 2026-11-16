"""Command line: gen | verify | selftest | solve | play | site | score"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

from . import NAME
from .grade import grade, log_attempt
from . import families
from .task import Task
from .workspace import write_workspace


def cmd_gen(a) -> int:
    if a.setup:  # the front door: a test of the ladder (setup and level) -> its family and level
        from . import ladder

        try:
            a.family, a.level = ladder.resolve(a.setup, a.level)
        except ladder.NotBuilt as e:
            print(e)
            return 2
    if a.setup and (a.family, a.level) == ("moves", 6):  # level 6 moves: its own generator, one per setup
        return _gen_l6(a)
    mod = families.level(a.family, a.level)
    os.makedirs(a.out, exist_ok=True)
    knobs = dict(kv.split("=", 1) for kv in (a.knob or []))
    if knobs and not hasattr(mod, "KNOBS_DEFAULT"):
        print(f"level {a.level} has no knobs")
        return 2
    from .jobs import pmap

    for line in pmap(_gen_one, [(a.family, a.level, s, knobs, a.out) for s in range(a.seed, a.seed + a.n)], a.jobs):
        print(line)
    return 0


def _gen_l6(a) -> int:
    """Level 6 (moves) on a setup: the reference is built and graded first, then the folder is written with
    ``_solution/`` (reference and broken copies) inside it; move that out before an agent sees the folder."""
    from .moves.l6.generate import broken_variants, generate, write_folder

    os.makedirs(a.out, exist_ok=True)
    say = lambda m: print(" ", m, file=sys.stderr, flush=True)  # noqa: E731
    for seed in range(a.seed, a.seed + a.n):
        task, d, start, asm = generate(a.setup, seed, progress=say)
        print("wrote", write_folder(task, d, start, asm, a.out, broken_variants(a.setup, seed, d)))
    return 0


def _gen_one(args):
    family, level, seed, knobs, out = args
    mod = families.level(family, level)
    if hasattr(mod, "write_task"):  # a level that writes its own folder (two parts and an assembly file)
        return mod.write_task(seed, out)
    task, draw, ref = mod.generate(seed, knobs) if knobs else mod.generate(seed)
    if hasattr(mod, "write_folder"):  # a level with its own folder (two parts and how they go together)
        d = mod.write_folder(task, draw, ref, out)
        return f"wrote {d}  ({draw['summary']})"
    d = write_workspace(task, ref, out, {"level": level, "seed": seed, "draw": draw})
    parts = " <-> ".join(itf.description if itf.kind == "solid" else f"{itf.kind} {itf.bolt}"
                         for itf in task.parts.values())
    return f"wrote {d}  ({parts}, {draw['summary']})"


MAX_CHECKS_ENV = "JOINTBENCH_MAX_CHECKS"


def _check_limit(a) -> int | None:
    """The number of checks allowed on this task folder, if any: --max-checks,
    else the environment variable a harness sets for the agent's shell."""
    if a.max_checks is not None:
        return a.max_checks
    v = os.environ.get(MAX_CHECKS_ENV, "").strip()
    return int(v) if v else None


def cmd_verify(a) -> int:
    """Grade, within the check limit when one is in force. Over the limit the
    grader refuses before measuring anything, so a refused check gives no
    feedback at all."""
    limit = _check_limit(a)
    if limit is None:
        return _verify(a)
    from .archive import checks_used

    if a.no_log:
        print(f"refused: a check limit is in force ({limit}), and a check that is not logged cannot be counted")
        return 4
    lock = _lock(a.task)
    if lock is False:
        print("refused: another check of this task is running; wait for it to finish")
        return 4
    try:
        used = checks_used(a.task)
        if used >= limit:
            print(f"refused: all {limit} checks for this task are used ({used} graded attempts on record)")
            return 4
        code = _verify(a)
        if not a.json:
            print(f"  checks used: {checks_used(a.task)} of {limit}")
        return code
    finally:
        if lock:
            lock.close()


def _lock(task_dir: str):
    """Hold a lock on the task folder while one check runs, so two checks
    started at once cannot both slip under the count. The operating system
    releases it when the process ends, however it ends. None where the
    platform has no such lock (Windows): the count still holds for checks
    run one after the other."""
    try:
        import fcntl
    except ImportError:
        return None
    f = open(os.path.join(task_dir, ".verify.lock"), "w")
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        f.close()
        return False
    return f


def _verify(a) -> int:
    task = Task.load(os.path.join(a.task, "task.json"))
    if task.family == "moves" and task.level == 6:  # level 6: parts and assembly.json, a linkage
        return _verify_moves(a, task)
    if task.family == "assemble" or (task.meta or {}).get("assembly"):  # two parts and an assembly file, not one output.step
        return _verify_assembly(a, task)
    step = a.file or os.path.join(a.task, "output.step")
    if a.build:
        sub = os.path.join(a.task, "submission.py")
        r = subprocess.run([sys.executable, sub], cwd=a.task, capture_output=True, text=True)
        if r.returncode != 0:
            print(f"submission.py failed (exit {r.returncode}):\n{r.stderr[-2000:]}")
            return 2
    if a.skip_reported:
        report = False
    elif a.report == "all":
        report = "all"
    elif a.report:
        names = {"clean": "CLEAN", "makeable": "MANUFACTURING", "sturdy": "SECTION_MIN"}
        report = {names[g.strip()] for g in a.report.split(",") if g.strip()}
    else:
        report = True
    rep = grade(task, step, optional=report)
    shown = json.dumps(rep.to_json(), indent=2) if a.json else rep.text()
    n = None
    if not a.no_log:
        from .archive import read_note, reported_from_args, submission_script

        script = None if a.no_script else submission_script(a.task, a.script)
        link = "ran" if a.build else ("named" if a.script else "found")
        n = log_attempt(a.task, step, rep, who=a.who, script=script, script_link=link,
                        feedback=shown, feedback_format="json" if a.json else "text",
                        note=read_note(a.note_file), tool=a.tool, reported=reported_from_args(a))
    if a.json:
        out = rep.to_json()
        out["attempt"] = None if rep.outcome() == "error" else n
        if rep.outcome() == "error":
            out["error_record"] = n  # kept under errors/, not an attempt
        print(json.dumps(out, indent=2))
    else:
        print(shown)
        if n is not None:
            if rep.outcome() == "error":
                print(f"  not an attempt: kept as error #{n} in {os.path.join(a.task, 'errors', f'{n:04d}')}")
            else:
                print(f"  attempt #{n} archived in {os.path.join(a.task, 'attempts', f'{n:04d}')}")
    return {"pass": 0, "fail": 1, "error": 3}[rep.outcome()]


def cmd_env(a) -> int:
    from . import env

    rec = env.record(model=a.model, provider=a.provider, tool=a.tool, tool_versions=a.tool_version,
                     mcp=a.mcp, agent_loop=a.agent_loop, tasks=a.task, track=a.track,
                     probe_programs=not a.no_programs)
    p = env.write(a.run, rec)
    m = rec["measured"]
    print(f"wrote {p}: {len(m['python_packages'])} Python packages, "
          f"CAD programs: {', '.join(m['cad_programs'] or {}) or 'none found'}, "
          f"box: {'yes' if m['machine']['box'] else 'no'}")
    return 0


def _verify_assembly(a, task) -> int:
    """Level 4a, assembles: grade part1/part2 and assembly.json (``--file`` names another
    assembly file). Each grading is archived and counted, as a one-part answer's is."""
    from .assemble.grade import ASSEMBLY_FILE, grade as grade_assembly

    if a.build:
        r = subprocess.run([sys.executable, os.path.join(a.task, "submission.py")], cwd=a.task,
                           capture_output=True, text=True)
        if r.returncode != 0:
            print(f"submission.py failed (exit {r.returncode}):\n{r.stderr[-2000:]}")
            return 2
    if (task.meta or {}).get("lock_joint"):  # the two parts joined by a push-and-turn lock (the pipe's level 5)
        from .assemble.locked import grade as grade_assembly
    afile = a.file or os.path.join(a.task, ASSEMBLY_FILE)
    rep = grade_assembly(task, afile)
    shown = json.dumps(rep.to_json(), indent=2) if a.json else rep.text()
    print(shown)
    if not a.no_log:  # archived and counted like a one-part answer: part1.step as the record's file, the rest copied
        import shutil

        from .archive import read_note, reported_from_args, submission_script

        script = None if a.no_script else submission_script(a.task, a.script)
        n = log_attempt(a.task, os.path.join(a.task, "part1.step"), rep, who=a.who, script=script,
                        script_link="ran" if a.build else ("named" if a.script else "found"), feedback=shown,
                        feedback_format="json" if a.json else "text", note=read_note(a.note_file), tool=a.tool,
                        reported=reported_from_args(a))
        where = os.path.join(a.task, "errors" if rep.outcome() == "error" else "attempts", f"{n:04d}")
        for f in ("part2.step", os.path.basename(afile)):
            src = os.path.join(os.path.dirname(afile) if f != "part2.step" else a.task, f)
            if os.path.exists(src) and os.path.isdir(where):
                shutil.copyfile(src, os.path.join(where, f))
        if not a.json:
            print(f"  {'not an attempt: kept as error' if rep.outcome() == 'error' else 'attempt'} #{n} archived in {where}")
    return {"pass": 0, "fail": 1, "error": 3}[rep.outcome()]


def _verify_moves(a, task) -> int:
    """Level 6, moves: grade the parts and ``assembly.json`` (``--file`` names another assembly file). Each
    grading is counted and archived as a one-part answer's is: the record names the assembly file; every part
    file it lists is copied into the attempt's folder beside it."""
    import shutil

    from .moves.l6.grade import grade_folder

    if a.build:
        r = subprocess.run([sys.executable, os.path.join(a.task, "submission.py")], cwd=a.task,
                           capture_output=True, text=True)
        if r.returncode != 0:
            print(f"submission.py failed (exit {r.returncode}):\n{r.stderr[-2000:]}")
            return 2
    afile = a.file or os.path.join(a.task, "assembly.json")
    rep = grade_folder(a.task, afile)
    shown = json.dumps(rep.to_json(), indent=2) if a.json else rep.text()
    print(shown)
    if not a.no_log:
        from .archive import read_note, reported_from_args, submission_script

        script = None if a.no_script else submission_script(a.task, a.script)
        n = log_attempt(a.task, afile if os.path.exists(afile) else None, rep, who=a.who, script=script,
                        script_link="ran" if a.build else ("named" if a.script else "found"), feedback=shown,
                        feedback_format="json" if a.json else "text", note=read_note(a.note_file), tool=a.tool,
                        reported=reported_from_args(a))
        where = os.path.join(a.task, "errors" if rep.outcome() == "error" else "attempts", f"{n:04d}")
        try:
            with open(afile, encoding="utf-8") as f:
                listed = [p.get("file") for p in json.load(f).get("parts", []) if isinstance(p, dict)]
        except (OSError, ValueError, AttributeError):
            listed = []
        base = os.path.dirname(os.path.abspath(afile))
        for fn in listed:
            if not isinstance(fn, str):
                continue
            src = os.path.join(base, fn)
            dst = os.path.join(where, os.path.basename(fn))
            if os.path.isfile(src) and os.path.isdir(where) and not os.path.exists(dst):
                shutil.copyfile(src, dst)
        if not a.json:
            print(f"  {'not an attempt: kept as error' if rep.outcome() == 'error' else 'attempt'} #{n} archived in {where}")
    return {"pass": 0, "fail": 1, "error": 3}[rep.outcome()]


def cmd_baselines(a) -> int:
    from .baseline import table, table_text

    res = table(a.level, a.n, a.seed, progress=lambda lv, s: print(f"  level {lv} seed {s}", file=sys.stderr),
                jobs=a.jobs, family=a.family)
    print(table_text(res))
    return 0


def cmd_freeze(a) -> int:
    from . import freeze as fz

    if a.check:
        probs = fz.check(a.check)
        print("\n".join(probs) if probs else f"{a.check}: every task matches its manifest")
        return 1 if probs else 0
    counts = {int(x.split(":")[0]): int(x.split(":")[1]) for x in a.level}
    knobs: dict = {}
    for x in a.knob or []:
        lv, kv = x.split(":", 1)
        k, v = kv.split("=", 1)
        knobs.setdefault(int(lv), {})[k] = v
    secret = None
    if a.hidden:
        secret = os.environ.get("JOINTBENCH_HIDDEN_SECRET")
        if not secret:
            print("a hidden set needs its secret in the environment variable JOINTBENCH_HIDDEN_SECRET")
            return 2
    man = fz.freeze(a.out, a.name, a.version, counts, knobs, a.seed, secret, a.avoid,
                    progress=lambda tid: print(f"  froze {tid}", file=sys.stderr), jobs=a.jobs, family=a.family)
    print(f"{a.out}: set {man['set']} version {man['version']} ({man['visibility']}), {len(man['tasks'])} tasks")
    return 0


def cmd_traces(a) -> int:
    from .traces import write

    n, n_att = write(a.target, a.out)
    print(f"wrote {n} records ({n_att} attempts) to {a.out}")
    return 0


def cmd_selftest(a) -> int:
    from . import selftest

    t0 = time.time()
    seeds = range(a.seed, a.seed + a.n)
    rows = selftest.run(seeds, a.level, progress=lambda s: print(f"  graded seed {s}", file=sys.stderr),
                        jobs=a.jobs, family=a.family)
    print(f"selftest: level {a.level}, {a.n} tasks (seeds {a.seed}..{a.seed + a.n - 1}), "
          f"{sum(r.n for r in rows)} gradings, {time.time() - t0:.0f} s\n")
    print(selftest.table(rows))
    ok = all(r.ok for r in rows)
    print("\nALL AS EXPECTED" if ok else "\nSOME ROWS DID NOT BEHAVE -- see the task ids marked above")
    return 0 if ok else 1


def cmd_solve(a) -> int:
    from .baseline import solve

    passed = 0
    for d in a.task:
        rep, out = solve(d, a.baseline)
        log_attempt(d, out, rep, who=f"baseline-{a.baseline}", tool="build123d")
        passed += int(rep.passed)
        if a.quiet:
            fails = ", ".join(rep.failed()) or "-"
            print(f"{'PASS' if rep.passed else 'FAIL'}  {rep.task_id:<10} failed: {fails}")
        else:
            print(rep.text())
    if len(a.task) > 1:
        print(f"\n{a.baseline} baseline passed {passed}/{len(a.task)} tasks")
    return 0 if passed == len(a.task) else 1


def cmd_play(a) -> int:
    from .web.server import serve

    return serve(a.target, host=a.host, port=a.port, open_browser=not a.no_browser, scores_file=a.scores)


def cmd_site(a) -> int:
    from .web.export import export

    return export(a.target, a.out, include_reference=not a.no_reference, scores_file=a.scores,
                  dev_runs=a.dev_runs, precompute=[x for x in (a.precompute or "").split(",") if x])


def cmd_score(a) -> int:
    from .web.score import write

    return write(a.runs, a.out)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog=NAME, description="3D CAD benchmark: design the connecting part")
    sp = p.add_subparsers(dest="cmd", required=True)

    g = sp.add_parser("gen", help="generate task workspaces")
    g.add_argument("--seed", type=int, default=0)
    g.add_argument("--n", type=int, default=1)
    g.add_argument("--setup", default=None, help="a test of the ladder: facing, flange or pipe (with --level 1-6); "
                   "seed 0 is the showcase. See `ladder`")
    g.add_argument("--family", default="connect", help="task family (see docs/families.md)")
    g.add_argument("--level", type=int, default=1)
    g.add_argument("--out", default="tasks")
    g.add_argument("--knob", action="append", metavar="NAME=VALUE",
                   help="level 3: parts=2|3, obstacles=N, margin=MM, detour=MM, process=fdm|pbf, stiffness=FRACTION (repeatable)")
    g.add_argument("--jobs", "-j", type=int, default=1, help="processes in parallel (0 = all cores)")
    g.set_defaults(fn=cmd_gen)

    ld = sp.add_parser("ladder", help="which tests exist: the setups at each level, built or coming")
    ld.set_defaults(fn=lambda a: print(__import__("jointbench.ladder", fromlist=["table"]).table()) or 0)

    v = sp.add_parser("verify", help="grade <task>/output.step")
    v.add_argument("task")
    v.add_argument("--file", help="grade this STEP instead of <task>/output.step")
    v.add_argument("--build", action="store_true", help="run <task>/submission.py first")
    v.add_argument("--json", action="store_true")
    v.add_argument("--report", metavar="all|clean,makeable,sturdy",
                   help="also compute these reported-only requirement groups (default: only what the "
                        "material score needs)")
    v.add_argument("--skip-reported", action="store_true",
                   help="compute no reported-only check, not even for the score (fastest; same verdict)")
    v.add_argument("--no-log", action="store_true", help="do not append to attempts.jsonl")
    v.add_argument("--max-checks", type=int, metavar="N",
                   help="refuse to grade once N attempts of this task are on record (default: the "
                        f"{MAX_CHECKS_ENV} environment variable; no limit if unset)")
    v.add_argument("--by", "--who", dest="who", default="verify", help="who submitted (label in the log)")
    v.add_argument("--script", help="script that made the STEP (default: <task>/submission.py if edited)")
    v.add_argument("--no-script", action="store_true", help="archive no script with this attempt")
    v.add_argument("--note-file", help="free text stored with the attempt as given (e.g. the agent's reasoning)")
    v.add_argument("--tool", help="CAD tool used, as reported: build123d, cadquery, openscad, freecad, anvil, other")
    g2 = v.add_argument_group("reported figures (stored as given, never estimated)")
    g2.add_argument("--wall-seconds", type=float)
    g2.add_argument("--tokens-in", type=int)
    g2.add_argument("--tokens-out", type=int)
    g2.add_argument("--cost", type=float)
    g2.add_argument("--currency")
    g2.add_argument("--price-date", help="date of the list price the cost was computed at (YYYY-MM-DD)")
    g2.add_argument("--model")
    g2.add_argument("--model-version")
    g2.add_argument("--tool-calls", type=int)
    g2.add_argument("--reported-file", help="JSON file with any of the figures above")
    v.set_defaults(fn=cmd_verify)

    ev = sp.add_parser("env", help="write the run's environment record (<run>/environment.json)")
    ev.add_argument("run", help="the run folder")
    ev.add_argument("--model", help="exact model id, as sent to the provider")
    ev.add_argument("--provider", help="how the model was reached: anthropic, openai, openrouter, ...")
    ev.add_argument("--track", help="model, tool, open or human")
    ev.add_argument("--tool", help="the CAD tool the run uses")
    ev.add_argument("--tool-version", action="append", metavar="NAME=VERSION",
                    help="a program or add-on the machine cannot read itself (repeatable)")
    ev.add_argument("--mcp", action="append", metavar="NAME=VERSION", help="a connector or MCP server (repeatable)")
    ev.add_argument("--agent-loop", action="append", metavar="NAME=VERSION", help="the agent loop / harness")
    ev.add_argument("--task", action="append", metavar="DIR", help="a task folder of the run (repeatable)")
    ev.add_argument("--no-programs", action="store_true", help="do not start CAD programs to read their versions")
    ev.set_defaults(fn=cmd_env)

    bl = sp.add_parser("baselines", help="table: every baseline and the reference across levels")
    bl.add_argument("--family", default="connect")
    bl.add_argument("--level", type=int, nargs="+", default=[1, 2, 3])
    bl.add_argument("--n", type=int, default=10)
    bl.add_argument("--seed", type=int, default=0)
    bl.add_argument("--jobs", "-j", type=int, default=1, help="processes in parallel (0 = all cores)")
    bl.set_defaults(fn=cmd_baselines)

    fr = sp.add_parser("freeze", help="write a frozen task set with a manifest (or --check one)")
    fr.add_argument("out", nargs="?", help="new set folder")
    fr.add_argument("--check", metavar="SET", help="check a set's files against its manifest")
    fr.add_argument("--family", default="connect")
    fr.add_argument("--name", default="trial")
    fr.add_argument("--version", type=int, default=1)
    fr.add_argument("--level", action="append", default=[], metavar="LEVEL:COUNT")
    fr.add_argument("--knob", action="append", metavar="LEVEL:NAME=VALUE")
    fr.add_argument("--seed", type=int, default=0, help="first seed of a public set")
    fr.add_argument("--hidden", action="store_true", help="seeds from JOINTBENCH_HIDDEN_SECRET; keep outside git")
    fr.add_argument("--avoid", action="append", metavar="SET", help="refuse seeds used by this set")
    fr.add_argument("--jobs", "-j", type=int, default=1, help="processes in parallel (0 = all cores)")
    fr.set_defaults(fn=cmd_freeze)

    tr = sp.add_parser("traces", help="export attempt sequences as training records (JSON lines)")
    tr.add_argument("target", help="a task folder, a folder of tasks, or a runs folder")
    tr.add_argument("--out", required=True)
    tr.set_defaults(fn=cmd_traces)

    s = sp.add_parser("selftest", help="prove the grader on references and broken copies")
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--n", type=int, default=10)
    s.add_argument("--family", default="connect")
    s.add_argument("--level", type=int, default=1)
    s.add_argument("--jobs", "-j", type=int, default=1, help="processes in parallel (0 = all cores)")
    s.set_defaults(fn=cmd_selftest)

    b = sp.add_parser("solve", help="run a scripted baseline on one or more tasks")
    b.add_argument("task", nargs="+")
    b.add_argument("--baseline", default="naive",
                   help="naive: one drilled block; template: the level-1 answer applied blindly; "
                        "hull / rod: the level-2 answer (plates joined straight) applied blindly")
    b.add_argument("--quiet", "-q", action="store_true", help="one line per task")
    b.set_defaults(fn=cmd_solve)

    pl = sp.add_parser("play", help="open the viewer on a task or folder of tasks; grade STEP uploads")
    pl.add_argument("target", nargs="?", default="tasks")
    pl.add_argument("--port", type=int, default=8765)
    pl.add_argument("--host", default="127.0.0.1")
    pl.add_argument("--no-browser", action="store_true")
    pl.add_argument("--scores", help="results file from `score` to show on the scores page")
    pl.set_defaults(fn=cmd_play)

    st = sp.add_parser("site", help="export the viewer as a static website")
    st.add_argument("target", nargs="+", help="task folders or folders of tasks")
    st.add_argument("--out", required=True)
    st.add_argument("--no-reference", action="store_true", help="leave the reference answers out")
    st.add_argument("--scores", help="results file from `score` to show on the scores page")
    st.add_argument("--dev-runs", action="store_true",
                    help="also show runs graded under earlier rules on the scoreboard, marked as development data")
    st.add_argument("--precompute", metavar="PROCESSES",
                    help="measure these processes (e.g. fdm,pbf,moulding) for every model run's submission, "
                         "so the compare page can switch process offline; slow (seconds to minutes per part)")
    st.set_defaults(fn=cmd_site)

    sc = sp.add_parser("score", help="score model runs: <runs>/<model>/<task>/attempts.jsonl -> one results file")
    sc.add_argument("runs")
    sc.add_argument("--out", required=True)
    sc.set_defaults(fn=cmd_score)

    a = p.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
