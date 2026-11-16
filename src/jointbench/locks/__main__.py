"""python -m jointbench.locks gen --seed 0 --out DIR      make a "locks" task folder (reference graded first)
python -m jointbench.locks verify DIR [--answer D]       grade DIR/part_A.step and part_B.step (or those in D)
python -m jointbench.locks check DIR                     grade the reference and every broken copy in DIR/_solution
python -m jointbench.locks view DIR OUT [--answer D]     a browser view: B pushed in, turned, pulled"""
from __future__ import annotations

import argparse
import json
import os
import sys


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m jointbench.locks")
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("gen")
    g.add_argument("--seed", type=int, default=0)
    g.add_argument("--out", required=True)
    v = sub.add_parser("verify")
    v.add_argument("folder")
    v.add_argument("--answer", default=None)
    v.add_argument("--json", action="store_true")
    v.add_argument("--no-manufacturing", action="store_true")
    c = sub.add_parser("check")
    c.add_argument("folder")
    c.add_argument("--no-manufacturing", action="store_true")
    w = sub.add_parser("view")
    w.add_argument("folder")
    w.add_argument("out")
    w.add_argument("--answer", default=None)
    a = ap.parse_args(argv)
    say = lambda s: print(" ", s, file=sys.stderr, flush=True)  # noqa: E731
    if a.cmd == "gen":
        from .generate import generate, write_folder

        task, d, parts = generate(a.seed, progress=say)
        print("wrote", write_folder(task, d, parts, a.out))
        return 0
    if a.cmd == "view":
        from .view import export

        print("wrote", export(a.folder, a.answer or os.path.join(a.folder, "_solution"), a.out))
        return 0
    from .grade import grade_folder

    mf = not a.no_manufacturing
    if a.cmd == "verify":
        rep = grade_folder(a.folder, a.answer, manufacturing=mf, progress=say)
        print(json.dumps(rep.to_json(), indent=2) if a.json else rep.text())
        return {"pass": 0, "fail": 1, "error": 3}[rep.outcome()]
    sol = os.path.join(a.folder, "_solution")
    rep = grade_folder(a.folder, sol, manufacturing=mf, progress=say)
    print("== reference\n" + rep.text(), flush=True)
    ok = rep.passed
    bro = os.path.join(sol, "broken")
    for name in sorted(os.listdir(bro)) if os.path.isdir(bro) else []:
        must = open(os.path.join(bro, name, "MUST_FAIL")).read().strip()
        why = open(os.path.join(bro, name, "WHY")).read().strip()
        kg_file = os.path.join(bro, name, "LOAD_KG")
        if os.path.exists(kg_file):
            from ..task import Task
            from .generate import with_load
            from .grade import grade

            task = with_load(Task.load(os.path.join(a.folder, "task.json")), float(open(kg_file).read()))
            r = grade(task, os.path.join(bro, name), manufacturing=mf, progress=say)
        else:
            r = grade_folder(a.folder, os.path.join(bro, name), manufacturing=mf, progress=say)
        hit = (not r.passed) and must in r.failed()
        ok &= hit
        print(f"\n== broken: {name} -- {why} (must fail {must}): {'as expected' if hit else 'NOT AS EXPECTED'}\n"
              + r.text(), flush=True)
    print("\nALL AS EXPECTED" if ok else "\nSOME DID NOT BEHAVE")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
