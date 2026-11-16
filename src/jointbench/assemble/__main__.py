"""python -m jointbench.assemble gen --seed 0 --out DIR      make a level 4a (assembles) task folder
python -m jointbench.assemble check DIR                       grade its reference and its broken copies
python -m jointbench.assemble view DIR OUT [--answer A]       STL files and a page: box, parts, each part's way in"""
from __future__ import annotations

import argparse
import os
import sys


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m jointbench.assemble")
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("gen")
    g.add_argument("--seed", type=int, default=0)
    g.add_argument("--out", required=True)
    c = sub.add_parser("check")
    c.add_argument("folder")
    v = sub.add_parser("view")
    v.add_argument("folder")
    v.add_argument("out")
    v.add_argument("--answer", default=None, help="folder with part1/part2/assembly.json (default: the hidden reference)")
    a = ap.parse_args(argv)
    if a.cmd == "view":
        from ..task import Task
        from .view import write

        task = Task.load(os.path.join(a.folder, "task.json"))
        ans = a.answer or os.path.join(a.folder, "_solution")
        print("wrote", write(task, ans, a.out, f"{task.id}, {'the hidden reference' if not a.answer else a.answer}"))
        return 0
    if a.cmd == "gen":
        from .generate import generate, write_folder

        task, d, ref = generate(a.seed, progress=lambda s: print(" ", s, file=sys.stderr))
        print("wrote", write_folder(task, d, ref, a.out), "--", d["summary"])
        return 0
    from ..task import Task
    from .generate import with_load
    from .grade import grade, grade_folder

    sol = os.path.join(a.folder, "_solution")
    ok = True
    rep = grade_folder(a.folder, os.path.join(sol, "assembly.json"))
    print("== reference\n" + rep.text())
    ok &= rep.passed
    bro = os.path.join(sol, "broken")
    for name in sorted(os.listdir(bro)) if os.path.isdir(bro) else []:
        must = open(os.path.join(bro, name, "MUST_FAIL")).read().strip()
        kg_file = os.path.join(bro, name, "LOAD_KG")
        task = Task.load(os.path.join(a.folder, "task.json"))
        note = ""
        if os.path.exists(kg_file):
            kg = float(open(kg_file).read())
            task, note = with_load(task, kg), f", graded under {kg:g} kg on B instead of the task's own load"
        r = grade(task, os.path.join(bro, name, "assembly.json"), stiffness=False)
        hit = must in r.failed()
        ok &= (not r.passed) and hit
        print(f"\n== broken: {name} (must fail {must}{note}): {'as expected' if hit and not r.passed else 'NOT AS EXPECTED'}\n"
              + r.text())
    print("\nALL AS EXPECTED" if ok else "\nSOME DID NOT BEHAVE")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
