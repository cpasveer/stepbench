"""python -m jointbench.moves.l6 gen --setup pipe --seed 0 --out DIR   a level 6 task folder (reference graded first)
python -m jointbench.moves.l6 verify DIR [--file A.json]               grade DIR/assembly.json
python -m jointbench.moves.l6 check DIR                                the reference and every broken copy
python -m jointbench.moves.l6 view DIR OUT [--file A.json]             the browser view (default: the reference)"""
from __future__ import annotations

import argparse
import json
import os
import sys


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m jointbench.moves.l6")
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("gen")
    g.add_argument("--setup", default="pipe")
    g.add_argument("--seed", type=int, default=0)
    g.add_argument("--out", required=True)
    g.add_argument("--no-check", action="store_true", help="do not grade the reference (quick look only)")
    v = sub.add_parser("verify")
    v.add_argument("folder")
    v.add_argument("--file", default=None)
    v.add_argument("--json", action="store_true")
    v.add_argument("--no-manufacturing", action="store_true")
    c = sub.add_parser("check")
    c.add_argument("folder")
    c.add_argument("--no-manufacturing", action="store_true")
    w = sub.add_parser("view")
    w.add_argument("folder")
    w.add_argument("out")
    w.add_argument("--file", default=None)
    a = ap.parse_args(argv)
    say = lambda s: print(" ", s, file=sys.stderr, flush=True)  # noqa: E731
    if a.cmd == "gen":
        from .generate import broken_variants, generate, write_folder

        task, d, start, asm = generate(a.setup, a.seed, check=not a.no_check, progress=say)
        print("wrote", write_folder(task, d, start, asm, a.out, broken_variants(a.setup, a.seed, d)))
        return 0
    if a.cmd == "view":
        from .view import export

        f = a.file or os.path.join(a.folder, "_solution", "assembly.json")
        print("wrote", export(a.folder, f, a.out))
        return 0
    from .grade import grade_folder

    mf = not a.no_manufacturing
    if a.cmd == "verify":
        rep = grade_folder(a.folder, a.file, manufacturing=mf, progress=say)
        print(json.dumps(rep.to_json(), indent=2) if a.json else rep.text())
        return {"pass": 0, "fail": 1, "error": 3}[rep.outcome()]
    sol = os.path.join(a.folder, "_solution")
    rep = grade_folder(a.folder, os.path.join(sol, "assembly.json"), manufacturing=mf, progress=say)
    print("== reference\n" + rep.text(), flush=True)
    ok = rep.passed
    bro = os.path.join(sol, "broken")
    for name in sorted(os.listdir(bro)) if os.path.isdir(bro) else []:
        must = open(os.path.join(bro, name, "MUST_FAIL")).read().strip()
        why = open(os.path.join(bro, name, "WHY")).read().strip()
        r = grade_folder(a.folder, os.path.join(bro, name, "assembly.json"), manufacturing=mf, progress=say)
        hit = (not r.passed) and must in r.failed()
        ok &= hit
        print(f"\n== broken: {name} -- {why} (must fail {must}): {'as expected' if hit else 'NOT AS EXPECTED'}\n"
              + r.text(), flush=True)
    print("\nALL AS EXPECTED" if ok else "\nSOME DID NOT BEHAVE")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
