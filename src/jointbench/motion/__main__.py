"""python -m jointbench.motion gen | verify | selftest | stepsize | show | agree"""
from __future__ import annotations

import argparse
import json
import sys


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m jointbench.motion")
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("gen", help="generate mechanism tasks")
    g.add_argument("--type", default="fourbar", help="fourbar | slider | yoke | oldham | cam | geneva | ratchet")
    g.add_argument("--seed", type=int, default=0)
    g.add_argument("--n", type=int, default=1)
    g.add_argument("--out", default="tasks")
    g.add_argument("--knob", action="append", default=[], help="name=value, e.g. remove=rocker, pivots=withheld, "
                   "obstacles=2, output_on_removed=1, placed=flat")
    v = sub.add_parser("verify", help="grade <task>/output.step")
    v.add_argument("task")
    v.add_argument("--step", default=None, help="path of the STEP to grade")
    v.add_argument("--step-deg", type=float, default=None)
    v.add_argument("--json", action="store_true")
    s = sub.add_parser("selftest", help="reference passes, broken variants fail")
    s.add_argument("--type", action="append", default=None)
    s.add_argument("--n", type=int, default=3)
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("-j", type=int, default=1, help="processes (0 = all cores)")
    s.add_argument("--json", default=None, help="write the rows here")
    z = sub.add_parser("stepsize", help="how small the grading step must be")
    z.add_argument("--n", type=int, default=3)
    z.add_argument("-j", type=int, default=1)
    z.add_argument("--json", default=None)
    a = sub.add_parser("agree", help="closed form vs numeric solve on generated mechanisms")
    a.add_argument("--n", type=int, default=20)
    a.add_argument("--type", action="append", default=None)
    w = sub.add_parser("show", help="draw a picture of a generated mechanism")
    w.add_argument("--type", default="fourbar")
    w.add_argument("--seed", type=int, default=0)
    w.add_argument("--out", default="mechanism.png")
    w.add_argument("--knob", action="append", default=[])
    args = ap.parse_args(argv)

    def knobs(lst):
        out = {}
        for kv in lst:
            k, val = kv.split("=", 1)
            try:
                val = json.loads(val)
            except json.JSONDecodeError:
                pass
            out[k] = val
        return out

    if args.cmd == "gen":
        from . import api as fam
        for i in range(args.n):
            path = fam.generate_folder(args.type, args.seed + i, knobs(args.knob), args.out)
            print(path)
        return 0
    if args.cmd == "verify":
        from .api import verify_folder
        rep = verify_folder(args.task, args.step, args.step_deg)
        print(json.dumps(rep.to_json(), indent=1) if args.json else rep.text(args.task))
        return 0 if rep.ok else 1
    if args.cmd == "selftest":
        from .selftest import run
        return run(args.type, args.n, args.seed, args.j, args.json)
    if args.cmd == "stepsize":
        from .stepsize import run
        return run(args.n, args.j, args.json)
    if args.cmd == "agree":
        from .selftest import agree
        return agree(args.type, args.n)
    if args.cmd == "show":
        from .api import show
        show(args.type, args.seed, knobs(args.knob), args.out)
        print(args.out)
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(main())
