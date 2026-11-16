"""python -m jointbench.tools  check | prepare <task>... --tool T | build <task> --tool T"""
from __future__ import annotations

import argparse
import sys

from . import NOT_PROVIDED, TOOLS, BuildFailed, ToolMissing, build, prepare, summary


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m jointbench.tools",
                                description="Starters and builds for answering a task in a given CAD tool.")
    sp = p.add_subparsers(dest="cmd", required=True)
    sp.add_parser("check", help="list the tools and whether each works on this machine")
    pr = sp.add_parser("prepare", help="write the tool's starter file and TOOL.md into task folders")
    pr.add_argument("task", nargs="+")
    pr.add_argument("--tool", required=True, choices=sorted(TOOLS))
    pr.add_argument("--runner", default="uv run",
                    help="command prefix written into the note, e.g. 'uv run --project /path/to/jointbench'")
    pr.add_argument("--force", action="store_true", help="overwrite an existing starter")
    b = sp.add_parser("build", help="build the tool's starter into <task>/output.step")
    b.add_argument("task")
    b.add_argument("--tool", required=True, choices=sorted(TOOLS))
    a = p.parse_args(argv)

    if a.cmd == "check":
        for name, t in TOOLS.items():
            why = t.available()
            print(f"{name:11s} {'ok' if why is None else 'MISSING'}  {why or ''}".rstrip())
        for name, why in NOT_PROVIDED.items():
            print(f"{name:11s} --  {why}")
        return 0
    if a.cmd == "prepare":
        for task in a.task:
            for f in prepare(task, a.tool, runner=a.runner, force=a.force):
                print("wrote", f)
        return 0
    try:
        out = build(a.task, a.tool)
    except (ToolMissing, BuildFailed, FileNotFoundError) as e:
        print(f"BUILD FAILED: {e}", file=sys.stderr)
        return 1
    print("wrote", out)
    print(summary(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
