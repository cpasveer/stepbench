"""python -m jointbench.stiffness <task-folder> [--file answer.step] [--json]
[--fields DIR] [--images DIR] [--selftest]

Exit code: 0 when a figure was computed (and, if there is a requirement,
it passed or is marginal), 1 when a requirement failed, 2 when no figure
could be computed (the reason is in the report)."""
from __future__ import annotations

import argparse
import json
import sys

from . import check, fe, write_fields, write_images


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m jointbench.stiffness",
                                 description="stiffness between the mounts, per unit material, against the reference")
    ap.add_argument("task", nargs="?", help="task folder (holds task.json)")
    ap.add_argument("--file", help="the answer to score (default: <task>/output.step)")
    ap.add_argument("--reference", help="the par answer (default: <task>/_solution/reference.step)")
    ap.add_argument("--no-par", action="store_true", help="do not analyse the reference")
    ap.add_argument("--element-size", type=float, help="override the element size (mm), for studies only")
    ap.add_argument("--quick", action="store_true",
                    help="coarser elements (3 mm): about 3 x faster, figure within +/- 4 %%")
    ap.add_argument("--require", type=float, metavar="N_PER_MM",
                    help="requirement mode: the weakest-way figure must reach this")
    ap.add_argument("--fraction", type=float,
                    help="requirement mode: this fraction of the reference's figure (default from settings.toml)")
    ap.add_argument("--screen", action="store_true",
                    help="requirement mode, cheapest first: coarse 4 mm analysis, full only near the requirement")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--fields", metavar="DIR", help="write the displacement field file here")
    ap.add_argument("--images", metavar="DIR", help="write a deformed-shape picture per load case here")
    ap.add_argument("--selftest", action="store_true", help="run the known-answer self-test")
    ap.add_argument("--convergence", action="store_true", help="with --selftest: also run the element-size study")
    a = ap.parse_args(argv)

    need = fe.available()
    if need:
        print(need, file=sys.stderr)
        return 3
    if a.selftest:
        from . import selftest

        rows, _ = selftest.run(progress=lambda s: print(f"  ... {s}", file=sys.stderr))
        print(selftest.table(rows))
        if a.images:
            for p in selftest.pictures(a.images):
                print(p)
        if a.convergence:
            part, mounts = selftest.cantilever()
            import os
            import tempfile

            from ..geometry import write_step

            with tempfile.TemporaryDirectory() as tmp:
                path = os.path.join(tmp, "cantilever.step")
                write_step(part, path)
                res = selftest.convergence([("cantilever", path, mounts)],
                                           progress=lambda s: print(f"  ... {s}", file=sys.stderr))
            print(selftest.convergence_table(res))
        return 0 if all(r.ok for r in rows) else 1
    if not a.task:
        ap.error("give a task folder, or --selftest")
    if a.screen:
        from . import screened

        rep = screened(a.task, a.file, require=a.require, fraction=a.fraction, reference=a.reference)
        print(json.dumps(rep.to_json(), indent=1) if a.json else rep.text())
        if not rep.ok:
            return 2
        return 1 if not rep.requirement.passed else 0
    rep = check(a.task, a.file, a.reference, par=not a.no_par, h="quick" if a.quick else a.element_size,
                require=a.require, fraction=a.fraction)
    if a.fields and rep.ok:
        print(f"fields: {write_fields(rep, a.fields)}", file=sys.stderr)
    if a.images and rep.ok:
        for p in write_images(rep, a.images):
            print(f"image: {p}", file=sys.stderr)
    print(json.dumps(rep.to_json(), indent=1) if a.json else rep.text())
    if not rep.ok:
        return 2
    return 1 if rep.requirement and not rep.requirement.passed else 0


if __name__ == "__main__":
    sys.exit(main())
