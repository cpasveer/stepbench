"""python -m jointbench.dfm <file.step> --process fdm [--direction x,y,z] [--json]
                         [--fields DIR] [--images DIR]
python -m jointbench.dfm --selftest [--level1 TASK_DIR ...]"""
from __future__ import annotations

import argparse
import json
import sys

from . import available, check, write_fields, write_images


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m jointbench.dfm",
                                description="manufacturing rule check of one STEP solid")
    p.add_argument("step", nargs="?")
    p.add_argument("--process", choices=available(), default="fdm")
    p.add_argument("--direction", help="declared build or pull direction as x,y,z (default: chosen by the checker)")
    p.add_argument("--declared", metavar="BUILD_JSON", help="read the declared process and direction from this file")
    p.add_argument("--rules", help="only these rules, comma-separated ids (default: all of the process's)")
    p.add_argument("--probe", metavar="X,Y,Z", help="measure wall, gap and section at the surface point nearest this")
    p.add_argument("--json", action="store_true")
    p.add_argument("--fields", metavar="DIR", help="write the mesh with every rule's field as JSON here")
    p.add_argument("--images", metavar="DIR", help="write one picture per rule here")
    p.add_argument("--selftest", action="store_true", help="prove the checker on generated parts")
    p.add_argument("--level1", nargs="*", default=[], metavar="TASK_DIR",
                   help="with --selftest: also check <dir>/_solution/reference.step of these tasks")
    a = p.parse_args(argv)
    if a.selftest:
        from . import selftest

        rows = selftest.run()
        print(selftest.table(rows))
        ok = all(x.ok for x in rows)
        print("\nALL AS EXPECTED" if ok else "\nSOME ROWS DID NOT BEHAVE -- see the rows marked above")
        if a.images:
            paths = selftest.pictures(a.images)
            print(f"\n{len(paths)} pictures, one per rule, written to {a.images}")
        if a.level1:
            print()
            print(selftest.level1_table(selftest.level1_refs(a.level1)))
        return 0 if ok else 1
    if not a.step:
        p.error("a STEP file is needed (or --selftest)")
    if a.probe:
        from . import probe

        print(json.dumps(probe(a.step, [float(v) for v in a.probe.split(",")]), indent=2))
        return 0
    d = tuple(float(x) for x in a.direction.split(",")) if a.direction else None
    if a.declared:
        from . import declaration

        proc, dd = declaration(a.declared)
        if proc:
            a.process = proc
        if dd is not None and d is None:
            d = tuple(dd)
    try:
        rep = check(a.step, a.process, d, a.rules.split(",") if a.rules else None)
    except (ValueError, NotImplementedError) as e:
        print(f"cannot check: {e}")
        return 2
    print(json.dumps(rep.to_json(), indent=2) if a.json else rep.text())
    if a.fields:
        path = write_fields(rep, a.fields)
        if not a.json:
            print(f"  fields written to {path}")
    if a.images:
        paths = write_images(rep, a.images)
        if not a.json:
            print(f"  {len(paths)} pictures written to {a.images}")
    return 0 if rep.passed else 1


if __name__ == "__main__":
    sys.exit(main())
