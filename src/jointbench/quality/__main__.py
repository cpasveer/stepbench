"""python -m jointbench.quality <file.step> [--json] [--fields DIR] [--images DIR]
python -m jointbench.quality --selftest [--images DIR]"""
from __future__ import annotations

import argparse
import json
import sys

from . import check, write_fields, write_images


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m jointbench.quality",
                                description="modelling-quality check of one STEP solid")
    p.add_argument("step", nargs="?")
    p.add_argument("--rules", help="only these rules, comma-separated ids (default: all)")
    p.add_argument("--json", action="store_true")
    p.add_argument("--fields", metavar="DIR", help="write the mesh with every rule's field as JSON here")
    p.add_argument("--images", metavar="DIR", help="write one picture per rule here")
    p.add_argument("--selftest", action="store_true", help="prove the checker on generated parts")
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
        return 0 if ok else 1
    if not a.step:
        p.error("a STEP file is needed (or --selftest)")
    try:
        rep = check(a.step, a.rules.split(",") if a.rules else None)
    except ValueError as e:
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
