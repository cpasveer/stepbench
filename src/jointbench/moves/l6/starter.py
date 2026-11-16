"""The starting point written into every level 6 task folder as ``submission.py``.

A placeholder answer in the right shape -- one part and an ``assembly.json`` -- that fails. The archive
recognises it unchanged (``archive.submission_script``) and does not record it as the model's code."""
from __future__ import annotations

SUBMISSION = '''"""Your answer to this task.

Write your parts as STEP files (one solid each, in world coordinates, assembled with
B at home) and assembly.json, next to this file. Read README.md for the brief; every
number you need is in task.json.

This placeholder writes one block and an assembly.json in the right shape. It will
fail. Replace it entirely, or model in any CAD program and write the files yourself.

    python submission.py                 # writes p1.step and assembly.json
    CLI_NAME verify . --by <your name> --tool <the CAD tool you used>

(the brief, README.md, says how to run them.)
"""
import json
import pathlib

from build123d import Box, Pos, export_step

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
MOTION = TASK["meta"]["motion"]  # fixed and moving fixture, the stated poses, the latch, the box


def main() -> None:
    p1 = Pos(0, 0, 0) * Box(10, 10, 10)  # placeholder
    export_step(p1, str(HERE / "p1.step"))
    asm = {
        "parts": [{"name": "p1", "file": "p1.step", "bolts_to": MOTION["fixed"], "insert_direction": [0, 0, -1]}],
        "pivots": [],
        "drive": {"pivot": "J1", "range_deg": 90},
        "fasteners": [],
    }
    (HERE / "assembly.json").write_text(json.dumps(asm, indent=2) + "\\n")


if __name__ == "__main__":
    main()
'''


def submission() -> str:
    from ... import NAME

    return SUBMISSION.replace("CLI_NAME", NAME)
