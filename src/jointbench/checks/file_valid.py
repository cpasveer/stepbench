"""FILE_VALID: the STEP loads and holds exactly one valid, closed solid."""
from __future__ import annotations

import os

from OCP.BRep import BRep_Tool
from OCP.BRepCheck import BRepCheck_Analyzer

from ..geometry import read_step
from .base import CheckResult, Context

NAME = "FILE_VALID"


def run(ctx: Context) -> CheckResult:
    if not os.path.exists(ctx.step_path):
        return CheckResult(NAME, False, f"no file at {os.path.basename(ctx.step_path)}")
    try:
        shape = read_step(ctx.step_path)
    except Exception as e:  # noqa: BLE001 - any loader failure is a fail
        return CheckResult(NAME, False, f"STEP did not load: {e}")
    solids = shape.solids()
    if len(solids) != 1:
        return CheckResult(NAME, False, f"expected exactly 1 solid, found {len(solids)}",
                           len(solids), "solids")
    s = solids[0]
    if not BRepCheck_Analyzer(s.wrapped).IsValid():
        return CheckResult(NAME, False, "the solid fails the OpenCASCADE validity check", 1, "solids")
    open_shells = [sh for sh in s.shells() if not BRep_Tool.IsClosed_s(sh.wrapped)]
    if open_shells:
        return CheckResult(NAME, False, f"{len(open_shells)} shell(s) not closed (the skin has gaps)",
                           1, "solids")
    v = float(s.volume)
    if v <= 0:
        return CheckResult(NAME, False, f"solid has non-positive volume {v:.3f} mm3", 1, "solids")
    ctx.shape, ctx.solid = shape, s
    return CheckResult(NAME, True, f"1 closed valid solid, {len(s.faces())} faces", 1, "solids")
