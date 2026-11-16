"""Proof the grader can be trusted.

For each generated task we grade the reference answer, a differently shaped
body with the same interfaces, and a set of deliberately broken copies of
the reference. Every copy goes through a real STEP file, so the whole path
an agent's answer takes is exercised. A row is "as expected" when a good
part passes, or a broken part fails on the named check it was broken for.

An expectation is one of:
  None                 the part passes
  "CHECK"              CHECK fails (required, or reported-only for this task)
  ("ok", "CHECK")      the part passes and CHECK, even if only reported, holds
Manufacturing rules are measured only on rows that name one, because the
checker takes several seconds per part.
"""
from __future__ import annotations

import os
import tempfile
from dataclasses import dataclass, field

from .geometry import write_step
from .grade import grade
from . import families


@dataclass
class Row:
    label: str
    expect: str | None
    n: int = 0
    as_expected: int = 0
    also_failed: set = field(default_factory=set)
    misses: list = field(default_factory=list)  # task ids that did not behave

    @property
    def applies(self) -> bool:
        return self.n > 0  # a stiffness row on tasks that do not require stiffness: n = 0

    @property
    def ok(self) -> bool:
        return self.as_expected == self.n


def _one(args):
    """All variants of one seed: [(task id, good, failed checks)] per variant."""
    family, level, seed = args
    mod = families.level(family, level)
    out = []
    with tempfile.TemporaryDirectory() as tmp:
        task, draw, _ = mod.generate(seed)
        for label, kw, expect in mod.SELFTEST_VARIANTS:
            if expect == "STIFFNESS" and not task.stiffness:
                out.append(None)  # the task does not require stiffness: the row does not apply
                continue
            part = mod.build_connector(task, draw, **kw(task))
            path = os.path.join(tmp, "variant.step")
            write_step(part, path)
            want = expect[1] if isinstance(expect, tuple) else expect
            mfg = isinstance(want, str) and (want.startswith(("FDM_", "IM_", "PBF_")) or want.startswith("*"))
            fit_row = isinstance(want, str) and want in ("HOLES", "SEATING", "NO_INTERFERENCE", "CLEAR_PATH",
                                                         "TOOL_ACCESS") and not isinstance(expect, tuple)
            # run only what the row is about; a row expecting a fit failure
            # stops before the slow checks once the verdict is settled
            need = {"MANUFACTURING"} if mfg else ({want} if want in ("SECTION_MIN",) else (False if fit_row else True))
            # the stiffness analysis takes minutes: it runs on the rows about
            # it (the reference, the lean answer, the slender shapes); the
            # others are about fit, section or makeability
            stiff_row = want == "STIFFNESS" or label == "reference answer" or label.startswith("lean answer")
            skip = frozenset() if stiff_row or not task.stiffness else frozenset({"STIFFNESS"})
            rep = grade(task, path, manufacturing=mfg or not fit_row, optional=need, score=False, skip=skip)
            failed = rep.failed() + rep.warnings()
            if isinstance(want, str) and want.startswith("*"):  # e.g. "*_WALL_MIN": the task's process's rule
                want = next((f for f in failed if f.endswith(want[1:])), want)
            if expect is None:
                good = rep.passed
            elif isinstance(expect, tuple):
                c = next((c for c in rep.checks if c.name == want), None)
                good = rep.passed and c is not None and c.ok is True
            else:
                good = want in failed
            out.append((task.id, good, [f for f in failed if f != want]))
    return out


def run(seeds, level: int = 1, progress=None, jobs: int = 1, family: str = "connect") -> list[Row]:
    from .jobs import pmap

    mod = families.level(family, level)
    rows = [Row(label, expect) for label, _, expect in mod.SELFTEST_VARIANTS]
    seeds = list(seeds)
    for seed, res in zip(seeds, pmap(_one, [(family, level, s) for s in seeds], jobs)):
        for row, r in zip(rows, res):
            if r is None:
                continue
            tid, good, also = r
            row.n += 1
            row.as_expected += int(good)
            row.also_failed.update(also)
            if not good:
                row.misses.append(tid)
        if progress:
            progress(seed)
    return rows


def table(rows: list[Row]) -> str:
    w = max(len(r.label) for r in rows)
    out = [f"{'variant':<{w}}  {'must':<24} {'as expected':>11}  also failed",
           f"{'-' * w}  {'-' * 24} {'-' * 11}  {'-' * 11}"]
    for r in rows:
        if r.expect is None:
            must = "pass"
        elif isinstance(r.expect, tuple):
            must = f"pass, {r.expect[1]} ok"
        else:
            must = f"fail {r.expect}"
        also = ", ".join(sorted(r.also_failed)) or "-"
        mark = "" if r.ok else "   <-- " + ", ".join(r.misses)
        got = f"{r.as_expected:>5}/{r.n:<5}" if r.applies else f"{'n/a':>8}   "
        out.append(f"{r.label:<{w}}  {must:<24} {got}  {also}{mark}")
    return "\n".join(out)
