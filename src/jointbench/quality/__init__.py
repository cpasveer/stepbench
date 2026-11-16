"""Modelling quality: is this solid modelled cleanly?

    from jointbench.quality import check
    rep = check("part.step"); print(rep.text())

    python -m jointbench.quality part.step [--json] [--fields DIR] [--images DIR]
    python -m jointbench.quality --selftest [--images DIR]

A valid solid that fits can still be untidy: a plate joined to a body a
hair out of line leaves a 0.1 mm step, a sliver face tapering to a knife-
thin tip, a 1 degree crease. The rules here (limits in ``quality.toml``)
find those signs, each with the measured number, its limit and where on the
part it is. Every rule is measured exactly on the solid's own faces, edges
and corners; nothing is sampled. Same file in, same report out.

The report also gives, for information only, the face and edge count and
the share of faces (and of area) that are planes, cylinders or other.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

from ..dfm import Report as _Report
from ..dfm.geom import Part, load
from ..dfm.rules import load_file
from .measure import run

RULES = os.path.join(os.path.dirname(__file__), "quality.toml")


@dataclass
class Report(_Report):
    info: dict = field(default_factory=dict)

    def text(self) -> str:
        n_ok = sum(1 for x in self.rules if x.ok)
        lines = [f"{'CLEAN' if self.passed else 'UNTIDY'}  ({n_ok}/{len(self.rules)} rules)  "
                 f"{self.process_name}  {self.file}"]
        for x in self.rules:
            mark = "ok  " if x.ok else "FAIL"
            lines.append(f"  {mark} {x.id:<16} {x.reason}")
        i = self.info
        if i:
            share = ", ".join(f"{k} {v:.0%}" for k, v in i["share_of_faces"].items())
            lines.append(f"  info {i['faces']} faces, {i['edges']} edges; faces by kind: {share}")
        lines.append("  every rule is measured exactly on the solid's faces, edges and corners")
        return "\n".join(lines)

    def to_json(self) -> dict:
        d = super().to_json()
        for k in ("direction_name", "direction", "direction_chosen", "candidates",
                  "sample_spacing_mm", "samples"):
            d.pop(k, None)
        d["info"] = self.info
        return d


def check_solid(solid, file: str = "", rules=None) -> Report:
    """``rules``: only these rule ids are reported (None: all). Every
    quality rule is exact and cheap, so the others are not worth skipping;
    they are left out of the report."""
    proc = load_file(RULES)
    if rules is not None and set(rules) - set(proc.rules):
        raise ValueError(f"not quality rules: {sorted(set(rules) - set(proc.rules))}")
    part = Part(solid, proc.settings["sample_spacing"])
    rules_out, info = run(part, proc)
    rules = [x for x in rules_out if rules is None or x.id in rules]
    return Report(file, proc.id, proc.name, "", [], "", rules, [], part.spacing, len(part.P), part, info=info)


def check(step_path: str, rules=None) -> Report:
    """Check one STEP file for modelling quality (``rules``: only these ids)."""
    return check_solid(load(step_path), os.path.basename(step_path), rules)


def write_fields(rep: Report, out_dir: str) -> str:
    from ..dfm.viz import write_fields as _w

    return _w(rep, rep.part, out_dir)


def write_images(rep: Report, out_dir: str) -> list[str]:
    from ..dfm.viz import write_images as _w

    return _w(rep, rep.part, out_dir)


__all__ = ["check", "check_solid", "Report", "write_fields", "write_images"]
