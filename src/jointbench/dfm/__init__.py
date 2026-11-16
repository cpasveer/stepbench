"""Manufacturing rule checker: can this solid be made by this process?

    from jointbench.dfm import check
    rep = check("part.step", "fdm")                 # build direction chosen
    rep = check("part.step", "moulding", (0, 0, 1)) # pull direction stated
    print(rep.text()); rep.to_json()

or from the command line:

    python -m jointbench.dfm part.step --process fdm [--direction 0,0,1] [--json]
                             [--fields DIR] [--images DIR]

Each process is a rule set whose limits live in ``processes/<id>.toml``.
For every rule the report gives pass or fail, a one-line reason, the
measured number against its limit, and where on the part it fails (a point
in world coordinates and the face index), so a viewer can mark it.

Direction. Printing and moulding both depend on one direction (the build
direction; the pull direction). If the caller states it, that is used. If
not, a fixed list of candidates is tried -- the six axis directions (three
lines for moulding), then the largest flat faces turned to face the bed or
the parting plane -- and the candidate that passes the most rules wins;
ties go to the smaller failing area, then to the earlier candidate. The
report says which direction was used and how it was chosen.

Same file and process in, same report out: the sampling grid depends only
on the geometry, every number is rounded before it is reported, and
nothing random or time-dependent enters.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import fdm, moulding, pbf
from .geom import Part, load, r
from .rules import Process, RuleResult, available, load_process

PROCESSES = {"fdm": fdm, "moulding": moulding, "pbf": pbf}


@dataclass
class Report:
    file: str
    process: str
    process_name: str
    direction_name: str
    direction: list[float]
    direction_chosen: str  # "as declared", or "chosen by the checker (best of N candidates: ...)"
    rules: list[RuleResult]
    candidates: list[dict] = field(default_factory=list)
    spacing: float = 1.0
    samples: int = 0
    part: object = field(default=None, repr=False)  # the measured Part, for fields and pictures
    support: str = ""  # filament: "allowed" or "forbidden"

    @property
    def passed(self) -> bool:
        """Every required rule passes (reported-only rules do not count)."""
        return all(x.ok for x in self.rules if x.required)

    def failed(self) -> list[str]:
        return [x.id for x in self.rules if not x.ok]

    def text(self) -> str:
        n_ok = sum(1 for x in self.rules if x.ok)
        dv = ", ".join(f"{c:g}" for c in self.direction)
        lines = [f"{'PASS' if self.passed else 'FAIL'}  ({n_ok}/{len(self.rules)} rules)  "
                 f"{self.process_name}  {self.file}"]
        if self.direction:
            lines.append(f"  {self.direction_name} ({dv}), {self.direction_chosen}")
        if self.support:
            lines.append(f"  support {self.support}")
        for x in self.rules:
            mark = ("ok  " if x.ok else "FAIL") if x.required else ("ok  " if x.ok else "info")
            lines.append(f"  {mark} {x.id:<18} {x.reason}" + ("" if x.required else "  (reported, not required)"))
        lines.append(f"  sampled rules judge {self.samples} mesh triangles, sides at most {self.spacing:g} mm")
        return "\n".join(lines)

    def to_json(self) -> dict:
        return {"file": self.file, "process": self.process, "process_name": self.process_name,
                "passed": self.passed, "direction_name": self.direction_name,
                "direction": self.direction, "direction_chosen": self.direction_chosen,
                "rules": [x.to_json() for x in self.rules], "candidates": self.candidates,
                "sample_spacing_mm": self.spacing, "samples": self.samples, "support": self.support or None}


def _order(proc: Process, results: list[RuleResult]) -> list[RuleResult]:
    pos = {rid: k for k, rid in enumerate(proc.rules)}
    return sorted(results, key=lambda x: pos[x.id])


def _fail_area(results: list[RuleResult]) -> float:
    a = 0.0
    for x in results:
        if not x.ok:
            a += float(x.details.get("area_over_limit_mm2") or x.details.get("area_short_mm2")
                       or (x.value if x.unit == "mm2" and x.value else 0.0) or 1.0)
    return a


class MeasurementError(RuntimeError):
    """The geometry kernel could not measure this part (not: the part is not
    makeable). A grader should report it as its own verdict, "error", and
    not count it as a graded attempt, as for a missing output.step."""


def _measured(what: str, fn, *args):
    try:
        return fn(*args)
    except (MeasurementError, ValueError, NotImplementedError):
        raise  # our own verdicts on the file (e.g. "expected exactly 1 solid"), not kernel failures
    except Exception as e:  # noqa: BLE001 -- OCP raises Standard_Failure and friends
        raise MeasurementError(f"{what}: {type(e).__name__}: {e}".rstrip(": ")) from e


# rules whose result depends on the build or pull direction: the costly
# direction search runs only if one of them is asked for
DIRECTIONAL = {"FDM_OVERHANG", "FDM_BRIDGE", "FDM_BED_CONTACT", "FDM_STANCE", "FDM_SUPPORT_REMOVABLE", "IM_DRAFT", "IM_UNDERCUT"}
SUPPORT_RULES = ("FDM_OVERHANG", "FDM_BRIDGE")  # reported, not required, when support is allowed

_PARTS: dict = {}  # (file hash, spacing) -> Part: one loaded, meshed and measured part per file


def part_for(step_path: str, spacing: float) -> Part:
    """The measured part for a STEP file, shared by every process and call
    with the same sample spacing (keyed by the file's content, so an edited
    file is measured afresh). Its measurements are computed once, when
    first needed, and kept."""
    import hashlib

    with open(step_path, "rb") as fh:
        key = (hashlib.sha256(fh.read()).hexdigest(), float(spacing))
    if key not in _PARTS:
        if len(_PARTS) >= 4:
            _PARTS.pop(next(iter(_PARTS)))
        _PARTS[key] = Part(load(step_path), spacing, source=step_path)
    return _PARTS[key]


def check_solid(solid, process: str, direction=None, file: str = "", rules=None, part: Part | None = None,
                support: str | None = None) -> Report:
    """``rules``: only these rule ids are measured and reported (None: all).
    Measurements no asked-for rule needs are skipped. ``support`` (filament
    only): "forbidden" (default; overhang and bridge rules required) or
    "allowed" (they are reported, not required; support counts as material).
    Either way support that could not be removed fails the part
    (FDM_SUPPORT_REMOVABLE)."""
    from dataclasses import replace

    proc = load_process(process)
    if proc.status != "built":
        raise NotImplementedError(f"process {process!r} is {proc.status}, not built yet "
                                  f"(rules listed in processes/{process}.toml)")
    if rules is not None:
        unknown = set(rules) - set(proc.rules)
        if unknown:
            raise ValueError(f"not rules of {process}: {sorted(unknown)}")
        proc = replace(proc, wanted=frozenset(rules))
    mod = PROCESSES[process]
    part = part if part is not None else Part(solid, proc.settings["sample_spacing"])
    base = [x for x in _measured(f"{process}, direction-free rules", mod.independent, part, proc) if proc.wants(x.id)]
    if not proc.direction or not proc.wants(*DIRECTIONAL):  # no asked-for rule depends on a direction
        from .locate import annotate

        annotate(base, part)
        return Report(file, proc.id, proc.name, "", [], "not needed", _order(proc, base), [],
                      part.spacing, len(part.P), part)
    if direction is not None:
        d = np.array(direction, dtype=float)
        d = d / np.linalg.norm(d)
        dep = [x for x in _measured(f"{process}, rules in the stated direction", mod.dependent, part, proc, d)
               if proc.wants(x.id)]
        chosen = "as declared"
        cands = []
    else:
        signed = process == "fdm"
        options = part.candidate_directions(int(proc.settings["candidate_faces"]), signed,
                                               int(proc.settings.get("cross_faces", 0)))
        scored = []
        for k, (v, name) in enumerate(options):
            dep_k = [x for x in _measured(f"{process}, rules in direction {name}", mod.dependent, part, proc, v)
                     if proc.wants(x.id)]
            n_pass = sum(x.ok for x in dep_k)
            scored.append((-n_pass, round(_fail_area(dep_k), 6), k, v, name, dep_k))
        scored.sort(key=lambda t: (t[0], t[1], t[2]))
        _, _, _, d, name, dep = scored[0]
        chosen = f"chosen by the checker (best of {len(options)} candidates: {name})"
        cands = [{"direction": [r(c, 6) for c in t[3]], "name": t[4],
                  "rules_passed": -t[0] + sum(x.ok for x in base),
                  "failed": [x.id for x in base + t[5] if not x.ok]}
                 for t in sorted(scored, key=lambda t: t[2])]
    sup = ""
    if process == "fdm":
        sup = support or ("allowed" if proc.settings.get("support_allowed", 0) else "forbidden")
        if sup not in ("allowed", "forbidden"):
            raise ValueError(f"support must be 'allowed' or 'forbidden', not {sup!r}")
        if sup == "allowed":
            for x in dep:
                if x.id in SUPPORT_RULES:
                    x.required = False
        if proc.wants("FDM_SUPPORT_REMOVABLE") and "FDM_SUPPORT_REMOVABLE" in proc.rules:
            dep = dep + [_measured("fdm, support removal", mod.support_removable, part, proc, d, dep)]
    from .locate import annotate

    annotate(base + dep, part)
    return Report(file, proc.id, proc.name, proc.direction, [r(c, 6) for c in d], chosen,
                  _order(proc, base + dep), cands, part.spacing, len(part.P), part, sup)


def check(step_path: str, process: str, direction=None, rules=None, support: str | None = None) -> Report:
    """Check one STEP file against one process. ``direction`` is the build or
    pull direction as (x, y, z); None lets the checker choose. ``rules``: only
    these rule ids (None: all of the process's rules). ``support`` (filament):
    "allowed" or "forbidden" (default)."""
    import os

    spacing = load_process(process).settings["sample_spacing"]
    part = _measured("loading and meshing the part", part_for, step_path, spacing)
    return check_solid(part.solid, process, direction, os.path.basename(step_path), rules, part, support)


def probe(step_path: str, point, spacing: float = 1.5) -> dict:
    """The measurements at any point of the part's surface (click-to-probe):
    the point is moved to the nearest spot on the surface, and there the
    wall thickness (straight through), the gap (straight out) and the
    section thickness (largest ball) are measured exactly, as the rules
    measure them at their samples."""
    import math as _m

    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
    from OCP.gp import gp_Pnt

    from .geom import _ball_points, _faces_extrema

    part = part_for(step_path, spacing)
    ext = part._cache.get("extrema") or _faces_extrema(part.faces)
    part._cache["extrema"] = ext
    ext.LoadS1(BRepBuilderAPI_MakeVertex(gp_Pnt(*map(float, point))).Vertex())
    ext.Perform()
    q = ext.PointOnShape2(1)
    q = np.array([q.X(), q.Y(), q.Z()])
    face = part.face_index(ext.SupportOnShape2(1))
    n = part.normal_at(face, q)
    t = part.ray(q, -n)
    g = part.ray(q, n)
    b = float(_ball_points(part.faces, q[None, :], n[None, :], np.array([t]), ext)[0])

    def num(v):
        return None if not _m.isfinite(v) else r(float(v), 4)

    return {"point": [r(float(c), 4) for c in q], "face": face, "moved_by": r(float(np.linalg.norm(q - np.asarray(point, float))), 4),
            "normal": [r(float(c), 6) for c in n], "wall_thickness": num(t), "gap": num(g),
            "section_thickness": num(b), "units": "mm"}


DECLARED_KEYS = {"fdm": "build_direction", "moulding": "pull_direction"}


def declaration(path: str) -> tuple[str | None, list | None]:
    """Read an agent's declaration: ``build.json`` beside output.step (or
    the file itself). Format (all keys optional except process)::

        {"process": "fdm",      "build_direction": [x, y, z]}  # from the bed up into the part
        {"process": "moulding", "pull_direction":  [x, y, z]}  # the line the mould halves part along
        {"process": "pbf"}                                     # no direction: the powder holds the part

    Returns (process, direction or None). A direction must be three finite
    numbers, not all zero; it need not be of unit length. A missing file
    gives (None, None): the checker then chooses the direction itself."""
    import json
    import math as _m
    import os

    f = os.path.join(path, "build.json") if os.path.isdir(path) else path
    if not os.path.exists(f):
        return None, None
    with open(f, encoding="utf-8") as fh:
        d = json.load(fh)
    proc = d.get("process")
    key = DECLARED_KEYS.get(proc)
    v = d.get(key) if key else None
    if v is None:
        return proc, None
    if (not isinstance(v, list) or len(v) != 3 or not all(isinstance(c, (int, float)) and _m.isfinite(c) for c in v)
            or not any(v)):
        raise ValueError(f"{key} in {os.path.basename(f)} must be three numbers, not all zero; got {v!r}")
    return proc, [float(c) for c in v]


def write_fields(rep: Report, out_dir: str) -> str:
    """Write the part's mesh with every rule's field (see docs/dfm.md)."""
    from .viz import write_fields as _w

    return _w(rep, rep.part, out_dir)


def write_images(rep: Report, out_dir: str) -> list[str]:
    """Write one picture per rule: the part coloured by that rule's field."""
    from .viz import write_images as _w

    return _w(rep, rep.part, out_dir)


__all__ = ["write_fields", "write_images", "check", "check_solid", "probe", "Report", "available", "load_process",
           "MeasurementError", "declaration"]
