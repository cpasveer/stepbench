"""Reading the process files, and the shape of a result.

A process file (``processes/<id>.toml``) holds the rules of one process and
their limits; this module turns it into plain objects. The code of a rule
never contains its limit: change the file and the checker follows.
"""
from __future__ import annotations

import os
import tomllib
import numpy as np
from dataclasses import dataclass, field

HERE = os.path.join(os.path.dirname(__file__), "processes")


@dataclass
class Rule:
    id: str
    title: str
    kind: str  # "min" or "max"
    value: float
    unit: str
    source: str

    def ok(self, measured: float | None) -> bool:
        """Is the measured number on the good side of the limit? A rule with
        nothing to measure (no holes for a hole rule) passes."""
        if measured is None:
            return True
        eps = 1e-9 * max(1.0, abs(self.value))
        return measured >= self.value - eps if self.kind == "min" else measured <= self.value + eps

    def limit_text(self) -> str:
        u = "" if self.unit in ("ratio",) else f" {self.unit}"
        return f"{'min' if self.kind == 'min' else 'max'} {self.value:g}{u}"


@dataclass
class Process:
    id: str
    name: str
    status: str
    direction: str
    direction_meaning: str
    rules: dict[str, Rule]
    settings: dict[str, float]
    setting_notes: dict[str, str]
    wanted: frozenset | None = None  # the rules asked for; None = all

    def wants(self, *rule_ids: str) -> bool:
        """True if any of these rules was asked for (so its measurement is needed)."""
        return self.wanted is None or any(r in self.wanted for r in rule_ids)


def available() -> list[str]:
    return sorted(f[:-5] for f in os.listdir(HERE) if f.endswith(".toml"))


def load_process(pid: str) -> Process:
    path = os.path.join(HERE, f"{pid}.toml")
    if not os.path.exists(path):
        raise ValueError(f"unknown process {pid!r}; known: {', '.join(available())}")
    return load_file(path)


def load_file(path: str) -> Process:
    """Read any rule file of this shape (the quality checker uses it too)."""
    with open(path, "rb") as f:
        d = tomllib.load(f)
    p = d["process"]
    rules = {rid: Rule(rid, r["title"], r["kind"], float(r["value"]), r["unit"], r["source"])
             for rid, r in d.get("rules", {}).items()}
    sets = {k: float(v["value"]) for k, v in d.get("settings", {}).items()}
    notes = {k: f"{v['value']:g} {v['unit']}: {v['source']}" for k, v in d.get("settings", {}).items()}
    return Process(p["id"], p["name"], p["status"], p["direction"], p["direction_meaning"],
                   rules, sets, notes)


@dataclass
class Field:
    """A rule's measurement spread over the part: one value per display
    triangle (NaN where the rule does not measure), and which triangles fail.
    ``kind`` "scalar" is a number to colour by; "flag" marks regions (1 = in
    the region). ``marks`` are extra things to draw: spans, edges, points."""

    id: str
    rule: str
    title: str
    unit: str
    kind: str  # "min" / "max": which side of the limit is good
    limit: float | None
    values: object  # numpy array, one per triangle
    fail: object  # numpy bool array, one per triangle
    type: str = "scalar"
    note: str = ""
    marks: list = field(default_factory=list)
    cut: list | None = None  # a point to cut the part open at, for inner faces

    def ready(self) -> "Field":
        """Compute values and fail flags given as functions (see fdm._surf)."""
        if callable(self.values):
            v = np.asarray(self.values(), float)
            self.values = np.where(np.isfinite(v), v, np.nan)
        if callable(self.fail):
            self.fail = np.asarray(self.fail(), dtype=bool)
        return self


@dataclass
class RuleResult:
    id: str
    title: str
    ok: bool
    reason: str
    value: float | None
    limit: float
    unit: str
    kind: str
    where: list[dict] = field(default_factory=list)  # [{"point": [x,y,z], "face": i}, ...]
    measured: str = "exact"  # "exact" or "sampled"
    details: dict = field(default_factory=dict)
    surface: Field | None = None  # the measurement over the part, for pictures
    required: bool = True  # False: reported, but does not decide the verdict (e.g. overhang when support is allowed)

    def to_json(self) -> dict:
        return {"rule": self.id, "title": self.title, "status": "pass" if self.ok else "fail",
                "reason": self.reason, "value": self.value, "limit": self.limit, "unit": self.unit,
                "kind": self.kind, "measured": self.measured, "where": self.where,
                "details": self.details, "required": self.required}


def result(rule: Rule, value, reason_pass: str, reason_fail: str, where=(), measured="exact",
           details=None, ok: bool | None = None) -> RuleResult:
    good = rule.ok(value) if ok is None else ok
    reason = reason_pass if good else reason_fail
    return RuleResult(rule.id, rule.title, good, reason, value, rule.value, rule.unit, rule.kind,
                      list(where), measured, details or {})
