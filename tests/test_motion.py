"""Tests for the motion package (src/jointbench/motion). The full self-test is
``python -m jointbench.motion selftest``; these are the quick checks."""
import json
import math

import pytest

from jointbench.motion import generate, kin, selftest
from jointbench.motion.grade import grade_solid
from jointbench.motion.kin import Mechanism


def _hidden(task, m):
    h = Mechanism.from_json(task["mechanism"])
    for pid, p in h.parts.items():
        if pid != task["removed"]:
            p.solid = m.meta["solids"][pid]
    return h


@pytest.fixture(scope="module")
def fourbar():
    return generate.make("fourbar", 1)


@pytest.mark.parametrize("mtype", ["fourbar", "slider", "yoke", "oldham"])
def test_closed_form_agrees_with_numeric(mtype):
    task, m, _ = generate.make(mtype, 2)
    a = kin.agreement(m, 2.0)
    assert a["ok"], a
    assert a["mm"] < 1e-6


def test_generation_is_deterministic():
    t1, _, _ = generate.make("slider", 3)
    t2, _, _ = generate.make("slider", 3)
    t1.pop("generation"), t2.pop("generation")
    assert json.dumps(t1, sort_keys=True) == json.dumps(t2, sort_keys=True)


def test_reference_passes_and_reshaped_passes(fourbar):
    task, m, ref = fourbar
    assert grade_solid(task, _hidden(task, m), ref).ok
    body = selftest.reshaped(m, task["removed"])
    assert grade_solid(task, _hidden(task, m), selftest._world(m, task["removed"], body)).ok


def test_shifted_pivot_matches_independent_path(fourbar):
    task, m, _ = fourbar
    rem = task["removed"]
    solid, feats = selftest.shifted(m, rem, 1.0)
    rep = grade_solid(task, _hidden(task, m), selftest._world(m, rem, solid))
    dev = selftest.path_error(task, m, rem, feats)
    out = next(c for c in rep.checks if c.name == "OUTPUT")
    assert abs(out.value - dev) < 1e-4
    assert (out.ok is False) == (dev > task["tolerances"]["angle_deg" if m.output["kind"] == "angle" else "point_mm"])


def test_jam_is_named_with_its_input_value(fourbar):
    task, m, _ = fourbar
    rem = task["removed"]
    rep = grade_solid(task, _hidden(task, m), selftest._world(m, rem, selftest.jams(m, rem, task)))
    c = next(c for c in rep.checks if c.name == "ASSEMBLES")
    assert c.ok is False and "jams at" in c.reason and "deg" in c.reason


def test_bump_fouls_where_it_comes_closest(fourbar):
    task, m, _ = fourbar
    rem = task["removed"]
    solid, closest = selftest.fouls(m, rem, m.meta["solids"])
    rep = grade_solid(task, _hidden(task, m), selftest._world(m, rem, solid))
    c = next(c for c in rep.checks if c.name == "NO_INTERFERENCE")
    assert c.ok is False
    angs = [h[0] % 360 for h in c.details["clashes"]]
    assert any(abs((a - closest % 360 + 180) % 360 - 180) <= 5.0 for a in angs)


def test_rrr_reports_no_position():
    task, m, _ = generate.make("fourbar", 4)
    m2 = m.copy()
    f = m2.parts["coupler"].features["h2"]
    m2.parts["coupler"].features["h2"] = kin.Feature(f.kind, (f.xy[0] - 40.0, f.xy[1]), f.size, f.z)
    sols, jam = kin.sweep(m2, kin.input_values(m2, 5.0))
    assert jam is not None and jam.short_by > 0 and math.isfinite(jam.value)


def test_geneva_reference_passes_and_narrow_slots_jam():
    from jointbench.motion import contact
    task, m, ref = contact.make("geneva", 1)
    hidden = Mechanism.from_json(task["mechanism"])
    solids = m.meta["solids"]
    assert contact.grade_solid(task, hidden, solids, ref).ok
    rows = {name: (rec, exp) for name, rec, exp in selftest.contact_variants(m, task)}
    rec, _ = rows["narrow_slots"]
    bad = selftest._world(m, "wheel", contact.build_part("wheel", rec))
    rep = contact.grade_solid(task, hidden, solids, bad)
    assert not rep.ok and rep.failed()[0] == "MOTION"


def test_equal_holes_keep_the_part_as_submitted(monkeypatch):
    """A bent rod has two equal holes, so both ways of matching them to the
    crank and slider pins reproduce the table; turned end for end the bend
    would lie on the wrong side and foul its obstacle. The grader must keep
    the matching that leaves the part where it was modelled, whatever order
    the holes are found in. (slider-L2-s4000 failed valid rods this way.)"""
    from jointbench.motion import grade as G, measure

    task, m, ref = generate.make("slider", 1, {"remove": "rod", "obstacles": 1})
    found = measure.all_features
    for order in (lambda xs: xs, lambda xs: list(reversed(xs))):
        monkeypatch.setattr(G, "all_features", lambda s, o=order: o(found(s)))
        rep = grade_solid(task, _hidden(task, m), ref)
        assert rep.ok, [c.reason for c in rep.checks if c.ok is False]
    monkeypatch.setattr(G, "all_features", found)
    bad = selftest.mirrored(m, task["removed"])
    rep = grade_solid(task, _hidden(task, m), selftest._world(m, task["removed"], bad))
    assert not rep.ok and "NO_INTERFERENCE" in rep.failed()
