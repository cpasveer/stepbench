"""The joints as task families, through the core's own generate, workspace and grader."""
from __future__ import annotations

import os
import re

import pytest

from jointbench import families
from jointbench.geometry import write_step
from jointbench.grade import grade
from jointbench.joints import ORDER
from jointbench.joints import common as C
from jointbench.joints import family as F
from jointbench.task import Task
from jointbench.workspace import write_workspace


def test_every_joint_is_registered():
    reg = families.registry()
    for name in ORDER:
        fam = reg[name]
        assert sorted(fam.levels) == list(range(1, F.n_levels(name) + 1))
        assert "HOLES" not in fam.core_checks and "NO_INTERFERENCE" in fam.core_checks
        lv = fam.levels[1]
        for attr in ("LEVEL", "SITUATION", "KNOBS_DEFAULT", "generate", "build_connector", "SELFTEST_VARIANTS"):
            assert hasattr(lv, attr)
    assert "MANUFACTURING" in reg["snapfit"].core_checks and "MANUFACTURING" not in reg["kinematic"].core_checks


def _folder(name, level, seed, tmp_path):
    task, draw, ref = families.level(name, level).generate(seed)
    ws = write_workspace(task, ref, str(tmp_path), {"level": level, "seed": seed, "draw": draw})
    return Task.load(os.path.join(ws, "task.json")), ws, task, draw


def test_generate_is_deterministic(tmp_path):
    a = families.level("kinematic", 2).generate(5)[0]
    b = families.level("kinematic", 2).generate(5)[0]
    assert a.dumps() == b.dumps() and a.id == "kinematic-L2-s5"
    assert a.tolerances["required_checks"] == ["CLEAN"]


@pytest.mark.parametrize("name,level", [("kinematic", 2), ("revolute", 1), ("interference", 1)])
def test_reference_passes_and_naive_fails_from_a_folder(name, level, tmp_path):
    task, ws, _, _ = _folder(name, level, 0, tmp_path)
    assert os.path.exists(os.path.join(ws, f"fixed_{next(iter(task.parts))}.step"))
    rep = grade(task, os.path.join(ws, "_solution", "reference.step"), optional=False, score=False)
    assert rep.passed, rep.text()
    assert "HOLES" not in [c.name for c in rep.checks]
    stub = open(os.path.join(ws, "submission.py")).read()
    assert "mounting_faces" not in stub and "bolts" not in stub and "fixed_parts" in stub
    naive = os.path.join(ws, "naive.step")
    write_step(families.get(name).baselines["naive"](task), naive)
    assert not grade(task, naive, optional=False, score=False).passed


def test_interference_overlap_is_declared_and_nothing_else(tmp_path):
    task, ws, mem, draw = _folder("interference", 1, 0, tmp_path)
    assert task.intended_overlaps and task.intended_overlaps[0]["kind"] == "cylinder"
    lv = families.level("interference", 1)
    sunk = os.path.join(ws, "sunk.step")
    write_step(lv.build_connector(task, draw, shoulder_sink=0.05) if task.meta["joint"]["params"]["given"] == "hub"
               else lv.build_connector(task, draw, dia="lo-0.01"), sunk)
    rep = grade(task, sunk, optional=False, score=False)
    assert "FIT_OVERLAP" in rep.failed()


# ------------------------------------------------- level 3 keeps the standard's numbers to itself


def _hidden_numbers(inst, told: str) -> set:
    """The joint's worked-out numbers that the level 3 brief does not state, as
    they would be written (repr, 4 and 3 decimals)."""
    out = set()

    def walk(v):
        if isinstance(v, dict):
            for x in v.values():
                walk(x)
        elif isinstance(v, (list, tuple)):
            for x in v:
                walk(x)
        elif isinstance(v, float) and abs(v * 100 - round(v * 100)) > 1e-6:  # not a round size
            forms = {repr(v), f"{v:.4f}", f"{v:.3f}"}
            if not any(f in told for f in forms):
                out.update(forms)
    walk(inst.p)
    return out


L3 = [n for n in ORDER if F.n_levels(n) >= 3]


@pytest.mark.parametrize("name", L3)
def test_level3_task_json_holds_no_hidden_values(name, tmp_path):
    task, ws, _, _ = _folder(name, 3, 2000, tmp_path)
    text = open(os.path.join(ws, "task.json")).read()
    readme = open(os.path.join(ws, "README.md")).read()
    import json
    joint = json.loads(text)["meta"]["joint"]
    for k in F.HIDDEN_AT_L3:
        assert k not in joint, k
    assert '"inner_seed"' not in text
    inst = F.instance(task)  # the grader still gets them back
    assert not inst.rules_told and inst.p
    leaked = sorted(set(re.findall(r"\d+\.\d+", text)) & _hidden_numbers(inst, readme))
    assert not leaked, leaked
    rep = grade(task, os.path.join(ws, "_solution", "reference.step"), optional=False, score=False)
    assert rep.passed, rep.text()


@pytest.mark.parametrize("name", L3)
def test_level3_failure_reasons_state_no_limits(name, tmp_path):
    task, ws, _, draw = _folder(name, 3, 2000, tmp_path)
    inst = F.instance(task)
    hidden = _hidden_numbers(inst, open(os.path.join(ws, "README.md")).read())
    lv = families.level(name, 3)
    mod = F.get(name)
    reasons = []
    for label, v, expect in mod.SELFTEST:  # every broken copy the joint knows, sized for this draw
        if not expect or v.get("check") or v.get("knobs"):
            continue
        sol = C.roundtrip(C.to_world(mod.reference(inst, **v.get("part", {})), inst.frame), str(tmp_path))
        for r in mod.check(sol, inst):
            if r.ok is False:  # the measured size may be stated; it is the agent's own
                own = {f"{r.value:.4f}", f"{r.value:.3f}", repr(r.value)} if r.value is not None else set()
                reasons.append((r.reason, own))
    assert reasons
    told = [r for r, own in reasons if set(re.findall(r"\d+\.\d+", r)) & (hidden - own)]
    assert not told, told
