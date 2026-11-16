"""Level 4d, "locks": the generator, the hidden bayonet, and the grader on it and its broken copies."""
from __future__ import annotations

import os
import re

import pytest

from jointbench import families
from jointbench.locks import generate as G
from jointbench.locks.grade import grade

SEED = 0


def test_generator_is_deterministic_and_states_the_motion():
    a = G.generate(SEED, check=False)
    b = G.generate(SEED, check=False)
    assert a[0].dumps() == b[0].dumps() and a[0].id == "locks-s0" and a[0].family == "locks"
    t, d = a[0], a[1]
    lk = t.meta["lock"]
    assert 30 <= abs(lk["turn_deg"]) <= 90 and lk["insert_direction"] == [0.0, 0.0, -1.0]
    assert t.meta["level_name"] == "4d" and t.tolerances["tool_length"] == 30.0
    assert 0.5 <= lk["pull_load_kg"] <= 1.5 and abs(lk["pull_load_N"] - lk["pull_load_kg"] * 9.81) < 1e-6
    assert "KEY_TURN" in t.tolerances["required_checks"] and t.tolerances["score"] == "material"
    assert set(t.tolerances["required_checks"]) == {"SEAT_INSIDE", "CLEAN", "MANUFACTURING", "KEY_TURN"}
    text = t.dumps()
    for word in ("lug", "phi0", "Ro", "bayonet"):
        assert word not in text
    assert families.get("locks").levels[4] is G


@pytest.fixture(scope="module")
def folder(tmp_path_factory):
    task, d, parts = G.generate(SEED, check=False)
    return task, d, G.write_folder(task, d, parts, str(tmp_path_factory.mktemp("locks")))


def test_folder_and_brief(folder):
    task, d, ws = folder
    assert {"task.json", "README.md", "fixed_A.step", "fixed_B.step", "fixed_B_turned.step", "_solution"} <= set(os.listdir(ws))
    readme = open(os.path.join(ws, "README.md")).read()
    for word in ("LOCKED", "UNLOCKS", "CONNECT", "part_A.step", f"{task.meta['lock']['turn_deg']:g} degrees"):
        assert word in readme
    for word in ("bayonet", "lugs?", "slots?", "tube", "grooves?"):  # whole words: "plug" is in the strength rule
        assert not re.search(rf"\b{word}\b", readme.lower()), word


def test_reference_passes_with_the_expected_play(folder):
    task, d, ws = folder
    rep = grade(task, os.path.join(ws, "_solution"), manufacturing=False)
    assert rep.passed, rep.text()
    locked = next(c for c in rep.checks if c.name == "LOCKED")
    assert abs(locked.value - G.REF_GAP * 2 ** 0.5) < 0.02  # the 45-degree gap closes after 0.4 x sqrt 2 mm
    assert {"part_A TOOL_ACCESS", "part_B TOOL_ACCESS", "part_A KEY_TURN", "part_B KEY_TURN", "TURN", "GAP", "UNLOCKS",
            "INTERLOCK", "PLAY", "LOCK_STRENGTH"} <= {c.name for c in rep.checks}
    strength = next(c for c in rep.checks if c.name == "LOCK_STRENGTH")
    assert strength.ok and 0 < strength.value < 1
    assert rep.info["material_used"]["grams"] > 0  # the score: printed weight of both parts


@pytest.mark.parametrize("name", [n for n, *_ in G.broken_variants({})])
def test_broken_copies_fail_on_the_named_check(folder, name):
    task, d, ws = folder
    bf = os.path.join(ws, "_solution", "broken", name)
    must = open(os.path.join(bf, "MUST_FAIL")).read().strip()
    kg = os.path.join(bf, "LOAD_KG")
    t = G.with_load(task, float(open(kg).read())) if os.path.exists(kg) else task
    rep = grade(t, bf, manufacturing=False)
    assert not rep.passed and must in rep.failed(), rep.text()
    if name == "overloaded":
        assert rep.failed() == ["LOCK_STRENGTH"]  # the reference itself: only the load is too much


def test_missing_file_fails_cleanly(folder, tmp_path):
    task, d, ws = folder
    rep = grade(task, str(tmp_path), manufacturing=False)
    assert not rep.passed and "part_A FILE_VALID" in rep.failed()
