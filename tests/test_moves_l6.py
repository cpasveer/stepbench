"""Level 6, moves, on the pipe setup: the generator (from its stored showcase draw), the folder, the brief,
the hidden reference passing the whole grader, and every broken copy failing on its named check."""
import json
import os

import pytest

from jointbench.moves.l6 import generate as G
from jointbench.moves.l6.grade import grade

SETUP, SEED = "pipe", 0
HAVE_DRAW = os.path.isfile(G.frozen_path(SETUP, SEED))
needs_draw = pytest.mark.skipif(not HAVE_DRAW, reason="no stored showcase draw for the pipe")


def test_each_nut_has_its_room_clear_of_the_other_stack():
    for bias in (-1.5, 0.0, 3.5):
        for crank_first in (True, False):
            L = G.layers(bias, crank_first)
            SL = G.slabs(L)
            for stack, other in (("crank", "rocker"), ("rocker", "crank")):
                inward = 1.0 if L[stack] < L["latch"] else -1.0
                face = L[stack] + inward * L["W"] / 2
                lo, hi = sorted((face, face + inward * G.NUT_ROOM))
                for k in ("_link", "_ear_lo", "_ear_hi"):
                    w0, w1 = SL[other + k]
                    assert w1 <= lo or w0 >= hi, (stack, other + k)


@pytest.fixture(scope="module")
def folder(tmp_path_factory):
    if not HAVE_DRAW:
        pytest.skip("no stored showcase draw for the pipe")
    task, d, start, asm = G.generate(SETUP, SEED, check=False)
    out = tmp_path_factory.mktemp("l6")
    return task, d, G.write_folder(task, d, start, asm, str(out), G.broken_variants(SETUP, SEED, d))


@needs_draw
def test_task_states_the_motion_box_latch_and_load_and_hides_the_linkage(folder):
    task, d, ws = folder
    assert task.id == "moves-L6-pipe-s0" and task.level == 6 and task.meta["level_name"] == "6"
    mo = task.meta["motion"]
    assert [t["name"] for t in mo["targets"]] == ["pose2", "pose1", "start"]
    assert mo["latch"]["pull_direction"] == [0, 0, -1] and mo["latch"]["load_kg"] == 1.0
    assert task.obstacles and "opening" in mo["box"]
    text = task.dumps()
    for word in ("O2", "crank", "rocker", "theta", "ledge", "lug"):
        assert word not in text
    assert {"SEAT_INSIDE", "CLEAN", "MANUFACTURING", "KEY_TURN"} <= set(task.tolerances["required_checks"])


@needs_draw
def test_folder_holds_what_a_model_needs_and_the_brief_states_the_rules(folder):
    task, d, ws = folder
    have = set(os.listdir(ws))
    assert {"task.json", "README.md", "box.step", "fixed_A.step", "fixed_B.step", "fixed_B_start.step",
            "fixed_B_pose1.step", "fixed_B_pose2.step", "_solution"} <= have
    readme = open(os.path.join(ws, "README.md"), encoding="utf-8").read()
    for words in ("## The box", "## The latch and the load", "## Pivots on ball bearings", "LATCH_STRENGTH",
                  "PIVOT_SEATS", '"path"', "jointbench.moves.l6 verify", "Model and assemble everything with B at home"):
        assert words in readme, words
    low = readme.lower()
    for word in ("crank", "rocker", "four-bar", "fourbar", "clevis", "ledge", "lug"):
        assert word not in low, word
    assert sorted(os.listdir(os.path.join(ws, "_solution", "broken"))) == sorted(n for n, *_ in G.BROKEN)


@needs_draw
def test_reference_passes_the_whole_grader(folder):
    task, d, ws = folder
    rep = grade(task, os.path.join(ws, "_solution", "assembly.json"))
    assert rep.passed, rep.text()
    names = [c.name for c in rep.checks]
    for n in ("PIVOTS", "MOBILITY", "MOTION", "GAP", "POSES", "INSERTION", "LATCH", "LATCH_STRENGTH", "PIVOT_SEATS",
              "frame TOOL_ACCESS", "carrier TOOL_ACCESS"):
        assert n in names, n
    poses = next(c for c in rep.checks if c.name == "POSES").details["poses"]
    assert all(p["position_mm"] < 1e-3 and p["angle_deg"] < 1e-3 for p in poses)


@needs_draw
@pytest.mark.parametrize("name", [n for n, *_ in G.BROKEN])
def test_broken_copies_fail_on_the_named_check(folder, name):
    task, d, ws = folder
    bf = os.path.join(ws, "_solution", "broken", name)
    must = open(os.path.join(bf, "MUST_FAIL"), encoding="utf-8").read().strip()
    rep = grade(task, os.path.join(bf, "assembly.json"), manufacturing=False)
    assert not rep.passed and must in rep.failed(), rep.text()


@needs_draw
def test_a_part_left_outside_the_box_fails_insertion(folder, tmp_path):
    task, d, ws = folder
    sol = os.path.join(ws, "_solution")
    asm = json.load(open(os.path.join(sol, "assembly.json"), encoding="utf-8"))
    for p in asm["parts"]:
        p["file"] = os.path.join(sol, p["file"])
        if p["name"] == "carrier":  # straight in sideways: through a wall
            p["insert_direction"] = [float(v) for v in -G.Plane(G.setup_for(SETUP, SEED).frame()).s]
    f = tmp_path / "assembly.json"
    f.write_text(json.dumps(asm))
    rep = grade(task, str(f), manufacturing=False)
    assert "INSERTION" in rep.failed(), rep.text()
