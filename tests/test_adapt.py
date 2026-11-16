"""Adapts: one part, two arrangements (the adapt family, angled mounts, no load)."""
from __future__ import annotations

import json
import os

import pytest

from jointbench import families
import math

from jointbench.families.adapt import angled as F
from jointbench.families.adapt.grading import grade_with_placement, parse_placement
from jointbench.geometry import Placement, rot_axis_angle
from jointbench.grade import grade
from jointbench.task import Task
from jointbench.workspace import write_workspace


@pytest.fixture(scope="module")
def made(tmp_path_factory):
    task, draw, ref = F.generate(0)
    out = tmp_path_factory.mktemp("adapt")
    ws = write_workspace(task, ref, str(out), {"level": 1, "seed": 0, "draw": draw})
    t = Task.load(os.path.join(ws, "task.json"))
    place = json.load(open(os.path.join(ws, "_solution", "placement.json")))
    return {"task": t, "mem": task, "draw": draw, "ws": ws, "ref": os.path.join(ws, "_solution", "reference.step"),
            "place": place}


# the placement tests are about fit: they leave out the slow printing and load analyses
FAST = frozenset({"MANUFACTURING", "STIFFNESS"})


def _grade(m, place):
    return grade_with_placement(m["task"], m["ref"], place, optional=False, score=False, skip=FAST)


def test_registered_and_deterministic(made):
    assert families.level("adapt", 1) is F
    again = F.generate(0)[0]
    assert again.dumps() == made["mem"].dumps() and made["task"].id == "adapts-angled-s0"
    assert made["task"].family == "adapt" and made["task"].level == 1


def test_each_arrangement_places_only_its_own_parts(made):
    t, ws = made["task"], made["ws"]
    assert set(t.poses[0]) == {"A", "B"} and set(t.poses[1]) == {"C", "D"}
    assert {b.part for b in t.bolts(0)} == {"A", "B"} and {b.part for b in t.bolts(1)} == {"C", "D"}
    assert set(t.fixed_solids(1)) == {"C", "D"}
    for n in "ABCD":
        assert os.path.exists(os.path.join(ws, f"fixed_{n}.step"))
    # every face is turned well away from the coordinate planes, in both arrangements
    for k in (0, 1):
        assert all(max(abs(v) for v in f.normal) <= F.AXIS_MAX for f in t.faces(k))


def test_mounts_are_angled_differently_and_spaced_differently(made):
    t = made["task"]
    ang, dist = [], []
    for k in (0, 1):
        f, g = t.faces(k)
        dot = sum(a * b for a, b in zip(f.normal, g.normal))
        ang.append(math.degrees(math.acos(max(-1.0, min(1.0, dot)))))
        dist.append(math.dist(f.origin, g.origin))
    for a in ang:  # never near parallel, never near square
        assert any(lo - 1e-6 <= a <= hi + 1e-6 for lo, hi in F.ANGLE_BANDS), ang
    assert abs(ang[0] - ang[1]) >= F.ANGLE_APART - 1e-6
    assert abs(dist[0] - dist[1]) >= F.DIST_APART - 1e-6


def test_task_states_level_three_in_both_arrangements(made):
    t = made["task"]
    req = t.tolerances["required_checks"]
    assert {"SEAT_INSIDE", "CLEAN", "MANUFACTURING", "STIFFNESS", "KEY_TURN"} <= set(req)
    assert t.tolerances["manufacturing"]["process"] == "fdm" and t.tolerances["score"] == "material"
    loaded = {a["name"]: a["stiffness"]["per_mount"] for a in t.meta["arrangements"]}
    assert [m["mount"] for m in loaded["1"]] == ["B"] and [m["mount"] for m in loaded["2"]] == ["D"]
    assert all(m["mass_kg"] == made["draw"]["load_kg"][a] for a, ms in loaded.items() for m in ms)


def test_reference_passes_everything_in_both_arrangements_from_the_folder(made):
    rep = grade(made["task"], made["ref"], optional=False)  # reads _solution/placement.json
    assert rep.passed, rep.text()
    assert [a["outcome"] for a in rep.arrangements] == ["pass", "pass"]
    # the part itself, once: clean modelling and printing, on the file as handed in
    part = [c for c in rep.checks if c.details.get("scope") == "part"]
    assert any(c.name.startswith("FDM_") for c in part) and any(c.name == "Q_STUB" for c in part)
    assert not any(c.name.startswith(("FDM_", "Q_")) for c in rep.checks if "arrangement" in c.details)
    # the stated load, in each arrangement
    st = [c for c in rep.checks if c.name == "STIFFNESS"]
    assert sorted(c.details["arrangement"] for c in st) == ["1", "2"] and all(c.ok for c in st)
    assert rep.info["material_used"]["grams"] > 0
    # every bolt turns with its hex key, in each arrangement
    kt = [c for c in rep.checks if c.name == "KEY_TURN"]
    assert sorted(c.details["arrangement"] for c in kt) == ["1", "2"] and all(c.ok for c in kt)
    txt = rep.text()
    assert "arrangement 2 (fixed parts C, D): the part passes" in txt
    assert "the part itself, as handed in" in txt and "g printed" in txt


def test_the_same_pose_in_both_arrangements_fails_in_the_second(made):
    p = made["place"]
    rep = _grade(made, {"1": p["1"], "2": p["1"]})
    assert not rep.passed
    assert [a["outcome"] for a in rep.arrangements] == ["pass", "fail"]
    bad = [c for c in rep.checks if c.ok is False and c.required]
    assert bad and all(c.details.get("arrangement") == "2" for c in bad)
    assert bad[0].reason.startswith("in arrangement 2: ") and ("C1" in bad[0].reason or "D1" in bad[0].reason)
    assert "arrangement 2 (fixed parts C, D): the part fails: " + bad[0].name in rep.text()


def test_a_small_shift_in_one_arrangement_fails_there_only(made):
    p = json.loads(json.dumps(made["place"]))
    r = p["2"]["rotation"]
    p["2"]["translation"] = [p["2"]["translation"][i] + 0.3 * r[i][1] for i in range(3)]
    rep = _grade(made, p)
    assert [a["outcome"] for a in rep.arrangements] == ["pass", "fail"]
    assert "HOLES" in rep.arrangements[1]["failed"]


def test_part_moved_with_its_placement_is_the_same_answer(made, tmp_path):
    """Moving output.step and undoing the move in placement.json changes nothing."""
    from jointbench.geometry import read_step, write_step

    g = Placement((12.5, -40.0, 7.0), rot_axis_angle((1.0, 2.0, 3.0), 37.0))
    moved = os.path.join(tmp_path, "moved.step")
    write_step(read_step(made["ref"]).moved(g.location()), moved)
    inv_r = tuple(tuple(g.rot[j][i] for j in range(3)) for i in range(3))
    inv = Placement(tuple(-sum(inv_r[i][k] * g.origin[k] for k in range(3)) for i in range(3)), inv_r)
    place = {}
    for a, e in made["place"].items():
        h = inv.then(Placement(tuple(e["translation"]), tuple(tuple(r) for r in e["rotation"])))
        place[a] = {"rotation": [list(r) for r in h.rot], "translation": list(h.origin)}
    rep = grade_with_placement(made["task"], moved, place, optional=False, score=False, skip=FAST)
    assert rep.passed, rep.text()


def test_placement_file_is_checked():
    eye = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    ok, res = parse_placement({"1": {"rotation": eye, "translation": [0, 0, 0]},
                               "2": {"rotation": eye, "translation": [1, 2, 3]}}, ["1", "2"])
    assert res.ok and ok["2"].origin == (1.0, 2.0, 3.0)
    _, res = parse_placement({"1": {"rotation": eye, "translation": [0, 0, 0]}}, ["1", "2"])
    assert res.ok is False and "arrangement 2" in res.reason
    mirror = [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]
    _, res = parse_placement({n: {"rotation": mirror, "translation": [0, 0, 0]} for n in "12"}, ["1", "2"])
    assert res.ok is False and "mirror" in res.reason
    skew = [[1, 0.01, 0], [0, 1, 0], [0, 0, 1]]
    _, res = parse_placement({n: {"rotation": skew, "translation": [0, 0, 0]} for n in "12"}, ["1", "2"])
    assert res.ok is False and "not a rotation" in res.reason
    # typed-in numbers a little off are squared up to the nearest rotation
    near = [[0.70711, -0.70711, 0], [0.70711, 0.70711, 0], [0, 0, 1]]
    got, res = parse_placement({n: {"rotation": near, "translation": [0, 0, 0]} for n in "12"}, ["1", "2"])
    assert res.ok
    r = got["1"].rot
    assert abs(sum(r[k][0] * r[k][1] for k in range(3))) < 1e-12 and abs(sum(v * v for v in [x[0] for x in r]) - 1) < 1e-12
    _, res = parse_placement([1, 2], ["1", "2"])
    assert res.ok is False


def test_missing_placement_file_fails_cleanly(made, tmp_path):
    import shutil

    step = os.path.join(tmp_path, "output.step")
    shutil.copyfile(made["ref"], step)
    t = Task.load(os.path.join(made["ws"], "task.json"))
    t.root = str(tmp_path)  # no placement.json anywhere
    rep = grade(t, step, optional=False, score=False)
    assert not rep.passed and rep.failed() == ["PLACEMENT"] and "no placement.json" in rep.text()


def test_trivial_draws_are_recognised(made):
    lay = F.layout(made["draw"])
    assert not F.trivial(lay)
    # arrangement 2 made a rigid copy of arrangement 1: one set of features would serve both
    g = Placement((30.0, -10.0, 5.0), rot_axis_angle((0.0, 1.0, 1.0), 50.0))
    pl = dict(lay["placements"], C=lay["placements"]["A"].then(g), D=lay["placements"]["B"].then(g))
    itf = dict(lay["itf"], C=lay["itf"]["A"], D=lay["itf"]["B"])
    assert F.trivial(dict(lay, placements=pl, itf=itf))
    # the same pair of mounts, swapped, is just as trivial
    pl2 = dict(pl, C=pl["D"], D=pl["C"])
    itf2 = dict(itf, C=itf["D"], D=itf["C"])
    assert F.trivial(dict(lay, placements=pl2, itf=itf2))


def test_brief_states_both_arrangements_and_no_relation(made):
    txt = open(os.path.join(made["ws"], "README.md")).read()
    for b in made["task"].bolts(0) + made["task"].bolts(1):
        assert f"| {b.id} |" in txt
    assert "placement.json" in txt and "R p + t" in txt and "mirror" in txt
    assert "Arrangement 1: fixed parts A and B" in txt and "Arrangement 2: fixed parts C and D" in txt
    # nothing of the answer's shape, nor of how the arrangements relate
    assert "at an angle to each other" in txt and "parallel" not in txt
    assert "printed in PLA" in txt and "lighter is better" in txt and "printed once" in txt
    sizes = {b.size for k in (0, 1) for b in made["task"].bolts(k)}  # keys for both arrangements' bolts
    assert "`KEY_TURN`" in txt and all(f"{z}: " in txt for z in sizes)
    for a in made["task"].meta["arrangements"]:  # each arrangement's stated load, in its own section
        for m in a["stiffness"]["per_mount"]:
            assert m["statement"] in txt
    for word in ("facet", "convex", "reference", "_solution", "same pose"):
        assert word not in txt.lower()
    for a in made["draw"]["angles"]:  # the angles are the model's to find
        assert f"{a:g} deg" not in txt and f"{a:g} degrees" not in txt
    stub = open(os.path.join(made["ws"], "submission.py")).read()
    assert "placement.json" in stub
