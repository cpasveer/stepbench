"""Level 4c, "moves": linkage synthesis -- the assembly file, the linkage solver, the generator, the grader."""
from __future__ import annotations

import json
import math
import os

import numpy as np
import pytest

from jointbench import families
from jointbench.geometry import Placement
from jointbench.moves import assembly as AS
from jointbench.moves import generate as G
from jointbench.moves import linkage as LK
from jointbench.moves import motion as MO
from jointbench.moves.grade import grade

SEED = 0


def _asm(**over):
    d = {"parts": [{"name": "p1", "file": "p1.step", "bolts_to": "A", "insert_direction": [0, 0, -1]},
                   {"name": "p2", "file": "p2.step", "bolts_to": "B", "insert_direction": [1, 0, 0]},
                   {"name": "p3", "file": "p3.step", "bolts_to": None, "insert_direction": [1, 0, 0]}],
         "pivots": [{"name": "J1", "joins": ["p1", "p3"], "point": [0, 0, 10], "axis": [0, 1, 0]},
                    {"name": "J2", "joins": ["p3", "p2"], "point": [0, 0, 50], "axis": [0, 1, 0]}],
         "drive": {"pivot": "J1", "range_deg": 60},
         "fasteners": [{"kind": "bolt", "size": "M5", "length": 30, "head_at": [0, 0, 0], "axis": [0, 1, 0],
                        "grip": 20, "held_by": "p1"}]}
    d.update(over)
    return d


def test_assembly_file_reads_and_names_what_is_wrong():
    a = AS.parse(_asm(), "A", "B")
    assert a.bodies() == ["A", "B", "p3"] and a.body_of("p1") == "A" and a.body_of("p3") == "p3"
    assert a.fasteners[0].label == "M5 x 30 bolt 1" and a.drive == "J1" and a.range_deg == 60
    with pytest.raises(AS.AssemblyError, match="already move as one body"):
        AS.parse(_asm(parts=_asm()["parts"] + [{"name": "p4", "file": "p4.step", "bolts_to": "A",
                                                "insert_direction": [1, 0, 0]}],
                      pivots=[{"name": "J1", "joins": ["p1", "p4"], "point": [0, 0, 0], "axis": [0, 1, 0]}]), "A", "B")
    with pytest.raises(AS.AssemblyError, match="drive names pivot"):
        AS.parse(_asm(drive={"pivot": "J9", "range_deg": 10}), "A", "B")
    with pytest.raises(AS.AssemblyError, match="not a catalogue length"):
        AS.parse(_asm(fasteners=[dict(_asm()["fasteners"][0], length=33)]), "A", "B")
    with pytest.raises(AS.AssemblyError, match="no part bolts to B"):
        AS.parse(_asm(parts=[_asm()["parts"][0], _asm()["parts"][2]],
                      pivots=[_asm()["pivots"][0]]), "A", "B")


def test_fastener_bodies_sit_where_stated():
    f = AS.parse(_asm(), "A", "B").fasteners[0]
    bb = f.screw().bounding_box()
    assert math.isclose(bb.min.Y, -5.0, abs_tol=1e-6) and math.isclose(bb.max.Y, 30.0, abs_tol=1e-6)
    nb = f.nut().bounding_box()
    assert math.isclose(nb.min.Y, 20.0, abs_tol=1e-6) and math.isclose(nb.max.Y, 24.7, abs_tol=1e-6)


def _fourbar_linkage(O2, C2, C4, O4, drive_axis=(0, 0, 1)):
    P = lambda xy: np.array([xy[0], xy[1], 0.0])  # noqa: E731
    return LK.build(["A", "B", "crank", "rocker"],
                    [("O2", "A", "crank", P(O2), np.array(drive_axis, float)),
                     ("C2", "crank", "B", P(C2), np.array([0, 0, 1.0])),
                     ("O4", "A", "rocker", P(O4), np.array([0, 0, 1.0])),
                     ("C4", "rocker", "B", P(C4), np.array([0, 0, 1.0]))], "O2")


def test_mobility_counts_free_motions():
    lk = _fourbar_linkage((0, 0), (5, 30), (45, 35), (40, 0))
    assert lk.mobility() == (1, 9)
    two = LK.build(["A", "B", "crank"], [("O2", "A", "crank", np.zeros(3), np.array([0, 0, 1.0])),
                                         ("C2", "crank", "B", np.array([5, 30, 0.0]), np.array([0, 0, 1.0]))], "O2")
    assert two.mobility()[0] == 2
    with pytest.raises(LK.LinkageError, match="parallel"):
        LK.build(["A", "B"], [("J", "A", "B", np.zeros(3), np.array([0, 0, 1.0])),
                              ("K", "A", "B", np.ones(3), np.array([0, 1, 1.0]))], "J")


def test_numeric_solve_matches_the_closed_form_fourbar():
    O2, C2, C4, O4 = (0.0, 0.0), (5.0, 30.0), (45.0, 35.0), (40.0, 0.0)
    lk = _fourbar_linkage(O2, C2, C4, O4)
    r2, r3, r4 = (math.dist(*x) for x in ((C2, O2), (C4, C2), (C4, O4)))
    th0 = math.atan2(C2[1], C2[0])
    mode = 1 if np.linalg.norm(LK.fourbar(O2, O4, r2, r3, r4, th0, 1)[1] - C4) < 1e-9 else -1
    path, stop = lk.sweep(-60.0, 0.5)
    assert stop is None
    for deg, q in path[::10]:
        A_, B_ = LK.fourbar(O2, O4, r2, r3, r4, th0 + math.radians(deg), mode)
        pl = lk.placement(q, lk.bodies.index("B"))
        assert np.allclose(pl.point((*C4, 0.0))[:2], B_, atol=1e-7)
        assert np.allclose(pl.point((*C2, 0.0))[:2], A_, atol=1e-7)
    # the drive pivot's axis pointing the other way turns the drive the other way
    lk2 = _fourbar_linkage(O2, C2, C4, O4, drive_axis=(0, 0, -1))
    p2, _ = lk2.sweep(60.0, 0.5)
    b1 = lk.placement(path[-1][1], lk.bodies.index("B")).point((*C4, 0.0))
    b2 = lk2.placement(p2[-1][1], lk2.bodies.index("B")).point((*C4, 0.0))
    assert np.allclose(b1, b2, atol=1e-7)


def test_a_linkage_that_cannot_reach_stops_the_motion():
    lk = _fourbar_linkage((0, 0), (0, 50), (20, 55), (40, 0))  # the coupler is the shortest link: a double rocker
    path, stop = lk.sweep(-170.0, 0.5)
    assert stop is not None and ("cannot close" in stop[1] or "dead point" in stop[1] or "jumps" in stop[1])
    assert 0 < abs(path[-1][0]) < 170


def test_hinge_miss_is_zero_for_poses_about_one_centre():
    d = {"psi": 0.0}
    F = G.Frame(d)
    start = Placement((10.0, 0.0, 50.0), ((1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0)))
    poses = [start] + [start.then(MO.about((0, 0, 20.0), tuple(F.n), a)) for a in (20, 50)]
    assert G.hinge_miss(poses, F) < 1e-6
    off = [start, poses[1], poses[2].then(MO.shift((0.0, 0.0, 15.0)))]
    # against a brute-force search over hinge centres in the plane
    b = [F.plane(p.origin) for p in off]
    angs = [0.0, math.radians(20), math.radians(50)]
    best = math.inf
    for cx in np.linspace(-60, 60, 241):
        for cz in np.linspace(-40, 80, 241):
            c = np.array([cx, cz])
            err = 0.0
            for k in (1, 2):
                t = angs[k]
                R = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
                err = max(err, float(np.linalg.norm(c + R @ (b[0] - c) - b[k])))
            best = min(best, err)
    assert abs(G.hinge_miss(off, F) - best) < 0.1 and best > 2
    # three poles: the shrinking grid against the same brute force
    four = off + [start.then(MO.about((30.0, 0, 0.0), tuple(F.n), 70))]
    b.append(F.plane(four[3].origin))
    angs.append(math.radians(70))
    best3 = math.inf
    for cx in np.linspace(-60, 80, 281):
        for cz in np.linspace(-60, 80, 281):
            c = np.array([cx, cz])
            err = 0.0
            for k in (1, 2, 3):
                t = angs[k]
                R = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
                err = max(err, float(np.linalg.norm(c + R @ (b[0] - c) - b[k])))
            best3 = min(best3, err)
    assert abs(G.hinge_miss(four, F) - best3) < 0.2


def test_generator_is_deterministic_and_hides_the_linkage():
    a = G.generate(SEED, check=False)
    b = G.generate(SEED, check=False)
    assert a[0].dumps() == b[0].dumps() and a[0].id == "moves-s0" and a[0].family == "moves"
    t, d = a[0], a[1]
    assert len(t.meta["motion"]["targets"]) == 3 and d["hinge_miss_mm"] >= G.HINGE_MISS_MIN
    text = t.dumps()
    for word in ("O2", "crank", "rocker", "theta0", "r2"):
        assert word not in text  # nothing of the hidden linkage is in task.json
    assert set(t.tolerances["required_checks"]) == {"SEAT_INSIDE", "CLEAN", "MANUFACTURING", "KEY_TURN"}
    assert t.meta["level_name"] == "4c" and t.tolerances["tool_length"] == 30.0
    assert all("spigot" not in itf.params for itf in t.parts.values())
    assert families.get("moves").levels[4] is G


@pytest.fixture(scope="module")
def folder(tmp_path_factory):
    task, d, solids, asm = G.generate(SEED, check=False)
    out = tmp_path_factory.mktemp("moves")
    return task, d, G.write_folder(task, d, solids, asm, str(out))


def test_folder_holds_what_a_model_needs_and_no_answer_outside_solution(folder):
    task, d, ws = folder
    have = set(os.listdir(ws))
    assert {"task.json", "README.md", "fixed_A.step", "fixed_B.step", "fixed_B_pose1.step", "fixed_B_pose2.step",
            "fixed_B_pose3.step", "_solution"} <= have
    readme = open(os.path.join(ws, "README.md")).read()
    assert "assembly.json" in readme and "MOBILITY" in readme and "POSES" in readme
    for word in ("crank", "rocker", "four-bar", "fourbar", "clevis", "axis: the line through"):
        assert word not in readme.lower()
    assert sorted(os.listdir(os.path.join(ws, "_solution", "broken"))) == sorted(n for n, *_ in G.broken_variants(d))


def test_reference_passes(folder):
    task, d, ws = folder
    rep = grade(task, os.path.join(ws, "_solution", "assembly.json"), manufacturing=False)
    assert rep.passed, rep.text()
    names = [c.name for c in rep.checks]
    for n in ("PIVOTS", "MOBILITY", "MOTION", "GAP", "POSES", "INTERLOCK", "PLAY", "INSERTION", "frame HOLES",
              "frame TOOL_ACCESS", "carrier TOOL_ACCESS"):
        assert n in names
    poses = next(c for c in rep.checks if c.name == "POSES").details["poses"]
    assert all(p["position_mm"] < 1e-3 and p["angle_deg"] < 1e-3 for p in poses)


@pytest.mark.parametrize("name", [n for n, *_ in G.broken_variants(G.generate(SEED, check=False)[1])])
def test_broken_copies_fail_on_the_named_check(folder, name):
    task, d, ws = folder
    bf = os.path.join(ws, "_solution", "broken", name)
    must = open(os.path.join(bf, "MUST_FAIL")).read().strip()
    rep = grade(task, os.path.join(bf, "assembly.json"), manufacturing=False)
    assert not rep.passed and must in rep.failed(), rep.text()


def test_missing_part_file_fails_cleanly(folder, tmp_path):
    task, d, ws = folder
    asm = json.load(open(os.path.join(ws, "_solution", "assembly.json")))
    p = tmp_path / "assembly.json"
    p.write_text(json.dumps(asm))
    rep = grade(task, str(p), manufacturing=False)
    assert not rep.passed and "frame FILE_VALID" in rep.failed()
