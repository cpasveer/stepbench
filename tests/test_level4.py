"""Level 4a, "assembles": two parts bolted to each other, put in along straight paths."""
from __future__ import annotations

import json
import os
import random

import build123d as bd
import numpy as np
import pytest

from jointbench.assemble import generate as G
from jointbench.assemble import joint as J
from jointbench.assemble.grade import grade
from jointbench.assemble.sweep import collisions
from jointbench.catalogue import BOLTS, NUTS, nut_side, plastic_allowables
from jointbench.requirements import draw_stated_load


# ---- the sweep


def test_sweep_sliding_is_free_running_into_is_not():
    part = bd.Pos(0, 0, 5) * bd.Box(10, 10, 10) - bd.Pos(0, 0, 5) * bd.Cylinder(2, 20)
    floor = bd.Pos(0, 0, -5) * bd.Box(50, 50, 10)
    wall = bd.Pos(20, 0, 5) * bd.Box(4, 50, 10)
    assert collisions(part, (0, 0, -1), {"floor": floor, "wall": wall}) == {"floor": 0.0, "wall": 0.0}
    assert collisions(part, (1, 0, 0), {"floor": floor, "wall": wall}) == {"floor": 0.0, "wall": 0.0}  # slides in from -x
    assert collisions(part, (-1, 0, 0), {"floor": floor, "wall": wall})["wall"] >= 400.0  # through the wall


def test_sweep_of_a_round_member_crossed_sideways_is_exact():
    pin = bd.Pos(0, 0, 15) * bd.Rot(0, 90, 0) * bd.Cylinder(2, 10)
    plate = bd.Pos(0, 10, 15) * bd.Box(20, 2, 20)
    assert collisions(pin, (0, -1, 0), {"p": plate})["p"] == pytest.approx(80.0, abs=0.01)  # 4 x 10 x 2
    assert collisions(pin, (0, 1, 0), {"p": plate})["p"] == 0.0  # moving away from it


def _slot():
    """A 4 mm wall with a 6 x 22 mm slot, and a 20 x 20 x 3 mm plate that must end flat below it."""
    wall = bd.Pos(0, 0, 0) * bd.Box(60, 60, 4) - bd.Box(6, 22, 6)
    plate = bd.Pos(0, 0, -20) * bd.Box(20, 20, 3)
    return wall, plate


def test_a_plate_passes_a_narrow_slot_only_turned_edge_on():
    from jointbench.assemble.sweep import path_collisions

    wall, plate = _slot()
    straight, _ = path_collisions(plate, (0, 0, -1), [], {"wall": wall})
    assert straight["wall"] > 100  # flat, it cannot pass
    # edge-on down through the slot, then turned flat once below the wall
    turned, rows = path_collisions(plate, (0, 0, -1), [{"turn": {"point": [0, 0, -20], "axis": [0, 1, 0], "degrees": 90}}],
                                   {"wall": wall})
    assert turned["wall"] == 0.0 and len(rows) == 2
    # turning while still in the slot catches the wall
    early, _ = path_collisions(bd.Pos(0, 0, 20) * plate, (0, 0, -1),
                               [{"turn": {"point": [0, 0, 0], "axis": [0, 1, 0], "degrees": 90}}], {"wall": wall})
    assert early["wall"] > 1.0


def test_a_path_is_checked_in_the_assembly_file(tmp_path):
    bad = {"parts": [{"name": "part1", "file": "part1.step", "bolts_to": "A", "insert_direction": [0, 0, -1],
                      "path": [{"spin": 3}]},
                     {"name": "part2", "file": "part2.step", "bolts_to": "B", "insert_direction": [-1, 0, 0]}],
           "joint": {"bolt": "M5", "head_on": "part2", "axis": [0, 0, 1], "positions": [[0, -10, 20], [0, 10, 20]]}}
    p = tmp_path / "a.json"
    p.write_text(json.dumps(bad))
    with pytest.raises(J.AssemblyError, match="path step 1"):
        J.load(str(p), ["A", "B"])
    bad["parts"][0]["path"] = [{"turn": {"point": [0, 0, 0], "axis": [0, 1, 0], "degrees": 30}}, {"move": [0, 0, -2]}]
    p.write_text(json.dumps(bad))
    a = J.load(str(p), ["A", "B"])
    assert a.parts[0].path[0]["turn"]["degrees"] == 30 and a.parts[0].path[1] == {"move": [0.0, 0.0, -2.0]}


# ---- the catalogue


def test_nut_side_entries_and_the_plastic_allowables():
    for sz in NUTS:
        n = BOLTS[nut_side(sz)]
        assert n.d == BOLTS[sz].d and n.clear_coarse == BOLTS[sz].clear_coarse and n.head_d == NUTS[sz].e
    al = plastic_allowables()
    assert al["bearing_MPa"] == 20.0 and al["shear_MPa"] == pytest.approx(40 / np.sqrt(3) / 2, abs=1e-3)
    assert "PLA" in al["material"] and "across layers" in al["source"]


# ---- the assembly file


def _asm(tmp_path, **over):
    d = {"parts": [{"name": "part1", "file": "part1.step", "bolts_to": "A", "insert_direction": [0, 0, -1]},
                   {"name": "part2", "file": "part2.step", "bolts_to": "B", "insert_direction": [-1, 0, 0]}],
         "joint": {"bolt": "M5", "head_on": "part2", "axis": [0, 0, 1], "positions": [[0, -10, 20], [0, 10, 20]]}}
    for k, v in over.items():
        if k in d["joint"]:
            d["joint"][k] = v
        else:
            d[k] = v
    p = tmp_path / "assembly.json"
    p.write_text(json.dumps(d))
    return str(p)


def test_assembly_file_is_checked(tmp_path):
    a = J.load(_asm(tmp_path), ["A", "B"])
    assert a.joint.bolt == "M5" and np.allclose(a.joint.centre, (0, 0, 20))
    for over, words in [({"bolt": "M7"}, "not allowed"),
                        ({"positions": [[0, 0, 20]]}, "at least 2"),
                        ({"positions": [[0, -10, 20], [0, 10, 21]]}, "one face"),
                        ({"positions": [[0, -2, 20], [0, 2, 20]]}, "apart"),
                        ({"head_on": "part3"}, "head_on")]:
        with pytest.raises(J.AssemblyError, match=words):
            J.load(_asm(tmp_path, **over), ["A", "B"])


def test_the_task_limits_the_joint_bolts(tmp_path):
    allowed = {"sizes": ["M3", "M4"], "max_bolts": 2}
    assert J.load(_asm(tmp_path, bolt="M4"), ["A", "B"], allowed).joint.bolt == "M4"
    with pytest.raises(J.AssemblyError, match="not allowed in this task"):
        J.load(_asm(tmp_path), ["A", "B"], allowed)  # M5
    with pytest.raises(J.AssemblyError, match="at most 2"):
        J.load(_asm(tmp_path, bolt="M3", positions=[[0, -10, 20], [0, 0, 20], [0, 10, 20]]), ["A", "B"], allowed)


# ---- the force in the joint bolts


def _joint(positions, axis=(0, 0, 1)):
    return J.Assembly([J.PartEntry("p1", "", "A", np.array([0, 0, -1.0])), J.PartEntry("p2", "", "B", np.array([-1, 0, 0.0]))],
                      J.Joint("M5", "p2", np.array(axis, float), [np.array(p, float) for p in positions]))


def test_bolt_forces_follow_the_bolt_group_rules():
    # two bolts on the y axis, a 20 x 40 mm contact face round them
    asm = _joint([(0, -10, 0), (0, 10, 0)])
    face = np.array([(x, y, 0) for x in (-10, 10) for y in (-20, 20)])
    al = plastic_allowables()
    # a load at the joint's centre: no turning; pulled along the axis each bolt takes half, and the nut's
    # small bearing ring decides (M5: nut face 6.9 mm, hole 5.8 mm)
    r = J.bolt_forces(asm, face, np.zeros(3), 100.0, True)
    ring = np.pi / 4 * (6.9 ** 2 - 5.8 ** 2)
    w = max(r["bolts"], key=lambda b: b["utilisation"])
    assert w["utilisation"] == pytest.approx(50 / (al["bearing_MPa"] * ring), rel=0.002) and "under the nut" in w["mode"]
    # far out along x: the turning load tips the joint about the 10 mm edge (pulled straight up:
    # 10/2 + 10 x 100 / 2 / 10 = 55 N each); the worst direction is at least that bad
    r = J.bolt_forces(asm, face, np.array([100.0, 0, 0]), 10.0, True)
    assert max(b["utilisation"] for b in r["bolts"]) >= 0.999 * 55 / (al["bearing_MPa"] * ring)
    r2 = J.bolt_forces(asm, face, np.array([100.0, 0, 0]), 10.0, False)
    assert max(b["utilisation"] for b in r2["bolts"]) == pytest.approx(max(b["utilisation"] for b in r["bolts"]), rel=2e-3)


def test_a_hole_near_the_edge_tears_out():
    asm = _joint([(0, -10, 0), (0, 10, 0)])
    face = np.array([(x, y, 0) for x in (-10, 10) for y in (-20, 20)])
    lp = np.array([60.0, 0, 50])
    far = J.bolt_forces(asm, face, lp, 14.7, True)
    sides = J.default_sides(asm)
    for g in sides["nut"]:
        g["edge"] = np.full(J.EDGE_DIRECTIONS, g["hole"] / 2 + 0.5)  # 0.5 mm of plastic to the edge all round
    near = J.bolt_forces(asm, face, lp, 14.7, True, sides=sides)
    w = max(near["bolts"], key=lambda b: b["utilisation"])
    assert w["utilisation"] > 2 * max(b["utilisation"] for b in far["bolts"]) and w["mode"] == "tear-out to the free edge"


def test_closer_bolts_and_a_smaller_face_carry_more():
    face_big = np.array([(x, y, 0) for x in (-10, 10) for y in (-20, 20)])
    face_small = np.array([(x, y, 0) for x in (-4, 4) for y in (-6, 6)])
    wide = J.bolt_forces(_joint([(0, -10, 0), (0, 10, 0)]), face_big, np.array([60.0, 0, 50]), 14.7, True)
    tight = J.bolt_forces(_joint([(0, -3, 0), (0, 3, 0)]), face_small, np.array([60.0, 0, 50]), 14.7, True)
    u = lambda r: max(b["utilisation"] for b in r["bolts"])
    assert u(tight) > 2 * u(wide)


def test_edge_distance_is_measured_on_the_part():
    from jointbench.assemble.grade import edge_distances

    plate = bd.Pos(0, 0, 2) * bd.Box(40, 20, 4) - bd.Pos(5, 0, 2) * bd.Cylinder(1.7, 6)
    e = edge_distances(plate, (5, 0, 0), np.array([0, 0, 1.0]), 2.0, 4)  # four directions in the joint frame
    assert sorted(np.round(e, 3)) == [10.0, 10.0, 15.0, 25.0]


def test_shear_keys_take_the_slide_off_the_bolts():
    asm = _joint([(0, -10, 0), (0, 10, 0)])
    face = np.array([(x, y, 0) for x in (-10, 10) for y in (-20, 20)])
    lp = np.array([60.0, 0, 50])
    bare = J.bolt_forces(asm, face, lp, 14.7, True)
    one = J.bolt_forces(asm, face, lp, 14.7, True, [{"normal": [1, 0, 0], "arm": 20.0}])
    both = J.bolt_forces(asm, face, lp, 14.7, True, [{"normal": [1, 0, 0], "arm": 20.0}, {"normal": [0, 1, 0], "arm": 20.0}])
    s = lambda r: max(b["by_mode"]["nut bearing on the hole wall"] for b in r["bolts"])
    assert s(both) == 0.0 and s(one) < s(bare) and one["keys"]["twist_taken"]


# ---- a whole task: the reference passes, broken copies fail where they should


@pytest.fixture(scope="module")
def made(tmp_path_factory):
    # the generator's own accepted draw: since KEY_TURN, a hand-picked draw's reference may not pass
    task, d, _ = G.generate(0)
    L = G.layout(d, d["T"])
    return task, d, L, tmp_path_factory.mktemp("l4")


def test_one_part_reaching_both_mounts_cannot_get_into_the_box(made):
    """The reference's two parts fused into one: through the opening at any height, then up or down into
    place, it hits the box; from any other side it meets a wall."""
    from jointbench.assemble.sweep import path_collisions

    task, d, L, tmp = made
    p1, p2, _ = G.build_reference(task, L)
    one = p1.fuse(p2).clean().solids()[0]
    bodies = dict(task.fixed_solids(0), **task.obstacle_solids(0))
    for dz in (-60.0, -30.0, -10.0, 0.0, 10.0, 30.0, 60.0):
        hits, _ = path_collisions(one, (1, 0, 0), [{"move": [0, 0, dz]}], bodies)
        assert max(hits.values()) > 1.0, dz
    for dv in [(-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1), (1, 0, -1), (1, 0, 1)]:
        assert max(collisions(one, np.array(dv, float), bodies).values()) > 1.0, dv


def test_each_reference_part_fits_the_opening_but_not_much_more(made):
    task, d, L, tmp = made
    op = task.meta["opening"]
    p1, p2, _ = G.build_reference(task, L)
    for p in (p1, p2):
        bb = p.bounding_box()
        assert bb.size.Y < op["y"][1] - op["y"][0] and bb.size.Z < op["z"][1] - op["z"][0]
    assert op["z"][1] - op["z"][0] < L["H"] - 10  # much lower than the box: nothing reaching floor to ceiling fits


def test_reference_passes_and_broken_copies_fail(made):
    task, d, L, tmp = made
    ref = G.build_reference(task, L)
    rep = grade(task, G.write_answer(str(tmp / "ref"), *ref), stiffness=False)
    assert rep.passed, rep.text()
    mu = rep.info["material_used"]  # the score: printed plastic of both parts, lighter is better
    assert set(mu["parts"]) == {"part1", "part2"} and mu["grams"] == pytest.approx(
        sum(m["grams"] for m in mu["parts"].values()), abs=0.02) and 0 < mu["used_cm3"] < rep.info["volume"] / 1000
    names = [c.name for c in rep.checks]
    for n in ("ASSEMBLY_FILE", "part1 HOLES", "part2 SEATING", "part1 TOOL_ACCESS", "part2 FDM_OVERHANG",
              "INSERTION", "JOINT_ROOM", "JOINT_REACH", "JOINT_BOLTS"):
        assert n in names
    for name, (q1, q2, a, must, kg) in G.broken(task, d).items():
        r = grade(G.with_load(task, kg) if kg else task, G.write_answer(str(tmp / name), q1, q2, a), stiffness=False)
        assert not r.passed and must in r.failed(), (name, r.text())
        why = next(c for c in r.checks if c.name == must).reason
        assert {"wrong-order": "runs into part2", "too-big-for-the-opening": "runs into obstacle wall_near",
                "key-cannot-reach": "no key reaches it from outside", "holes-do-not-line-up": "off the bolt axis",
                "thin-flange-edge-holes": "tear-out"}[name] in why, why


def test_grading_is_deterministic(made):
    task, d, L, tmp = made
    p = G.write_answer(str(tmp / "det"), *G.build_reference(task, L))
    a = grade(task, p, stiffness=False).to_json()
    b = grade(task, p, stiffness=False).to_json()
    assert a == b


def test_task_json_round_trips_and_is_a_level_4_family(made, tmp_path):
    from jointbench import families
    from jointbench.task import Task

    task, *_ = made
    p = tmp_path / "task.json"
    p.write_text(task.dumps())
    t = Task.load(str(p))
    assert t.level == 4 and t.family == "assemble" and families.of(t).levels[4] is G
    assert t.id == "L4a-s0" and t.meta["level_label"] == "4a" and t.meta["level_name"] == "assembles"
    assert t.stiffness["mode"] == "stated" and len(t.obstacles_at(0)) == 15
    from jointbench.assemble.brief import brief

    text = brief(t)
    for word in ("Level 4a, assembles", "assembly.json", "insert_direction", "JOINT_BOLTS", "INSERTION", "JOINT_REACH", "wall_near_above",
                 "one opening", "uses the least material wins"):
        assert word in text
    # the brief states requirements, never the answer
    assert "[0.0, 0.0, -1.0]" not in text and "through the opening onto A" not in text


def test_the_view_shows_each_part_on_its_way_in(made, tmp_path):
    from jointbench.assemble.view import write

    task, d, L, tmp = made
    ans = tmp_path / "ans"
    G.write_answer(str(ans), *G.build_reference(task, L))
    out = write(task, str(ans), str(tmp_path / "view"))
    w = json.loads((tmp_path / "view" / "ways.json").read_text())
    assert w["order"] == ["part1", "part2"] and w["opening"] == task.meta["opening"]
    for name in ("box", "fixed_A", "fixed_B", "part1", "part2"):
        assert os.path.getsize(os.path.join(out, name + ".stl")) > 0
    for way in w["ways"]:
        assert np.allclose(way["frames"][-1], np.eye(4).ravel(), atol=1e-6)  # the last pose is the part in place
        start = np.array(way["trail"][0])
        assert start[0] < task.meta["opening"]["wall_x"]  # it starts outside the box, beyond the opening
