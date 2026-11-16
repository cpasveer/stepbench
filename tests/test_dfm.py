import json
import os

import build123d as bd
import pytest

from jointbench.dfm import available, check, load_process, selftest
from jointbench.dfm.__main__ import main
from jointbench.geometry import write_step


def _step(part, tmp_path, name="part.step"):
    p = os.path.join(tmp_path, name)
    write_step(part, p)
    return p


def test_selftest_every_row_as_expected():
    rows = selftest.run()
    bad = [r.label for r in rows if not r.ok]
    assert not bad, selftest.table(rows)


def test_every_rule_has_a_pass_and_a_fail_part():
    for pid in ("fdm", "moulding", "pbf"):
        rules = set(load_process(pid).rules)
        procs = [c for c in selftest.CASES if c[2] == pid]
        assert any(c[4] is None for c in procs)
        failing = {r for c in procs if c[4] for r in c[4].split("|")}
        assert rules <= failing, rules - failing


def test_process_files_state_unit_and_source():
    assert {"fdm", "moulding", "pbf", "milling"} <= set(available())
    for pid in available():
        proc = load_process(pid)
        for rule in proc.rules.values():
            assert rule.unit and rule.source and rule.kind in ("min", "max")


def test_report_has_every_rule_of_the_file_in_order(tmp_path):
    p = _step(selftest.plate(), tmp_path)
    for pid in ("fdm", "moulding"):
        rep = check(p, pid, (0, 0, 1))
        assert [x.id for x in rep.rules] == list(load_process(pid).rules)


def test_same_file_same_report(tmp_path):
    p = _step(selftest.ribbed(1.3), tmp_path)
    for pid in ("fdm", "moulding"):
        a = json.dumps(check(p, pid).to_json(), sort_keys=True)
        b = json.dumps(check(p, pid).to_json(), sort_keys=True)
        assert a == b


def test_failure_is_located_on_the_part(tmp_path):
    rep = check(_step(selftest.fin(0.7), tmp_path), "fdm", (0, 0, 1))
    wall = next(x for x in rep.rules if x.id == "FDM_WALL_MIN")
    assert not wall.ok
    x, y, z = wall.where[0]["point"]
    assert abs(x) <= 0.35 + 1e-6 and z >= 5.0  # on the fin, not the plate


def test_stated_direction_is_used_and_reported(tmp_path):
    p = _step(selftest.overhang(50), tmp_path)
    up = check(p, "fdm", (0, 0, 1))
    assert up.direction == [0, 0, 1] and up.direction_chosen == "as declared"
    assert "FDM_OVERHANG" in up.failed()
    free = check(p, "fdm")
    assert free.passed and free.direction_chosen.startswith("chosen by the checker")


def test_cli_json_and_exit_code(tmp_path, capsys):
    p = _step(selftest.holed(1.6), tmp_path)
    code = main([p, "--process", "fdm", "--direction", "0,0,1", "--json"])
    out = json.loads(capsys.readouterr().out)
    assert code == 1 and out["passed"] is False
    assert [r["rule"] for r in out["rules"] if r["status"] == "fail"] == ["FDM_HOLE_MIN"]


def test_milling_is_planned_not_built(tmp_path):
    p = _step(selftest.plate(), tmp_path)
    with pytest.raises(NotImplementedError):
        check(p, "milling")


def test_more_than_one_solid_is_refused(tmp_path):
    two = bd.Compound([bd.Box(5, 5, 5).solids()[0], bd.Pos(20, 0, 0) * bd.Box(5, 5, 5).solids()[0]])
    p = os.path.join(tmp_path, "two.step")
    bd.export_step(two, p)
    with pytest.raises(ValueError):
        check(p, "fdm")


def test_fields_file_has_mesh_and_one_field_per_rule(tmp_path):
    from jointbench.dfm import write_fields

    p = _step(selftest.bridge(12), tmp_path)
    rep = check(p, "fdm", (0, 0, 1))
    data = json.load(open(write_fields(rep, str(tmp_path))))
    assert data["format"] == "jointbench-dfm-fields" and data["direction"]["vector"] == [0, 0, 1]
    mesh = data["mesh"]
    n = len(mesh["triangles"])
    assert n == len(mesh["triangle_face"]) and mesh["vertices"] and mesh["edges"]
    assert max(max(t) for t in mesh["triangles"]) < len(mesh["vertices"])
    assert [f["rule"] for f in data["fields"]] == [x.id for x in rep.rules]
    for f in data["fields"]:
        assert len(f["values"]) == n and f["unit"]
    bridge = next(f for f in data["fields"] if f["rule"] == "FDM_BRIDGE")
    assert bridge["status"] == "fail" and bridge["fail"]
    assert all(bridge["values"][i] > 10 for i in bridge["fail"])
    assert any(m["type"] == "segment" and m["fail"] for m in bridge["marks"])


def test_one_picture_per_rule(tmp_path):
    from PIL import Image

    from jointbench.dfm import write_images

    p = _step(selftest.overhang(50), tmp_path)
    paths = write_images(check(p, "fdm", (0, 0, 1)), str(tmp_path / "img"))
    assert len(paths) == len(load_process("fdm").rules)
    assert Image.open(paths[0]).size == (1200, 760)


def test_powder_bed_has_no_direction_and_no_overhang_rules(tmp_path):
    rep = check(_step(selftest.overhang(60), tmp_path), "pbf")
    assert rep.direction == [] and rep.candidates == []
    assert not any("OVERHANG" in x.id or "BRIDGE" in x.id for x in rep.rules)
    assert rep.passed, rep.text()


def test_asking_for_some_rules_reports_only_those_and_same_values(tmp_path):
    p = _step(selftest.fin(0.7), tmp_path)
    full = check(p, "moulding", (0, 0, 1))
    some = check(p, "moulding", (0, 0, 1), rules=["IM_WALL_MIN", "IM_SHARP_CORNER"])
    assert [x.id for x in some.rules] == ["IM_WALL_MIN", "IM_SHARP_CORNER"]
    for x in some.rules:
        y = next(y for y in full.rules if y.id == x.id)
        assert (x.ok, x.value) == (y.ok, y.value)
    with pytest.raises(ValueError):
        check(p, "moulding", rules=["FDM_WALL_MIN"])


def test_compiled_inside_test_matches_exact(tmp_path, monkeypatch):
    from jointbench.dfm import _PARTS, geom

    if not geom.fast_available():
        pytest.skip("compiled helper not built")
    p = _step(selftest.bridge(12), tmp_path)
    part = geom.Part(geom.load(p), 1.5)
    rng = __import__("numpy").random.default_rng(0)
    lo, hi = part.V.min(axis=0) - 1, part.V.max(axis=0) + 1
    pts = lo + rng.random((3000, 3)) * (hi - lo)
    assert (part.inside_many(pts) == [part.inside(q) for q in pts]).all()
    fast = check(p, "fdm", (0, 0, 1))
    monkeypatch.setenv("JOINTBENCH_NATIVE", "0")
    _PARTS.clear()
    slow = check(p, "fdm", (0, 0, 1))
    assert fast.to_json() == slow.to_json()


def test_material_estimate_shell_and_infill(tmp_path):
    from jointbench.dfm.material import material

    p = _step(selftest.plate(40, 30, 10), tmp_path)  # 12 cm3 solid block
    m = material(p, "fdm", (0, 0, 1))
    assert abs(m["solid_volume_cm3"] - 12.0) < 1e-6
    # skins: 2 x 12 cm2 x 0.8 mm top/bottom + 14 cm2 x 1.2 mm sides = 3.6 cm3; 20 % of the rest
    assert abs(m["shell_cm3"] - 3.6) < 0.1
    assert abs(m["used_cm3"] - (3.6 + 0.2 * 8.4)) < 0.1
    thin = material(_step(selftest.plate(40, 30, 1.0), tmp_path, "thin.step"), "fdm", (0, 0, 1))
    assert abs(thin["share_of_solid"] - 1.0) < 0.02  # a 1 mm plate is all skin
    assert material(p, "pbf")["grams"] == pytest.approx(12.0 * 1.01)


def test_fields_v2_corner_values_probes_and_probe_point(tmp_path):
    from jointbench.dfm import probe, write_fields

    p = _step(selftest.boss(8), tmp_path)
    data = json.load(open(write_fields(check(p, "pbf"), str(tmp_path))))
    assert data["version"] == 2 and data["sampling"]["verdict_spacing_mm"] == 1.5
    sec = next(f for f in data["fields"] if f["id"] == "section_thickness")
    assert len(sec["corner_values"]) == len(data["mesh"]["triangles"])
    assert all(v is not None for v in sec["values"])  # complete map, passing regions included
    assert {q["label"] for q in sec["probes"]} >= {"worst", "lowest", "median", "highest"}
    at = probe(p, (0, 0, 1.0))  # under the boss: 10 mm straight through, 8 mm section
    assert at["wall_thickness"] == 10.0 and at["section_thickness"] == 8.0 and at["moved_by"] == 1.0


def test_ray_fallback_matches_intersector(tmp_path):
    import numpy as np

    from jointbench.dfm import geom

    class Failing:  # stands in for an intersector that raises, as OCP did on a valid part
        def Perform(self, *a):
            raise RuntimeError("Standard_Failure")

    part = geom.Part(geom.load(_step(selftest.boss(8), tmp_path)), 1.5)
    for i in range(0, len(part.P), 97):
        for s in (-1, 1):
            d = s * part.N[i]
            a = part.ray(part.P[i], d)
            b = geom._ray_exact(Failing(), part.faces, part.P[i], d, geom.RAY_START, part.inside)
            assert (np.isinf(a) and np.isinf(b)) or abs(a - b) < 1e-6


def test_kernel_failure_is_a_measurement_error(tmp_path, monkeypatch):
    from jointbench.dfm import MeasurementError, pbf

    def broken(part, proc):
        raise RuntimeError("Standard_Failure")

    monkeypatch.setattr(pbf, "independent", broken)
    with pytest.raises(MeasurementError, match="pbf, direction-free rules"):
        check(_step(selftest.boss(5), tmp_path), "pbf")


def test_declared_direction_is_used_and_reported(tmp_path):
    from jointbench.dfm import declaration

    (tmp_path / "build.json").write_text('{"process": "fdm", "build_direction": [0, 1, 0]}')
    proc, d = declaration(str(tmp_path))
    assert proc == "fdm" and d == [0.0, 1.0, 0.0]
    p = _step(selftest.overhang(50), tmp_path)
    assert check(p, proc, d).passed and check(p, proc, d).direction_chosen == "as declared"
    assert not check(p, "fdm", (0, 0, 1)).passed
    assert check(p, "fdm").direction_chosen.startswith("chosen by the checker")
    (tmp_path / "bad.json").write_text('{"process": "fdm", "build_direction": [0, 0, 0]}')
    with pytest.raises(ValueError):
        declaration(str(tmp_path / "bad.json"))
    assert declaration(str(tmp_path / "missing")) == (None, None)


def test_support_allowed_counts_material_and_unremovable_support_fails(tmp_path):
    from jointbench.dfm.material import material

    p = _step(selftest.shelf(), tmp_path)
    assert not check(p, "fdm", (0, 0, 1)).passed  # without support: overhang fails
    ok = check(p, "fdm", (0, 0, 1), support="allowed")
    assert ok.passed and ok.support == "allowed"
    ov = next(x for x in ok.rules if x.id == "FDM_OVERHANG")
    assert not ov.ok and not ov.required  # reported, not required
    m = material(p, "fdm", (0, 0, 1), support="allowed")
    assert m["support_cm3"] > 0 and m["support_counted"]
    assert abs(m["used_cm3"] - (m["shell_cm3"] + m["infill_cm3"] + m["support_cm3"])) < 1e-9
    lip = check(_step(selftest.tight_lip(2.0), tmp_path, "lip.step"), "fdm", (0, 0, 1), support="allowed")
    assert lip.failed() and "FDM_SUPPORT_REMOVABLE" in [x.id for x in lip.rules if not x.ok and x.required]


def test_failures_say_which_faces_they_lie_between(tmp_path):
    # gap: two 2 x 16 x 8 fins 0.4 mm apart -- between their facing 16 x 8 faces, across x
    r = check(_step(selftest.slotted(0.4), tmp_path), "fdm", (0, 0, 1))
    gap = next(x for x in r.rules if x.id == "FDM_GAP_MIN")
    b = gap.details["between"]
    assert b["face_types"] == ["plane face 16.0 x 8.0 mm"] * 2
    import re

    assert abs(abs(b["direction"][0]) - 1) < 1e-6
    assert 15 <= int(re.search(r"strip about (\d+) mm", b["extent"]).group(1)) <= 18  # the fins are 16 mm long
    assert "plane face 16.0 x 8.0 mm" in gap.reason and gap.where[0]["point"]  # the point stays
    # wall: a 0.7 mm fin -- between its two 16 x 8 sides
    w = next(x for x in check(_step(selftest.fin(0.7), tmp_path, "f.step"), "fdm", (0, 0, 1)).rules
             if x.id == "FDM_WALL_MIN")
    assert w.details["between"]["face_types"] == ["plane face 16.0 x 10.0 mm"] * 2
    # transition: a 2 mm plate meeting an 8 mm boss
    t = next(x for x in check(_step(selftest.boss(8), tmp_path, "b.step"), "pbf").rules if x.id == "PBF_TRANSITION")
    assert "section 2.00 mm" in t.reason and "plane face 8.0 x 8.0 mm" in t.reason


def _keel(rise_deg: float):
    """A wall on the bed with a V-shaped arm sticking out of it: the arm's two underside faces lean
    30 deg from vertical (within the 45 deg limit) and meet in a downward edge (a keel) that rises
    from the wall at ``rise_deg`` above horizontal."""
    import math

    wall = bd.Box(10, 60, 80, align=(bd.Align.MAX, bd.Align.CENTER, bd.Align.MIN))
    h, w = 17.32, 10.0  # faces 30 deg from vertical
    tri = bd.Plane.YZ * bd.Polygon((0, 0), (w, h), (-w, h), align=None)
    arm = bd.extrude(tri, 30)
    arm = arm.rotate(bd.Axis((0, 0, 0), (0, 1, 0)), -rise_deg)
    # end the arm in a level top face, so only the keel and the two underside faces face down
    arm = arm & bd.Box(100, 100, 40, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX)).moved(bd.Location((0, 0, 15)))
    arm = arm.moved(bd.Location((-1, 0, 20)))
    arm = arm & bd.Box(100, 100, 100, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((-1, 0, 0)))  # nothing behind the wall
    return wall + arm


def test_downward_edge_steeper_than_the_limit_needs_support(tmp_path):
    flat = check(_step(_keel(0), tmp_path), "fdm", (0, 0, 1))
    r = {x.id: x for x in flat.rules}["FDM_OVERHANG"]
    assert not r.ok and r.value > 80 and "downward edge" in r.reason
    assert r.details["area_over_limit_mm2"] == 0.0  # every face passes on its own: only the edge fails


def test_downward_edge_within_the_limit_passes(tmp_path):
    steep = check(_step(_keel(60), tmp_path), "fdm", (0, 0, 1))
    r = {x.id: x for x in steep.rules}["FDM_OVERHANG"]
    assert r.details["keel_edges"] >= 1 and r.details["steepest_keel_deg"] <= 45.0



def _keel_beam(span: float):
    """Two walls on the bed, ``span`` mm apart, joined at mid height by a beam with a V-shaped
    underside: its two faces lean 30 deg from vertical and meet in a level keel that runs from one
    wall to the other."""
    walls = [bd.Box(10, 60, 80, align=(bd.Align.MAX, bd.Align.CENTER, bd.Align.MIN)),
             bd.Pos(span, 0, 0) * bd.Box(10, 60, 80, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))]
    h, w = 17.32, 10.0
    beam = bd.extrude(bd.Plane.YZ * bd.Polygon((0, 0), (w, h), (-w, h), align=None), span)
    return walls[0] + walls[1] + beam.moved(bd.Location((0, 0, 20)))


def test_flat_keel_that_starts_in_mid_air_still_fails(tmp_path):
    # the arm leans down away from the wall: its keel's lowest point is the free tip
    r = {x.id: x for x in check(_step(_keel(-10), tmp_path), "fdm", (0, 0, 1)).rules}["FDM_OVERHANG"]
    assert not r.ok and r.value > 63.5 and "starts in mid-air" in r.reason
    assert r.details["flat_keel_runs"].get("mid-air", 0) >= 1


def test_flat_keel_between_two_supports_is_a_bridge(tmp_path):
    r = {x.id: x for x in check(_step(_keel_beam(8), tmp_path), "fdm", (0, 0, 1)).rules}["FDM_OVERHANG"]
    assert r.ok and r.details["steepest_keel_deg"] > 80 and r.details["flat_keel_runs"] == {"bridge": 1}, r.reason
    long = {x.id: x for x in check(_step(_keel_beam(20), tmp_path, "l.step"), "fdm", (0, 0, 1)).rules}
    o = long["FDM_OVERHANG"]
    assert not o.ok and "bridge limit" in o.reason and o.details["flat_keel_runs"] == {"long bridge": 1}


def test_flat_keel_growing_from_printed_material_passes(tmp_path):
    # Sonnet flange L3 try 6: a curved keel leaves a wall nearly level and steepens as it rises
    r = {x.id: x for x in check(_step(_keel(8), tmp_path), "fdm", (0, 0, 1)).rules}["FDM_OVERHANG"]
    assert r.details["steepest_keel_deg"] > 63.5 and r.details["flat_keel_runs"] == {"grows": 1}
    assert r.ok, r.reason

# ---- FDM_WALL_MIN: a wall is two roughly opposite faces, not a corner -------


def _corner_reading(angle_deg, x):
    """A prism whose edge along z has an inside angle ``angle_deg``. A wall
    sample is put on the face y = 0, x mm from that edge; returns the ray's
    reading there and whether the wall rule takes it for a corner."""
    import math

    import numpy as np

    from jointbench.dfm import load_process
    from jointbench.dfm.fdm import _corner_walls
    from jointbench.dfm.geom import Part

    a = math.radians(angle_deg)
    tri = bd.Polygon((0, 0), (20, 0), (20 * math.cos(a), 20 * math.sin(a)), align=None)
    part = Part(bd.extrude(tri, 10), 1.5)
    k = int(np.flatnonzero(np.abs(part.N[:, 1] + 1) < 1e-9)[0])  # a sample on the face y = 0
    part.P[k] = (x, 0.0, 5.0)
    t = np.full(len(part.P), np.inf)
    t[k] = part.ray(part.P[k], -part.N[k])
    corner = _corner_walls(part, t, 0.8, load_process("fdm").settings["wall_open_corner"])
    return t[k], bool(corner[k])


def test_wall_ray_across_a_near_square_corner_is_no_wall():
    # Fable pipe L2 try 1: 0.38 mm read 0.016 mm from an 88 deg edge
    t, corner = _corner_reading(88.0, 0.0155)
    assert t < 0.8 and corner


def test_wall_ray_across_a_sharp_wedge_is_still_a_wall():
    t, corner = _corner_reading(20.0, 1.0)  # a 20 deg wedge, 0.36 mm thick 1 mm from its tip
    assert t < 0.8 and not corner
    t, corner = _corner_reading(40.0, 0.5)  # a 40 deg edge (seen on Sonnet's flanges): still a wall
    assert t < 0.8 and not corner


def test_thin_plate_still_fails_wall_min(tmp_path):
    rep = check(_step(selftest.fin(0.4), tmp_path), "fdm", direction=(0, 0, 1))
    w = next(x for x in rep.rules if x.id == "FDM_WALL_MIN")
    assert not w.ok and abs(w.value - 0.4) < 1e-3

