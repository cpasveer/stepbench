import json
import os

from jointbench.dfm.rules import load_file
from jointbench.geometry import write_step
from jointbench.quality import RULES, check, selftest, write_fields
from jointbench.quality.__main__ import main


def _step(part, tmp_path, name="part.step"):
    p = os.path.join(tmp_path, name)
    write_step(part, p)
    return p


def test_selftest_every_row_as_expected():
    rows = selftest.run()
    assert all(r.ok for r in rows), selftest.table(rows)


def test_every_rule_has_a_pass_and_a_fail_part():
    rules = set(load_file(RULES).rules)
    failing = {m for c in selftest.CASES for m in c[2]}
    assert rules <= failing, rules - failing
    assert any(not c[2] for c in selftest.CASES)


def test_owner_example_fails_on_step_and_wedge(tmp_path):
    rep = check(_step(selftest.misaligned(), tmp_path))
    assert {"Q_SMALL_STEP", "Q_THIN_WEDGE"} <= set(rep.failed())
    step = next(x for x in rep.rules if x.id == "Q_SMALL_STEP")
    assert 0 < step.value <= 0.11 and step.where and step.where[0]["point"]


def test_clean_parts_pass_and_report_info(tmp_path):
    rep = check(_step(selftest.flush(), tmp_path))
    assert rep.passed, rep.text()
    assert rep.info["faces"] == 6 and rep.info["share_of_faces"] == {"plane": 1.0}


def test_same_file_same_report(tmp_path):
    p = _step(selftest.misaligned(), tmp_path)
    assert json.dumps(check(p).to_json(), sort_keys=True) == json.dumps(check(p).to_json(), sort_keys=True)


def test_fields_have_one_value_per_triangle(tmp_path):
    rep = check(_step(selftest.stepped(0.12), tmp_path))
    data = json.load(open(write_fields(rep, str(tmp_path))))
    n = len(data["mesh"]["triangles"])
    assert [f["rule"] for f in data["fields"]] == [x.id for x in rep.rules]
    assert all(len(f["values"]) == n for f in data["fields"])
    step = next(f for f in data["fields"] if f["rule"] == "Q_SMALL_STEP")
    assert step["fail"] and step["marks"]


def test_cli_json_and_exit_code(tmp_path, capsys):
    p = _step(selftest.wedge(4), tmp_path)
    code = main([p, "--json"])
    out = json.loads(capsys.readouterr().out)
    assert code == 1 and [r["rule"] for r in out["rules"] if r["status"] == "fail"] == ["Q_THIN_WEDGE"]


def test_bar_run_into_plates_is_a_stub_and_trimmed_bar_is_not(tmp_path):
    bad = check(_step(selftest.bar_joint(50, through=True), tmp_path, "a.step"))
    good = check(_step(selftest.bar_joint(50, through=False), tmp_path, "b.step"))
    assert bad.failed() == ["Q_STUB"] and 0 < next(x for x in bad.rules if x.id == "Q_STUB").value < 5
    assert good.passed, good.text()
    assert good.info["inside_corners_sharp"] > 0 and good.info["inside_corners_filleted"] == 0


def test_rim_flush_with_plate_edge_is_not_a_stub(tmp_path):
    assert check(_step(selftest.rim(), tmp_path)).passed


def test_counterbore_in_a_curved_surface_is_no_stub():
    """A counterbore cut into a saddle's curved top, with a pad nearby whose
    underside faces down: the curved top dips below the counterbore floor's
    plane, but it is no member's side (found on Sonnet's pipe bracket, B1)."""
    import build123d as bd

    from jointbench.quality import check_solid

    R = 38.0
    sad = bd.Rot(0, 90, 0) * (bd.Cylinder(R + 8, 100) - bd.Cylinder(R, 102))
    sad = sad & (bd.Pos(0, 0, R) * bd.Box(110, 60, 2 * R))
    body = sad + bd.Pos(0, 20, R + 12) * bd.Box(30, 20, 12)
    body = body - bd.Pos(-30, 0, R + 8) * bd.Cylinder(5.6, 6.0) - bd.Pos(-30, 0, R) * bd.Cylinder(3.3, 30)
    rep = check_solid(body.solids()[0])
    stub = next(x for x in rep.rules if x.id == "Q_STUB")
    assert stub.ok, stub.reason


# ---- tangent cusps (Q_THIN_WEDGE) -------------------------------------------
# A fillet that blends tangent into a face, and ends on a third face, leaves a
# cusp (a 0 deg corner) in that third face's outline. The material there is
# no wedge. A cusp between a round and a flat that meet at a knife edge is.

def _min_box(x, y, z, sx, sy, sz):
    import build123d as bd

    return bd.Pos(x, y, z) * bd.Box(sx, sy, sz, align=(bd.Align.MIN,) * 3)


def _round_edges_at(s, z, r):
    import build123d as bd

    es = s.edges().filter_by(bd.Axis.Y).filter_by(lambda e: abs(e.center().Z - z) < 1e-6 and abs(e.center().Y) < 1e-6)
    return bd.fillet(es, r)


def root_fillet_rib():
    """A rib under a flange, its two ends joined to the flange's underside by
    concave root fillets: the rib's side faces end in cusps (Fable facing L2)."""
    return _round_edges_at(_min_box(0, -10, 20, 40, 20, 10) + _min_box(10, -2, 0, 20, 4, 20), 20, 2)


def rounded_slot():
    """A block with a through slot whose end edges at the top are rounded
    (convex fillets): the slot's side faces end in cusps."""
    return _round_edges_at(_min_box(0, -10, 0, 40, 20, 20) - _min_box(10, -2, 0, 20, 4, 20), 20, 2)


def spandrel(r=5.0):
    """The corner of a square minus a quarter circle, extruded: two real
    knife edges of material, where the arc runs tangent into the flat sides."""
    import build123d as bd

    sk = bd.Rectangle(r, r, align=(bd.Align.MIN, bd.Align.MIN)) - bd.Pos(r, r) * bd.Circle(r)
    return bd.extrude(sk, 3)


def _wedge(solid):
    from jointbench.quality import check_solid

    rep = check_solid(solid)
    return rep, next(x for x in rep.rules if x.id == "Q_THIN_WEDGE")


def test_cusp_where_a_root_fillet_ends_is_no_thin_wedge():
    from jointbench.quality.measure import face_corners

    rep, w = _wedge(root_fillet_rib())
    cusps = [c for c in face_corners(rep.part) if c["angle"] < 5]
    assert len(cusps) == 4 and all(c["blend"] for c in cusps)  # the cusps are there, and are blends
    assert w.ok and w.value == 90.0, w.reason


def test_cusp_where_a_convex_round_ends_is_no_thin_wedge():
    rep, w = _wedge(rounded_slot())
    assert w.ok and w.value == 90.0, w.reason


def test_knife_edge_between_a_round_and_a_tangent_flat_still_fails():
    from jointbench.quality.measure import face_corners

    rep, w = _wedge(spandrel())
    cusps = [c for c in face_corners(rep.part) if c["angle"] < 5]
    assert cusps and not any(c["blend"] for c in cusps)  # tangent edges, but a sliver of material
    assert not w.ok and w.value < 1.0
    kinds = {x["kind"] for x in w.details["wedges"]}
    assert {"face corner", "knife edge"} <= kinds
