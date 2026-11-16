import json
import os

import pytest

from jointbench.stiffness import fe

pytestmark = pytest.mark.skipif(fe.available() is not None, reason="stiffness extra not installed")


def _rep(build, tmp_path, name="part", **kw):
    from jointbench.geometry import write_step
    from jointbench.stiffness import analyse

    part, mounts = build()
    p = os.path.join(tmp_path, f"{name}.step")
    write_step(part, p)
    return analyse(p, mounts, **kw)


def test_selftest_every_row_as_expected():
    from jointbench.stiffness import selftest

    rows, _ = selftest.run()
    assert all(r.ok for r in rows), selftest.table(rows)


def test_cantilever_bending_matches_textbook(tmp_path):
    from jointbench.stiffness import selftest

    rep = _rep(selftest.cantilever, tmp_path, keep_fields=False)
    want = selftest.textbook()
    fx = next(c for c in rep.cases if c.label == "Fx at B")
    assert abs(fx.stiffness / want["Fx at B"] - 1) < 0.03
    assert rep.symmetry < 1e-6


def test_report_json_and_fields(tmp_path):
    from jointbench.stiffness import selftest, write_fields

    rep = _rep(selftest.cantilever, tmp_path)
    j = rep.to_json()
    assert j["ok"] and len(j["cases"]) == 6 and j["weakest_force"]["unit"] == "N/mm"
    data = json.load(open(write_fields(rep, str(tmp_path))))
    nv, nt = len(data["mesh"]["vertices"]), len(data["mesh"]["triangles"])
    assert data["format"] == "jointbench-stiffness-fields" and data["version"] == 2
    assert {m["role"] for m in data["mounts"]} == {"held", "loaded"}
    for c in data["cases"]:
        assert len(c["displacement"]) == nv and len(c["magnitude"]) == nt


def test_two_separate_solids_give_no_figure(tmp_path):
    import build123d as bd

    from jointbench.stiffness import selftest

    def two():
        part, mounts = selftest.cantilever()
        return part + bd.Pos(0, 50, 0) * bd.Box(5, 5, 5), mounts

    rep = _rep(two, tmp_path, quality_on_failure=False)
    assert not rep.ok and "separate solids" in rep.error


def test_weakest_figure_is_axis_free(tmp_path):
    """The figure is the weakest combined load: never stiffer than any
    single force case, and the same whichever way the mount's axes point."""
    import numpy as np

    from jointbench.stiffness import _weakest

    rng = np.random.default_rng(0)
    a = rng.normal(size=(6, 6))
    C = a @ a.T + 6 * np.eye(6)
    w = _weakest("B", C, 50.0)
    assert w.stiffness <= min(1 / C[k, k] for k in range(3)) + 1e-12
    q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    T = np.zeros((6, 6))
    T[:3, :3] = T[3:, 3:] = q
    assert abs(_weakest("B", T @ C @ T.T, 50.0).stiffness / w.stiffness - 1) < 1e-9


def test_spec_requirement_from_bolts_and_span():
    import numpy as np

    from jointbench.stiffness import spec_requirements
    from jointbench.stiffness.mounts import Mount

    def m(name, x, bolt, n):
        return Mount(name, np.array([x, 0.0, 0.0]), np.eye(3)[0], np.eye(3)[1], np.eye(3)[2], (30, 30),
                     bolt_size=bolt, n_bolts=n)

    r = spec_requirements([m("A", 0, "M5", 2), m("B", 90, "M3", 4)])[0]
    # weaker mount: 4 x M3 = 20.12 mm2 < 2 x M5 = 28.4 mm2; load 1.2 x 20.12; deflection 90 / 250
    assert r["bolt_area_mm2"] == 20.12
    assert abs(r["required"] - 1.2 * 20.12 / (90 / 250)) < 1e-9


def test_stress_fields_envelope_and_beam_check(tmp_path):
    import numpy as np

    from jointbench.stiffness import selftest, write_fields

    rep = _rep(selftest.cantilever, tmp_path)
    P = rep.surface[0]
    s = rep.stresses["Fx at B"]  # 1 N across the beam at x = 100, held at x = 0
    sel = (abs(P[:, 0] - 50) < 1.0) & (abs(np.abs(P[:, 1]) - 5) < 1e-6)
    assert abs(s[sel].max() / (50 * 5 / (10 ** 4 / 12)) - 1) < 0.05  # M c / I at mid-span
    data = json.load(open(write_fields(rep, str(tmp_path))))
    env = data["stress_envelope"]
    assert data["version"] == 2 and env["unit"] == "MPa" and len(env["vertex_values"]) == len(data["mesh"]["vertices"])
    assert env["scale"][1] <= env["range"][1] and "NOT a requirement" in data["stress_note"]


def test_strength_materials_and_ball_pattern():
    import numpy as np

    from jointbench.stiffness.strength import _ball_pattern, material

    assert material("fdm")["allowable"] == 20.0 and material("pbf")["allowable"] == 24.0
    pts, w = _ball_pattern(0.5)
    assert np.linalg.norm(pts, axis=1).max() <= 0.5 + 1e-12
    assert abs(w.sum() - 0.5 ** 3) < 1e-9  # weights add up to the ball's volume over 4 pi / 3


def test_strength_sharp_loaded_corner_fails_radius_rule(tmp_path):
    from jointbench.stiffness import analyse, selftest
    from jointbench.stiffness.strength import judge

    part, mounts = selftest.filleted_bracket(0.0)
    from jointbench.geometry import write_step

    p = str(tmp_path / "sharp.step")
    write_step(part, p)
    res = judge(analyse(p, mounts, curvature=32), p, "fdm")
    assert res.verdict == "fail" and not res.radius_ok and res.smallest_loaded_radius == 0.0


def test_strength_notch_peak_matches_peterson(tmp_path):
    import numpy as np

    from jointbench.geometry import write_step
    from jointbench.stiffness import analyse, selftest

    part, mounts = selftest.notched_bar(4.0)
    p = str(tmp_path / "notch.step")
    write_step(part, p)
    rep = analyse(p, mounts, curvature=32)
    P = rep.surface[0]
    near = np.linalg.norm(P[:, :2] - np.array([60, 0]), axis=1) < 20
    kt = rep.stresses["Fz at B"][near].max() * (20 - 8) * 6
    assert abs(kt / selftest.kt_semicircular(4.0, 20) - 1) < 0.03


def test_process_material_scales_requirement_with_modulus():
    """In the process's plastic every figure and every requirement drop by
    the same factor, so the verdict is the one calibrated in aluminium."""
    import numpy as np

    from jointbench.stiffness import material_for, spec_requirements
    from jointbench.stiffness.mounts import Mount

    def m(name, x):
        return Mount(name, np.array([x, 0.0, 0.0]), np.eye(3)[0], np.eye(3)[1], np.eye(3)[2], (30, 30),
                     bolt_size="M3", n_bolts=4)

    al, pla = material_for(None), material_for("fdm")
    assert al["E"] == 68900.0 and pla["E"] == 3500.0 and pla["density"] > 0
    a = spec_requirements([m("A", 0), m("B", 90)])[0]
    b = spec_requirements([m("A", 0), m("B", 90)], pla["E"])[0]
    assert abs(a["required"] / b["required"] - al["E"] / pla["E"]) < 1e-9
    assert a["design_load_N"] == b["design_load_N"]  # the load itself stays physical


def test_round_member_is_meshed_with_curved_elements(tmp_path):
    """A round rod: the edges of its elements follow the surface, none had
    to be straightened, and the figure does not drop as a polygon would."""
    from jointbench.stiffness import selftest

    rep = _rep(selftest.flimsy_column, tmp_path, keep_fields=False, h=4.0)
    assert rep.ok and rep.curved and rep.straightened >= 0
    fine = _rep(selftest.flimsy_column, tmp_path, name="fine", keep_fields=False, h=2.0)
    assert abs(rep.figure.stiffness / fine.figure.stiffness - 1) < 0.02


def test_stated_load_sets_the_requirement_in_the_process_material():
    """A stated mass on a fixture: required = weight / (span / 250), stated in plain words with the
    process's real material, and judged without the aluminium calibration."""
    from jointbench.levels import level1
    from jointbench.requirements import apply_stated_load

    task, draw, ref = level1.generate(0, {"stiffness": 0})
    task.tolerances["manufacturing"] = {"process": "fdm", "quantity": 1, "direction": None}
    apply_stated_load(task, {"B": 2.0})
    m = task.stiffness["per_mount"][0]
    assert task.stiffness["mode"] == "stated" and "STIFFNESS" in task.tolerances["required_checks"]
    assert abs(m["required"] - 2.0 * 9.81 / (m["span_mm"] / 250)) / m["required"] < 1e-3
    assert "2 kg" in m["statement"] and "PLA" in m["statement"]
