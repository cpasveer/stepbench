"""The joint catalogue: standards tables, determinism, and every self-test row on one seed."""
from __future__ import annotations

import json
import os

import pytest

from jointbench import joints
from jointbench.joints import common as C
from jointbench.joints.__main__ import main
from jointbench.joints.standards import key_for, shaft_zone, zone


def test_iso286_known_values():
    # textbook values: 25 H7 = 0/+21, 25 p6 = +22/+35, 25 s6 = +35/+48, 8 P9 = -15/-51, 2 N9 = -4/-29
    assert zone(25, "H7") == pytest.approx((25.0, 25.021))
    assert shaft_zone(25, "p", 6) == pytest.approx((25.022, 25.035))
    assert zone(25, "s6") == pytest.approx((25.035, 25.048))
    assert zone(8, "P9") == pytest.approx((7.949, 7.985))
    assert zone(2, "N9") == pytest.approx((1.971, 1.996))
    assert zone(10, "f7") == pytest.approx((9.972, 9.987))


def test_din6885_keys():
    k = key_for(40)
    assert (k.b, k.h, k.t1, k.t2) == (12, 8, 5.0, 3.3)
    assert key_for(30).b == 8  # "over 22 up to 30"
    assert key_for(30.5).b == 10


@pytest.mark.parametrize("name", joints.ORDER)
def test_generate_is_deterministic(name):
    m = joints.get(name)
    a, b = m.generate(3), m.generate(3)
    assert json.dumps(a.to_json(), sort_keys=True) == json.dumps(b.to_json(), sort_keys=True)
    assert m.brief(a) == m.brief(b)
    assert m.generate(4).to_json() != a.to_json()


@pytest.mark.parametrize("name", joints.ORDER)
def test_selftest_rows(name):
    rows = C.selftest(joints.get(name), seeds=[7], verbose=False)
    bad = [(r["label"], r["expect"], r["failed"]) for r in rows if not r["ok"]]
    assert not bad, bad


def test_gen_and_verify(tmp_path, capsys):
    out = str(tmp_path)
    assert main(["keyed", "--gen", "--seed", "2", "--out", out]) == 0
    d = os.path.join(out, "keyed-s2")
    assert os.path.exists(os.path.join(d, "README.md"))
    ref = os.path.join(d, "_solution", "reference.step")
    assert main(["keyed", "--verify", d, "--step", ref]) == 0
    fx = [f for f in os.listdir(d) if f.startswith("fixed_")][0]
    assert main(["keyed", "--verify", d, "--step", os.path.join(d, fx)]) == 1  # the given part is not an answer
