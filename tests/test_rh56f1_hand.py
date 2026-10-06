"""RH56F1 canonical hand control (components/rh56f1.yaml control.admittance) — 10.06."""

from __future__ import annotations

import pytest
import yaml

from robot_control.rh56f1_hand import COMPONENT, AdmParams, AdmState, adm_step, load_admittance, load_protection


def test_contract_has_both_inputs_and_the_admittance():
    raw = yaml.safe_load(COMPONENT.read_text())
    cmd = raw["driver"]["command"]
    assert cmd["position"]["topic"] == "/hand_<side>/angle_set"
    assert cmd["admittance"]["topic"] == "/hand_<side>/angle_target"
    p = load_admittance()
    assert p.joints == (1, 1, 1, 1, 1, 0)  # thumb_1 position only
    assert p.argv().count(",") == 15
    assert load_protection()["current_limit_ma"] == 800 and load_protection()["finger_mode"] == 0


def test_free_space_follows_the_target():
    p, s = load_admittance(), AdmState()
    for _ in range(500):
        cmd = adm_step(p, s, 3, 0.002, 1300.0, 1400.0, 20.0, 0)
    assert cmd == 1300.0


def test_rigid_contact_settles_within_a_register():
    """10.06 right index on a cup: 1 register ~100 g."""
    p, s = load_admittance(), AdmState()
    contact, target, actual, forces = 1400.0, 1300.0, 1500.0, []
    for _ in range(4000):
        force = max(contact - actual, 0.0) * 100.0
        cmd = adm_step(p, s, 3, 0.002, target, actual, force, 300)
        actual += max(min(cmd - actual, 2.0), -2.0)
        forces.append(force)
    tail = forces[-500:]
    assert max(tail) - min(tail) <= 100.0 and 250 < tail[-1] < 400


def test_link_1_contact_counts_more_and_soft_cap():
    p = AdmParams()
    a, b = AdmState(), AdmState()
    for _ in range(5000):
        adm_step(p, a, 3, 0.002, 1300.0, 1300.0, 440.0, 300)
        adm_step(p, b, 3, 0.002, 1300.0, 1300.0, 440.0, 0)
    assert b.y[3] == pytest.approx(a.y[3] / 0.7, rel=0.01)
    c = AdmState()
    for _ in range(10000):
        adm_step(p, c, 3, 0.002, 1300.0, 1300.0, 1040.0, 300)
    assert c.y[3] == pytest.approx(1000 / 3.6 + 200 / 0.36, rel=0.01)


def test_bad_parameters_are_refused():
    with pytest.raises(ValueError):
        AdmParams(k_g_per_reg=0)
    with pytest.raises(ValueError):
        AdmParams.from_cfg({"k": 1})
