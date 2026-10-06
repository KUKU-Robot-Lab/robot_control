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
    assert p.argv().count(",") == 17
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
    assert max(tail) - min(tail) <= 100.0 and 250 < tail[-1] < 500  # k x penetration, + up to the hold band


def test_link_1_contact_counts_more_and_soft_cap():
    p = AdmParams(hold_band_g=0.0, proximal_scale=0.7)
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


def test_hold_band_keeps_the_offset_between_hand_steps():
    """10.06 right index + cup at 500 Hz: the hand steps 3-5 registers, the force hunted 80 <-> 300 g."""
    p, s = load_admittance(), AdmState()
    s.y[3] = 60.0                      # k y = 216 g
    for f in (130.0, 290.0, 180.0):    # readings within the band of 216 + 40 deadband
        adm_step(p, s, 3, 0.002, 1300.0, 1360.0, f + 40.0, 300)
        assert s.y[3] == pytest.approx(60.0)
    adm_step(p, s, 3, 0.002, 1300.0, 1360.0, 500.0, 300)  # far outside: moves
    assert s.y[3] > 60.0



def _rigid_cup(p, pen_reg, approach_reg_s=275.0, delay_ticks=12, kp_g_per_reg=55.0, seconds=6.0):
    """10.06 right index + rigid cup: the finger stops at the cup and the firmware pushes ~55 g per register of
    command past it; the force reading arrives `delay_ticks` cycles late."""
    s, contact, actual, seen, forces = AdmState(), 1300.0, 1500.0, [0.0] * delay_ticks, []
    for k in range(int(seconds / 0.002)):
        target = max(contact - pen_reg, 1500.0 - approach_reg_s * k * 0.002)
        cmd = adm_step(p, s, 3, 0.002, target, actual, seen.pop(0), 300)
        actual = max(actual + max(min(cmd - actual, 1.76), -1.76), contact)
        force = min(max(actual - cmd, 0.0) * kp_g_per_reg, 1800.0) if actual <= contact else 0.0
        seen.append(force)
        forces.append(force)
    return forces


def test_rigid_cup_grip_follows_the_penetration_without_a_cap():
    """10.06: the lead cap held 0.4 rad on a rigid cup at ~350 g (real 353 / 362 g)."""
    p = load_admittance()
    # 24 ms force delay. At 50 ms this plant cycles with tau_contact 0.3 (settles at 1.0): the 10.06 evening
    # trial on the real hand decides; revert to 1.0 if it cycles there.
    for pen, delay in ((220.0, 12), (110.0, 12)):
        forces = _rigid_cup(p, pen, delay_ticks=delay)
        tail = forces[-500:]
        assert max(tail) - min(tail) < 1.0                                # settled, no cycle
        assert abs(tail[-1] - p.k_g_per_reg * pen) <= p.deadband_g + p.hold_band_g
        assert max(forces) < 950.0                                        # 0.5 rad/s approach


def test_fast_free_space_closing_is_not_slowed():
    """10.06: closing at 4 rad/s reads 50-60 g over rest with nothing touched."""
    p, s = load_admittance(), AdmState()
    for actual in range(1600, 1400, -4):
        assert adm_step(p, s, 3, 0.002, 1100.0, float(actual), 60.0 + p.deadband_g, 0) == 1100.0


def test_contact_restarts_the_command_from_the_finger_then_closes_at_the_rate():
    p, s = load_admittance(), AdmState()
    f = 400.0
    cmd = adm_step(p, s, 3, 0.002, 1100.0, 1300.0, f + p.deadband_g, 300)
    assert cmd == pytest.approx(1300.0 - p.rate_reg_s * (1 - f / p.f_max_g) * 0.002)
    cmd2 = adm_step(p, s, 3, 0.002, 1100.0, 1300.0, f + p.deadband_g, 300)
    assert cmd2 < cmd and cmd - cmd2 == pytest.approx(p.rate_reg_s * 0.5 * 0.002, rel=0.01)
    assert adm_step(p, s, 3, 0.002, 1500.0, 1300.0, f + p.deadband_g, 300) > 1500.0   # opening: at once (+ y)
