"""The OpenArm + Inspire RH56F1 profile (the arm4090 robot)."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from robot_control.profile import load_profile


ROOT = Path(__file__).parents[1]
PROFILE = ROOT / "src/robot_control/profiles/openarm_rh56f1.yaml"
TESOLLO = ROOT / "src/robot_control/profiles/openarm_tesollo.yaml"
HAND_JOINTS = ("thumb_1", "thumb_2", "index_1", "middle_1", "ring_1", "pinky_1")


@pytest.fixture(scope="module")
def profile():
    return load_profile(PROFILE)


def test_profile_covers_two_arms_and_six_actuators_per_hand(profile):
    assert profile.name == "openarm_rh56f1"
    assert profile.asset_id == "openarm_rh56f1_bi_rl"
    assert len(profile.joints) == 26
    assert set(profile.groups) == {
        "openarm_right_arm", "openarm_left_arm", "rh56f1_right_hand", "rh56f1_left_hand",
    }
    for side, prefix in (("right", "r"), ("left", "l")):
        group = profile.groups[f"rh56f1_{side}_hand"]
        assert group.joints == tuple(f"{prefix}_hj_{name}" for name in HAND_JOINTS)


def test_arms_are_the_tesollo_arms(profile):
    tesollo = load_profile(TESOLLO)
    arm = {j.canonical: j for j in tesollo.joints if "_aj_" in j.canonical}

    assert {j.canonical: j for j in profile.joints if "_aj_" in j.canonical} == arm
    for name in ("openarm_right_arm", "openarm_left_arm"):
        mine, theirs = profile.groups[name], tesollo.groups[name]
        assert mine.joints == theirs.joints
        assert mine.controller == theirs.controller
        assert mine.moveit_group == theirs.moveit_group
        assert mine.tip_link == theirs.tip_link
        assert mine.effort_controller == theirs.effort_controller


def test_hand_groups_point_at_the_driver_but_are_not_executable(profile):
    # The vendor driver has no trajectory action and publishes vendor units, so
    # robotctl must not treat the hands as commandable joint groups.
    for side in ("right", "left"):
        group = profile.groups[f"rh56f1_{side}_hand"]
        assert group.controller is None
        assert group.state_topic == f"/hand_{side}/angle_actual"
        assert f"rh56f1_{side}_hand" not in profile.executable_groups()


def test_hand_limits_match_the_asset_urdf(profile):
    urdf = ET.parse(profile.asset_urdf_path).getroot()
    limits = {
        joint.get("name"): joint.find("limit")
        for joint in urdf.findall("joint")
        if joint.find("limit") is not None
    }
    for joint in profile.joints:
        if "_hj_" not in joint.canonical:
            continue
        limit = limits[joint.canonical]
        assert joint.lower == pytest.approx(float(limit.get("lower")), abs=1e-5)
        assert joint.upper == pytest.approx(float(limit.get("upper")), abs=1e-5)
