"""The profile's joint limits, against the description the robot is built from.

The profile bounds what may be commanded, and `design_pose_set` samples poses
inside those bounds. When they were placeholders — a round +-3.14 and +-2.0 on
every arm joint — the designer produced poses past the hard stops. The arm
juddered against them, could not droop, and the sweep read as stiction: the
tracking error came back equal to the whole commanded angle, and every gravity
scale measured identically.

The effort numbers mattered more. The profile authorized 20 N.m on wrist joints
the description rates at 7, so the gate would have approved roughly three times
what the hardware is built for on the one path that publishes torque.

Widths rather than endpoints, because the bimanual xacro mirrors each arm and
offsets joint 2 by a quarter turn per side. Mirroring and offsetting both
preserve a range's width, so this compares the invariant instead of restating
the transform and drifting from it.

The DG5F hand carried the same placeholder shape for longer: a round +-1.5 on
all twenty joints. The hand's ranges are not symmetric and mostly not centred
on zero — the thumb's second joint only travels negative, the other fingers'
only positive — so a symmetric bound authorizes a curl command that drives four
fingers straight into their stops. Endpoints rather than widths there: the
right hand is described in its own frame, with nothing mirroring it.
"""

from pathlib import Path
from xml.etree import ElementTree

import pytest
import yaml

from robot_control.profile import load_builtin_profile
from robot_control.srdf import repository_root


LIMITS = "ros_ws/src/openarm_description/config/arm/v10/joint_limits.yaml"
ARM_GROUPS = ("openarm_right_arm", "openarm_left_arm")
TOLERANCE_RAD = 1e-3

#: The vendor driver's description of the hand that is mounted (DG-5F-M, short
#: base) — the hand's equivalent of the arm's joint_limits.yaml. It disagrees
#: with the Tesollo CAD release (repo/tesollo/tesollo_model, which the training
#: asset is built from) on nine joints per hand. Since 2026-09-23 the driver is
#: the reference, because it is what the real hand enforces; the CAD table is
#: to be brought in line in the URDF/USD later.
#: The short and default bases share the finger chain, so their limits match.
HAND_DESCRIPTION = "ros_ws/src/dg5f_ros2/dg5f_description/urdf/dg5f_right_short.urdf"


@pytest.fixture(scope="module")
def described():
    path = repository_root() / LIMITS
    if not path.is_file():
        pytest.skip(f"vendored joint limits not found: {path}")
    raw = yaml.safe_load(path.read_text())
    return {
        name: body["limit"]
        for name, body in raw.items()
        if isinstance(body, dict) and "limit" in body
    }


@pytest.fixture(scope="module")
def profile():
    return load_builtin_profile("openarm_tesollo")


def _arm_joints(profile):
    """Each arm joint, paired with the description entry it is built from."""
    by_canonical = {joint.canonical: joint for joint in profile.joints}
    for group_name in ARM_GROUPS:
        for index, canonical in enumerate(profile.groups[group_name].joints, start=1):
            yield by_canonical[canonical], f"joint{index}"


def test_every_arm_joint_spans_the_range_the_description_gives_it(profile, described):
    for joint, key in _arm_joints(profile):
        limit = described[key]
        expected = float(limit["upper"]) - float(limit["lower"])
        actual = joint.upper - joint.lower

        assert actual == pytest.approx(expected, abs=TOLERANCE_RAD), (
            f"{joint.canonical} spans {actual:.4f} rad against the description's "
            f"{expected:.4f}; a wider profile designs poses past the hard stops"
        )


def test_no_joint_may_be_driven_past_what_the_hardware_is_rated_for(profile, described):
    for joint, key in _arm_joints(profile):
        rated = float(described[key]["effort"])

        assert joint.effort <= rated, (
            f"{joint.canonical} authorizes {joint.effort:g} N.m against a rated "
            f"{rated:g}; r2s identify publishes torque through this bound"
        )


def test_commanded_speed_stays_within_the_description(profile, described):
    for joint, key in _arm_joints(profile):
        rated = float(described[key]["velocity"])

        assert joint.velocity <= rated, (
            f"{joint.canonical} allows {joint.velocity:g} rad/s against a rated "
            f"{rated:g}"
        )


#: Each hand's groups and the driver description file its limits come from. The left
#: hand is the right reflected, so its off-centre ranges are flipped — checked
#: against its own file rather than derived, for the same reason the arms are.
HANDS = {
    "right": (
        ("tesollo_abduction", "tesollo_curl", "tesollo_pip", "tesollo_dip"),
        HAND_DESCRIPTION,
    ),
    "left": (
        ("tesollo_left_abduction", "tesollo_left_curl", "tesollo_left_pip", "tesollo_left_dip"),
        "ros_ws/src/dg5f_ros2/dg5f_description/urdf/dg5f_left_short.urdf",
    ),
}


@pytest.fixture(scope="module", params=sorted(HANDS))
def hand(request, profile):
    """Each hand joint, and the description limits keyed by vendor joint name."""
    groups, description = HANDS[request.param]
    path = repository_root() / description
    if not path.is_file():
        pytest.skip(f"hand description not found: {path}")
    root = ElementTree.parse(path).getroot()
    described = {
        joint.get("name"): joint.find("limit")
        for joint in root.findall("joint")
        if joint.find("limit") is not None
    }
    by_canonical = {joint.canonical: joint for joint in profile.joints}
    joints = [by_canonical[c] for name in groups for c in profile.groups[name].joints]
    return joints, described


def test_every_hand_joint_ends_where_the_description_ends(hand):
    joints, described_hand = hand
    for joint in joints:
        limit = described_hand[joint.source]
        lower, upper = float(limit.get("lower")), float(limit.get("upper"))

        assert (joint.lower, joint.upper) == pytest.approx(
            (lower, upper), abs=TOLERANCE_RAD
        ), (
            f"{joint.canonical} is bounded [{joint.lower:+.4f}, {joint.upper:+.4f}] "
            f"against the description's [{lower:+.4f}, {upper:+.4f}]; a bound "
            "wider than the stop authorizes a command the finger cannot reach"
        )


def test_no_hand_joint_may_be_driven_past_what_the_hardware_is_rated_for(hand):
    joints, described_hand = hand
    for joint in joints:
        rated = float(described_hand[joint.source].get("effort"))

        assert joint.effort <= rated, (
            f"{joint.canonical} authorizes {joint.effort:g} N.m against a rated "
            f"{rated:g}"
        )


def test_commanded_hand_speed_stays_within_the_description(hand):
    joints, described_hand = hand
    for joint in joints:
        rated = float(described_hand[joint.source].get("velocity"))

        assert joint.velocity <= rated, (
            f"{joint.canonical} allows {joint.velocity:g} rad/s against a rated "
            f"{rated:g}"
        )


#: A step between identification poses, in radians. Not a limit — a typical
#: move, used to state the default duration as the speed an operator sees.
TYPICAL_REPOSITIONING_RAD = 1.5
REACTABLE_RAD_PER_SEC = 0.2


def test_a_commanded_move_is_slow_enough_to_be_stopped_by_hand():
    """The bound that actually governs how fast the arm goes, and why here.

    The joint velocity limit cannot serve this. The excitation is a small fast
    dither whose peak slew is frequency times amplitude, so a cap low enough to
    slow a large repositioning refuses the measurement outright — and at 3 s a
    1.5 rad pose change already ran at 0.5 rad/s, which a lower cap would not
    have caught either. The duration is what the operator experiences.
    """
    from robot_control.cli import DEFAULT_DURATION_SEC

    speed = TYPICAL_REPOSITIONING_RAD / DEFAULT_DURATION_SEC

    assert speed <= REACTABLE_RAD_PER_SEC, (
        f"a {TYPICAL_REPOSITIONING_RAD:g} rad move over the default "
        f"{DEFAULT_DURATION_SEC:g} s runs at {speed:.2f} rad/s, faster than an "
        "operator can react to"
    )
