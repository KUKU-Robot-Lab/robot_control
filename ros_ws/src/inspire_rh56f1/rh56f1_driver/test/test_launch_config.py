"""The YAML that rh56f1_driver.launch.py hands the vendor node.

Runs without hardware: it builds and writes the configs only, it never starts
the node or opens a port.
"""

import importlib.util
from pathlib import Path

import pytest
import yaml

LAUNCH = Path(__file__).parents[1] / "launch" / "rh56f1_driver.launch.py"


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("rh56f1_driver_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def raw_args(mod, tmp_path, **overrides):
    base = {name: default for name, default, _ in mod.ARGUMENTS}
    base["runtime_dir"] = str(tmp_path)
    base.update(overrides)
    return base


@pytest.mark.parametrize("transport, protocol", [("rs485", "RH56F1_485"), ("canfd", "RH56F1_canfd")])
def test_transport_selects_registered_vendor_protocol(mod, tmp_path, transport, protocol):
    args = mod.validate_args(raw_args(mod, tmp_path, transport=transport))

    device = mod.build_device_config(args)

    assert device["protocol"]["type"] == protocol


def test_one_hand_one_port_per_process(mod, tmp_path):
    args = mod.validate_args(raw_args(
        mod, tmp_path, side="left", port="/dev/ttyUSB7", baud="921600", hand_id="3"))

    device = mod.build_device_config(args)

    assert device["devices"] == [
        {"name": "hand_left", "port": "/dev/ttyUSB7", "baudrate": 921600, "Hand_ID": 3}
    ]


def test_joint_names_follow_vendor_slot_order(mod):
    assert mod.joint_names("right") == [
        "r_hj_pinky_1", "r_hj_ring_1", "r_hj_middle_1",
        "r_hj_index_1", "r_hj_thumb_2", "r_hj_thumb_1",
    ]
    assert mod.joint_names("left")[0] == "l_hj_pinky_1"
    assert mod.joint_names("left")[5] == "l_hj_thumb_1"


def test_topics_live_under_the_hand_namespace(mod, tmp_path):
    args = mod.validate_args(raw_args(mod, tmp_path, side="right"))

    node = mod.build_controller_config(args)["device_nodes"][0]
    topics = {t["name"]: t for t in node["topics"]}

    assert node["device"] == "hand_right"
    assert topics["angle_control"]["command_topic"] == "/hand_right/angle_set"
    assert topics["angle_control"]["state_topic"] == "/hand_right/angle_actual"
    assert topics["force_control"]["state_topic"] == "/hand_right/force_actual"
    assert topics["speed_control"]["command_topic"] == "/hand_right/speed_set"
    assert topics["touch_control"]["state_topic"] == "/hand_right/touch_data"
    assert "current_control" not in topics


def test_current_is_read_only_when_enabled(mod, tmp_path):
    args = mod.validate_args(raw_args(mod, tmp_path, enable_current="true", enable_touch="false"))

    topics = {t["name"]: t for t in mod.build_controller_config(args)["device_nodes"][0]["topics"]}

    assert "touch_control" not in topics
    assert topics["current_control"]["registers"] == {"read": ["currentAct"]}
    assert "command_topic" not in topics["current_control"]


def test_default_services_start_no_motion_and_write_no_flash(mod, tmp_path):
    args = mod.validate_args(raw_args(mod, tmp_path))

    services = mod.build_controller_config(args)["device_nodes"][0]["services"]
    registers = {s["register_name"] for s in services}

    assert registers == {"errorCode", "status", "temp", "clearError", "pause", "stop"}
    for moving_or_persistent in ("angleSet", "actionSeqIndex", "defaultSpeedSet",
                                 "defaultForceSet", "id", "baudRate", "resetPara"):
        assert moving_or_persistent not in registers


def test_admin_services_are_opt_in(mod, tmp_path):
    args = mod.validate_args(raw_args(mod, tmp_path, admin_services="true"))

    registers = {s["register_name"] for s in
                 mod.build_controller_config(args)["device_nodes"][0]["services"]}

    assert {"id", "baudRate", "mode", "actionSeqIndex"} <= registers
    assert "actionLibraryIndex" not in registers


def test_written_files_round_trip(mod, tmp_path):
    args = mod.validate_args(raw_args(mod, tmp_path, side="left"))

    device_path, controller_path = mod.write_configs(args)

    assert device_path.parent == tmp_path / "left"
    assert yaml.safe_load(device_path.read_text()) == mod.build_device_config(args)
    assert yaml.safe_load(controller_path.read_text()) == mod.build_controller_config(args)


@pytest.mark.parametrize("override", [
    {"side": "middle"},
    {"transport": "can"},
    {"port": " "},
    {"baud": "fast"},
    {"baud": "0"},
    {"hand_id": "0"},
    {"hand_id": "255"},
    {"update_rate": "0"},
    {"update_rate": "1000"},
    {"enable_touch": "maybe"},
    {"log_level": "LOUD"},
])
def test_rejects_off_contract_arguments(mod, tmp_path, override):
    with pytest.raises(mod.Rh56f1LaunchError):
        mod.validate_args(raw_args(mod, tmp_path, **override))
