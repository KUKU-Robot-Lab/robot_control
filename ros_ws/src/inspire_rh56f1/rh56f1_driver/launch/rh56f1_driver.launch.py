"""One Inspire RH56F1 hand on its own port: one vendor driver process per hand.

The vendor node (inspire_control_ros2/inspire_control_node) takes two YAML
files, not ROS parameters: a device file (protocol type, port, baud, Hand_ID,
logging) and a controller file (topics, services, joint names, rate). This
launch writes both from its arguments into ``<runtime_dir>/<side>/`` and starts
the vendor node on them, so the vendor code stays unchanged.

Nothing here commands motion. The vendor node only reads registers on its
timer and writes a register when a message arrives on a command topic or a
set_* service is called; this launch publishes nothing and wires no motion
service (no set_angle, no action sequence) unless ``admin_services:=true``.

Arguments (the side wrappers set ``side`` and a default ``port``):
  side          right | left
  transport     rs485 | canfd      -> protocol.type RH56F1_485 | RH56F1_canfd
  port          serial device of this hand (RS485 adapter, or the serial port
                the USB-CANFD converter presents; the vendor code has no
                SocketCAN path)
  baud          serial baud rate (hand factory default 115200)
  hand_id       Hand_ID register value of this hand, 1..254
  update_rate   read/publish loop rate in Hz (vendor default 50)
  enable_touch  read the 68-byte tactile block each cycle
  enable_current publish actuator current (read only, no current_set)
  admin_services expose config-writing services (id, baud, reset, flash
                defaults, calibration, mode, action sequence)
  log_level     vendor spdlog level
  runtime_dir   where the generated YAML and the vendor log file go
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIDES = ("right", "left")
PROTOCOL_BY_TRANSPORT = {"rs485": "RH56F1_485", "canfd": "RH56F1_canfd"}
LOG_LEVELS = ("TRACE", "DEBUG", "INFO", "WARN", "ERROR", "CRITICAL", "OFF")
HAND_ID_RANGE = (1, 254)
MAX_UPDATE_RATE_HZ = 200.0

# Vendor register slot order of every 6-value array (angle, force, speed,
# current, error, status, temp), index 0..5. The canonical names are those of
# the hdgp asset openarm_rh56f1_bi_rl. The two thumb joints are matched by
# range: slot 4 "thumb bend" travels ~25 deg (manual 110..135 deg) like
# *_hj_thumb_2 (0..0.4746 rad = 27.2 deg); slot 5 "thumb rotation" travels
# ~115 deg (manual 60..175 deg) like *_hj_thumb_1 (0..2.0944 rad = 120 deg).
SLOT_FINGERS = ("pinky_1", "ring_1", "middle_1", "index_1", "thumb_2", "thumb_1")

DEFAULT_SERVICES = (
    # (register, kind, service suffix)
    ("errorCode", "get", "get_errorCode"),
    ("status", "get", "get_status"),
    ("temp", "get", "get_temp"),
    ("clearError", "set", "set_clearError"),
    ("pause", "set", "set_pause"),
    ("stop", "set", "set_stop"),
)
# Services that write persistent configuration or start motion on their own.
# actionLibraryIndex is left out: the RH56F1 register map has no such register.
ADMIN_SERVICES = (
    ("id", "set", "set_id"),
    ("baudRate", "set", "set_baudRate"),
    ("resetPara", "set", "set_resetPara"),
    ("gestureForceClb", "set", "set_gestureForceClb"),
    ("defaultSpeedSet", "set", "set_defaultSpeed"),
    ("defaultForceSet", "set", "set_defaultForceSet"),
    ("mode", "set", "set_mode"),
    ("actionSeqIndex", "set", "set_actionSeqIndex"),
)


class Rh56f1LaunchError(ValueError):
    pass


def side_prefix(side: str) -> str:
    return "r" if side == "right" else "l"


def device_name(side: str) -> str:
    return f"hand_{side}"


def joint_names(side: str) -> list[str]:
    """Canonical joint names in vendor slot order."""
    prefix = side_prefix(side)
    return [f"{prefix}_hj_{finger}" for finger in SLOT_FINGERS]


def _as_bool(value: str, label: str) -> bool:
    lowered = value.strip().lower()
    if lowered in ("true", "1", "yes"):
        return True
    if lowered in ("false", "0", "no"):
        return False
    raise Rh56f1LaunchError(f"{label} must be true or false, got {value!r}")


def _as_int(value: str, label: str) -> int:
    try:
        return int(value)
    except ValueError as error:
        raise Rh56f1LaunchError(f"{label} must be an integer, got {value!r}") from error


def validate_args(raw: dict[str, str]) -> dict[str, Any]:
    """Check and type the launch arguments; raise on anything off-contract."""
    side = raw["side"].strip().lower()
    if side not in SIDES:
        raise Rh56f1LaunchError(f"side must be one of {SIDES}, got {raw['side']!r}")
    transport = raw["transport"].strip().lower()
    if transport not in PROTOCOL_BY_TRANSPORT:
        raise Rh56f1LaunchError(
            f"transport must be one of {tuple(PROTOCOL_BY_TRANSPORT)}, got {raw['transport']!r}"
        )
    port = raw["port"].strip()
    if not port:
        raise Rh56f1LaunchError("port must not be empty")
    baud = _as_int(raw["baud"], "baud")
    if baud <= 0:
        raise Rh56f1LaunchError(f"baud must be positive, got {baud}")
    hand_id = _as_int(raw["hand_id"], "hand_id")
    if not HAND_ID_RANGE[0] <= hand_id <= HAND_ID_RANGE[1]:
        raise Rh56f1LaunchError(f"hand_id must be in {HAND_ID_RANGE}, got {hand_id}")
    try:
        update_rate = float(raw["update_rate"])
    except ValueError as error:
        raise Rh56f1LaunchError(f"update_rate must be a number, got {raw['update_rate']!r}") from error
    if not 0.0 < update_rate <= MAX_UPDATE_RATE_HZ:
        raise Rh56f1LaunchError(
            f"update_rate must be in (0, {MAX_UPDATE_RATE_HZ}], got {update_rate}"
        )
    log_level = raw["log_level"].strip().upper()
    if log_level not in LOG_LEVELS:
        raise Rh56f1LaunchError(f"log_level must be one of {LOG_LEVELS}, got {raw['log_level']!r}")
    runtime_dir = Path(os.path.expanduser(raw["runtime_dir"].strip()))
    return {
        "side": side,
        "transport": transport,
        "port": port,
        "baud": baud,
        "hand_id": hand_id,
        "update_rate": update_rate,
        "enable_touch": _as_bool(raw["enable_touch"], "enable_touch"),
        "enable_current": _as_bool(raw["enable_current"], "enable_current"),
        "admin_services": _as_bool(raw["admin_services"], "admin_services"),
        "log_level": log_level,
        "runtime_dir": runtime_dir,
    }


def build_device_config(args: dict[str, Any]) -> dict[str, Any]:
    """The vendor device_protocol_config.yaml for one hand on one port."""
    side = args["side"]
    return {
        "protocol": {"type": PROTOCOL_BY_TRANSPORT[args["transport"]]},
        "devices": [
            {
                "name": device_name(side),
                "port": args["port"],
                "baudrate": args["baud"],
                "Hand_ID": args["hand_id"],
            }
        ],
        "logging": {
            "level": args["log_level"],
            "file": str(args["runtime_dir"] / side / f"{device_name(side)}.log"),
            "console": True,
            "file_enable": True,
            "max_file_size_mb": 10,
            "max_files": 5,
        },
    }


def _topics(args: dict[str, Any]) -> list[dict[str, Any]]:
    ns = f"/{device_name(args['side'])}"
    topics: list[dict[str, Any]] = [
        {
            "name": "angle_control",
            "registers": {"write": ["angleSet"], "read": ["angleAct"]},
            "command_topic": f"{ns}/angle_set",
            "state_topic": f"{ns}/angle_actual",
        },
        {
            "name": "force_control",
            "registers": {"write": ["forceSet"], "read": ["forceAct"]},
            "command_topic": f"{ns}/force_set",
            "state_topic": f"{ns}/force_actual",
        },
        {
            "name": "speed_control",
            "registers": {"write": ["speedSet"]},
            "command_topic": f"{ns}/speed_set",
        },
    ]
    if args["enable_current"]:
        # Read only: currentSet (1016) is the flash-saved current protection
        # value, not a per-cycle command.
        topics.append({
            "name": "current_control",
            "registers": {"read": ["currentAct"]},
            "state_topic": f"{ns}/current_actual",
        })
    if args["enable_touch"]:
        topics.append({
            "name": "touch_control",
            "registers": {"read": ["touchAct"]},
            "state_topic": f"{ns}/touch_data",
            "touch_version": 1,
        })
    return topics


def _services(args: dict[str, Any]) -> list[dict[str, Any]]:
    ns = f"/{device_name(args['side'])}"
    table = DEFAULT_SERVICES + (ADMIN_SERVICES if args["admin_services"] else ())
    services = []
    for register, kind, suffix in table:
        key = "set_service_name" if kind == "set" else "get_service_name"
        services.append({
            "register_name": register,
            key: f"{ns}/{suffix}",
            "is_write_register": kind == "set",
        })
    return services


def build_controller_config(args: dict[str, Any]) -> dict[str, Any]:
    """The vendor ros2_controller_config.yaml for one hand."""
    side = args["side"]
    return {
        "device_nodes": [
            {
                "device": device_name(side),
                "update_rate": args["update_rate"],
                "publish_header": {"frame_id": f"{side_prefix(side)}_hl_base"},
                "joint_names": joint_names(side),
                "topics": _topics(args),
                "services": _services(args),
            }
        ]
    }


def write_configs(args: dict[str, Any]) -> tuple[Path, Path]:
    out_dir = args["runtime_dir"] / args["side"]
    out_dir.mkdir(parents=True, exist_ok=True)
    device_path = out_dir / "device_protocol_config.yaml"
    controller_path = out_dir / "ros2_controller_config.yaml"
    device_path.write_text(yaml.safe_dump(build_device_config(args), sort_keys=False))
    controller_path.write_text(yaml.safe_dump(build_controller_config(args), sort_keys=False))
    return device_path, controller_path


ARGUMENTS = (
    ("side", "right", "right | left"),
    ("transport", "rs485", "rs485 | canfd"),
    ("port", "/dev/ttyUSB0", "serial device of this hand (verify on arm4090)"),
    ("baud", "115200", "serial baud rate"),
    ("hand_id", "1", "Hand_ID register of this hand (1..254)"),
    ("update_rate", "50", "read/publish loop rate in Hz"),
    ("enable_touch", "true", "read and publish the tactile block"),
    ("enable_current", "false", "publish actuator current (read only)"),
    ("admin_services", "false", "expose config-writing and action-sequence services"),
    ("log_level", "WARN", "vendor spdlog level"),
    ("runtime_dir", "~/.ros/rh56f1", "generated YAML and log directory"),
)


def _launch_setup(context, *_args, **_kwargs):
    raw = {name: LaunchConfiguration(name).perform(context) for name, _, _ in ARGUMENTS}
    args = validate_args(raw)
    device_path, controller_path = write_configs(args)
    name = device_name(args["side"])
    summary = (
        f"RH56F1 {args['side']}: {PROTOCOL_BY_TRANSPORT[args['transport']]} "
        f"port={args['port']} baud={args['baud']} hand_id={args['hand_id']} "
        f"rate={args['update_rate']}Hz touch={args['enable_touch']} "
        f"configs={device_path.parent}"
    )
    node = Node(
        package="inspire_control_ros2",
        executable="inspire_control_node",
        name=f"{name}_node",
        namespace=name,
        parameters=[{"use_sim_time": False}],
        arguments=[
            "--device-config", str(device_path),
            "--controller-config", str(controller_path),
            "--device", name,
        ],
        output="screen",
        emulate_tty=True,
    )
    return [LogInfo(msg=summary), node]


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS
    ]
    return LaunchDescription(declared + [OpaqueFunction(function=_launch_setup)])
