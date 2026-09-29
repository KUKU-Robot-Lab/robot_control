"""RH56F1 left hand driver: rh56f1_driver.launch.py with side:=left.

Every other argument of rh56f1_driver.launch.py (transport, baud, hand_id,
update_rate, enable_touch, enable_current, admin_services, log_level,
runtime_dir) passes through from the command line. The port default is a
placeholder until the arm4090 device path is verified; prefer a stable
/dev/serial/by-id/... path there.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

SIDE = "left"
DEFAULT_PORT = "/dev/ttyUSB1"


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "port", default_value=DEFAULT_PORT,
            description=f"serial device of the {SIDE} hand (verify on arm4090)",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("rh56f1_driver"), "launch", "rh56f1_driver.launch.py",
            ])),
            launch_arguments={"side": SIDE, "port": LaunchConfiguration("port")}.items(),
        ),
    ])
