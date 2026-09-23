from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("network_interface", default_value="lo"),
        DeclareLaunchArgument("validation_mode", default_value="z1_waypoint_validation"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_path("b2z1_bringup"),
                    "launch",
                    "mujoco",
                    "mujoco_bringup.launch.py",
                )
            ),
            launch_arguments={
                "network_interface": LaunchConfiguration("network_interface"),
                "validation_mode": LaunchConfiguration("validation_mode"),
            }.items(),
        ),
    ])
