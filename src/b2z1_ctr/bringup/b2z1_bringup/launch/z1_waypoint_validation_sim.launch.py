from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_path
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("network_interface", default_value="lo"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_path("b2z1_bringup"),
                    "launch",
                    "mujoco",
                    "z1_waypoint_validation.launch.py",
                )
            ),
            launch_arguments={"network_interface": LaunchConfiguration("network_interface")}.items(),
        )
    ])
