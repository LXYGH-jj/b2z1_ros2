from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
import os


def generate_launch_description():
    bringup_share = get_package_share_path("b2z1_bringup")
    return LaunchDescription([
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "real", "b2_real_lowcmd_stand.launch.py")
            )
        )
    ])
