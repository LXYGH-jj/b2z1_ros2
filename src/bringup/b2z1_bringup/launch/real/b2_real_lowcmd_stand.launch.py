from launch import LaunchDescription
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node
import os


def generate_launch_description():
    trajectory_config = os.path.join(
        get_package_share_path("b2z1_b2_trajectories"),
        "config",
        "b2_stand_trajectory.yaml",
    )
    return LaunchDescription([
        Node(
            package="b2z1_b2_lowcmd",
            executable="b2_real_lowcmd_stand_node",
            output="screen",
            parameters=[trajectory_config],
        ),
    ])
