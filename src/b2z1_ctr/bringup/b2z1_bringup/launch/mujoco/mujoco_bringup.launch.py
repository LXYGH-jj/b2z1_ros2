from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_path
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    network_interface = LaunchConfiguration("network_interface")
    validation_mode = LaunchConfiguration("validation_mode")
    trajectory_config = os.path.join(
        get_package_share_path("b2z1_b2_trajectories"),
        "config",
        "b2_stand_trajectory.yaml",
    )
    bridge_config = os.path.join(
        get_package_share_path("b2z1_mujoco_bridge"),
        "config",
        "bridge.yaml",
    )

    mujoco_bridge = Node(
        package="b2z1_mujoco_bridge",
        executable="b2z1_mujoco_bridge_node",
        output="screen",
        parameters=[
            trajectory_config,
            bridge_config,
            {
                "network_interface": network_interface,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("network_interface", default_value="lo"),
        DeclareLaunchArgument("validation_mode", default_value="z1_waypoint_validation"),
        mujoco_bridge,
        Node(
            package="b2z1_examples",
            executable=validation_mode,
            output="screen",
        ),
    ])
