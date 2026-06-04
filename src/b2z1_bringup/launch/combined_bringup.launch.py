from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
import os


def generate_launch_description():
    launch_z1_bringup = LaunchConfiguration("launch_z1_bringup")
    with_gripper = LaunchConfiguration("with_gripper")
    z1_controller = LaunchConfiguration("z1_controller")
    rviz = LaunchConfiguration("rviz")

    z1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path("z1_bringup"), "launch", "z1.launch.py")
        ),
        launch_arguments={
            "sim_ignition": "false",
            "rviz": rviz,
            "with_gripper": with_gripper,
            "starting_controller": z1_controller,
        }.items(),
        condition=IfCondition(launch_z1_bringup),
    )

    b2_bridge = Node(
        package="b2z1_b2_bridge",
        executable="b2_bridge_node",
        output="screen",
    )

    coordinator = Node(
        package="b2z1_coordinator",
        executable="coordinator_node",
        output="screen",
    )

    use_home_example = LaunchConfiguration("use_home_example")
    use_reach_example = LaunchConfiguration("use_reach_example")

    stand_and_home = Node(
        package="b2z1_examples",
        executable="stand_and_home",
        output="screen",
        condition=IfCondition(use_home_example),
    )

    stand_and_reach = Node(
        package="b2z1_examples",
        executable="stand_and_reach",
        output="screen",
        condition=IfCondition(use_reach_example),
    )

    return LaunchDescription([
        DeclareLaunchArgument("launch_z1_bringup", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("with_gripper", default_value="false"),
        DeclareLaunchArgument("z1_controller", default_value="joint_trajectory_controller"),
        DeclareLaunchArgument("use_home_example", default_value="true"),
        DeclareLaunchArgument("use_reach_example", default_value="false"),
        z1_launch,
        b2_bridge,
        coordinator,
        stand_and_home,
        stand_and_reach,
    ])
