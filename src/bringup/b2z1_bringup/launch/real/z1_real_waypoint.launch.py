from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    sim_ignition = LaunchConfiguration("sim_ignition")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("z1_bringup"), "/launch/z1.launch.py"]
        ),
        launch_arguments={
            "sim_ignition": sim_ignition,
            "starting_controller": "joint_trajectory_controller",
        }.items(),
    )

    waypoint_executable = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="b2z1_z1_control",
                executable="z1_waypoint_test",
                output="screen",
            ),
        ],
    )

    return [bringup_launch, waypoint_executable]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim_ignition",
                default_value="false",
                description="Launch Z1 through ignition simulation or real hardware interface.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
