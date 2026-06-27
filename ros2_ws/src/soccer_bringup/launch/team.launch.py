"""Bring up a team of N soccerbots — fully decentralized, no master (blueprint §8).

Each robot is the IDENTICAL graph from robot.launch.py under its own namespace
(``/robot_1`` ...) and player id. They share only the global ``/team_data`` topic
(world model + role bids), so the role auction in soccer_strategy assigns roles
with no central coordinator and degrades gracefully if a robot drops out.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _spawn_robots(context, *args, **kwargs):
    num = int(LaunchConfiguration("num_robots").perform(context))
    sim = LaunchConfiguration("sim").perform(context)
    team = LaunchConfiguration("team_number").perform(context)
    robot_launch = PathJoinSubstitution(
        [FindPackageShare("soccer_bringup"), "launch", "robot.launch.py"]
    )

    actions = []
    for i in range(1, num + 1):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([robot_launch]),
                launch_arguments={
                    "robot_name": f"robot_{i}",
                    "player_id": str(i),
                    "team_number": team,
                    "sim": sim,
                }.items(),
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("num_robots", default_value="2"),
        DeclareLaunchArgument("team_number", default_value="1"),
        DeclareLaunchArgument("sim", default_value="true"),
        OpaqueFunction(function=_spawn_robots),
    ])
