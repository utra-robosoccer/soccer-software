"""Bring up ONE fully-namespaced MiniBot (blueprint §8, §11).

The SAME launch file runs the robot in simulation or on hardware — the only
difference is the ``sim`` argument, which flips the ros2_control hardware plugin
(blueprint §10). Everything is pushed under a per-robot namespace (``/robot_1``,
...) so the identical graph can be replicated for the whole fleet by
``team.launch.py``.

Layers wired here:
  L0  (real only) MCU serial — via the hardware plugin inside controller_manager
  L1  controller_manager + residual_rl_controller + mpc_node
  L2  ekf_node (odometry)
  L3  detector / fieldline / projection / mcl_node
  L4  strategy_node + teamcomm_node
  L5  gc_bridge_node
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_name = LaunchConfiguration("robot_name")
    player_id = LaunchConfiguration("player_id")
    team_number = LaunchConfiguration("team_number")
    sim = LaunchConfiguration("sim")

    description_share = FindPackageShare("soccer_description")
    xacro_file = PathJoinSubstitution([description_share, "urdf", "minibot.urdf.xacro"])
    controllers_yaml = PathJoinSubstitution([description_share, "config", "controllers.yaml"])

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " sim:=", sim]), value_type=str
    )

    robot_group = GroupAction([
        PushRosNamespace(robot_name),

        # ── L1: robot_state_publisher + ros2_control controller manager ──
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}, controllers_yaml],
            output="screen",
        ),
        # Spawners (the controller manager makes the spawners wait as needed).
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster"], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["imu_sensor_broadcaster"], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["residual_rl_controller"], output="screen"),

        # ── L1: MPC reference generator ──
        Node(package="soccer_control", executable="mpc_node", output="screen"),

        # ── Sim-only synthetic camera (stands in for the Isaac camera sensor) ──
        Node(package="soccer_bringup", executable="sim_camera_node",
             condition=IfCondition(sim), output="screen"),

        # ── L3: perception ──
        Node(package="soccer_perception", executable="detector_node", output="screen"),
        Node(package="soccer_perception", executable="fieldline_node", output="screen"),
        Node(package="soccer_perception", executable="projection_node",
             parameters=[{"use_depth": False}], output="screen"),

        # ── L2 + L3: two-tier localization ──
        Node(package="soccer_localization", executable="ekf_node", output="screen"),
        Node(package="soccer_localization", executable="mcl_node", output="screen"),

        # ── L4: strategy + team comms ──
        Node(package="soccer_strategy", executable="strategy_node",
             parameters=[{"player_id": player_id, "goalie_id": 1}], output="screen"),
        Node(package="soccer_teamcomm", executable="teamcomm_node",
             parameters=[{"player_id": player_id}], output="screen"),

        # ── L5: GameController bridge ──
        Node(package="game_controller_bridge", executable="gc_bridge_node",
             parameters=[{"player_id": player_id, "team_number": team_number}],
             output="screen"),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robot_1"),
        DeclareLaunchArgument("player_id", default_value="1"),
        DeclareLaunchArgument("team_number", default_value="1"),
        DeclareLaunchArgument("sim", default_value="true",
                              description="true: sim hardware plugin; false: real MCU serial"),
        robot_group,
    ])
