"""Produce the repo's generic camera contract from a real ZED camera.

This is the real-hardware counterpart to the sim-only ``sim_camera_node``. It
runs the :mod:`soccer_bringup.camera_bridge` node, which maps the Stereolabs ZED
wrapper's native ``/zed/zed_node/...`` topics onto the driver-agnostic contract
(``camera/image_raw``, ``camera/depth``, ``camera_info``, ``imu/data``) the
perception / localization stack consumes (``docs/jetson_zed_workflow.md`` §3, §6).

Two deployment shapes are supported:

* **Two containers (default, recommended).** The ZED wrapper runs in the
  ``zed-driver-image`` container (``ros2 launch zed_wrapper zed_camera.launch.py
  camera_model:=zedm``) and publishes the native topics on the DDS network; the
  app container includes THIS launch (``launch_driver:=false``) to bridge them.
* **One container (dev / spike).** Set ``launch_driver:=true`` to also start the
  ZED wrapper in-process — only valid in an image that contains the ZED SDK +
  wrapper (i.e. the ``zed-driver-image``).

``robot.launch.py camera:=zed`` includes this file INSIDE the per-robot namespace
group, so the bridge's relative outputs land on ``/<robot_name>/camera/...``.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    camera_model = LaunchConfiguration("camera_model")
    launch_driver = LaunchConfiguration("launch_driver")

    # Optional in-process ZED wrapper (single-container mode only).
    zed_launch = PathJoinSubstitution(
        [FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"]
    )
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([zed_launch]),
        launch_arguments={"camera_model": camera_model}.items(),
        condition=IfCondition(launch_driver),
    )

    # The ZED → generic-contract bridge (relative outputs inherit the namespace).
    bridge = Node(
        package="soccer_bringup",
        executable="camera_bridge_node",
        name="camera_bridge",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera_model", default_value="zedm",
                              description="ZED model (zedm = ZED Mini)."),
        DeclareLaunchArgument("launch_driver", default_value="false",
                              description="true: also start the ZED wrapper "
                                          "in-process (zed-driver-image only)."),
        driver,
        bridge,
    ])
