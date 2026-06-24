"""ZED → contract camera bridge (the real-hardware analogue of ``sim_camera``).

The perception / localization stack subscribes to **driver-agnostic** contract
topics (``camera/image_raw``, ``camera/depth``, ``camera_info``, ``imu/data``) so
the camera is a drop-in (see ``docs/jetson_zed_workflow.md`` §3). The Stereolabs
ZED wrapper instead publishes under ``/zed/zed_node/...`` **and** its node is a
*composable* node — which launch-level ``SetRemap`` cannot reliably remap. This
node bridges the two.

Why a node and not ``topic_tools relay``: a relay republishes on a *fixed* QoS,
which mismatches sensor data and silently drops frames. Here the subscriptions
use **best-effort SensorData** QoS (compatible with the wrapper whether it
publishes reliable or best-effort) and re-publish on the **reliable** contract
QoS the rest of the graph already expects. Messages are forwarded as-is (no
copy / decode), so the bridge is cheap even at 30 fps.

Default source topics are the ones verified live on the Jetson Orin Nano + ZED
Mini (ZED SDK 5.x / wrapper release_5.4); every source is a parameter so a
different ``camera_name`` / model only changes launch arguments, not code.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, Imu

# Contract publishers use the default reliable QoS the existing consumers
# (detector / fieldline / projection / ekf) subscribe with.
_CONTRACT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class CameraBridge(Node):
    """Forwards the ZED wrapper's native topics onto the generic camera contract."""

    def __init__(self) -> None:
        super().__init__("camera_bridge")
        # Source topics (ZED SDK 5.x naming) — overridable for other models/names.
        self.declare_parameter("image_in", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("depth_in", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("camera_info_in", "/zed/zed_node/rgb/color/rect/camera_info")
        self.declare_parameter("imu_in", "/zed/zed_node/imu/data")

        # (source, destination, type) — destinations are RELATIVE so the node's
        # namespace (e.g. /robot_1 from robot.launch.py) prefixes them.
        bridges = [
            (self.get_parameter("image_in").value, "camera/image_raw", Image),
            (self.get_parameter("depth_in").value, "camera/depth", Image),
            (self.get_parameter("camera_info_in").value, "camera_info", CameraInfo),
            (self.get_parameter("imu_in").value, "imu/data", Imu),
        ]
        for src, dst, msg_type in bridges:
            self._bridge(msg_type, src, dst)

        self.get_logger().info("camera_bridge up (ZED → generic camera contract).")

    def _bridge(self, msg_type, src: str, dst: str) -> None:
        pub = self.create_publisher(msg_type, dst, _CONTRACT_QOS)
        # Forward the message object unchanged; bind pub via default arg.
        self.create_subscription(
            msg_type, src, lambda msg, p=pub: p.publish(msg), qos_profile_sensor_data
        )


def main() -> None:
    rclpy.init()
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
