"""Team communication node (L4, blueprint §8).

Assembles this robot's contribution to the shared world model — self pose (from
the MCL), ball estimate (from projection), and current role bid (from strategy)
— into one ``TeamData`` packet and publishes it on the GLOBAL ``/team_data``
topic at a modest rate. Every robot subscribes to ``/team_data``, so the role
auction in ``soccer_strategy`` sees all bids and converges to the same
assignment with NO master.

On a multi-machine fleet the global topic is carried by DDS over 5 GHz Wi-Fi
(CycloneDDS); per-team Domain ID isolation keeps opponents from cross-talking
(blueprint §8). Within a single simulation graph the global topic already shares
the data, so the same node works in both settings unchanged.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from soccer_msgs.msg import RoleBid, TeamData


class TeamCommNode(Node):
    def __init__(self) -> None:
        super().__init__("teamcomm_node")
        self.declare_parameter("player_id", 1)
        self.declare_parameter("rate_hz", 4.0)  # blueprint §8: lightweight, ~few Hz
        self._player_id = int(self.get_parameter("player_id").value)
        rate = float(self.get_parameter("rate_hz").value)

        self._data = TeamData()
        self._data.player_id = self._player_id
        self._data.status = TeamData.STATUS_ACTIVE
        self._data.bid = RoleBid(role=RoleBid.ROLE_UNASSIGNED, cost=1e6)

        # Best-effort, like a real UDP team channel.
        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._pub = self.create_publisher(TeamData, "/team_data", qos)

        # Gather local contributions (namespaced topics within this robot).
        self.create_subscription(PoseWithCovarianceStamped, "mcl_pose", self._on_pose, 5)
        self.create_subscription(PointStamped, "ball/point", self._on_ball, 5)
        self.create_subscription(RoleBid, "strategy/role_bid", self._on_bid, 5)

        self.create_timer(1.0 / rate, self._broadcast)
        self.get_logger().info(f"teamcomm_node up (player {self._player_id}).")

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._data.pose.x = msg.pose.pose.position.x
        self._data.pose.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._data.pose.theta = 2.0 * math.atan2(q.z, q.w)
        self._data.localized = True

    def _on_ball(self, msg: PointStamped) -> None:
        self._data.ball_pose.x = msg.point.x
        self._data.ball_pose.y = msg.point.y
        self._data.ball_detected = True
        self._data.ball_confidence = 1.0

    def _on_bid(self, msg: RoleBid) -> None:
        self._data.bid = msg

    def _broadcast(self) -> None:
        self._data.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._data)
        # Ball/role freshness is re-asserted by incoming messages; reset the flag
        # so a stale ball is not advertised forever.
        self._data.ball_detected = False


def main() -> None:
    rclpy.init()
    node = TeamCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
