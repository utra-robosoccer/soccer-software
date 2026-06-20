"""GameController bridge node (L5, blueprint §7).

Listens for the GameController broadcast on UDP 3838, publishes a clean
``gc/game_state`` (``soccer_msgs/GameState``) that the whole namespaced stack
subscribes to, and sends the mandatory status return on 3939 so the GC does not
flag this robot as dead. It is the ultimate authority: a non-PLAYING state or a
non-zero penalty forces the strategy to halt.
"""
from __future__ import annotations

import socket

import rclpy
from rclpy.node import Node
from soccer_msgs.msg import GameState

from game_controller_bridge import gc_protocol as gc

_STATE_MAP = {
    gc.STATE_INITIAL: GameState.GAMESTATE_INITIAL,
    gc.STATE_READY: GameState.GAMESTATE_READY,
    gc.STATE_SET: GameState.GAMESTATE_SET,
    gc.STATE_PLAYING: GameState.GAMESTATE_PLAYING,
    gc.STATE_FINISHED: GameState.GAMESTATE_FINISHED,
}


class GcBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("gc_bridge_node")
        self.declare_parameter("team_number", 1)
        self.declare_parameter("player_id", 1)
        self.declare_parameter("data_port", gc.DATA_PORT)
        self.declare_parameter("return_port", gc.RETURN_PORT)
        self._team = int(self.get_parameter("team_number").value)
        self._player = int(self.get_parameter("player_id").value)
        self._return_port = int(self.get_parameter("return_port").value)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._sock.bind(("0.0.0.0", int(self.get_parameter("data_port").value)))
        self._sock.setblocking(False)
        self._gc_addr = None  # learned from the first received packet

        self._pub = self.create_publisher(GameState, "gc/game_state", 10)
        self.create_timer(0.02, self._poll)        # 50 Hz socket drain
        self.create_timer(1.0, self._send_return)   # 1 Hz status return
        self.get_logger().info(
            f"gc_bridge_node up (team {self._team}, player {self._player})."
        )

    def _poll(self) -> None:
        # Drain all queued datagrams; keep only the freshest control packet.
        latest = None
        while True:
            try:
                data, addr = self._sock.recvfrom(2048)
            except BlockingIOError:
                break
            parsed = gc.parse_control_data(data)
            if parsed is not None:
                latest = parsed
                self._gc_addr = addr[0]
        if latest is not None:
            self._publish(latest)

    def _publish(self, c) -> None:
        msg = GameState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.gamestate = _STATE_MAP.get(c.state, GameState.GAMESTATE_INITIAL)
        msg.first_half = bool(c.first_half)
        msg.seconds_remaining = int(c.secs_remaining)

        # Penalty for *this* robot (penalty != 0 overrides everything).
        penalized = False
        for team in c.teams:
            if team.team_number == self._team and 1 <= self._player <= len(team.players):
                penalized = team.players[self._player - 1].penalty != gc.PENALTY_NONE
                msg.own_score = team.score
            else:
                msg.rival_score = team.score
        msg.penalized = penalized
        self._pub.publish(msg)

    def _send_return(self) -> None:
        if self._gc_addr is None:
            return
        packet = gc.build_return_data(self._team, self._player, gc.MSG_ALIVE)
        try:
            self._sock.sendto(packet, (self._gc_addr, self._return_port))
        except OSError as exc:  # network blip; not fatal
            self.get_logger().warn(f"return send failed: {exc}")


def main() -> None:
    rclpy.init()
    node = GcBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
