#!/usr/bin/env python3
"""Mock RoboCup GameController (test tool).

Broadcasts ``HlRoboCupGameControlData`` on UDP 3838 at 2 Hz so the
``game_controller_bridge`` on each robot transitions the team into PLAYING (or
any state you pick) without the real GameController. Reuses the SAME wire-format
module as the bridge, so the round-trip is faithful.

    python tools/mock_gamecontroller.py --state playing --rate 2
    python tools/mock_gamecontroller.py --state set --penalize 2   # penalize player 2
"""
from __future__ import annotations

import argparse
import os
import socket
import sys
import time

# Import the shared protocol straight from the ROS package source.
sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "game_controller_bridge")
)
from game_controller_bridge import gc_protocol as gc  # noqa: E402

_STATES = {
    "initial": gc.STATE_INITIAL,
    "ready": gc.STATE_READY,
    "set": gc.STATE_SET,
    "playing": gc.STATE_PLAYING,
    "finished": gc.STATE_FINISHED,
}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", choices=_STATES, default="playing")
    ap.add_argument("--rate", type=float, default=2.0, help="broadcast rate (Hz)")
    ap.add_argument("--team", type=int, default=1)
    ap.add_argument("--penalize", type=int, default=0, help="penalize this player id (0=none)")
    ap.add_argument("--address", default="255.255.255.255")
    args = ap.parse_args()

    control = gc.ControlData(state=_STATES[args.state], secs_remaining=600)
    control.teams[0].team_number = args.team
    control.teams[1].team_number = args.team + 1
    if args.penalize:
        control.teams[0].players[args.penalize - 1].penalty = 5

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    print(f"Broadcasting {args.state.upper()} on :{gc.DATA_PORT} @ {args.rate} Hz "
          f"(Ctrl-C to stop)")
    pkt = 0
    period = 1.0 / args.rate
    try:
        while True:
            sock.sendto(gc.pack_control_data(control, pkt), (args.address, gc.DATA_PORT))
            pkt = (pkt + 1) & 0xFF
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nstopped.")


if __name__ == "__main__":
    main()
