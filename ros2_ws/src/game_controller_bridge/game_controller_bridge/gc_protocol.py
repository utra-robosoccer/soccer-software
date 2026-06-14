"""RoboCup Humanoid GameController wire protocol (blueprint §7).

A pragmatic, self-consistent implementation of the GameController packets:

* **Control** (GC -> robots), UDP broadcast on **3838** @ 2 Hz, header ``b"RGme"``.
* **Return** (robots -> GC), UDP unicast on **3939** @ ~1 Hz, header ``b"RGrt"``.

``pack_control_data`` / ``parse_control_data`` round-trip, so the same module
backs both the bridge node and the mock GameController in ``tools/``. The field
layout follows the documented struct for the core control fields; the exact
struct version used at a competition must be matched before the event, but the
``GameState`` mapping the rest of the stack consumes stays identical.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field

DATA_PORT = 3838
RETURN_PORT = 3939
CONTROL_HEADER = b"RGme"
RETURN_HEADER = b"RGrt"
STRUCT_VERSION = 12
MAX_NUM_PLAYERS = 4

# Primary game states (map 1:1 to soccer_msgs/GameState).
STATE_INITIAL = 0
STATE_READY = 1
STATE_SET = 2
STATE_PLAYING = 3
STATE_FINISHED = 4

# Penalties (non-zero => robot must freeze).
PENALTY_NONE = 0

# Return-packet messages.
MSG_ALIVE = 1
MSG_GOAL = 2

_CONTROL_FMT = "<4sHBBBBBBB4sBhhh"  # header .. secondary_time
_TEAM_FMT = "<BBBBH"                  # team_number,color,score,penalty_shot,single_shots
_PLAYER_FMT = "<BB"                   # penalty, secs_till_unpenalised
_RETURN_FMT = "<4sHBBB"               # header, version, team, player, message


@dataclass
class PlayerInfo:
    penalty: int = PENALTY_NONE
    secs_till_unpenalised: int = 0


@dataclass
class TeamInfo:
    team_number: int = 0
    team_color: int = 0
    score: int = 0
    penalty_shot: int = 0
    single_shots: int = 0
    players: list = field(default_factory=lambda: [PlayerInfo() for _ in range(MAX_NUM_PLAYERS)])


@dataclass
class ControlData:
    state: int = STATE_INITIAL
    first_half: int = 1
    kick_off_team: int = 0
    secondary_state: int = 0
    secs_remaining: int = 600
    teams: list = field(default_factory=lambda: [TeamInfo(), TeamInfo()])


def pack_control_data(c: ControlData, packet_number: int = 0) -> bytes:
    buf = struct.pack(
        _CONTROL_FMT, CONTROL_HEADER, STRUCT_VERSION, packet_number & 0xFF,
        MAX_NUM_PLAYERS, 0, c.state, c.first_half, c.kick_off_team,
        c.secondary_state, b"\x00\x00\x00\x00", 0, 0, c.secs_remaining, 0,
    )
    for team in c.teams[:2]:
        buf += struct.pack(
            _TEAM_FMT, team.team_number, team.team_color, team.score,
            team.penalty_shot, team.single_shots,
        )
        for p in team.players[:MAX_NUM_PLAYERS]:
            buf += struct.pack(_PLAYER_FMT, p.penalty, p.secs_till_unpenalised)
    return buf


def parse_control_data(data: bytes) -> ControlData | None:
    head = struct.calcsize(_CONTROL_FMT)
    if len(data) < head or data[:4] != CONTROL_HEADER:
        return None
    (_h, _ver, _pkt, _ppt, _gt, state, first_half, kick_off_team,
     secondary_state, _info, _dit, _dt, secs_remaining, _st) = struct.unpack(
        _CONTROL_FMT, data[:head])
    c = ControlData(
        state=state, first_half=first_half, kick_off_team=kick_off_team,
        secondary_state=secondary_state, secs_remaining=secs_remaining, teams=[],
    )
    off = head
    tsz = struct.calcsize(_TEAM_FMT)
    psz = struct.calcsize(_PLAYER_FMT)
    for _ in range(2):
        if off + tsz > len(data):
            break
        tn, tc, score, pshot, sshots = struct.unpack(_TEAM_FMT, data[off:off + tsz])
        off += tsz
        players = []
        for _p in range(MAX_NUM_PLAYERS):
            if off + psz > len(data):
                players.append(PlayerInfo())
                continue
            pen, secs = struct.unpack(_PLAYER_FMT, data[off:off + psz])
            off += psz
            players.append(PlayerInfo(pen, secs))
        c.teams.append(TeamInfo(tn, tc, score, pshot, sshots, players))
    return c


def build_return_data(team: int, player: int, message: int = MSG_ALIVE) -> bytes:
    return struct.pack(_RETURN_FMT, RETURN_HEADER, STRUCT_VERSION, team, player, message)
