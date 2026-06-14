"""Round-trip tests for the GameController wire protocol."""
from game_controller_bridge import gc_protocol as gc


def test_control_roundtrip_preserves_state_and_penalty():
    c = gc.ControlData(state=gc.STATE_PLAYING, secs_remaining=512)
    c.teams[0].team_number = 7
    c.teams[0].score = 2
    c.teams[0].players[2].penalty = 5  # player 3 penalized
    raw = gc.pack_control_data(c, packet_number=42)

    parsed = gc.parse_control_data(raw)
    assert parsed is not None
    assert parsed.state == gc.STATE_PLAYING
    assert parsed.secs_remaining == 512
    assert parsed.teams[0].team_number == 7
    assert parsed.teams[0].score == 2
    assert parsed.teams[0].players[2].penalty == 5
    assert parsed.teams[0].players[0].penalty == gc.PENALTY_NONE


def test_parse_rejects_garbage():
    assert gc.parse_control_data(b"not a gc packet") is None


def test_return_packet_has_correct_header():
    pkt = gc.build_return_data(team=3, player=2, message=gc.MSG_ALIVE)
    assert pkt[:4] == gc.RETURN_HEADER
