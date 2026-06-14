# Clean game state published by game_controller_bridge after parsing UDP :3838.
# This is the ULTIMATE AUTHORITY (blueprint §7): HALT/PENALIZED overrides all AI.
std_msgs/Header header

uint8 GAMESTATE_INITIAL=0
uint8 GAMESTATE_READY=1
uint8 GAMESTATE_SET=2
uint8 GAMESTATE_PLAYING=3
uint8 GAMESTATE_FINISHED=4
uint8 gamestate

# True when this robot is penalized (must freeze regardless of gamestate).
bool penalized

bool first_half
uint8 own_score
uint8 rival_score

# Seconds remaining in the current half (informational).
int32 seconds_remaining
