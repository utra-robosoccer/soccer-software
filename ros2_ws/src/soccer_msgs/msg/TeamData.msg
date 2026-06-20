# Decentralized team-communication packet broadcast by every robot over DDS
# (blueprint §8, RoboCup RobocupProtocol style). Carries the robot's piece of
# the shared world model plus its current role bid for the auction.

std_msgs/Header header

uint8 player_id

uint8 STATUS_INACTIVE = 0
uint8 STATUS_ACTIVE = 1
uint8 STATUS_PENALIZED = 2
uint8 status

# Self pose on the field (map frame), from the Tier-2 MCL.
geometry_msgs/Pose2D pose
bool localized

# Ball estimate in the map frame; ball_detected=false if this robot can't see it.
geometry_msgs/Pose2D ball_pose
bool ball_detected
float64 ball_confidence

# This robot's current bid in the role auction.
RoleBid bid
