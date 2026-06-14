# High-level command handed from Strategy (L4 Behavior Tree) down to Control
# (L1 MPC + residual RL). Strategy speaks in intent; Control turns intent into a
# joint reference trajectory for the single neck_pan motor.

std_msgs/Header header

uint8 MODE_IDLE = 0        # hold current position (penalized / halt)
uint8 MODE_SCAN = 1        # sweep the camera to search for the ball / localize
uint8 MODE_TRACK = 2       # point the camera at a target bearing
uint8 mode

# Target bearing for MODE_TRACK, in radians in the base frame
# (e.g. the bearing to the ball). Ignored for IDLE/SCAN.
float64 target_bearing
