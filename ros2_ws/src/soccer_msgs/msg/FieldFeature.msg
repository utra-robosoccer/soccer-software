# A static field landmark observation used as a measurement by the Tier-2 MCL
# particle filter (localization_strategy_report §3.1). These are the asymmetric
# anchors that disambiguate the symmetric field: lines, junctions, goalposts,
# and the center circle.

uint8 TYPE_LINE_POINT = 0    # a point sampled along a white field line
uint8 TYPE_L_JUNCTION = 1
uint8 TYPE_T_JUNCTION = 2
uint8 TYPE_X_JUNCTION = 3
uint8 TYPE_GOALPOST = 4
uint8 TYPE_CENTER_CIRCLE = 5
uint8 type

# Position in the robot base frame (metres), back-projected with ZED depth
# (or the flat-ground homography fallback).
geometry_msgs/Point position

float64 confidence
