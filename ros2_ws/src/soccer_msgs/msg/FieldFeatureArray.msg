# A bundle of field features from one camera frame, consumed by the MCL node.
# The line points form the "point cloud" that is weighted against the
# precomputed likelihood-field map.
std_msgs/Header header
FieldFeature[] features
