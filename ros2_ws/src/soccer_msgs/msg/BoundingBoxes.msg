# All detections from one camera frame. Header carries the camera frame_id +
# stamp so downstream projection can look up the correct TF.
std_msgs/Header header
BoundingBox[] bounding_boxes
