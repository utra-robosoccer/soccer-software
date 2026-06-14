# A single 2D image-space detection produced by the perception detector
# (RF-DETR / YOLO). The base point is used for accurate 3D ground projection:
# for a ball it is the contact point, for a goalpost it is where it meets the floor.

float64 probability     # detector confidence in [0, 1]

int64 xmin
int64 ymin
int64 xmax
int64 ymax

# Ground-contact pixel used for 3D projection (see soccer_perception projection).
int64 xbase
int64 ybase

int16 id                # stable track id (-1 if untracked)
string class_id         # "ball" | "goalpost" | "robot"
