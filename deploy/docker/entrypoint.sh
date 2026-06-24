#!/usr/bin/env bash
# Source ROS + whichever workspace this image built, then exec the container
# command. The Jetson lineage (deploy/docker/Dockerfile.jetson) builds the ZED
# wrapper into /ros2_ws and the soccer workspace into /ws, so check both.
set -e
source /opt/ros/jazzy/setup.bash
for ws in /ws/install /ros2_ws/install; do
  if [ -f "$ws/setup.bash" ]; then
    source "$ws/setup.bash"
  fi
done
exec "$@"
