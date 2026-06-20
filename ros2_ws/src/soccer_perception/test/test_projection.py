"""Unit tests for the camera projection math (no ROS required)."""
import numpy as np

from soccer_perception.camera_model import (
    PinholeCamera,
    project_flat_ground,
    project_with_depth,
)


def _cam() -> PinholeCamera:
    return PinholeCamera(fx=550.0, fy=550.0, cx=320.0, cy=240.0, width=640, height=480,
                         mount_height=0.30, tilt=0.35)


def test_depth_projection_centre_pixel():
    cam = _cam()
    p = project_with_depth(cam, cam.cx, cam.cy, 2.0)
    # Centre pixel projects straight ahead at the given depth.
    assert np.isclose(p[0], 0.0)
    assert np.isclose(p[1], 0.0)
    assert np.isclose(p[2], 2.0)


def test_flat_ground_returns_point_in_front():
    cam = _cam()
    # A pixel below the principal point looks downward -> hits the ground ahead.
    p = project_flat_ground(cam, cam.cx, cam.height - 1)
    assert p is not None
    assert p[0] > 0.0  # in front of the robot
    assert np.isclose(p[2], 0.0)  # on the floor


def test_flat_ground_above_horizon_is_none():
    cam = _cam()
    # A pixel well above the principal point points up -> no ground hit.
    assert project_flat_ground(cam, cam.cx, 0) is None
