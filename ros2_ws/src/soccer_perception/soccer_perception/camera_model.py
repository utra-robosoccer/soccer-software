"""Camera projection helpers.

Two ways to turn a pixel into a 3D point in the camera frame:

* :func:`project_with_depth` — preferred. Uses the ZED stereo *depth* at the
  pixel, giving an accurate 3D point even for objects above the ground. This
  replaces the fragile flat-ground assumption (localization report §3.2, §6).
* :func:`project_flat_ground` — fallback. The classic monocular pinhole +
  flat-ground homography (`find_floor_coordinate` in the legacy repo): intersect
  the pixel ray with the z=0 plane. Only valid for ground-contact points.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class PinholeCamera:
    """Minimal pinhole intrinsics."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    # Height of the optical centre above the floor (m) and downward tilt (rad),
    # used only by the flat-ground fallback.
    mount_height: float = 0.30
    tilt: float = 0.0

    def ray(self, u: float, v: float) -> np.ndarray:
        """Unit ray (camera optical frame: z fwd, x right, y down) for pixel (u, v)."""
        d = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        return d / np.linalg.norm(d)


def project_with_depth(cam: PinholeCamera, u: float, v: float, depth_m: float) -> np.ndarray:
    """Back-project a pixel to a 3D point using measured stereo depth (metres)."""
    x = (u - cam.cx) / cam.fx * depth_m
    y = (v - cam.cy) / cam.fy * depth_m
    return np.array([x, y, depth_m])


def project_flat_ground(cam: PinholeCamera, u: float, v: float) -> np.ndarray | None:
    """Intersect the pixel ray with the ground plane z=0 (camera height above floor).

    Returns the ground point in a base-aligned frame (x forward, y left, z up), or
    ``None`` if the ray points at or above the horizon.
    """
    r = cam.ray(u, v)
    # Rotate the optical-frame ray into a base-aligned frame (x fwd, y left, z up),
    # applying the fixed downward tilt of the camera.
    ct, st = np.cos(cam.tilt), np.sin(cam.tilt)
    fwd = np.array([r[2] * ct + r[1] * st, -r[0], -r[1] * ct + r[2] * st])
    if fwd[2] >= -1e-6:  # ray not pointing down -> no ground intersection
        return None
    t = cam.mount_height / -fwd[2]
    return np.array([fwd[0] * t, fwd[1] * t, 0.0])
