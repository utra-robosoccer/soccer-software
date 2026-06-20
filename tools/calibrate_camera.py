#!/usr/bin/env python3
"""Monocular camera intrinsics calibration helper (dev tool).

Thin wrapper around OpenCV's chessboard calibration to produce the `fx, fy, cx,
cy` used by `soccer_perception/camera_model.py`. Point it at a folder of
chessboard images captured from the MiniBot camera.

    python tools/calibrate_camera.py --images calib/*.jpg --cols 9 --rows 6 --square 0.025
"""
from __future__ import annotations

import argparse
import glob

import cv2
import numpy as np


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True, help="glob of calibration images")
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=0.025, help="square size (m)")
    args = ap.parse_args()

    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square

    obj_points, img_points = [], []
    shape = None
    for path in sorted(glob.glob(args.images)):
        img = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        shape = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows))
        if found:
            obj_points.append(objp)
            img_points.append(corners)
            print(f"  + {path}")
        else:
            print(f"  - {path} (no chessboard)")

    if not obj_points:
        raise SystemExit("No chessboards found — check --cols/--rows and images.")

    _, K, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, shape, None, None)
    print("\nfx, fy, cx, cy =", K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    print("distortion =", dist.ravel())


if __name__ == "__main__":
    main()
