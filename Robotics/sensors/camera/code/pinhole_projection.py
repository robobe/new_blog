#!/usr/bin/env python3
"""Project a world point to a pixel and recover its camera ray."""

import numpy as np


def main() -> None:
    # Robot world axes: X forward, Y left, Z up.
    # OpenCV camera axes: X right, Y down, Z forward.
    camera_position_w = np.array([1.0, 0.0, 1.0])
    rotation_cw = np.array(
        [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]
    )

    # Intrinsics for a 640 x 480 image.
    intrinsic = np.array(
        [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]]
    )
    point_w = np.array([5.0, -1.0, 0.0])

    # World -> camera: subtract the camera position, then rotate.
    point_c = rotation_cw @ (point_w - camera_position_w)
    if point_c[2] <= 0:
        raise ValueError("The point is behind the camera")

    normalized = point_c / point_c[2]
    homogeneous_pixel = intrinsic @ normalized
    pixel = homogeneous_pixel[:2] / homogeneous_pixel[2]

    # Pixel -> ray: K^-1 recovers a direction, but not the original depth.
    ray_c = np.linalg.inv(intrinsic) @ np.array([pixel[0], pixel[1], 1.0])
    ray_c /= np.linalg.norm(ray_c)

    print(f"World point:       {point_w}")
    print(f"Camera point:      {point_c}")
    print(f"Normalized point:  {normalized[:2]}")
    print(f"Pixel (u, v):      {pixel}")
    print(f"Unit camera ray:   {ray_c}")
    print("A depth value is still required to recover a unique 3D point.")


if __name__ == "__main__":
    main()
