#!/usr/bin/env python3
"""Project a 3D point with OpenCV's projectPoints function."""

import cv2
import numpy as np


object_points = np.array([[5.0, -1.0, 0.0]], dtype=np.float64)
camera_matrix = np.array(
    [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]]
)

# OpenCV transforms a world point as Xc = Rcw * Xw + t, where t = -Rcw*Cw.
rotation_cw = np.array(
    [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]
)
camera_position_w = np.array([1.0, 0.0, 1.0], dtype=np.float64)
rotation_vector, _ = cv2.Rodrigues(rotation_cw)
translation_vector = -rotation_cw @ camera_position_w
distortion = np.zeros(5, dtype=np.float64)

image_points, _ = cv2.projectPoints(
    object_points,
    rotation_vector,
    translation_vector,
    camera_matrix,
    distortion,
)
print(f"Pixel (u, v): {image_points.reshape(-1, 2)[0]}")
