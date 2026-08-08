#!/usr/bin/env python3
"""Estimate camera depth from an object's known and observed widths."""

import math


def estimate_distance(
    real_width_m: float,
    object_width_px: float,
    image_width_px: int,
    horizontal_fov_deg: float,
) -> float:
    """Estimate optical-axis depth from a known object width."""
    if real_width_m <= 0:
        raise ValueError("Real width must be greater than zero")
    if object_width_px <= 0:
        raise ValueError("Object pixel width must be greater than zero")
    if not 0 < horizontal_fov_deg < 180:
        raise ValueError("Horizontal FOV must be between 0 and 180 degrees")

    fov_rad = math.radians(horizontal_fov_deg)
    focal_length_px = image_width_px / (2.0 * math.tan(fov_rad / 2.0))
    return real_width_m * focal_length_px / object_width_px


distance = estimate_distance(
    real_width_m=0.5,
    object_width_px=200,
    image_width_px=1920,
    horizontal_fov_deg=90,
)
print(f"Estimated distance: {distance:.2f} m")
