import numpy as np

# Given
u, v = 320, 240
width, height = 640, 480
pitch_deg = 30
C_world = np.array([0, 0, 10])

# Assume fx, fy from 90° FOV
fov_x = np.deg2rad(90)
fx = fy = width / (2 * np.tan(fov_x / 2))
cx, cy = width / 2, height / 2

# Step 1: Pixel to camera coordinates
x_c = (u - cx) / fx
y_c = (v - cy) / fy
ray_cam = np.array([x_c, y_c, 1.0])

# Step 2: Rotation matrix (pitch about x-axis)
pitch = np.deg2rad(pitch_deg)
R_x = np.array([
    [1, 0, 0],
    [0, np.cos(pitch), -np.sin(pitch)],
    [0, np.sin(pitch), np.cos(pitch)]
])

# Step 3: Ray in world coordinates
ray_world = R_x @ ray_cam
ray_world /= np.linalg.norm(ray_world)

# Step 4: Intersection with z=0 plane
lambda_ = -C_world[2] / ray_world[2]
P_world = C_world + lambda_ * ray_world

print("World point at z=0:", P_world)
