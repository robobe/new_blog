import numpy as np

# Camera parameters
width, height = 640, 480
fov_x = np.deg2rad(50)
fx = fy = width / (2 * np.tan(fov_x / 2))
cx, cy = width / 2, height / 2

K = np.array([
    [fx, 0, cx],
    [0, fy, cy],
    [0,  0,  1]
])

# Camera pose in world
C_world = np.array([0, 0, 100])
pitch_deg = 20
pitch = np.deg2rad(pitch_deg)

# Rotation world->camera
R_x = np.array([
    [1, 0, 0],
    [0, np.cos(-pitch), -np.sin(-pitch)],
    [0, np.sin(-pitch),  np.cos(-pitch)]
])

# Translation
t = -R_x @ C_world

# Example world point (z=0)
P_world = np.array([5, 0, 0])  # <-- change to your point

# Transform to camera coordinates
P_cam = R_x @ P_world + t

# Project to pixels
p_pixel = K @ P_cam
u = p_pixel[0] / p_pixel[2]
v = p_pixel[1] / p_pixel[2]

print("Pixel coordinates:", (u, v))
