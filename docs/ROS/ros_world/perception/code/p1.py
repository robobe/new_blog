import numpy as np
import matplotlib.pyplot as plt

# ----------------------------
# 1. Define the camera intrinsics
# ----------------------------
fx = 800  # focal length in pixels (x-axis)
fy = 800  # focal length in pixels (y-axis)
cx = 320  # principal point x (image center for 640 width)
cy = 240  # principal point y (image center for 480 height)

K = np.array([
    [fx,  0, cx],
    [0,  fy, cy],
    [0,   0,  1]
])

# ----------------------------
# 2. Define a 3D point in camera coordinates (X, Y, Z)
# x right, y down, z forward
#    Note: Z must be > 0 (in front of camera)
# ----------------------------
P_camera = np.array([0.0, 0.0, 15])  # meters
P_camera1 = np.array([1.0, 0.0, 15])  # meters
P_camera2 = np.array([-1.0, 0.0, 15])  # meters

# ----------------------------
# 3. Project point to pixel coordinates
# ----------------------------
u_data = []
v_data = []
for P in [P_camera, P_camera1, P_camera2]:
    if P[2] <= 0:
        raise ValueError("Z coordinate must be positive (point in front of camera).")
    p_homogeneous = K @ P  # multiply intrinsic matrix
    u = p_homogeneous[0] / p_homogeneous[2]
    v = p_homogeneous[1] / p_homogeneous[2]

    print(f"Pixel coordinates: ({u:.2f}, {v:.2f})")
    u_data.append(u)
    v_data.append(v)

# ----------------------------
# 4. Visualize with matplotlib
# ----------------------------
img_width, img_height = 640, 480

fig, ax = plt.subplots()
ax.set_xlim(0, img_width)
ax.set_ylim(img_height, 0)  # invert y-axis for image coords
ax.set_aspect('equal')
ax.grid(True)

# draw image boundary
rect = plt.Rectangle((0, 0), img_width, img_height, fill=False, color='black')
ax.add_patch(rect)
print(u_data, v_data)
# plot the projected point
ax.plot(u_data, v_data, 'ro', markersize=8)
ax.set_title('Projected Point on Camera Plane')
ax.set_xlabel('u (pixels)')
ax.set_ylabel('v (pixels)')

plt.show()
