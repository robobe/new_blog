import numpy as np
import matplotlib.pyplot as plt

# Camera pose
C_world = np.array([0, 0, 10])
pitch_deg = 80
theta = np.deg2rad(pitch_deg)

# Rotation matrix for pitch about x-axis
R_y = np.array([
    [np.cos(theta), 0, np.sin(theta)],
    [0, 1, 0],
    [-np.sin(theta), 0, np.cos(theta)]
])

# Camera axes in camera frame
camera_forward = np.array([0, 0, 1])  # Z-forward in camera coords
camera_right   = np.array([1, 0, 0])  # X-right in camera coords
camera_up      = np.array([0, -1, 0]) # Y-up (adjusted for image coords)

# Rotate to world frame
forward_world = R_y @ camera_forward
right_world   = R_y @ camera_right
up_world      = R_y @ camera_up

# Plot setup
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot world origin axes
ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='World X')
ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='World Y')
ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='World Z')

# Plot camera position
ax.scatter(*C_world, color='k', s=50, label='Camera')

# Plot camera axes
scale = 2
ax.quiver(*C_world, *(forward_world*scale), color='b', linewidth=2, label='Camera forward')
ax.quiver(*C_world, *(right_world*scale),   color='r', linewidth=2, label='Camera right')
ax.quiver(*C_world, *(up_world*scale),      color='g', linewidth=2, label='Camera up')

# Labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Camera in World Coordinates')

# Set fixed limits
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.set_zlim([0, 10])
# ax.set_box_aspect([1, 1, 1])

# ax.legend()
plt.show()
