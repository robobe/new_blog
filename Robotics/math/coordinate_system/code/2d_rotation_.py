import numpy as np
import matplotlib.pyplot as plt

# Set up an array of points
x_points = np.array([2, 2, 0.5, -1, -1, 2])
y_points = np.array([-1, 2, 3, 2, -1, -1])

points = np.vstack((x_points, y_points))

# Rotation matrix
theta = 30
theta_rad = np.deg2rad(theta)

rot_mat = np.array([
    [np.cos(theta_rad), -np.sin(theta_rad)],
    [np.sin(theta_rad),  np.cos(theta_rad)]
])

rot_pts = rot_mat @ points

# Plot
plt.figure()

plt.plot(0, 0, '+k', label="Origin")
plt.grid(True)

plt.plot(points[0, :], points[1, :], 'x-k', label="Original Points")
plt.plot(rot_pts[0, :], rot_pts[1, :], 'x-r', label="Rotated Points")

plt.legend()
plt.axis('equal')

plt.show()