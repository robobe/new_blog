import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# --- Angles (degrees) ---
roll, pitch, yaw = 90, 45, 180

# --- Intrinsic rotation (body axes: roll -> pitch -> yaw) ---
rot_intrinsic = R.from_euler("zyx", [yaw, pitch, roll], degrees=True)
zyx_intrinsic = rot_intrinsic.as_matrix()

rot_intrinsic2 = R.from_euler("xyz", [roll, pitch, yaw], degrees=True)
xyz_intrinsic = rot_intrinsic2.as_matrix()

# --- Extrinsic rotation (world axes: roll -> pitch -> yaw) ---
rot_extrinsic = R.from_euler("ZYX", [yaw, pitch, roll], degrees=True)
ZYX_extrinsic = rot_extrinsic.as_matrix()

rot_extrinsic2 = R.from_euler("XYZ", [roll, pitch, yaw], degrees=True)
XYZ_extrinsic = rot_extrinsic2.as_matrix()

# --- Helper plot function ---
def plot_axes(R, ax, title):
    origin = np.zeros(3)
    x_axis = R @ np.array([1, 0, 0])
    y_axis = R @ np.array([0, 1, 0])
    z_axis = R @ np.array([0, 0, 1])

    ax.quiver(*origin, *x_axis, color="r", label="X")
    ax.quiver(*origin, *y_axis, color="g", label="Y")
    ax.quiver(*origin, *z_axis, color="b", label="Z")

    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(title)
    ax.legend()

# --- Plot both ---
fig = plt.figure(figsize=(12,6))

# Vector
R0 = np.eye(3)

# Rotate vector
v_intrinsic = zyx_intrinsic @ R0
v_extrinsic = ZYX_extrinsic @ R0

ax1 = fig.add_subplot(221, projection="3d")
plot_axes(v_intrinsic, ax1, "zyx (body frame)")
ax1.get_legend().remove() 

ax1 = fig.add_subplot(223, projection="3d")
plot_axes(xyz_intrinsic, ax1, "xyz")
ax1.get_legend().remove() 

ax2 = fig.add_subplot(222, projection="3d")
plot_axes(v_extrinsic, ax2, "ZYX")
ax2.get_legend().remove()

ax2 = fig.add_subplot(224, projection="3d")
plot_axes(XYZ_extrinsic, ax2, "XYZ ()")
ax2.get_legend().remove()
# plt.tight_layout()
plt.show()

# --- Print matrices ---
np.set_printoptions(precision=4, suppress=True)
print("Intrinsic Rotation Matrix:\n", zyx_intrinsic)
print("\nExtrinsic Rotation Matrix:\n", ZYX_extrinsic)
