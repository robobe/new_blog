import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

# Define two orientations (Euler angles in degrees)
r0 = R.from_euler('xyz', [0, 0, 0], degrees=True)     # facing forward
r1 = R.from_euler('xyz', [0, 90, 0], degrees=True)    # rotate 90° around Y

# Define key times and rotations
key_times = [0, 1]              # start=0, end=1
key_rots = R.concatenate([r0, r1])  # list of rotations

# Create SLERP object
slerp = Slerp(key_times, key_rots)

# Interpolation at multiple times
times = np.linspace(0, 1, 5)    # t=0.0,0.25,0.5,0.75,1.0
interp_rots = slerp(times)

# Print quaternions
print("Interpolated quaternions [x, y, z, w]:")
for q in interp_rots.as_quat():
    print(q.round(3))
