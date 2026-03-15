import numpy as np
import matplotlib.pyplot as plt


v = np.array([3, 2])

theta = np.deg2rad(30)

R = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta),  np.cos(theta)]
])

v_rot = R @ v

print(v_rot)

plt.quiver(0,0,v[0],v[1],angles='xy',scale_units='xy',scale=1,color='green')
plt.quiver(0,0,v_rot[0],v_rot[1],angles='xy',scale_units='xy',scale=1,color='red')

plt.xlim(-5,5)
plt.ylim(-5,5)
plt.grid()
plt.gca().set_aspect('equal')

plt.show()