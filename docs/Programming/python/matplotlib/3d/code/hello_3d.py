import numpy as np
import matplotlib.pyplot as plt

fig, axs = plt.subplots(2, 2, figsize=(4, 3), layout='constrained', subplot_kw={'projection': '3d'})

x = [0, 3]
y = [0, 2]
z = [0, 5]

# 3D line segment
ax = axs[0, 0]
ax.plot(x, y, z, linewidth=2)  # 3D line
ax.set_xlabel('X') 
ax.set_ylabel('Y') 
ax.set_zlabel('Z')
ax.set_title('3D line segment')

# scatter plot
ax = axs[0, 1]
ax.scatter(x, y, z, color='r')  # 3D scatter
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D scatter plot')

# scatter quiver
x, y, z = [0], [0], [0]
u, v, w = 1, 0, 0
ax = axs[1, 0]
ax.quiver(x[0], y[0], z[0], u, v, w)
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_title('3D quiver')

plt.show()
