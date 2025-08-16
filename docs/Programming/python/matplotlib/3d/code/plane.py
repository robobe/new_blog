import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # registers 3D projection

# Make a grid over X,Y
x = np.linspace(-5, 5, 2)
y = np.linspace(-5, 5, 2)
X, Y = np.meshgrid(x, y)
print(X, Y)
Z = np.zeros_like(X)  # XY plane -> z = 0 everywhere

# Plot
fig = plt.figure(figsize=(7, 5))
ax = fig.add_subplot(111, projection="3d")
ax.plot_surface(X, Y, Z, alpha=0.4, edgecolor='k')  # semi-transparent plane

ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
ax.set_title("XY plane (z = 0)")
ax.set_xlim(-5, 5); ax.set_ylim(-5, 5); ax.set_zlim(-3, 3)

plt.tight_layout()
plt.show()
