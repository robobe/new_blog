# 3D Viewer for Sensor Fusion Example using PyVista
# pip install pyvista scipy numpy

import pyvista as pv
import numpy as np
from scipy.spatial.transform import Rotation
import time

plotter = pv.Plotter()

# -------------------
# World coordinate frame
# -------------------
world_axes = pv.Axes(show_actor=True)
plotter.add_actor(world_axes.actor)

# -------------------
# Body cube
# -------------------
cube = pv.Cube(center=(0,0,0), x_length=0.5, y_length=0.5, z_length=0.5)
cube_actor = plotter.add_mesh(cube, color="lightblue", opacity=0.7)

# -------------------
# Body coordinate frame
# -------------------
origin = np.array([0,0,0])

x_axis = pv.Line(origin, [1,0,0])
y_axis = pv.Line(origin, [0,1,0])
z_axis = pv.Line(origin, [0,0,1])

body_x = plotter.add_mesh(x_axis, color="red", line_width=5)
body_y = plotter.add_mesh(y_axis, color="green", line_width=5)
body_z = plotter.add_mesh(z_axis, color="blue", line_width=5)

plotter.show(auto_close=False)

angle = 0

while True:

    angle += 1

    r = Rotation.from_euler("xyz", [angle, angle/2, angle/3], degrees=True)

    R = r.as_matrix()

    T = np.eye(4)
    T[:3,:3] = R

    # Apply transform to body elements
    cube_actor.user_matrix = T
    body_x.user_matrix = T
    body_y.user_matrix = T
    body_z.user_matrix = T

    plotter.render()

    time.sleep(0.02)