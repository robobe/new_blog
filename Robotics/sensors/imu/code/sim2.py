import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# -----------------------------
# Simulation settings
# -----------------------------
dt = 0.05
T = 20.0
steps = int(T / dt)

# Constant body-frame acceleration:
# [forward, left] in the robot body frame
a_body = np.array([1.0, 0.0])

# Constant yaw rate [rad/s]
omega = 0.4

# State in world frame
x = 0.0
y = 0.0
theta = 0.0
v_world = np.array([0.0, 0.0])

# History buffers
x_hist = []
y_hist = []
theta_hist = []
a_world_hist = []
t_hist = []

# -----------------------------
# Simulate motion
# -----------------------------
for k in range(steps):
    t = k * dt

    # Rotation matrix: body -> world
    R = np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta),  math.cos(theta)]
    ])

    # Convert acceleration from body frame to world frame
    a_world = R @ a_body

    # Integrate velocity and position in world frame
    v_world = v_world + a_world * dt
    x = x + v_world[0] * dt
    y = y + v_world[1] * dt

    # Update heading
    theta = theta + omega * dt

    # Save history
    x_hist.append(x)
    y_hist.append(y)
    theta_hist.append(theta)
    a_world_hist.append(a_world.copy())
    t_hist.append(t)

# Convert to arrays
x_hist = np.array(x_hist)
y_hist = np.array(y_hist)
theta_hist = np.array(theta_hist)
a_world_hist = np.array(a_world_hist)
t_hist = np.array(t_hist)

# -----------------------------
# Plot setup
# -----------------------------
fig, (ax_world, ax_text) = plt.subplots(
    1, 2, figsize=(12, 6), gridspec_kw={"width_ratios": [2, 1]}
)

ax_world.set_title("Body-frame vs World-frame Acceleration")
ax_world.set_xlabel("x world [m]")
ax_world.set_ylabel("y world [m]")
ax_world.grid(True)
ax_world.axis("equal")

# Initial limits
view = 6.0
ax_world.set_xlim(-view, view)
ax_world.set_ylim(-view, view)

# Path
path_line, = ax_world.plot([], [], label="Trajectory")

# Vehicle point
vehicle_point, = ax_world.plot([], [], "ro", label="Vehicle")

# Heading line
heading_line, = ax_world.plot([], [], "r-", linewidth=2, label="Body x-axis")

# Body acceleration line
body_acc_line, = ax_world.plot([], [], "--", linewidth=2, label="Body accel")

# World acceleration line
world_acc_line, = ax_world.plot([], [], "-", linewidth=2, label="World accel")

ax_world.legend(loc="upper left")

# Text panel
ax_text.axis("off")
text_box = ax_text.text(
    0.02, 0.98, "",
    va="top",
    family="monospace",
    fontsize=11
)

# -----------------------------
# Animation functions
# -----------------------------
def init():
    path_line.set_data([], [])
    vehicle_point.set_data([], [])
    heading_line.set_data([], [])
    body_acc_line.set_data([], [])
    world_acc_line.set_data([], [])
    text_box.set_text("")
    return (
        path_line,
        vehicle_point,
        heading_line,
        body_acc_line,
        world_acc_line,
        text_box,
    )

def update(frame):
    x = x_hist[frame]
    y = y_hist[frame]
    theta = theta_hist[frame]
    a_world = a_world_hist[frame]

    # Draw path so far
    path_line.set_data(x_hist[:frame + 1], y_hist[:frame + 1])

    # Vehicle point
    vehicle_point.set_data([x], [y])

    # Heading line (robot forward direction)
    heading_len = 1.0
    hx = x + heading_len * math.cos(theta)
    hy = y + heading_len * math.sin(theta)
    heading_line.set_data([x, hx], [y, hy])

    # Body acceleration line
    # Since a_body = [forward, left], [1,0] points along heading
    body_len = 1.2
    bx = x + body_len * math.cos(theta)
    by = y + body_len * math.sin(theta)
    body_acc_line.set_data([x, bx], [y, by])

    # World acceleration line
    world_scale = 1.2
    wx = x + world_scale * a_world[0]
    wy = y + world_scale * a_world[1]
    world_acc_line.set_data([x, wx], [y, wy])

    # Dynamic view: keep robot centered
    ax_world.set_xlim(x - view, x + view)
    ax_world.set_ylim(y - view, y + view)

    # Text info
    text_box.set_text(
        f"time = {t_hist[frame]:5.2f} s\n\n"
        f"theta = {theta:6.3f} rad\n"
        f"      = {math.degrees(theta):6.2f} deg\n\n"
        f"a_body  = [{a_body[0]:5.2f}, {a_body[1]:5.2f}] m/s^2\n"
        f"a_world = [{a_world[0]:5.2f}, {a_world[1]:5.2f}] m/s^2\n\n"
        f"Meaning:\n"
        f"- a_body is fixed in robot axes\n"
        f"- a_world changes with heading"
    )

    return (
        path_line,
        vehicle_point,
        heading_line,
        body_acc_line,
        world_acc_line,
        text_box,
    )

# -----------------------------
# Run animation
# -----------------------------
ani = FuncAnimation(
    fig,
    update,
    frames=len(t_hist),
    init_func=init,
    interval=dt * 1000,
    blit=False,   # important when changing axis limits dynamically
    repeat=True
)

plt.tight_layout()
plt.show()