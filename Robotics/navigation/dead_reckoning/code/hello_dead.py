from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import c4dynamics as c4d


np.random.seed(7)  # Reproducible sensor noise

dt = 0.01
time = np.arange(0.0, 10.0, dt)

robot = c4d.rigidbody()
imu = c4d.sensors.imu(
    acc_std=0.02,                         # accelerometer noise [m/s²]
    gyro_std=0.005,                       # gyroscope noise [rad/s]
    acc_bias=[0.01, -0.008, 0.0],         # acceleration bias creates position drift
    gyro_bias=[0.0, 0.0, np.deg2rad(0.2)],  # yaw-rate bias creates yaw drift
    dt=dt,
)

# Dead reckoning starts at the known initial state.
dead_x = dead_y = dead_vx = dead_vy = dead_yaw = 0.0
true_x, true_y, true_yaw = [], [], []
estimated_x, estimated_y, estimated_yaw = [], [], []

for t in time:
    # Accelerate, then follow a constant-speed turn.
    speed = min(t, 2.0)
    robot.r = 0.0 if t < 2.0 else np.deg2rad(18.0)
    robot.psi += robot.r * dt
    robot.vx = speed * np.cos(robot.psi)
    robot.vy = -speed * np.sin(robot.psi)  # C4Dynamics navigation axes
    robot.x += robot.vx * dt
    robot.y += robot.vy * dt

    ax, ay, _az, _p, _q, measured_r = imu.measure(robot, t=t)

    dead_yaw += measured_r * dt
    c, s = np.cos(dead_yaw), np.sin(dead_yaw)
    world_ax = c * ax - s * ay
    world_ay = -s * ax - c * ay
    dead_vx += world_ax * dt
    dead_vy += world_ay * dt
    dead_x += dead_vx * dt
    dead_y += dead_vy * dt

    true_x.append(robot.x)
    true_y.append(robot.y)
    true_yaw.append(robot.psi)
    estimated_x.append(dead_x)
    estimated_y.append(dead_y)
    estimated_yaw.append(dead_yaw)

fig, (trajectory, yaw_plot) = plt.subplots(1, 2, figsize=(12, 5))
trajectory.plot(true_x, true_y, label="Ground truth", linewidth=2)
trajectory.plot(estimated_x, estimated_y, "--", label="Dead reckoning")

# Add heading arrows to both paths.
step = len(time) // 12
for x, y, yaw, color in (
    (true_x, true_y, true_yaw, "C0"),
    (estimated_x, estimated_y, estimated_yaw, "C1"),
):
    trajectory.quiver(
        x[::step], y[::step], np.cos(yaw[::step]), -np.sin(yaw[::step]),
        color=color, angles="xy", scale_units="xy", scale=2.5, width=0.006,
    )

trajectory.set(xlabel="x [m]", ylabel="y [m]", title="Trajectory and yaw")
trajectory.axis("equal")
trajectory.grid()
trajectory.legend()

yaw_plot.plot(time, np.rad2deg(true_yaw), label="Ground truth")
yaw_plot.plot(time, np.rad2deg(estimated_yaw), "--", label="Dead reckoning")
yaw_plot.set(xlabel="time [s]", ylabel="yaw [deg]", title="Yaw drift")
yaw_plot.grid()
yaw_plot.legend()

fig.tight_layout()
fig.savefig(Path(__file__).parent.parent / "images/dead_reckoning.png", dpi=150)
plt.show()
