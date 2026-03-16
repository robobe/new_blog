import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

dt = 0.05
T = 20.0
steps = int(T / dt)
g = 9.81

# Perfect circle motion
v = 1.2
omega = 0.6

# IMU bias
acc_bias = np.array([0.08, -0.05, 0.12])
gyro_bias = np.array([0.01, -0.015, 0.02])

# IMU noise std
acc_noise_std = np.array([0.05, 0.05, 0.05])
gyro_noise_std = np.array([0.01, 0.01, 0.01])

rng = np.random.default_rng(42)

x, y, theta = 0.0, 0.0, 0.0

t_hist = []
x_hist = []
y_hist = []
theta_hist = []
acc_meas_hist = []
gyro_meas_hist = []

for k in range(steps):
    t = k * dt

    # Ground truth update
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta += omega * dt

    # Ideal IMU in body frame
    acc_true = np.array([
        0.0,        # dv/dt
        v * omega,  # lateral acceleration
        -g
    ])

    gyro_true = np.array([
        0.0,
        0.0,
        omega
    ])

    # Add bias and noise
    acc_meas = acc_true + acc_bias + rng.normal(0.0, acc_noise_std)
    gyro_meas = gyro_true + gyro_bias + rng.normal(0.0, gyro_noise_std)

    t_hist.append(t)
    x_hist.append(x)
    y_hist.append(y)
    theta_hist.append(theta)
    acc_meas_hist.append(acc_meas)
    gyro_meas_hist.append(gyro_meas)

t_hist = np.array(t_hist)
x_hist = np.array(x_hist)
y_hist = np.array(y_hist)
theta_hist = np.array(theta_hist)
acc_meas_hist = np.array(acc_meas_hist)
gyro_meas_hist = np.array(gyro_meas_hist)

fig = plt.figure(figsize=(12, 8))

ax_path = fig.add_subplot(2, 2, 1)
ax_path.set_title("Ground Truth Circle Motion")
ax_path.set_xlabel("x [m]")
ax_path.set_ylabel("y [m]")
ax_path.grid(True)
ax_path.axis("equal")

margin = 1.0
ax_path.set_xlim(x_hist.min() - margin, x_hist.max() + margin)
ax_path.set_ylim(y_hist.min() - margin, y_hist.max() + margin)

path_line, = ax_path.plot([], [], label="Ground truth path")
vehicle_point, = ax_path.plot([], [], "ro", label="Vehicle")
heading_line, = ax_path.plot([], [], "r-", linewidth=2, label="Heading")
ax_path.legend()

ax_acc = fig.add_subplot(2, 2, 2)
ax_acc.set_title("Accelerometer (measured)")
ax_acc.set_xlabel("time [s]")
ax_acc.set_ylabel("m/s²")
ax_acc.grid(True)
ax_acc.set_xlim(t_hist[0], t_hist[-1])
ax_acc.set_ylim(acc_meas_hist.min() - 0.5, acc_meas_hist.max() + 0.5)

accx_line, = ax_acc.plot([], [], label="acc_x")
accy_line, = ax_acc.plot([], [], label="acc_y")
accz_line, = ax_acc.plot([], [], label="acc_z")
ax_acc.legend()

ax_vel = fig.add_subplot(2, 2, 3)
ax_vel.set_title("Velocity State")
ax_vel.axis("off")
vel_text = ax_vel.text(
    0.05, 0.95, "",
    transform=ax_vel.transAxes,
    va="top",
    ha="left",
    fontsize=12,
    family="monospace",
)

ax_gyro = fig.add_subplot(2, 2, 4)
ax_gyro.set_title("Gyroscope (measured)")
ax_gyro.set_xlabel("time [s]")
ax_gyro.set_ylabel("rad/s")
ax_gyro.grid(True)
ax_gyro.set_xlim(t_hist[0], t_hist[-1])
ax_gyro.set_ylim(gyro_meas_hist.min() - 0.1, gyro_meas_hist.max() + 0.1)

gyrox_line, = ax_gyro.plot([], [], label="gyro_x")
gyroy_line, = ax_gyro.plot([], [], label="gyro_y")
gyroz_line, = ax_gyro.plot([], [], label="gyro_z")
ax_gyro.legend()

def init():
    path_line.set_data([], [])
    vehicle_point.set_data([], [])
    heading_line.set_data([], [])
    accx_line.set_data([], [])
    accy_line.set_data([], [])
    accz_line.set_data([], [])
    gyrox_line.set_data([], [])
    gyroy_line.set_data([], [])
    gyroz_line.set_data([], [])
    vel_text.set_text("")
    return (
        path_line, vehicle_point, heading_line,
        accx_line, accy_line, accz_line,
        gyrox_line, gyroy_line, gyroz_line, vel_text
    )

def update(frame):
    x = x_hist[frame]
    y = y_hist[frame]
    th = theta_hist[frame]

    path_line.set_data(x_hist[:frame + 1], y_hist[:frame + 1])
    vehicle_point.set_data([x], [y])

    heading_len = 0.5
    hx = x + heading_len * math.cos(th)
    hy = y + heading_len * math.sin(th)
    heading_line.set_data([x, hx], [y, hy])

    tt = t_hist[:frame + 1]
    accx_line.set_data(tt, acc_meas_hist[:frame + 1, 0])
    accy_line.set_data(tt, acc_meas_hist[:frame + 1, 1])
    accz_line.set_data(tt, acc_meas_hist[:frame + 1, 2])

    gyrox_line.set_data(tt, gyro_meas_hist[:frame + 1, 0])
    gyroy_line.set_data(tt, gyro_meas_hist[:frame + 1, 1])
    gyroz_line.set_data(tt, gyro_meas_hist[:frame + 1, 2])

    vx = v * math.cos(th)
    vy = v * math.sin(th)
    vel_text.set_text(
        f"v_x      = {vx:6.3f} m/s\n"
        f"v_y      = {vy:6.3f} m/s\n"
        f"yaw_rate = {omega:6.3f} rad/s"
    )

    return (
        path_line, vehicle_point, heading_line,
        accx_line, accy_line, accz_line,
        gyrox_line, gyroy_line, gyroz_line, vel_text
    )

ani = FuncAnimation(
    fig,
    update,
    frames=len(t_hist),
    init_func=init,
    interval=dt * 1000,
    blit=True,
    repeat=True
)

plt.tight_layout()
plt.show()
