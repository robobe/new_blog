import numpy as np
import matplotlib.pyplot as plt


def rot_x(phi):
    c, s = np.cos(phi), np.sin(phi)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])


def rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])


def rot_z(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])


# ---------------------------
# Simulation settings
# ---------------------------
dt = 0.01
t = np.arange(0, 20, dt)
n = len(t)
g = 9.81

# ground-truth Euler angles
roll = np.deg2rad(10.0) * np.sin(2 * np.pi * 0.2 * t)
pitch = np.deg2rad(7.0) * np.sin(2 * np.pi * 0.1 * t)
yaw = np.deg2rad(20.0) * t / 20.0 * 10.0   # slow yaw rotation

# approximate angular rates from Euler differences
roll_dot = np.gradient(roll, dt)
pitch_dot = np.gradient(pitch, dt)
yaw_dot = np.gradient(yaw, dt)

# for simple learning demo, use these as gyro body rates
gyro_true = np.column_stack((roll_dot, pitch_dot, yaw_dot))

# accelerometer from gravity projected into body frame
acc_true = np.zeros((n, 3))
gravity_world = np.array([0.0, 0.0, g])

for i in range(n):
    R = rot_z(yaw[i]) @ rot_y(pitch[i]) @ rot_x(roll[i])
    # world -> body
    acc_true[i] = R.T @ gravity_world

# ---------------------------
# Add sensor imperfections
# ---------------------------
gyro_bias = np.array([0.01, -0.02, 0.015])      # rad/s
acc_bias = np.array([0.1, -0.05, 0.08])         # m/s^2

gyro_noise_std = 0.01
acc_noise_std = 0.08

rng = np.random.default_rng(42)

gyro_meas = gyro_true + gyro_bias + rng.normal(0, gyro_noise_std, gyro_true.shape)
acc_meas = acc_true + acc_bias + rng.normal(0, acc_noise_std, acc_true.shape)

# ---------------------------
# Plot
# ---------------------------
fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

ax[0].plot(t, np.rad2deg(roll), label="roll")
ax[0].plot(t, np.rad2deg(pitch), label="pitch")
ax[0].plot(t, np.rad2deg(yaw), label="yaw")
ax[0].set_ylabel("Angle [deg]")
ax[0].legend()
ax[0].grid(True)

ax[1].plot(t, gyro_meas[:, 0], label="gx")
ax[1].plot(t, gyro_meas[:, 1], label="gy")
ax[1].plot(t, gyro_meas[:, 2], label="gz")
ax[1].set_ylabel("Gyro [rad/s]")
ax[1].legend()
ax[1].grid(True)

ax[2].plot(t, acc_meas[:, 0], label="ax")
ax[2].plot(t, acc_meas[:, 1], label="ay")
ax[2].plot(t, acc_meas[:, 2], label="az")
ax[2].set_ylabel("Accel [m/s²]")
ax[2].set_xlabel("Time [s]")
ax[2].legend()
ax[2].grid(True)

plt.tight_layout()
plt.show()