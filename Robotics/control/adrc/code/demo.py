import matplotlib.pyplot as plt

dt = 0.01
T = 8.0
steps = int(T / dt)

target = 1.0

# physical system
x = 0.0
v = 0.0
mass = 1.0
friction = 0.8

# PID
kp = 40.0
ki = 5.0
kd = 8.0
integral = 0.0
prev_error = 0.0

xs_pid = []

for i in range(steps):
    t = i * dt

    # external disturbance, like wind / push
    disturbance = 0.0
    if 3.0 < t < 4.0:
        disturbance = -8.0

    error = target - x
    integral += error * dt
    derivative = (error - prev_error) / dt

    u = kp * error + ki * integral + kd * derivative
    prev_error = error

    # plant physics
    a = (u + disturbance - friction * v) / mass
    v += a * dt
    x += v * dt

    xs_pid.append(x)


# ADRC
x = 0.0
v = 0.0

# ESO states
z1 = 0.0   # estimated position
z2 = 0.0   # estimated velocity
z3 = 0.0   # estimated disturbance

b0 = 1.0

# controller gains
kp = 40.0
kd = 8.0

# observer gains
w0 = 25.0
beta1 = 3 * w0
beta2 = 3 * w0 * w0
beta3 = w0 * w0 * w0

u = 0.0
xs_adrc = []
dist_est = []

for i in range(steps):
    t = i * dt

    disturbance = 0.0
    if 3.0 < t < 4.0:
        disturbance = -8.0

    y = x

    # ESO update
    e = z1 - y
    z1 += dt * (z2 - beta1 * e)
    z2 += dt * (z3 - beta2 * e + b0 * u)
    z3 += dt * (-beta3 * e)

    # ADRC control law
    u0 = kp * (target - z1) - kd * z2
    u = (u0 - z3) / b0

    # plant physics
    a = (u + disturbance - friction * v) / mass
    v += a * dt
    x += v * dt

    xs_adrc.append(x)
    dist_est.append(z3)


time = [i * dt for i in range(steps)]

plt.plot(time, xs_pid, label="PID")
plt.plot(time, xs_adrc, label="ADRC")
plt.axhline(target, linestyle="--", label="target")
plt.xlabel("time [s]")
plt.ylabel("position")
plt.legend()
plt.grid()
plt.show()