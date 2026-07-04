import numpy as np
import matplotlib.pyplot as plt


class SimpleMass:
    def __init__(self):
        self.x = 0.0
        self.v = 0.0

    def disturbance(self, t):
        # Unknown disturbance: force changes with time
        return 0.5 * np.sin(2 * t)

    def update(self, u, dt, t):
        d = self.disturbance(t)
        a = u + d

        self.v += a * dt
        self.x += self.v * dt

        return self.x


class ADRC:
    def __init__(self, b0=1.0, observer_bandwidth=20.0, controller_bandwidth=5.0):
        self.b0 = b0

        # ESO states
        self.z1 = 0.0  # estimated position
        self.z2 = 0.0  # estimated velocity
        self.z3 = 0.0  # estimated disturbance

        # ESO gains for 2nd order system
        w0 = observer_bandwidth
        self.beta1 = 3 * w0
        self.beta2 = 3 * w0**2
        self.beta3 = w0**3

        # PD controller gains
        wc = controller_bandwidth
        self.kp = wc**2
        self.kd = 2 * wc

    def update(self, y, ref, dt):
        error = self.z1 - y

        # Extended State Observer
        self.z1 += (self.z2 - self.beta1 * error) * dt
        self.z2 += (self.z3 + self.b0 * 0.0 - self.beta2 * error) * dt
        self.z3 += (-self.beta3 * error) * dt

        # Desired acceleration from PD
        v_desired = 0.0
        a_desired = self.kp * (ref - self.z1) + self.kd * (v_desired - self.z2)

        # Disturbance compensation
        u = (a_desired - self.z3) / self.b0

        return u


dt = 0.001
T = 5.0
steps = int(T / dt)

plant = SimpleMass()
controller = ADRC(
    b0=1.0,
    observer_bandwidth=30.0,
    controller_bandwidth=5.0,
)

time_log = []
x_log = []
ref_log = []
u_log = []
disturbance_est_log = []

ref = 1.0

for i in range(steps):
    t = i * dt

    y = plant.x
    u = controller.update(y, ref, dt)
    x = plant.update(u, dt, t)

    time_log.append(t)
    x_log.append(x)
    ref_log.append(ref)
    u_log.append(u)
    disturbance_est_log.append(controller.z3)

plt.figure()
plt.plot(time_log, x_log, label="position")
plt.plot(time_log, ref_log, "--", label="reference")
plt.xlabel("time [s]")
plt.ylabel("position")
plt.legend()
plt.grid()

plt.figure()
plt.plot(time_log, disturbance_est_log, label="estimated disturbance")
plt.xlabel("time [s]")
plt.ylabel("disturbance estimate")
plt.legend()
plt.grid()

plt.show()