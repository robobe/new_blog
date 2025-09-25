import numpy as np
import matplotlib.pyplot as plt

# Simulated noisy signal
t = np.linspace(0, 2*np.pi, 200)
x = np.sin(t) + 0.5*np.random.randn(len(t))

# Exponential Moving Average (IIR LPF)
alpha = 0.9
y = np.zeros_like(x)
for i in range(1, len(x)):
    y[i] = (1 - alpha) * x[i] + alpha * y[i-1]

plt.figure(figsize=(10,5))
plt.plot(t, x, label="Noisy Signal")
plt.plot(t, y, label=f"IIR Low-pass (alpha={alpha})", linewidth=2)
plt.legend()
plt.show()
