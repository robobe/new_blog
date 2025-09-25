import numpy as np
import matplotlib.pyplot as plt
from collections import deque

class MovingAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
        self.sum = 0.0

    def update(self, new_value):
        if len(self.buffer) == self.window_size:
            self.sum -= self.buffer[0]
        self.buffer.append(new_value)
        self.sum += new_value
        return self.sum / len(self.buffer)


# Generate noisy signal (simulate stream)
t = np.linspace(0, 4*np.pi, 200)
signal = np.sin(t) + 0.5*np.random.randn(len(t))

# Apply streaming moving average
ma = MovingAverage(window_size=5)
ma_15 = MovingAverage(window_size=15)
filtered = []
filtered2 = []

for x in signal:
    y = ma.update(x)
    filtered.append(y)
    y2 = ma_15.update(x)
    filtered2.append(y2)

# Plot results
plt.figure(figsize=(10,5))
plt.plot(t, signal, label="Noisy Signal", alpha=0.6)
plt.plot(t, filtered, label="Streaming SMA (window=5)", linewidth=2)
plt.plot(t, filtered2, label="Streaming SMA (window=15)", linewidth=2)
plt.legend()
plt.show()
