import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Data & figure
x = np.linspace(0, 2*np.pi, 400)
fig, ax = plt.subplots()
(line,) = ax.plot([], [], lw=2)   # artist to update
ax.set_xlim(0, 2*np.pi)
ax.set_ylim(-1.2, 1.2)
ax.grid(True)

def init():
    line.set_data([], [])
    return (line,)       # return a tuple/list of artists

def update(frame):
    # frame is an int (0..N-1) or any object from your frames iterable
    print(f"Frame {frame}")  # for debugging
    y = np.sin(x + 0.05*frame)
    line.set_data(x, y)  # mutate existing artist
    return (line,)

anim = FuncAnimation(
    fig, update, 
    init_func=init,
    frames=200,          # or frames=range(200) or any iterable/generator
    interval=20,         # milliseconds between frames
    blit=True            # only re-draw changed artists (faster on many backends)
)

plt.show()
