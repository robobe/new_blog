"""Animate or export a spring-cart controlled by a linear ADRC controller."""

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter
from matplotlib.patches import Rectangle, FancyArrowPatch


def simulate():
    dt = 0.001
    times = np.arange(0, 6, dt)
    mass = 1.0
    spring_stiffness = 2.0
    target = 1.0
    kp, kd = 16.0, 8.0
    position = speed = 0.0
    z1 = z2 = z3 = 0.0
    b0 = 1.0 / mass
    w = 25.0
    beta1, beta2, beta3 = 3 * w, 3 * w**2, w**3
    history = []

    for t in times:
        extra_force = 1.0 if t >= 2.0 else 0.0
        control_force = (kp * (target - z1) - kd * z2 - z3) / b0
        spring_force = -spring_stiffness * position
        acceleration = (control_force + spring_force + extra_force) / mass
        history.append([
            position, z1, speed, z2,
            (spring_force + extra_force) / mass, z3, control_force,
        ])

        error = position - z1
        dz1 = z2 + beta1 * error
        dz2 = b0 * control_force + z3 + beta2 * error
        dz3 = beta3 * error
        z1 += dz1 * dt
        z2 += dz2 * dt
        z3 += dz3 * dt

        position += speed * dt + 0.5 * acceleration * dt**2
        speed += acceleration * dt

    return times, np.array(history)


def make_animation(times, history):
    fig, axes = plt.subplots(
        4, 1, figsize=(10, 9),
        gridspec_kw={"height_ratios": [1.2, 1, 1, 1]},
    )
    cart_ax = axes[0]
    end_position = history[-1, 0]
    cart_ax.set_xlim(-1, end_position + 3)
    cart_ax.set_ylim(-0.15, 1.5)
    cart_ax.set_yticks([])
    cart_ax.set_xlabel("Position on track (m)")
    cart_ax.set_title("ADRC-controlled spring cart — target 1 m; extra push 1 N at 2 s")
    cart_ax.axhline(0, color="gray")
    cart_ax.axvline(1.0, color="tab:red", linestyle="--", label="Target: 1 m")

    # The cart's center represents its measured position.
    cart = Rectangle((-0.35, 0.08), 0.7, 0.35, color="tab:blue")
    cart_ax.add_patch(cart)
    wheel1, = cart_ax.plot([], [], "ko", markersize=6)
    wheel2, = cart_ax.plot([], [], "ko", markersize=6)

    motor_arrow = FancyArrowPatch(
        (0, 0.62), (0, 0.62), arrowstyle="->",
        mutation_scale=15, color="tab:green", linewidth=2,
    )
    push_arrow = FancyArrowPatch(
        (0, 0.96), (2, 0.96), arrowstyle="->",
        mutation_scale=15, color="tab:orange", linewidth=2,
    )
    cart_ax.add_patch(motor_arrow)
    cart_ax.add_patch(push_arrow)
    motor_label = cart_ax.text(0, 0.73, "", color="tab:green")
    push_label = cart_ax.text(0, 1.07, "Extra push: 1 N", color="tab:orange")
    status = cart_ax.text(0.01, 0.93, "", transform=cart_ax.transAxes)

    labels = ["Position (m)", "Speed (m/s)", "Total disturbance (m/s²)"]
    markers = []
    for i, ax in enumerate(axes[1:]):
        ax.plot(times, history[:, 2*i], color="tab:blue", label="Real")
        ax.plot(times, history[:, 2*i+1], "--",
                color="tab:orange", label="Observer guess")
        ax.axvline(2, color="gray", linestyle=":")
        real_dot, = ax.plot([], [], "o", color="tab:blue")
        guessed_dot, = ax.plot([], [], "o", color="tab:orange")
        time_line = ax.axvline(0, color="gray", alpha=0.5)
        markers.append((real_dot, guessed_dot, time_line))
        ax.set_xlim(0, 6)
        ax.set_ylabel(labels[i])
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left")
    axes[-1].set_xlabel("Time (s)")

    def update(index):
        t = times[index]
        x = history[index, 0]
        v = history[index, 2]
        control_force = history[index, 6]
        pushed = t >= 2.0
        cart.set_x(x - 0.35)
        wheel1.set_data([x - 0.22], [0.04])
        wheel2.set_data([x + 0.22], [0.04])
        motor_arrow.set_positions((x, 0.62), (x + 0.12 * control_force, 0.62))
        motor_label.set_position((x, 0.73))
        motor_label.set_text(f"Motor: {control_force:.1f} N")
        push_arrow.set_positions((x, 0.96), (x + 2, 0.96))
        push_label.set_position((x, 1.07))
        push_arrow.set_visible(pushed)
        push_label.set_visible(pushed)
        status.set_text(f"t = {t:.2f} s    x = {x:.2f} m    v = {v:.2f} m/s")

        for i, (real_dot, guessed_dot, time_line) in enumerate(markers):
            real_dot.set_data([t], [history[index, 2*i]])
            guessed_dot.set_data([t], [history[index, 2*i+1]])
            time_line.set_xdata([t, t])

    # Physics runs at 1,000 Hz; display approximately 30 frames per second.
    frames = np.unique(np.append(np.arange(0, len(times), 33), len(times)-1))
    update(0)
    fig.tight_layout()
    animation = FuncAnimation(
        fig, update, frames=frames, interval=33,
        repeat=True, blit=False, cache_frame_data=False,
    )
    return fig, animation, update


def save_animation(animation, output_dir="."):
    """Save the animation as an MP4 video and an animated GIF."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    fps = 30

    def progress(frame, total):
        if frame == 0 or frame + 1 == total or frame % 30 == 0:
            print(f"  frame {frame + 1}/{total}", flush=True)

    video_path = output_dir / "mass-spring.mp4"
    gif_path = output_dir / "mass-spring.gif"
    print(f"Writing video: {video_path.resolve()}", flush=True)
    animation.save(
        video_path, writer=FFMpegWriter(fps=fps, bitrate=1800),
        dpi=100, progress_callback=progress,
    )
    print(f"Writing GIF:   {gif_path.resolve()}", flush=True)
    animation.save(
        gif_path, writer=PillowWriter(fps=fps),
        dpi=100, progress_callback=progress,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--export", action="store_true",
        help="save mass-spring.mp4 and mass-spring.gif instead of opening a window",
    )
    parser.add_argument(
        "--output-dir", default=".",
        help="directory for exported files (default: current directory)",
    )
    args = parser.parse_args()

    times, history = simulate()
    fig, animation, update = make_animation(times, history)
    if args.export:
        save_animation(animation, args.output_dir)
        print("Export complete", flush=True)
    else:
        plt.show()
