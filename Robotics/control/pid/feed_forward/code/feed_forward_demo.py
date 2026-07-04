import argparse
from dataclasses import dataclass, field

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button


CASES = {
    "Feedback only": False,
    "Feed-forward": True,
}


@dataclass
class State:
    x: float = 0.0
    v: float = 0.0


class VerticalMass:
    def __init__(self, mass: float, damping: float, gravity: float, force_limit: float):
        self.mass = mass
        self.damping = damping
        self.gravity = gravity
        self.force_limit = force_limit
        self.state = State()

    def reset(self):
        self.state = State()

    def update(self, force: float, dt: float) -> State:
        force = clamp(force, 0.0, self.force_limit)
        gravity_force = self.mass * self.gravity
        damping_force = -self.damping * self.state.v
        acceleration = (force - gravity_force + damping_force) / self.mass

        self.state.v += acceleration * dt
        self.state.x += self.state.v * dt

        return self.state


class PDController:
    def __init__(
        self,
        kp: float,
        kd: float,
        target: float,
        feed_forward_force: float,
        use_feed_forward: bool,
    ):
        self.kp = kp
        self.kd = kd
        self.target = target
        self.feed_forward_force = feed_forward_force
        self.use_feed_forward = use_feed_forward
        self.prev_error = 0.0

    def reset(self):
        self.prev_error = 0.0

    def set_feed_forward(self, enabled: bool):
        self.use_feed_forward = enabled

    def compute(self, measurement: float, dt: float) -> tuple[float, float, float]:
        error = self.target - measurement
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        feedback = self.kp * error + self.kd * derivative
        feed_forward = self.feed_forward_force if self.use_feed_forward else 0.0
        command = feedback + feed_forward

        return command, feedback, feed_forward


@dataclass
class SimulationLog:
    time: list[float] = field(default_factory=list)
    position: list[float] = field(default_factory=list)
    velocity: list[float] = field(default_factory=list)
    command: list[float] = field(default_factory=list)
    feedback: list[float] = field(default_factory=list)
    feed_forward: list[float] = field(default_factory=list)

    def add(
        self,
        t: float,
        state: State,
        command: float,
        feedback: float,
        feed_forward: float,
    ):
        self.time.append(t)
        self.position.append(state.x)
        self.velocity.append(state.v)
        self.command.append(command)
        self.feedback.append(feedback)
        self.feed_forward.append(feed_forward)

    def clear(self):
        self.time.clear()
        self.position.clear()
        self.velocity.clear()
        self.command.clear()
        self.feedback.clear()
        self.feed_forward.clear()


class FeedForwardSimulation:
    def __init__(
        self,
        system: VerticalMass,
        controller: PDController,
        log: SimulationLog,
        dt: float,
    ):
        self.system = system
        self.controller = controller
        self.log = log
        self.dt = dt
        self.t = 0.0
        self.running = False
        self.case_name = "Feedback only"

    def reset(self):
        self.system.reset()
        self.controller.reset()
        self.log.clear()
        self.t = 0.0

    def set_case(self, name: str):
        self.case_name = name
        self.controller.set_feed_forward(CASES[name])
        self.reset()
        self.running = True

    def step(self):
        command, feedback, feed_forward = self.controller.compute(
            measurement=self.system.state.x,
            dt=self.dt,
        )
        state = self.system.update(command, self.dt)

        self.t += self.dt
        self.log.add(self.t, state, command, feedback, feed_forward)

        return state, command, feedback, feed_forward


class FeedForwardGraph:
    def __init__(self, simulation: FeedForwardSimulation, target: float):
        self.simulation = simulation
        self.target = target
        self.animation = None

        self.fig, (self.ax_x, self.ax_v, self.ax_u) = plt.subplots(
            3, 1, figsize=(9, 8)
        )
        self.fig.subplots_adjust(bottom=0.22, hspace=0.35)

        self.line_x, = self.ax_x.plot([], [], label="position")
        self.line_v, = self.ax_v.plot([], [], label="velocity")
        self.line_u, = self.ax_u.plot([], [], label="total command")
        self.line_ff, = self.ax_u.plot([], [], label="feed-forward")

        self._setup_axes()
        self._setup_buttons()
        self._set_title()

    def _setup_axes(self):
        self.ax_x.axhline(self.target, linestyle="--", label="target")
        self.ax_x.set_ylabel("x")
        self.ax_x.set_ylim(-0.5, 2.0)
        self.ax_x.grid()
        self.ax_x.legend()

        self.ax_v.set_ylabel("v")
        self.ax_v.set_ylim(-1.5, 3.0)
        self.ax_v.grid()
        self.ax_v.legend()

        gravity_force = self.simulation.system.mass * self.simulation.system.gravity
        self.ax_u.axhline(gravity_force, linestyle="--", color="gray", label="mg")
        self.ax_u.set_ylabel("force")
        self.ax_u.set_xlabel("time [s]")
        self.ax_u.set_ylim(-5, 50)
        self.ax_u.grid()
        self.ax_u.legend()

        for ax in [self.ax_x, self.ax_v, self.ax_u]:
            ax.set_xlim(0, 6)

    def _setup_buttons(self):
        feedback_ax = self.fig.add_axes([0.18, 0.05, 0.2, 0.05])
        feed_forward_ax = self.fig.add_axes([0.42, 0.05, 0.2, 0.05])
        reset_ax = self.fig.add_axes([0.66, 0.05, 0.14, 0.05])

        self.feedback_button = Button(feedback_ax, "Feedback only")
        self.feed_forward_button = Button(feed_forward_ax, "Feed-forward")
        self.reset_button = Button(reset_ax, "Reset")

        self.feedback_button.on_clicked(lambda event: self._run_case("Feedback only"))
        self.feed_forward_button.on_clicked(lambda event: self._run_case("Feed-forward"))
        self.reset_button.on_clicked(self._reset_simulation)

    def _run_case(self, name: str):
        self.simulation.set_case(name)
        self._clear_lines()
        self._set_title()
        self.fig.canvas.draw_idle()

    def _reset_simulation(self, event):
        self.simulation.reset()
        self.simulation.running = False
        self._clear_lines()
        self._set_title()
        self.fig.canvas.draw_idle()

    def _clear_lines(self):
        for line in [self.line_x, self.line_v, self.line_u, self.line_ff]:
            line.set_data([], [])

    def _set_title(self):
        self.fig.suptitle(f"Feed-forward demo: {self.simulation.case_name}")

    def update(self, frame):
        if not self.simulation.running:
            return self.line_x, self.line_v, self.line_u, self.line_ff

        self.simulation.step()
        log = self.simulation.log

        self.line_x.set_data(log.time, log.position)
        self.line_v.set_data(log.time, log.velocity)
        self.line_u.set_data(log.time, log.command)
        self.line_ff.set_data(log.time, log.feed_forward)

        if self.simulation.t >= 6.0:
            self.simulation.running = False

        return self.line_x, self.line_v, self.line_u, self.line_ff

    def run(self):
        self.animation = FuncAnimation(
            self.fig,
            self.update,
            interval=10,
            blit=False,
        )
        plt.show()


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def create_simulation() -> FeedForwardSimulation:
    mass = 1.0
    gravity = 9.81
    target = 1.5

    system = VerticalMass(
        mass=mass,
        damping=2.0,
        gravity=gravity,
        force_limit=25.0,
    )
    controller = PDController(
        kp=25.0,
        kd=10.0,
        target=target,
        feed_forward_force=mass * gravity,
        use_feed_forward=CASES["Feedback only"],
    )

    return FeedForwardSimulation(
        system=system,
        controller=controller,
        log=SimulationLog(),
        dt=0.005,
    )


def run_full_case(name: str) -> SimulationLog:
    simulation = create_simulation()
    simulation.set_case(name)

    while simulation.t < 6.0:
        simulation.step()

    return simulation.log


def save_screenshot(path: str):
    target = 1.5
    feedback_only = run_full_case("Feedback only")
    feed_forward = run_full_case("Feed-forward")

    fig, (ax_x, ax_v, ax_u) = plt.subplots(3, 1, figsize=(9, 8))
    fig.subplots_adjust(hspace=0.35)
    fig.suptitle("Feedback only vs feed-forward")

    ax_x.axhline(target, linestyle="--", color="gray", label="target")
    ax_x.plot(feedback_only.time, feedback_only.position, label="feedback only")
    ax_x.plot(feed_forward.time, feed_forward.position, label="feed-forward")
    ax_x.set_ylabel("x")
    ax_x.set_ylim(-0.5, 2.0)
    ax_x.grid()
    ax_x.legend()

    ax_v.plot(feedback_only.time, feedback_only.velocity, label="feedback only")
    ax_v.plot(feed_forward.time, feed_forward.velocity, label="feed-forward")
    ax_v.set_ylabel("v")
    ax_v.set_ylim(-1.5, 3.0)
    ax_v.grid()
    ax_v.legend()

    ax_u.axhline(9.81, linestyle="--", color="gray", label="mg")
    ax_u.plot(feedback_only.time, feedback_only.command, label="feedback command")
    ax_u.plot(feed_forward.time, feed_forward.command, label="feed-forward command")
    ax_u.set_ylabel("force")
    ax_u.set_xlabel("time [s]")
    ax_u.set_ylim(-5, 50)
    ax_u.grid()
    ax_u.legend()

    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--save", help="Save a comparison screenshot and exit.")
    args = parser.parse_args()

    if args.save:
        save_screenshot(args.save)
        return

    graph = FeedForwardGraph(
        simulation=create_simulation(),
        target=1.5,
    )
    graph.run()


if __name__ == "__main__":
    main()
