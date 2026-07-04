import argparse
from dataclasses import dataclass, field

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button


CASES = {
    "Windup": None,
    "Anti-windup": 1.0,
}


@dataclass
class State:
    x: float = 0.0
    v: float = 0.0


class SaturatedMass:
    def __init__(self, mass: float, friction: float, force_limit: float):
        self.mass = mass
        self.friction = friction
        self.force_limit = force_limit
        self.state = State()

    def reset(self):
        self.state = State()

    def update(self, force: float, dt: float) -> State:
        force = clamp(force, -self.force_limit, self.force_limit)
        friction_force = -self.friction * self.state.v
        acceleration = (force + friction_force) / self.mass

        self.state.v += acceleration * dt
        self.state.x += self.state.v * dt

        return self.state


class PID:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        target: float,
        integral_limit: float | None = None,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def set_integral_limit(self, integral_limit: float | None):
        self.integral_limit = integral_limit

    def compute(self, measurement: float, dt: float) -> tuple[float, float]:
        error = self.target - measurement

        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = clamp(
                self.integral,
                -self.integral_limit,
                self.integral_limit,
            )

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        command = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )
        return command, self.integral


@dataclass
class SimulationLog:
    time: list[float] = field(default_factory=list)
    position: list[float] = field(default_factory=list)
    velocity: list[float] = field(default_factory=list)
    command: list[float] = field(default_factory=list)
    force: list[float] = field(default_factory=list)
    integral: list[float] = field(default_factory=list)

    def add(
        self,
        t: float,
        state: State,
        command: float,
        force: float,
        integral: float,
    ):
        self.time.append(t)
        self.position.append(state.x)
        self.velocity.append(state.v)
        self.command.append(command)
        self.force.append(force)
        self.integral.append(integral)

    def clear(self):
        self.time.clear()
        self.position.clear()
        self.velocity.clear()
        self.command.clear()
        self.force.clear()
        self.integral.clear()


class WindupSimulation:
    def __init__(
        self,
        system: SaturatedMass,
        controller: PID,
        log: SimulationLog,
        dt: float,
    ):
        self.system = system
        self.controller = controller
        self.log = log
        self.dt = dt
        self.t = 0.0
        self.running = False
        self.case_name = "Windup"

    def reset(self):
        self.system.reset()
        self.controller.reset()
        self.log.clear()
        self.t = 0.0

    def set_case(self, name: str):
        self.case_name = name
        self.controller.set_integral_limit(CASES[name])
        self.reset()
        self.running = True

    def step(self):
        command, integral = self.controller.compute(
            measurement=self.system.state.x,
            dt=self.dt,
        )
        force = clamp(
            command,
            -self.system.force_limit,
            self.system.force_limit,
        )
        state = self.system.update(command, self.dt)

        self.t += self.dt
        self.log.add(self.t, state, command, force, integral)

        return state, command, force, integral


class WindupGraph:
    def __init__(self, simulation: WindupSimulation, target: float):
        self.simulation = simulation
        self.target = target
        self.animation = None

        self.fig, (self.ax_x, self.ax_u, self.ax_i) = plt.subplots(
            3, 1, figsize=(9, 8)
        )
        self.fig.subplots_adjust(bottom=0.22, hspace=0.35)

        self.line_x, = self.ax_x.plot([], [], label="position")
        self.line_u, = self.ax_u.plot([], [], label="PID command")
        self.line_force, = self.ax_u.plot([], [], label="limited force")
        self.line_i, = self.ax_i.plot([], [], label="integral")

        self._setup_axes()
        self._setup_buttons()
        self._set_title()

    def _setup_axes(self):
        self.ax_x.axhline(self.target, linestyle="--", label="target")
        self.ax_x.set_ylabel("x")
        self.ax_x.set_ylim(-2, 18)
        self.ax_x.grid()
        self.ax_x.legend()

        limit = self.simulation.system.force_limit
        self.ax_u.axhline(limit, linestyle="--", color="gray", label="force limit")
        self.ax_u.axhline(-limit, linestyle="--", color="gray")
        self.ax_u.set_ylabel("u")
        self.ax_u.set_ylim(-25, 35)
        self.ax_u.grid()
        self.ax_u.legend()

        self.ax_i.set_ylabel("integral")
        self.ax_i.set_xlabel("time [s]")
        self.ax_i.set_ylim(-12, 12)
        self.ax_i.grid()
        self.ax_i.legend()

        for ax in [self.ax_x, self.ax_u, self.ax_i]:
            ax.set_xlim(0, 20)

    def _setup_buttons(self):
        windup_ax = self.fig.add_axes([0.18, 0.05, 0.18, 0.05])
        anti_windup_ax = self.fig.add_axes([0.41, 0.05, 0.2, 0.05])
        reset_ax = self.fig.add_axes([0.66, 0.05, 0.14, 0.05])

        self.windup_button = Button(windup_ax, "Windup")
        self.anti_windup_button = Button(anti_windup_ax, "Anti-windup")
        self.reset_button = Button(reset_ax, "Reset")

        self.windup_button.on_clicked(lambda event: self._run_case("Windup"))
        self.anti_windup_button.on_clicked(
            lambda event: self._run_case("Anti-windup")
        )
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
        for line in [self.line_x, self.line_u, self.line_force, self.line_i]:
            line.set_data([], [])

    def _set_title(self):
        mode = self.simulation.case_name
        self.fig.suptitle(f"PID integral windup demo: {mode}")

    def update(self, frame):
        if not self.simulation.running:
            return self.line_x, self.line_u, self.line_force, self.line_i

        self.simulation.step()
        log = self.simulation.log

        self.line_x.set_data(log.time, log.position)
        self.line_u.set_data(log.time, log.command)
        self.line_force.set_data(log.time, log.force)
        self.line_i.set_data(log.time, log.integral)

        if self.simulation.t >= 20.0:
            self.simulation.running = False

        return self.line_x, self.line_u, self.line_force, self.line_i

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


def create_simulation() -> WindupSimulation:
    target = 10.0
    system = SaturatedMass(
        mass=1.0,
        friction=1.2,
        force_limit=4.0,
    )
    controller = PID(
        kp=2.0,
        ki=1.0,
        kd=3.0,
        target=target,
        integral_limit=CASES["Windup"],
    )
    return WindupSimulation(
        system=system,
        controller=controller,
        log=SimulationLog(),
        dt=0.01,
    )


def run_full_case(name: str) -> SimulationLog:
    simulation = create_simulation()
    simulation.set_case(name)

    while simulation.t < 20.0:
        simulation.step()

    return simulation.log


def save_screenshot(path: str):
    target = 10.0
    windup = run_full_case("Windup")
    anti_windup = run_full_case("Anti-windup")

    fig, (ax_x, ax_u, ax_i) = plt.subplots(3, 1, figsize=(9, 8))
    fig.subplots_adjust(hspace=0.35)
    fig.suptitle("PID windup vs anti-windup")

    ax_x.axhline(target, linestyle="--", color="gray", label="target")
    ax_x.plot(windup.time, windup.position, label="windup")
    ax_x.plot(anti_windup.time, anti_windup.position, label="anti-windup")
    ax_x.set_ylabel("x")
    ax_x.set_ylim(-2, 18)
    ax_x.grid()
    ax_x.legend()

    ax_u.axhline(4.0, linestyle="--", color="gray", label="force limit")
    ax_u.axhline(-4.0, linestyle="--", color="gray")
    ax_u.plot(windup.time, windup.command, label="windup command")
    ax_u.plot(anti_windup.time, anti_windup.command, label="anti-windup command")
    ax_u.set_ylabel("u")
    ax_u.set_ylim(-25, 35)
    ax_u.grid()
    ax_u.legend()

    ax_i.plot(windup.time, windup.integral, label="windup integral")
    ax_i.plot(anti_windup.time, anti_windup.integral, label="anti-windup integral")
    ax_i.set_ylabel("integral")
    ax_i.set_xlabel("time [s]")
    ax_i.set_ylim(-12, 12)
    ax_i.grid()
    ax_i.legend()

    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--save", help="Save a comparison screenshot and exit.")
    args = parser.parse_args()

    if args.save:
        save_screenshot(args.save)
        return

    graph = WindupGraph(
        simulation=create_simulation(),
        target=10.0,
    )
    graph.run()


if __name__ == "__main__":
    main()
