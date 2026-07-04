import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider
from dataclasses import dataclass, field


PID_PRESETS = {
    "Slow": (4.0, 0.0, 2.0),
    "Good": (25.0, 0.5, 8.0),
    "Oscillate": (80.0, 0.0, 0.0),
}


@dataclass
class SystemState:
    x: float = 0.0   # position
    v: float = 0.0   # velocity


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, target: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, measurement: float, dt: float) -> float:
        error = self.target - measurement

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        self.prev_error = error
        return output


class MassFrictionSystem:
    def __init__(self, mass: float, friction: float):
        self.mass = mass
        self.friction = friction
        self.state = SystemState()

    def reset(self):
        self.state = SystemState()

    def update(self, force: float, dt: float) -> SystemState:
        """

        # Friction force is proportional to the velocity 
        # and acts in the opposite direction
        """
        friction_force = -self.friction * self.state.v

        acceleration = (force + friction_force) / self.mass

        self.state.v += acceleration * dt
        self.state.x += self.state.v * dt

        return self.state


@dataclass
class SimulationLog:
    time: list[float] = field(default_factory=list)
    position: list[float] = field(default_factory=list)
    velocity: list[float] = field(default_factory=list)
    control: list[float] = field(default_factory=list)

    def add(self, t: float, state: SystemState, control: float):
        self.time.append(t)
        self.position.append(state.x)
        self.velocity.append(state.v)
        self.control.append(control)

    def keep_last_seconds(self, seconds: float):
        while self.time and self.time[-1] - self.time[0] > seconds:
            self.time.pop(0)
            self.position.pop(0)
            self.velocity.pop(0)
            self.control.pop(0)

    def clear(self):
        self.time.clear()
        self.position.clear()
        self.velocity.clear()
        self.control.clear()


class PIDSimulation:
    def __init__(
        self,
        system: MassFrictionSystem,
        controller: PIDController,
        log: SimulationLog,
        dt: float,
    ):
        self.system = system
        self.controller = controller
        self.log = log
        self.dt = dt
        self.t = 0.0
        self.running = False

    def reset(self):
        self.system.reset()
        self.controller.reset()
        self.log.clear()
        self.t = 0.0

    def step(self):
        x = self.system.state.x

        control_force = self.controller.compute(
            measurement=x,
            dt=self.dt,
        )

        state = self.system.update(
            force=control_force,
            dt=self.dt,
        )

        self.t += self.dt
        self.log.add(self.t, state, control_force)

        return state, control_force


class DynamicGraph:
    def __init__(self, simulation: PIDSimulation, target: float):
        self.simulation = simulation
        self.target = target

        self.fig, (self.ax_x, self.ax_v, self.ax_u) = plt.subplots(
            3, 1, figsize=(8, 8)
        )
        self.fig.subplots_adjust(bottom=0.36)

        self.line_x, = self.ax_x.plot([], [], label="position")
        self.line_v, = self.ax_v.plot([], [], label="velocity")
        self.line_u, = self.ax_u.plot([], [], label="control force")

        self._setup_axes()
        self._setup_sliders()
        self._setup_buttons()

    def _setup_axes(self):
        self.ax_x.axhline(self.target, linestyle="--", label="target")
        self.ax_x.set_ylabel("x")
        self.ax_x.set_ylim(-0.5, 1.5)
        self.ax_x.grid()
        self.ax_x.legend()

        self.ax_v.set_ylabel("v")
        self.ax_v.set_ylim(-2, 2)
        self.ax_v.grid()
        self.ax_v.legend()

        self.ax_u.set_ylabel("u")
        self.ax_u.set_xlabel("time [s]")
        self.ax_u.set_ylim(-50, 50)
        self.ax_u.grid()
        self.ax_u.legend()

    def _setup_sliders(self):
        slider_specs = [
            ("Kp", 0.04, 0.27, 0.0, 100.0, self.simulation.controller.kp),
            ("Ki", 0.04, 0.22, 0.0, 20.0, self.simulation.controller.ki),
            ("Kd", 0.04, 0.17, 0.0, 30.0, self.simulation.controller.kd),
        ]

        self.sliders = {}
        for label, left, bottom, min_value, max_value, initial in slider_specs:
            ax = self.fig.add_axes([left, bottom, 0.88, 0.025])
            slider = Slider(
                ax=ax,
                label=label,
                valmin=min_value,
                valmax=max_value,
                valinit=initial,
            )
            slider.on_changed(self._update_pid_values)
            self.sliders[label] = slider

    def _setup_buttons(self):
        start_ax = self.fig.add_axes([0.35, 0.025, 0.12, 0.04])
        reset_ax = self.fig.add_axes([0.53, 0.025, 0.12, 0.04])

        self.start_button = Button(start_ax, "Start")
        self.reset_button = Button(reset_ax, "Reset")

        self.start_button.on_clicked(self._toggle_running)
        self.reset_button.on_clicked(self._reset_simulation)

        self.preset_buttons = {}
        for index, name in enumerate(PID_PRESETS):
            left = 0.23 + index * 0.19
            ax = self.fig.add_axes([left, 0.095, 0.16, 0.04])
            button = Button(ax, name)
            button.on_clicked(
                lambda event, preset_name=name: self._apply_preset(preset_name)
            )
            self.preset_buttons[name] = button

    def _update_pid_values(self, value):
        controller = self.simulation.controller
        controller.kp = self.sliders["Kp"].val
        controller.ki = self.sliders["Ki"].val
        controller.kd = self.sliders["Kd"].val

    def _toggle_running(self, event):
        self.simulation.running = not self.simulation.running
        label = "Pause" if self.simulation.running else "Start"
        self.start_button.label.set_text(label)
        self.fig.canvas.draw_idle()

    def _reset_simulation(self, event):
        self.simulation.reset()
        self.simulation.running = False
        self.start_button.label.set_text("Start")
        self.line_x.set_data([], [])
        self.line_v.set_data([], [])
        self.line_u.set_data([], [])
        for ax in [self.ax_x, self.ax_v, self.ax_u]:
            ax.set_xlim(0, 5)
        self.fig.canvas.draw_idle()

    def _apply_preset(self, name: str):
        kp, ki, kd = PID_PRESETS[name]
        self.sliders["Kp"].set_val(kp)
        self.sliders["Ki"].set_val(ki)
        self.sliders["Kd"].set_val(kd)
        self._reset_simulation(None)

    def update(self, frame):
        if not self.simulation.running:
            return self.line_x, self.line_v, self.line_u

        self.simulation.step()
        self.simulation.log.keep_last_seconds(5.0)

        log = self.simulation.log
        t = self.simulation.t

        x_min = max(0, t - 5)
        x_max = max(5, t)

        for ax in [self.ax_x, self.ax_v, self.ax_u]:
            ax.set_xlim(x_min, x_max)

        self.line_x.set_data(log.time, log.position)
        self.line_v.set_data(log.time, log.velocity)
        self.line_u.set_data(log.time, log.control)

        return self.line_x, self.line_v, self.line_u

    def run(self):
        self.animation = FuncAnimation(
            self.fig,
            self.update,
            interval=10,
            blit=False,
        )
        plt.show()


def main():
    dt = 0.01
    target = 1.0

    system = MassFrictionSystem(
        mass=1.0,
        friction=2.0,
    )

    controller = PIDController(
        kp=PID_PRESETS["Good"][0],
        ki=PID_PRESETS["Good"][1],
        kd=PID_PRESETS["Good"][2],
        target=target,
    )

    log = SimulationLog()

    simulation = PIDSimulation(
        system=system,
        controller=controller,
        log=log,
        dt=dt,
    )

    graph = DynamicGraph(
        simulation=simulation,
        target=target,
    )

    graph.run()


if __name__ == "__main__":
    main()
