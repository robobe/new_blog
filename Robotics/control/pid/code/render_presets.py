from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from simple_system import (
    MassFrictionSystem,
    PIDController,
    PIDSimulation,
    PID_PRESETS,
    SimulationLog,
)


def render_preset(name: str, output_path: Path):
    dt = 0.01
    target = 1.0
    steps = int(5.0 / dt)
    kp, ki, kd = PID_PRESETS[name]

    system = MassFrictionSystem(mass=1.0, friction=2.0)
    controller = PIDController(kp=kp, ki=ki, kd=kd, target=target)
    log = SimulationLog()
    simulation = PIDSimulation(
        system=system,
        controller=controller,
        log=log,
        dt=dt,
    )

    for _ in range(steps):
        simulation.step()

    fig, (ax_x, ax_v, ax_u) = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    fig.suptitle(
        f"{name} PID response after 5 seconds: Kp={kp}, Ki={ki}, Kd={kd}"
    )

    ax_x.plot(log.time, log.position, label="position")
    ax_x.axhline(target, linestyle="--", color="tab:gray", label="target")
    ax_x.set_ylabel("x")
    ax_x.set_ylim(-1.5, 2.5)
    ax_x.grid(True)
    ax_x.legend()

    ax_v.plot(log.time, log.velocity, label="velocity", color="tab:orange")
    ax_v.set_ylabel("v")
    ax_v.set_ylim(-10, 10)
    ax_v.grid(True)
    ax_v.legend()

    ax_u.plot(log.time, log.control, label="control force", color="tab:green")
    ax_u.set_ylabel("u")
    ax_u.set_xlabel("time [s]")
    ax_u.set_ylim(-100, 100)
    ax_u.grid(True)
    ax_u.legend()

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def main():
    output_dir = Path(__file__).parent.parent / "images"
    output_dir.mkdir(exist_ok=True)

    output_files = {
        "Slow": "slow_response.png",
        "Good": "good_response.png",
        "Oscillate": "oscillate_response.png",
    }

    for name, filename in output_files.items():
        output_path = output_dir / filename
        render_preset(name, output_path)
        print(output_path)


if __name__ == "__main__":
    main()
