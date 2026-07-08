---
title: Quadcopter in Gazebo Harmonic
tags:
    - gazebo
    - harmonic
    - quadcopter
    - plugin
---

This demo runs the Gazebo Harmonic quadcopter world from `code/quadcopter.sdf`.
The world loads the X3 UAV model from Gazebo Fuel and attaches one
`gz::sim::systems::MulticopterMotorModel` plugin for each rotor.

## Run

```bash
gz sim docs/Simulation/Gazebo/demo_worlds/quadcopter/code/quadcopter.sdf
```

Send equal motor speeds to all four rotors to lift the vehicle:

```bash
gz topic -t /X3/gazebo/command/motor_speed \
  --msgtype gz.msgs.Actuators \
  -p 'velocity:[700, 700, 700, 700]'
```

Stop the rotors:

```bash
gz topic -t /X3/gazebo/command/motor_speed \
  --msgtype gz.msgs.Actuators \
  -p 'velocity:[0, 0, 0, 0]'
```

## World setup

The world starts the common Gazebo systems:

- `gz::sim::systems::Physics` runs the physics step.
- `gz::sim::systems::SceneBroadcaster` publishes scene state to the GUI.
- `gz::sim::systems::UserCommands` handles commands from the GUI and CLI.
- `gz::sim::systems::Sensors` enables sensor updates with the `ogre2` renderer.

The X3 UAV is included from Gazebo Fuel:

```xml
<include>
  <uri>
    https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV/4
  </uri>
  ...
</include>
```

## Multicopter motor plugin

Each rotor has its own `MulticopterMotorModel` plugin:

```xml
<plugin
  filename="gz-sim-multicopter-motor-model-system"
  name="gz::sim::systems::MulticopterMotorModel">
  <robotNamespace>X3</robotNamespace>
  <jointName>X3/rotor_0_joint</jointName>
  <linkName>X3/rotor_0</linkName>
  <turningDirection>ccw</turningDirection>
  <commandSubTopic>gazebo/command/motor_speed</commandSubTopic>
  <actuator_number>0</actuator_number>
</plugin>
```

The plugin reads the `gz.msgs.Actuators` command message from
`/X3/gazebo/command/motor_speed`. The `actuator_number` selects which value in
the `velocity` array belongs to this rotor. For example, rotor 0 reads
`velocity[0]`, rotor 1 reads `velocity[1]`, and so on.

Important fields:

| Field | Meaning |
|---|---|
| `robotNamespace` | Prefix used for the command and published motor speed topics. |
| `jointName` | Rotor joint that receives the motor velocity command. |
| `linkName` | Rotor link where thrust and drag forces are applied. |
| `turningDirection` | Rotor spin direction, `cw` or `ccw`, used for yaw torque. |
| `timeConstantUp` / `timeConstantDown` | Motor response when speeding up or slowing down. |
| `maxRotVelocity` | Maximum rotor angular velocity. |
| `motorConstant` | Converts rotor speed squared into thrust. |
| `momentConstant` | Converts thrust into reaction torque. |
| `commandSubTopic` | Topic suffix for actuator commands. |
| `actuator_number` | Index in the actuator velocity array for this rotor. |
| `rotorDragCoefficient` | Aerodynamic drag coefficient for the rotor. |
| `rollingMomentCoefficient` | Rolling moment coefficient caused by rotor motion. |
| `motorSpeedPubTopic` | Topic suffix used to publish the simulated motor speed. |
| `rotorVelocitySlowdownSim` | Visual / simulation slowdown factor for rotor velocity. |
| `motorType` | Motor control mode. This demo uses `velocity`. |

Two rotors turn counter-clockwise and two turn clockwise. This balances the
reaction torques so the vehicle can lift without immediately spinning around
the yaw axis.

## Documentation

The official Gazebo Sim API page for the plugin is:

[MulticopterMotorModel class reference](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1MulticopterMotorModel.html){:target="_blank"}

That page describes the system as the plugin that applies thrust forces to
models with spinning propellers and points to the quadcopter SDF example used by
Gazebo Sim.

<details>
<summary>World source code</summary>

```xml
--8<-- "docs/Simulation/Gazebo/demo_worlds/quadcopter/code/quadcopter.sdf"
```
</details>


---

## Choosing motor parameters

`motorType` describes how the actuator command should be interpreted. Gazebo's
`MulticopterMotorModel` recognizes these values:

| `motorType` | Status | Command meaning |
|---|---|---|
| `velocity` | Supported | `Actuators.velocity[actuator_number]` is the target rotor speed in rad/s. |
| `position` | Parsed, but not implemented | Intended for a rotor position target. Do not use for this model. |
| `force` | Parsed, but not implemented | Intended for a direct force command. Do not use for this model. |

For normal quadcopter simulation, use `velocity`. The plugin filters the target
speed with `timeConstantUp` and `timeConstantDown`, commands the rotor joint,
then computes the propeller force from the simulated rotor speed.

The most important aerodynamic parameter is `motorConstant`. It is the thrust
coefficient:

```text
thrust_N = motorConstant * rotor_speed_rad_s^2
```

To set it from a real motor and propeller, use thrust-stand data for the same
propeller, motor, battery voltage, and air density that you want to simulate:

1. Measure static thrust at several throttle points.
2. Measure or estimate rotor RPM at the same points.
3. Convert RPM to rad/s:

```text
rotor_speed_rad_s = rpm * 2 * pi / 60
```

4. Compute `motorConstant` for each point:

```text
motorConstant = thrust_N / rotor_speed_rad_s^2
```

5. Use an average value around the hover-to-cruise operating range. Avoid using
   only the maximum-throttle point, because props often become less efficient
   near the top of the throttle range.

`momentConstant` maps thrust into the reaction torque that yaws the vehicle:

```text
reaction_torque_Nm = thrust_N * momentConstant
```

If your thrust stand measures torque, set:

```text
momentConstant = torque_Nm / thrust_N
```

If you only have propeller datasheet data, start with the demo value and tune it
until equal motor speeds produce little yaw drift while differential motor speeds
produce a believable yaw response.

`rotorDragCoefficient` and `rollingMomentCoefficient` model aerodynamic effects
caused by the rotor moving through the air:

```text
air_drag_force = -abs(rotor_speed_rad_s) * rotorDragCoefficient * side_velocity
rolling_moment = -abs(rotor_speed_rad_s) * rollingMomentCoefficient * side_velocity
```

These are harder to identify from a normal static thrust test because the drone
must have sideways airflow. A practical workflow is:

1. Set `motorConstant` first so the simulated drone hovers at the same total
   rotor speed as the real drone.
2. Set `momentConstant` so yaw acceleration looks realistic.
3. Start with the demo values for `rotorDragCoefficient` and
   `rollingMomentCoefficient`.
4. Fly or replay a sideways translation test. Increase the coefficients if the
   simulated drone slides too freely or has too little aerodynamic damping.
   Decrease them if it slows down too aggressively.

Use `maxRotVelocity` as the real maximum rotor speed in rad/s. For example, a
motor and propeller combination that reaches `7600 rpm` should use:

```text
maxRotVelocity = 7600 * 2 * pi / 60 = 795.9 rad/s
```

`rotorVelocitySlowdownSim` does not change the aerodynamic model directly. It
keeps the physics joint speed lower to avoid numerical and visual aliasing while
the plugin still uses the full rotor speed for thrust calculations.
