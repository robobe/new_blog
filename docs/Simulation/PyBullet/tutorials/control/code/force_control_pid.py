import pybullet as p
import pybullet_data
import time
import pathlib
import math


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


class PidController:
    def __init__(
        self,
        kp=0.0,
        ki=0.0,
        kd=0.0,
        output_limit=0.0,
        integral_limit=0.0,
        feedforward=0.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.feedforward = feedforward
        self.integral_error = 0.0
        self.previous_error = None
        self.previous_time = None

    def update(self, setpoint, current):
        current_time = time.monotonic()

        # Proportional error: how far the joint is from the target right now.
        error = setpoint - current

        # The time delta since the last update is needed to calculate the integral and
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = current_time - self.previous_time

        if dt > 0.0:
            # Integral term: accumulate error over time.
            # This helps remove steady-state error, but must be clamped so it
            # does not grow forever when the joint cannot reach the target.
            self.integral_error += error * dt
            self.integral_error = clamp(
                self.integral_error,
                -self.integral_limit,
                self.integral_limit,
            )

        if self.previous_error is None or dt <= 0.0:
            # No previous sample exists on the first call, so the derivative
            # cannot be calculated yet.
            derivative_error = 0.0
        else:
            # Derivative term: estimate how fast the error is changing.
            # It adds damping by resisting fast changes and reducing overshoot.
            derivative_error = (error - self.previous_error) / dt

        output = (
            self.kp * error
            + self.ki * self.integral_error
            + self.kd * derivative_error
            + self.feedforward
        )
        output = clamp(output, -self.output_limit, self.output_limit)

        self.previous_error = error
        self.previous_time = current_time

        return output


# setup
p.connect(p.GUI)
p.resetSimulation()  # type: ignore
p.setGravity(gravX=0, gravY=0, gravZ=-9.8)
p.setAdditionalSearchPath(path=pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

# add current file location and its 'urdf' subfolder to search paths
cwd = pathlib.Path(__file__).parent.resolve()
cwd = cwd.joinpath("urdf")
p.setAdditionalSearchPath(cwd.as_posix())

# load URDF
robot = p.loadURDF("rrbot.urdf", basePosition=[0, 0, 0.5], useFixedBase=True)

joint_id = 0
p.resetJointState(robot, joint_id, 0.0)

# Disable the default motor before applying custom torque control.
p.setJointMotorControl2(
    bodyUniqueId=robot,
    jointIndex=joint_id,
    controlMode=p.VELOCITY_CONTROL,
    force=0,
)

# PID and torque-control parameters.
target_angle_param = p.addUserDebugParameter("target angle (deg)", -180, 180, 45)
kp_param = p.addUserDebugParameter("kp", 0, 500, 80)
ki_param = p.addUserDebugParameter("ki", 0, 100, 0)
kd_param = p.addUserDebugParameter("kd", 0, 100, 8)
max_torque_param = p.addUserDebugParameter("max torque", 0, 1000, 200)
integral_limit_param = p.addUserDebugParameter("integral limit", 0, 20, 5)
feedforward_torque_param = p.addUserDebugParameter("feedforward torque", -200, 200, 0)
time_step_param = p.addUserDebugParameter("time step", 1 / 1000, 1 / 60, 1 / 240)

pid = PidController()

p.setRealTimeSimulation(False)
try:
    while True:
        keys = p.getKeyboardEvents()

        # 27 = ESC
        if 27 in keys and keys[27] & p.KEY_WAS_TRIGGERED:
            print("ESC pressed, exiting...")
            break

        target_angle = math.radians(p.readUserDebugParameter(target_angle_param))
        time_step = p.readUserDebugParameter(time_step_param)

        pid.kp = p.readUserDebugParameter(kp_param)
        pid.ki = p.readUserDebugParameter(ki_param)
        pid.kd = p.readUserDebugParameter(kd_param)
        pid.output_limit = p.readUserDebugParameter(max_torque_param)
        pid.integral_limit = p.readUserDebugParameter(integral_limit_param)
        pid.feedforward = p.readUserDebugParameter(feedforward_torque_param)

        position, _, _, _ = p.getJointState(robot, joint_id)
        torque = pid.update(target_angle, position)

        p.setJointMotorControl2(
            bodyUniqueId=robot,
            jointIndex=joint_id,
            controlMode=p.TORQUE_CONTROL,
            force=torque,
        )

        p.setTimeStep(time_step)
        p.stepSimulation()
        time.sleep(time_step)

finally:
    p.disconnect()
