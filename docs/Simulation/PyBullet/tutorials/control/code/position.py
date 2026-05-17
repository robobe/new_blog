import pybullet as p
import pybullet_data
import time
import pathlib
import math

# setup
p.connect(p.GUI)
p.resetSimulation() # type: ignore
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

# Use the debug panel sliders to change the target while the simulation runs.
# PyBullet uses radians internally, but degrees are easier to adjust manually.
target_angle_param = p.addUserDebugParameter("target angle (deg)", -180, 180, 45)
max_force_param = p.addUserDebugParameter("max motor force", 0, 1000, 200)
position_gain_param = p.addUserDebugParameter("position gain", 0, 1, 0.3)
velocity_gain_param = p.addUserDebugParameter("velocity gain", 0, 1, 1.0)

p.setRealTimeSimulation(True)
try:
    while True:
        keys = p.getKeyboardEvents()

        # 27 = ESC
        if 27 in keys and keys[27] & p.KEY_WAS_TRIGGERED:
            print("ESC pressed, exiting...")
            break

        target_angle_deg = p.readUserDebugParameter(target_angle_param)
        target_angle_rad = math.radians(target_angle_deg)
        max_force = p.readUserDebugParameter(max_force_param)
        position_gain = p.readUserDebugParameter(position_gain_param)
        velocity_gain = p.readUserDebugParameter(velocity_gain_param)

        p.setJointMotorControl2(
            bodyUniqueId=robot,
            jointIndex=joint_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle_rad,
            force=max_force,
            positionGain=position_gain,
            velocityGain=velocity_gain,
        )

        time.sleep(1/240)

finally:
    p.disconnect()
