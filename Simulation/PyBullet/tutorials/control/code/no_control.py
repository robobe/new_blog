import pybullet as p
import pybullet_data
import time
import pathlib

# setup
p.connect(p.GUI)
p.resetSimulation() # type: ignore
p.setGravity(gravX=0, gravY=0, gravZ=-9.8)
p.setAdditionalSearchPath(path=pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")

# add current file location and its 'urdf' subfolder to search paths
cwd = pathlib.Path(__file__).parent.resolve()
cwd = cwd.joinpath("urdf")
p.setAdditionalSearchPath(cwd.as_posix())

# load URDF
robot = p.loadURDF("rrbot.urdf", basePosition=[0, 0, 0.5], useFixedBase=True)

# In PyBullet, joints often have a default motor/controller active. 
# If you leave it enabled, 
# the joint may try to hold position or velocity instead of moving naturally. 
# This line switches joint 0 to velocity control but sets force=0, 
# meaning the motor is allowed to apply zero torque.
p.setJointMotorControl2(robot, 0, p.VELOCITY_CONTROL, force=0)  # free joint
p.resetJointState(robot, 0, 0.2)  # small initial angle so gravity can act

p.setRealTimeSimulation(True)
try:
    while True:
        keys = p.getKeyboardEvents()

        # 27 = ESC
        if 27 in keys and keys[27] & p.KEY_WAS_TRIGGERED:
            print("ESC pressed, exiting...")
            break

        time.sleep(1/240)

finally:
    p.disconnect()