import pybullet as p
import time
import pybullet_data


p.connect(p.GUI)
p.setGravity(0, 0, -9.8)

# Default PyBullet assets
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf")

p.setAdditionalSearchPath("/workspace/demos/urdf")
robot_id = p.loadURDF("rrbot.urdf", useFixedBase=True)
# Print joint information
num_joints = p.getNumJoints(robot_id)
print("Total joints:", num_joints)
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    print(i, info[1].decode("utf-8"), "| type =", info[2])

while p.isConnected():
    p.stepSimulation()
    time.sleep(1./240.)
