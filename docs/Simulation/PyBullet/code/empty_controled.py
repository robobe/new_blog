import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)

plane_id = p.loadURDF("plane.urdf")

while True:
    p.stepSimulation()
    time.sleep(1 / 240)