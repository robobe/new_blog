import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)

plane_id = p.loadURDF("plane.urdf")

box_id = p.loadURDF(
    "cube.urdf",
    basePosition=[0, 0, 1],
)

for _ in range(1000):
    p.stepSimulation()
    time.sleep(1 / 240)