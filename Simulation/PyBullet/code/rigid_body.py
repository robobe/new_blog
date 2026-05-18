import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")

collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[0.2, 0.2, 0.2],
)

visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[0.2, 0.2, 0.2],
    rgbaColor=[1, 0, 0, 1],
)

body = p.createMultiBody(
    baseMass=1.0,
    baseCollisionShapeIndex=collision,
    baseVisualShapeIndex=visual,
    basePosition=[0, 0, 1],
)

while True:
    p.stepSimulation()
    time.sleep(1 / 240)