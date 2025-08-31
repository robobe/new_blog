import pybullet as p
import pybullet_data
import time

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Create ground plane
planeId = p.loadURDF("plane.urdf")

# Add texture to the ground
textureId = p.loadTexture("tarmac.png")
p.changeVisualShape(planeId, -1, textureUniqueId=textureId)

# Set gravity and run simulation
p.setGravity(0, 0, -9.81)

camera_pos = [0, 0, 10]  # Start at x=0, y=0, z=10m
velocity = 5.0  # 5 m/s in x-direction
width, height = 640, 480
fov, aspect, near, far = 60, width/height, 0.1, 100.0
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
target_pos = [camera_pos[0], camera_pos[1], 0]  # Look downward

while True:
    p.stepSimulation()
    view_matrix = p.computeViewMatrix(camera_pos, target_pos, [0, 1, 0])  # Up vector: y-axis
    _, _, rgb, _, _ = p.getCameraImage(width, height, view_matrix, projection_matrix,
                                    renderer=p.ER_BULLET_HARDWARE_OPENGL)
    time.sleep(1./30.)