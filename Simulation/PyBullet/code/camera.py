import time

import cv2
import numpy as np
import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
p.loadURDF("cube_small.urdf", [0, 0, 1])

width = 640
height = 480

view_matrix = p.computeViewMatrix(
    cameraEyePosition=[2, -3, 2],
    cameraTargetPosition=[0, 0, 0.5],
    cameraUpVector=[0, 0, 1],
)

projection_matrix = p.computeProjectionMatrixFOV(
    fov=60,
    aspect=width / height,
    nearVal=0.1,
    farVal=100,
)

while True:
    p.stepSimulation()

    _, _, rgba, depth, seg = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
    )

    rgba = np.array(rgba, dtype=np.uint8).reshape(height, width, 4)

    # PyBullet gives RGBA, OpenCV expects BGR
    bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)

    cv2.imshow("PyBullet Camera", bgr)

    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
        break

    time.sleep(1 / 240)

cv2.destroyAllWindows()
p.disconnect()