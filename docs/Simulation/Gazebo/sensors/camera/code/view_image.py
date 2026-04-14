import numpy as np
import cv2
import time

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

def on_image(msg: Image):
    # Get image size
    width = msg.width
    height = msg.height

    # Convert raw bytes to numpy
    img = np.frombuffer(msg.data, dtype=np.uint8)

    # Most Gazebo cameras use RGB8
    img = img.reshape((height, width, 3))

    # Convert RGB → BGR for OpenCV
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # Show image
    cv2.imshow("Gazebo Camera", img)
    cv2.waitKey(1)

node = Node()

ok = node.subscribe(Image, "/camera", on_image)
if not ok:
    raise RuntimeError("Failed to subscribe")

print("Listening to camera...")

# Keep program alive
while True:
    time.sleep(1)