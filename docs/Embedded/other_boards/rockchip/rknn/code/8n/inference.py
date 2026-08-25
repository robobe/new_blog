import cv2
import numpy as np
from rknnlite.api import RKNNLite


MODEL = "yolov8n.rknn"
IMAGE = "bus.jpg"
rknn = RKNNLite()

ret = rknn.load_rknn(MODEL)
if ret != 0:
    raise RuntimeError("Could not load RKNN")

ret = rknn.init_runtime()
if ret != 0:
    raise RuntimeError("Could not initialize RKNN runtime")


image = cv2.imread(IMAGE)

image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

image = cv2.resize(image, (640, 640))

input_tensor = np.expand_dims(image, axis=0)

outputs = rknn.inference(
    inputs=[input_tensor]
)

for i, output in enumerate(outputs):
    print(i, output.shape)

rknn.release()