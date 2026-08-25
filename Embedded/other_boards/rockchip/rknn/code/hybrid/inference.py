import cv2
import numpy as np
from rknnlite.api import RKNNLite


MODEL = "yolo26n-rk3566.rknn"
#MODEL = "yolo26n_hybrid.rknn"
#MODEL = "yolo26n0fp16.rknn"
IMAGE = "bus.jpg"
OUTPUT_IMAGE = "bus_detected.jpg"

CONF_THRESHOLD = 0.25
IOU_THRESHOLD = 0.45


# --------------------------------------------------
# Load RKNN model
# --------------------------------------------------

rknn = RKNNLite()

ret = rknn.load_rknn(MODEL)
if ret != 0:
    raise RuntimeError("Could not load RKNN")

ret = rknn.init_runtime()
if ret != 0:
    raise RuntimeError("Could not initialize RKNN runtime")


# --------------------------------------------------
# Load image
# --------------------------------------------------

image_bgr = cv2.imread(IMAGE)

if image_bgr is None:
    raise RuntimeError(f"Could not read {IMAGE}")

# Resize for YOLO
image_bgr = cv2.resize(image_bgr, (640, 640))

# RKNN input uses RGB
image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

input_tensor = np.expand_dims(image_rgb, axis=0)


# --------------------------------------------------
# Inference
# --------------------------------------------------

outputs = rknn.inference(inputs=[input_tensor])

for i, output in enumerate(outputs):
    print(
        f"output[{i}]: "
        f"shape={output.shape}, "
        f"dtype={output.dtype}, "
        f"min={output.min()}, "
        f"max={output.max()}"
    )


# --------------------------------------------------
# Decode YOLO output
# --------------------------------------------------

output = outputs[0]

pred = output[0]

# (84, 8400) -> (8400, 84)
if pred.shape[0] < pred.shape[1]:
    pred = pred.T

boxes = []
scores = []
class_ids = []

for detection in pred:

    x, y, w, h = detection[:4]

    class_scores = detection[4:]

    class_id = np.argmax(class_scores)
    confidence = class_scores[class_id]

    if confidence < CONF_THRESHOLD:
        continue

    left = int(x - w / 2)
    top = int(y - h / 2)

    boxes.append([
        left,
        top,
        int(w),
        int(h)
    ])

    scores.append(float(confidence))
    class_ids.append(int(class_id))


# --------------------------------------------------
# NMS
# --------------------------------------------------

indices = cv2.dnn.NMSBoxes(
    boxes,
    scores,
    CONF_THRESHOLD,
    IOU_THRESHOLD
)


# --------------------------------------------------
# Draw detections
# --------------------------------------------------

for i in indices:

    x, y, w, h = boxes[i]

    class_id = class_ids[i]
    confidence = scores[i]

    # Rectangle
    cv2.rectangle(
        image_bgr,
        (x, y),
        (x + w, y + h),
        (0, 255, 0),
        2
    )

    # Text
    label = f"{class_id} {confidence:.2f}"

    cv2.putText(
        image_bgr,
        label,
        (x, max(y - 10, 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2
    )

    print(
        f"class={class_id} "
        f"confidence={confidence:.3f} "
        f"bbox=({x}, {y}, {w}, {h})"
    )


# --------------------------------------------------
# Save image
# --------------------------------------------------

cv2.imwrite(OUTPUT_IMAGE, image_bgr)

print(f"Saved: {OUTPUT_IMAGE}")

rknn.release()