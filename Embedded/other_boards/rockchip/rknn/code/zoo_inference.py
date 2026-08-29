#!/usr/bin/env python3
"""Minimal YOLOv8 RKNN inference example for a Rockchip board."""

import cv2
import numpy as np
from rknnlite.api import RKNNLite

OBJ_THRESH = 0.25
NMS_THRESH = 0.45
IMG_SIZE = (640, 640)  # width, height
MODEL_PATH = "yolov8n_i8.rknn"
IMAGE_PATH = "bus.jpg"
OUTPUT_PATH = "bus_detected.jpg"


def dfl(position):
    """Convert YOLOv8 distribution logits into four box distances."""
    n, channels, height, width = position.shape
    bins = channels // 4
    values = position.reshape(n, 4, bins, height, width)
    values = np.exp(values - values.max(axis=2, keepdims=True))
    values /= values.sum(axis=2, keepdims=True)
    return (values * np.arange(bins).reshape(1, 1, bins, 1, 1)).sum(axis=2)


def box_process(position):
    """Decode one output branch into x1, y1, x2, y2 boxes."""
    grid_h, grid_w = position.shape[2:]
    col, row = np.meshgrid(np.arange(grid_w), np.arange(grid_h))
    grid = np.stack((col, row)).reshape(1, 2, grid_h, grid_w)
    stride = np.array([IMG_SIZE[0] // grid_w, IMG_SIZE[1] // grid_h]).reshape(1, 2, 1, 1)
    position = dfl(position)
    return np.concatenate(((grid + 0.5 - position[:, :2]) * stride,
                           (grid + 0.5 + position[:, 2:]) * stride), axis=1)


def nms_boxes(boxes, scores):
    """Keep the highest-scoring box among overlapping boxes."""
    x1, y1, x2, y2 = boxes.T
    areas = (x2 - x1) * (y2 - y1)
    order, keep = scores.argsort()[::-1], []
    while order.size:
        i = order[0]
        keep.append(i)
        intersection = (
            np.maximum(0, np.minimum(x2[i], x2[order[1:]]) - np.maximum(x1[i], x1[order[1:]]) + 1e-5)
            * np.maximum(0, np.minimum(y2[i], y2[order[1:]]) - np.maximum(y1[i], y1[order[1:]]) + 1e-5)
        )
        overlap = intersection / (areas[i] + areas[order[1:]] - intersection)
        order = order[np.where(overlap <= NMS_THRESH)[0] + 1]
    return np.asarray(keep)


def post_process(outputs):
    """Decode three YOLO scales, filter weak boxes, then run NMS per class."""
    boxes, probabilities = [], []
    branches = 3
    outputs_per_branch = len(outputs) // branches
    if outputs_per_branch < 2:
        raise ValueError(f"Expected at least 6 model outputs, got {len(outputs)}")
    for i in range(branches):
        boxes.append(box_process(outputs[outputs_per_branch * i]))
        probabilities.append(outputs[outputs_per_branch * i + 1])

    flatten = lambda value: value.transpose(0, 2, 3, 1).reshape(-1, value.shape[1])
    boxes = np.concatenate([flatten(value) for value in boxes])
    probabilities = np.concatenate([flatten(value) for value in probabilities])
    classes = probabilities.argmax(axis=1)
    scores = probabilities.max(axis=1)
    selected = scores >= OBJ_THRESH
    boxes, classes, scores = boxes[selected], classes[selected], scores[selected]

    kept = []
    for class_id in np.unique(classes):
        indices = np.where(classes == class_id)[0]
        kept.extend(indices[nms_boxes(boxes[indices], scores[indices])])
    if not kept:
        return None, None, None
    kept = np.asarray(kept)
    return boxes[kept], classes[kept], scores[kept]


def main():
    model = RKNNLite()
    try:
        # 1. Load the converted model and connect RKNNLite to the board's NPU.
        if model.load_rknn(MODEL_PATH) != 0:
            raise RuntimeError(f"Failed to load {MODEL_PATH}")
        if model.init_runtime() != 0:
            raise RuntimeError("Failed to initialize RKNN runtime")

        # 2. The example image already matches the model's 640 x 640 input.
        # ponytail: fixed-size input keeps the demo focused; add letterboxing for arbitrary images.
        source = cv2.imread(IMAGE_PATH)
        if source is None:
            raise RuntimeError(f"Failed to read {IMAGE_PATH}")
        if source.shape[:2] != IMG_SIZE[::-1]:
            raise ValueError(
                f"Expected a {IMG_SIZE[0]} x {IMG_SIZE[1]} image, got {source.shape[1]} x {source.shape[0]}"
            )

        # 3. RKNN expects RGB with a leading batch dimension: (1, 640, 640, 3).
        image = cv2.cvtColor(source, cv2.COLOR_BGR2RGB)
        # image
        #     └── 640 rows × 640 columns × 3 channels
# 
        # input_tensor
        #     └── batch containing 1 image
        #         └── 640 rows × 640 columns × 3 channels
        # np.expand_dims(image, axis=0).shape
        # (1, 640, 640, 3)
        #
        #  These two expressions are equivalent:
        #   input_tensor = np.expand_dims(image, axis=0)
        #   input_tensor = image[None]
        outputs = model.inference(inputs=[image[None]])
        if outputs is None:
            raise RuntimeError("RKNN inference failed")

        # 4. Convert raw model outputs into final boxes, class IDs, and scores.
        boxes, classes, scores = post_process(outputs)
        if boxes is not None:
            for box, class_id, score in zip(boxes, classes, scores):
                x1, y1, x2, y2 = map(int, box)
                label = f"class {class_id}: {score:.2f}"
                print(f"{label} @ ({x1} {y1} {x2} {y2})")
                cv2.rectangle(source, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(source, label, (x1, max(y1 - 6, 0)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 255), 2)

        # 5. Save the annotated result even when no objects pass the threshold.
        if not cv2.imwrite(OUTPUT_PATH, source):
            raise RuntimeError(f"Failed to write {OUTPUT_PATH}")
        print(f"Saved {OUTPUT_PATH}")
    finally:
        model.release()


if __name__ == "__main__":
    main()
