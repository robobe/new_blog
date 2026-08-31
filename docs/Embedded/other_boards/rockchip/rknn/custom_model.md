---
title: Convert Custom Trained YOLO Models
tags:
    - yolo
    - custom
    - rknn
    - radxa
---

[source reference](https://docs.radxa.com/en/som/cm/cm3/app-development/ai/rknn-custom-yolo)

Convert method approach
- FP16/ mixed quantization
- INT8 quantization (RKNN model zoo post processing)

## INT8 quantization

The Rockchip-optimized version of YOLO11 follows one main idea:

> Let the **NPU** run the neural-network calculations,
>  then let the **CPU** perform the small final processing steps.

```text
YOLO11 model (.pt)
        ↓
Export optimized ONNX
        ↓
Convert ONNX to RKNN
        ↓
Run neural network on Rockchip NPU
        ↓
Decode and filter results on CPU
        ↓
Final boxes, classes, and confidence scores
```

## Topics covered by the Rockchip optimization

### Remove post-processing from the model

A normal YOLO model produces raw numbers that must be converted into:

- Bounding-box coordinates
- Class names such as `person` or `bus`
- Confidence scores
- The final set of non-overlapping detections

These steps are called **post-processing**.

Some post-processing operations do not run efficiently on a Rockchip NPU and can be unfriendly to INT8 quantization. Therefore, the optimized model returns raw predictions, while the CPU completes the processing. See the [Rockchip optimization README](https://github.com/airockchip/ultralytics_yolo11/blob/main/RKOPT_README.md).

A simple analogy:

- **NPU:** finds possible objects quickly.
- **CPU:** translates and cleans up the answers.

### Move DFL decoding to the CPU

DFL, or **Distribution Focal Loss**, is part of how modern YOLO models represent bounding-box coordinates.

Instead of directly predicting one coordinate, the model predicts several possible distance values. Post-processing then calculates the final distance.

```text
Model prediction: several possible distances
                         ↓
DFL decoding: calculate one final distance
                         ↓
Final bounding box: x1, y1, x2, y2
```

DFL is useful during training, but its final decoding operations can slow inference on the NPU. The optimization does **not** remove DFL from training. It moves the final decoding step outside the exported model and performs it on the CPU.

### Add a score-sum output

For each possible object location, YOLO predicts many class scores:

```text
person: 0.86
car:    0.01
bus:    0.02
...
```

The optimized model adds one small output containing a summary of those scores. The CPU can check that value first:

```text
Is the score summary below the threshold?
    Yes → skip this location
    No  → examine its class scores and decode its box
```

This reduces unnecessary CPU work. The extra output is created in the optimized [YOLO detection head](https://github.com/airockchip/ultralytics_yolo11/blob/main/ultralytics/nn/modules/head.py).

It does not improve detection accuracy. It is only a speed optimization.

### Export the model to ONNX

Configure the model path and export settings in:

```text
ultralytics/cfg/default.yaml
```

Then export the trained YOLO11 model:

```bash
export PYTHONPATH=./
python ./ultralytics/engine/exporter.py
```

The resulting ONNX model has an output format designed for Rockchip deployment.

### Convert ONNX to RKNN

ONNX is an intermediate model format. It still needs to be converted:

```text
YOLO .pt → ONNX → RKNN
```

RKNN-Toolkit2 performs the conversion and usually quantizes the model to INT8. The resulting `.rknn` file can run on supported Rockchip hardware such as RK3566, RK3568, or RK3588.

Conversion and deployment examples are available in the [official RKNN Model Zoo](https://github.com/airockchip/rknn_model_zoo/tree/main/examples/yolo11).

## Understanding the model outputs

A detection model normally produces predictions at three image scales:

```text
80 × 80  → small objects
40 × 40  → medium objects
20 × 20  → large objects
```

For every scale, the optimized model returns three outputs:

```text
Bounding-box distribution
Class probabilities
Score summary
```

Therefore, you commonly receive nine output tensors:

```text
3 scales × 3 outputs = 9 tensors
```

The Model Zoo's simple [Python YOLO11 example](https://github.com/airockchip/rknn_model_zoo/blob/main/examples/yolo11/python/yolo11.py) uses the box and class outputs but ignores the score-summary output. The optimized [C++ post-processing](https://github.com/airockchip/rknn_model_zoo/blob/main/examples/yolo11/cpp/postprocess.cc) can use that summary for faster filtering.

## Beginner workflow

You do not need to understand the DFL mathematics to use this model. Focus on this pipeline:

1. Train or download a YOLO11 `.pt` model.
2. Export it using Rockchip's YOLO11 fork.
3. Convert the ONNX model into `.rknn`.
4. Load the `.rknn` model on the board.
5. Prepare the image.
6. Run NPU inference.
7. Decode boxes and class scores on the CPU.
8. Apply confidence filtering and NMS.
9. Draw the remaining detections.

The optimization changes **where the work happens**, not what YOLO is supposed to detect.
