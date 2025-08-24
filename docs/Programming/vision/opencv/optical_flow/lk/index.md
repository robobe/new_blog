---
title: OpenCV lucas kanade
tags:
    - opencv
    - optical flow
    - lk
    - lucas-kanade
    - goodFeaturesToTrack
    - calcOpticalFlowPyrLK
    - cuda
---

{{ page_folder_links() }}

#TODO:
- API
  - understand error and it's usage

## Demo
- Flow between two frames

<details>
    <summary>LK cpu version</summary>

```python
--8<-- "docs/Programming/vision/opencv/optical_flow/lk/code/simple.py"
```
</details>


| frame_1  | frame_2  | Flow  |
|---|---|---|
| ![](code/test_frame1.jpg)  | ![](code/test_frame2.jpg)  | ![](code/lk_result.jpg)  |


---

## Demo: Cuda
Create cuda version using pin memory for better performance [check opencv with cuda](Programming/vision/opencv/cuda/)


- `cv2.cuda.createGoodFeaturesToTrackDetector`
- `cv2.cuda.SparsePyrLKOpticalFlow_create`

### API
<details>
    <summary>detect</summary>
    d_pts = det.detect(d_gray, mask=None, stream=None)

- d_gray: cv2.cuda_GpuMat, CV_8UC1, grayscale, non-empty.
- mask (optional): cv2.cuda_GpuMat (CV_8U), same size; non-zero=allowed region.
- stream (optional): cv2.cuda_Stream for async execution (some builds omit this param—if you get a TypeError, just drop it).
- d_pts: cv2.cuda_GpuMat of shape (N, 1), type CV_32FC2 (each element is a corner (x, y) in float32).
</details>


<details>
    <summary>calc</summary>
    nextPts, status, err = flow.calc(prevImg, nextImg, prevPts, stream=None)

- prevImg, nextImg: cv.cuda_GpuMat grayscale (CV_8UC1)
- prevPts: cv.cuda_GpuMat of shape (N,1), type CV_32FC2 (points as (x,y))
- returns: nextPts (N×1, CV_32FC2), status (N×1, CV_8U), err (N×1, CV_32F)
</details>



<details>
    <summary>LK cuda version</summary>

```python
--8<-- "docs/Programming/vision/opencv/optical_flow/lk/code/simple_cuda.py"
```
</details>


---

# Demo: Cuda with pin memory and stream

<details>
    <summary>Cuda version with pin memory and stream</summary>

```python
--8<-- "docs/Programming/vision/opencv/optical_flow/lk/code/simple_cuda_stream.py"
```
</details>


---

# Demo: Cuda with Video/Image sequence
- upload new_point back to gpu for next iteration

<details>
    <summary>Cuda version with Image sequence</summary>

```python
--8<-- "docs/Programming/vision/opencv/optical_flow/lk/code/cuda_image_sequence.py"
```
</details>