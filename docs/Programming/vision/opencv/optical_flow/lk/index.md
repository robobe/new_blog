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


## Demo

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

