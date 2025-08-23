---
title: OpenCV Optical Flow
tags:
    - optical flow
    - opencv
---

{{ page_folder_links() }}

Optical flow is the estimation of the apparent motion of objects or pixels between consecutive frames in a video, based on how the brightness pattern changes.

- Input → two frames (images)
- Output → displacement vectors (u,v) showing where each pixel (or feature) moved

## Types

- Gradient-based (Lucas–Kanade, Horn–Schunck)
- Feature-based (track sparse keypoints)
- Dense methods (Farnebäck, TV-L1, Brox)
- Deep learning–based (FlowNet, RAFT).

## Dense vs sparse

### Dense
Motion is computed fro every pixel in the image


### Sparse
Motion is computed only for selected points (usually features like corners or blobs).

<div class="grid-container">
    <div class="grid-item">
        <a href="lk">
            <p>Lucas–Kanade</p>
        </a>
    </div>
    <div class="grid-item">
    <a href=opencv>
        <p>---</p>
        </a>
    </div>
    <div class="grid-item">
    <a href=nvidia>
        <p>---</p>
        </a>
    </div>
     

</div>