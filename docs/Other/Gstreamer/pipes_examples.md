---
tags:
    - gstreamer
    - pipes
    - demos
---

# Gstreamer pipes example

## Control framerate
Control 60hz source and reduce stream to 10 hz
Queue save only the newest frames


```bash
gst-launch-1.0 v4l2src device=/dev/video4 \
! video/x-raw,width=640,height=480,framerate=60/1 \
! videoconvert \
! queue max-size-buffers=1 leaky=downstream \
! videorate \
! video/x-raw,framerate=10/1 \
! fpsdisplaysink
```