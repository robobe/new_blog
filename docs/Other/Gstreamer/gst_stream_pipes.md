---
tags:
    - gst
    - gstreamer
    - streaming
---

# GStreamer video stream pipe

## H265 cpu encoder

```bash
gst-launch-1.0 videotestsrc \
! video/x-raw, width=640, height=480, framerate=30/1, format=I420 \
! videoconvert \
! x265enc tune=zerolatency speed-preset=ultrafast key-int-max=30 bitrate=500 \
! rtph265pay config-interval=1 mtu=1400 \
! udpsink host=127.0.0.1 port=5000 sync=true
```

| property  | description  | more ...  |
|---|---|---|
| config-interval  | Send VPS, SPS and PPS Insertion Interval in seconds (default 0)  | help decoder to interpret video stream |


```bash
## receive
gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp, encoding-name=H265, payload=96 \
! rtpjitterbuffer latency=10 \
! rtph265depay \
! decodebin \
! fpsdisplaysink sync=true
```