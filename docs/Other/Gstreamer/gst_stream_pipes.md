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

---

## H265 nvidia jetson

```bash
gst-launch-1.0 videotestsrc ! video/x-raw, width=640, height=480, framerate=30/1, format=I420 \
! nvvidconv ! 'video/x-raw(memory:NVMM)' \
! nvv4l2h265enc preset-level=UltraFastPreset \
bitrate=500000 vbv-size=500000 control-rate=GST_V4L2_VIDENC_CONSTANT_BITRATE \
! rtph265pay config-interval=1 \
! udpsink host=10.0.0.1 port=5000 sync=true
```

| property  | description  | more ...  |
|---|---|---|
| vbv-size  | Control the encoder buffer size, affecting bitrate variability | Smaller values = Stable bitrate (good for live streaming) |

```bash
## receive
gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp, encoding-name=H265, payload=96 \
! rtpjitterbuffer latency=10 \
! rtph265depay \
! decodebin \
! fpsdisplaysink sync=true
```