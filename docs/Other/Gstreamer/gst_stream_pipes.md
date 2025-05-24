---
tags:
    - gstreamer
    - streaming
    - h265
    - h264
---

# GStreamer video stream pipe
Using gstreamer to stream h264/h265 over network

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
| key-int-max | defines the maximum interval between keyframes, keyframes: is a full image |


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

### Multicast
IPv4 multicast address define from `222.0.0.0` to `239.255.255.255`

To Stream over multicast just change the ip to multicast range
**udpsink** support multicast by default

!!! tip "lo multicast"
    By default multicast disabled on LO

    ```bash
    #check 
    ifconfig lo | grep -i multicast

    # enable
    sudo ifconfig lo multicast
    #or
    sudo ip list set lo multicast on
    ```
     
```bash
gst-launch-1.0 videotestsrc \
! video/x-raw, width=640, height=480, framerate=30/1, format=I420 \
! videoconvert \
! x265enc tune=zerolatency speed-preset=ultrafast key-int-max=30 bitrate=500 \
! rtph265pay config-interval=1 mtu=1400 \
! udpsink host=224.0.0.1 port=5000 sync=true
```

---

# H264

```bash title="sender"
gst-launch-1.0 videotestsrc is-live=true \
! video/x-raw,width=640,height=480,framerate=20/1 \
! videoconvert \
! x264enc tune=zerolatency speed-preset=ultrafast bitrate=500 key-int-max=20 \
! rtph264pay config-interval=1 pt=96 \
! udpsink host=127.0.0.1 port=5000 sync=true

```

```bash title="receiver"
gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" \
! rtph264depay \
! avdec_h264 \
! videoconvert \
! autovideosink sync=true
```

!!! tip "Fast sync between sender and receiver"
    The idea H264/H265 is to send full frame in intervals and between the send partial changes

    - I-frames: (keyframe) full image
    - P-frames:
    - B-frames:

    x264 uses adaptive keyframe intervals, which can go very long without IDRs (especially for static syne)
    The receiver wait for i-frame to sync and it can take 6-8 sec when we are in static sync
    use key-int-max=20 force the encoder to send i-frame each 30 frames, in 20 fps rate it send full image every 1 sec.
     