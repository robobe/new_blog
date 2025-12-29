---
title: GStreamer h264 pipes
tags:
    - gstreamer
    - h264
---

Gstreamer h264 encoder has few implementation for different hardware support

- cpu 
- nvidia
- intel (iGPU)



| Encoder name        | Hardware / Architecture | Platform                      | Plugin package        | Latency | Performance | Notes / When to use                                         |
| ------------------- | ----------------------- | ----------------------------- | --------------------- | ------- | ----------- | ----------------------------------------------------------- |
| **`nvh264enc`**     | NVIDIA **NVENC** (GPU)  | Desktop NVIDIA (RTX / GTX)    | `gst-plugins-bad`     | ⭐⭐⭐⭐    | ⭐⭐⭐⭐⭐       | **Best choice** for RTX. Low latency, very high throughput. |
| **`nvv4l2h264enc`** | NVIDIA **Tegra V4L2**   | Jetson (Orin / Xavier / Nano) | Jetson Multimedia API | ⭐⭐⭐⭐    | ⭐⭐⭐⭐⭐       | Jetson-only. Will NOT work on RTX desktops.                 |
| **`vaapih264enc`**  | **VAAPI** (iGPU)        | Intel / AMD iGPU              | `gstreamer1.0-vaapi`  | ⭐⭐⭐     | ⭐⭐⭐⭐        | Uses `/dev/dri`. Not for NVIDIA systems.                    |
| **`x264enc`**       | CPU (x264)              | Any                           | `gst-plugins-ugly`    | ⭐⭐      | ⭐⭐⭐         | Software encoder. Good quality, higher CPU usage.           |
| **`avenc_h264`**    | CPU (FFmpeg/libav)      | Any                           | `gstreamer1.0-libav`  | ⭐⭐      | ⭐⭐          | Universal fallback. Simplest, but slowest.                  |
| **`v4l2h264enc`**   | V4L2 HW encode          | Embedded SoCs                 | `gst-plugins-good`    | ⭐⭐⭐     | ⭐⭐⭐         | Generic V4L2; quality varies widely by hardware.            |


## Demo
Send video data over network using h264 and RTP


### Receiver
The receiver pipe common for all sender's
```bash
gst-launch-1.0 \
  udpsrc port=5000 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" \
  ! rtph264depay \
  ! avdec_h264 \
  ! videoconvert \
  ! autovideosink

```

---

### x264enc
#### install

```bash
sudo apt update
sudo apt install -y \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-libav

```

#### Sender

```bash
gst-launch-1.0 \
  videotestsrc is-live=true pattern=ball \
  ! video/x-raw,width=640,height=480,framerate=30/1 \
  ! videoconvert \
  ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 \
  ! rtph264pay pt=96 config-interval=1 \
  ! udpsink host=127.0.0.1 port=5000

```

---


### Using Intel iGPU

#### Install
```bash title="install plugins"
sudo apt install -y \
  gstreamer1.0-vaapi \
  intel-media-va-driver \
  vainfo
```

#### Sender
```bash title="intel encoder"
gst-launch-1.0 \
  videotestsrc is-live=true \
  ! videoconvert \
  ! vaapih264enc \
  ! rtph264pay pt=96 \
  ! udpsink host=127.0.0.1 port=5000

```

#### Receiver 

!!! warning "leggy decoding"
    On my machine the cpu decoder `avdec_h264` work better then `vaapih264dec`
    
```bash
gst-launch-1.0 \
  udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
  ! rtph264depay \
  ! h264parse \
  ! vaapih264dec \
  ! videoconvert \
  ! autovideosink

```

```bash title="with jitter"
gst-launch-1.0 \
  udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
  ! rtpjitterbuffer latency=100 drop-on-latency=true \
  ! rtph264depay \
  ! h264parse \
  ! vaapih264dec \
  ! videoconvert \
  ! autovideosink

```

---

### Using nvidia (NVDEC)
- nvh264enc
- nvh264dec

!!! info "jetson"
    NVIDIA Jetson hardware using `nvv4l2h264enc` element
    

Check that the PC support NVENC
```bash 
nvidia-smi -q | grep -i encoder
```

The plugin implement in bad package

#### Sender 
```bash
gst-launch-1.0 \
  videotestsrc is-live=true \
  ! videoconvert \
  ! nvh264enc preset=low-latency-hq rc-mode=cbr bitrate=2000 \
  ! rtph264pay pt=96 config-interval=1 \
  ! udpsink host=127.0.0.1 port=5000

```

#### Receiver

```bash
gst-launch-1.0 \
  udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
  ! rtph264depay \
  ! h264parse \
  ! nvh264dec \
  ! videoconvert \
  ! autovideosink

```

---

TODO: check jetson pipe

### Jetson

```
gst-launch-1.0 \
  videotestsrc is-live=true do-timestamp=true \
  ! video/x-raw,width=1280,height=720,framerate=30/1 \
  ! nvvidconv \
  ! nvv4l2h264enc insert-sps-pps=true iframeinterval=30 bitrate=2000000 \
  ! h264parse \
  ! rtph264pay pt=96 config-interval=1 \
  ! udpsink host=127.0.0.1 port=5000

```

- insert-sps-pps=true    # required for streaming
- iframeinterval=30     # one keyframe per second @30fps
- bitrate=2000000       # 2 Mbps (adjust as needed)

```
gst-launch-1.0 \
  udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
  ! rtpjitterbuffer latency=50 \
  ! rtph264depay \
  ! h264parse \
  ! nvv4l2decoder \
  ! nvvidconv \
  ! autovideosink

```

---

## Demo

```
gst-launch-1.0 \
  videotestsrc is-live=true do-timestamp=true \
  ! video/x-raw,width=640,height=480,framerate=30/1 \
  ! timeoverlay font-desc="Sans, 24" halignment=left valignment=top shaded-background=true \
  ! videoconvert \
  ! nvh264enc preset=low-latency-hq rc-mode=cbr bitrate=2000 \
  ! h264parse \
  ! rtph264pay pt=96 config-interval=1 \
  ! udpsink host=127.0.0.1 port=5000

```

```
gst-launch-1.0 \
  udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
  ! rtpjitterbuffer latency=50 \
  ! rtph264depay \
  ! h264parse \
  ! avdec_h264 \
  ! videoconvert \
  ! fpsdisplaysink
```