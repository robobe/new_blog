---
title: GStreamer h264 pipes
tags:
    - gstreamer
    - h264
---
H.264 / H.265 encoder compress raw video frames into smaller bitstream.

basic idea

```
Raw frames
   ↓
Find repeated areas between frames
   ↓
Store only differences/motion
   ↓
Quantize details to reduce size
   ↓
Entropy coding
   ↓
Compressed H.264/H.265 stream
```

```
Frame 1: full image
Frame 2: only changes
Frame 3: only changes
```

It use frame types

| Frame   | Meaning                                 |
| ------- | --------------------------------------- |
| I-frame | Full frame, large size                  |
| P-frame | Difference from previous frame          |
| B-frame | Difference using previous/future frames |


---

## Demo: Stream camera over udp using h264 and RTP


### Understand camera output

```
v4l2-ctl -d /dev/video0 --list-formats-ext
ioctl: VIDIOC_ENUM_FMT
	Type: Video Capture

	[0]: 'MJPG' (Motion-JPEG, compressed)
		Size: Discrete 1920x1080
			Interval: Discrete 0.033s (30.000 fps)
		Size: Discrete 1280x960
			Interval: Discrete 0.033s (30.000 fps)
		Size: Discrete 1280x720
			Interval: Discrete 0.033s (30.000 fps)
		Size: Discrete 640x480
			Interval: Discrete 0.033s (30.000 fps)
		Size: Discrete 640x360
			Interval: Discrete 0.033s (30.000 fps)
	[1]: 'YUYV' (YUYV 4:2:2)
		Size: Discrete 640x480
			Interval: Discrete 0.033s (30.000 fps)
		Size: Discrete 640x360
			Interval: Discrete 0.033s (30.000 fps)
```

#### understand x264enc input

```bash title="check sink pad for incoming format"
gst-inspect-1.0 x264enc
Factory Details:
  Rank                     primary (256)
  Long-name                x264 H.264 Encoder
  ...

Plugin Details:
  Name                     x264
  Description              libx264-based H.264 encoder plugin
  ...
GObject
 +----GInitiallyUnowned
       +----GstObject
             +----GstElement
                   +----GstVideoEncoder
                         +----GstX264Enc

Implemented Interfaces:
  GstPreset

Pad Templates:
  SINK template: 'sink'
    Availability: Always
    Capabilities:
      video/x-raw
              framerate: [ 0/1, 2147483647/1 ]
                  width: [ 1, 2147483647 ]
                 height: [ 1, 2147483647 ]
                 format: { (string)Y444, (string)Y42B, (string)I420, (string)YV12, (string)NV12, (string)GRAY8, (string)Y444_10LE, (string)I422_10LE, (string)I420_10LE }

```
### Pipe
```bash title="sender"
gst-launch-1.0 -v v4l2src device=/dev/video0 \
! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 \
! videoconvert \
! x264enc bitrate=1000 speed-preset=ultrafast \
  tune=zerolatency \
  key-int-max=30 \
  bframes=0 byte-stream=true \
! h264parse config-interval=1 \
! rtph264pay pt=96 mtu=1200 config-interval=1 \
! udpsink host=127.0.0.1 port=5600 sync=false async=false
```

```bash title="receiver"
gst-launch-1.0 -v udpsrc port=5600 caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" \
! rtpjitterbuffer latency=200 \
! rtph264depay \
! h264parse \
! avdec_h264 \
! videoconvert \
! autovideosink sync=false


```

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
The following pipe use different type of encoder that use hardware and different implementation

- [#x264enc](#x264enc)
- [#nvh264enc](#using-nvidia-nvdec)
- [#vaapih264enc](#using-intel-igpu)

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