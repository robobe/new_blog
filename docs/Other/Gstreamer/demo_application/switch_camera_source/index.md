---
title: GStreamer switch camera source mode
tags:
    - gstreamer
    - camera
    - v4l2
---

# Switch Camera Source Mode

This demo changes the camera capture mode while the pipeline is running.
The mode is controlled by the caps after `v4l2src`:

```text
video/x-raw,format=YUY2,width=640,height=480,framerate=30/1
```

In the Python code, this caps is placed on a named `capsfilter`:

```python
"v4l2src device=/dev/video0 ! "
"capsfilter name=source_filter "
"caps=video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
```

When we want to switch camera mode, we change the capsfilter to a new supported
camera mode:

```python
new_caps = Gst.Caps.from_string(
    f"video/x-raw,format=YUY2,width={width},height={height},framerate=30/1"
)

source_filter.set_property("caps", new_caps)
```

For example, the demo switches between:

```text
640x480 @ 30 FPS
640x360 @ 30 FPS
```

## Why Pause And Play

A camera source cannot always change resolution while it is actively pushing
frames. The camera driver and GStreamer need a moment to renegotiate the stream
format.

So the demo does this:

```python
wait_state(Gst.State.PAUSED)
source_filter.set_property("caps", new_caps)
wait_state(Gst.State.PLAYING)
```

The important idea:

```text
PLAYING
  ↓
PAUSED      stop active streaming, keep pipeline ready
  ↓
change caps
  ↓
PLAYING     start streaming again with the new camera mode
```

`PAUSED` is useful because the pipeline is not destroyed. The elements stay
created, but streaming is stopped enough for the source caps to change.

## Encoder Bitrate

When the resolution changes, the required bitrate usually changes too.
A smaller frame can use a lower bitrate:

```python
encoder.set_property("bitrate", bitrate_kbps)
```

In the demo:

```python
switch_camera_mode(640, 360, 200)
switch_camera_mode(640, 480, 300)
```

## Force Keyframe

After changing the camera mode, the receiver may need a fresh full H.264 frame.
The demo sends a force keyframe event to the encoder:

```python
force_keyframe()
```

This helps the receiver recover quickly after the mode change instead of waiting
for the next normal keyframe.

## Know Which Camera Modes Are Supported

Before changing caps, check which modes the camera supports.

Use `v4l2-ctl`:

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

Example output:

```text
[0]: 'MJPG' (Motion-JPEG, compressed)
    Size: Discrete 1280x720
        Interval: Discrete 0.033s (30.000 fps)
    Size: Discrete 640x480
        Interval: Discrete 0.033s (30.000 fps)

[1]: 'YUYV' (YUYV 4:2:2)
    Size: Discrete 640x480
        Interval: Discrete 0.033s (30.000 fps)
    Size: Discrete 640x360
        Interval: Discrete 0.033s (30.000 fps)
```

The GStreamer caps must match one of these supported modes.

For `YUYV`, GStreamer usually uses the raw format name `YUY2`:

```text
video/x-raw,format=YUY2,width=640,height=480,framerate=30/1
```

If the camera does not support the requested format, resolution, or FPS, the
pipeline may fail to renegotiate or return an error.

## Demo Code

<details>
<summary>demo.py</summary>

```python
--8<-- "docs/Other/Gstreamer/demo_application/switch_camera_source/code/demo.py"
```

</details>

---

## Receiver Pipeline

The demo sends RTP/H.264 over UDP to port `5600`.
Run this receiver in another terminal before starting the Python sender:

```bash
gst-launch-1.0 -v \
  udpsrc port=5600 caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" ! \
  rtph264depay ! \
  h264parse ! \
  avdec_h264 ! \
  videoconvert ! \
  autovideosink sync=false
```

If the sender uses `host=127.0.0.1`, the stream stays on the loopback
interface. To receive the stream on another machine, change the sender
`udpsink host` to the receiver machine IP address and keep the same port.

## Measure Network Usage With iftop

Use `iftop` to watch the UDP traffic while the demo switches between camera
modes and encoder bitrates.

For the default local sender, monitor the loopback interface:

```bash
sudo iftop -i lo -f "udp port 5600"
```

For a sender that streams to another machine, replace the interface with the
network interface used for that route, for example:

```bash
sudo iftop -i eth0 -f "udp port 5600"
```

While the demo is running, the measured bandwidth should drop when the pipeline
switches to `640x360` with `200` kbps and rise again when it switches back to
`640x480` with `300` kbps. **The values shown by `iftop` include packet overhead,
so they will not exactly match the encoder bitrate.**
