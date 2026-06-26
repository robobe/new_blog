---
title: Control Video Bandwidth
tags:
    - gstreamer
    - video
    - bandwidth
    - fastapi
---

# Control Video Bandwidth

This demo controls the bandwidth of a live camera stream without rebuilding the
pipeline. It changes three things while the stream is running:

- `videocrop` reduces the visible frame area before encoding.
- `videorate` plus a capsfilter lowers the output FPS.
- `x264enc` properties control encoder bitrate and keyframe interval.

The example sends RTP/H.264 over UDP to `127.0.0.1:5600` and exposes a FastAPI
server for runtime control.

## Pipeline

The pipeline starts with a fixed camera mode, then applies crop, FPS, encoder,
and RTP payload stages:

```python
self.pipeline = Gst.parse_launch(
    "v4l2src device=/dev/video0 ! "
    "video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
    "videocrop name=center_crop ! "
    "videorate name=rate drop-only=true ! "
    "capsfilter name=stream_caps ! "
    "videoconvert ! "
    "x264enc name=encoder "
    "bitrate=300 speed-preset=ultrafast tune=zerolatency "
    "key-int-max=30 vbv-buf-capacity=1000 "
    "bframes=0 byte-stream=true ! "
    "h264parse config-interval=1 ! "
    "rtph264pay pt=96 mtu=1400 config-interval=1 ! "
    "identity name=bandwidth_meter silent=true ! "
    "udpsink host=127.0.0.1 port=5600 sync=false async=false"
)
```

The named elements are saved after `Gst.parse_launch()`:

```python
self.crop = self.pipeline.get_by_name("center_crop")
self.capsfilter = self.pipeline.get_by_name("stream_caps")
self.encoder = self.pipeline.get_by_name("encoder")
self.bandwidth_meter = self.pipeline.get_by_name("bandwidth_meter")
```

## Reduce Resolution With videocrop

`videocrop` does not ask the camera for a new mode. The camera still captures
`640x480`, and the pipeline crops the center of that frame before encoding.
This is useful when the camera does not support the exact smaller resolution you
want to stream.

For a target size, the controller calculates the crop borders:

```python
def _center_crop_values(self, width: int, height: int):
    left = (CAMERA_WIDTH - width) // 2
    right = CAMERA_WIDTH - width - left
    top = (CAMERA_HEIGHT - height) // 2
    bottom = CAMERA_HEIGHT - height - top
    return {"left": left, "right": right, "top": top, "bottom": bottom}
```

Then it applies those values to `videocrop`:

```python
self.crop.set_property("left", crop["left"])
self.crop.set_property("right", crop["right"])
self.crop.set_property("top", crop["top"])
self.crop.set_property("bottom", crop["bottom"])
```

A `320x240` output from a `640x480` camera frame crops `160` pixels from the
left and right, and `120` pixels from the top and bottom.

## Lower FPS With videorate

`videorate` adapts the buffer cadence. In this demo it is configured with
`drop-only=true`, so lowering the FPS drops frames instead of duplicating frames
to raise FPS.

The output rate is controlled by the capsfilter after `videorate`:

```python
caps = Gst.Caps.from_string(
    f"video/x-raw,width={width},height={height},framerate={fps}/1"
)
self.capsfilter.set_property("caps", caps)
```

For example, setting `fps=10` produces:

```text
video/x-raw,width=640,height=480,framerate=10/1
```

Lower FPS reduces bandwidth because fewer frames reach the encoder.

## Control Encoder Bandwidth

The encoder is the main bandwidth control point. `x264enc` uses bitrate in
kilobits per second:

```python
self.encoder.set_property("bitrate", bitrate_kbps)
self.encoder.set_property("key-int-max", key_int_max)
```

The demo starts with:

```text
bitrate=300
key-int-max=30
vbv-buf-capacity=1000
tune=zerolatency
speed-preset=ultrafast
```

Use lower bitrate values for constrained links. Use a shorter keyframe interval
when the receiver must recover quickly after stream changes, at the cost of more
bandwidth.

After changing encoder settings, the controller asks the encoder to produce a
fresh keyframe:

```python
event = GstVideo.video_event_new_downstream_force_key_unit(
    Gst.CLOCK_TIME_NONE,
    Gst.CLOCK_TIME_NONE,
    0,
    True,
    0,
)
sinkpad.send_event(event)
```

## GLib.MainContext()

FastAPI handles HTTP requests on the asyncio side. GStreamer owns the pipeline
on a GLib thread. The controller uses `GLib.MainContext()` and a command queue so
pipeline changes are applied from the GStreamer thread.

```python
self.commands: queue.Queue[Command] = queue.Queue()
self.context = GLib.MainContext()
self.loop = GLib.MainLoop.new(self.context, False)
self.thread = threading.Thread(target=self._thread_main, daemon=True)
```

Each API request submits a command:

```python
future = stream_controller.submit(name, args)
return await asyncio.wait_for(asyncio.wrap_future(future), timeout=3)
```

The controller wakes the GLib context and handles queued commands:

```python
def submit(self, name: str, args: dict | None = None) -> Future:
    future = Future()
    self.commands.put(Command(name=name, args=args or {}, future=future))
    self._invoke(self._process_commands)
    return future

def _invoke(self, callback):
    self.context.invoke_full(GLib.PRIORITY_DEFAULT, callback)
```

This keeps the web server responsive while the pipeline state and element
properties remain owned by the GStreamer side.

## API

Start the app:

```bash
python3 bandwidth_app.py
```

Open the browser UI:

```text
http://localhost:8002
```

Useful endpoints:

```text
GET  /status
GET  /profiles
POST /profile
POST /stream
```

Apply the smaller low-bandwidth profile:

```bash
curl -X POST http://localhost:8002/profile \
  -H "Content-Type: application/json" \
  -d '{"profile":"center_320x240"}'
```

Apply custom stream settings:

```bash
curl -X POST http://localhost:8002/stream \
  -H "Content-Type: application/json" \
  -d '{"width":320,"height":240,"fps":10,"bitrate_kbps":100,"key_int_max":30}'
```

## Receiver Pipeline

Run this receiver in another terminal:

```bash
gst-launch-1.0 -v \
  udpsrc port=5600 caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" ! \
  rtph264depay ! \
  h264parse ! \
  avdec_h264 ! \
  videoconvert ! \
  autovideosink sync=false
```

## Measure Bandwidth

The pipeline includes an `identity` element named `bandwidth_meter`. A pad probe
counts outgoing RTP bytes over a rolling one-second window:

```python
srcpad = self.bandwidth_meter.get_static_pad("src")
srcpad.add_probe(Gst.PadProbeType.BUFFER, self._on_bandwidth_buffer)
```

The `/status` response reports:

```text
measured_bandwidth_bps
measured_bandwidth_kbps
measured_window_seconds
measured_total_bytes
```

You can also inspect the UDP traffic with `iftop`:

```bash
sudo iftop -i lo -f "udp port 5600"
```

## Demo Code

<details>
<summary>bandwidth_control.py</summary>

```python
--8<-- "docs/Other/Gstreamer/demo_application/control_video_bandwidth/code/bandwidth_control.py"
```

</details>

<details>
<summary>bandwidth_app.py</summary>

```python
--8<-- "docs/Other/Gstreamer/demo_application/control_video_bandwidth/code/bandwidth_app.py"
```

</details>

<details>
<summary>videoscale_demo.py</summary>

```python
--8<-- "docs/Other/Gstreamer/demo_application/control_video_bandwidth/code/videoscale_demo.py"
```

</details>

<details>
<summary>static/bandwidth.html</summary>

```html
--8<-- "docs/Other/Gstreamer/demo_application/control_video_bandwidth/code/static/bandwidth.html"
```

</details>
