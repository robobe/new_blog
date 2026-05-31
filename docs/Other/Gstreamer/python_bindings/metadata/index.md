---
title: GStreamer metadata with Python bindings
tags:
    - gstreamer
    - python
    - bindings
    - metadata
---

## Object detection metadata plugin

A common video analytics pipeline looks like this:

```bash
camera/file ! decoder ! videoconvert ! objectdetect ! next-element
```

The `objectdetect` element receives each video frame, runs an object detector,
and sends detection results downstream. A detection result usually contains:

- object class, for example `person` or `car`
- confidence score
- bounding box: `x`, `y`, `width`, `height`
- frame timestamp, usually from `buffer.pts`

There are two good ways to send the result.

### Option 1: real buffer metadata

This is the best design for a production plugin. The detector attaches metadata
to the same `Gst.Buffer` that carries the video frame.

```text
Gst.Buffer
  video frame bytes
  metadata:
    objects:
      - label: person
        confidence: 0.91
        box: [120, 40, 80, 180]
```

Downstream elements can then read the detections from the buffer without
changing the video caps.

For true custom `Gst.Meta`, write the plugin in C, C++, or Rust. Python
bindings are good for application logic and prototypes, but custom `Gst.Meta`
registration is not the cleanest path from Python.

Use this design when:

- another GStreamer element must consume the detection data
- metadata must stay attached to the frame
- the pipeline may pass through queues, encoders, muxers, or multiple branches
- you need a reusable plugin

### Option 2: Python prototype with bus messages

For a Python prototype, make a transform element that passes the frame forward
and posts object detections on the pipeline bus. This is not real buffer
metadata, but it is simple and useful while developing the detector.

```text
video buffer -> objectdetect -> same video buffer downstream
                     |
                     +-> bus message with detections
```

Install the Python plugin loader:

```bash
sudo apt install -y gstreamer1.0-python3 python3-gi python3-gst-1.0
```

Example element shape:

```python title="object_detect.py"
import json
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstBase", "1.0")
from gi.repository import Gst, GstBase, GObject

Gst.init(None)


class ObjectDetect(GstBase.BaseTransform):
    __gstmetadata__ = (
        "ObjectDetect",
        "Filter/Analyzer/Video",
        "Detect objects and publish detection metadata",
        "Your Name",
    )

    __gsttemplates__ = (
        Gst.PadTemplate.new(
            "sink",
            Gst.PadDirection.SINK,
            Gst.PadPresence.ALWAYS,
            Gst.Caps.from_string("video/x-raw,format=RGB"),
        ),
        Gst.PadTemplate.new(
            "src",
            Gst.PadDirection.SRC,
            Gst.PadPresence.ALWAYS,
            Gst.Caps.from_string("video/x-raw,format=RGB"),
        ),
    )

    def do_transform_ip(self, buffer):
        detections = self.detect(buffer)
        self.post_detections(buffer, detections)
        return Gst.FlowReturn.OK

    def detect(self, buffer):
        # Map the frame here and run your model.
        # Keep this example small: return one fake detection.
        return [
            {
                "label": "person",
                "confidence": 0.91,
                "box": {"x": 120, "y": 40, "width": 80, "height": 180},
            }
        ]

    def post_detections(self, buffer, detections):
        structure = Gst.Structure.new_empty("object-detections")
        structure.set_value("pts", int(buffer.pts))
        structure.set_value("detections", json.dumps(detections))
        message = Gst.Message.new_element(self, structure)
        self.post_message(message)


GObject.type_register(ObjectDetect)
__gstelementfactory__ = ("objectdetect", Gst.Rank.NONE, ObjectDetect)
```

Use it in a pipeline after converting to the format your detector expects:

```bash
GST_PLUGIN_PATH=$PWD \
gst-launch-1.0 videotestsrc ! videoconvert ! video/x-raw,format=RGB ! objectdetect ! fakesink
```

In a Python application, listen to the bus:

```python title="read_detection_messages.py"
import json
import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst

Gst.init(None)

pipeline = Gst.parse_launch(
    "videotestsrc ! videoconvert ! video/x-raw,format=RGB ! objectdetect ! fakesink"
)

bus = pipeline.get_bus()
pipeline.set_state(Gst.State.PLAYING)

while True:
    msg = bus.timed_pop_filtered(
        Gst.SECOND,
        Gst.MessageType.ELEMENT | Gst.MessageType.ERROR | Gst.MessageType.EOS,
    )
    if not msg:
        continue

    if msg.type == Gst.MessageType.ELEMENT:
        structure = msg.get_structure()
        if structure and structure.get_name() == "object-detections":
            pts = structure.get_value("pts")
            detections = json.loads(structure.get_value("detections"))
            print("pts:", pts, "detections:", detections)

    if msg.type in (Gst.MessageType.ERROR, Gst.MessageType.EOS):
        break

pipeline.set_state(Gst.State.NULL)
```

### Mapping the frame

Inside `detect()`, map the buffer and convert it to an array. You must know the
frame width, height, and format from caps.

```python
success, map_info = buffer.map(Gst.MapFlags.READ)
if not success:
    return []

try:
    frame_bytes = map_info.data
    # Convert frame_bytes to numpy according to caps:
    # RGB => shape is height, width, 3
    # Then run OpenCV, ONNX Runtime, TensorRT, PyTorch, or another detector.
finally:
    buffer.unmap(map_info)
```

For performance, avoid copying frame data when possible. If the detector needs a
different format, put `videoconvert` and a caps filter before your element.

### Production plugin design

For a production plugin that sends real metadata downstream:

1. Implement a `GstBaseTransform` element.
2. Accept a fixed raw video format on the sink pad, for example `video/x-raw,format=RGB`.
3. Run inference in `transform_ip()` so the original buffer continues downstream.
4. Attach object detections as custom `Gst.Meta` or a known analytics metadata type.
5. Write a second downstream element or pad probe that reads the metadata.

In C/Rust, the rough flow is:

```text
transform_ip(buffer):
    frame = map(buffer)
    detections = run_detector(frame)
    meta = gst_buffer_add_custom_meta(buffer, "object-detections")
    meta.objects = detections
    unmap(buffer)
    return GST_FLOW_OK
```

Use Python first to validate the model and pipeline behavior. Move the metadata
attachment to C/Rust when another GStreamer element must consume the detections
as real per-buffer metadata.
