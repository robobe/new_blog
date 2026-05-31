---
title: Send data from a Python plugin to the GStreamer bus
tags:
    - gstreamer
    - python
    - plugin
    - bus
    - message
---

# Send data from a Python plugin to the GStreamer bus

A plugin can send small control or analytics data to the application by posting
a message on the GStreamer bus. This is useful for detections, counters,
warnings, state changes, and debug data.

The video buffer still flows downstream normally:

```text
videotestsrc ! busmessagefilter ! fakesink
                    |
                    +-> Gst.Message on the pipeline bus
```

Use the bus for small messages. Do not send full frames through the bus.

## Plugin code

The plugin below posts one `Gst.MessageType.ELEMENT` message for every buffer.
The message contains a `Gst.Structure` named `frame-info`.

```python title="../code/python/bus_message_plugin.py"
#!/usr/bin/env python3
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstBase", "1.0")
from gi.repository import Gst, GstBase, GObject

Gst.init(None)


class BusMessageFilter(GstBase.BaseTransform):
    __gstmetadata__ = (
        "BusMessageFilter",
        "Filter/Video",
        "Post one bus message for each video buffer",
        "example",
    )

    __gsttemplates__ = (
        Gst.PadTemplate.new(
            "sink",
            Gst.PadDirection.SINK,
            Gst.PadPresence.ALWAYS,
            Gst.Caps.from_string("video/x-raw"),
        ),
        Gst.PadTemplate.new(
            "src",
            Gst.PadDirection.SRC,
            Gst.PadPresence.ALWAYS,
            Gst.Caps.from_string("video/x-raw"),
        ),
    )

    def __init__(self):
        super().__init__()
        self.frame_count = 0

    def do_transform_ip(self, buffer):
        self.frame_count += 1

        structure = Gst.Structure.new_empty("frame-info")
        structure.set_value("frame-count", self.frame_count)
        structure.set_value("pts", int(buffer.pts))
        structure.set_value("duration", int(buffer.duration))

        message = Gst.Message.new_element(self, structure)
        self.post_message(message)

        return Gst.FlowReturn.OK


GObject.type_register(BusMessageFilter)
__gstelementfactory__ = ("busmessagefilter", Gst.Rank.NONE, BusMessageFilter)
```

Important blocks:

- `Gst.Structure.new_empty("frame-info")` creates the message payload.
- `structure.set_value(...)` adds values to the payload.
- `Gst.Message.new_element(self, structure)` creates an element message.
- `self.post_message(message)` sends it to the pipeline bus.
- `return Gst.FlowReturn.OK` keeps the buffer moving downstream.

## Folder layout

The Python plugin loader expects this layout:

```text
code/
  python/
    bus_message_plugin.py
  read_bus_messages.py
```

Run commands from the `code` folder and point `GST_PLUGIN_PATH` to that folder.

## Check that GStreamer sees the plugin

```bash
cd docs/Other/Gstreamer/custom_plugin/python/code

GST_PLUGIN_PATH=$PWD gst-inspect-1.0 busmessagefilter
```

If the element is not found, enable plugin loading logs:

```bash
GST_PLUGIN_PATH=$PWD \
GST_DEBUG=python:6,pythonplugin:6,GST_PLUGIN_LOADING:5 \
gst-inspect-1.0 busmessagefilter
```

## See messages with gst-launch

`gst-launch-1.0 -m` prints bus messages.

```bash
GST_PLUGIN_PATH=$PWD \
gst-launch-1.0 -m videotestsrc num-buffers=5 ! videoconvert ! busmessagefilter ! fakesink
```

This is good for a quick check, but a real application should read the bus
directly.

## Read messages in Python

```python title="../code/read_bus_messages.py"
#!/usr/bin/env python3
import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst

Gst.init(None)

pipeline = Gst.parse_launch(
    "videotestsrc num-buffers=5 ! "
    "videoconvert ! "
    "busmessagefilter ! "
    "fakesink"
)

bus = pipeline.get_bus()
pipeline.set_state(Gst.State.PLAYING)

while True:
    message = bus.timed_pop_filtered(
        Gst.CLOCK_TIME_NONE,
        Gst.MessageType.ELEMENT | Gst.MessageType.ERROR | Gst.MessageType.EOS,
    )

    if message.type == Gst.MessageType.ELEMENT:
        structure = message.get_structure()
        if structure and structure.get_name() == "frame-info":
            print(
                "frame:",
                structure.get_value("frame-count"),
                "pts:",
                structure.get_value("pts"),
                "duration:",
                structure.get_value("duration"),
            )

    elif message.type == Gst.MessageType.ERROR:
        error, debug = message.parse_error()
        print("error:", error.message)
        print("debug:", debug)
        break

    elif message.type == Gst.MessageType.EOS:
        break

pipeline.set_state(Gst.State.NULL)
```

Run it:

```bash
GST_PLUGIN_PATH=$PWD python3 read_bus_messages.py
```

Expected output:

```text
frame: 1 pts: 0 duration: 33333333
frame: 2 pts: 33333333 duration: 33333333
frame: 3 pts: 66666666 duration: 33333333
```

## When to use bus messages

Use bus messages when the application needs to know about data produced by the
plugin.

Good examples:

- object detections as JSON or simple fields
- frame counters
- tracking status
- warnings from the plugin
- model load status

Do not use bus messages when another GStreamer element must receive metadata
attached to the exact frame. For that, use real `Gst.Meta` on the buffer.
