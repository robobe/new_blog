---
tags:
    - gstreamer
    - python
    - bindings
    - gst-bindings
---

# GStreamer python bindings

## Install

```bash
sudo apt-get install gstreamer1.0-tools gstreamer1.0-python3 

```

## Simple usage

```python
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

# Create a GStreamer pipeline
pipeline = Gst.parse_launch("videotestsrc ! videoconvert ! autovideosink")

# Start playing the video
pipeline.set_state(Gst.State.PLAYING)

# Creates a main loop to keep the program running.
loop = GLib.MainLoop()
loop.run()
```

---

## AppSrc

todo: explain the code

```python
--8<-- "docs/Other/Gstreamer/python_bindings/gst_appsrc_example.py"
```