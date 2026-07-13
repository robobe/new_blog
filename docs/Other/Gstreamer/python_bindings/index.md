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

<div class="grid-container">
    <div class="grid-item">
        <a href="gtksink">
        <p>gtksink</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="app_src">
        <p>app src</p>
        </a>
    </div>
    <div class="grid-item">
    <a href="app_sink">
        <p>app sink</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="metadata">
        <p>metadata</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="threading">
        <p>threading</p>
        </a>
    </div>
    
</div>

## Simple usage

```python
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

# Create a GStreamer pipeline
PIPELINE = "videotestsrc ! videoconvert ! autovideosink"
pipeline = Gst.parse_launch()

# Start playing the video
pipeline.set_state(Gst.State.PLAYING)

# Creates a main loop to keep the program running.
loop = GLib.MainLoop()
try:
    loop.run()
except KeyboardInterrupt:
    pass
```

---

### pull method
The main thread owns the loop

```python
import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst

Gst.init(None)

PIPELINE = "videotestsrc ! videoconvert ! autovideosink"

pipeline = Gst.parse_launch(PIPELINE)

bus = pipeline.get_bus()

pipeline.set_state(Gst.State.PLAYING)

try:
    while True:
        msg = bus.timed_pop_filtered(
            100 * Gst.MSECOND,  # timeout
            Gst.MessageType.ERROR
            | Gst.MessageType.EOS
            | Gst.MessageType.STATE_CHANGED
        )

        if msg is None:
            continue

        if msg.type == Gst.MessageType.ERROR:
            err, debug = msg.parse_error()
            print(f"ERROR: {err}")
            print(f"DEBUG: {debug}")
            break

        elif msg.type == Gst.MessageType.EOS:
            print("End of stream")
            break

        elif msg.type == Gst.MessageType.STATE_CHANGED:
            if msg.src == pipeline:
                old, new, pending = msg.parse_state_changed()
                print(
                    f"Pipeline state changed: "
                    f"{old.value_nick} -> {new.value_nick}"
                )

except KeyboardInterrupt:
    print("Interrupted")

finally:
    pipeline.set_state(Gst.State.NULL)
```

---

- With `GLib.MainLoop()`: **GStreamer/GLib drives your program**
- With `pull mode`: **Your program drives GStreamer**
