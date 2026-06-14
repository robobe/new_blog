---
title: Read plugin metadata with appsink
tags:
    - gstreamer
    - python
    - plugin
    - appsink
    - metadata
---

# Read plugin metadata with appsink

Another way to get data from a plugin is to attach metadata to each buffer and
pull the buffer from `appsink`.

This keeps the data attached to the frame:

```text
videotestsrc ! appsinkmetafilter ! appsink
                    |
                    +-> GstVideoRegionOfInterestMeta on the buffer
```

This example uses `GstVideo.VideoRegionOfInterestMeta`, which is useful for
object detection boxes.

!!! info "gst_buffer_get_video_region_of_interest_meta_id"
    `gst_buffer_get_video_region_of_interest_meta_id()` is used to retrieve a specific Region of Interest (ROI) metadata attached to a GstBuffer.

    The ROI metadata type is GstVideoRegionOfInterestMeta and is defined in the GStreamer video library.

    ```
    GstMeta
        └── GstVideoRegionOfInterestMeta
            └── optional Gst.Structure params
    ```

    Attaches a Gst.Structure inside the ROI meta as extra parameters.

    ```python
    params = Gst.Structure.new_empty("detection")
    roi.add_param(params)
    ```


## Plugin code

The plugin adds one ROI metadata object to every buffer. The ROI contains a
label and bounding box. Extra fields, such as confidence and frame count, are
stored in a `Gst.Structure` parameter attached to the ROI.

```python title="../code/python/appsink_meta_plugin.py"
#!/usr/bin/env python3
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstBase", "1.0")
gi.require_version("GstVideo", "1.0")
from gi.repository import Gst, GstBase, GstVideo, GObject

Gst.init(None)


class AppSinkMetaFilter(GstBase.BaseTransform):
    __gstmetadata__ = (
        "AppSinkMetaFilter",
        "Filter/Analyzer/Video",
        "Attach ROI metadata to buffers for appsink",
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

        roi = GstVideo.buffer_add_video_region_of_interest_meta(
            buffer,
            "person",
            120,
            40,
            80,
            180,
        )

        # add params to roi object 
        params = Gst.Structure.new_empty("detection")
        params.set_value("frame-count", self.frame_count)
        params.set_value("confidence", 0.91)
        roi.add_param(params)

        return Gst.FlowReturn.OK


GObject.type_register(AppSinkMetaFilter)
__gstelementfactory__ = ("appsinkmetafilter", Gst.Rank.NONE, AppSinkMetaFilter)
```

Important blocks:

- `buffer_add_video_region_of_interest_meta(...)` attaches object metadata.
- The ROI label is stored as a GStreamer quark.
- `roi.add_param(params)` attaches extra fields to the ROI.
- The buffer continues downstream to `appsink`.

---

## Read the metadata from appsink

!!! tip "GST_PLUGIN_PATH"
    Don't forget using / export GST_PLUGIN_PATH environment variable
    

### Pull style
The application **pulls** samples from `appsink`, gets the sample buffer, and reads the ROI metadata using `buffer_get_video_region_of_interest_meta_id` from that buffer.

```python title="read_appsink_meta.py"
#!/usr/bin/env python3
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
gi.require_version("GstVideo", "1.0")
from gi.repository import GLib, Gst, GstApp, GstVideo

Gst.init(None)

pipeline = Gst.parse_launch(
    "videotestsrc num-buffers=5 ! "
    "videoconvert ! "
    "appsinkmetafilter ! "
    "appsink name=sink emit-signals=false sync=false"
)

appsink = pipeline.get_by_name("sink")
pipeline.set_state(Gst.State.PLAYING)

while True:
    sample = appsink.try_pull_sample(Gst.SECOND)
    if sample is None:
        break

    buffer = sample.get_buffer()
    roi = GstVideo.buffer_get_video_region_of_interest_meta_id(buffer, 0)
    if not roi:
        print("sample has no ROI metadata")
        continue

    label = GLib.quark_to_string(roi.roi_type)
    params = roi.get_param("detection")

    print(
        "label:",
        label,
        "box:",
        (roi.x, roi.y, roi.w, roi.h),
        "frame:",
        params.get_value("frame-count") if params else None,
        "confidence:",
        params.get_value("confidence") if params else None,
        "pts:",
        int(buffer.pts),
    )

pipeline.set_state(Gst.State.NULL)
```

### Callback style

You can also let `appsink` emit a `new-sample` signal and read each sample from
a callback. This is useful when the application already uses a `GLib.MainLoop`.
The metadata is still buffer metadata, so the callback still gets the sample
from `appsink` and then reads the `Gst.Buffer`.

```python title="read_appsink_meta_callback.py"
#!/usr/bin/env python3
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
from gi.repository import GLib, Gst, GstVideo

Gst.init(None)


def print_roi_from_sample(sample):
    buffer = sample.get_buffer()
    roi = GstVideo.buffer_get_video_region_of_interest_meta_id(buffer, 0)
    if not roi:
        print("sample has no ROI metadata")
        return

    label = GLib.quark_to_string(roi.roi_type)
    params = roi.get_param("detection")

    print(
        "label:",
        label,
        "box:",
        (roi.x, roi.y, roi.w, roi.h),
        "frame:",
        params.get_value("frame-count") if params else None,
        "confidence:",
        params.get_value("confidence") if params else None,
        "pts:",
        int(buffer.pts),
    )


def on_new_sample(appsink):
    sample = appsink.emit("pull-sample")
    if sample is None:
        return Gst.FlowReturn.ERROR

    print_roi_from_sample(sample)
    return Gst.FlowReturn.OK


def on_bus_message(bus, message, loop):
    if message.type == Gst.MessageType.EOS:
        loop.quit()
    elif message.type == Gst.MessageType.ERROR:
        error, debug = message.parse_error()
        print("error:", error.message)
        if debug:
            print("debug:", debug)
        loop.quit()


pipeline = Gst.parse_launch(
    "videotestsrc num-buffers=5 ! "
    "videoconvert ! "
    "appsinkmetafilter ! "
    "appsink name=sink emit-signals=true sync=false"
)

loop = GLib.MainLoop()

appsink = pipeline.get_by_name("sink")
appsink.connect("new-sample", on_new_sample)

bus = pipeline.get_bus()
bus.add_signal_watch()
bus.connect("message", on_bus_message, loop)

pipeline.set_state(Gst.State.PLAYING)
loop.run()
pipeline.set_state(Gst.State.NULL)
```

This is a callback example, not a `Gst.Event` example. A `Gst.Event` is normally
used for pipeline control information such as EOS, seek, flush, or custom
upstream/downstream events. Per-frame ROI data should stay on the `Gst.Buffer`
as metadata.

## Run

Run commands from the `code` folder:

```bash
cd docs/Other/Gstreamer/custom_plugin/python/code
```

Check that GStreamer sees the plugin:

```bash
GST_PLUGIN_PATH=$PWD gst-inspect-1.0 appsinkmetafilter
```

Run the app:

```bash
GST_PLUGIN_PATH=$PWD python3 read_appsink_meta.py
```

Or run the callback version:

```bash
GST_PLUGIN_PATH=$PWD python3 read_appsink_meta_callback.py
```

Expected output:

```text
label: person box: (120, 40, 80, 180) frame: 1 confidence: 0.91 pts: 0
label: person box: (120, 40, 80, 180) frame: 2 confidence: 0.91 pts: 33333333
```



## Pass metadata between plugins

Metadata attached to a `Gst.Buffer` can be read by the next plugin in the pipe.
For example:

```text
videotestsrc ! fakedetector ! roiprinter ! appsink
                 |              |          |
                 |              |          +-> application reads ROI metadata
                 |              +-> reads ROI metadata from the buffer
                 +-> attaches ROI metadata to the buffer
```

`fakedetector` creates the detection. `roiprinter` reads the detection from the
same buffer and passes the buffer downstream unchanged. `appsink` can still read
the same metadata after both plugins.

### Detector plugin

The detector attaches one fake object box to the buffer.

<details>
<summary>fakedetector plugin</summary>
```
--8<-- "docs/Other/Gstreamer/custom_plugin/python/code/python/fake_detector_plugin.py"
```
</details>


### Next plugin reads the metadata

The next plugin reads the bounding box from the same buffer. This example only
prints it. A real drawing plugin would use the box values to draw on the frame.

<details>
<summary>roiprinter plugin</summary>
```
--8<-- "docs/Other/Gstreamer/custom_plugin/python/code/python/roi_printer_plugin.py"
```
</details>

### pipe 

<details>
<summary>pipe</summary>
```
--8<-- "docs/Other/Gstreamer/custom_plugin/python/code/read_roi_chain.py"
```
</details>


---

### Run:

```bash
GST_PLUGIN_PATH=$PWD gst-inspect-1.0 fakedetector roiprinter
GST_PLUGIN_PATH=$PWD python3 read_roi_chain.py
```

Expected output:

```text
roi: person box: (80, 60, 180, 120) confidence: 0.91
appsink: person box: (80, 60, 180, 120) confidence: 0.91 pts: 0
```


