# GStreamer

GStreamer is a multimedia framework for building audio and video applications.
It works by connecting small processing blocks, called **elements**, into a
**pipeline**. Each element has a focused job: read media, decode it, convert it,
encode it, display it, save it, or send it over the network.

A pipeline moves buffers from a source element to one or more processing
elements and finally to a sink element.

```mermaid
flowchart LR
    A["Source<br/>camera, file, network"] --> B["Element<br/>decoder"]
    B --> C["Element<br/>converter/filter"]
    C --> D["Sink<br/>screen, file, stream"]
```

<div class="grid-container">
    <div class="grid-item">
        <a href="gstreamer">
        <p>Element and Pipes</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="python_bindings">
            <p>python_bindings</p>
        </a>
    </div>
     <div class="grid-item">
        <a href="debug_and_trace">
            <p>gstreamer debug and trace</p>
        </a>
    </div>
     <div class="grid-item">
        <a href="custom_plugin/python/">
            <p>custom plugin</p>
        </a>
    </div>
     <div class="grid-item">
        <a href="metadata">
            <p>Metadata</p>
        </a>
    </div>
     <div class="grid-item">
        <a href="pad_prob">
            <p>PAD Prob</p>
        </a>
    </div>
</div>

## install

```bash
sudo apt install -y \
gstreamer1.0-tools \
gstreamer1.0-plugins-base \
gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly \
gstreamer1.0-libav
```


```bash title="python binding"
sudo apt install -y \
python3-gi \
python3-gst-1.0 \
gir1.2-gstreamer-1.0
```

---

## Basic GStreamer terms

Use the pipeline image as a map: data usually moves from left to right, from a
source, through elements, and into a sink. Control messages and questions can
also move between elements.

![alt text](images/gst_framework.png)

### Pipeline

A **pipeline** is the full media graph. It owns the elements and controls their
state: `NULL`, `READY`, `PAUSED`, and `PLAYING`.

```bash title="Simple test video pipeline"
gst-launch-1.0 videotestsrc ! videoconvert ! autovideosink
```

This pipeline creates test video, converts it to a display format, and shows it
on the screen.

### Element

An **element** is one processing block inside the pipeline. Each element has one
job.

```bash
videotestsrc ! videoconvert ! autovideosink
```

- `videotestsrc` creates video frames.
- `videoconvert` converts video format when needed.
- `autovideosink` displays the video.

### Pad

A **pad** is the connection point of an element.

- A **src pad** sends data out.
- A **sink pad** receives data in.

In this example, `videotestsrc` has a source pad connected to the sink pad of
`videoconvert`:

```bash
videotestsrc ! videoconvert
```

### caps

# Caps

In GStreamer, caps means capabilities: a description of what kind of media data can flow through a pad.

A caps string between elements is usually a shorthand for inserting a capsfilter element.

```bash
videotestsrc ! videoconvert ! video/x-raw,format=BGR ! autovideosink
```

```bash
videotestsrc ! videoconvert ! capsfilter caps="video/x-raw,format=BGR" ! autovideosink
```

!!! Note
    “At this exact point in the pipeline, only buffers matching this format may pass.”


## Auto negotiation

```bash
videotestsrc ! videoconvert ! autovideosink
```

GStreamer negotiates automatically.
videotestsrc may produce many possible raw video formats. videoconvert can convert between formats. autovideosink chooses a real video sink available on your system.

GStreamer tries to find a compatible format that all linked elements can handle.

---

Caps are attached to pads

Every GStreamer element has pads.

![alt text](images/caps.png)

### Demo

A's output and B's input must agree on video/x-raw,format=BGR.

```
element A ! video/x-raw,format=BGR ! element B
```

**Only BGR raw video may pass from element_a to element_b.**

!!! Warning
    It does not perform conversion itself.
    only videoconvert,videoscale and videorate are actually change the data


using `gst-inspect` we can check the capability of element pads

```bash
gst-inspect videotestsrc

#

Pad Templates:
  SRC template: 'src'
    Availability: Always
    Capabilities:
      video/x-raw
                 format: { (string)A444_16LE, (string)A444_16BE, (string)AYUV64, (string)RG
BA64_LE, (string)ARGB64, (string)ARGB64_LE, (string)BGRA64_LE, (string)ABGR64_LE, (string)R
GBA64_BE, (string)ARGB64_BE, (string)BGRA64_BE, (string)ABGR64_BE, (string)A422_16LE, (stri
ng)A422_16BE, (string)A420_16LE, (string)A420_16BE, (string)A444_12LE, (string)GBRA_12LE, (
string)A444_12BE
```

### usage

```bash
videoconvert ! video/x-raw,format=BGR ! autovideosink
```

```bash
videoconvert ! capsfilter caps="video/x-raw,format=BGR" ! autovideosink
```

```bash
videoconvert ! 'video/x-raw,format=BGR,width=640,height=480' ! autovideosink
```

!!! Note
    Quoting is especially useful when caps contain parentheses or lists.


---

### Common example

- videoscale    handles width/height
- videorate     handles framerate
- videoconvert  handles pixel format

#### Rate
```
source ! video/x-raw,framerate=30/1 ! videorate ! video/x-raw,framerate=10/1 ! sink
```

!!! note
    `videorate` understand that downstream the pipe expected to video at 10 HZ

```bash title="control rate"
gst-launch-1.0 -v \
  videotestsrc is-live=true ! \
  video/x-raw,framerate=30/1 ! \
  videorate ! \
  video/x-raw,framerate=10/1 ! \
  fpsdisplaysink
```

#TODO: explain how videorate handler down and up rate


#### scale

```bash
gst-launch-1.0 -v \
  videotestsrc is-live=true ! \
  video/x-raw,width=1280,height=720 ! \
  videoscale ! \
  video/x-raw,width=640,height=360 ! \
  videoconvert ! \
  autovideosink
```

#### format

```bash
gst-launch-1.0 -v   videotestsrc is-live=true \
! video/x-raw,format=I420 \
! videoconvert \
! video/x-raw,format=YV12 \
! autovideosink
```

---

### Source

A **source** is an element that starts data flow. It can read from a camera,
file, network stream, or generate test data.

```bash title="Camera source"
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! autovideosink
```

### Sink

A **sink** is an element that ends data flow. It can show video, play audio,
write a file, or send data to the network.

```bash title="Write test video to a file"
gst-launch-1.0 videotestsrc num-buffers=100 ! videoconvert ! x264enc ! mp4mux ! filesink location=test.mp4
```

### Buffer

A **buffer** is one chunk of media data moving through the pipeline. For video,
one buffer is usually one frame. For audio, one buffer is usually a small group
of samples.

Example flow:

```text
videotestsrc creates buffer -> videoconvert changes format -> autovideosink displays buffer
```

### Bus

The **bus** carries messages from the pipeline to the application. The
application uses it to read errors, end-of-stream messages, warnings, and state
changes.

```bash title="Show bus messages while running"
GST_DEBUG=2 gst-launch-1.0 videotestsrc num-buffers=30 ! fakesink
```

When the source sends 30 buffers, the pipeline posts an `EOS` message on the
bus.

### Event

An **event** is a control signal that travels through pads. Events are used for
actions like start, stop, seek, flush, and end-of-stream.

```bash title="Finite pipeline that sends EOS"
gst-launch-1.0 videotestsrc num-buffers=10 ! fakesink
```

After 10 buffers, `videotestsrc` sends an `EOS` event downstream to `fakesink`.

### Query

A **query** asks an element or pipeline for information. Common queries ask for
duration, current position, latency, or supported capabilities.

```bash title="Inspect element pads and caps"
gst-inspect-1.0 videoconvert
```

For example, an application can query a playing file pipeline for the current
position to update a progress bar.


---

## Logging

GStreamer uses debug categories and levels to control log output. The main
environment variable is `GST_DEBUG`, which can enable logs globally or for
specific categories.

Common debug levels are:

- `0`: none
- `1`: errors
- `2`: warnings
- `3`: info
- `4`: debug
- `5`: log
- `6` and higher: very verbose tracing

```bash title="Show warning and error logs"
GST_DEBUG=2 gst-launch-1.0 videotestsrc num-buffers=3 ! fakesink
```

```bash title="Show debug logs for all categories"
GST_DEBUG=4 gst-launch-1.0 videotestsrc num-buffers=3 ! fakesink
```

You can also target one category instead of enabling noisy logs for the whole
pipeline.

```bash title="Show detailed logs only for caps negotiation"
GST_DEBUG=GST_CAPS:5 gst-launch-1.0 videotestsrc ! videoconvert ! fakesink
```

Multiple categories can be combined with commas.

```bash title="Debug filesrc and decodebin"
GST_DEBUG=filesrc:4,decodebin:5 gst-launch-1.0 filesrc location=video.mp4 ! decodebin ! fakesink
```

To save logs to a file, set `GST_DEBUG_FILE`.

```bash title="Write logs to a file"
GST_DEBUG=3 GST_DEBUG_FILE=gstreamer.log gst-launch-1.0 videotestsrc num-buffers=3 ! fakesink
```
