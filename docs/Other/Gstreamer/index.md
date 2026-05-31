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
