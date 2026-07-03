---
title: Python source element
tags:
    - gstreamer
    - python
    - plugin
    - source
---

# Python source element

This example creates a custom GStreamer source element in Python. The element is
named `simplepyimagesrc` and generates raw RGB video frames without reading from
a camera, file, or network stream.

The plugin is useful when you need a synthetic source for tests, demos, or for
feeding generated frames into a normal GStreamer pipeline.

## Full code

<details>
<summary>imagesrc.py</summary>
```
--8<-- "docs/Other/Gstreamer/custom_plugin/python/source_element/code/imagesrc.py"
```
</details>

## Element type

`SimplePyImageSrc` inherits from `GstBase.BaseSrc`. A source element has only a
source pad and produces buffers when the pipeline asks for more data.

```python
class SimplePyImageSrc(GstBase.BaseSrc):
```

Use `BaseSrc` when the plugin creates data. Use `BaseTransform` when the plugin
receives buffers, modifies or inspects them, and pushes them downstream.

## Caps and timing

The constants at the top define the stream shape:

- `WIDTH`, `HEIGHT`, and `CHANNELS` define the frame size.
- `FPS` defines the frame rate.
- `FRAME_SIZE` is the number of bytes in each RGB frame.
- `DURATION` is the timestamp distance between two frames.

`CAPS` advertises the exact format the element produces:

```python
CAPS = Gst.Caps.from_string(
    f"video/x-raw,format=RGB,width={WIDTH},height={HEIGHT},framerate={FPS}/1"
)
```

Downstream elements use these caps to know that the buffers are raw RGB video at
`320x240` and `30/1` fps.

## Pad template

The source pad template declares that this element always has one `src` pad and
that the pad produces the caps defined above.

```python
Gst.PadTemplate.new(
    "src",
    Gst.PadDirection.SRC,
    Gst.PadPresence.ALWAYS,
    CAPS,
)
```

This is the only pad template needed for a pure source element.

## Startup

The constructor configures the source to use time-based offsets and sets the
source caps.

```python
self.set_format(Gst.Format.TIME)
self.set_caps(CAPS)
```

`do_start()` resets `frame_number` each time the element starts. That makes the
first generated buffer start at timestamp `0`.

The element is not seekable:

```python
def do_is_seekable(self):
    return False
```

That is correct for a live-style generated stream that cannot jump to an
arbitrary position.

## Buffer creation

`do_create()` is the core method. GStreamer calls it whenever it needs the next
buffer from the source.

The example fills a `bytearray` with an RGB gradient:

```python
data[index] = (x + shift) % 256
data[index + 1] = (y + shift) % 256
data[index + 2] = shift
```

`shift` changes with `frame_number`, so each frame is slightly different.

Then the code allocates a `Gst.Buffer`, copies the image bytes into it, and sets
the timing metadata:

```python
buffer.pts = self.frame_number * DURATION
buffer.dts = buffer.pts
buffer.duration = DURATION
buffer.offset = self.frame_number
```

Correct timestamps are important because downstream elements, sinks, encoders,
and muxers use them for playback timing and synchronization.

Finally, the method returns `Gst.FlowReturn.OK` and the new buffer:

```python
return Gst.FlowReturn.OK, buffer
```

## Register the plugin

The last lines register the Python class with GObject and expose it to
GStreamer as an element factory named `simplepyimagesrc`.

```python
GObject.type_register(SimplePyImageSrc)
__gstelementfactory__ = (
    "simplepyimagesrc",
    Gst.Rank.NONE,
    SimplePyImageSrc,
)
```

The factory name is the name used in `gst-launch-1.0` pipelines.

## Run

Put the plugin file where the Python plugin loader can discover it, then inspect
the element:

```bash
GST_PLUGIN_PATH=$PWD gst-inspect-1.0 simplepyimagesrc
```

Run a preview pipeline:

```bash
GST_PLUGIN_PATH=$PWD gst-launch-1.0 simplepyimagesrc ! videoconvert ! autovideosink
```

For a headless test:

```bash
GST_PLUGIN_PATH=$PWD gst-launch-1.0 simplepyimagesrc num-buffers=30 ! fakesink
```
