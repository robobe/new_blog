---
title: GStreamer Python plugin
tags:
    - gstreamer
    - python
    - plugin
---

# GStreamer Python plugin

A Python plugin is useful for fast prototyping. For high-performance frame
processing or custom `Gst.Meta`, move the final plugin to C, C++, or Rust.

## Posts

<div class="grid-container">
    <div class="grid-item">
        <a href="bus_message">
            <p>bus message</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="appsink_meta">
            <p>appsink metadata</p>
        </a>
    </div>
</div>

## Install

```bash
sudo apt install -y gstreamer1.0-python3-plugin-loader python3-gi python3-gst-1.0
```

## Folder layout

The Python plugin loader searches for Python files inside a `python/` directory
under `GST_PLUGIN_PATH`.

```text
my_plugins/
  python/
    minimal_plugin.py
```

Run GStreamer with:

```bash
GST_PLUGIN_PATH=$PWD/my_plugins gst-inspect-1.0 minimalfilter
```

## Basic blocks

### Imports and init

Load the GStreamer bindings and initialize GStreamer.

```python
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstBase", "1.0")
from gi.repository import Gst, GstBase, GObject

Gst.init(None)
```

### Element class

Most simple filters inherit from `GstBase.BaseTransform`. It receives one buffer
on the sink pad, processes it, and pushes one buffer on the source pad.

```python
class MinimalFilter(GstBase.BaseTransform):
    pass
```

### Plugin metadata

`__gstmetadata__` describes the element shown by `gst-inspect-1.0`.

```python
__gstmetadata__ = (
    "MinimalFilter",
    "Filter/Video",
    "Minimal Python GStreamer passthrough element",
    "example",
)
```

### Pad templates

Pad templates define what the element can accept and produce. This example
accepts any raw video and outputs any raw video.

```python
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
```

### Buffer processing

`do_transform_ip()` means transform in-place. The same buffer continues
downstream.

```python
def do_transform_ip(self, buffer):
    print("buffer pts:", buffer.pts)
    return Gst.FlowReturn.OK
```

Use this method to inspect timestamps, map frame bytes, run a detector, draw on
the frame, or post messages.

### Register the element

The factory name is the name used in a pipeline.

```python
GObject.type_register(MinimalFilter)
__gstelementfactory__ = ("minimalfilter", Gst.Rank.NONE, MinimalFilter)
```

## Minimal plugin

Full example:

```python title="code/minimal_plugin.py"
#!/usr/bin/env python3
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstBase", "1.0")
from gi.repository import Gst, GstBase, GObject

Gst.init(None)


class MinimalFilter(GstBase.BaseTransform):
    __gstmetadata__ = (
        "MinimalFilter",
        "Filter/Video",
        "Minimal Python GStreamer passthrough element",
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

    def do_transform_ip(self, buffer):
        print("buffer pts:", buffer.pts)
        return Gst.FlowReturn.OK


GObject.type_register(MinimalFilter)
__gstelementfactory__ = ("minimalfilter", Gst.Rank.NONE, MinimalFilter)
```

## Test

Put the file under `python/minimal_plugin.py`, then run:

```bash
GST_PLUGIN_PATH=$PWD gst-inspect-1.0 minimalfilter
```

Run a pipeline:

```bash
GST_PLUGIN_PATH=$PWD gst-launch-1.0 videotestsrc num-buffers=5 ! minimalfilter ! fakesink
```

If GStreamer does not find the element, enable plugin loading logs:

```bash
GST_PLUGIN_PATH=$PWD \
GST_DEBUG=python:6,pythonplugin:6,GST_PLUGIN_LOADING:5 \
gst-inspect-1.0 minimalfilter
```

If the registry cache is stale, remove it:

```bash
rm ~/.cache/gstreamer-1.0/registry.*.bin
```
