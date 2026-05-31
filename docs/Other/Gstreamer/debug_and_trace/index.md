---
title: GStreamer debug and trace
tags:
    - gstreamer
    - debug
    - trace
    - gst-shark
    - identity
---

## Debug
[Basic tutorial 11: Debugging tools](https://gstreamer.freedesktop.org/documentation/tutorials/basic/debugging-tools.html?gi-language=c)

GStreamer pipelines can fail because an element cannot link, caps are wrong,
timestamps are bad, one branch is slower than the other, or a sink is blocking
the whole pipe. Debug tools help answer three questions:

- What elements are in the pipeline?
- What caps and buffers flow between them?
- Which element is slow or blocked?

## Basic debug commands

Use `gst-inspect-1.0` before building a pipe. It shows element pads, caps,
properties, and plugin information.

```bash title="Inspect an element"
gst-inspect-1.0 videoconvert
```

Use `gst-launch-1.0 -v` to print caps negotiation while the pipeline starts.
This is the fastest way to see the format that moves between elements.

```bash title="Print negotiated caps"
gst-launch-1.0 -v videotestsrc num-buffers=5 ! videoconvert ! fakesink
```

Use `GST_DEBUG` when caps output is not enough. Level `2` prints warnings,
`3` adds info, `4` adds debug messages, and higher levels are very noisy.

```bash title="Show warnings"
GST_DEBUG=2 gst-launch-1.0 videotestsrc ! videoconvert ! autovideosink
```

```bash title="Debug only caps negotiation"
GST_DEBUG=GST_CAPS:5 gst-launch-1.0 videotestsrc ! videoconvert ! fakesink
```

## Debug monitor

For large logs, use a debug log viewer instead of reading terminal output.
`gst-debug-viewer` can open a GStreamer debug log, filter by category, search
for warnings, and show the order of messages.

```bash title="Write a debug log"
GST_DEBUG=3 GST_DEBUG_FILE=gst.log gst-launch-1.0 videotestsrc num-buffers=30 ! fakesink
```



## Pipeline graph

GStreamer can dump the pipeline graph to Graphviz `.dot` files. This is useful
for checking what `decodebin`, `playbin`, or dynamic pads created internally.

```bash title="Dump pipeline graph"
mkdir -p /tmp/gst-dot
GST_DEBUG_DUMP_DOT_DIR=/tmp/gst-dot gst-launch-1.0 videotestsrc num-buffers=5 ! videoconvert ! fakesink
```

```bash title="Convert graph to PNG"
dot -Tpng /tmp/gst-dot/*.dot -O
```

Open the generated image and check the real element chain, pad names, and caps.

## Elements that help debug a pipeline

### fakesink

Use `fakesink` to remove the real output from the test. If the pipeline works
with `fakesink` but not with `autovideosink`, the sink or display path is the
problem.

```bash
gst-launch-1.0 videotestsrc ! videoconvert ! fakesink
```

### fpsdisplaysink

Use `fpsdisplaysink` to measure rendered FPS. It is useful for finding a slow
decoder, converter, encoder, or sink.

```bash
gst-launch-1.0 videotestsrc ! videoconvert ! fpsdisplaysink text-overlay=false sync=false
```

### identity

`identity` passes buffers through and can print information about them. It is a
good probe point between two elements.

```bash title="Print buffers passing through identity"
gst-launch-1.0 videotestsrc num-buffers=5 ! identity silent=false ! fakesink
```

It can also sleep for each buffer, which is useful for simulating a slow element.

```bash title="Simulate a slow stage"
gst-launch-1.0 videotestsrc ! identity sleep-time=50000 ! fpsdisplaysink sync=false
```

### queue

`queue` creates a new thread and decouples pipeline branches. Put queues around
suspected slow parts to see where buffering grows or where the pipe blocks.

```bash title="Debug a split pipeline with queues"
gst-launch-1.0 videotestsrc ! tee name=t \
  t. ! queue ! videoconvert ! fpsdisplaysink text-overlay=false sync=false \
  t. ! queue ! x264enc tune=zerolatency ! fakesink
```

If one branch is slow and there is no queue, it can block the other branch.

## Finding a bottleneck

Start with a simple pipe and add one element at a time.

```bash title="Source only"
gst-launch-1.0 videotestsrc ! fakesink
```

```bash title="Add conversion"
gst-launch-1.0 videotestsrc ! videoconvert ! fakesink
```

```bash title="Add encoder"
gst-launch-1.0 videotestsrc ! videoconvert ! x264enc tune=zerolatency ! fakesink
```

If performance drops after adding one element, inspect that element first. Check
its caps, properties, thread boundary, and whether the sink is synchronized to
the clock.

Useful checks:

- Add `-v` to verify caps.
- Replace the sink with `fakesink`.
- Add `fpsdisplaysink` to measure FPS.
- Add `identity silent=false` before and after the suspected element.
- Add `queue` before heavy branches.
- Run with `GST_DEBUG=2` and fix warnings first.

## Trace
```bash title="from source to sink"
GST_TRACERS=latency GST_DEBUG=GST_TRACER:7 gst-launch-1.0 videotestsrc num-buffers=30 ! fakesink
```

GStreamer tracers measure runtime behavior. The latency tracer reports how long
buffers take to move through the pipeline or through each element.

```bash
GST_TRACERS="latency(flags=element+pipeline+reported)" GST_DEBUG=GST_TRACER:7 \
  gst-launch-1.0 videotestsrc num-buffers=30 ! videoconvert ! fakesink
```

For long-running performance work, GstShark can collect and visualize tracing
data such as latency, CPU usage, queue levels, and scheduling behavior.

[GstShark getting start](https://developer.ridgerun.com/wiki/index.php/GstShark_-_Getting_Started#Getting_the_code)
