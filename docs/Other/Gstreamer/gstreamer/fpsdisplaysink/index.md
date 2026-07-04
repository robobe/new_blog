---
title: fpsdisplaysink
tags:
    - gstreamer
    - fps
    - sink
---

## What it is

`fpsdisplaysink` is a video sink wrapper used for quick performance checks. It
receives video buffers, sends them to a real video sink, and measures:

- current FPS
- average FPS
- dropped-frame rate
- rendered-frame count
- dropped-frame count

By default it draws the FPS text on top of the video. It can also print or emit
the measurements without drawing text.

```bash
gst-launch-1.0 videotestsrc ! fpsdisplaysink
```

## Use another sink as backend

`fpsdisplaysink` is not usually the final renderer by itself. It owns an
internal `video-sink` property, and that property can be set to another sink.

Use `xvimagesink`:

```bash
gst-launch-1.0 videotestsrc ! fpsdisplaysink video-sink=xvimagesink
```

Use `autovideosink`:

```bash
gst-launch-1.0 videotestsrc ! fpsdisplaysink video-sink=autovideosink
```

Use `fakesink` when you want to benchmark the pipeline without displaying
video:

```bash
gst-launch-1.0 videotestsrc ! fpsdisplaysink video-sink=fakesink text-overlay=false
```

Use a more complex backend by quoting the sink bin:

```bash
gst-launch-1.0 videotestsrc ! fpsdisplaysink \
    video-sink="glimagesink sync=false"
```

Set `video-sink` before the pipeline starts. In application code, configure it
while the pipeline is still in `NULL` state.

## Useful properties

| Property | Use |
| --- | --- |
| `video-sink` | Select the real backend sink used to render or consume video. |
| `text-overlay` | Show FPS text on the video. Set `false` for headless logging or benchmarking. |
| `fps-update-interval` | Measurement update period in milliseconds. Default is `500`. |
| `sync` | Sync rendering to the pipeline clock. Set `false` to measure how fast the pipeline can run. |
| `silent` | Disable status message output. |
| `signal-fps-measurements` | Emit the `fps-measurements` signal for application code. |

Example: measure max throughput without display sync:

```bash
gst-launch-1.0 videotestsrc num-buffers=300 ! fpsdisplaysink \
    video-sink=fakesink \
    text-overlay=false \
    sync=false
```

Example: update the FPS display once per second:

```bash
gst-launch-1.0 videotestsrc ! fpsdisplaysink fps-update-interval=1000
```

## Runtime measurements

The element exposes read-only counters:

- `frames-rendered`: number of frames rendered by the backend sink
- `frames-dropped`: number of frames dropped by the backend sink
- `min-fps`: minimum measured FPS
- `max-fps`: maximum measured FPS
- `last-message`: current status text

For applications, enable `signal-fps-measurements=true` and connect to the
`fps-measurements` signal. The callback receives:

- `fps`: current measured FPS
- `droprate`: current dropped-frame rate
- `avgfps`: average FPS

## Common patterns

Display video and overlay FPS:

```bash
gst-launch-1.0 v4l2src ! videoconvert ! fpsdisplaysink
```

Measure a decode pipeline without drawing overlay text:

```bash
gst-launch-1.0 filesrc location=input.mp4 ! decodebin ! videoconvert ! \
    fpsdisplaysink text-overlay=false video-sink=autovideosink
```

Benchmark a pipeline without a real display:

```bash
gst-launch-1.0 filesrc location=input.mp4 ! decodebin ! videoconvert ! \
    fpsdisplaysink text-overlay=false video-sink=fakesink sync=false
```

## Reference

- [GStreamer fpsdisplaysink documentation](https://gstreamer.freedesktop.org/documentation/debugutilsbad/fpsdisplaysink.html)
