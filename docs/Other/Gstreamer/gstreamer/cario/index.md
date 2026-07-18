---
title: Cairo overlay
tags:
    - gstreamer
    - cairo
    - overlay
    - graphics
---

`cairooverlay` is a GStreamer video element that lets an application draw with
Cairo on top of each video frame. It is useful for debug overlays, bounding
boxes, labels, masks, simple HUDs, timestamps, and other 2D graphics.

## Install

```bash
sudo apt update

sudo apt install \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good

# python binding
sudo apt install \
    python3-gi \
    python3-gst-1.0 \
    gir1.2-gstreamer-1.0

## cpp dependencies

sudo apt install \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libcairo2-dev

```

```bash title="inspect"
gst-inspect-1.0 cairooverlay
```

---

## usage

```text
video source ! raw video caps ! cairooverlay ! videoconvert ! video sink
```

`cairooverlay` emits a `draw` signal for each frame. The callback receives a
`cairo.Context`, the frame timestamp, and the frame duration. You draw into the
context, and GStreamer composites the result over the frame.

## Example

<details>
<summary>demo code</summary>
```python title="code/hello.py"
--8<-- "docs/Other/Gstreamer/gstreamer/cario/code/hello.py"
```
</details>




The example creates this pipeline:

```python
pipeline = Gst.parse_launch(
    "videotestsrc is-live=true pattern=ball ! "
    "video/x-raw,width=640,height=480,framerate=30/1 ! "
    "cairooverlay name=overlay ! "
    "videoconvert ! autovideosink"
)
```

Important parts:

- `videotestsrc is-live=true pattern=ball` creates a live test video.
- `video/x-raw,width=640,height=480,framerate=30/1` fixes the frame size and rate.
- `cairooverlay name=overlay` creates the element that will call our draw function.
- `overlay.connect("draw", on_draw)` connects Python code to the overlay.
- `on_draw(overlay, context, timestamp, duration)` receives the Cairo context.
- `context.show_text(...)` draws text.
- `context.rectangle(...)` and `context.stroke()` draw an animated rectangle.

The rectangle position is animated with:

```python
x = 100 + 50 * math.sin(t)
```

Because the draw callback runs for every frame, changing `x` over time makes the
overlay move.

## Cairo Drawing Options

Cairo is a 2D vector drawing library. The main drawing model is:

1. Select a source color, gradient, pattern, or image.
2. Build a path.
3. Fill, stroke, clip, or paint.

Common drawing operations:

| Category | Cairo calls | Use |
| --- | --- | --- |
| Lines | `move_to`, `line_to`, `rel_line_to`, `stroke` | Polylines, arrows, crosshairs |
| Rectangles | `rectangle`, `fill`, `stroke` | Boxes, panels, detection bounding boxes |
| Curves | `curve_to`, `rel_curve_to` | Bezier curves and smooth paths |
| Arcs | `arc`, `arc_negative` | Circles, rings, gauges, rounded shapes |
| Text | `select_font_face`, `set_font_size`, `show_text`, `text_extents` | Simple labels and timestamps |
| Images | `set_source_surface`, `paint` | Draw another Cairo image surface |
| Paths | `new_path`, `close_path`, `fill_preserve`, `stroke_preserve` | Complex custom shapes |
| Clipping | `clip`, `reset_clip` | Limit drawing to a region |
| Transforms | `translate`, `scale`, `rotate`, `save`, `restore` | Move, resize, rotate, isolate state |
| Transparency | `set_source_rgba` | Alpha blended graphics |
| Line style | `set_line_width`, `set_dash`, `set_line_cap`, `set_line_join` | Dashed boxes, thick lines, rounded joins |
| Compositing | `set_operator` | Control how new pixels combine with old pixels |

Shapes Cairo can draw directly:

- Point markers using small circles or rectangles.
- Straight lines and connected polylines.
- Rectangles and filled rectangles.
- Circles and ellipses using `arc` plus `scale`.
- Circular arcs and rings.
- Triangles, polygons, and arbitrary closed paths.
- Bezier curves.
- Text labels.
- Image stamps from another Cairo surface.
- Masks and clipped regions.

For advanced text layout such as multilingual shaping, line wrapping, rich fonts,
and right-to-left text, use Pango/PangoCairo instead of only `show_text`.

## Using Cairo With Plugin Metadata

You can draw metadata from a previous plugin, but the clean design depends on
where the metadata lives.

If the previous plugin attaches metadata to the buffer, for example
`GstVideoRegionOfInterestMeta`, a downstream drawing element can read that
metadata and draw boxes or labels on the same frame. This is usually the best
design for detection overlays:

```text
source ! detector-plugin-adds-meta ! draw-overlay-plugin ! sink
```

Inside a custom plugin, use a `GstBase.BaseTransform` / `GstVideoFilter` style
element. In `transform_ip`, map the video buffer for write access, read the
metadata from the same `Gst.Buffer`, create a Cairo image surface over the mapped
memory, draw, flush the surface, and unmap the buffer.

`cairooverlay` itself is easier for application-level overlays, but it does not
pass the `Gst.Buffer` to the `draw` callback. The callback only receives the
Cairo context, timestamp, and duration. If you need metadata with `cairooverlay`,
you normally keep metadata in an application-side structure keyed by timestamp or
frame number, then look it up in the draw callback. That works for simple cases,
but a custom draw plugin is more reliable when exact buffer metadata matters.

## Alternatives For Drawing From A Plugin

Options when drawing directly in a plugin:

| Option | Good for | Notes |
| --- | --- | --- |
| Custom `GstBaseTransform` with Cairo | Boxes, labels, vector HUDs | Map raw video memory and draw with Cairo. Works best with Cairo-friendly formats such as `BGRA`, `ARGB`, or `RGBA`. |
| `GstVideoOverlayComposition` / overlay composition meta | Hardware-friendly overlays | Attach overlay composition metadata instead of modifying pixels. Downstream elements may composite it. |
| OpenCV in a plugin | Computer vision overlays | Useful if the plugin already uses OpenCV and buffers are mapped to `numpy` or `cv::Mat`. |
| `textoverlay` / `timeoverlay` | Simple text only | Built-in elements; no custom drawing. |
| `compositor` | Combining streams or images | Good for arranging video layers, not for per-object vector drawing. |
| Application `cairooverlay` | Fast prototypes | Easy callback API, but no direct buffer metadata in the callback. |

Practical rule:

- Use `cairooverlay` when the application controls the overlay data.
- Use a custom transform/filter plugin when the overlay must use metadata
  attached to the exact `Gst.Buffer`.
- Use overlay composition metadata when you want to avoid permanently modifying
  the video pixels and your sink path supports it.
