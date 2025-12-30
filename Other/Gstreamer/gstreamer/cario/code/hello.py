import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
gi.require_version("GLib", "2.0")

from gi.repository import Gst, GLib
import cairo
import math

Gst.init(None)

pipeline = Gst.parse_launch(
    "videotestsrc is-live=true pattern=ball ! "
    "video/x-raw,width=640,height=480,framerate=30/1 ! "
    "cairooverlay name=overlay ! "
    "videoconvert ! autovideosink"
)

overlay = pipeline.get_by_name("overlay")

start_time = GLib.get_monotonic_time()

def on_draw(overlay, context, timestamp, duration):
    """
    context   -> cairo.Context
    timestamp -> GstClockTime (ns)
    """

    # Get time in seconds
    t = (GLib.get_monotonic_time() - start_time) / 1e6

    # ---- Draw text ----
    context.set_source_rgba(1, 1, 1, 1)  # white
    context.select_font_face(
        "Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD
    )
    context.set_font_size(40)
    context.move_to(20, 50)
    context.show_text(f"Time: {t:.1f}s")

    # ---- Draw rectangle ----
    x = 100 + 50 * math.sin(t)
    context.set_source_rgba(1, 0, 0, 0.8)  # red
    context.set_line_width(4)
    context.rectangle(x, 100, 200, 120)
    context.stroke()

# Connect draw signal
overlay.connect("draw", on_draw)

pipeline.set_state(Gst.State.PLAYING)

loop = GLib.MainLoop()
try:
    loop.run()
except KeyboardInterrupt:
    pass
finally:
    pipeline.set_state(Gst.State.NULL)
