import gi
gi.require_version("Gst", "1.0")
gi.require_version("GLib", "2.0")

from gi.repository import Gst, GLib
import time

# Initialize GStreamer
Gst.init(None)

# Create pipeline
pipeline = Gst.parse_launch(
    "videotestsrc is-live=true pattern=smpte ! "
    "video/x-raw,width=640,height=480,framerate=30/1 ! "
    "identity name=probe signal-handoffs=true ! "
    "textoverlay name=overlay "
    "font-desc='Sans 48' "
    "halignment=left valignment=top shaded-background=false ! "
    "videoconvert ! autovideosink"
)

# Get elements
probe = pipeline.get_by_name("probe")
overlay = pipeline.get_by_name("overlay")

# State for FPS calculation
frame_count = 0
last_time = time.time()

# identity callback
def on_handoff(identity, buffer, pad=None):
    global frame_count, last_time

    frame_count += 1
    now = time.time()

    # Update overlay once per second
    if now - last_time >= 1.0:
        fps = frame_count
        frame_count = 0
        last_time = now

        pts_sec = buffer.pts / Gst.SECOND
        overlay.set_property(
            "text",
            f"FPS: {fps}\nPTS: {pts_sec:.2f}s"
        )

# Connect callback
probe.connect("handoff", on_handoff)

# Start pipeline
pipeline.set_state(Gst.State.PLAYING)

# Run main loop
loop = GLib.MainLoop()

try:
    loop.run()
except KeyboardInterrupt:
    pass
finally:
    pipeline.set_state(Gst.State.NULL)
