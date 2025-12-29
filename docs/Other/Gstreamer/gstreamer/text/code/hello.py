import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

Gst.init(None)

# --- pipeline ---
pipeline = Gst.parse_launch(
    "videotestsrc is-live=true ! "
    "video/x-raw,width=640,height=480 ! "
    "textoverlay name=overlay halignment=left valignment=top shaded-background=false ! "
    "videoconvert ! autovideosink"
)

overlay = pipeline.get_by_name("overlay")

# --- dynamic update ---
counter = 0

def update_text():
    global counter
    counter += 1
    overlay.set_property("text", f"Counter: {counter}")
    return True   # keep timer running

# update every 500 ms
GLib.timeout_add(500, update_text)

pipeline.set_state(Gst.State.PLAYING)

GLib.MainLoop().run()
