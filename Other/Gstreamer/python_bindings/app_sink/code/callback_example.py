import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

Gst.init(None)

pipeline = Gst.parse_launch(
    "videotestsrc ! videoconvert ! appsink name=sink emit-signals=true"
)

appsink = pipeline.get_by_name("sink")


def on_new_sample(sink):
    sample = sink.emit("pull-sample")
    print("Frame")
    return Gst.FlowReturn.OK


appsink.connect("new-sample", on_new_sample)

pipeline.set_state(Gst.State.PLAYING)

GLib.MainLoop().run()