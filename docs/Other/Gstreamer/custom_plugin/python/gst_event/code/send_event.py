import gi

gi.require_version("Gst", "1.0")

from gi.repository import Gst

Gst.init(None)

pipeline = Gst.parse_launch(
    "videotestsrc is-live=true ! eventlistener name=listener ! fakesink"
)

listener = pipeline.get_by_name("listener")
sinkpad = listener.get_static_pad("sink")

pipeline.set_state(Gst.State.PLAYING)

Gst.util_usleep(500_000)

structure = Gst.Structure.new_empty("app-control")
structure.set_value("command", "set-threshold")
structure.set_value("value", 42)

event = Gst.Event.new_custom(
    Gst.EventType.CUSTOM_DOWNSTREAM,
    structure,
)

sinkpad.send_event(event)

Gst.util_usleep(500_000)

pipeline.set_state(Gst.State.NULL)
