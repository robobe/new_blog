#!/usr/bin/env python3
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

# Pipeline description as a string
pipeline_str = """
compositor name=comp sink_0::xpos=0 sink_0::ypos=0 sink_1::xpos=640 sink_1::ypos=0 !
    videoconvert ! autovideosink
    v4l2src device=/dev/video0 ! videoconvert ! videoscale ! video/x-raw,width=640,height=480 ! comp.sink_0
    v4l2src device=/dev/video4 ! videoconvert ! videoscale ! video/x-raw,width=640,height=480 ! comp.sink_1
"""

print("Pipeline:\n", pipeline_str)

# Create pipeline from string
pipeline = Gst.parse_launch(pipeline_str)

# Run the pipeline
loop = GLib.MainLoop()

bus = pipeline.get_bus()
bus.add_signal_watch()

def on_message(bus, message):
    t = message.type
    if t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print("Error:", err, debug)
        loop.quit()
    elif t == Gst.MessageType.EOS:
        print("End of stream")
        loop.quit()

bus.connect("message", on_message)

pipeline.set_state(Gst.State.PLAYING)

try:
    loop.run()
except KeyboardInterrupt:
    pass
finally:
    pipeline.set_state(Gst.State.NULL)
