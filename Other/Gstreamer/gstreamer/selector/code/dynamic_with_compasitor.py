#!/usr/bin/env python3
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

pipeline_str = """
    compositor name=final_comp sink_0::xpos=0 sink_0::ypos=0 sink_1::xpos=640 sink_1::ypos=0 !
        videoconvert ! autovideosink

    v4l2src device=/dev/video0 ! videoconvert ! tee name=tee0
    tee0. ! queue ! selector.sink_0
    tee0. ! queue ! videoscale ! video/x-raw,width=320,height=240 ! both_comp.sink_0

    v4l2src device=/dev/video4 ! videoconvert ! tee name=tee1
    tee1. ! queue ! selector.sink_1
    tee1. ! queue ! videoscale ! video/x-raw,width=320,height=240 ! both_comp.sink_1

    input-selector name=selector ! videoscale ! video/x-raw,width=640,height=480 ! final_comp.sink_0

    compositor name=both_comp sink_0::xpos=0 sink_0::ypos=0 sink_1::xpos=320 sink_1::ypos=0 !
        videoscale ! video/x-raw,width=640,height=480 ! final_comp.sink_1
"""



print("Pipeline:\n", pipeline_str)
pipeline = Gst.parse_launch(pipeline_str)

selector = pipeline.get_by_name("selector")
pads = selector.sinkpads

# Track which pad is active
current_index = {"i": 0}

def switch_source():
    current_index["i"] = (current_index["i"] + 1) % len(pads)
    pad = pads[current_index["i"]]
    selector.set_property("active-pad", pad)
    print(f"Switched selector to camera {current_index['i']}")
    return True  # repeat

bus = pipeline.get_bus()
bus.add_signal_watch()

def on_message(bus, message):
    if message.type == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print("Error:", err, debug)
        loop.quit()
    elif message.type == Gst.MessageType.EOS:
        print("End of stream")
        loop.quit()

bus.connect("message", on_message)

pipeline.set_state(Gst.State.PLAYING)

# Switch every 5s
GLib.timeout_add_seconds(5, switch_source)

loop = GLib.MainLoop()
try:
    loop.run()
except KeyboardInterrupt:
    pass
finally:
    pipeline.set_state(Gst.State.NULL)
