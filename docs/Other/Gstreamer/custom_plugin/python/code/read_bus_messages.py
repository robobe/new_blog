#!/usr/bin/env python3
import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst

Gst.init(None)

pipeline = Gst.parse_launch(
    "videotestsrc num-buffers=5 ! "
    "videoconvert ! "
    "busmessagefilter ! "
    "fakesink"
)

bus = pipeline.get_bus()
pipeline.set_state(Gst.State.PLAYING)

while True:
    message = bus.timed_pop_filtered(
        Gst.CLOCK_TIME_NONE,
        Gst.MessageType.ELEMENT | Gst.MessageType.ERROR | Gst.MessageType.EOS,
    )

    if message.type == Gst.MessageType.ELEMENT:
        structure = message.get_structure()
        if structure and structure.get_name() == "frame-info":
            print(
                "frame:",
                structure.get_value("frame-count"),
                "pts:",
                structure.get_value("pts"),
                "duration:",
                structure.get_value("duration"),
            )

    elif message.type == Gst.MessageType.ERROR:
        error, debug = message.parse_error()
        print("error:", error.message)
        print("debug:", debug)
        break

    elif message.type == Gst.MessageType.EOS:
        break

pipeline.set_state(Gst.State.NULL)
