#!/usr/bin/env python3

import gi

gi.require_version("Gst", "1.0")

from gi.repository import GLib, Gst

Gst.init(None)


def buffer_probe(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    print(f"buffer pts={buffer.pts}, size={buffer.get_size()}")
    return Gst.PadProbeReturn.OK


pipeline = Gst.parse_launch(
    "videotestsrc is-live=true ! "
    "identity name=tap ! "
    "videoconvert ! "
    "autovideosink"
)

tap = pipeline.get_by_name("tap")
pad = tap.get_static_pad("sink")
pad.add_probe(Gst.PadProbeType.BUFFER, buffer_probe, None)

loop = GLib.MainLoop()
bus = pipeline.get_bus()
bus.add_signal_watch()


def on_message(bus, message, loop):
    if message.type == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print("ERROR:", err)
        print("DEBUG:", debug)
        loop.quit()
    elif message.type == Gst.MessageType.EOS:
        loop.quit()


bus.connect("message", on_message, loop)
pipeline.set_state(Gst.State.PLAYING)

try:
    loop.run()
except KeyboardInterrupt:
    pass
finally:
    pipeline.set_state(Gst.State.NULL)