#!/usr/bin/env python3
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

Gst.init(None)



def on_identity_handoff(identity, buffer):
    print("\n[BUFFER FLOW]")
    print("buffer size:", buffer.get_size())
    print("PTS:", buffer.pts)
    print("DTS:", buffer.dts)
    print("duration:", buffer.duration)

    # metadata-like information
    caps = identity.get_static_pad("src").get_current_caps()
    print("caps:", caps.to_string())


def on_bus_message(bus, message, loop):
    print(f"[BUS MESSAGE] {Gst.MessageType.get_name(message.type)}")

    if message.type == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print("ERROR:", err, debug)
        loop.quit()

    elif message.type == Gst.MessageType.EOS:
        print("EOS")
        loop.quit()


def add_pad_probe(pad, info):
    if info.type & Gst.PadProbeType.BUFFER:
        print("[PAD PROBE] buffer passed")

    if info.type & Gst.PadProbeType.EVENT_DOWNSTREAM:
        event = info.get_event()
        print("[EVENT downstream]", Gst.EventType.get_name(event.type))

    if info.type & Gst.PadProbeType.QUERY_DOWNSTREAM:
        query = info.get_query()
        print("[QUERY downstream]", Gst.QueryType.get_name(query.type))

    return Gst.PadProbeReturn.OK


pipeline = Gst.parse_launch("""
videotestsrc num-buffers=5 !
video/x-raw,format=RGB,width=320,height=240,framerate=1/1 !
identity name=watcher signal-handoffs=true !
fakesink
""")

loop = GLib.MainLoop()

bus = pipeline.get_bus()
bus.add_signal_watch()
bus.connect("message", on_bus_message, loop)

identity = pipeline.get_by_name("watcher")
identity.connect("handoff", on_identity_handoff)

sink_pad = identity.get_static_pad("sink")
sink_pad.add_probe(
    Gst.PadProbeType.BUFFER |
    Gst.PadProbeType.EVENT_DOWNSTREAM |
    Gst.PadProbeType.QUERY_DOWNSTREAM,
    add_pad_probe,
)

pipeline.set_state(Gst.State.PLAYING)

try:
    loop.run()
finally:
    pipeline.set_state(Gst.State.NULL)