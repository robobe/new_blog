# klv_receiver.py
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

def on_new_sample(sink):
    sample = sink.emit("pull-sample")
    buf = sample.get_buffer()
    success, map_info = buf.map(Gst.MapFlags.READ)
    if success:
        data = map_info.data
        print("Received KLV:", list(data))
        buf.unmap(map_info)
    return Gst.FlowReturn.OK

pipeline = Gst.parse_launch("""
udpsrc port=5002 caps="application/x-rtp, media=application, encoding-name=SMPTE336M, payload=96" !
rtpklvdepay ! appsink name=sink emit-signals=true sync=true
""")

sink = pipeline.get_by_name("sink")
sink.connect("new-sample", on_new_sample)

pipeline.set_state(Gst.State.PLAYING)
loop = GLib.MainLoop()
loop.run()
