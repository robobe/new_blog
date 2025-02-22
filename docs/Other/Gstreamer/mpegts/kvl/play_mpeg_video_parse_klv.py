import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

def bus_callback(bus, message, loop):
    """
    Handle GStreamer bus messages.
    """
    if message.type == Gst.MessageType.EOS:
        print("End-of-stream received, exiting.")
        loop.quit()
    elif message.type == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(f"Error: {err}, Debug Info: {debug}")
        loop.quit()
    return True

def on_new_sample(app_sink):
    klv_sample = klv_sink.emit("pull-sample")
    if klv_sample:
        buffer = klv_sample.get_buffer()
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if success:
            klv_data = bytes(map_info.data)
            buffer.unmap(map_info)
            data = parse_klv(klv_data)
            GLib.log_default_handler("MyApp", GLib.LogLevelFlags.LEVEL_MESSAGE, f"{data}")
    
    return Gst.FlowReturn.OK

def parse_klv(klv_data):
    i = 0
    while i < len(klv_data):
        key = klv_data[i]
        length = klv_data[i + 1]
        value = klv_data[i + 2 : i + 2 + length]
        str_value = value.decode('utf-8')
        i += 2 + length
    return str_value

pipeline_description = """
udpsrc port=5000 caps="application/x-rtp, media=(string)video, payload=(int)33" \
    ! rtpjitterbuffer latency=200 \
    ! rtpmp2tdepay \
    ! tsdemux name=demux \
demux. ! multiqueue name=mq ! video/x-h264 ! decodebin ! videoconvert ! autovideosink
demux. ! mq. mq. ! meta/x-klv ! appsink name=klv_sink
"""
pipeline = Gst.parse_launch(pipeline_description)
counter = 0

klv_sink = pipeline.get_by_name("klv_sink")
klv_sink.set_property("emit-signals", True)
klv_sink.set_property("drop", True)
handler_id = klv_sink.connect("new-sample", on_new_sample)
pipeline.set_state(Gst.State.PLAYING)


bus = pipeline.get_bus()
bus.add_signal_watch()
loop = GLib.MainLoop()
bus.connect("message", bus_callback, loop)

try:
    print("Playing video with appsrc. Press Ctrl+C to stop.")
    loop.run()
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    # Stop the pipeline
    
    pipeline.set_state(Gst.State.NULL)