import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import time

Gst.init()

pipeline_str = "appsrc name=klv_source caps=meta/x-klv,parsed=true format=3 ! rtpklvpay ! udpsink host=127.0.0.1 port=5002"
pipeline = Gst.parse_launch(pipeline_str)

appsrc = pipeline.get_by_name("klv_source")

def push_klv_data():
    # Create dummy KLV data
    klv_data = bytes([
        0x06, 0x0E, 0x2B, 0x34, 0x01, 0x01, 0x01, 0x01, 0x0E, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00,  # Key
        0x02,  # Length
        0x01, 0x02   # Value
    ])

    buf = Gst.Buffer.new_allocate(None, len(klv_data), None)
    buf.fill(0, klv_data)

    # Add timing information (time in nanoseconds)
    pts = Gst.CLOCK_TIME_NONE
    if hasattr(push_klv_data, 'timestamp'):
        pts = push_klv_data.timestamp
    else:
        pts = 0
        push_klv_data.timestamp = pts

    buf.pts = pts  # Presentation timestamp
    buf.dts = pts  # Decoding timestamp (same as PTS for KLV)
    buf.duration = 1000000000  # Duration in nanoseconds (1 second)

    # Update timestamp for next buffer
    push_klv_data.timestamp += 1000000000  # Increment by 1 second

    appsrc.emit("push-buffer", buf)
    return True

# Set up the main loop
loop = GLib.MainLoop()

# Start the pipeline
pipeline.set_state(Gst.State.PLAYING)

# Push KLV data periodically
GLib.timeout_add_seconds(1, push_klv_data)

try:
    loop.run()
except KeyboardInterrupt:
    pass

# Clean up
pipeline.set_state(Gst.State.NULL)
