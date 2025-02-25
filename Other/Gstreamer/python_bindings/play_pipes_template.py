import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

SENDER_PIPE = """videotestsrc \
! video/x-raw, width=640, height=480, framerate=30/1, format=I420 \
! videoconvert \
! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 bitrate=500 \
! rtph264pay config-interval=1 mtu=1400 \
! udpsink host=127.0.0.1 port=5000 sync=true"""

RECEIVER_PIPE = """
udpsrc port=5000 \
! application/x-rtp, encoding-name=H264, payload=96 \
! rtpjitterbuffer latency=10 \
! rtph264depay \
! decodebin \
! fpsdisplaysink sync=true
"""
# Create a GStreamer pipeline
sender_pipeline = Gst.parse_launch(SENDER_PIPE)
receiver_pipeline = Gst.parse_launch(RECEIVER_PIPE)

sender_pipeline.set_state(Gst.State.PLAYING)
receiver_pipeline.set_state(Gst.State.PLAYING)

loop = GLib.MainLoop()
loop.run()