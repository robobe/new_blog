"""simple pipeline and wait for bus message

Tags:
    - add_signal_watch
    - bus
    - pipeline
        - set_state
        - get_bus
"""
import gi
import numpy as np
import cv2
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

WIDTH, height, FRAMERATE = 640, 480, 20
PIPELINE = f"""appsrc is-live=true name=source is-live=true format=time \
! video/x-raw,format=BGR,width={WIDTH},height={height},framerate={FRAMERATE}/1 \
! videoconvert \
! timeoverlay time-mode=buffer-count \
! x264enc tune=zerolatency bitrate=1000 speed-preset=ultrafast key-int-max={FRAMERATE} \
! video/x-h264,profile=main \
! mpegtsmux \
! udpsink host=127.0.0.1 port=5000"""


# change pipe for rtp support
PIPELINE = f"""appsrc is-live=true name=source is-live=true format=time \
! video/x-raw,format=BGR,width={WIDTH},height={height},framerate={FRAMERATE}/1 \
! videoconvert \
! timeoverlay time-mode=buffer-count \
! queue \
! x264enc tune=zerolatency bitrate=1000 speed-preset=ultrafast key-int-max={FRAMERATE} \
! video/x-h264,profile=main \
! mpegtsmux alignment=7 \
! rtpmp2tpay \
! udpsink host=127.0.0.1 port=5000"""

def gen_frame(id:int) -> bytes:
    """Generate omage
    Create black image with counter text
    """
    frame = np.zeros((height, WIDTH, 3), dtype=np.uint8)
    cv2.putText(frame, f"Appsrc Video {id}", (50, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
    # Convert the frame to a GStreamer buffer
    data = frame.tobytes()
    return data

def push_data():
    """push gen image to gst buffer
    set pts dts and duration
    Returns:
        _type_: _description_
    """
    data = gen_frame(push_data.counter)
    buf = Gst.Buffer.new_allocate(None, len(data), None)
    buf.fill(0, data)

    duration_ns = int(1e9 / FRAMERATE)
    buf.pts = buf.dts = push_data.counter * duration_ns
    buf.duration = duration_ns
    GLib.log_default_handler("MyApp", GLib.LogLevelFlags.LEVEL_MESSAGE, f"This {push_data.counter}")
    # Push the buffer to appsrc
    retval = appsrc.emit("push-buffer", buf)
    if retval != Gst.FlowReturn.OK:
        print("Error pushing buffer:", retval)
        loop.quit()

    push_data.counter += 1
    return True

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
    elif message.type == Gst.MessageType.STATE_CHANGED:
        old, new, pending = message.parse_state_changed()
        print(f"State changed from {old.value_name} to {new.value_name}")
    return True

pipeline = Gst.parse_launch(PIPELINE)
appsrc = pipeline.get_by_name("source")
pipeline.set_state(Gst.State.PLAYING)
push_data.counter = 0
bus = pipeline.get_bus()
bus.add_signal_watch()
loop = GLib.MainLoop()
bus.connect("message", bus_callback, loop)
# gst timer
GLib.timeout_add(1000 // FRAMERATE, push_data)
try:
    loop.run()
except KeyboardInterrupt:
    print("Exiting...")
finally:
    pipeline.set_state(Gst.State.NULL)