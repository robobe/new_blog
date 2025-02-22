import gi
import numpy as np
import cv2
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

Gst.init(None)

WIDTH, height, FRAMERATE = 640, 480, 5
PIPELINE = f"""mpegtsmux name=mux alignment=7 ! rtpmp2tpay ! udpsink host=127.0.0.1 port=5000 \
appsrc is-live=true name=source is-live=true format=time \
! video/x-raw,format=BGR,width={WIDTH},height={height},framerate={FRAMERATE}/1 \
! videoconvert \
! timeoverlay time-mode=buffer-count \
! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast key-int-max=15 \
! video/x-h264,profile=main ! mux. \
appsrc name=klv_source is-live=true format=time ! meta/x-klv,parsed=true !  mux. 
"""


def gen_frame(id:int) -> bytes:
    """Generate omage
    Create black image with counter text
    """
    frame = np.zeros((height, WIDTH, 3), dtype=np.uint8)
    cv2.putText(frame, f"Appsrc Video {id}", (50, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
    # Convert the frame to a GStreamer buffer
    data = frame.tobytes()
    return data

def encode_klv(key: bytes, value: bytes) -> bytes:
    """
    Encodes a Key-Length-Value (KLV) triplet.

    :param key: The key as a byte string.
    :param value: The value as a byte string.
    :return: Encoded KLV as a byte string.
    """
    # Encode the length using BER TLV (Basic Encoding Rules for Length)
    length = len(value)
    if length < 128:
        length_bytes = length.to_bytes(1, 'big')  # Short form: single byte
    else:
        # Long form: first byte indicates the number of length bytes
        length_bytes = b'\x80' + length.to_bytes((length.bit_length() + 7) // 8, 'big')
    
    # Combine key, length, and value
    klv = key + length_bytes + value
    return klv

def gen_klv(id) -> bytes:
    key = b'\x01'  # Example key
    data = f"klv: {id}"
    value = data.encode('utf-8')
    klv_bytes = encode_klv(key, value)
    return klv_bytes

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

    ## klv buffer
    klv_bytes = gen_klv(push_data.counter)
    klv_packet_size = len(klv_bytes)
    klvbuf = Gst.Buffer.new_allocate(None, klv_packet_size, None)
    klvbuf.fill(0, klv_bytes)
    klvbuf.pts = buf.dts = push_data.counter * duration_ns
    klvbuf.duration = duration_ns
    retval = klv_appsrc.emit("push-buffer", klvbuf)
    if retval != Gst.FlowReturn.OK:
        print("Error pushing buffer:", retval)
        loop.quit()
    ## klv buffer

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
klv_appsrc = pipeline.get_by_name("klv_source")
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