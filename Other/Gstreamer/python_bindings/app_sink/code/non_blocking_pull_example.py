import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst

Gst.init(None)

pipeline = Gst.parse_launch(
    "videotestsrc num-buffers=5 ! "
    "videoconvert ! "
    "appsink name=sink emit-signals=false sync=false"
)

appsink = pipeline.get_by_name("sink")
bus = pipeline.get_bus()

pipeline.set_state(Gst.State.PLAYING)

while True:
    sample = appsink.emit("try-pull-sample", 100 * Gst.MSECOND)
    if sample is not None:
        buffer = sample.get_buffer()
        print("Frame pts:", buffer.pts)
        continue

    message = bus.timed_pop_filtered(
        0,
        Gst.MessageType.ERROR | Gst.MessageType.EOS,
    )
    if message is None:
        print("No sample ready yet")
        continue

    # Stop only when the pipeline reports EOS or ERROR on the bus.
    if message.type == Gst.MessageType.ERROR:
        error, debug = message.parse_error()
        print("Error:", error, debug)

    break

pipeline.set_state(Gst.State.NULL)
