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

pipeline.set_state(Gst.State.PLAYING)

while True:
    sample = appsink.emit("pull-sample")
    if sample is None:
        # pull-sample blocks until a sample is ready. None means EOS or shutdown,
        # not "no sample yet", so continuing would loop after the stream ended.
        break

    buffer = sample.get_buffer()
    print("Frame pts:", buffer.pts)

pipeline.set_state(Gst.State.NULL)
