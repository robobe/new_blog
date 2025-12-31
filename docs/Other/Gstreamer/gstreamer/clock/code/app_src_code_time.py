import gi, time
gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)

FPS = 30
DURATION = Gst.SECOND // FPS
SIZE = 640 * 480 * 3  # RGB

pipeline = Gst.parse_launch(
    'appsrc name=src  is-live=true format=time  '
    'caps=video/x-raw,format=RGB,width=640,height=480,framerate=30/1 '
    '! videoconvert ! fpsdisplaysink text-overlay=true'
)

appsrc = pipeline.get_by_name('src')

pipeline.set_state(Gst.State.PLAYING)

start_time = time.monotonic()

while True:
    now = time.monotonic()
    pts = int((now - start_time) * Gst.SECOND)

    buf = Gst.Buffer.new_allocate(None, SIZE, None)
    buf.pts = pts
    buf.duration = DURATION

    appsrc.emit("push-buffer", buf)
    time.sleep(1 / FPS)
