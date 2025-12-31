import gi, time
gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)

FPS = 30
DURATION = Gst.SECOND // FPS

pipeline = Gst.parse_launch(
    'appsrc name=src '
    'caps=video/x-raw,format=RGB,width=640,height=480,framerate=30/1 '
    '! videoconvert ! fpsdisplaysink sync=true'
)

appsrc = pipeline.get_by_name('src')

pipeline.set_state(Gst.State.PLAYING)

pts = 0

while True:
    buf = Gst.Buffer.new_allocate(None, 640*480*3, None)
    buf.pts = pts
    buf.duration = DURATION
    pts += DURATION

    appsrc.emit('push-buffer', buf)
    time.sleep(1 / FPS)
