# Simple GStreamer compositor with two videotestsrc side by side
gst-launch-1.0 \
  compositor name=comp \
    sink_0::xpos=0 sink_0::ypos=0 sink_0::width=320 sink_0::height=240 \
    sink_1::xpos=320 sink_1::ypos=0 sink_1::width=320 sink_1::height=240 ! \
  video/x-raw,width=640,height=240 ! \
  videoconvert ! \
  autovideosink \
  videotestsrc pattern=0 ! \
    video/x-raw,width=320,height=240 ! \
    comp.sink_0 \
  videotestsrc pattern=1 ! \
    video/x-raw,width=320,height=240 ! \
    comp.sink_1
