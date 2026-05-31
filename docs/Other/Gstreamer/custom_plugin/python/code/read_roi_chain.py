#!/usr/bin/env python3
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
gi.require_version("GstVideo", "1.0")
from gi.repository import GLib, Gst, GstApp, GstVideo

Gst.init(None)

pipeline = Gst.parse_launch(
    "videotestsrc num-buffers=5 ! "
    "videoconvert ! "
    "fakedetector ! "
    "roiprinter ! "
    "appsink name=sink emit-signals=false sync=false"
)

appsink = pipeline.get_by_name("sink")
pipeline.set_state(Gst.State.PLAYING)

while True:
    sample = appsink.try_pull_sample(Gst.SECOND)
    if sample is None:
        break

    buffer = sample.get_buffer()
    roi = GstVideo.buffer_get_video_region_of_interest_meta_id(buffer, 0)
    if not roi:
        print("appsink: no ROI metadata")
        continue

    label = GLib.quark_to_string(roi.roi_type)
    params = roi.get_param("detection")
    confidence = params.get_value("confidence") if params else None

    print(
        "appsink:",
        label,
        "box:",
        (roi.x, roi.y, roi.w, roi.h),
        "confidence:",
        confidence,
        "pts:",
        int(buffer.pts),
    )

pipeline.set_state(Gst.State.NULL)
