#!/usr/bin/env python3
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstBase", "1.0")
gi.require_version("GstVideo", "1.0")
from gi.repository import Gst, GstBase, GstVideo, GObject

Gst.init(None)


class FakeDetector(GstBase.BaseTransform):
    __gstmetadata__ = (
        "FakeDetector",
        "Filter/Analyzer/Video",
        "Attach one fake object detection ROI to each buffer",
        "example",
    )

    __gsttemplates__ = (
        Gst.PadTemplate.new(
            "sink",
            Gst.PadDirection.SINK,
            Gst.PadPresence.ALWAYS,
            Gst.Caps.from_string("video/x-raw"),
        ),
        Gst.PadTemplate.new(
            "src",
            Gst.PadDirection.SRC,
            Gst.PadPresence.ALWAYS,
            Gst.Caps.from_string("video/x-raw"),
        ),
    )

    def __init__(self):
        super().__init__()
        self.frame_count = 0

    def do_transform_ip(self, buffer):
        self.frame_count += 1

        roi = GstVideo.buffer_add_video_region_of_interest_meta(
            buffer,
            "person",
            80,
            60,
            180,
            120,
        )

        params = Gst.Structure.new_empty("detection")
        params.set_value("frame-count", self.frame_count)
        params.set_value("confidence", 0.91)
        roi.add_param(params)

        return Gst.FlowReturn.OK


GObject.type_register(FakeDetector)
__gstelementfactory__ = ("fakedetector", Gst.Rank.NONE, FakeDetector)
