#!/usr/bin/env python3
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstBase", "1.0")
from gi.repository import Gst, GstBase, GObject

Gst.init(None)


class BusMessageFilter(GstBase.BaseTransform):
    __gstmetadata__ = (
        "BusMessageFilter",
        "Filter/Video",
        "Post one bus message for each video buffer",
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

        structure = Gst.Structure.new_empty("frame-info")
        structure.set_value("frame-count", self.frame_count)
        structure.set_value("pts", int(buffer.pts))
        structure.set_value("duration", int(buffer.duration))

        message = Gst.Message.new_element(self, structure)
        self.post_message(message)

        return Gst.FlowReturn.OK


GObject.type_register(BusMessageFilter)
__gstelementfactory__ = ("busmessagefilter", Gst.Rank.NONE, BusMessageFilter)
