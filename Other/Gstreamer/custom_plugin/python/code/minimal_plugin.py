#!/usr/bin/env python3
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstBase", "1.0")
from gi.repository import Gst, GstBase, GObject

Gst.init(None)


class MinimalFilter(GstBase.BaseTransform):
    __gstmetadata__ = (
        "MinimalFilter",
        "Filter/Video",
        "Minimal Python GStreamer passthrough element",
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

    def do_transform_ip(self, buffer):
        print("buffer pts:", buffer.pts)
        return Gst.FlowReturn.OK


GObject.type_register(MinimalFilter)
__gstelementfactory__ = ("minimalfilter", Gst.Rank.NONE, MinimalFilter)
