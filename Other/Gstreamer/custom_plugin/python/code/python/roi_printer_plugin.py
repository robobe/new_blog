#!/usr/bin/env python3
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstBase", "1.0")
gi.require_version("GstVideo", "1.0")
from gi.repository import GLib, Gst, GstBase, GstVideo, GObject

Gst.init(None)


class RoiPrinter(GstBase.BaseTransform):
    __gstmetadata__ = (
        "RoiPrinter",
        "Filter/Analyzer/Video",
        "Read ROI metadata from buffers and print it",
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
        roi = GstVideo.buffer_get_video_region_of_interest_meta_id(buffer, 0)
        if roi:
            label = GLib.quark_to_string(roi.roi_type)
            params = roi.get_param("detection")
            confidence = params.get_value("confidence") if params else None
            print(
                "roi:",
                label,
                "box:",
                (roi.x, roi.y, roi.w, roi.h),
                "confidence:",
                confidence,
            )
        return Gst.FlowReturn.OK


GObject.type_register(RoiPrinter)
__gstelementfactory__ = ("roiprinter", Gst.Rank.NONE, RoiPrinter)
