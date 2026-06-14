import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstBase", "1.0")

from gi.repository import Gst, GstBase, GObject

Gst.init(None)


class EventListener(GstBase.BaseTransform):

    __gstmetadata__ = (
        "EventListener",
        "Transform",
        "Listen to custom downstream events",
        "example",
    )

    __gsttemplates__ = (
        Gst.PadTemplate.new(
            "sink",
            Gst.PadDirection.SINK,
            Gst.PadPresence.ALWAYS,
            Gst.Caps.new_any(),
        ),
        Gst.PadTemplate.new(
            "src",
            Gst.PadDirection.SRC,
            Gst.PadPresence.ALWAYS,
            Gst.Caps.new_any(),
        ),
    )

    def do_sink_event(self, event):

        if event.type == Gst.EventType.CUSTOM_DOWNSTREAM:
            structure = event.get_structure()

            if structure and structure.get_name() == "app-control":
                print(
                    "event:",
                    structure.get_value("command"),
                    structure.get_value("value"),
                )

        return GstBase.BaseTransform.do_sink_event(self, event)

    def do_transform_ip(self, buffer):
        return Gst.FlowReturn.OK


GObject.type_register(EventListener)

__gstelementfactory__ = (
    "eventlistener",
    Gst.Rank.NONE,
    EventListener,
)
