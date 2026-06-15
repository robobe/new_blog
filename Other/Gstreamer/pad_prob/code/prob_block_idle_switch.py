#!/usr/bin/env python3

import gi

gi.require_version("Gst", "1.0")

from gi.repository import GLib, Gst

Gst.init(None)


class ProbeSwitchExample:
    def __init__(self):
        self.pipeline = Gst.Pipeline.new("probe-switch-example")
        self.using_effect = False

        self.source = Gst.ElementFactory.make("videotestsrc", "source")
        self.pre_switch_converter = Gst.ElementFactory.make(
            "videoconvert",
            "pre_switch_converter",
        )
        self.pre_switch_caps = Gst.ElementFactory.make("capsfilter", "pre_switch_caps")
        self.upstream_queue = Gst.ElementFactory.make("queue", "upstream_queue")
        self.normal_filter = Gst.ElementFactory.make("identity", "normal_filter")
        self.effect_filter = Gst.ElementFactory.make("videobalance", "effect_filter")
        self.downstream_queue = Gst.ElementFactory.make("queue", "downstream_queue")
        self.converter = Gst.ElementFactory.make("videoconvert", "converter")
        self.sink = Gst.ElementFactory.make("autovideosink", "sink")

        self.elements = (
            self.source,
            self.pre_switch_converter,
            self.pre_switch_caps,
            self.upstream_queue,
            self.normal_filter,
            self.effect_filter,
            self.downstream_queue,
            self.converter,
            self.sink,
        )
        if any(element is None for element in self.elements):
            raise RuntimeError("failed to create one or more GStreamer elements")

        self.source.set_property("is-live", True)
        self.source.set_property("pattern", "smpte")
        self.pre_switch_caps.set_property(
            "caps",
            Gst.Caps.from_string("video/x-raw,format=RGB"),
        )
        self.effect_filter.set_property("brightness", -0.15)
        self.effect_filter.set_property("contrast", 1.8)
        self.effect_filter.set_property("saturation", 0.0)

        for element in self.elements:
            self.pipeline.add(element)

        if not self.source.link(self.pre_switch_converter):
            raise RuntimeError("failed to link source to pre-switch converter")
        if not self.pre_switch_converter.link(self.pre_switch_caps):
            raise RuntimeError("failed to link pre-switch converter to capsfilter")
        if not self.pre_switch_caps.link(self.upstream_queue):
            raise RuntimeError("failed to link capsfilter to upstream queue")
        if not self.upstream_queue.link(self.normal_filter):
            raise RuntimeError("failed to link upstream queue to normal filter")
        if not self.normal_filter.link(self.downstream_queue):
            raise RuntimeError("failed to link normal filter to downstream queue")
        if not self.downstream_queue.link(self.converter):
            raise RuntimeError("failed to link downstream queue to converter")
        if not self.converter.link(self.sink):
            raise RuntimeError("failed to link converter to sink")

        self.switch_pad = self.upstream_queue.get_static_pad("src")
        if self.switch_pad is None:
            raise RuntimeError("failed to get upstream queue src pad")

    def current_filter(self):
        return self.effect_filter if self.using_effect else self.normal_filter

    def next_filter(self):
        return self.normal_filter if self.using_effect else self.effect_filter

    def install_block_probe(self):
        print("install BLOCK | BUFFER probe")
        self.switch_pad.add_probe(
            Gst.PadProbeType.BLOCK | Gst.PadProbeType.BUFFER,
            self.block_buffer_probe,
            None,
        )
        return GLib.SOURCE_REMOVE

    def block_buffer_probe(self, pad, info, user_data):
        # BLOCK holds this pad at the next buffer. REMOVE unblocks it immediately.
        buffer = info.get_buffer()
        pts = buffer.pts if buffer is not None else Gst.CLOCK_TIME_NONE
        print(f"BLOCK | BUFFER probe fired, stream paused at buffer pts={pts}")
        return Gst.PadProbeReturn.REMOVE

    def install_idle_probe(self):
        print("install IDLE probe for pipeline switch")
        self.switch_pad.add_probe(Gst.PadProbeType.IDLE, self.idle_switch_probe, None)
        return GLib.SOURCE_CONTINUE

    def idle_switch_probe(self, pad, info, user_data):
        # IDLE runs while the pad is blocked, giving a stable point to relink.
        old_filter = self.current_filter()
        new_filter = self.next_filter()
        new_name = "grayscale effect" if not self.using_effect else "normal identity"

        print(f"IDLE probe fired, switching to {new_name}")

        old_filter.unlink(self.downstream_queue)
        self.upstream_queue.unlink(old_filter)
        old_filter.set_state(Gst.State.NULL)

        if not self.upstream_queue.link(new_filter):
            raise RuntimeError(f"failed to link upstream queue to {new_filter.name}")
        if not new_filter.link(self.downstream_queue):
            raise RuntimeError(f"failed to link {new_filter.name} to downstream queue")

        new_filter.sync_state_with_parent()
        self.using_effect = not self.using_effect
        return Gst.PadProbeReturn.REMOVE


example = ProbeSwitchExample()
loop = GLib.MainLoop()
bus = example.pipeline.get_bus()
bus.add_signal_watch()


def on_message(bus, message, loop):
    if message.type == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print("ERROR:", err)
        print("DEBUG:", debug)
        loop.quit()
    elif message.type == Gst.MessageType.EOS:
        loop.quit()


bus.connect("message", on_message, loop)

GLib.timeout_add_seconds(2, example.install_block_probe)
GLib.timeout_add_seconds(5, example.install_idle_probe)

example.pipeline.set_state(Gst.State.PLAYING)

try:
    loop.run()
except KeyboardInterrupt:
    pass
finally:
    example.pipeline.set_state(Gst.State.NULL)
