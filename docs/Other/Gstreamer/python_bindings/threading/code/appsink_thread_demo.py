import argparse
import threading
import time

import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst


def thread_label():
    thread = threading.current_thread()
    return f"{thread.name} ident={threading.get_ident()}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--queue", action="store_true")
    parser.add_argument("--slow-callback", action="store_true")
    args = parser.parse_args()

    Gst.init(None)

    queue = "queue max-size-buffers=4 leaky=no ! " if args.queue else ""
    pipeline_desc = (
        "videotestsrc is-live=true pattern=ball ! "
        "video/x-raw,framerate=5/1 ! "
        f"{queue}"
        "videoconvert ! "
        "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=false"
    )

    pipeline = Gst.parse_launch(pipeline_desc)
    appsink = pipeline.get_by_name("sink")
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    loop = GLib.MainLoop()
    frame_count = 0

    print("main started on:", thread_label())
    print("pipeline:", pipeline_desc)

    def on_new_sample(sink):
        nonlocal frame_count
        frame_count += 1

        sample = sink.emit("pull-sample")
        buffer = sample.get_buffer()
        print(
            "appsink callback:",
            thread_label(),
            "frame:",
            frame_count,
            "pts:",
            buffer.pts,
        )

        if args.slow_callback:
            time.sleep(1.0)

        if frame_count >= 10:
            loop.quit()

        return Gst.FlowReturn.OK

    def on_bus_message(_bus, message):
        print("bus watch:", thread_label(), "message:", message.type.value_nick)

        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print("ERROR:", err)
            print("DEBUG:", debug)
            loop.quit()
        elif message.type == Gst.MessageType.EOS:
            loop.quit()

    appsink.connect("new-sample", on_new_sample)
    bus.connect("message", on_bus_message)

    pipeline.set_state(Gst.State.PLAYING)

    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)
        bus.remove_signal_watch()


if __name__ == "__main__":
    main()
