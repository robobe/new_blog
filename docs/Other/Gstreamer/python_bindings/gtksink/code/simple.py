import gi # type: ignore

gi.require_version("Gtk", "3.0")
gi.require_version("Gst", "1.0")

from gi.repository import Gtk, Gst  # type: ignore # noqa: E402

Gst.init(None)

pipeline = Gst.parse_launch(
    "videotestsrc ! videoconvert ! gtksink name=sink"
)

sink = pipeline.get_by_name("sink")

# Get the GTK widget from gtksink
video_widget = sink.props.widget


def on_button_press(widget, event):
    print(
        f"Mouse click: "
        f"x={event.x:.1f}, "
        f"y={event.y:.1f}, "
        f"button={event.button}"
    )


video_widget.add_events(
    1 << 8   # GDK_BUTTON_PRESS_MASK
)

video_widget.connect(
    "button-press-event",
    on_button_press
)

window = Gtk.Window()
window.set_default_size(800, 600)
window.add(video_widget)

window.connect("destroy", Gtk.main_quit)

window.show_all()

pipeline.set_state(Gst.State.PLAYING)

Gtk.main()

pipeline.set_state(Gst.State.NULL)