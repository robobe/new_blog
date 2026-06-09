---
tags:
    - gstreamer
    - python
    - bindings
    - gtk
    - gtksink
---

# GTK sink

`gtksink` is a GStreamer video sink that exposes the video output as a GTK
widget. This is useful when a Python GTK application needs to embed video inside
its own window instead of opening a separate video window.

The important part is the sink `widget` property:

```python
video_widget = sink.props.widget
```

You can then add that widget to a GTK layout, connect GTK events to it, and run
the normal GTK main loop.

## Install

On Ubuntu or Debian:

```bash
sudo apt install python3-gi gir1.2-gtk-3.0 gir1.2-gstreamer-1.0 gstreamer1.0-gtk3 gstreamer1.0-plugins-base
```

Check that the element is available:

```bash
gst-inspect-1.0 gtksink
```

## Simple usage

The example creates a `videotestsrc` pipeline, gets the GTK widget from
`gtksink`, adds it to a `Gtk.Window`, and prints mouse-click coordinates on the
video widget.

<details>
<summary>Simple gtksink example</summary>

```python
--8<-- "docs/Other/Gstreamer/python_bindings/gtksink/code/simple.py"
```

</details>


---

## Demo
- Mouse click event
- Keyboard press event


<details>
<summary>Simple gtksink example</summary>

```python
--8<-- "docs/Other/Gstreamer/python_bindings/gtksink/code/mouse_keyboard.py"
```

</details>