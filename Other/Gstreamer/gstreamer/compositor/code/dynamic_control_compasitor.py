#!/usr/bin/env python3
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

import threading
from fastapi import FastAPI
import uvicorn

Gst.init(None)

# --- Pipeline setup ---
pipeline = Gst.Pipeline.new("api-comp")

comp = Gst.ElementFactory.make("compositor", "comp")
capsfilter = Gst.ElementFactory.make("capsfilter", "caps")
caps = Gst.Caps.from_string("video/x-raw,width=640,height=480,framerate=30/1")
capsfilter.set_property("caps", caps)

sink = Gst.ElementFactory.make("ximagesink", "sink")  # window output

pipeline.add(comp)
pipeline.add(capsfilter)
pipeline.add(sink)

comp.link(capsfilter)
capsfilter.link(sink)

sources = {}  # name -> (src, conv, sinkpad)

# --- Safe functions to add/remove sources ---
# def update_layout(name, xpos, ypos, w, h):

def safe_add_source(name, pattern, xpos, ypos, w, h):
    if name in sources:
        print(f"{name} already exists")
        
        return f"{name} already exists"
    src = Gst.ElementFactory.make("videotestsrc", name)
    src.set_property("is-live", True)
    src.set_property("pattern", pattern)
    conv = Gst.ElementFactory.make("videoconvert", f"{name}_conv")

    # Add elements to pipeline
    pipeline.add(src)
    pipeline.add(conv)
    src.link(conv)

    sinkpad = comp.get_request_pad("sink_%u")
    conv.link(comp)

    # Set layout
    sinkpad.set_property("xpos", xpos)
    sinkpad.set_property("ypos", ypos)
    sinkpad.set_property("width", w)
    sinkpad.set_property("height", h)

    # Sync state with pipeline
    src.sync_state_with_parent()
    conv.sync_state_with_parent()

    sources[name] = (src, conv, sinkpad)
    print(f"Added {name}")
    return f"Added {name}"

def safe_remove_source(name):
    if name not in sources:
        return f"{name} not found"
    src, conv, sinkpad = sources.pop(name)
    print(f"Removing {name}")

    # Pause the elements first
    src.set_state(Gst.State.NULL)
    conv.set_state(Gst.State.NULL)

    # Release pad
    comp.release_request_pad(sinkpad)
    conv.unlink(comp)

    # Remove from pipeline
    pipeline.remove(src)
    pipeline.remove(conv)

    return f"Removed {name}"

# --- Layouts ---
def layout_side_by_side():
    """Two sources side by side"""
    safe_add_source("src1", 0, 0, 0, 320, 480)
    safe_add_source("src2", 1, 320, 0, 320, 480)
    safe_remove_source("src3")

def layout_pip():
    """Fullscreen src1 with small overlay src3"""
    safe_add_source("src1", 0, 0, 0, 640, 480)
    safe_add_source("src3", 2, 440, 340, 200, 140)
    safe_remove_source("src2")

# --- FastAPI server ---
app = FastAPI()

@app.get("/add/{name}")
async def api_add_source(name: str, pattern: int = 0, xpos: int = 0, ypos: int = 0, w: int = 320, h: int = 240):
    def do():
        pipeline.set_state(Gst.State.PAUSED)
        safe_add_source(name, pattern, xpos, ypos, w, h)
        pipeline.set_state(Gst.State.PLAYING)
        return False  # <- important to avoid infinite loop
    GLib.idle_add(do)
    return {"status": f"Adding {name}"}

@app.get("/remove/{name}")
async def api_remove_source(name: str):
    def do():
        pipeline.set_state(Gst.State.PAUSED)
        safe_remove_source(name)
        pipeline.set_state(Gst.State.PLAYING)
        return False
    GLib.idle_add(do)
    return {"status": f"Removing {name}"}

@app.get("/layout/{name}")
async def api_set_layout(name: str):
    def do():
        pipeline.set_state(Gst.State.PAUSED)
        if name == "side_by_side":
            layout_side_by_side()
        elif name == "pip":
            layout_pip()
        pipeline.set_state(Gst.State.PLAYING)
        return False
    GLib.idle_add(do)
    return {"status": f"Switching to {name}"}

# --- Run GStreamer main loop in a background thread ---
def gst_loop():
    pipeline.set_state(Gst.State.PLAYING)
    layout_side_by_side()
    loop = GLib.MainLoop()
    loop.run()

gst_thread = threading.Thread(target=gst_loop, daemon=True)
gst_thread.start()

# --- Run FastAPI ---
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)