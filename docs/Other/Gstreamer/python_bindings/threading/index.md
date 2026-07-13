---
title: GStreamer Python Threading
tags:
    - gstreamer
    - python
    - bindings
    - threading
    - appsink
---

# GStreamer Python threading

In a Python GStreamer application there are usually two worlds:

- Python application thread, often the main thread
- GStreamer streaming threads, created by GStreamer elements

The important rule:

> The pipeline does not run inside the Python main thread.

The Python main thread usually starts the pipeline and runs a `GLib.MainLoop`.
The actual buffer processing is done by GStreamer streaming threads.

---

## Simple pipeline

```python
pipeline.set_state(Gst.State.PLAYING)
loop = GLib.MainLoop()
loop.run()
```

The main thread is blocked inside `loop.run()`.

That does not mean the pipeline is blocked.

GStreamer has its own streaming task threads that move buffers from element to
element.

```text
Python main thread
    |
    | runs GLib.MainLoop
    v
waits for events, bus messages, timers

GStreamer streaming thread
    |
    v
source -> filter -> sink
```

---

## What happens when the pipeline has a queue

Some elements create a new streaming thread.

The common example is `queue`.

```text
videotestsrc ! queue ! videoconvert ! appsink
```

The `queue` separates the pipeline into two streaming parts.

```text
thread A                         thread B
--------                         --------
videotestsrc  --->  queue  --->  videoconvert  --->  appsink
```

This means the source side can continue pushing buffers until the queue is full.

If the downstream side is slow, the queue fills up. After the queue is full, the
upstream thread is blocked too.

So `queue` gives buffering and a thread boundary, but it does not make slow code
free.

---

## Which thread calls appsink callback

When using `appsink emit-signals=true`, the `new-sample` callback is called from
the GStreamer streaming thread that pushes the sample into `appsink`.

It is not called from the Python main thread.

```python
appsink.connect("new-sample", on_new_sample)
```

```python
def on_new_sample(sink):
    sample = sink.emit("pull-sample")
    return Gst.FlowReturn.OK
```

If this callback is slow, it blocks that streaming thread.

For example, this is bad in a real-time pipeline:

```python
def on_new_sample(sink):
    sample = sink.emit("pull-sample")
    heavy_python_work(sample)
    return Gst.FlowReturn.OK
```

Better pattern:

- pull the sample
- copy or reference only the data you need
- push work to your own worker thread or queue
- return quickly

---

## Demo application

Download the code:

- [appsink_thread_demo.py](code/appsink_thread_demo.py)

Go to the code folder first:

```bash
cd docs/Other/Gstreamer/python_bindings/threading/code
```

Run the demo:

```bash
python3 appsink_thread_demo.py
```

<details>
<summary>Code</summary>

```python
--8<-- "docs/Other/Gstreamer/python_bindings/threading/code/appsink_thread_demo.py"
```

</details>

Arguments:

| Argument | Meaning |
| -------- | ------- |
| no argument | run a simple `videotestsrc ! videoconvert ! appsink` pipeline |
| `--queue` | insert a GStreamer `queue` before `videoconvert`, creating a thread boundary in the pipeline |
| `--slow-callback` | sleep inside the appsink callback to show how a slow Python callback blocks the streaming path |
| `--queue --slow-callback` | combine both cases: queue adds buffering, but the slow callback can still block after the queue fills |

Examples:

```bash
python3 appsink_thread_demo.py --queue
```

```bash
python3 appsink_thread_demo.py --slow-callback
```

```bash
python3 appsink_thread_demo.py --queue --slow-callback
```

The demo prints the Python thread name and thread id from:

- the main code
- the bus watch callback
- the appsink `new-sample` callback

The printed output shows where each callback runs.

Example shape:

```text
main started on: MainThread ident=123
bus watch: MainThread ident=123 message: state-changed
appsink callback: Dummy-1 ident=456 frame: 1 pts: 123456789
```

Meaning:

| Output field | Meaning |
| ------------ | ------- |
| `MainThread` | the normal Python main thread |
| `Dummy-1` or another name | a non-Python-created thread that Python sees after GStreamer calls Python code |
| `ident=...` | Python thread id; use it to compare whether two prints came from the same thread |
| `frame` | frame counter from the appsink callback |
| `pts` | GStreamer buffer presentation timestamp |
| `message` | bus message type, such as `state-changed`, `error`, or `eos` |

You should see that the appsink callback is not running in the same place as the
main setup code.

The exact thread names can vary, but the callback is called from the streaming
path, not from normal Python control flow.

---

## Bus messages and the main loop thread

The pipeline bus is used for control messages:

- `ERROR`
- `EOS`
- `STATE_CHANGED`
- warnings
- application messages

There are two common ways to read the bus.

### Poll the bus yourself

```python
msg = bus.timed_pop_filtered(
    100 * Gst.MSECOND,
    Gst.MessageType.ERROR | Gst.MessageType.EOS,
)
```

In this mode, the thread that calls `timed_pop_filtered` handles the message.

If the Python main thread calls it, then the Python main thread handles the
message.

### Use a bus watch

```python
bus.add_signal_watch()
bus.connect("message", on_bus_message)
loop = GLib.MainLoop()
loop.run()
```

In this mode, the bus callback is dispatched by the `GLib.MainLoop` thread.

Usually that is the Python main thread, because the main thread usually calls
`loop.run()`.

```text
GStreamer streaming thread
    |
    | posts message to bus
    v
GstBus
    |
    | GLib dispatch
    v
Python main thread running GLib.MainLoop
```

So bus watch callbacks are a good place for control logic:

- stop the loop on `EOS`
- print errors
- update application state

Do not do heavy frame processing in the bus callback.

---

## Sync bus handler

There is another bus style: sync handler.

```python
bus.set_sync_handler(on_sync_message)
```

A sync bus handler is called in the thread that posts the message.

That can be a GStreamer streaming thread.

Use it only when you really need synchronous handling. For most Python
applications, `bus.add_signal_watch()` is simpler and safer.

---

## Does the Python GIL affect the GStreamer pipeline?

Short answer:

> The GIL does not control GStreamer C code, but it does matter when GStreamer
> calls Python code.

GStreamer elements are implemented mostly in C. Their internal processing can run
in GStreamer threads without being controlled by the Python GIL.

So this is mostly correct:

> The Python GIL does not directly stop the GStreamer pipeline processing inside
> C elements.

But when a GStreamer thread calls a Python callback, like `appsink new-sample`,
that callback must acquire the Python GIL.

That means:

- C elements can continue to run in their own threads
- Python callbacks still obey the GIL
- a slow Python appsink callback blocks the GStreamer streaming thread that
  called it
- if several GStreamer threads call Python callbacks at the same time, only one
  Python callback executes Python bytecode at a time

Practical rule:

> Keep Python callbacks short. Let GStreamer stream. Move heavy Python work out
> of the streaming callback.

---

## Summary

| Part | Thread |
| ---- | ------ |
| Python setup code | Python main thread |
| `GLib.MainLoop.run()` | the thread that calls it, usually main |
| GStreamer buffer processing | GStreamer streaming threads |
| `queue` element | creates a thread boundary |
| `appsink new-sample` callback | streaming thread that reaches appsink |
| bus watch callback | GLib main loop thread |
| sync bus handler | thread that posts the bus message |
| Python code inside callbacks | affected by the GIL |
| C code inside GStreamer elements | not directly controlled by the GIL |
