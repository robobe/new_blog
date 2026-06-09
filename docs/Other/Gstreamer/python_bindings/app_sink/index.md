---
tags:
    - gstreamer
    - python
    - bindings
    - gst-bindings
    - appsink
---

In Python GStreamer bindings, `emit-signals` is a boolean property that tells elements such as `appsink` to emit high-level GObject signals instead of requiring the application to poll for data. For `appsink`, enabling this property lets you handle buffers through signals like `new-sample`.

# AppSink callback

`appsink` can emit a `new-sample` signal whenever a buffer is ready. In Python, connect that signal to a callback and pull the sample from the sink inside the callback.

The pipeline must set `emit-signals=true` on `appsink`; otherwise `appsink` will not emit `new-sample`, and the callback will not run.

<details>
<summary>Callback example</summary>

```python
--8<-- "docs/Other/Gstreamer/python_bindings/app_sink/code/callback_example.py"
```

</details>

---

# AppSink pull mode

In pull mode, the application asks `appsink` for samples directly. Keep
`emit-signals=false`; no `new-sample` callback is used.

`pull-sample` is the blocking version. It has no timeout: the call waits until a
sample is available or until the stream reaches EOS. If it returns `None`, the
stream is done or the sink is shutting down, so the loop should stop instead of
continuing to wait for another frame.

<details>
<summary>Blocking pull example</summary>

```python
--8<-- "docs/Other/Gstreamer/python_bindings/app_sink/code/blocking_pull_example.py"
```

</details>

`try-pull-sample` is the timeout-based style. It waits only for the timeout you
pass, returns `None` when no sample is ready yet, and lets the application do
other work or check the bus before trying again. In that case, `continue` is the
right behavior unless the bus reports EOS or an error.

<details>
<summary>Non-blocking pull example</summary>

```python
--8<-- "docs/Other/Gstreamer/python_bindings/app_sink/code/non_blocking_pull_example.py"
```

</details>
