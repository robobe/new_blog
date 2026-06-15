---
title: H.26X SEI
tags:
    - metadata
    - gstreamer
    - sei
---

H.264 SEI (Supplemental Enhancement Information) is optional metadata carried
inside an H.264 video bitstream. SEI messages do not change the decoded pixels,
but they let encoders, decoders, or downstream applications attach extra
information to frames.

Common uses include timestamps, camera or sensor metadata, HDR/display
information, captions, stream identifiers, and custom application data. In a
GStreamer pipeline, SEI is useful when metadata must stay synchronized with the
video frames and travel together with the encoded stream.

## `h264_sei_pipe.py`

`h264_sei_pipe.py` demonstrates a small app bridge: one GStreamer pipeline
encodes raw video to Annex B H.264 access units, an `appsink` gives Python each
encoded frame, Python inserts a `user_data_unregistered` SEI NAL unit, and an
`appsrc` pushes the modified H.264 frame to an RTP/UDP pipeline.

The SEI is injected before the first VCL NAL unit, so metadata is carried in the
same access unit as the frame without changing the decoded image.

```python
access_unit = buffer_to_bytes(buffer)
payload = build_sei_payload(buffer)
output_data = insert_sei_before_first_vcl(access_unit, payload)

output_buffer = Gst.Buffer.new_wrapped(output_data)
clone_timing_and_flags(buffer, output_buffer)
appsrc.emit("push-buffer", output_buffer)
```
!!! info "VCL Video Coding Layer"
    ```
    [SPS/PPS/AUD metadata] [SEI metadata] [actual encoded frame data]
    ```

The payload is wrapped as user_data_unregistered SEI, identified by a UUID, then inserted into the H.264 access unit before the actual video slice.
The UUID acts like an identifier for your custom metadata format. Many tools or apps can insert SEI data into the same H.264 stream, so the receiver needs a way to know which SEI messages belong to your application.

```
SEI payload = [16-byte UUID][your JSON/message bytes]
```

## Receiver

`h264_sei_receiver.py` receives the RTP stream, depayloads it back to Annex B
H.264 access units, and attaches a pad probe before the decoder:

```text
udpsrc ! rtpjitterbuffer ! rtph264depay ! h264parse ! identity name=sei_tap ! avdec_h264
```

The probe sees each encoded access unit while the SEI NAL units are still
present. It maps the `Gst.Buffer`, scans Annex B NAL units for SEI NAL type 6,
parses `user_data_unregistered` payloads, keeps only messages whose first 16
bytes match the application UUID, and then decodes the remaining bytes as the
application payload:

```python
access_unit = buffer_to_bytes(buffer)
payloads = extract_user_data_unregistered(access_unit)

for payload in payloads:
    decoded = json.loads(payload.decode("utf-8"))
```

This keeps metadata extraction outside the decoder and does not alter the video
frames. The cost is still on the streaming path: every probed buffer is mapped
and scanned, and every matching SEI payload is decoded. Keep the probe work
small, avoid blocking I/O or heavy parsing in the callback, and move expensive
processing to another thread or queue if the receiver must sustain high
framerate or high-bitrate streams.

## H.265 / HEVC version

The same idea is implemented in `h265_sei.py`, `h265_sei_pipe.py`, and
`h265_sei_receiver.py`. The GStreamer shape is the same, but the codec elements
change to HEVC:

```text
x265enc ! h265parse ! rtph265pay
udpsrc ! rtph265depay ! h265parse ! identity name=sei_tap ! avdec_h265
```

The parser differences are in the NAL-unit header. H.265 NAL units have a
two-byte header, the prefix SEI NAL type is `39`, and VCL slice NAL types are
`0..31`. The SEI payload itself still uses `user_data_unregistered`:

```text
HEVC prefix SEI = [NAL type 39 header][payload type 5][size][16-byte UUID][payload bytes]
```

Run `h265_sei_receiver.py` first, then `h265_sei_pipe.py`. The H.265 example
uses UDP port `5002` so it can run beside the H.264 example on port `5000`.

!!! info "Annex B"
    Annex B is a common byte-stream format for H.264 and H.265 video.
    In Annex B, each encoded chunk, called a NAL unit, is separated by a start code:

    ```
    00 00 01
    ```

    ```title="Annex B stream looks roughly like:"
    [start code][SPS]
    [start code][PPS]
    [start code][SEI]
    [start code][video slice]
    [start code][video slice]
    ```

---

## Demos:
### h264
<details>
<summary>Helper</summary>

```python
--8<-- "docs/Other/Gstreamer/metadata/h26x_sei/code/h264_sei.py"
```

</details>

<details>
<summary>Sender</summary>

```python
--8<-- "docs/Other/Gstreamer/metadata/h26x_sei/code/h264_sei_pipe.py"
```

</details>

<details>
<summary>receiver</summary>

```python
--8<-- "docs/Other/Gstreamer/metadata/h26x_sei/code/h264_sei_receiver.py"
```

</details>

### h265

<details>
<summary>Helper</summary>

```python
--8<-- "docs/Other/Gstreamer/metadata/h26x_sei/code/h265_sei.py"
```

</details>

<details>
<summary>Sender</summary>

```python
--8<-- "docs/Other/Gstreamer/metadata/h26x_sei/code/h265_sei_pipe.py"
```

</details>

<details>
<summary>Receiver</summary>

```python
--8<-- "docs/Other/Gstreamer/metadata/h26x_sei/code/h265_sei_receiver.py"
```

</details>

### nvidia h265

<details>
<summary>Sender</summary>

```python
--8<-- "docs/Other/Gstreamer/metadata/h26x_sei/code/nvidia_h265_sei_pipe.py"
```

</details>

<details>
<summary>Receiver</summary>

```python
--8<-- "docs/Other/Gstreamer/metadata/h26x_sei/code/nvidia_h265_sei_receiver.py"
```

</details>

---

## Demo using plugin

<details>
<summary>Plugin</summary>

```python
--8<-- "docs/Other/Gstreamer/metadata/h26x_sei/code/plugins/gstbt_h264_sei.py"
```

</details>

<details>
<summary>Sender use the plugin</summary>

```python
--8<-- "docs/Other/Gstreamer/metadata/h26x_sei/code/h264_sei_sender.py"
```

</details>

<details>
<summary>Plugin</summary>

```python
--8<-- "docs/Other/Gstreamer/metadata/h26x_sei/code/h264_sei_receiver.py"
```

</details>

## `do_prepare_output_buffer`

`BtH264Sei` is a non-in-place `GstBase.BaseTransform`, so it does not edit the
incoming encoded buffer directly. `do_prepare_output_buffer` is where the plugin
builds the replacement output buffer for each H.264 access unit:

```python
access_unit = buffer_to_bytes(input_buffer)
payload = build_sei_payload(input_buffer)
output_data = insert_sei_before_first_vcl(access_unit, payload)
output_buffer = Gst.Buffer.new_wrapped(output_data)
clone_timing_and_flags(input_buffer, output_buffer)
```

The method maps the input `Gst.Buffer` to bytes, creates the JSON metadata
payload, inserts it as an SEI NAL unit before the first VCL NAL unit, then wraps
the modified bytes in a new `Gst.Buffer`. Because this creates a fresh buffer,
the original PTS, DTS, duration, offsets, and buffer flags are copied to keep the
encoded frame synchronized with the rest of the pipeline.

If the input buffer cannot be mapped, the method returns `Gst.FlowReturn.ERROR`.
Otherwise it returns `Gst.FlowReturn.OK` with the prepared output buffer. The
`do_transform` method can then be a no-op because the output buffer has already
been fully produced by `do_prepare_output_buffer`.
