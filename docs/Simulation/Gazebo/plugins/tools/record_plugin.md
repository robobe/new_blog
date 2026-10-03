---
title: Gazebo Video Recorder plugin
tags:
    - gazebo
    - plugin
    - video
    - recording
---

# Gazebo Video Recorder plugin

`VideoRecorder` is a Gazebo GUI plugin that records the active 3D scene. Add
it inside the world `<gui>` element, then use its on-screen control to start
and stop recording.

The `<gz-gui>` block configures only the recorder control in the Gazebo window:
`state` makes it floating, `x` and `y` place it, and `width`, `height`, and
`showTitleBar` make it a compact 50 by 50 pixel button.

The `<record_video>` block controls the output:

- `use_sim_time`: timestamps the video with simulation time. A simulation that
  runs slower than real time still plays back at simulation speed.
- `lockstep`: waits for encoding before the next scene update. Enable it when
  every recorded frame matters; leave it `false` when simulation speed matters
  more than complete frame capture.
- `bitrate`: encoding target in bits per second. `4000000` is 4 Mbps; increase
  it for more detail and larger output files.

This example records using simulation time at 4 Mbps without slowing the
simulation for lockstep encoding. See the
[Gazebo Video Recorder documentation](https://gazebosim.org/api/gazebo/6/videorecorder.html)
for the available settings.

## SDF example

```xml
<plugin filename="VideoRecorder" name="VideoRecorder">
    <gz-gui>
        <property key="resizable" type="bool">false</property>
        <property key="x" type="double">300</property>
        <property key="y" type="double">50</property>
        <property key="width" type="double">50</property>
        <property key="height" type="double">50</property>
        <property key="state" type="string">floating</property>
        <property key="showTitleBar" type="bool">false</property>
    </gz-gui>
    <record_video>
        <use_sim_time>true</use_sim_time>
        <lockstep>false</lockstep>
        <bitrate>4000000</bitrate>
    </record_video>
</plugin>
```
