---
title: Gazebo PosePublisher
tags:
    - gazebo
    - plugin
---


{{ page_folder_links() }}


```xml
<model name="my_box">
  <plugin
    filename="gz-sim-pose-publisher-system"
    name="gz::sim::systems::PosePublisher">
    <publish_model_pose>true</publish_model_pose>
    <publish_link_pose>true</publish_link_pose>
    <use_pose_vector_msg>false</use_pose_vector_msg>
    <update_frequency>10</update_frequency>
  </plugin>
</model>
```

```bash
gz topic --info -t /model/my_box/pose
#
Publishers [Address, Message Type]:
  tcp://172.19.0.1:37695, gz.msgs.Pose
```

## Usage
- Publish Ground truth