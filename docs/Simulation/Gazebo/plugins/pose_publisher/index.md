---
title: Gazebo PosePublisher
tags:
    - gazebo
    - plugin
---

{{ page_folder_links() }}

Pose publisher system. Attach to an entity to publish the transform of its **child entities** in the form of gz::msgs::Pose messages, or a single gz::msgs::Pose_V message if "use_pose_vector_msg" is true. 


## Add plugin to my robot model


```xml title="add to my urdf"
<gazebo>
    <plugin
        filename="gz-sim-pose-publisher-system"
        name="gz::sim::systems::PosePublisher">
        <publish_model_pose>false</publish_model_pose>
        <publish_link_pose>true</publish_link_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg>
        <update_frequency>10</update_frequency>
    </plugin>
</gazebo>
```

## Gz topics

```bash
gz topic --info -t  /model/my_bot/pose
#
Publishers [Address, Message Type]:
  tcp://172.19.0.1:37695, gz.msgs.Pose
```

## Config

| Parameter                   | Description |
|----------------------------|-------------|
| publish_link_pose          | Set to `true` to publish link pose. |
| publish_visual_pose        | Set to `true` to publish visual pose. |
| publish_collision_pose     | Set to `true` to publish collision pose. |
| publish_sensor_pose        | Set to `true` to publish sensor pose. |
| publish_model_pose         | Set to `true` to publish model pose. |
| publish_nested_model_pose  | Set to `true` to publish nested model pose. The pose of the model that contains this system is also published unless `publish_model_pose` is set to `false`. |
| use_pose_vector_msg        | Set to `true` to publish a `gz::msgs::Pose_V` message instead of multiple `gz::msgs::Pose` messages. |
| update_frequency           | Frequency of pose publications in Hz. A negative frequency publishes as fast as possible (i.e., at the rate of the simulation step). |
| static_publisher           | Set to `true` to publish static poses on a "`<scoped_entity_name>/pose_static`" topic. This will cause only dynamic poses to be published on the "`<scoped_entity_name>/pose`" topic. |
| static_update_frequency    | Frequency of static pose publications in Hz. A negative frequency publishes as fast as possible (i.e., at the rate of the simulation step). |

