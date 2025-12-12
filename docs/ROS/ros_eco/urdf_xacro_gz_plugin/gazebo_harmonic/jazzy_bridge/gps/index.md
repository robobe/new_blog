---
title: Gazebo ROS GPS/NavSat Bridge
tags:
    - ros
    - gazebo
    - bridge
    - jazzy
    - harmonic
    - gps
    - navsat
---

Bridge NavSat sensor

- Add `spherical_coordinates` to world
- Add model with `navsat` sensor

!!! Tip
    Version 1.9 include  
        [gps sdf](https://sdformat.org/spec/1.9/sensor#sensor_gps) for full tags and attributes
        [navsat sdf](https://sdformat.org/spec/1.9/sensor#sensor_navsat) for full tags and attributes

        
    [check type attribute](https://sdformat.org/spec/1.9/sensor#sensor_type)
    The "ray", "gpu_ray", and "gps" types are  
    equivalent to "lidar", "gpu_lidar", and  
    "navsat",   respectively. It is **preferred** to use 
    "lidar", "gpu_lidar", and "navsat" since "ray", 
    "gpu_ray", and "gps" will be **deprecated**. The 
    "ray", "gpu_ray", and "gps" types are maintained 
    for legacy support.



```xml title="spherical_coordinates"
<spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <world_frame_orientation>ENU</world_frame_orientation>
    <latitude_deg>47.478950</latitude_deg>
    <longitude_deg>19.057785</longitude_deg>
    <elevation>0</elevation>
    <heading_deg>0</heading_deg>
</spherical_coordinates>
```

```xml title="link model with navsat sensor"
<model name="robot_with_gps">
    <pose>0 0 0.1 0 0 0</pose>
    <static>false</static>

    <link name="base_link">

        <!-- Visual marker -->
        <visual name="visual">
            <geometry>
                <box>
                    <size>0.4 0.4 0.2</size>
                </box>
            </geometry>
            <material>
                <ambient>0.2 0.2 0.8 1</ambient>
            </material>
        </visual>

            <collision name="collision">
            <geometry>
                <box>
                    <size>0.4 0.4 0.2</size>
                </box>
            </geometry>
            </collision>
            

        <!-- Inertial so physics works -->
        <inertial>
            <mass>1.0</mass>
            <inertia>
                <ixx>0.01</ixx>
                <iyy>0.01</iyy>
                <izz>0.01</izz>
            </inertia>
        </inertial>

        <!-- GPS / NavSat sensor -->
        <sensor name="gps" type="navsat">
            <pose>0 0 0.2 0 0 0</pose>
            <always_on>1</always_on>
            <update_rate>5</update_rate>
            <gps>
                <position_sensing>

                </position_sensing>
            </gps>
        </sensor>

    </link>
</model>
```

### View sensor data

```bash
gz topic -e -t /world/empty/model/robot_with_gps/link/base_link/sensor/gps/navsat

#
header {
  stamp {
    sec: 58
    nsec: 200000000
  }
  data {
    key: "seq"
    value: "18"
  }
}
latitude_deg: 47.47894999999999
longitude_deg: 19.057785
altitude: 0.29999999981373549
velocity_east: -6.9698894381645786e-18
velocity_north: 1.3028327721371246e-18
velocity_up: -2.10442947790046e-12
frame_id: "robot_with_gps::base_link::gps"
```

---

## ROS

[Bridge communication between ROS and Gazebo](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)
### Bridge file

```yaml title="config/bridge.yaml"
- ros_topic_name: "/gps"
  gz_topic_name: "/world/empty/model/robot_with_gps/link/base_link/sensor/gps/navsat"
  ros_type_name: "gps_msgs/msg/GPSFix"
  gz_type_name: "gz.msgs.NavSat"
  direction: GZ_TO_ROS
```

#### launch bridge

```yaml
launch:

  - arg:
      name: "bridge_config_file"
      default: "$(find-pkg-share bumperbot_bringup)/config/bridge.yaml"

  #ros-gz bridge
  - node:
      pkg: "ros_gz_bridge"
      exec: "parameter_bridge"
      output: screen
      args:
          "--ros-args -p config_file:=$(var bridge_config_file)"
```
