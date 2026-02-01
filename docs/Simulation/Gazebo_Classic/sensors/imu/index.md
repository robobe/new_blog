---
title: Gazebo classic IMU sensor and ROS2 plugin
tags:
    - gazebo
    - classic
    - sensors
    - imu
---

```xml title="urdf"
<?xml version="1.0"?>

<robot name="imu_box">
    <gazebo reference="imu_link">
        <material>Gazebo/Red</material>
    </gazebo>




    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.5 0.5" />
            </geometry>
        </visual>
        <collision>
            <geometry>
                <box size="0.5 0.5 0.5" />
            </geometry>
        </collision>
        <inertial>
            <mass value="5.0" />
            <inertia ixx="0.208333" ixy="0" ixz="0" iyy="0.208333" iyz="0" izz="0.208333" />
        </inertial>
    </link>

    <!-- IMU -->
    <link name="imu_link">
        <visual>
            <geometry>
                <box size="0.05 0.03 0.01" />
            </geometry>
        </visual>
        <collision>
            <geometry>
                <box size="0.05 0.03 0.01" />
            </geometry>
        </collision>
        <inertial>
            <mass value="0.05" />
            <inertia ixx="4.1667e-06" ixy="0" ixz="0" iyy="4.1667e-06" iyz="0" izz="7.5e-06" />
        </inertial>
    </link>

    <!-- IMU Joint -->
    <joint name="imu_joint" type="fixed">
        <parent link="base_link" />
        <child link="imu_link" />
        <origin xyz="0 0 0.355" rpy="0 -0.707 0" />
    </joint>
    <!-- 1.5707963 -->
    <!-- IMU Plugin -->
    <gazebo reference="imu_link">
        <sensor type="imu" name="imu">
            <always_on>true</always_on>
            <update_rate>50</update_rate>
            <imu>
                <angular_velocity>
                    <x>
                        <noise type="none">
                            <mean>0.0</mean>
                            <stddev>0.0002</stddev>
                        </noise>
                    </x>
                    <y>
                        <noise type="none">
                            <mean>0.0</mean>
                            <stddev>0.0002</stddev>
                        </noise>
                    </y>
                    <z>
                        <noise type="none">
                            <mean>0.0</mean>
                            <stddev>0.0002</stddev>
                        </noise>
                    </z>
                </angular_velocity>
                <linear_acceleration>
                    <x>
                        <noise type="none">
                            <mean>0.0</mean>
                            <stddev>0.002</stddev>
                        </noise>
                    </x>
                    <y>
                        <noise type="none">
                            <mean>0.0</mean>
                            <stddev>0.002</stddev>
                        </noise>
                    </y>
                    <z>
                        <noise type="none">
                            <mean>0.0</mean>
                            <stddev>0.002</stddev>
                        </noise>
                    </z>
                </linear_acceleration>
            </imu>
            <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
                <ros>
                    <namespace>/</namespace>
                    <remapping>~/out:=imu</remapping>
                </ros>
                <alwaysOn>true</alwaysOn>
                <updateRate>50.0</updateRate>
                <initial_orientation_as_reference>false</initial_orientation_as_reference>
                <bodyName>imu_link</bodyName>
                <frameName>imu_link</frameName>
            </plugin>
        </sensor>
    </gazebo>
</robot>
```

---

