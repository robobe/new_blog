---
title: Mavros IMU
tags:
    - imu
    - mavros
---

{{ page_folder_links() }}

## mavlink messages

- **attitude**: The attitude in the aeronautical frame (x-front, y-right, z-down).)
- **attitude_quaternion**: The attitude in the aeronautical frame (q: w, x, y, z)
- **highres_imu (105)**: The IMU readings in SI units in NED body frame
- **raw_imu (27)**: The RAW IMU readings for a 9DOF sensor, which is identified by the id (default IMU1). This message should always contain the true raw values without any scaling to allow data capture and system debugging.
- **scaled_imu (26)**: The RAW IMU readings for the usual 9DOF sensor setup. 
- **scaled_pressure**: 
   
!!! note "ATTITUDE_QUATERNION_COV(61)"
    This message not use by mavros and Not support by ardupilot
     

| mavlink  | mavros topic(message) |
|---|---|
| ATTITUDE (30) | **data** (sensor_msgs::msg::Imu)  |
| ATTITUDE_QUATERNION(31) |  **data** (sensor_msgs::msg::Imu)  |
| HIGHRES_IMU(105) | **data_raw** (sensor_msgs::msg::Imu)  |
| RAW_IMU(27) | **data_raw** (sensor_msgs::msg::Imu)  |
| SCALED_IMU(26) | **data_raw** (sensor_msgs::msg::Imu)  |
| SCALED_PRESSURE |   |

### Message Topic mapping
there two imu topic and the code map the priorities mavlink message

- **~/data**: The topic publish/mapping mavlink `ATTITUDE` or `ATTITUDE_QUATERNION` if publish by the FCU 
- **~/data_raw**: the topic publish/mapping selected mavlink message with priority `HIGHRES_IMU` or `SCALED_IMU` or `RAW_IMU` 

#### Message priority variables
- **has_hr_imu** = set to true automaticlay if system send `HIGHRES_IMU` message, disabled handle/publish `RAW_IMU` and `SCALED_IMU` message.
- **has_raw_imu** = set to true automaticlay if system send `RAW_IMU` message;
- **has_scaled_imu** = set to true automaticlay if system send `SCALED_IMU` message, disabled handle/publish `RAW_IMU` message.
- **has_att_quat** : set to true automaticlay if system send `ATTITUDE_QUATERNION` message, disabled handle/publish `ATTITUDE` message.


!!! warning "data_raw" vs "data"
    **data_raw**: Publish only gyro and accelerometer
    **data**: Publish Orientation and gyro
     

---

## ROS Imu msg
sensor_msgs/msg/Imu.msg

```
std_msgs/msg/Header header
geometry_msgs/msg/Quaternion orientation
double[9] orientation_covariance
geometry_msgs/msg/Vector3 angular_velocity
double[9] angular_velocity_covariance
geometry_msgs/msg/Vector3 linear_acceleration
double[9] linear_acceleration_covariance
```

### Covariance
The message covariance filed by parameters



| parameter name  | field name  |
|---|---|
| linear_acceleration_stdev  | IMU.linear_acceleration_covariance  |
| angular_velocity_stdev  | IMU.angular_velocity_covariance  |
| orientation_stdev / unk_orientation_cov | IMU.orientation_covariance  |
| magnetic_stdev  | MagneticField.magnetic_field_covariance  |


#### The -1 convention in IMU covariance

Each covariance array (orientation_covariance, angular_velocity_covariance, linear_acceleration_covariance) is a 3×3 matrix stored row-major.

Normally you’d put variances (σ²) on the diagonal.

**BUT: If a sensor does not provide that measurement, the convention is to set the first element to -1.**

---

## Demo: Attitude message (/mavros/imu/data)

```bash  title="set message stream"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 30, message_rate: 1.0}"
```


```bash
ros2 topic echo --once /mavros/imu/data
#
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: base_link
orientation:
  x: 0.00042285808111321393
  y: -4.743792265771591e-05
  z: -0.7445043500394287
  w: -0.6676174740920542
orientation_covariance:
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
angular_velocity:
  x: -0.0001230202615261078
  y: 9.356345981359479e-05
  z: -0.00024927849881350994
angular_velocity_covariance:
- 1.2184696791468346e-07
- 0.0
- 0.0
- 0.0
- 1.2184696791468346e-07
- 0.0
- 0.0
- 0.0
- 1.2184696791468346e-07
linear_acceleration:
  x: 0.0
  y: 0.0
  z: 0.0
linear_acceleration_covariance:
- -1.0
- 0.0
- 0.0
- 0.0
- 8.999999999999999e-08
- 0.0
- 0.0
- 0.0
- 8.999999999999999e-08
```

#### Covariance explain

- **orientation_covariance**: default orientation_stdev is: 1.0
- **angular_velocity_covariance**: default angular_velocity_stdev: 0.02*(pi/180)
- **linear_acceleration_covariance**: -1 , **no orientation sensor data**

!!! summery "calculate variance from angular_velocity_stdev parameter"
     
    ##### 
    $$0.02^\circ = 0.02 \times \frac{\pi}{180} = 0.00034906585 \ \text{rad/s}$$
    **Variance = (stdev)²**
    $$\sigma^2 = (0.00034906585)^2
    = 1.21846968 \times 10^{-7} \ (\text{rad/s})^2$$




---

### Demo:  scaled_imu message (/mavros/imu/data_raw)

```bash title="set message stream"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 26, message_rate: 1.0}"
```

```bash
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 26, message_rate: 1.0}"
```

```bash
ros2 topic echo --once /mavros/imu/data_raw
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: base_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
orientation_covariance:
- -1.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_covariance:
- 1.2184696791468346e-07
- 0.0
- 0.0
- 0.0
- 1.2184696791468346e-07
- 0.0
- 0.0
- 0.0
- 1.2184696791468346e-07
linear_acceleration:
  x: 0.0
  y: 1.202169221539125e-15
  z: 9.81645665
linear_acceleration_covariance:
- 8.999999999999999e-08
- 0.0
- 0.0
- 0.0
- 8.999999999999999e-08
- 0.0
- 0.0
- 0.0
- 8.999999999999999e-08
---
```

### Covariance explain

- orientation_covariance: -1 , **no orientation sensor data**
- angular_velocity_covariance: like `data` message
- linear_acceleration_covariance: default value 0.0003 is `9e-8`