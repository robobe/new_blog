---
title: Read IMU Data from Gazebo harmonic
tags:
    - gazebo
    - gz_transport
    - gz_msgs
    - harmonic
---
{{ page_folder_links() }}

Read imu message from gazebo simulation using cpp

## prerequisites

```bash
sudo apt install build-essential
sudo apt install \
    libgz-transport12-dev
    libgz-msgs10-dev
    libgz-tools-dev
    libgz-common5-dev
    libgz-math7-dev
```

<details>
    <summary>world with IMU</summary>

```xml
--8<-- "docs/Simulation/Gazebo/sensors/code/imu_world.sdf"
```
</details>

```cpp title="read_imu.cpp"
#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <iostream>

void imuCallback(const gz::msgs::IMU &msg)
{
  std::cout << "IMU Data:" << std::endl;
  std::cout << "  Linear Accel: "
            << msg.linear_acceleration().x() << ", "
            << msg.linear_acceleration().y() << ", "
            << msg.linear_acceleration().z() << std::endl;

  // std::cout << "  Angular Vel: "
  //           << msg.angular_velocity().x() << ", "
  //           << msg.angular_velocity().y() << ", "
  //           << msg.angular_velocity().z() << std::endl;
}

int main()
{
  gz::transport::Node node;

  std::string topic = "/imu";
  if (!node.Subscribe(topic, imuCallback))
  {
    std::cerr << "Failed to subscribe to topic: " << topic << std::endl;
    return -1;
  }

  std::cout << "Listening to IMU data on: " << topic << std::endl;
  gz::transport::waitForShutdown();
  return 0;
}

```


```cmake title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.22)
project(imu_subscriber)

# Find Gazebo dependencies
find_package(gz-transport12 REQUIRED)
find_package(gz-msgs10 REQUIRED)


# Add executable
add_executable(read_imu read_imu.cpp)
target_link_libraries(read_imu
    gz-transport12::gz-transport12
    ${GZ-MSGS_LIBRARIES}
)
```