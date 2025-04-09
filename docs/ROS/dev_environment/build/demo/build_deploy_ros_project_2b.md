---
tags:
    - ros
    - deploy
    - tutorial
---

# ROS2 project from development to deployment, Part 2: Using github actions to Automatic build and deploy

- rosdep
- colcon
- bloom
- fakeroot


```yaml title="build package using github action"
name: My Action
on: [workflow_dispatch]
jobs:
  build_pc_pkg_interface:
    runs-on: pc
    env:
      PROJECT_NAME: pkg_interface
    steps:
      - name: rosdep
        run: |
          export ROSDISTRO_INDEX_URL=file://`pwd`/index-v4.yaml \
          && export ROS_DISTRO=humble \
          && echo "yaml file://`pwd`/rosdep/base.yaml" > /etc/ros/rosdep/sources.list.d/20-default.list \
          && echo "yaml file://`pwd`/rosdep/python.yaml" >> /etc/ros/rosdep/sources.list.d/20-default.list \
          && echo "yaml file://`pwd`/rosdep/custom_rosdep.yaml" > /etc/ros/rosdep/sources.list.d/30-custom.list \
          && rosdep update --rosdistro humble
      - name: colcon
        run: | 
          colcon build --packages-up-to $PROJECT_NAME 
      - name: bloom
        run: | 
          export ROSDISTRO_INDEX_URL=file://`pwd`/index-v4.yaml \
          && source /opt/ros/humble/setup.bash \
          && source install/setup.bash \
          && cd src/$PROJECT_NAME \
          && bloom-generate rosdebian 
      - name: fakeroot
        run: |
          source install/setup.bash \
          && cd src/$PROJECT_NAME \
          && fakeroot debian/rules binary
```