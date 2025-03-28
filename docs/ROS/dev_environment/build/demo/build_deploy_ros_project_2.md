---
tags:
    - ros
    - deploy
    - tutorial
---

# ROS2 project from development to deployment, Part 2: Build Debian packages

Run the command From package folder

!!! tip "deb location"
    The deb file locate in parent folder: `src` folder
     

```bash
bloom-generate rosdebian --ros-distro humble
```

```bash title="build deb"
fakeroot debian/rules binary
```


---

## Build offline without internet

### files

rosdep files index location at `/etc/ros/rosdep/sources.list.d` the default file `20-default.list` can be download from github `rosdistro/rosdep/source.list.d/20-default.list`

| file  |  description |
|---|---|
| base.yaml  | system dependencies that are not ROS specific like boost, opencv   |
| python.yaml | map python package name |


bloom try to download `index-v4.yaml`  file from github rosdistro it can be override by setting the `ROSDISTRO_INDEX_URL` environment variable, the default `index-v4.yaml` map each ros distro to yaml file that also locate in rosdistro github. 


### Build Offline rosdep and bloom
- Download base.yaml, python.yaml, index-v4.yaml and the request distribution.yaml file in our case `humble` to local folder
- update `20-default.list` file entry to point to the download file (replace http:// with file://)
- Add project_rosdep.yaml file that map the workspace packages, it can be locate in the project folder or in other central location with other projects.
- update this location in `/etc/ros/rosdep/sources.list.d` as `30-custom_project.list`
- set `ROSDISTRO_INDEX_URL` to index-v4.yaml local location
- Run `rosdep update --rosdistro humble`
- Run `bloom-generate rosdebian --ros-distro humble` to build


