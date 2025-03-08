---
tags:
    - ros
    - aptly
    - deb
    - reosdep
    - bloom
    - docker
---

# Using Docker and Aptly to check package installation

## Aptly

!!! warning aptly update
    Download new version 1.60
    The ubuntu repo version is 1.4.0 and support Zstd compression the use by `bloom-generate`

    
     
Using aptly to create a custom repository for ROS packages. This is useful when you want to create a custom repository for your ROS packages.

- Create a new repository
- Add packages to the repository
- Publish the repository
- Add the repository to the sources list


### Create a new repository
```bash
aptly -distribution="jammy" -architectures="amd64" \
repo \
create \
my_ros_app_repo
```

#### Add packages to the repository
```bash
aptly repo add my_ros_app_repo \
ros-humble-pkg-client_0.0.0-0jammy_amd64.deb \
ros-humble-pkg-server_0.0.0-0jammy_amd64.deb
```

#### Publish the repository
```bash
aptly -architectures="amd64" -skip-signing=true \
publish repo -architectures="amd64" \
my_ros_app_repo local
```

##### Serve the repository
```bash
aptly serve
# aptly serve -listen=":8081"
```

---

## Docker

Install ROS Package on a docker container
The docker image base on ubuntu 22.04 with ROS humble base


```bash
docker run -it --rm \
--net host \
--hostname ros \
--user user \
humble:dev /bin/bash
```

### Add the repository to the sources list

```bash
deb [trusted=true] http://127.0.0.1:8081/local/ jammy main

```

### Check for packages
```bash 
apt update
apt search ros-humble-pkg*
Sorting... Done
Full Text Search... Done
ros-humble-pkg-client/jammy 0.0.0-0jammy amd64
  TODO: Package description

ros-humble-pkg-server/jammy 0.0.0-0jammy amd64
  TODO: Package description

```

![alt text](image-2.png)

![alt text](image-3.png)