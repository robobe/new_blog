---
tags:
    - ros
    - vscode
    - docker
    - devcontainer
    - dev environment
---

# Using VSCode devcontainer and docker as dev environment


## Project Docker images 

![](images/docker_dev_layers.drawio.png)


### Common ROS image
Docker file base on `ubuntu` base image version depend on ROS version. 
It can be base on `nvidia` cuda image.

The image include `ros core` installation and common dev tools , it add none root user name `user` (id 1000)

TBD: include gazebo in the image (current version include it)

### Project runtime
Docker file base on `common_ros` image, it include all project dependencies `deb`,`python` and others.  
It use for check project debian (deb) package installation.  
The main propose it to be the docker image for Application/project production deployment.  
TBD: use this image for CI/CD

### Project Dev/Build
Docker file base on `runtime` image, it include all other tool for development, test and build.
Include ROS dev tools and other helper tools for day to day development.

### Ongoing
Dockerfile That use by `devcontainer` it lite Dockerfile for all things and ongoing installation.
The idea is to move the ongoing installation and settings to the project runtime and dev images.
Keep the `devcontainer` Dockerfile lite and fast to build.


!!! note "TBD"
    Separate the `ros common` image to `OS` and `ROS Core` images.
     