---
tags:
    - ros
    - vscode
    - docker
    - devcontainer
    - dev environment
---

# Using VSCode devcontainer and docker as dev environment

<div class="grid-container">
    <div class="grid-item">
        <a href="dev_container">
        <img src="images/dev_container.png" width="150" height="150">
        <p>devcontainer</p>
        </a>
    </div>
    <div class="grid-item">
     <a href="remote_ssh">
        <img src="images/remote_ssh.png" width="150" height="150">
        <p>remote</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="project_templates">
        <p>projects template</p>
        </a>
    </div>
    
</div>

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
     