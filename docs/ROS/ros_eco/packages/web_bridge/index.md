---
title: ROS2 web bridge
tags:
    - web_bridge
    - rosbridge_suite
---

{{ page_folder_links() }}

WebBridge  is a bridge between ROS and the Web.
- exposes ROS topics, services, and parameters over WebSockets using JSON messages.
- A browser or any web app can connect via roslib.js (JavaScript client).
- There are other client implementation except js like python `roslibpy`

### install

```
sudo apt install ros-humble-ros-humble-rosbridge-suite
```


### usage

```bash title="launch"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

#### minimal 
- Connect for javascript
- Open `Console` from web developer tools for view log output
```html
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
<title>ROS Bridge Demo</title>

<script>
  // Connecting to ROS
  // -----------------
  var ros = new ROSLIB.Ros();
  ros.connect('ws://localhost:9090');
    ros.on('connection', function() {
        console.log('Connected to websocket server.');
    });
    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
    });
    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
    }); 
    // -----------------    
</script>
```

---

#### Parameter
- declare parameter

```js
var param1 = new ROSLIB.Param({
        ros: ros,
        name: '/minimal:param1'
    });
```

!!! tip "parameter name"
    `node_name:param_name`
     

- get / set

```js
//get
param1.get(function(value) {
            displayParam1(value);
        });
//set
param1.set(newValue);
```

<details>
    <summary>ros2 node</summary>

```python
--8<-- "docs/ROS/ros_eco/packages/web_bridge/code/ros2_parameter_demo.py"
```
</details>


<details>
    <summary>HTML/Js</summary>

```html title="full demo"
--8<-- "docs/ROS/ros_eco/packages/web_bridge/code/parameter.html"
```
</details>

---

## Subscriber
Implement subscriber to ros topic
The demo topic publish int counter
The js subscribe using bridge


!!! tip "don't forget"
     ```bash
     ros2 launch rosbridge_server rosbridge_websocket_launch.xml
     ```

<details>
    <summary>simple publisher</summary>

```python
--8<-- "docs/ROS/ros_eco/packages/web_bridge/code/simple_pub.py"
```
</details>


<details>
    <summary>simple web bridge subscriber</summary>

```js
--8<-- "/home/user/projects/blog/docs/ROS/ros_eco/packages/web_bridge/code/sub.html"
```
</details>


---

## Service

!!! tip "don't forget"
     ```bash
     ros2 launch rosbridge_server rosbridge_websocket_launch.xml
     ```

<details>
    <summary>simple service</summary>

```python
--8<-- "docs/ROS/ros_eco/packages/web_bridge/code/simple_service.py"
```
</details>


<details>
    <summary>simple web bridge subscriber</summary>

```js
--8<-- "/home/user/projects/blog/docs/ROS/ros_eco/packages/web_bridge/code/service.html"
```
</details>