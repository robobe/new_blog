---
tags:
    - zenoh
    - bridge
    - python
---

# Connect ROS via zenoh bridge with python

Using zenoh bridge to pub/sub and message call from python script to ROS2 and vice versa.

!!! info "versions"
    - ubuntu 22.04
    - ROS humble
    - cycloneDDS
    - zenoh 1.2.0
    - zenoh-plugin-ros2dds 1.2.0
    - pycdr2 1.0.0
     
## Install zenoh-plugin-ros2dds
Download and extract , bridge standalone executable 

[zenoh-plugin-ros2dds-1.2.0-x86_64-unknown-linux-gnu-standalone.zip ](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases)

## Install 
The IDL part of the CycloneDDS package as standalone version, to support packages that need CDR (de)serialisation without the Cyclone DDS API.

```
pip install pycdr2
pip install eclipse-zenoh==1.2.0
```


## Sub Demo
Send ROS2 messages to zenoh and receive it using python script

```bash title="Terminal 1"
export ROS_DISTRO=humble
./zenoh-bridge-ros2dds
```

```bash title="Terminal 2"
ros2 topic pub /my_int32_topic std_msgs/msg/Int32 "{data: 10}" --rate 1
```

```python
import zenoh
from dataclasses import dataclass
from pycdr2 import IdlStruct 
from pycdr2.types import int32
import time

@dataclass
class Int32(IdlStruct, typename="Int32"):
    data: int32


def callback(sample):
    msg = Int32.deserialize(bytes(sample.payload))
    print(f"Received: {msg.data}")


conf = zenoh.Config()    
session = zenoh.open(conf)
sub = session.declare_subscriber('my_int32_topic', callback)

while True:
    time.sleep(1)
```

---

## Pub Demo
Pub ROS2 messages to zenoh and subscribe using ROS

!!! warning "subscriber"
     For this demo to work there must be running subscriber ROS node for this topic.


```bash title="Terminal 1"
export ROS_DISTRO=humble
./zenoh-bridge-ros2dds
```

```python title="zenoh pub"
import zenoh
from dataclasses import dataclass
from pycdr2 import IdlStruct 
from pycdr2.types import int32
import time

@dataclass
class Int32(IdlStruct, typename="Int32"):
    data: int32

conf = zenoh.Config()    
session = zenoh.open(conf)
counter = 0
while True:
    time.sleep(1)
    session.put('my_int32_topic', Int32(data=counter).serialize())
    counter += 1
    print(f"Published: {counter}")
```

```python title="subscriber node"
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

TOPIC = "my_int32_topic"

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        self.create_subscription(Int32, TOPIC, self.callback, 10)
        self.get_logger().info("Hello ROS2")

    def callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Service Demo
Call ROS2 service from zenoh python 

### Demo
- Service node with message type `std_srv.srv.Trigger`
- zenoh ros bridge
- zenoh service client (get)


#### ROS Service Node
ROS node that run service with nessage `std_srv.srv.Trigger`

```python title="demo_service.py"
#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_services_default

TOPIC = "trigger_srv"

class MyNode(Node):
    def __init__(self):
        node_name="minimal_srv"
        super().__init__(node_name)
        self.srv = self.create_service(Trigger, TOPIC, self.handler, qos_profile=qos_profile_services_default)
        self.get_logger().info("Hello ROS2")

    def handler(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("Trigger service called")
        response.success = True
        response.message = "success"
        print(response)
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Zenoh client

```python title="zenoh get"
import zenoh

from dataclasses import dataclass
from pycdr2 import IdlStruct
from zenoh import Reply
import time

# ros2 service call /trigger_srv std_srvs/srv/Trigger

TOPIC = "trigger_srv"

@dataclass
class TriggerRequest(IdlStruct):
   pass


@dataclass
class TriggerResponse(IdlStruct):
   success: bool
   message: str
   


def handle_replay(reply: Reply):
    try:
        print(reply.ok)
        message = TriggerResponse.deserialize(reply.ok.payload.to_bytes())

        print(f">> Received {message}")
    except Exception as e:
        print(e)
        print(">> Received ERROR")

conf = zenoh.Config()    
session = zenoh.open(conf)

req = TriggerRequest().serialize()
replies = session.get(TOPIC, payload=req, handler=handle_replay)

time.sleep(2)
session.close()

```

#### usage

```bash title="Terminal 1"
export ROS_DISTRO=humble
./zenoh-bridge-ros2dds
```

```bash title="terminal 2"
# run ros service
```

```bash title="check service"
# using ros2 cli

ros2 service call /trigger_srv std_srvs/srv/Trigger
```

```bash title="zeno client"
# run zeno client
python3 ros_srv_call.py 


>> Received TriggerResponse(success=True, message='success')
```
