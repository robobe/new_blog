---
tags:
    - ros
    - qos
---

# Playing with ROS QoS


| QoS Parameter | Description | Options|
| ------------  | ----------  | ------ |
| Reliability|	Defines message delivery guarantees	| **RELIABLE** (guaranteed delivery, higher latency)  **BEST_EFFORT** (lower latency, no guarantee) |
| Durability|	Determines if old messages are available to late subscribers |	**VOLATILE** (no history, new subscribers get only new messages)   **TRANSIENT_LOCAL** (stores past messages for late joiners)
| History|	Controls how many messages are stored before being sent	|**KEEP_LAST(N)** (stores last N messages)  **KEEP_ALL** (stores all messages, limited by memory) |
| Depth |	Number of messages to store in KEEP_LAST(N)	| Any positive integer (default 10)|
| Deadline|	Maximum allowed time between consecutive messages	| Duration (e.g., 1s, 500ms); triggers an event if missed|
| Lifespan|	How long messages remain valid before being discarded	| Duration in sec (default infinity)|
| Liveliness|	Ensures publishers are active| **AUTOMATIC** (system-managed) **MANUAL_BY_TOPIC**  (publisher must assert liveliness)|
| Lease  Duration |	Maximum time a publisher can remain silent before being considered inactive	| Duration (e.g., 2s, 5s)|


## Profiles
rclpy define predefine profiles that define in the rmw

- qos_profile_system_default
- qos_profile_sensor_data

<details>
    <summary>rmw profile</summary>
!!! note "humble version"
    [qos_profiles.h](https://github.com/ros2/rmw/blob/humble/rmw/include/rmw/qos_profiles.h)
     
```cpp
static const rmw_qos_profile_t rmw_qos_profile_system_default =
{
  RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
  RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT,
  RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
  RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  5,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};
```
</details>



## Demos
### Publisher Queue size
Simulate publisher queue size control for late binding subscribers


#### qos_profile_system_default
In this profile the depth is zero and from result we saw that the subscribe not receives any message from history

<details>
    <summary>demo code</summary>
```python
--8<-- "docs/ROS/ros_basic/ros_qos/code/pub_qos_queue_system_profile.py"
```

</details>


```bash title="result" linenums="1" hl_lines="4"
[INFO] [1742102162.025442844] [pub_node]: Hello PUB
[INFO] [1742102162.026666484] [sub_node]: Hello SUB
[INFO] [1742102167.029141089] [sub_node]: --Create subscriber--
[INFO] [1742102168.029089360] [sub_node]: Received: hello 5
[INFO] [1742102169.028827506] [sub_node]: Received: hello 6
[INFO] [1742102170.029061624] [sub_node]: Received: hello 7
[INFO] [1742102171.029371443] [sub_node]: Received: hello 8
[INFO] [1742102172.028809769] [sub_node]: Received: hello 9
[INFO] [1742102173.028644708] [sub_node]: Received: hello 10
```

---

#### qos_profile_sensor_data
In this profile the depth set to **5** but the the durability set to **VOLATILE** (no history, new subscribers get only new messages), the result saw that late subscriber got only new messages

<details>
    <summary>demo code</summary>
```python
--8<-- "docs/ROS/ros_basic/ros_qos/code/pub_qos_queue_sensor_profile.py"
```

</details>

```bash title="result" linenums="1" hl_lines="4"
[INFO] [1742103996.715762333] [pub_node]: Hello PUB
[INFO] [1742103996.717355657] [sub_node]: Hello SUB
[INFO] [1742104001.719799187] [sub_node]: --Create subscriber--
[INFO] [1742104002.713190568] [sub_node]: Received: hello 5
[INFO] [1742104003.713665142] [sub_node]: Received: hello 6
[INFO] [1742104004.713264404] [sub_node]: Received: hello 7
[INFO] [1742104005.713393306] [sub_node]: Received: hello 8
[INFO] [1742104006.713138227] [sub_node]: Received: hello 9
[INFO] [1742104007.713481727] [sub_node]: Received: hello 10
```

---

#### Queue and Durability
Allow late subscriber to get history


- Durability: **TRANSIENT_LOCAL** stores past messages for late joiners
- depth: 10
  
**Subscriber QOS settings**
The **queue depth** in the subscriber refer to the number of messages the subscriber can store before processing them
The durability must be transient local because in **VOLATILE** means it **won't receive old message**

- depth: 10
- durability: **TRANSIENT_LOCAL** 
  
<details>
    <summary>demo code</summary>

```python
--8<-- "docs/ROS/ros_basic/ros_qos/code/pub_qos_queue_and_transient_local.py"
```
</details>

From the result we saw that we got five (0-4) message from publisher history in the same time slot almost

```bash title="result" linenums="1" hl_lines="4-8"
[INFO] [1742104307.703495396] [pub_node]: Hello PUB
[INFO] [1742104307.704634957] [sub_node]: Hello SUB
[INFO] [1742104312.706942284] [sub_node]: --Create subscriber--
[INFO] [1742104312.709927579] [sub_node]: Received: hello 0
[INFO] [1742104312.712135500] [sub_node]: Received: hello 1
[INFO] [1742104312.714533710] [sub_node]: Received: hello 2
[INFO] [1742104312.717377731] [sub_node]: Received: hello 3
[INFO] [1742104312.719738278] [sub_node]: Received: hello 4
[INFO] [1742104313.701471401] [sub_node]: Received: hello 5
[INFO] [1742104314.701632809] [sub_node]: Received: hello 6
[INFO] [1742104315.699930835] [sub_node]: Received: hello 7
[INFO] [1742104316.701096571] [sub_node]: Received: hello 8
[INFO] [1742104317.701580589] [sub_node]: Received: hello 9
[INFO] [1742104318.701305347] [sub_node]: Received: hello 10
```

---

#### Publisher livespan
controls how long a message remains valid in the publisher’s queue before it is discarded.

lifespan = {0,0} default message never expire store until queue is exceeded

## TODO: finish example with 0,3,5 durations

**Publisher QOS settings**

```bash title="duration infinity" linenums="1" hl_lines="4-8"
[INFO] [1742119324.479144324] [pub_node]: Hello PUB
[INFO] [1742119324.480279021] [sub_node]: Hello SUB
[INFO] [1742119329.482579604] [sub_node]: --Create subscriber--
[INFO] [1742119329.487386342] [sub_node]: Received: hello 0
[INFO] [1742119329.490330176] [sub_node]: Received: hello 1
[INFO] [1742119329.493336125] [sub_node]: Received: hello 2
[INFO] [1742119329.494006959] [sub_node]: Received: hello 3
[INFO] [1742119329.494655886] [sub_node]: Received: hello 4
[INFO] [1742119330.477924823] [sub_node]: Received: hello 5
[INFO] [1742119331.477839792] [sub_node]: Received: hello 6
[INFO] [1742119332.478648855] [sub_node]: Received: hello 7
[INFO] [1742119333.479034641] [sub_node]: Received: hello 8
[INFO] [1742119334.479176439] [sub_node]: Received: hello 9
[INFO] [1742119335.479199605] [sub_node]: Received: hello 10
```

---

## TODO: finish qos with deadline register to SubscriptionEventCallbacks

<details><summary>QoS and deadline event</summary>
```
--8<-- "docs/ROS/ros_world/qos/sub_qos_deadline.py"
```
</details>