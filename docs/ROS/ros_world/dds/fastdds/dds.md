---
title: DDS communication in ROS system 
tags:
    - fastdds
    - dds
    - rtps
---
Describe two nodes communication

## Discovery
Using RTOS discovery protocol (SPDP + SEDP)
Each node (**DDS Participant**) send multicast message (usually 239.255.0.1:7400 domain 0)
That announce

- I publish topic `/topic names`
- I subscribe to topics `/topic name`

## Matching
Once discovered

- Publisher = **DataWrite**
- Subscriber = **DataReader**

DDS Checks

- topic name
- message type
- QoS

## Transport selection
How to send data

- SHM 
- UDP (RTPS)
- TCP (rarely use)

## Data flow
- Message serialize using CDR format
- Wrapped into RTPS data message

## Continuous communication
After connection
- data flow (unicast)
- discovery still run in background
- Nodes send periodic messages
  - HEARTBEAT
  - LIVELINESS

| Feature       | ❤️ HEARTBEAT       | 👋 LIVELINESS          |
| ------------- | ------------------ | ---------------------- |
| Purpose       | Data delivery      | Node health            |
| Scope         | Per message stream | Per publisher          |
| Uses QoS      | RELIABLE only      | Always (if configured) |
| Response      | ACKNACK            | No ACK                 |
| Failure means | Data loss          | Publisher dead         |
| Frequency     | High (data flow)   | Lower (lease duration) |


## Reliability
if QoS == RELIABLE
The DDS implement reliability over UDO