---
tags:
    - arduino
    - esp32
    - mqtt
    - tutorial
---

# ESP32 MQTT
Connect esp32 running arduino framework to ubuntu machine using MQTT protocol

Connect ESP32 to PlotJuggler to visualize data

## Prerequisites
Install mqtt broker on ubuntu machine

```bash
sudo apt install mosquitto mosquitto-clients
```

```bash
sudo systemctl status mosquitto
```
### Test mqtt broker
- Without authentication

```bash title="terminal 1, subscriber"
mosquitto_sub -t "hello/topic"
```

```bash title="terminal 2, publisher"
mosquitto_pub -t 'hello/topic' -m 'hello MQTT'
```


# UNDER CONSTRUCTION