---
tags:
    - betaflight
---


{{ page_folder_links() }}

## betaflight sitl

```bash
sudo apt update && sudo apt upgrade
sudo apt -y install build-essential git curl clang-18 python3 python-is-python3
git clone https://github.com/betaflight/betaflight.git
cd betaflight
make arm_sdk_install
make configs
make TARGET=SITL
```

### betaflight web app
[betaflight documentation ](https://betaflight.com/docs/development/SITL)

```bash
git clone git@github.com:novnc/websockify-other.git
cd websockify-other/c
make

```

```bash title="run proxy"
./websockify 127.0.0.1:6761 127.0.0.1:5761
```

```bash title="run sitl"
./obj/betaflight_2025.12.0-beta_SITL
```

#### config web app to manual connection

!!! warning "work only with chrome base"
     
![](images/webapp_config.png)

Then establish a connection by using address ws://127.0.0.1:6761 in the Port field and clicking Connect.


![](images/web_app.png)

---

## vehicle_gateway
[github](https://github.com/osrf/vehicle_gateway/tree/main)

- [betaflight_gazebo](): gazebo plugin
- [betaflight_sim](): Gazebo worlds and iris model config with betaflight plugin
- [betaflight_demo](): using vehicle_gateway to control to quadcopter
- [betaflight_controller](): using port 9004 to control the quadcopter, translate joy message to motors channel control

![alt text](images/vehicle_gateway.png)

---

## Resources
- [SimITL - Sim In the Loop](https://github.com/AJ92/SimITL)
- [pr0p is a quadcopter racing simulator](https://pr0p.dev/)
- [FPV autonomous operation with Betaflight and Raspberry Pi](https://medium.com/illumination/fpv-autonomous-operation-with-betaflight-and-raspberry-pi-0caeb4b3ca69)
- [Autopilot for FPV Combat Drone on Betaflight (Empty version)](https://github.com/under0tech/autopilot_bee_ept)