---
title: Mavros rc plugin
tags:
    - ros
    - mavros
    - rc
    - rc_channels_override
    - rc_channels_raw
---

{{ page_folder_links() }}

## Mavlink

| Message  |  Desc |
|---|---|
| RC_CHANNELS_RAW(35)  | The RAW values of the RC channels received (8 channels) |
| RC_CHANNELS(65)  | The PPM values of the RC channels received (18 channels)  |
| SERVO_OUTPUT_RAW(36)  | The RAW values of the servo outputs   |
| RC_CHANNELS_OVERRIDE(70)  | The RAW values of the RC channels sent to the MAV to override info received from the RC radio  | 

!!! note "RC_CHANNELS_RAW vs RC_CHANNELS"
    The different is that the `rc_channel` send the same information until 18 channels in one message,
    the `rc_channel_raw` send 8 channels in make if it from main or aux port

    `rc_channel` has `chancount` field the mark the number of channels supported but the command return maximum of 18 channels
     

## Mavros

| mavros  | mavlink  |  |
|---|---|---|
| /mavros/rc/in  | RC_CHANNELS_RAW / RC_CHANNELS | pub  |   
| /mavros/rc/out  | SERVO_OUTPUT_RAW  | pub  |
| /mavros/rc/override  |  RC_CHANNELS_OVERRIDE | sub  |

---

### RC_CHANNELS_RAW / RC_CHANNELS
If `rc_channels` receive then `rc_channel_raw` dropped

---

### RC_CHANNELS_OVERRIDE
Allows an external system like QGroundControl, MAVROS, or your own script to simulate RC transmitter inputs by injecting RC values directly into the autopilot.

- Send at ~10–50 Hz for reliable override 
- ArduPilot expects regular updates or will revert to RC input
- Use 65535 for any channel you don't want to override

!!! warning "Ardupilot "
    Ardupilot accept rc_channels_override only from gcs_id (usually 255)
    

    ```c
    void GCS_MAVLINK::handle_rc_channels_override(const mavlink_message_t &msg)
    {
        if(msg.sysid != gcs().sysid_gcs()) {
            return; // Only accept control from our gcs
        }

    ```

    We need to start mavros with system_id 255

    ```yaml
    /**/mavros:
        ros__parameters:
            system_id: 255
    ```

    ```bash
    [INFO] [1753461231.734660719] [mavros.mavros]: MAVROS UAS via /uas1 started. MY ID 255.191, TARGET ID 1.1
    ```
    
    ```bash
    ros2 param get mavros/mavros system_id
    Integer value is: 255
    ```