from evdev import InputDevice, ecodes

# dev = InputDevice("/dev/input/event18")
dev = InputDevice("/dev/input/by-id/usb-STMicroelectronics_BETAFPV_Joystick_5643437C4100-event-joystick")

for event in dev.read_loop():

    if event.type == ecodes.EV_ABS:
        print(
            "AXIS",
            ecodes.ABS[event.code],
            event.value
        )