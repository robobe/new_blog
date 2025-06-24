import gpiod
import time

CHIP_NAME = "gpiochip4"  # On Raspberry Pi 5
LINE_OFFSET = 17         # BCM GPIO number

# Open the GPIO chip and get the line
chip = gpiod.Chip(CHIP_NAME)
line = chip.get_line(LINE_OFFSET)

# Request the line as output (v2.x API)
line.request(consumer="python-gpiod", type=gpiod.LINE_REQ_DIR_OUT)

# Blink loop
for _ in range(10):
    line.set_value(1)
    time.sleep(0.5)
    line.set_value(0)
    time.sleep(0.5)

line.release()
