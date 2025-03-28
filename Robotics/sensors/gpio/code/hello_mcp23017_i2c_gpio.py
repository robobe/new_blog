from smbus2 import SMBus
import time

I2C_BUS = 1         # Raspberry Pi uses I2C bus 1
I2C_ADDR = 0x20     # MCP23017 I2C address

# MCP23017 Register Addresses
IODIRA = 0x00       # I/O direction register for GPIOA
GPIOA = 0x12        # Register to read/write GPIOA
OLATA = 0x14        # Output latch register for GPIOA

with SMBus(I2C_BUS) as bus:
    # Set all GPIOA pins as output (0x00 means all pins are outputs)
    bus.write_byte_data(I2C_ADDR, IODIRA, 0x00)

    while True:
        # Turn PA0 ON (set bit 0 to 1)
        bus.write_byte_data(I2C_ADDR, OLATA, 0x01)
        print("PA0 ON")
        time.sleep(1)

        # Turn PA0 OFF (set bit 0 to 0)
        bus.write_byte_data(I2C_ADDR, OLATA, 0x00)
        print("PA0 OFF")
        time.sleep(1)
