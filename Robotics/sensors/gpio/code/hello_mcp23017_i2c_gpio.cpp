#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>
#include <cstdint>  // For fixed-width integers

int main() {
    int file;
    const char *i2cBus = "/dev/i2c-1"; // I2C bus
    const int deviceAddress = 0x20;    // MCP23017 I2C address

    // Open the I2C bus
    file = open(i2cBus, O_RDWR);
    if (file < 0) {
        std::cerr << "Failed to open I2C bus\n";
        return -1;
    }

    // Connect to the MCP23017 device
    if (ioctl(file, I2C_SLAVE, deviceAddress) < 0) {
        std::cerr << "Failed to connect to I2C device\n";
        close(file);
        return -1;
    }

    // Set IODIRA register (0x00) to 0x00 (all output)
    uint8_t config[2] = {0x00, 0x00};
    if (write(file, config, 2) != 2) {
        std::cerr << "Failed to write to I2C device\n";
    }

    // Toggle PA0 port using GPIOA (0x14)
    for (int i=0; i<10; i++)
    {
        uint8_t state = i%2;
        uint8_t output[2] = {0x14, state};
        if (write(file, output, 2) != 2) {
            std::cerr << "Failed to write GPIOA\n";
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // // Read GPIOA (0x12)
    // uint8_t reg = 0x12;
    // if (write(file, &reg, 1) != 1) {
    //     std::cerr << "Failed to set register address\n";
    // }

    // uint8_t data;
    // if (read(file, &data, 1) != 1) {
    //     std::cerr << "Failed to read from I2C device\n";
    // } else {
    //     std::cout << "GPIOA Data: 0x" << std::hex << (int)data << std::dec << std::endl;
    // }

    close(file);
    return 0;
}
