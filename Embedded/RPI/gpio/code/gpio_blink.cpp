#include <gpiod.h>
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    const char* chipname = "gpiochip4";  // Raspberry Pi 5 J8 header GPIOs
    unsigned int line_num = 17;          // GPIO17 (BCM)

    gpiod_chip* chip = gpiod_chip_open_by_name(chipname);
    if (!chip) {
        std::cerr << "Failed to open GPIO chip\n";
        return 1;
    }

    gpiod_line* line = gpiod_chip_get_line(chip, line_num);
    if (!line) {
        std::cerr << "Failed to get line\n";
        gpiod_chip_close(chip);
        return 1;
    }

    if (gpiod_line_request_output(line, "gpio_toggle", 0) < 0) {
        std::cerr << "Failed to request line as output\n";
        gpiod_chip_close(chip);
        return 1;
    }

    for (int i = 0; i < 10; ++i) {
        gpiod_line_set_value(line, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        gpiod_line_set_value(line, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    gpiod_line_release(line);
    gpiod_chip_close(chip);
    return 0;
}
