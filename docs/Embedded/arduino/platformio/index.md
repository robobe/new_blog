---
tags:
    - platformio
    - arduino
---

# Platformio

## Arduino
### ESP32
The Arduino framework for ESP32 is actually a wrapper around the Espressif ESP-IDF. This means that while you write Arduino-style code (setup() and loop() functions), under the hood, ESP-IDF components handle tasks like Wi-Fi, Bluetooth, and RTOS scheduling.
[Arduino ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/)

#### Config 
```init
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
upload_speed = 921600
```

!!! note "Upgrade Arduino ESP32 version"
    - **framework-arduinoespressif32**: core Arduino-ESP32 framework package
    - **framework-arduinoespressif32-libs**: additional precompiled libraries that can speed up compilation.

    ```ini
    [env:esp32dev]
    platform = espressif32
    platform_packages=
    framework-arduinoespressif32 @ https://github.com/espressif/arduino-esp32.git#3.0.3
    framework-arduinoespressif32-libs @ https://github.com/espressif/arduino-esp32/releases/download/3.0.3/esp32-arduino-libs-3.0.3.zip
    board = esp32dev
    framework = arduino

    ```