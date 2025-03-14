---
tags:
    - arduino
    - json
    - library
---

# ArduinoJson

[Read and more](https://arduinojson.org/)

## Demos
### Deserialization

<details>
    <summary>platformio ini</summary>
    Using ESP32 hardware 

```ini
[env:seeed_xiao]
platform = espressif32
platform_packages=
  framework-arduinoespressif32 @ https://github.com/espressif/arduino-esp32.git#3.0.3
  framework-arduinoespressif32-libs @ https://github.com/espressif/arduino-esp32/releases/download/3.0.3/esp32-arduino-libs-3.0.3.zip

board = seeed_xiao_esp32s3
framework = arduino
upload_port = /dev/ttyACM0
monitor_speed = 115200

lib_deps =
    bblanchon/ArduinoJson
```
</details>




- [Parser json demo](https://arduinojson.org/v7/example/parser/){:target="_blank"}