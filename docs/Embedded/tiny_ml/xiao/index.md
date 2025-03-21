---
tags:
    - tiny_ml
    - xiao
    - tf-lite
---

# TinyML

## Hardware

[Seeed Studio XIAO ESP32S3](https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32S3-Plus-p-6361.html?qid=eyJjX3NlYXJjaF9xdWVyeSI6IioiLCJjX3NlYXJjaF9yZXN1bHRfcG9zIjoxLCJjX3RvdGFsX3Jlc3VsdHMiOjUsImNfc2VhcmNoX3Jlc3VsdF90eXBlIjoiUHJvZHVjdCIsImNfc2VhcmNoX2ZpbHRlcnMiOiJzdG9yZUNvZGU6W3JldGFpbGVyXSAmJiBjYXRlZ29yeV9pZHM6WzIzNTNdICYmIHF1YW50aXR5X2FuZF9zdG9ja19zdGF0dXM6WzFdIn0%3D)

![alt text](images/xiao_layout.png)

![alt text](images/xiao_pinout.png)


<details>
    <summary>blink code</summary>

```cpp title="simple blink"
#include <Arduino.h>

const int ledPin = 21; // Change this to the pin your LED is connected to

void setup() {
    pinMode(ledPin, OUTPUT);
}

void loop() {
    digitalWrite(ledPin, HIGH); // Turn the LED on
    delay(1000);                // Wait for a second
    digitalWrite(ledPin, LOW);  // Turn the LED off
    delay(1000);                // Wait for a second
}
```
</details>





```init title="platformio.ini"
[env:seeed_xiao_esp32s3]
platform = espressif32
platform_packages=
  framework-arduinoespressif32 @ https://github.com/espressif/arduino-esp32.git#3.0.3
  framework-arduinoespressif32-libs @ https://github.com/espressif/arduino-esp32/releases/download/3.0.3/esp32-arduino-libs-3.0.3.zip

board = seeed_xiao_esp32s3
framework = arduino

```