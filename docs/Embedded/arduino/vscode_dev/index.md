---
tags:
    - arduino
    - dev
    - vscode
    - pio
    - platformio
---

# Using VSCode with platformio cli as Arduino dev environment
Using **Platformio CLI** to develop, build, and upload firmware for embedded systems


!!! note "Platformio VSCode extension"
    Graphical interface that integrates with VS Code for an easier development experience.
     
## Install

```bash
pip install platformio
```

## Basic command cheat 

| Command |	Description |
| ------ | ----------- |
|pio project init --board <board_name>	| Create a new project|
|pio lib install <library_name>	| Install libraries|
|pio run |	Build firmware|
|pio run --target upload |	Upload code to the board|
|pio device monitor	| Open Serial Monitor|
|pio run --target | clean	Clean compiled files|


## Init project

```
pio project init --board <board name>
```

```bash
# search for arduino mega 2560 atmelmegaavr bord
pio boards | grep -i mega | grep 2560

# select megaatmega2560 from the list

pio project init --board megaatmega2560
```

## Project struct

```
├── .vscode
|        └── c_cpp_properties.json
├── include
|        └── helper.hpp
├── lib                                # External libraries (if manually added)
├── platformio.ini                     # PlatformIO configuration file
├── README.md
├── src
|    ├── helper.cpp
|    └── main.cpp
└── test
```

!!! tip "create c_cpp_properties.json"
    ```
    pio init --ide vscode
    ```
     

---

!!! tip "code organize"
    - include libraries
    - defines and constants
    - global variables
    - user defined function or function signatures
    - setup
    - loop
     
## Demo

Use blink LED demo to learn how to split project to

- Multiple file
- Create Library
- Arduino OOP


### Multiple files
Split utils function to header and code  
[good reference](https://youtu.be/BdstuZP6l5E)

#### Project
```
├── include
│   └── led_function.hpp
├── platformio.ini
└── src
    ├── led_function.cpp
    └── main.cpp
```

```cpp title="lef_function.hpp"
#ifndef LED_FUNCTUIN_H
#define LED_FUNCTUIN_H

#include <Arduino.h>

void powerOnLed(byte pin);
void powerOffLed(byte pin);

#endif
```

```cpp title="lef_function.cpp"
#include "led_function.hpp"

void powerOnLed(byte pin){
    digitalWrite(pin, HIGH);   // Turn the LED on
  }
  
  void powerOffLed(byte pin){
    digitalWrite(pin, LOW);   // Turn the LED on
  }
```

```cpp title="main.cpp"
#include <Arduino.h>
#include "led_function.hpp"

# define LED_PIN 13

void setup() {
  Serial.begin(115200);
  // Print log
  Serial.println("setup");
  pinMode(LED_PIN, OUTPUT);
}

void loop() {

  powerOnLed(LED_PIN);
  delay(1000);                   // Wait for a second
  powerOffLed(LED_PIN);
  delay(1000);                   // Wait for a second
}
```

---

### Create custom Library
[Create an Arduino library](https://youtu.be/IiZl3p-ZohM){:target="_blank"}

# TODO: Arduino custom library

---

### Arduino OOP
[Arduino OOP [40-Minute Crash Course]](https://youtu.be/cUVryWbVkXk){:target="_blank"}

# TODO: arduino OOP 