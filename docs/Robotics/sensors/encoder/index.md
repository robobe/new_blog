---
tags:
  - sensor
  - encoder
  - AS5048
---



## Wiring

![alt text](images/AS5048.png)
| color  | desc                       | uno pin        |
| ------ | -------------------------- | -------------- |
| black  | CSN  (chip select)         | D10 (any gpio) |
| purple | CLK                        | D13 (SCK)      |
| yellow | MOSI (master out slave in) | D11 (COPI)     |
| green  | MISO (master in slave out) | D12 (CIPO)     |
| red    | 5/3.3                      |                |
| white  | GND                        |                |

white = purple
red = green
green = yellow
yellow = black
purple = red
black = white


![alt text](images/spi_uno.png)

## Code

```cpp
#include <SPI.h>

// Define the AS5048A SPI settings
SPISettings AS5048ASettings(1000000, MSBFIRST, SPI_MODE1);

// Define the chip select pin
const int CSPin = 10;

void setup() {
  // Set the chip select pin as an output
  pinMode(CSPin, OUTPUT);
  // Begin SPI communication
  SPI.begin();
  // Pull the chip select pin high to deselect the sensor
  digitalWrite(CSPin, HIGH);
}

void loop() {
  // Read the angle from the sensor
  unsigned int angle = readAS5048A();
  // Print the angle to the Serial Monitor
  Serial.println(angle);
  delay(1000); // Wait for 1 second
}

unsigned int readAS5048A() {
  // Variable to store the angle
  unsigned int angle = 0;
  // Pull the chip select pin low to select the sensor
  digitalWrite(CSPin, LOW);
  // Start SPI transaction with the defined settings
  SPI.beginTransaction(AS5048ASettings);
  // Send the command to read the angle
  SPI.transfer(0xFF);
  // Read the high byte of the angle
  angle = SPI.transfer(0x00);
  // Shift the high byte and read the low byte
  angle = (angle << 8) | SPI.transfer(0x00);
  // End the SPI transaction
  SPI.endTransaction();
  // Pull the chip select pin high to deselect the sensor
  digitalWrite(CSPin, HIGH);
  // Return the angle
  return angle;
}
```

## Reference

- [How to Use AS5048 Magnetic Encoder Position Sensor: Examples, Pinouts, and Specs ](https://docs.cirkitdesigner.com/component/1567e458-ca18-4c2a-bde0-71c2a0b48ecf)
