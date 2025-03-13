---
tags:
    - arduino
    - mqtt
    - PubSubClient
    - Mosquito
---

# Connect Arduino with PC using MQTT
Publish data from arduino that run on ESP32 device to pc that run `mosquito` MQTT broker via wifi

## PC
- Install 
- Config

### Config
Add this line allow broker to listen to all port and allow connection without user and password

```title="/etc/mosquitto/mosquitto.conf"
listener 1883 0.0.0.0
allow_anonymous true
```

```bash title="restart service"
systemctl restart mosquitto
```

---

```ini
[env:esp32dev]
platform = espressif32
platform_packages=
  framework-arduinoespressif32 @ https://github.com/espressif/arduino-esp32.git#3.0.3
  framework-arduinoespressif32-libs @ https://github.com/espressif/arduino-esp32/releases/download/3.0.3/esp32-arduino-libs-3.0.3.zip
board = esp32dev
framework = arduino
serial_speed = 115200
monitor_speed = 115200
lib_deps = 
    PubSubClient
```

## Arduino code

```cpp
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi
const char *ssid = "<>"; // Enter your Wi-Fi name
const char *password = "<>";  // Enter Wi-Fi password

// MQTT Broker
const char *mqtt_broker = "<>";
const char *topic = "mqtt/esp32";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
      Serial.print((char) payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}

void setup() {
    // Set software serial baud to 115200;
    Serial.begin(115200);
    // Connecting to a WiFi network
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the Wi-Fi network");
    //connecting to a mqtt broker
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
    while (!client.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        if (client.connect(client_id.c_str())) {
            Serial.println("Public EMQX MQTT broker connected");
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
    // Publish and subscribe
    client.publish(topic, "Hi, I'm ESP32 ^^");
    client.subscribe(topic);
}



void loop() {
    client.loop();
}

```

## usage
```
mosquitto_sub -t "mqtt/esp32"
```

---

## Reference
- [MQTT on ESP32: A Beginner's Guide](https://www.emqx.com/en/blog/esp32-connects-to-the-free-public-mqtt-broker#getting-started-with-mqtt-on-esp32)
- [PubSibClient API](https://pubsubclient.knolleary.net/)