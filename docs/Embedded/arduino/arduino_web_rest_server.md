---
tags:
    - arduino
    - esp32
    - web
    - REST
---

# Implement RESET Server using Arduino on ESP32 board

Using esp32 board with arduino platform to create REST server

BOM

|   |   |
|---|---|
| esp32  | seeed_xiao_esp32s3  |




### init project

```ini
[env:seeed_xiao_esp32]
platform = espressif32
board = seeed_xiao_esp32s3
framework = arduino
lib_deps = bblanchon/ArduinoJson@^7.3.0
```

#### init vscode project 
Create vscode `c_cpp_properties.json` for declared board in the platformio.ini files

```
pio init --ide vscode
```

#### Install libraries

```
pio lib install bblanchon/ArduinoJson@^7.3.0
```

---

## REST App

- Simple code that init the wifi and create **GET REST** api `getValues` that return json


<details><summary>code</summary>
```cpp
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#pragma region consts
const char* ssid = "xxx";
const char* password = "xxxx";
#pragma endregion

#pragma region globals
WebServer server(80);
StaticJsonDocument<1024> jsonDocument;
char buffer[1024];
#pragma endregion

#pragma region REST API
void getValues(){
  Serial.println("get all values");
  jsonDocument.clear();
	jsonDocument["name"] = "hello";
  serializeJson(jsonDocument, buffer);
	server.send(200, "application/json", buffer);
}
void setupApi() {
  server.on("/getValues", getValues);
  // server.on("/setStatus", HTTP_POST, handlePost);
 
  // start server
  server.begin();
}

#pragma endregion REST API

#pragma region init
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(".!");
    delay(1000);
  }
  Serial.println(WiFi.localIP());

}
#pragma endregion

void setup() {
  Serial.begin(115200);
  initWiFi();
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());
  setupApi();
}

void loop() {
  server.handleClient();
}
```
</details>

### build and upload

```bash
# build
pio run

# upload
pio run --target upload
```


### usage

```
http://10.100.102.24/getValues
```

---

### Add POST

<details><summary>Post</summary>

```cpp
void handlePost() {
  if (server.hasArg("plain")) {  // Check if there is a body
      String message = server.arg("plain");  // Get POST body
      deserializeJson(jsonDocument, message);
      Serial.println("Received POST Data: " + message);
      Serial.println("Received json Data: " + String((const char*)jsonDocument["cmd"]));
      server.send(HTTP_OK, CONTENT_TYPE, "{\"status\":\"success\"}");
  } else {
      server.send(HTTP_ERROR, "text/plain", "Bad Request");
  }
}

void setupApi() {
    server.on("/getValues", getValues);
    server.on("/setStatus", HTTP_POST, handlePost);

    server.begin();
}
```

</details>


#### usage

```bash
curl -X POST http://10.100.102.24/setStatus \
     -H "Content-Type: application/json" \
     -d '{"cmd": "value"}'

# result
{"status":"success"}(venv)
```

---

## Reference
- [ESP32 Useful Wi-Fi Library Functions ](https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino/)
- [How to build a REST API server with ESP32user](https://www.techrm.com/how-to-build-a-rest-api-server-with-esp32/)