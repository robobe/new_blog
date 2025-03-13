import paho.mqtt.client as mqtt
import math
import json
from time import sleep
import numpy as np

# MQTT Broker details
BROKER = "localhost"  # Change to your MQTT broker address
PORT = 1883  # Default MQTT port
TOPIC = "sensor/data"  # MQTT topic to publish data

# Create MQTT client
client = mqtt.Client()
client.connect(BROKER, PORT, 60)

time_counter = 0.0

while True:
    sleep(0.05)  # 50 ms sleep for 10 Hz
    time_counter += 0.05  # Increment time

    # Create data payload
    data = {
        "timestamp": time_counter,
        "test_data": {
            "cos": math.cos(time_counter),
            "sin": math.sin(time_counter),
            "floor": np.floor(np.cos(time_counter)),
            "ceil": np.ceil(np.cos(time_counter))
        }
    }

    # Convert to JSON and publish
    client.publish(TOPIC, json.dumps(data))
    print(f"Published: {data}")
