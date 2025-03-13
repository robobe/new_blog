import websocket
import math
import json
from time import sleep
import numpy as np


ws = websocket.WebSocket()
ws.connect("ws://localhost:9871")


time = 0.0

while True:
   sleep(0.05)
   time += 0.05
   print(time)
   data = {
       "timestamp": time,
       "test_data": {
           "cos": math.cos(time),
           "sin": math.sin(time),
           "floor": np.floor(np.cos(time)),
           "ceil": np.ceil(np.cos(time))
       }
   }

   ws.send(json.dumps(data))