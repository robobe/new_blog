import zmq
import math
import json
from time import sleep
import numpy as np


context = zmq.Context()
serverSocket = context.socket(zmq.PUB)
port = 9872
serverSocket.bind("tcp://*:"+str(port))
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

   serverSocket.send_string(json.dumps(data))