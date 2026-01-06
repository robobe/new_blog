import zmq
import threading
import time

ctx = zmq.Context() # type: ignore[attr-defined]

def publisher():
    pub = ctx.socket(zmq.PUB) # type: ignore[attr-defined]
    pub.bind("inproc://events")
    time.sleep(0.1)  # allow subscribers to connect
    pub.send_multipart([b"topic", b"hello"])

def subscriber():
    sub = ctx.socket(zmq.SUB) # type: ignore[attr-defined]
    sub.connect("inproc://events")
    sub.setsockopt(zmq.SUBSCRIBE, b"topic") # type: ignore[attr-defined]
    print(sub.recv_multipart())

t1 = threading.Thread(target=subscriber)
t2 = threading.Thread(target=publisher)

t1.start()
t2.start()

t1.join()
t2.join()
