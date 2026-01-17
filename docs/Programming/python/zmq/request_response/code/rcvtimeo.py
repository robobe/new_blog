import zmq
import time

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

# Set both send and receive timeouts
socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5-second receive timeout
socket.setsockopt(zmq.SNDTIMEO, 5000)  # 5-second send timeout
socket.setsockopt(zmq.LINGER, 0)       # Prevent blocking on close

try:
    print("Sending request...")
    socket.send(b"Hello")  # Send request (this can block)
    
    message = socket.recv()  # Wait for response (times out after 5 sec)
    print(f"Received reply: {message.decode()}")

except zmq.error.Again:
    print("Timeout: No response received, exiting.")

finally:
    socket.close()  # Ensure socket is closed properly
    context.term()  # Terminate the context to free resources

print("Program exited.")
