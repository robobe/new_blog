import zmq

context = zmq.Context()
timeout_ms = 5000  # 5-second timeout
max_retries = 3    # Number of retries

for attempt in range(1, max_retries + 1):
    print(f"Attempt {attempt}: Sending request...")

    # Create a new REQ socket for each attempt
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")
    socket.setsockopt(zmq.LINGER, 0)  # Prevent blocking on close

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)  # Monitor socket for a response

    try:
        socket.send(b"Hello")  # Send request

        socks = dict(poller.poll(timeout_ms))  # Wait for response with timeout

        if socket in socks:  # If response is received
            message = socket.recv()
            print(f"Received reply: {message.decode()}")
            socket.close()
            break  # Exit loop on success
        else:
            print("Timeout: No response received.")

    except zmq.ZMQError as e:
        print(f"ZeroMQ Error: {e}")

    finally:
        socket.close()  # Ensure the socket is properly closed

    if attempt == max_retries:
        print("Max retries reached. Exiting.")

context.term()  # Clean up context
print("Program exited.")
