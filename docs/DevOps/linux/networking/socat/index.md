---
title: SOCAT
tags:
    - tools
    - socat
    - network
---
**socat** = SOcket CAT
A general-purpose tool for connecting two data streams (network, serial, files, pipes, PTYs, etc.).

### Demo

```bash title="keyboard->udp"
socat STDIO,raw,echo=0 UDP:127.0.0.1:7000
```

```bash title="udp->screen"
socat UDP4-LISTEN:7000,reuseaddr STDOUT
```

---

### udp 2 serial
- **UDP-LISTEN** = Connected UDP socket
- **UDP-RECVFROM** = Unconnected UDP socket - raw UDP socket with source tracking

!!! tip ""
    **Connected UDP socket (UDP-LISTEN)**
    - You call connect() on a UDP socket
    - Kernel remembers one peer
    You can:
        - send() (no address needed)
        - recv() (only from that peer)
    - Replies automatically go to that peer

    **Unconnected UDP socket (UDP-RECVFROM)**
    - You call bind() only
    - You must:
        - recvfrom() (get source address)
        - sendto() (specify destination)
    - Can talk to many peers


#### Udp listener
Any UDP packet sent to host IP will be forwarded to the serial device.
The **sender’s** address is remembered so responses can be sent back.

```bash
socat -d -d \
  UDP4-LISTEN:14550,reuseaddr \
  FILE:/dev/ttyUSB0,b115200,raw,echo=0
```

- **reuseaddr**
Allows rebinding to the same port
Useful if:
  - the program crashed
  - you restart quickly


- **raw**
Disables all terminal processing:

  - No newline translation
  - No character filtering
  - No flow control
  - Binary-safe

- **echo=0**
Disable echo, Prevents characters sent to the serial port from being echoed back

---

#### multiple clients
- One serial port multiple udp clients


!!! warning "fork"
    fork open multiple process that write and read against the serial process
    
!!! tip "echo server"
    I use usb2uart and connect the TX with RX
    
```bash
socat -d -d   UDP4-RECVFROM:14550,fork   FILE:/dev/ttyUSB0,b115200,raw,echo=0
```

```bash title="test"
# using nc as client
# can open multiple nc client

nc -u 127.0.0.1 14550
```

- **UDP4-RECVFROM:14550**
    - Create a UDP IPv4 socket
    - Bind to local port 14550
    - Receive datagrams from any remote host/port
    - Reply packets go back to the source of the last received datagram
- **fork** 
For each new incoming peer, socat:
fork()s a child process
The child handles data forwarding
The parent keeps listening
For **UDP**, it works per sender address (IP:port), not per connection.

---

### TCP 2 serial
Not working for multiple connection

```bash
socat -d -d \
  TCP4-LISTEN:14550,reuseaddr,fork \
  FILE:/dev/ttyUSB0,b115200,raw,echo=0


```