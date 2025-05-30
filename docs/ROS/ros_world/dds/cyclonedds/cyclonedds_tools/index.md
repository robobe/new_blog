---
tags:
    - dds
    - cyclonedds
    - tools
---

# Cyclonedds tools


```bash title="install"
sudo apt install cyclonedds-tools
```

## ddsperf
Measures primarily data throughput and latency of the cyclone-based applications within the network or within the same board,

[Benchmarking Tools](https://cyclonedds.io/docs/cyclonedds/latest/config/benchmarking.html)

!!! tip Network throughput
    Network throughput is a measure of how much data is successfully transferred from one point to another in a given period of time across a network. It's typically expressed in bits per second (bps)


!!! tip Network latency
    Latency is the time delay wheen the packet is send and when it is received.

    using **ping** whe measure to round trip latency for a packet 
     
     

```bash title="terminal 1"
ddsperf pub size 1k 1Hz
```

```bash title="terminal 2"
ddsperf sub
#

[15894] 1.000  size 1024 total 1 lost 0 delta 1 lost 0 rate 0.00 kS/s 0.01 Mb/s (0.00 kS/s 0.00 Mb/s)
[15894] 2.000  size 1024 total 2 lost 0 delta 1 lost 0 rate 0.00 kS/s 0.01 Mb/s (0.00 kS/s 0.00 Mb/s)
[15894] 3.000  size 1024 total 3 lost 0 delta 1 lost 0 rate 0.00 kS/s 0.01 Mb/s (0.00 kS/s 0.00 Mb/s)
[15894] 4.000  size 1024 total 4 lost 0 delta 1 lost 0 rate 0.00 kS/s 0.01 Mb/s (0.00 kS/s 0.00 Mb/s)
[15894] 4.000  rss:6.4MB vcsw:11 ivcsw:0 ddsperf:1%+0%

```


- **pid**
- **time**
- **size** of the data involved in this test (For example, 1024 bytes, which is the “size 1k” defined in the pub command).
- **total** packets received .
- The total packets **lost** (For example, 0).
- **delta**: the packets received in a 1 second reporting 
- The packets **lost** in a 1 second report period (For example, 0).
- The number of samples processed by the Sub application in 1s (unit KS/s is 1000 samples per second).


```bash title="publisher"
ddsperf pub size 1k 10Hz
```

```bash
ddsperf sub
[15916] participant ubuntu:15916: new (self)
[15916] participant dev:5138: new
[15916] 1.001  size 1024 total 10 lost 0 delta 10 lost 0 rate 0.01 kS/s 0.08 Mb/s (0.00 kS/s 0.01 Mb/s)
[15916] 2.000  size 1024 total 20 lost 0 delta 10 lost 0 rate 0.01 kS/s 0.08 Mb/s (0.00 kS/s 0.02 Mb/s)
[15916] 3.000  size 1024 total 30 lost 0 delta 10 lost 0 rate 0.01 kS/s 0.08 Mb/s (0.00 kS/s 0.02 Mb/s)
[15916] 4.000  size 1024 total 40 lost 0 delta 10 lost 0 rate 0.01 kS/s 0.08 Mb/s (0.00 kS/s 0.03 Mb/s)
[15916] 4.000  rss:6.5MB vcsw:28 ivcsw:1 ddsperf:0%+1%
```

```bash
[15916] 4.000  rss:6.5MB vcsw:28 ivcsw:1 ddsperf:0%+1%
```

- rss: process memory usage
- vcsw: context switch (idle) 
- ivcsw: context switch (block)
- ddsperf: cpu user space, cpu kernel space
---

### using iperf3
iperf (or more commonly iperf3) is a powerful network testing tool used to measure:

- Network throughput (bandwidth)
- Data transfer rate (in both directions)
- Packet loss (UDP mode)
- Jitter (UDP mode)
- Retransmissions (TCP mode)
    
```bash title="install"
sudo apt install iperf3
```

```bash title="run server"
iperf3 -s
```

```bash title="run client"
iperf3 -c <server-ip>

#
# send udp packet at 10Mbps
iperf3 -c 192.168.1.100 -u -b 10M
```

```bash title="Measure Throughput"
# run server

# run client
iperf3 -c 192.168.1.100

# result
[ ID] Interval           Transfer     Bitrate         Retr  Cwnd
[  5]   0.00-1.00   sec   113 MBytes   952 Mbits/sec    0    329 KBytes 
```

- Transfer: Total amount of data send during the duration
- Bitrate: average speed over the duration