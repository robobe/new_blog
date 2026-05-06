---
title: ZMQ Pub Sub
tags:
    - zmq
    - python
    - pub / sub
---
PUB/SUB is a one-to-many broadcast pattern.

| Property        | Meaning                 |
| --------------- | ----------------------- |
| One-to-many     | One PUB → many SUBs     |
| Fire-and-forget | No delivery guarantee   |
| Best effort     | Messages can be dropped |
| Asynchronous    | PUB never blocks on SUB |
| Scalable        | Add subscribers freely  |

### Topics 

ZMQ filtering happens on the subscriber side using a prefix match.

```python
sub.setsockopt(zmq.SUBSCRIBE, b"telemetry/")
```

!!! info "multiple topics"
    subscriber can register to multiple topic just multiple the code line above


---

<div class="grid-container">
    <div class="grid-item">
        <a href="#multiple-publisher-one-subscriber-with-multiple-topics">
        <p>Multiple topics</p>
        </a>
    </div>
     <div class="grid-item">
        <a href="">
        <p>-</p>
        </a>
    </div>
     <div class="grid-item">
        <a href="poller">
        <p>Poller</p>
        </a>
    </div>
</div>

---

## Demos:

### Multiple publisher one subscriber
Uses one SUB socket connected to multiple PUB endpoints.

<details>
<summary>Code</summary>
```python
--8<-- "docs/Programming/python/zmq/pub_sub/code/multiple_pub_on_sub.py"
```
</details>


### Multiple publisher one subscriber with multiple topics
Uses one SUB socket connected to multiple PUB endpoints and listen only to specific topics.

<details>
<summary>Code</summary>
```python
--8<-- "docs/Programming/python/zmq/pub_sub/code/multiple_pub_on_sub_topics.py"
```
</details>
