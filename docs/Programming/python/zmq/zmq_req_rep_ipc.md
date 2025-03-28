---
tags:
    - python
    - zmq
    - msgpack
    - ipc
---
# Client/Server Req/Rep zmq pattern with msgpack and IPC as transport


<details>
    <summary>ZMQ Ipc</summary>

```python
--8<-- "docs/Programming/python/zmq/code/ipc_req_rep_msgpack.py"
```
</details>

!!! note "shm"
    I try using `shm` protocol but i got an error that the protocol not supported
    I read the i need to compile with the draft option, 

    ```python
    import zmq
    print(zmq.has("draft")) 

    ```

     