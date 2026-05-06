---
title: heapq python simple priority queue
tags:
    - python
    - queue
    - data structure
---

heapq is special tree-base structure where the **smallest element is always at the top**

- Push (insert): O(log n)
- Pop (remove smallest): O(log n)
- Peek smallest: O(1)

```python
import heapq

heap = []

# push elements
heapq.heappush(heap, 5)
heapq.heappush(heap, 1)
heapq.heappush(heap, 3)

print(heap)  # not sorted, but heap structure

# get smallest
print(heap[0])  # 1

# pop smallest
smallest = heapq.heappop(heap)
print(smallest)  # 1
```

---

## Demo: Scheduler

```python
--8<-- "docs/Programming/python/python/libraries/heapq/code/scheduler.py"
```

## Demo: Scheduler that feed from another thread

```python
--8<-- "docs/Programming/python/python/libraries/heapq/code/scheduler_ex.py"
```