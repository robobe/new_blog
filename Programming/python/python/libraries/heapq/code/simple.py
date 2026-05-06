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
