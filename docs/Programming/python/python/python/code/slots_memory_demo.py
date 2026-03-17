import tracemalloc

class Normal:
    def __init__(self):
        self.x = 1
        self.y = 2

class Slotted:
    __slots__ = ("x", "y")

    def __init__(self):
        self.x = 1
        self.y = 2

def measure(cls, count=1):
    tracemalloc.start()
    objs = [cls() for _ in range(count)]
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return current

print("Normal:", measure(Normal))
print("Slotted:", measure(Slotted))