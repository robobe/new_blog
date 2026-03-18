def numbers():
    for i in range(10):
        yield i

def evens(nums):
    for n in nums:
        if n % 2 == 0:
            yield n

def square(nums):
    for n in nums:
        yield n * n

pipeline = square(evens(numbers()))

for x in pipeline:
    print(x)