def accumulator():
    total = 0
    while True:
        value = yield total
        total += value

acc = accumulator()
print(next(acc))  # Start the generator, prints 0
print(acc.send(5))  # Send 5, prints 5
print(acc.send(10))  # Send 10, prints 15