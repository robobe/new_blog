def my_generator():
    try:
        while True:
            yield "running"
    finally:
        print("cleanup on close")


gen = my_generator()
print(next(gen))  # "running"
print(next(gen))  # "running"
gen.close()  # This will trigger the cleanup code in the generator