def greet():
    print("Hello")

print(id(greet))  # This will print the memory address of the function object
print(type(greet))  # This will print <class 'function'>, indicating that greet