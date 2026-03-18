def outer():
    x = 10
    
    def inner():
        return x
    
    return inner

func = outer()
print(func.__closure__)
print(func.__closure__[0].cell_contents)
print(func())  # 10